#include "FreeRTOS.h"
#include "task.h"
#include <platform_stdlib.h>
#include "platform_opts.h"
#include "mjpeg2jpeg.h"

typedef struct _segment_info {
	u8 *start_ptr;  //pointer to start of segment
	u32 offset; // offset from the start of picture to 0xffxx marker
	u32 len; // segment length including length byte
	u8 is_set;
	struct _segment_info *next; //pointer to next seg_info
} seg_info;

#define JFIF_SEG_LEN      18
static const char jfif_segment[] = {
	0xff, 0xe0,                     // APP0
	0x00, 0x10,                     // APP0 header size (including
	// this field, but excluding preceding)
	0x4a, 0x46, 0x49, 0x46, 0x00,   // ID string 'JFIF\0'
	0x01, 0x01,                     // version
	0x00,                           // bits per type
	0x00, 0x00,                     // X density
	0x00, 0x00,                     // Y density
	0x00,                           // X thumbnail size
	0x00,                           // Y thumbnail size
};

static const int dht_segment_size = 420;
static const char dht_segment_head[] = { 0xFF, 0xC4, 0x01, 0xA2, 0x00 };
static const char dht_segment_frag[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
	0x0a, 0x0b, 0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* Set up the standard Huffman tables (cf. JPEG standard section K.3) */
/* IMPORTANT: these are only valid for 8-bit data precision! */
const char avpriv_mjpeg_bits_dc_luminance[17] =
{ /* 0-base */ 0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
const char avpriv_mjpeg_val_dc[12] =
{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

const char avpriv_mjpeg_bits_dc_chrominance[17] =
{ /* 0-base */ 0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };

const char avpriv_mjpeg_bits_ac_luminance[17] =
{ /* 0-base */ 0, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d };
const char avpriv_mjpeg_val_ac_luminance[] = {
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
	0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
	0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
	0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
	0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa
};

const char avpriv_mjpeg_bits_ac_chrominance[17] =
{ /* 0-base */ 0, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 };

const char avpriv_mjpeg_val_ac_chrominance[] = {
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
	0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
	0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
	0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
	0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
	0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
	0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
	0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
	0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
	0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
	0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
	0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
	0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
	0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
	0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
	0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa
};

static char *append(char *buf, char *src, int size)
{
	memcpy(buf, src, size);
	return buf + size;
}

static u8 *append_dht_segment(u8 *buf)
{
	buf = append(buf, (char *)dht_segment_head, sizeof(dht_segment_head));
	buf = append(buf, (char *)avpriv_mjpeg_bits_dc_luminance + 1, 16);
	buf = append(buf, (char *)dht_segment_frag, sizeof(dht_segment_frag));
	buf = append(buf, (char *)avpriv_mjpeg_val_dc, 12);
	*(buf++) = 0x10;
	buf = append(buf, (char *)avpriv_mjpeg_bits_ac_luminance + 1, 16);
	buf = append(buf, (char *)avpriv_mjpeg_val_ac_luminance, 162);
	*(buf++) = 0x11;
	buf = append(buf, (char *)avpriv_mjpeg_bits_ac_chrominance + 1, 16);
	buf = append(buf, (char *)avpriv_mjpeg_val_ac_chrominance, 162);
	return buf;
}

static seg_info *get_seg(seg_info *seg)
{
	seg_info *avail_seg = seg;
	seg_info *new_seg = NULL;
	if (!avail_seg->is_set) {
		return avail_seg;
	}
	while (avail_seg->next != seg) {
		if (!avail_seg->next->is_set) {
			return avail_seg->next;
		}
		avail_seg = avail_seg->next;
	}
	new_seg = malloc(sizeof(seg_info));
	if (new_seg == NULL) {
		return NULL;
	}
	memset(new_seg, 0, sizeof(seg_info));
	new_seg->next = avail_seg->next;
	avail_seg->next = new_seg;
	return avail_seg->next;
}
static void free_seg(seg_info *seg)
{
	seg_info *avail_seg = seg->next;
	seg_info *next_seg = NULL;
	while (avail_seg != seg) {
		next_seg = avail_seg->next;
		free(avail_seg);
		avail_seg = next_seg;
	}
	free(seg);
}

int mjpeg2jfif(AVFrame *in_frame)
{
	u8 *data = in_frame->FrameData;
	int len = in_frame->FrameLength;
	u8 *ptr = data;
	int cnt = 0;
	seg_info *trim_seg = NULL;
	seg_info *temp_seg;
	u32 seg_size_high, seg_size_low;
	int output_len = len;
	u32 offset = 0;
	u8 *tmp_buf, *tmp1, *tmp2;
	tmp1 = tmp2 = NULL;
	if (data == NULL) {
		printf("\n\rno data!");
		return -1;
	}
	//printf("\n\rdata addr:0x%x", ptr);
	if (*ptr == 0xff && *(ptr + 1) == 0xd8) {
		cnt += 2;
		ptr += 2;
	} else {
		printf("\n\rnot a mjpeg file!");
		return -1;
	}

	//find all APP segment
	while (cnt < len) {
		if (*ptr == 0xff && *(ptr + 1) == 0xe0) {
			if (trim_seg == NULL) {
				trim_seg = malloc(sizeof(seg_info));
				if (trim_seg == NULL) {
					printf("\n\rtrim segment create fail");
					break;
				}
				memset(trim_seg, 0, sizeof(seg_info));
				trim_seg->next = trim_seg;
			}
			temp_seg = get_seg(trim_seg);
			if (temp_seg == NULL) {
				printf("\n\rtrim segment create fail");
				goto clear;
			}
			//store trim segment info
			temp_seg->start_ptr = ptr;
			temp_seg->offset = ptr - data;
			ptr += 2;
			cnt += 2;
			seg_size_high = (*ptr) << 8;
			seg_size_low = *(ptr + 1);
			//printf("\n\r%d", seg_size_high);
			//printf("\n\r%d", seg_size_low);
			temp_seg->len = seg_size_high + seg_size_low;
			temp_seg->is_set = 1;
			//skip segment;
			ptr += temp_seg->len;
			cnt += temp_seg->len;
			output_len -= (temp_seg->len + 2);
			//printf("\n\rseg len:%d", temp_seg->len);
		}
		if (*ptr == 0xff && *(ptr + 1) == 0xda) {
			break;
		}
		ptr++;
		cnt++;
	}

	if (trim_seg == NULL) {
		return -1;
	}
	output_len += JFIF_SEG_LEN;
	output_len += dht_segment_size;
	//printf("\n\rlen:%d o_len:%d", len, output_len);

	if ((tmp_buf = malloc(output_len)) == NULL) {
		printf("\n\rcreate temp  buffer failed");
		goto clear;
	}
	//do trim here
	temp_seg = trim_seg;
	ptr = data;
	cnt = 0;
	//append SOF
	tmp1 = append(tmp_buf, ptr, 2);
	ptr += 2;
	//append APP0 JFIF
	tmp1 = append(tmp1, (char *)jfif_segment, JFIF_SEG_LEN);
	//append Huffman Table
	tmp1 = append_dht_segment(tmp1);
	if (ptr < temp_seg->start_ptr) {
		offset = temp_seg->start_ptr - ptr;
		tmp1 = append(tmp1, ptr, offset);
	}
	ptr = temp_seg->start_ptr + temp_seg->len + 2;
	temp_seg = temp_seg->next;
	while (temp_seg != trim_seg) {
		if (ptr < temp_seg->start_ptr) {
			offset = temp_seg->start_ptr - ptr;
			tmp1 = append(tmp1, ptr, offset);
		}
		ptr = temp_seg->start_ptr + temp_seg->len + 2;
		temp_seg = temp_seg->next;
	}

	offset = data + len - ptr;
	tmp1 = append(tmp1, ptr, offset);
	memcpy(data, tmp_buf, tmp1 - tmp_buf);
	in_frame->FrameData = data;
	in_frame->FrameLength = tmp1 - tmp_buf;
	free(tmp_buf);
clear:
	free_seg(trim_seg);
	return 0;
}