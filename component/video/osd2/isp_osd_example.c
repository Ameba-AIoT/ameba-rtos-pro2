#include <platform_stdlib.h>
#include <platform_opts.h>
#include <build_info.h>
#include "log_service.h"
#include "atcmd_isp.h"
#include "main.h"
#include "flash_api.h"
#include "hal_osd_util.h"
#include "osd_custom.h"
#include "osd_pict_custom.h"
//#include "osd_font_custom.h"
#include "osd_api.h"
#include "isp_osd_example.h"
#include "video_api.h"
#include "isp_ctrl_api.h"
#include "logo_osd.h"

#define CHANGE_FONT 0
#define USE_CUSTOM_1BPP 0
#if CHANGE_FONT
#if USE_CUSTOM_1BPP
#include "custom_font_1bpp.h"
#else
#include "custom_font_argb4444.h"
#endif
#endif

#define RECT_EN 0

static osd_text_info_st s_txt_info_time;
static osd_text_info_st s_txt_info_date;
static osd_text_info_st s_txt_info_string;
static osd_text_info_st s_txt_info_iq_string[6];
static char string_buf[6][64] = {0};
static char teststring[] = "RTK-AmebaPro2";
static char teststring_empty[] = " ";

void iq_update_info(void *arg)
{
	while (1) {
		unsigned char *iq_addr = video_get_iq_buf();
		int dvalue, dvalue2, dvalue3;
		int exp, gain;

		sprintf(string_buf[0], "IQ Version: %04d/%02d/%02d %02d:%02d:%02d", *(unsigned short *)(iq_addr + 12), iq_addr[14], iq_addr[15], iq_addr[16],
				iq_addr[17], *(unsigned short *)(iq_addr + 18));

		dvalue = -1;
		isp_get_exposure_time(&dvalue);
		exp = dvalue;
		dvalue2 = -1;
		isp_get_ae_gain(&dvalue2);
		gain = dvalue2;
		//sprintf(string_buf[1], "[AE]Exposure: %.3f AE-Gain: %.3f ET-Gain: %.3f", ((float)dvalue)/1000.0f, ((float)dvalue)/256.0f, ((float)(exp*gain))/25600.0f);
		sprintf(string_buf[1], "ET:%6d AEG:%4d", dvalue, dvalue2);

		dvalue = -1;
		isp_get_red_balance(&dvalue);
		dvalue2 = -1;
		isp_get_blue_balance(&dvalue2);
		dvalue3 = -1;
		isp_get_wb_temperature(&dvalue3);
		sprintf(string_buf[2], "R-Gain:%d B-Gain:%d CT:%d", dvalue, dvalue2, dvalue3);

		dvalue = -1;
		isp_get_day_night(&dvalue);
		sprintf(string_buf[3], "Mode:%d ETGain:%.2f", dvalue, ((float)(exp * gain)) / 25600.0f);

		for (int i = 0; i < 4; i++) {
			s_txt_info_iq_string[i].str = string_buf[i];
			rts_osd_update_info(rts_osd2_type_text, &s_txt_info_iq_string[i]);
			vTaskDelay(10);
		}
		vTaskDelay(250);
	}
	vTaskDelete(NULL);
}
static void init_osd_bitmap_pos(osd_pict_st *bmp_info, int chn_id, uint32_t start_x, uint32_t start_y, uint32_t width, uint32_t height)
{
	bmp_info->chn_id = chn_id;
	bmp_info->osd2.start_x = start_x;
	bmp_info->osd2.start_y = start_y;
	bmp_info->osd2.end_x = bmp_info->osd2.start_x + width;
	bmp_info->osd2.end_y = bmp_info->osd2.start_y + height;
}
static void init_osd_bitmap_blk(osd_pict_st *bmp_info, int blk_idx, enum rts_osd2_blk_fmt blk_fmt, uint32_t clr_1bpp)
{
	bmp_info->osd2.blk_idx = blk_idx;
	bmp_info->osd2.blk_fmt = blk_fmt;
	bmp_info->osd2.color_1bpp = clr_1bpp;//0xAABBGGRR
}
static void init_osd_bitmap_buf(osd_pict_st *bmp_info, uint8_t *buf, uint32_t buf_len)
{
	bmp_info->osd2.buf = buf;
	bmp_info->osd2.len = buf_len;
}

static void init_osd_txt(osd_text_info_st *txt_info, int chn_id, int blk_idx, rt_font_st font, uint32_t rotate, uint32_t start_x, uint32_t start_y, char *str)
{
	txt_info->chn_id = chn_id;
	txt_info->font = font;
	txt_info->blk_idx = blk_idx;
	txt_info->rotate = rotate;
	txt_info->start_x = start_x;
	txt_info->start_y = start_y;
	txt_info->str = str;
}

static rt_font_st font = {
	.bg_enable		= OSD_TEXT_FONT_BG_ENABLE,
	.bg_color		= OSD_TEXT_FONT_BG_COLOR,
	.ch_color		= OSD_TEXT_FONT_CH_COLOR,
	.block_alpha	= OSD_TEXT_FONT_BLOCK_ALPHA,
	.h_gap			= OSD_TEXT_FONT_H_GAP,
	.v_gap			= OSD_TEXT_FONT_V_GAP,
	.date_fmt		= osd_date_fmt_9,
	.time_fmt		= osd_time_fmt_12_4,
};

extern void rts_osd_task(void *arg);
static osd_pict_st posd2_pic_0, posd2_pic_1, posd2_pic_2;

#if RECT_EN
static osd_pict_st posd2_rect_0, posd2_rect_1, posd2_rect_2;

static int task_on = 0;
static int dir_x[3] = {20, 20, 4};
static int dir_y[3] = {20, 10, 2};
void rect_update(void *arg)
{
	task_on = 1;
	while (task_on) {
		if (posd2_rect_0.osd2.start_x + dir_x[0] < 0 || posd2_rect_0.osd2.end_x + dir_x[0] < 0 ||
			posd2_rect_0.osd2.start_x + dir_x[0] >= 1920 || posd2_rect_0.osd2.end_x + dir_x[0] >= 1920) {
			dir_x[0] = -dir_x[0];
		}
		posd2_rect_0.osd2.start_x += dir_x[0];
		posd2_rect_0.osd2.end_x   += dir_x[0];

		//if(posd2_rect_1.osd2.start_y+dir_y[1]<0 || posd2_rect_1.osd2.end_y+dir_y[1]<0 ||
		//	posd2_rect_1.osd2.start_y+dir_y[1]>=1080 || posd2_rect_1.osd2.end_y+dir_y[1]>=1080) {
		//	dir_y[1] = -dir_y[1];
		//}
		//posd2_rect_1.osd2.start_y += dir_y[1];
		//posd2_rect_1.osd2.end_y   += dir_y[1];
		if (posd2_rect_1.osd2.start_x >= posd2_rect_1.osd2.end_x ||
			posd2_rect_1.osd2.start_y >= posd2_rect_1.osd2.end_y ||
			posd2_rect_1.osd2.start_x < 0 || posd2_rect_1.osd2.end_x < 0 ||
			posd2_rect_1.osd2.start_y < 0 || posd2_rect_1.osd2.end_y < 0 ||
			posd2_rect_1.osd2.start_x >= 1920 || posd2_rect_1.osd2.end_x >= 1920 ||
			posd2_rect_1.osd2.start_y >= 1080 || posd2_rect_1.osd2.end_y >= 1080) {
			dir_x[1] = -dir_x[1];
			dir_y[1] = -dir_y[1];
		}
		//printf("X: %d %d  Y: %d %d\r\n", posd2_rect_1.osd2.start_x, posd2_rect_1.osd2.end_x, posd2_rect_1.osd2.start_y, posd2_rect_1.osd2.end_y);
		posd2_rect_1.osd2.start_x += dir_x[1];
		posd2_rect_1.osd2.end_x   -= dir_x[1];
		posd2_rect_1.osd2.start_y += dir_y[1];
		posd2_rect_1.osd2.end_y   -= dir_y[1];

		if (posd2_rect_2.osd2.start_x >= posd2_rect_2.osd2.end_x ||
			posd2_rect_2.osd2.start_y >= posd2_rect_2.osd2.end_y ||
			posd2_rect_2.osd2.start_x < 0 || posd2_rect_2.osd2.end_x < 0 ||
			posd2_rect_2.osd2.start_y < 0 || posd2_rect_2.osd2.end_y < 0 ||
			posd2_rect_2.osd2.start_x >= 1920 || posd2_rect_2.osd2.end_x >= 1920 ||
			posd2_rect_2.osd2.start_y >= 1080 || posd2_rect_2.osd2.end_y >= 1080) {
			dir_x[2] = -dir_x[2];
			dir_y[2] = -dir_y[2];
		}
		//printf("X: %d %d  Y: %d %d\r\n", posd2_rect_2.osd2.start_x, posd2_rect_2.osd2.end_x, posd2_rect_2.osd2.start_y, posd2_rect_2.osd2.end_y);
		posd2_rect_2.osd2.start_x += dir_x[2];
		posd2_rect_2.osd2.end_x   -= dir_x[2];
		posd2_rect_2.osd2.start_y += dir_y[2];
		posd2_rect_2.osd2.end_y   -= dir_y[2];

		rts_osd_update_info(rts_osd2_type_rect_extn, &posd2_rect_0);
		rts_osd_update_info(rts_osd2_type_rect_extn, &posd2_rect_1);
		rts_osd_update_info(rts_osd2_type_rect_extn, &posd2_rect_2);

		vTaskDelay(50);
	}
	vTaskDelete(NULL);
}
#endif
static const int ach_id[RTS_MAX_STM_COUNT] = {0, 1, 2, 3, 4};
static unsigned char *resize0_osd[RTS_MAX_STM_COUNT] = {NULL};
static unsigned char *resize1_osd[RTS_MAX_STM_COUNT] = {NULL};
static unsigned char *resize2_osd[RTS_MAX_STM_COUNT] = {NULL};
void example_isp_osd(int idx, int ch_id, int txt_w, int txt_h)
{
	int ch = ch_id;
	printf("Text/Logo OSD Test\r\n");

	font.osd_char_w		= txt_w;
	font.osd_char_h		= txt_h;
	if (idx == 0) {
		int resize0_w = PICT4_WIDTH * 7 / 6;
		int resize0_h = PICT4_HEIGHT * 7 / 6;
		int resize0_f = PICT4_BLK_FMT;
		int resize0_s = rts_osd_pict_heapsize_cal(resize0_w, resize0_h, resize0_f);
		int resize1_w = PICT3_WIDTH * 4 / 4;
		int resize1_h = PICT3_HEIGHT * 4 / 4;
		int resize1_f = PICT3_BLK_FMT;
		int resize1_s = rts_osd_pict_heapsize_cal(resize1_w, resize1_h, resize1_f);
		int resize2_w = PICT2_WIDTH * 5 / 3;
		int resize2_h = PICT2_HEIGHT * 5 / 3;
		int resize2_f = PICT2_BLK_FMT;
		int resize2_s = rts_osd_pict_heapsize_cal(resize2_w, resize2_h, resize2_f);
		if (resize0_osd[ch]) {
			free(resize0_osd[ch]);
		}
		if (resize1_osd[ch]) {
			free(resize1_osd[ch]);
		}
		if (resize2_osd[ch]) {
			free(resize2_osd[ch]);
		}

		printf("[osd] Heap available:%d\r\n", xPortGetFreeHeapSize());
		rts_osd_set_frame_size(ch_id, 1920, 1080);
		rts_osd_init(ch_id, txt_w, txt_h, (int)(8.0f * 3600));

		resize0_w = (resize0_w + 1) & (~1); //2 alignment
		resize0_h = (resize0_h + 1) & (~1); //2 alignment
		resize0_osd[ch] = malloc(resize0_s);
		rts_osd_pict_resize(resize0_osd[ch], PICT4_NAME, PICT4_WIDTH, PICT4_HEIGHT, resize0_w, resize0_h, resize0_f);

		resize1_w = (resize1_w + 1) & (~1); //2 alignment
		resize1_h = (resize1_h + 1) & (~1); //2 alignment
		resize1_osd[ch] = malloc(resize1_s);
		rts_osd_pict_resize(resize1_osd[ch], PICT3_NAME, PICT3_WIDTH, PICT3_HEIGHT, resize1_w, resize1_h, resize1_f);

		resize2_w = (resize2_w + 1) & (~1); //2 alignment
		resize2_h = (resize2_h + 1) & (~1); //2 alignment
		resize2_osd[ch] = malloc(resize2_s);
		rts_osd_pict_resize(resize2_osd[ch], PICT2_NAME, PICT2_WIDTH, PICT2_HEIGHT, resize2_w, resize2_h, resize2_f);

		if (ch == 0) {
#if CHANGE_FONT
			rts_set_font_char_size(ch_id, txt_w, txt_h, eng_bin_custom_32x64, NULL);
#endif
			init_osd_txt(&s_txt_info_time, ch, 0, font, OSD_TEXT_ROTATE, 10 + 320 + 50, 10, 0);
			init_osd_txt(&s_txt_info_date, ch, 1, font, OSD_TEXT_ROTATE, 10, 10, 0);
			init_osd_txt(&s_txt_info_string, ch, 5, font, RT_ROTATE_90R, 10, 10 + 100, teststring);

			init_osd_bitmap_pos(&posd2_pic_0, ch, 150, 200, resize0_w, resize0_h);
			init_osd_bitmap_pos(&posd2_pic_1, ch, 150 + resize0_w + 50, 200, resize1_w, resize1_h);
			init_osd_bitmap_pos(&posd2_pic_2, ch, 150 + resize0_w + 50 + resize1_w + 50, 200, resize2_w, resize2_h);
		} else if (ch == 1) {
			init_osd_txt(&s_txt_info_time, ch, 0, font, OSD_TEXT_ROTATE, 10, 10, 0);
			init_osd_txt(&s_txt_info_date, ch, 1, font, OSD_TEXT_ROTATE, 10 + 320 + 50, 10, 0);
			init_osd_txt(&s_txt_info_string, ch, 5, font, RT_ROTATE_90R, 10, 10 + 100, teststring);

			init_osd_bitmap_pos(&posd2_pic_0, ch, 150 + resize0_w + 50 + resize1_w + 50, 200, resize0_w, resize0_h);
			init_osd_bitmap_pos(&posd2_pic_1, ch, 150, 300, resize1_w, resize1_h);
			init_osd_bitmap_pos(&posd2_pic_2, ch, 150 + resize0_w + 50, 400, resize2_w, resize2_h);
		} else if (ch == 2) {
			init_osd_txt(&s_txt_info_time, ch, 0, font, OSD_TEXT_ROTATE, 10, 10, 0);
			init_osd_txt(&s_txt_info_date, ch, 1, font, OSD_TEXT_ROTATE, 10 + 320 + 50, 10, 0);
			init_osd_txt(&s_txt_info_string, ch, 5, font, RT_ROTATE_90R, 10, 10 + 100, teststring);

			init_osd_bitmap_pos(&posd2_pic_0, ch, 150 + resize0_w + 50, 200, resize0_w, resize0_h);
			init_osd_bitmap_pos(&posd2_pic_1, ch, 150 + resize0_w + 50 + resize1_w + 50, 350, resize1_w, resize1_h);
			init_osd_bitmap_pos(&posd2_pic_2, ch, 150, 450, resize2_w, resize2_h);
		}
		init_osd_bitmap_blk(&posd2_pic_0, 2, resize0_f, 0);
		init_osd_bitmap_buf(&posd2_pic_0, resize0_osd[ch], resize0_s);
		init_osd_bitmap_blk(&posd2_pic_1, 3, resize1_f, 0);
		init_osd_bitmap_buf(&posd2_pic_1, resize1_osd[ch], resize1_s);
		init_osd_bitmap_blk(&posd2_pic_2, 4, resize2_f, 0x000000FF);
		init_osd_bitmap_buf(&posd2_pic_2, resize2_osd[ch], resize2_s);

		rts_osd_set_info(rts_osd2_type_date, &s_txt_info_date);
		rts_osd_set_info(rts_osd2_type_time, &s_txt_info_time);
		rts_osd_set_info(rts_osd2_type_pict, &posd2_pic_0);
		rts_osd_set_info(rts_osd2_type_pict, &posd2_pic_1);
		rts_osd_set_info(rts_osd2_type_pict, &posd2_pic_2);
		rts_osd_set_info(rts_osd2_type_text, &s_txt_info_string);

		printf("[osd] Heap available:%d\r\n", xPortGetFreeHeapSize());

		if (xTaskCreate(rts_osd_task, "OSD", 10 * 1024, (void *)(ach_id + ch), tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
			printf("\n\r%s xTaskCreate failed", __FUNCTION__);
		}

#if RECT_EN
		// setup line width.
		posd2_rect_0.osd2.RSVD[0] = 10;
		posd2_rect_1.osd2.RSVD[0] = 6;
		posd2_rect_2.osd2.RSVD[0] = 4;

		// setup force update.
		posd2_rect_0.osd2.RSVD[4] = 0;
		posd2_rect_1.osd2.RSVD[4] = 0;
		posd2_rect_2.osd2.RSVD[4] = 1;

		int offset = 256;
		init_osd_bitmap_pos(&posd2_rect_0, ch, 150, 200 + offset, resize0_w, resize0_h);
		init_osd_bitmap_pos(&posd2_rect_1, ch, 150 + resize0_w + 50 + resize1_w, 200 + offset, resize1_w, resize1_h);
		init_osd_bitmap_pos(&posd2_rect_2, ch, 150 + resize0_w + 50 + resize1_w + 50, 200 + offset, resize2_w, resize2_h);
		init_osd_bitmap_blk(&posd2_rect_0, 6, RTS_OSD2_BLK_FMT_RGBA2222, 0xFFFF0000);
		init_osd_bitmap_blk(&posd2_rect_1, 7, resize1_f, 0xFF00FF00);
		init_osd_bitmap_blk(&posd2_rect_2, 8, resize2_f, 0xFF0000FF);
		rts_osd_set_info(rts_osd2_type_rect_extn, &posd2_rect_0);
		rts_osd_set_info(rts_osd2_type_rect_extn, &posd2_rect_1);
		rts_osd_set_info(rts_osd2_type_rect_extn, &posd2_rect_2);
		if (xTaskCreate(rect_update, "osd_iq_update", 10 * 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
			printf("\n\r%s xTaskCreate failed", __FUNCTION__);
		}
#endif
	} else if (idx == 1) {
		hal_video_print(0);
		printf("[osd] Heap available:%d\r\n", xPortGetFreeHeapSize());
		rts_osd_init(ch_id, txt_w, txt_h, (int)(8.0f * 3600));

		for (int i = 0; i < 6; i++) {
			init_osd_txt(s_txt_info_iq_string + i, ch, i, font, RT_ROTATE_0, 10, 10 + 10 + (txt_h + 5) * i, teststring_empty);
		}

		rts_osd_set_info(rts_osd2_type_text, &s_txt_info_iq_string[0]);
		rts_osd_set_info(rts_osd2_type_text, &s_txt_info_iq_string[1]);
		rts_osd_set_info(rts_osd2_type_text, &s_txt_info_iq_string[2]);
		rts_osd_set_info(rts_osd2_type_text, &s_txt_info_iq_string[3]);
		//rts_osd_set_info(rts_osd2_type_text, &s_txt_info_iq_string[4]);
		//rts_osd_set_info(rts_osd2_type_text, &s_txt_info_iq_string[5]);

		printf("[osd] Heap available:%d\r\n", xPortGetFreeHeapSize());

		if (xTaskCreate(rts_osd_task, "OSD", 10 * 1024, (void *)(ach_id + ch), tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
			printf("\n\r%s xTaskCreate failed", __FUNCTION__);
		}
		if (xTaskCreate(iq_update_info, "osd_iq_update", 10 * 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
			printf("\n\r%s xTaskCreate failed", __FUNCTION__);
		}
	}
}