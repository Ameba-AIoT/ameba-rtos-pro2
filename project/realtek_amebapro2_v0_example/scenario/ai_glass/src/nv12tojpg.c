/******************************************************************************
*
* Copyright(c) 2007 - 2023 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include <stdint.h>
#include <stdlib.h>

#include "nv12tojpg.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_write.h"
//#include "test_stb_image_resize.h"

#define LIMIT(x, lower, upper) do{ if(x<lower) x=lower; else if(x>upper) x=upper; } while(0)

typedef struct {
	uint32_t max_pos;
	uint32_t last_pos;
	void *data;
} custom_stbi_mem_context;

typedef struct img_s {
	unsigned int width;
	unsigned int height;
	union {
		unsigned char *data;
		struct {
			unsigned char *y_data;
			unsigned char *uv_data;
		} yuv;
	};
} img_t;

static void custom_stbi_write_mem(void *context, void *data, int size)
{
	custom_stbi_mem_context *c = (custom_stbi_mem_context *)context;
	char *dst = (char *)c->data;
	char *src = (char *)data;
	int cur_pos = c->last_pos;
	if ((cur_pos + size) > c->max_pos) {
		printf("error: fail to write jpeg data, buffer too small\r\n");
		return;
	}
	for (int i = 0; i < size; i++) {
		dst[cur_pos++] = src[i];
	}
	c->last_pos = cur_pos;
}

static void img_nv12_resize_nearest(img_t *im_in, img_t *im_out)
{
	int sw = im_in->width;
	int sh = im_in->height;
	int dw = im_out->width;
	int dh = im_out->height;
	uint32_t srcy, srcx, src_index;
	uint32_t xrIntFloat_16 = (sw << 16) / dw + 1; //better than float division
	uint32_t yrIntFloat_16 = (sh << 16) / dh + 1;
	uint8_t *dst_uv = im_out->yuv.uv_data;
	if (!dst_uv) {
		dst_uv = im_out->data + dh * dw; //memory start pointer of dest uv
	}
	uint8_t *src_uv = im_in->data + sh * sw; //memory start pointer of source uv
	uint8_t *dst_uv_yScanline = NULL;
	uint8_t *src_uv_yScanline = NULL;
	uint8_t *dst_y_slice = im_out->data; //memory start pointer of dest y
	uint8_t *src_y_slice;
	uint8_t *sp;
	uint8_t *dp;

	for (int y = 0; y < (dh & ~7); ++y) { //'dh & ~7' is to generate faster assembly code
		srcy = (y * yrIntFloat_16) >> 16;
		src_y_slice = im_in->data + srcy * sw;

		if ((y & 1) == 0) {
			dst_uv_yScanline = dst_uv + (y / 2) * dw;
			src_uv_yScanline = src_uv + (srcy / 2) * sw;
		}

		for (int x = 0; x < (dw & ~7); ++x) {
			srcx = (x * xrIntFloat_16) >> 16;
			dst_y_slice[x] = src_y_slice[srcx];

			if ((y & 1) == 0) { //y is even
				if ((x & 1) == 0) { //x is even
					src_index = (srcx / 2) * 2;

					sp = dst_uv_yScanline + x;
					dp = src_uv_yScanline + src_index;
					*sp = *dp;
					++sp;
					++dp;
					*sp = *dp;
				}
			}
		}
		dst_y_slice += dw;
	}
}

static void img_nv12_resize_bilinear(img_t *im_in, img_t *im_out)
{
	uint8_t *src = (uint8_t *)im_in->data;
	uint8_t *dst = (uint8_t *)im_out->data;

	int x, y;
	int ox, oy;
	int tmpx, tmpy;
	int xratio = (im_in->width << 8) / im_out->width;
	int yratio = (im_in->height << 8) / im_out->height;
	uint8_t *dst_y = dst;
	uint8_t *dst_uv = im_out->yuv.uv_data;
	if (!dst_uv) {
		dst_uv = dst + im_out->height * im_out->width;
	}
	uint8_t *src_y = src;
	uint8_t *src_uv = src + im_in->height * im_in->width;
	uint8_t y_plane_color[2][2];
	int offsetY;
	int y_final;

	uint8_t *dst_uv_yScanline = NULL;
	uint8_t *src_uv_yScanline = NULL;
	unsigned long int src_index = 0;
	uint8_t *sp = NULL;
	uint8_t *dp = NULL;

	tmpy = 0;
	for (int j = 0; j < (im_out->height & ~7); ++j) {

		oy = tmpy >> 8;
		y = tmpy & 0xFF;
		tmpx = 0;
		int y_inverse_factor = 0x100 - y;

		if ((j & 1) == 0) {
			dst_uv_yScanline = dst_uv + (j / 2) * im_out->width;
			src_uv_yScanline = src_uv + (oy / 2) * im_in->width;
		}

		for (int i = 0; i < (im_out->width & ~7); ++i) {

			ox = tmpx >> 8;
			x = tmpx & 0xFF;
			int x_inverse_factor = 0x100 - x;

			offsetY = oy * im_in->width;
			//Y use bilinear
			y_plane_color[0][0] = src_y[offsetY + ox];
			y_plane_color[1][0] = src_y[offsetY + ox + 1];
			y_plane_color[0][1] = src_y[offsetY + im_in->width + ox];
			y_plane_color[1][1] = src_y[offsetY + im_in->width + ox + 1];

			y_final = x_inverse_factor * y_inverse_factor * y_plane_color[0][0]
					  + x * y_inverse_factor * y_plane_color[1][0]
					  + x_inverse_factor * y * y_plane_color[0][1]
					  + x * y * y_plane_color[1][1];
			y_final = y_final >> 16;
			LIMIT(y_final, 0, 255);
			dst_y[ j * im_out->width + i] = (uint8_t)y_final; //set Y in dest array

			//UV use nearest
			if ((j & 1) == 0) { //y is even
				if ((i & 1) == 0) { //x is even
					src_index = (ox / 2) * 2;
					sp = dst_uv_yScanline + i;
					dp = src_uv_yScanline + src_index;
					*sp = *dp;
					++sp;
					++dp;
					*sp = *dp;
				}
			}
			tmpx += xratio;
		}
		tmpy += yratio;
	}
}

//STBIR_DEFAULT_FILTER_UPSAMPLE => STBIR_FILTER_CATMULLROM
//STBIR_DEFAULT_FILTER_DOWNSAMPLE => STBIR_FILTER_MITCHELL
static void img_nv12_resize_stb(img_t *im_in, img_t *im_out)
{
#if 0
	uint8_t *dst_uv = im_out->yuv.uv_data;
	if (!dst_uv) {
		dst_uv = im_out->data + im_out->width * im_out->height;
		printf("YUV\r\n");
	}

	test_stbir_resize_uint8(im_in->data, im_in->width, im_in->height, 0, im_out->data, im_out->width, im_out->height, 0, 1);
	test_stbir_resize_uint8(im_in->data + im_in->width * im_in->height, im_in->width / 2, im_in->height / 2, 0, im_out->data + im_out->width * im_out->height,
							im_out->width / 2,
							im_out->height / 2, 0, 2);
#endif
}

static void jpegEnc_from_nv12(img_t *nv12, uint8_t *p_jpeg_buf, uint32_t jpeg_buf_size, uint32_t *p_jpeg_size, int quality)
{
	custom_stbi_mem_context context;
	context.max_pos = jpeg_buf_size;
	context.last_pos = 0;
	context.data = (void *)p_jpeg_buf;
	stbi_write_jpg_to_func_from_nv12(custom_stbi_write_mem, &context, nv12->width, nv12->height, 3, nv12->data, quality);
	*p_jpeg_size = context.last_pos;
}

/******************************************************************************
*Customized Jpeg Function
******************************************************************************/

void custom_jpegEnc_from_nv12(uint8_t *nv12_data, uint32_t nv12_width, uint32_t nv12_height, uint8_t *p_jpeg_buf, uint32_t jpeg_quality, uint32_t jpeg_buf_size,
							  uint32_t *p_jpeg_size)
{
	//create instance
	img_t im_nv12 = {
		.width = nv12_width,
		.height = nv12_height,
		.data = nv12_data,
	};

	//jpeg encode from NV12 directly
	jpegEnc_from_nv12(&im_nv12, p_jpeg_buf, jpeg_buf_size, p_jpeg_size, jpeg_quality);
}

/******************************************************************************
*Customized Resize Function
******************************************************************************/

void custom_resize_for_nv12(uint8_t *input_data, uint32_t input_width, uint32_t input_height, uint8_t *output_data, uint32_t output_width,
							uint32_t output_height)
{
	//create instance for input
	img_t input_nv12 = {
		.width = input_width,
		.height = input_height,
		.data = input_data,
		.yuv.y_data = input_data,
		.yuv.uv_data = input_data + input_width * input_height,
	};
	//create instance for output
	img_t output_nv12 = {
		.width = output_width,
		.height = output_height,
		.data = output_data,
		.yuv.y_data = output_data,
		.yuv.uv_data = output_data + output_width * output_height,
	};

	//jpeg encode from NV12 directly
	//img_nv12_resize_stb(&input_nv12, &output_nv12);
	img_nv12_resize_bilinear(&input_nv12, &output_nv12);
	//img_nv12_resize_nearest(&input_nv12, &output_nv12);
}
