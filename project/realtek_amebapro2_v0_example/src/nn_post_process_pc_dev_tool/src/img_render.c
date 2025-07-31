#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "module_vipnn.h"
#include "img_render.h"
#include "nbg_reader.h"

#define LIMIT(x, lower, upper) do{if(x<lower) x=lower; else if(x>upper) x=upper;}while(0)

void draw_box(uint8_t *img_buf, int x1, int y1, int x2, int y2, int width, int height, uint8_t r, uint8_t g, uint8_t b)
{
	LIMIT(x1, 0, width - 1);
	LIMIT(x2, 0, width - 1);
	LIMIT(y1, 0, height - 1);
	LIMIT(y2, 0, height - 1);

	for (int i = x1; i <= x2; ++i) {
		img_buf[i + y1 * width + 0 * width * height] = r;
		img_buf[i + y2 * width + 0 * width * height] = r;

		img_buf[i + y1 * width + 1 * width * height] = g;
		img_buf[i + y2 * width + 1 * width * height] = g;

		img_buf[i + y1 * width + 2 * width * height] = b;
		img_buf[i + y2 * width + 2 * width * height] = b;
	}
	for (int i = y1; i <= y2; ++i) {
		img_buf[x1 + i * width + 0 * width * height] = r;
		img_buf[x2 + i * width + 0 * width * height] = r;

		img_buf[x1 + i * width + 1 * width * height] = g;
		img_buf[x2 + i * width + 1 * width * height] = g;

		img_buf[x1 + i * width + 2 * width * height] = b;
		img_buf[x2 + i * width + 2 * width * height] = b;
	}
}

void draw_box_width(uint8_t *img_buf, int x1, int y1, int x2, int y2, int width, int height, int w, uint8_t r, uint8_t g, uint8_t b)
{
	for (int i = 0; i < w; ++i) {
		draw_box(img_buf, x1 + i, y1 + i, x2 - i, y2 - i, width, height, r, g, b);
	}
}

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

void image_rendering(void *post_res, char *imge_name_in, char *imge_name_out, nn_res_t type)
{
	vipnn_out_buf_t *out = (vipnn_out_buf_t *)post_res;

	objdetect_res_t *objdet_res = (objdetect_res_t *)&out->res[0];
	facedetect_res_t *facedet_res = (facedetect_res_t *)&out->res[0];

	int obj_num = out->res_cnt;

	int w, h, c;
	int channels = 3;
	FILE *fp = fopen(imge_name_in, "rb");
	uint8_t *im_data = stbi_load_from_file(fp, &w, &h, &c, channels);
	printf("\r\nimage data size: w:%d, h:%d, c:%d\r\n", w, h, c);

	if (c != 1 && c != 3) {
		printf("error: it's not an image file\r\n");
		return;
	}

	/* rgb packed to rgb planar */
	int data_size = w * h * c;
	uint8_t *rgb_planar_buf = (uint8_t *)malloc(data_size);
	for (int k = 0; k < c; k++) {
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				int dst_i = i + w * j + w * h * k;
				int src_i = k + c * i + c * w * j;
				rgb_planar_buf[dst_i] = im_data[src_i];
			}
		}
	}

	/* draw bbox */
	printf("object num = %d\r\n", obj_num);
	if (obj_num > 0) {
		for (int i = 0; i < obj_num; i++) {
			int class_id, xmin, ymin, xmax, ymax;
			if (type == OBJDECT_TYPE) {
				class_id = (int)objdet_res[i].result[0];
				xmin = (int)(objdet_res[i].result[2] * w);
				ymin = (int)(objdet_res[i].result[3] * h);
				xmax = (int)(objdet_res[i].result[4] * w);
				ymax = (int)(objdet_res[i].result[5] * h);
			} else if (type == FACEDECT_TYPE) {
				class_id = (int)facedet_res[i].result[0];
				xmin = (int)(facedet_res[i].result[2] * w);
				ymin = (int)(facedet_res[i].result[3] * h);
				xmax = (int)(facedet_res[i].result[4] * w);
				ymax = (int)(facedet_res[i].result[5] * h);
			}
			LIMIT(xmin, 0, w);
			LIMIT(xmax, 0, w);
			LIMIT(ymin, 0, h);
			LIMIT(ymax, 0, h);
			printf("%d,c%d:%d %d %d %d\n\r", i, class_id, xmin, ymin, xmax, ymax);

			draw_box_width(rgb_planar_buf, xmin, ymin, xmax, ymax, w, h, 3, 255, 255, 255);
		}
	}

	/* save image */
	for (int k = 0; k < c; ++k) {
		for (int i = 0; i < w * h; ++i) {
			im_data[i * c + k] = (uint8_t)(rgb_planar_buf[i + k * w * h]);
		}
	}
	stbi_write_jpg(imge_name_out, w, h, c, im_data, 80);

	free(rgb_planar_buf);
	stbi_image_free(im_data);
	fclose(fp);

	printf("save image to %s done \r\n", imge_name_out);
}
