//------------------------------------------------------
// yolov9tiny (PyTorch&Onnx)
//------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "nn_utils/sigmoid.h"
#include "nn_utils/iou.h"
#include "nn_utils/nms.h"
#include "nn_utils/quantize.h"
#include "nn_utils/tensor.h"
#include "model_yolov9_sim.h"

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

//MAX_DETECT_OBJ_NUM defined in module_vipnn.h
static box_t res_box[MAX_DETECT_OBJ_NUM];
static box_t *p_res_box[MAX_DETECT_OBJ_NUM];
static int box_idx;

static float yolov9_confidence_thresh = 0.3;    // default
static float yolov9_nms_thresh = 0.3;      // default

//static float *pAnchor = NULL;

typedef struct data_format_s {
	nn_tensor_format_t *format;
	nn_tensor_dim_t *dim;
} data_format_t;

void *yolov9_get_network_filename_init(void)
{
	return (void *)"NN_MDL/yolov9_tiny.nb";
}

static int yolov9_in_width, yolov9_in_height;

int yolov9_preprocess(void *data_in, nn_data_param_t *data_param, void *tensor_in, nn_tensor_param_t *tensor_param)
{
	void **tensor = (void **)tensor_in;
	rect_t *roi = &data_param->img.roi;

	yolov9_in_width  = tensor_param->dim[0].size[0];
	yolov9_in_height = tensor_param->dim[0].size[1];

	return 0;
}

static void yolov9_decode(void *bb_box_info, void *preds, int num_class, data_format_t *fmt_bb, data_format_t *fmt_p)
{
	int num_anchor = fmt_bb->dim->size[0]; //3549
	int num_bb = fmt_bb->dim->size[1];  //4
	for (int idx = 0; idx < num_anchor; idx++) {
		int cur_label = 0;

		//printf("mode: %d\n", param->format[0].buf_type);
		switch (fmt_p->format->buf_type) {
		default:
		case VIP_BUFFER_FORMAT_UINT8: {
			uint8_t *tmp_pred_u8 = (uint8_t *)preds + idx;
			for (int label = 0; label < num_class; label++) {
				if (tmp_pred_u8[label*num_anchor] > tmp_pred_u8[cur_label]) {
					cur_label = label*num_anchor;
				}
			}
			break;
		}
		case VIP_BUFFER_FORMAT_INT8: {
			int8_t *tmp_pred_s8 = (int8_t *)preds + idx;
			for (int label = 0; label < num_class; label++) {
				if (tmp_pred_s8[label*num_anchor] > tmp_pred_s8[cur_label]) {
					cur_label = label*num_anchor;
				}
			}
			break;
		}
		case VIP_BUFFER_FORMAT_INT16: {
			int16_t *tmp_pred_s16 = (int16_t *)preds + idx;
			for (int label = 0; label < num_class; label++) {
				if (tmp_pred_s16[label*num_anchor] > tmp_pred_s16[cur_label]) {
					cur_label = label*num_anchor;
				}
			}
			break;
		}
		case VIP_BUFFER_FORMAT_FP16: {
			__fp16 *tmp_pred_f16 = (__fp16 *)preds + idx;
			for (int label = 0; label < num_class; label++) {
				if (tmp_pred_f16[label] > tmp_pred_f16[cur_label]) {
					cur_label = label;
				}
			}
			break;
		}
		case VIP_BUFFER_FORMAT_FP32: {
			float *tmp_pred_f32 = (float *)preds + idx;
			for (int label = 0; label < num_class; label++) {
				if (tmp_pred_f32[label] > tmp_pred_f32[cur_label]) {
					cur_label = label;
				}
			}
			break;
		}
		}
		
		float score = get_tensor_value(preds, idx + cur_label, fmt_p->format);
		if (score > yolov9_confidence_thresh) {
			int bbox_pred_start_idx = idx;
			int bbox_pred_cnt = 4;  // 32
			float bbox_pred[bbox_pred_cnt];
			//printf("zero point: %d scale: %f\n", fmt_bb->format->zero_point, fmt_bb->format->scale);
			for (int i = 0; i < bbox_pred_cnt; i++) {
				bbox_pred[i] = get_tensor_value(bb_box_info, bbox_pred_start_idx + i*num_anchor, fmt_bb->format);
			}
			float cx = bbox_pred[0];
			float cy = bbox_pred[1];
			float ow = bbox_pred[2];
			float oh = bbox_pred[3];
			float xmin = (cx - ow*0.5) / yolov9_in_width;
			float ymin = (cy - oh*0.5) / yolov9_in_height;
			float xmax = (cx + ow*0.5) / yolov9_in_width;
			float ymax = (cy + oh*0.5) / yolov9_in_height;
			xmin = max(xmin, 0);
			ymin = max(ymin, 0);
			xmax = min(xmax, 1);
			ymax = min(ymax, 1);
			int class_id;
			class_id = cur_label / num_anchor;
			res_box[box_idx].class_idx = class_id;
			res_box[box_idx].prob = score;
			res_box[box_idx].x = xmin;
			res_box[box_idx].y = ymin;
			res_box[box_idx].w = xmax - xmin; 
			res_box[box_idx].h = ymax - ymin;
			box_idx++;

		}
	}
}

int yolov9_postprocess(void *tensor_out, nn_tensor_param_t *param, void *res)
{
	void **tensor = (void **)tensor_out;

	objdetect_res_t *od_res = (objdetect_res_t *)res;
	// reset box & index
	box_idx = 0;
	memset(res_box, 0, sizeof(res_box));

	int num_anchor = param->dim[0].size[0];;
	int classes = param->dim[1].size[1];
	
	data_format_t fmt_bb, fmt_p;
	fmt_bb.format = &param->format[0];
	fmt_bb.dim = &param->dim[0];
	fmt_p.format = &param->format[1];
	fmt_p.dim = &param->dim[1];
	yolov9_decode((void *)tensor[0], (void *)tensor[1], classes, &fmt_bb, &fmt_p);

	// NMS
	do_nms(classes, box_idx, yolov9_nms_thresh, res_box, p_res_box, DIOU);

	// dump result
	/*
	for (int i = 0; i < box_idx; i++) {
		box_t *b = &res_box[i];
		printf("x y w h = %f %f %f %f\n\r", b->x, b->y, b->w, b->h);
		printf("x y w h = %3.0f %3.0f %3.0f %3.0f\n\r", b->x * nanodet_in_width, b->y * nanodet_in_height, b->w * nanodet_in_width, b->h * nanodet_in_height);
		printf("p %2.6f, class %d, invalid %d\n\r", b->prob, b->class_idx, b->invalid);
	}
	*/

	// fill result
	int od_num = 0;
	for (int i = 0; i < box_idx; i++) {
		box_t *obj = &res_box[i];

		if (obj->invalid == 0) {
			od_res[od_num].result[0] = obj->class_idx;
			od_res[od_num].result[1] = obj->prob;
			od_res[od_num].result[2] = obj->x;	// top_x
			od_res[od_num].result[3] = obj->y;	// top_y
			od_res[od_num].result[4] = obj->x + obj->w; // bottom_x
			od_res[od_num].result[5] = obj->y + obj->h; // bottom_y
			od_num++;
		}
	}
	return od_num;
}
