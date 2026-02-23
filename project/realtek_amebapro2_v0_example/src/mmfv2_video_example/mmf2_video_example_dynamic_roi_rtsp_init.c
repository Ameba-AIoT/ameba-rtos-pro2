/******************************************************************************
*
* Copyright(c) 2007 - 2026 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "module_rtsp2.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "log_service.h"
#include "sensor.h"
#include "module_vipnn.h"
#include "model_yolo.h"
#include "nn_utils/class_name.h"
#include "osd_render.h"
/*
Example Description:
	This example demonstrates a dynamic ROI switching across multiple video channels (CH0, CH1, CH4).

1. Resolution & ROI Configuration:
	- Max ROI: Original Resolution (2560x1440)
	- Min ROI: 1/2 Width x 1/2 Height of Original Resolution (1280x720)

2. Channel Resolution Settings:
	- CH0: (Max ROI + Min ROI) / 2 -> RTSP Streaming
			(e.g., 1920x1080)
	- CH1: Min ROI -> RTSP Streaming
			(e.g., 1280x720)
	- CH4: Fixed size smaller than Min ROI -> AI Inference
			(e.g., 416x416)

3. Dynamic ROI Flow:
	Use DYN_ZOOM_MODE to switch between scale-down and scale-up modes.
	(1) Scale Down
		CH0: 1:1 Crop -> Max ROI -> 1:1 Crop
		CH1: No ROI
		CH4: 1:1 Crop -> Max ROI -> 1:1 Crop

	(2) Scale Up (support in voe1680 and later version)
		CH0, CH1, CH4: 1:1 Crop -> Min ROI -> 1:1 Crop

*/

#define SCALE_DOWN_MODE 0
#define SCALE_UP_MODE 1
#define DYN_ZOOM_MODE SCALE_DOWN_MODE

static void atcmd_userctrl_init(void);
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v1		= NULL;

static mm_context_t *video_v2_ctx			= NULL;
static mm_context_t *rtsp2_v2_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v2 = NULL;

static mm_context_t *video_rgb_ctx			= NULL;
static mm_context_t *vipnn_ctx            = NULL;
static mm_siso_t *siso_array_vipnn         = NULL;

#define V1_CHANNEL 0
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR
#define USE_H265 0
#if USE_H265
#include "sample_h265.h"
#define VIDEO_TYPE VIDEO_HEVC
#define VIDEO_CODEC AV_CODEC_ID_H265
#else
#include "sample_h264.h"
#define VIDEO_TYPE VIDEO_H264
#define VIDEO_CODEC AV_CODEC_ID_H264
#endif
static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.bps = V1_BPS,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1
};

static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.bps      = V1_BPS
		}
	}
};

#define V2_CHANNEL 1
#define V2_BPS 512*1024
#define V2_RCMODE 2 // 1: CBR, 2: VBR
static video_params_t video_v2_params = {
	.stream_id = V2_CHANNEL,
	.type = VIDEO_TYPE,
	.bps = V2_BPS,
	.rc_mode = V2_RCMODE,
	.use_static_addr = 1
};

static rtsp2_params_t rtsp2_v2_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.bps      = V2_BPS
		}
	}
};

#define NN_CHANNEL 4
#define NN_RESOLUTION VIDEO_VGA //don't care for NN
#define NN_FPS 10
#define NN_GOP NN_FPS //don't care for NN
#define NN_BPS 1024*1024 //don't care for NN
#define NN_TYPE VIDEO_RGB
/* model selection: yolov4_tiny, yolov7_tiny, nanodet_plus_m, yolov9_tiny
 * please make sure the choosed model is also selected in amebapro2_fwfs_nn_models.json */
#define NN_MODEL_OBJ    yolov4_tiny
/* RGB video resolution
 * please make sure the resolution is matched to model input size and thereby to avoid SW image resizing */
#define NN_WIDTH	416
#define NN_HEIGHT	416

static float nn_confidence_thresh = 0.5;
static float nn_nms_thresh = 0.3;
static int desired_class_list[] = {0, 2, 5, 7}; //represent the objects to be detected, please refer to yolov9/data/coco.yaml for the category of class id
static nn_desired_class_t desired_class_param = {
	.class_info = desired_class_list,
	.len = (sizeof(desired_class_list) / sizeof(int))
};

static video_params_t video_v4_params = {
	.stream_id 		= NN_CHANNEL,
	.type 			= NN_TYPE,
	.resolution	 	= NN_RESOLUTION,
	.width 			= NN_WIDTH,
	.height 		= NN_HEIGHT,
	.bps 			= NN_BPS,
	.fps 			= NN_FPS,
	.gop 			= NN_GOP,
	.direct_output 	= 0,
	.use_static_addr = 1,
};

static nn_data_param_t roi_nn = {
	.img = {
		.width = NN_WIDTH,
		.height = NN_HEIGHT,
		.rgb = 0, // set to 1 if want RGB->BGR or BGR->RGB
		.roi = {
			.xmin = 0,
			.ymin = 0,
			.xmax = NN_WIDTH,
			.ymax = NN_HEIGHT,
		}
	},
	.codec_type = AV_CODEC_ID_RGB888
};

static int osd_idx_zoom_info = 0;
static void osd_show_zoom_info(int ch, int input_width, int input_height)
{	
	canvas_create_bitmap(ch, osd_idx_zoom_info, RTS_OSD2_BLK_FMT_1BPP);
#if DYN_ZOOM_MODE == SCALE_DOWN_MODE
	char text_str[20] = "scale down";
#else
	char text_str[20] = "scale up";
#endif
	canvas_set_text(ch, osd_idx_zoom_info, 50, 50, text_str, COLOR_CYAN);
	snprintf(text_str, sizeof(text_str), "roi %dx%d", input_width, input_height);
	canvas_set_text(ch, osd_idx_zoom_info, 50, 100, text_str, COLOR_CYAN);
	canvas_update(ch, osd_idx_zoom_info, 1);
}

#define LIMIT(x, lower, upper) if(x<lower) x=lower; else if(x>upper) x=upper;
static int check_in_list(int class_indx)
{
	for (int i = 0; i < (sizeof(desired_class_list) / sizeof(int)); i++) {
		if (class_indx == desired_class_list[i]) {
			return class_indx;
		}
	}
	return -1;
}

static int osd_idx_nn_draw = 1;
static void nn_set_object(void *p, void *img_param)
{
	int i = 0;
	vipnn_out_buf_t *out = (vipnn_out_buf_t *)p;
	objdetect_res_t *res = (objdetect_res_t *)&out->res[0];

	int obj_num = out->res_cnt;

	nn_data_param_t *im = (nn_data_param_t *)img_param;

	if (!p || !img_param)	{
		return;
	}

	int im_h = video_v1_params.height;
	int im_w = video_v1_params.width;

	float ratio_w = (float)im_w / (float)im->img.width;
	float ratio_h = (float)im_h / (float)im->img.height;
	int roi_h, roi_w, roi_x, roi_y;
	int use_roi = 0;
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_GET_ROI_STAT, (int)&use_roi);
	if (use_roi) { //resize
		roi_w = (int)((im->img.roi.xmax - im->img.roi.xmin) * ratio_w);
		roi_h = (int)((im->img.roi.ymax - im->img.roi.ymin) * ratio_h);
		roi_x = (int)(im->img.roi.xmin * ratio_w);
		roi_y = (int)(im->img.roi.ymin * ratio_h);
	} else {  //crop
		float ratio = ratio_h < ratio_w ? ratio_h : ratio_w;
		roi_w = (int)((im->img.roi.xmax - im->img.roi.xmin) * ratio);
		roi_h = (int)((im->img.roi.ymax - im->img.roi.ymin) * ratio);
		roi_x = (int)(im->img.roi.xmin * ratio + (im_w - roi_w) / 2);
		roi_y = (int)(im->img.roi.ymin * ratio + (im_h - roi_h) / 2);
	}

	//printf("object num = %d\r\n", obj_num);
	canvas_create_bitmap(V1_CHANNEL, osd_idx_nn_draw, RTS_OSD2_BLK_FMT_1BPP);
	if (obj_num > 0) {
		for (i = 0; i < obj_num; i++) {
			int obj_class = (int)res[i].result[0];
			int class_id = check_in_list(obj_class); //show class in desired_class_list
			//int class_id = obj_class; //coco label
			if (class_id != -1) {
				int xmin = (int)(res[i].result[2] * roi_w) + roi_x;
				int ymin = (int)(res[i].result[3] * roi_h) + roi_y;
				int xmax = (int)(res[i].result[4] * roi_w) + roi_x;
				int ymax = (int)(res[i].result[5] * roi_h) + roi_y;
				LIMIT(xmin, 0, im_w)
				LIMIT(xmax, 0, im_w)
				LIMIT(ymin, 0, im_h)
				LIMIT(ymax, 0, im_h)
				//printf("%d,c%d:%d %d %d %d\n\r", i, class_id, xmin, ymin, xmax, ymax);
				canvas_set_rect(V1_CHANNEL, osd_idx_nn_draw, xmin, ymin, xmax, ymax, 3, COLOR_WHITE);
				char text_str[20];
				snprintf(text_str, sizeof(text_str), "%s %d", coco_name_get_by_id(class_id), (int)(res[i].result[1] * 100));
				canvas_set_text(V1_CHANNEL, osd_idx_nn_draw, xmin, ymin - 32, text_str, COLOR_CYAN);
			}
		}
	}
	canvas_update(V1_CHANNEL, osd_idx_nn_draw, 1);
}

static void crop_info_update(isp_crop_t *crop_info, int start_x, int start_y, int width, int height)
{
	crop_info->start_x = start_x;
	crop_info->start_y = start_y;
	crop_info->width = width;
	crop_info->height = height;
}

void mmf2_video_example_dynamic_roi_rtsp_init(void)
{
	atcmd_userctrl_init();
	
	// setup roi max, min width
	int max_width = (sensor_params[USE_SENSOR].sensor_width & ~15), max_height = (sensor_params[USE_SENSOR].sensor_height & ~3);
	int min_width = (sensor_params[USE_SENSOR].sensor_width / 2 & ~15), min_height = (sensor_params[USE_SENSOR].sensor_height / 2 & ~3);
	const int steps = 20;
	
	/*sensor capacity check & video parameter setting*/
	video_v1_params.resolution = VIDEO_FHD;
	video_v1_params.width = (max_width + min_width) / 2 & ~15; //align-16
	video_v1_params.height = (max_height + min_height) / 2 & ~3; //align-4
	video_v1_params.fps = sensor_params[USE_SENSOR].sensor_fps;
	video_v1_params.gop = sensor_params[USE_SENSOR].sensor_fps;
	//set up ch0 1x1 output roi
	video_roi_t roi;
	roi.xmin = 0;
	roi.ymin = 0;
	roi.xmax = roi.xmin + video_v1_params.width;
	roi.ymax = roi.ymin + video_v1_params.height;
	video_v1_params.use_roi = 1;
	memcpy(&(video_v1_params.roi), &roi, sizeof(roi));
	rtsp2_v1_params.u.v.fps = sensor_params[USE_SENSOR].sensor_fps;

	video_v2_params.width = min_width;
	video_v2_params.height = min_height;
	video_v2_params.fps = sensor_params[USE_SENSOR].sensor_fps;
	video_v2_params.gop = sensor_params[USE_SENSOR].sensor_fps;
	rtsp2_v2_params.u.v.fps = sensor_params[USE_SENSOR].sensor_fps;
	
	//align ch4 roi with ch0 settings
	video_v4_params.use_roi = 1;
	memcpy(&(video_v4_params.roi), &roi, sizeof(roi));

#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, video_v1_params.width, video_v1_params.height, V1_BPS, 0,
						1, video_v2_params.width, video_v2_params.height, V2_BPS, 0,
						0, 0, 0, 0, 0,
						1, video_v4_params.width, video_v4_params.height);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 0, NULL, 0, NULL, 0, NULL);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	// ------ Channel 1--------------
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_example_dynamic_roi_rtsp_fail;
	}

	//--------------RTSP---------------
	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);

	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_video_example_dynamic_roi_rtsp_fail;
	}

	//--------------Link---------------------------
	siso_video_rtsp_v1 = siso_create();
	if (siso_video_rtsp_v1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v1_ctx, 0);
		siso_start(siso_video_rtsp_v1);
	} else {
		printf("siso_video_rtsp_v1 open fail\n\r");
		goto mmf2_video_example_dynamic_roi_rtsp_fail;
	}
	printf("siso_video_rtsp_v1 started\n\r");

	video_v2_ctx = mm_module_open(&video_module);
	if (video_v2_ctx) {
		mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v2_params);
		mm_module_ctrl(video_v2_ctx, MM_CMD_SET_QUEUE_LEN, video_v2_params.fps * 3);
		mm_module_ctrl(video_v2_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_example_dynamic_roi_rtsp_fail;
	}

	rtsp2_v2_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v2_ctx) {
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v2_params);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_video_example_dynamic_roi_rtsp_fail;
	}

	siso_video_rtsp_v2 = siso_create();
	if (siso_video_rtsp_v2) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_rtsp_v2, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_rtsp_v2, MMIC_CMD_ADD_INPUT, (uint32_t)video_v2_ctx, 0);
		siso_ctrl(siso_video_rtsp_v2, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v2_ctx, 0);
		siso_start(siso_video_rtsp_v2);
	} else {
		rt_printf("simo open fail\n\r");
		goto mmf2_video_example_dynamic_roi_rtsp_fail;
	}

	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_example_dynamic_roi_rtsp_fail;
	}

	vipnn_ctx = mm_module_open(&vipnn_module);
	if (vipnn_ctx) {
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_DISPPOST, (int)nn_set_object);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_CONFIDENCE_THRES, (int)&nn_confidence_thresh);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_NMS_THRES, (int)&nn_nms_thresh);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_DESIRED_CLASS, (int)&desired_class_param);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(objdetect_res_t));		// result size
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count

		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_video_example_dynamic_roi_rtsp_fail;
	}

	siso_array_vipnn = siso_create();
	if (siso_array_vipnn) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_array_vipnn, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_array_vipnn, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		siso_ctrl(siso_array_vipnn, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_array_vipnn, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_ctrl(siso_array_vipnn, MMIC_CMD_ADD_OUTPUT, (uint32_t)vipnn_ctx, 0);
		siso_start(siso_array_vipnn);
	} else {
		printf("siso_array_vipnn open fail\n\r");
		goto mmf2_video_example_dynamic_roi_rtsp_fail;
	}
	
	int ch_enable[3] = {1, 0, 0};
	int char_resize_w[3] = {16, 0, 0}, char_resize_h[3] = {32, 0, 0};
	int ch_width[3] = {video_v1_params.width, 0, 0}, ch_height[3] = {video_v1_params.height, 0, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);
	isp_crop_t crop_info;

#if DYN_ZOOM_MODE == SCALE_DOWN_MODE
	video_v1_params.dyn_scale_up_en = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_APPLY, V2_CHANNEL);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);

	//1:1 -> scale down -> 1:1
	for (int i = 0; i <= 2 * steps; i++) {
		int input_width, input_height;
		if(i <= steps) {
			//1:1 -> scale down
			input_width = video_v1_params.width + (max_width - video_v1_params.width) * i / steps;
			input_height = video_v1_params.height + (max_height - video_v1_params.height) * i / steps;
		} else {
			//scale down -> 1:1
			input_width = max_width - (max_width - video_v1_params.width) * (i - steps) / steps;
			input_height = max_height - (max_height - video_v1_params.height) * (i - steps) / steps;
		}
		input_width = (input_width + 1) & ~1;  //force 2 aligned
		input_height = (input_height + 1) & ~1;  //force 2 aligned

		//set dynamic zoom to ch0, ch4
		crop_info_update(&crop_info, 0, 0, input_width, input_height);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_DYN_ROI, (int)&crop_info);
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_DYN_ROI, (int)&crop_info);
		printf("scale down %dx%d->%dx%d\n", input_width, input_height, video_v1_params.width, video_v1_params.height);
		
		osd_show_zoom_info(V1_CHANNEL, input_width, input_height);
		vTaskDelay(500);
	}

#elif DYN_ZOOM_MODE == SCALE_UP_MODE
	//1:1 -> scale up -> 1:1
	video_v1_params.dyn_scale_up_en = 1;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_APPLY, V2_CHANNEL);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);

	for (int i = 0; i <= steps * 2; i++) {
		int input_width, input_height;
		if(i <= steps) {
			//1:1 -> scale up
			input_width = video_v1_params.width - (video_v1_params.width - min_width) * i / steps;
			input_height = video_v1_params.height - (video_v1_params.height - min_height) * i / steps;
		} else {
			//scale up -> 1:1
			input_width = min_width + (video_v1_params.width - min_width) * (i - steps) / steps;
			input_height = min_height + (video_v1_params.height - min_height) * (i - steps) / steps;
		}

		input_width = (input_width + 1) & ~1;  //force 2 aligned
		input_height = (input_height + 1) & ~1;  //force 2 aligned
		crop_info_update(&crop_info, 0, 0, input_width, input_height);
		//when ch0 set scale up roi will automatically apply to all video channel
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_DYN_ROI, (int)&crop_info);
		printf("scale up %dx%d->%dx%d\n", input_width, input_height, video_v1_params.width, video_v1_params.height);
		
		osd_show_zoom_info(V1_CHANNEL, input_width, input_height);
		vTaskDelay(500);
	}
#endif

	return;
mmf2_video_example_dynamic_roi_rtsp_fail:

	return;

}

static const char *example = "mmf2_video_example_av_rtsp_mp4";
static void example_deinit(void)
{
	osd_render_task_stop();
	osd_render_dev_deinit_all();

	//Pause Linker
	siso_pause(siso_video_rtsp_v1);
	siso_pause(siso_video_rtsp_v2);
	siso_pause(siso_array_vipnn);

	//Stop module
	mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
	mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_STREAM_STOP, V2_CHANNEL);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_STREAM_STOP, NN_CHANNEL);

	//Delete linker
	siso_delete(siso_video_rtsp_v1);
	siso_delete(siso_video_rtsp_v2);
	siso_delete(siso_array_vipnn);

	//Close module
	mm_module_close(rtsp2_v1_ctx);
	mm_module_close(video_v1_ctx);
	mm_module_close(rtsp2_v2_ctx);
	mm_module_close(video_v2_ctx);
	mm_module_close(vipnn_ctx);
	mm_module_close(video_rgb_ctx);

	video_voe_release();
}

static void fUC(void *arg)
{
	static uint32_t user_cmd = 0;

	if (!strcmp(arg, "TD")) {
		if (user_cmd & USR_CMD_EXAMPLE_DEINIT) {
			printf("invalid state, can not do %s deinit!\r\n", example);
		} else {
			example_deinit();
			user_cmd = USR_CMD_EXAMPLE_DEINIT;
			printf("deinit %s\r\n", example);
		}
	} else if (!strcmp(arg, "TSR")) {
		if (user_cmd & USR_CMD_EXAMPLE_DEINIT) {
			printf("reinit %s\r\n", example);
			sys_reset();
		} else {
			printf("invalid state, can not do %s reinit!\r\n", example);
		}
	} else {
		printf("invalid cmd");
	}

	printf("user command 0x%lx\r\n", user_cmd);
}

static log_item_t userctrl_items[] = {
	{"UC", fUC, },
};

static void atcmd_userctrl_init(void)
{
	log_service_add_table(userctrl_items, sizeof(userctrl_items) / sizeof(userctrl_items[0]));
}
