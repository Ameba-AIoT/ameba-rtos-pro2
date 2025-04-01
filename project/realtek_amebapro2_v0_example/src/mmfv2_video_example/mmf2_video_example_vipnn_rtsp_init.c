/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "module_vipnn.h"
#include "module_rtsp2.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "log_service.h"
#include "avcodec.h"

#include "img_sample/input_image_640x360x3.h"
#include "img_sample/input_image_416x416x3.c"
#include "nn_utils/class_name.h"
#include "model_yolo.h"
#include "model_nanodet.h"
#include "model_yolov9.h"

#include "hal_video.h"
#include "hal_isp.h"

/*****************************************************************************
* ISP channel : 4
* Video type  : RGB
*****************************************************************************/
#define RTSP_CHANNEL 0
#define RTSP_RESOLUTION VIDEO_FHD
#define RTSP_FPS 30
#define RTSP_GOP 30
#define RTSP_BPS 1*1024*1024
#define VIDEO_RCMODE 2 // 1: CBR, 2: VBR

#define USE_H265 0

#if USE_H265
#include "sample_h265.h"
#define RTSP_TYPE VIDEO_HEVC
#define RTSP_CODEC AV_CODEC_ID_H265
#else
#include "sample_h264.h"
#define RTSP_TYPE VIDEO_H264
#define RTSP_CODEC AV_CODEC_ID_H264
#endif

#if RTSP_RESOLUTION == VIDEO_VGA
#define RTSP_WIDTH	640
#define RTSP_HEIGHT	480
#elif RTSP_RESOLUTION == VIDEO_HD
#define RTSP_WIDTH	1280
#define RTSP_HEIGHT	720
#elif RTSP_RESOLUTION == VIDEO_FHD
#define RTSP_WIDTH	1920
#define RTSP_HEIGHT	1080
#endif

static video_params_t video_v1_params = {
	.stream_id 		= RTSP_CHANNEL,
	.type 			= RTSP_TYPE,
	.resolution 	= RTSP_RESOLUTION,
	.width 			= RTSP_WIDTH,
	.height 		= RTSP_HEIGHT,
	.bps            = RTSP_BPS,
	.fps 			= RTSP_FPS,
	.gop 			= RTSP_GOP,
	.rc_mode        = VIDEO_RCMODE,
	.use_static_addr = 1,
	//.fcs = 1
};

static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = RTSP_CODEC,
			.fps      = RTSP_FPS,
			.bps      = RTSP_BPS
		}
	}
};

// NN model config //
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
#define NN_WIDTH	576  //416
#define NN_HEIGHT	320  //416

define_model(yolov4_tiny_320p)
#define USE_OBJDET_MODEL use_model(yolov4_tiny_320p)

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
	.use_roi = 1,
	.roi = {
		.xmin = 0,
		.ymin = 0,
		.xmax = sensor_params[USE_SENSOR].sensor_width,
		.ymax = sensor_params[USE_SENSOR].sensor_height,
	}
};

#include "module_array.h"
static array_params_t h264_array_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.codec_id = AV_CODEC_ID_RGB888,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.v = {
			.fps    = 5,
		}
	}
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

#define V1_ENA 1
#define V4_ENA 1
#define V4_SIM 0

static void atcmd_userctrl_init(void);
static mm_context_t *array_ctx            = NULL;
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v1			= NULL;

static mm_context_t *video_rgb_ctx			= NULL;
static mm_context_t *vipnn_ctx            = NULL;
static mm_siso_t *siso_array_vipnn         = NULL;


//--------------------------------------------
// Draw Rect
//--------------------------------------------
#include "osd_render.h"
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

	int im_h = RTSP_HEIGHT;
	int im_w = RTSP_WIDTH;

	float ratio_w = (float)im_w / (float)im->img.width;
	float ratio_h = (float)im_h / (float)im->img.height;
	int roi_h, roi_w, roi_x, roi_y;
	if (video_v4_params.use_roi == 1) { //resize
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

	printf("object num = %d\r\n", obj_num);
	canvas_create_bitmap(RTSP_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
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
				printf("%d,c%d:%d %d %d %d\n\r", i, class_id, xmin, ymin, xmax, ymax);
				canvas_set_rect(RTSP_CHANNEL, 0, xmin, ymin, xmax, ymax, 3, COLOR_WHITE);
				char text_str[20];
				snprintf(text_str, sizeof(text_str), "%s %d", coco_name_get_by_id(class_id), (int)(res[i].result[1] * 100));
				canvas_set_text(RTSP_CHANNEL, 0, xmin, ymin - 32, text_str, COLOR_CYAN);
			}
		}
	}
	canvas_update(RTSP_CHANNEL, 0, 1);

}

/* User can load model binary from any filesystem */
#define USER_LOAD_MODEL     0
#if USER_LOAD_MODEL
#include "vfs.h"
static void *example_get_model_name(void)
{
	/* set filename of customized model */
	return (void *)"sd:/NN_MDL/yolov4_tiny.nb";
}

extern void yolov4_set_network_init_info(void *m);
extern int yolo_preprocess(void *data_in, nn_data_param_t *data_param, void *tensor_in, nn_tensor_param_t *tensor_param);
extern int yolo_postprocess(void *tensor_out, nn_tensor_param_t *param, void *res);
extern void yolo_set_confidence_thresh(void *confidence_thresh);
extern void yolo_set_nms_thresh(void *nms_thresh);
/* create user model object */
nnmodel_t yolov4_tiny_from_sd = {
	.nb 			= example_get_model_name,
	.set_init_info  = yolov4_set_network_init_info,
	.preprocess 	= yolo_preprocess,
	.postprocess 	= yolo_postprocess,
	.model_src 		= MODEL_SRC_FILE,
	.set_confidence_thresh   = yolo_set_confidence_thresh,
	.set_nms_thresh     = yolo_set_nms_thresh,

	.name = "YOLOv4t_SD"
};
#endif

void mmf2_video_example_vipnn_rtsp_init(void)
{
	atcmd_userctrl_init();
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(V1_ENA, RTSP_WIDTH, RTSP_HEIGHT, RTSP_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						V4_ENA, NN_WIDTH, NN_HEIGHT);
#else
	int voe_heap_size = video_voe_presetting_by_params((V1_ENA ? &video_v1_params : NULL), 0, NULL, 0, NULL, 0, (V4_ENA ? &video_v4_params : NULL));
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

#if V1_ENA
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, RTSP_FPS * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_vnn_rtsp_fail;
	}

	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		printf("RTSP2 open fail\n\r");
		goto mmf2_example_vnn_rtsp_fail;
	}
#endif

#if V4_ENA
#if V4_SIM==0
	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_vnn_rtsp_fail;
	}
#else
	array_t array;
	array.data_addr = (uint32_t) testRGB_640x360;
	array.data_len = (uint32_t) 640 * 360 * 3;
	array_ctx = mm_module_open(&array_module);
	if (array_ctx) {
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_PARAMS, (int)&h264_array_params);
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_ARRAY, (int)&array);
		mm_module_ctrl(array_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(array_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(array_ctx, CMD_ARRAY_APPLY, 0);
		mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 1);	// streamming on
	} else {
		printf("ARRAY open fail\n\r");
		goto mmf2_example_vnn_rtsp_fail;
	}
#endif
	// VIPNN
	vipnn_ctx = mm_module_open(&vipnn_module);
	if (vipnn_ctx) {
#if USER_LOAD_MODEL
		vfs_init(NULL);  // init filesystem
		vfs_user_register("sd", VFS_FATFS, VFS_INF_SD);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_MODEL, (int)&yolov4_tiny_from_sd);
#else
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
#endif
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
		goto mmf2_example_vnn_rtsp_fail;
	}
	printf("VIPNN opened\n\r");
#endif

	//--------------Link---------------------------

#if V1_ENA
	siso_video_rtsp_v1 = siso_create();
	if (siso_video_rtsp_v1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v1_ctx, 0);
		siso_start(siso_video_rtsp_v1);
	} else {
		printf("siso2 open fail\n\r");
		goto mmf2_example_vnn_rtsp_fail;
	}

	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, RTSP_CHANNEL);

#endif
#if 1 //V4_ENA
	siso_array_vipnn = siso_create();
	if (siso_array_vipnn) {
#if V4_SIM==0
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_array_vipnn, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_array_vipnn, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		siso_ctrl(siso_array_vipnn, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_array_vipnn, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
#else
		siso_ctrl(siso_array_vipnn, MMIC_CMD_ADD_INPUT, (uint32_t)array_ctx, 0);
#endif
		siso_ctrl(siso_array_vipnn, MMIC_CMD_ADD_OUTPUT, (uint32_t)vipnn_ctx, 0);
		siso_start(siso_array_vipnn);
	} else {
		printf("siso_array_vipnn open fail\n\r");
		goto mmf2_example_vnn_rtsp_fail;
	}
#if V4_ENA
#if V4_SIM==0
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);
#endif
#endif

	printf("siso_array_vipnn started\n\r");
#endif

#if V1_ENA && V4_ENA
	int ch_enable[3] = {1, 0, 0};
	int char_resize_w[3] = {16, 0, 0}, char_resize_h[3] = {32, 0, 0};
	int ch_width[3] = {RTSP_WIDTH, 0, 0}, ch_height[3] = {RTSP_HEIGHT, 0, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);
#endif

	return;
mmf2_example_vnn_rtsp_fail:

	return;
}

static const char *example = "mmf2_video_example_vipnn_rtsp";
static void example_deinit(void)
{
#if V1_ENA && V4_ENA
	osd_render_task_stop();
	osd_render_dev_deinit_all();
#endif
	//Pause Linker
	siso_pause(siso_video_rtsp_v1);
	siso_pause(siso_array_vipnn);

	//Stop module
	mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, RTSP_CHANNEL);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_STREAM_STOP, NN_CHANNEL);

	//Delete linker
	siso_delete(siso_video_rtsp_v1);
	siso_delete(siso_array_vipnn);

	//Close module
	mm_module_close(rtsp2_v1_ctx);
	mm_module_close(video_v1_ctx);
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
