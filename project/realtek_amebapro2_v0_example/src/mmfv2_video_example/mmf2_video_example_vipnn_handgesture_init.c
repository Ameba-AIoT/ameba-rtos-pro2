/******************************************************************************
*
* Copyright(c) 2007 - 2024 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "module_vipnn.h"
#include "module_rtsp2.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "avcodec.h"
#include "log_service.h"
#include "model_palm_detection.h"
#include "model_hand_landmark.h"
#include "hal_video.h"
#include "hal_isp.h"
#include "roi_delta_qp/roi_delta_qp.h"
#include <math.h>

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
#define NN_FPS 10
#define NN_GOP NN_FPS
#define NN_TYPE VIDEO_RGB
// palm detection model
#define NN_MODEL_OBJ   palm_detection_fwfs
#define NN_MODEL2_OBJ hand_landmark_fwfs
#define NN_WIDTH    192
#define NN_HEIGHT   192
define_model(palm_detection_lite_int16)
define_model(hand_landmark_lite_int16)
#define USE_PALMDET_MODEL use_model(palm_detection_lite_int16)
#define USE_HANDLAND_MODEL use_model(hand_landmark_lite_int16)
static video_params_t video_v4_params = {
	.stream_id 		= NN_CHANNEL,
	.type 			= NN_TYPE,
	.width 			= NN_WIDTH,
	.height 		= NN_HEIGHT,
	.fps 			= NN_FPS,
	.gop 			= NN_GOP,
	.direct_output 	= 0,
	.use_static_addr = 1
};
static nn_data_param_t palm_nn_roi = {
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
static nn_data_param_t handlandmark_nn_roi = {
	.img = {
		.width = 224,
		.height = 224,
		.rgb = 0, // set to 1 if want RGB->BGR or BGR->RGB
		.roi = {
			.xmin = 0,
			.ymin = 0,
			.xmax = 224,
			.ymax = 224,
		}
	},
	.codec_type = AV_CODEC_ID_NN_RAW
};

static float nn_confidence_thresh = 0.5;
static float nn_nms_thresh = 0.3;

static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;
static mm_context_t *video_rgb_ctx			= NULL;
static mm_context_t *palmdet_ctx              = NULL;
static mm_context_t *handland_ctx         = NULL;
static mm_siso_t *siso_video_rtsp_v1        = NULL;
static mm_siso_t *siso_video_vipnn          = NULL;
static mm_siso_t *siso_palm_hand          = NULL;
//--------------------------------------------
// Draw Rect
//--------------------------------------------
#include "osd_render.h"
static TimerHandle_t osd_cleanup_timer = NULL;

#define HAND_LINK_LAYER 0
#define HAND_JOINT_LAYER 1
#define HAND_DETECT_REGION_LAYER 2

#define LIMIT(x, lower, upper) if(x<lower) x=lower; else if(x>upper) x=upper;
static void hand_cleanup_callback(TimerHandle_t xTimer)
{
	(void)xTimer;
	canvas_create_bitmap(RTSP_CHANNEL, HAND_JOINT_LAYER, RTS_OSD2_BLK_FMT_1BPP);
	canvas_create_bitmap(RTSP_CHANNEL, HAND_LINK_LAYER, RTS_OSD2_BLK_FMT_1BPP);
	canvas_update(RTSP_CHANNEL, HAND_JOINT_LAYER, 0);
	canvas_update(RTSP_CHANNEL, HAND_LINK_LAYER, 1);
}

/*
Correcting coordinates
After performing a series of rotations, scaling, and translations, the coordinates need to be corrected back.
For example, if you previously rotated the hand to point the middle finger upwards, you need to rotate it back here; otherwise, the drawn hand will always have the middle finger pointing up.
*/
int fix_landmark(handland_res_t *lm)
{
	float cx = (float) handlandmark_nn_roi.img.width / 2.0;
	float cy = (float) handlandmark_nn_roi.img.height / 2.0;
	float cos_theta = cosf(lm->theta);
	float sin_theta = sinf(lm->theta);
	float scale = lm->w / (float)(handlandmark_nn_roi.img.width / 2.0) / lm->ratio;
	for (int i = 0; i < HAND_LANDMARK_NUM; i++) {
		//printf("before:%d x = %f, y = %f\r\n", i, lm->landmark3d.pos[i].x, lm->landmark3d.pos[i].y);
		//Place your hand at the edge of the screen. Sometimes, the landmark will go out of the frame. Let's correct it here first.
		LIMIT(lm->landmark3d.pos[i].x, 0, handlandmark_nn_roi.img.width);
		LIMIT(lm->landmark3d.pos[i].y, 0, handlandmark_nn_roi.img.height);
		float dx = lm->landmark3d.pos[i].x - cx;
		float dy = lm->landmark3d.pos[i].y - cy;
		float x = dx * cos_theta - dy * sin_theta + cx;
		float y = dx * sin_theta + dy * cos_theta + cy;
		if (x < 0 || y < 0) {
			//Place your hand at the edge of the screen. After calibration, it might extend far beyond the screen, so let's not draw this for now.
			printf("error! x = %f, y = %f\r\n", x, y);
			return -1;
		}
		x = x * scale;
		y = y * scale;
		lm->landmark3d.pos[i].x = x + lm->offset_x;
		lm->landmark3d.pos[i].y = y + lm->offset_y;
		//printf("after:%d x = %f, y = %f\r\n", i, lm->landmark3d.pos[i].x, lm->landmark3d.pos[i].y);

		//Here are some additional adjustments to make the skeleton more accurate when the hand is in the upper corner of the screen.
		if (lm->offset_x < 20) {
			lm->landmark3d.pos[i].x = lm->landmark3d.pos[i].x - (20 - lm->offset_x);
			LIMIT(lm->landmark3d.pos[i].x, 0, handlandmark_nn_roi.img.width);
		}
		if (lm->offset_y < 20) {
			lm->landmark3d.pos[i].y = lm->landmark3d.pos[i].y - (20 - lm->offset_y);
			LIMIT(lm->landmark3d.pos[i].y, 0, handlandmark_nn_roi.img.height);
		}
	}
	//printf("lm->offset_x = %d, lm->offset_y = %d\r\n", lm->offset_x, lm->offset_y);
	return 0;
}

static void draw_hand_object(void *p, void *img_param)
{
	if (!p || !img_param) {
		return;
	}

	vipnn_out_buf_t *out = (vipnn_out_buf_t *)p;
	handland_res_t *res = (handland_res_t *)&out->res[0];
	nn_data_param_t *im = (nn_data_param_t *)img_param;

	int im_h = RTSP_HEIGHT;
	int im_w = RTSP_WIDTH;

	//printf("im->width = %d, im->height = %d\r\n", im->img.width, im->img.height);
	im->img.width = im->img.height = handlandmark_nn_roi.img.width;

	float ratio_w = (float)im_w / (float)im->img.width;
	float ratio_h = (float)im_h / (float)im->img.height;
	float ratio = ratio_h < ratio_w ? ratio_h : ratio_w;

	roi_delta_qp_set_param(RTSP_CHANNEL, 0, 0, RTSP_WIDTH, RTSP_HEIGHT, ROI_DELTA_QP_MAX);
	canvas_create_bitmap(RTSP_CHANNEL, HAND_JOINT_LAYER, RTS_OSD2_BLK_FMT_1BPP);
	canvas_create_bitmap(RTSP_CHANNEL, HAND_LINK_LAYER, RTS_OSD2_BLK_FMT_1BPP);

	if (res->handedness != HANDEDNESS_NOTFOUND) {
		//printf("theta:%f w:%d h:%d\r\n", res->theta, res->w, res->h);
		if (fix_landmark(res) < 0) {
			return;
		}

		//First, draw the points for the joints.
		for (int i = 0; i < HAND_LANDMARK_NUM; i++) {
			//printf("res->landmark3d.pos[i].x = %f, res->landmark3d.pos[i].y = %f, res->landmark3d.pos[i].z = %f\r\n", res->landmark3d.pos[i].x, res->landmark3d.pos[i].y, res->landmark3d.pos[i].z);
			int xr = res->landmark3d.pos[i].x * ratio + (RTSP_WIDTH - RTSP_HEIGHT) / 2;
			int yr = res->landmark3d.pos[i].y * ratio;
			canvas_set_point(RTSP_CHANNEL, HAND_JOINT_LAYER, xr, yr, 12, COLOR_RED);
			//res->landmark3d.pos[i].z = roundf(q2f(&llm[(i*3+2)*datasize], llm_fmt.format));//Not needed to draw OSD
			//printf("llm %d(%f,%f,%f)\r\n", i, hand_landmark_res->landmark3d.pos[i].x, hand_landmark_res->landmark3d.pos[i].y, hand_landmark_res->landmark3d.pos[i].z);
		}

		/*
		#        8   12  16  20
		#        |   |   |   |
		#        7   11  15  19
		#    4   |   |   |   |
		#    |   6   10  14  18
		#    3   |   |   |   |
		#    |   5---9---13--17
		#    2    \         /
		#     \    \       /
		#      1    \     /
		#       \    \   /
		#        ------0-
		connections = [
			(0, 1), (1, 2), (2, 3), (3, 4),
			(5, 6), (6, 7), (7, 8),
			(9, 10), (10, 11), (11, 12),
			(13, 14), (14, 15), (15, 16),
			(17, 18), (18, 19), (19, 20),
			(0, 5), (5, 9), (9, 13), (13, 17), (0, 17), (2,5)
		*/
		int connections[22][2] = {
			{0, 1}, {1, 2}, {2, 3}, {3, 4},
			{5, 6}, {6, 7}, {7, 8},
			{9, 10}, {10, 11}, {11, 12},
			{13, 14}, {14, 15}, {15, 16},
			{17, 18}, {18, 19}, {19, 20},
			{0, 5}, {5, 9}, {9, 13}, {13, 17}, {0, 17}, {2, 5}
		};

		//Draw the connections of the joints
		for (int i = 0; i < 22; i++) {
			int idx_start = connections[i][0];
			int idx_end = connections[i][1];
			int start_x = res->landmark3d.pos[idx_start].x * ratio + (RTSP_WIDTH - RTSP_HEIGHT) / 2;
			int start_y = res->landmark3d.pos[idx_start].y * ratio;
			int end_x = res->landmark3d.pos[idx_end].x * ratio + (RTSP_WIDTH - RTSP_HEIGHT) / 2;
			int end_y = res->landmark3d.pos[idx_end].y * ratio;
			//printf("start_x = %d, start_y = %d, end_x = %d, end_y = %d\r\n", start_x, start_y, end_x, end_y);
			canvas_set_line(RTSP_CHANNEL, HAND_LINK_LAYER, start_x, start_y, end_x, end_y, 8, COLOR_GREEN);
		}
		if (osd_cleanup_timer) {
			xTimerReset(osd_cleanup_timer, 10);
		}
	}

	canvas_update(RTSP_CHANNEL, HAND_JOINT_LAYER, 0);
	canvas_update(RTSP_CHANNEL, HAND_LINK_LAYER, 1);
}

void mmf2_video_example_vipnn_handgesture_init(void)
{
	USE_PALMDET_MODEL;
	int voe_heap_size = video_voe_presetting(1, RTSP_WIDTH, RTSP_HEIGHT, RTSP_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						1, NN_WIDTH, NN_HEIGHT);
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, RTSP_FPS);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_example_vipnn_handgesture_fail;
	}
	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		printf("RTSP2 open fail\n\r");
		goto mmf2_video_example_vipnn_handgesture_fail;
	}
	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_example_vipnn_handgesture_fail;
	}
	// VIPNN
	palmdet_ctx = mm_module_open(&vipnn_module);
	if (palmdet_ctx) {
		mm_module_ctrl(palmdet_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(palmdet_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&palm_nn_roi);
		mm_module_ctrl(palmdet_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(palmdetect_res_t));		// result size
		mm_module_ctrl(palmdet_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_HAND_DETECT_NUM);		// result max count
		mm_module_ctrl(palmdet_ctx, MM_CMD_SET_DATAGROUP, MM_GROUP_START);
		mm_module_ctrl(palmdet_ctx, CMD_VIPNN_SET_CONFIDENCE_THRES, (int)&nn_confidence_thresh);
		mm_module_ctrl(palmdet_ctx, CMD_VIPNN_SET_NMS_THRES, (int)&nn_nms_thresh);
		mm_module_ctrl(palmdet_ctx, CMD_VIPNN_SET_OUTPUT, 1);  //enable module output
		mm_module_ctrl(palmdet_ctx, MM_CMD_SET_QUEUE_LEN, 1);
		mm_module_ctrl(palmdet_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(palmdet_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_video_example_vipnn_handgesture_fail;
	}
	printf("VIPNN opened\n\r");

	handland_ctx = mm_module_open(&vipnn_module);
	if (handland_ctx) {
		mm_module_ctrl(handland_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL2_OBJ);
		mm_module_ctrl(handland_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&handlandmark_nn_roi);
		mm_module_ctrl(handland_ctx, CMD_VIPNN_SET_CASCADE, 2);		// this module is cascade mode
		mm_module_ctrl(handland_ctx, CMD_VIPNN_SET_OUTPUT, 1);		// output
		mm_module_ctrl(handland_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(handland_res_t));		// result size
		mm_module_ctrl(handland_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_HAND_DETECT_NUM);		// result max count

		mm_module_ctrl(handland_ctx, MM_CMD_SET_DATAGROUP, MM_GROUP_END);
		mm_module_ctrl(handland_ctx, MM_CMD_SET_QUEUE_LEN, 1);
		mm_module_ctrl(handland_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(handland_ctx, CMD_VIPNN_SET_DISPPOST, (int)draw_hand_object);
		mm_module_ctrl(handland_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN2 open fail\n\r");
		goto mmf2_video_example_vipnn_handgesture_fail;
	}
	printf("VIPNN2 opened\n\r");

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
		goto mmf2_video_example_vipnn_handgesture_fail;
	}
	printf("siso_video_rtsp_v1 started\n\r");
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, RTSP_CHANNEL);	// start channel 0
	siso_video_vipnn = siso_create();
	if (siso_video_vipnn) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_vipnn, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_ADD_OUTPUT, (uint32_t)palmdet_ctx, 0);
		siso_start(siso_video_vipnn);
	} else {
		printf("siso_video_vipnn open fail\n\r");
		goto mmf2_video_example_vipnn_handgesture_fail;
	}
	printf("siso_video_vipnn started\n\r");


	siso_palm_hand = siso_create();
	if (siso_palm_hand) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_palm_hand, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_palm_hand, MMIC_CMD_ADD_INPUT, (uint32_t)palmdet_ctx, 0);
		siso_ctrl(siso_palm_hand, MMIC_CMD_ADD_OUTPUT, (uint32_t)handland_ctx, 0);
		siso_start(siso_palm_hand);
	} else {
		printf("siso_palm_hand open fail\n\r");
		goto mmf2_video_example_vipnn_handgesture_fail;
	}

	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);	// start channel 4
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);
	int ch_enable[3] = {1, 0, 0};
	int char_resize_w[3] = {16, 0, 0}, char_resize_h[3] = {32, 0, 0};
	int ch_width[3] = {RTSP_WIDTH, 0, 0}, ch_height[3] = {RTSP_HEIGHT, 0, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);

	//draw hand detect region
	canvas_create_bitmap(RTSP_CHANNEL, HAND_DETECT_REGION_LAYER, RTS_OSD2_BLK_FMT_1BPP);
	canvas_set_rect(RTSP_CHANNEL, HAND_DETECT_REGION_LAYER, (RTSP_WIDTH - RTSP_HEIGHT) / 2, 5, RTSP_WIDTH - (RTSP_WIDTH - RTSP_HEIGHT) / 2, RTSP_HEIGHT - 5, 3,
					COLOR_BLUE);
	canvas_update(RTSP_CHANNEL, HAND_DETECT_REGION_LAYER, 1);

	osd_cleanup_timer = xTimerCreate("OSD clean timer", 1000 / portTICK_PERIOD_MS, pdTRUE, NULL, hand_cleanup_callback);
	return;

mmf2_video_example_vipnn_handgesture_fail:
	return;
}

