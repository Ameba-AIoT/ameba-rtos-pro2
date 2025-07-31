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
#include "module_facerecog.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "avcodec.h"

#include "model_mobilefacenet.h"
#include "model_scrfd.h"
#include "model_landmark_sim.h"

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
#define NN_GOP NN_FPS
#define NN_BPS 1024*1024 //don't care for NN
#define NN_TYPE VIDEO_RGB

#define NN_MODEL_OBJ   	scrfd_fwfs
#define NN_MODEL2_OBJ   mbfacenet_fwfs
#define NN_MODEL3_OBJ	lmsim_fwfs
#define NN_WIDTH	576
#define NN_HEIGHT	320

define_model(scrfd320p)
define_model(mobilefacenet_i8)

#define USE_FACEDET_MODEL use_model(scrfd320p)
#define USE_FACENET_MODEL use_model(mobilefacenet_i8)

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
	.use_static_addr = 1
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
	.codec_type = AV_CODEC_ID_RGB888,
};

#define V1_ENA 1
#define V4_ENA 1

static mm_context_t *video_v1_ctx           = NULL;
static mm_context_t *rtsp2_v1_ctx           = NULL;
static mm_siso_t *siso_video_rtsp_v1        = NULL;

static mm_context_t *video_rgb_ctx          = NULL;
static mm_context_t *facedet_ctx            = NULL;
static mm_context_t *landmark_ctx           = NULL;
static mm_context_t *facenet_ctx            = NULL;
static mm_context_t *facerecog_ctx          = NULL;
static mm_siso_t *siso_rgb_facedet            = NULL;
static mm_siso_t *siso_facedet_landmark     = NULL;
static mm_siso_t *siso_landmark_facenet     = NULL;
static mm_siso_t *siso_facenet_facerecog    = NULL;

static void atcmd_frc_init(void *ctx);

//--------------------------------------------
// Draw Rect
//--------------------------------------------
#include "osd_render.h"

static TimerHandle_t osd_cleanup_timer = NULL;
static void osd_cleanup_callback(TimerHandle_t xTimer)
{
	(void)xTimer;
	canvas_create_bitmap(RTSP_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
	canvas_create_bitmap(RTSP_CHANNEL, 1, RTS_OSD2_BLK_FMT_1BPP);
	canvas_update(RTSP_CHANNEL, 0, 0);
	canvas_update(RTSP_CHANNEL, 1, 1);
}

#define LIMIT(x, lower, upper) if(x<lower) x=lower; else if(x>upper) x=upper;

static void face_draw_object(void *p, void *img_param)
{
	int i = 0;
	frc_draw_t *fdraw = (frc_draw_t *)p;

	if (!p)	{
		return;
	}

	int im_h = RTSP_HEIGHT;
	int im_w = RTSP_WIDTH;

	//printf("object num = %d\r\n", fdraw->obj_cnt);
	canvas_create_bitmap(RTSP_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
	canvas_create_bitmap(RTSP_CHANNEL, 1, RTS_OSD2_BLK_FMT_1BPP);
	if (fdraw->obj_cnt > 0) {
		for (i = 0; i < fdraw->obj_cnt; i++) {
			frc_bbox_t *bbox = &fdraw->bbox[i];
			int x_offset = 0, y_offset = 0;
			float ratio;
			if ((float)im_w / (float)im_h > (float)fdraw->pic_width / (float)fdraw->pic_height) {
				ratio = (float)im_h / (float)fdraw->pic_height;
				x_offset = (im_w - (float)fdraw->pic_width * ratio) / 2;
			} else {
				ratio = (float)im_w / (float)fdraw->pic_width;
				y_offset = (im_h - (float)fdraw->pic_height * ratio) / 2;
			}

			//printf("[fr] w %d h %d\n\r", fdraw->pic_width, fdraw->pic_height);
			//printf("%d,c %s:%f %f %f %f\n\r", i, fdraw->obj_name[i], bbox->xmin, bbox->ymin, bbox->xmax, bbox->ymax);

			int xmin = (int)(bbox->xmin * ratio * (float)fdraw->pic_width) + x_offset;
			int ymin = (int)(bbox->ymin * ratio * (float)fdraw->pic_height) + y_offset;
			int xmax = (int)(bbox->xmax * ratio * (float)fdraw->pic_width) + x_offset;
			int ymax = (int)(bbox->ymax * ratio * (float)fdraw->pic_height) + y_offset;
			LIMIT(xmin, 0, im_w)
			LIMIT(xmax, 0, im_w)
			LIMIT(ymin, 0, im_h)
			LIMIT(ymax, 0, im_h)

			//printf("%d,c%s:%d %d %d %d\n\r", i, fdraw->obj_name[i], xmin, ymin, xmax, ymax);

			if (!strcmp(fdraw->obj_name[i], "unknown")) {
				canvas_set_rect(RTSP_CHANNEL, 0, xmin, ymin, xmax, ymax, 3, COLOR_RED);
				canvas_set_text(RTSP_CHANNEL, 0, xmin, ymin - 32, fdraw->obj_name[i], COLOR_RED);
			} else {
				canvas_set_rect(RTSP_CHANNEL, 1, xmin, ymin, xmax, ymax, 3, COLOR_GREEN);
				canvas_set_text(RTSP_CHANNEL, 1, xmin, ymin - 32, fdraw->obj_name[i], COLOR_GREEN);
			}
		}
		if (osd_cleanup_timer) {
			xTimerReset(osd_cleanup_timer, 10);
		}
	}
	canvas_update(RTSP_CHANNEL, 0, 0);
	canvas_update(RTSP_CHANNEL, 1, 1);

}

void mmf2_video_example_fd_lm_mfn_sim_rtsp_init(void)
{
	USE_FACEDET_MODEL;
	USE_FACENET_MODEL;
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
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, RTSP_FPS);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_face_rtsp_fail;
	}

	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		printf("RTSP2 open fail\n\r");
		goto mmf2_example_face_rtsp_fail;
	}
#endif

#if V4_ENA
	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_face_rtsp_fail;
	}

	// VIPNN
	facedet_ctx = mm_module_open(&vipnn_module);
	if (facedet_ctx) {
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_OUTPUT, 1);		// output
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(facedetect_res_t));		// result size
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count

		mm_module_ctrl(facedet_ctx, MM_CMD_SET_DATAGROUP, MM_GROUP_START);

		mm_module_ctrl(facedet_ctx, MM_CMD_SET_QUEUE_LEN, 1);
		mm_module_ctrl(facedet_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_example_face_rtsp_fail;
	}
	printf("VIPNN opened\n\r");


	// VIPNN
	landmark_ctx = mm_module_open(&vipnn_module);
	if (landmark_ctx) {
		mm_module_ctrl(landmark_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL3_OBJ);
		mm_module_ctrl(landmark_ctx, CMD_VIPNN_SET_CASCADE, 2);		// this module is cascade mode
		//mm_module_ctrl(landmark_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(landmark_ctx, CMD_VIPNN_SET_OUTPUT, 1);		// output
		mm_module_ctrl(landmark_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(facedetect_res_t));		// result size
		mm_module_ctrl(landmark_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count

		mm_module_ctrl(landmark_ctx, MM_CMD_SET_QUEUE_LEN, 1);
		mm_module_ctrl(landmark_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(landmark_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN3 open fail\n\r");
		goto mmf2_example_face_rtsp_fail;
	}
	printf("VIPNN3 opened\n\r");

	// VIPNN
	facenet_ctx = mm_module_open(&vipnn_module);
	if (facenet_ctx) {
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL2_OBJ);
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_CASCADE, 2);		// this module is cascade mode
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_OUTPUT, 1);		// output
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(face_feature_res_t));		// result size
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count

		mm_module_ctrl(facenet_ctx, MM_CMD_SET_DATAGROUP, MM_GROUP_END);
		mm_module_ctrl(facenet_ctx, MM_CMD_SET_QUEUE_LEN, 1);
		mm_module_ctrl(facenet_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN2 open fail\n\r");
		goto mmf2_example_face_rtsp_fail;
	}
	printf("VIPNN2 opened\n\r");

	// FACERECOG
	facerecog_ctx = mm_module_open(&facerecog_module);
	if (facerecog_ctx) {
		mm_module_ctrl(facerecog_ctx, CMD_FRC_SET_THRES100, 99);  // 99/100 = 0.99 --> set a value to get lowest FP rate
		mm_module_ctrl(facerecog_ctx, CMD_FRC_SET_OSD_DRAW, (int)face_draw_object);
	} else {
		printf("FACERECOG open fail\n\r");
		goto mmf2_example_face_rtsp_fail;
	}
	printf("FACERECOG opened\n\r");

	atcmd_frc_init(facerecog_ctx);
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
		goto mmf2_example_face_rtsp_fail;
	}

	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, RTSP_CHANNEL);	// start channel 0
#endif

#if V4_ENA
	siso_facenet_facerecog = siso_create();
	if (siso_facenet_facerecog) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_facenet_facerecog, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_facenet_facerecog, MMIC_CMD_ADD_INPUT, (uint32_t)facenet_ctx, 0);
		siso_ctrl(siso_facenet_facerecog, MMIC_CMD_ADD_OUTPUT, (uint32_t)facerecog_ctx, 0);
		siso_start(siso_facenet_facerecog);
	} else {
		printf("siso_facenet_facerecog open fail\n\r");
		goto mmf2_example_face_rtsp_fail;
	}
	printf("siso_facenet_facerecog started\n\r");

	siso_landmark_facenet = siso_create();
	if (siso_landmark_facenet) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_landmark_facenet, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_landmark_facenet, MMIC_CMD_ADD_INPUT, (uint32_t)landmark_ctx, 0);
		siso_ctrl(siso_landmark_facenet, MMIC_CMD_ADD_OUTPUT, (uint32_t)facenet_ctx, 0);
		siso_start(siso_landmark_facenet);
	} else {
		printf("siso_landmark_facenet open fail\n\r");
		goto mmf2_example_face_rtsp_fail;
	}
	printf("siso_landmark_facenet started\n\r");


	siso_facedet_landmark = siso_create();
	if (siso_facedet_landmark) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_facedet_landmark, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_facedet_landmark, MMIC_CMD_ADD_INPUT, (uint32_t)facedet_ctx, 0);
		siso_ctrl(siso_facedet_landmark, MMIC_CMD_ADD_OUTPUT, (uint32_t)landmark_ctx, 0);
		siso_start(siso_facedet_landmark);
	} else {
		printf("siso_facedet_landmark open fail\n\r");
		goto mmf2_example_face_rtsp_fail;
	}
	printf("siso_facedet_landmark started\n\r");

	siso_rgb_facedet = siso_create();
	if (siso_rgb_facedet) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_rgb_facedet, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_rgb_facedet, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		siso_ctrl(siso_rgb_facedet, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_rgb_facedet, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_ctrl(siso_rgb_facedet, MMIC_CMD_ADD_OUTPUT, (uint32_t)facedet_ctx, 0);
		siso_start(siso_rgb_facedet);
	} else {
		printf("siso_rgb_facedet open fail\n\r");
		goto mmf2_example_face_rtsp_fail;
	}
	printf("siso_rgb_facedet started\n\r");

	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);	// start channel 4
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);
#endif

#if V1_ENA && V4_ENA
	int ch_enable[3] = {1, 0, 0};
	int char_resize_w[3] = {16, 0, 0}, char_resize_h[3] = {32, 0, 0};
	int ch_width[3] = {RTSP_WIDTH, 0, 0}, ch_height[3] = {RTSP_HEIGHT, 0, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);
	osd_cleanup_timer = xTimerCreate("OSD clean timer", 500 / portTICK_PERIOD_MS, pdTRUE, NULL, osd_cleanup_callback);
#endif

	return;
mmf2_example_face_rtsp_fail:

	return;
}


//-----------------------------------------------------------------------------------------------

#include "log_service.h"
static void *g_frc_ctx = NULL;
static void fFREG(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};
	argc = parse_param(arg, argv);

	if (!g_frc_ctx)	{
		return;
	}
	printf("enter register mode\n\r");
	mm_module_ctrl(g_frc_ctx, CMD_FRC_REGISTER_MODE, (int)argv[1]);
}

static void fFRRM(void *arg)
{
	if (!g_frc_ctx)	{
		return;
	}
	printf("enter recognition mode\n\r");
	mm_module_ctrl(g_frc_ctx, CMD_FRC_RECOGNITION_MODE, 0);
}


static void fFRFL(void *arg)
{
	if (!g_frc_ctx)	{
		return;
	}
	printf("load feature\n\r");
	mm_module_ctrl(g_frc_ctx, CMD_FRC_LOAD_FEATURES, 0);
}


static void fFRFS(void *arg)
{
	if (!g_frc_ctx)	{
		return;
	}
	printf("save feature\n\r");
	mm_module_ctrl(g_frc_ctx, CMD_FRC_SAVE_FEATURES, 0);
}


static void fFRFR(void *arg)
{
	if (!g_frc_ctx)	{
		return;
	}
	printf("reset features\n\r");
	mm_module_ctrl(g_frc_ctx, CMD_FRC_RESET_FEATURES, 0);
}

static void fFRSC(void *arg)
{
	if (!g_frc_ctx)	{
		return;
	}
	int argc = 0;
	char *argv[MAX_ARGC] = {0};
	argc = parse_param(arg, argv);
	(void)argc;

	int score = strtol(argv[1], NULL, 0);
	printf("set face recognition score threshold to %.2f\n\r", (float)score / 100.0);
	mm_module_ctrl(facerecog_ctx, CMD_FRC_SET_THRES100, score);
}


static log_item_t nn_frc_items[] = {
	{"FREG", fFREG,},
	{"FRRM", fFRRM,},
	{"FRFL", fFRFL,},
	{"FRFS", fFRFS,},
	{"FRFR", fFRFR,},
	{"FRSC", fFRSC,}
};

static void atcmd_frc_init(void *ctx)
{
	g_frc_ctx = ctx;
	log_service_add_table(nn_frc_items, sizeof(nn_frc_items) / sizeof(nn_frc_items[0]));
}