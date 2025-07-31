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
#include "avcodec.h"
#include "log_service.h"

#include "model_scrfd.h"

#include "hal_video.h"
#include "hal_isp.h"

/*****************************************************************************
* ISP channel : 0,1
* Video type  : H264/HEVC
*****************************************************************************/

#define V1_CHANNEL 0
#define V2_CHANNEL 1

#define RTSP_RESOLUTION VIDEO_HD
#define RTSP_FPS 30
#define RTSP_GOP 30
#define RTSP_BPS 1*1024*1024
#define RTSP_RCMODE 2 // 1: CBR, 2: VBR

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

#if RTSP_RESOLUTION == VIDEO_VGA
#define RTSP_WIDTH  640
#define RTSP_HEIGHT 480
#elif RTSP_RESOLUTION == VIDEO_HD
#define RTSP_WIDTH  1280
#define RTSP_HEIGHT	720
#elif RTSP_RESOLUTION == VIDEO_FHD
#define RTSP_WIDTH  1920
#define RTSP_HEIGHT	1080
#endif

#define SYNC_FPS 5

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = RTSP_RESOLUTION,
	.width = RTSP_WIDTH,
	.height = RTSP_HEIGHT,
	.bps = RTSP_BPS,
	.fps = RTSP_FPS,
	.gop = RTSP_GOP,
	.rc_mode = RTSP_RCMODE,
	.use_static_addr = 1,
};

static video_params_t video_v2_params = {
	.stream_id = V2_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = RTSP_RESOLUTION,
	.width = RTSP_WIDTH,
	.height = RTSP_HEIGHT,
	.bps = RTSP_BPS,
	.gop = RTSP_GOP,
	.rc_mode = RTSP_RCMODE,
	.use_static_addr = 1,

	/* Enable sync mode. The video frame will not be encoded untill calling "osd update", which is included in canvas_update()
	 * In other words, OSD can draw the bounding box from NN to the video frame without latency
	 * However, the restriction is that the fps of this video channel should be same with NN channel */
	.out_mode = MODE_SYNC,
	.fps = SYNC_FPS,
};

static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.fps = RTSP_FPS,
			.bps = RTSP_BPS
		}
	}
};

static rtsp2_params_t rtsp2_v2_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.fps = RTSP_FPS,
			.bps = RTSP_BPS
		}
	}
};

/*****************************************************************************
* ISP channel : 4
* Video type  : RGB
*****************************************************************************/

#define NN_CHANNEL 4
#define NN_RESOLUTION VIDEO_VGA //don't care for NN
#define NN_FPS SYNC_FPS
#define NN_GOP NN_FPS //don't care for NN
#define NN_BPS 1024*1024 //don't care for NN
#define NN_TYPE VIDEO_RGB

// SCRFD
#define NN_MODEL_OBJ   scrfd_fwfs
#define NN_WIDTH    576
#define NN_HEIGHT   320

static video_params_t video_v4_params = {
	.stream_id = NN_CHANNEL,
	.type = NN_TYPE,
	.resolution = NN_RESOLUTION,
	.width = NN_WIDTH,
	.height = NN_HEIGHT,
	.bps = NN_BPS,
	.fps = NN_FPS,
	.gop = NN_GOP,
	.direct_output = 0,
	.use_static_addr = 1
};

static nn_data_param_t roi_nn = {
	.img = {
		.width = NN_WIDTH,
		.height = NN_HEIGHT,
		.roi = {
			.xmin = 0,
			.ymin = 0,
			.xmax = NN_WIDTH,
			.ymax = NN_HEIGHT,
		}
	},
	.codec_type = AV_CODEC_ID_RGB888
};

static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *video_v2_ctx			= NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;
static mm_context_t *rtsp2_v2_ctx			= NULL;
static mm_context_t *video_rgb_ctx			= NULL;
static mm_context_t *vipnn_ctx              = NULL;

static mm_siso_t *siso_video_rtsp_v1        = NULL;
static mm_siso_t *siso_video_rtsp_v2        = NULL;
static mm_siso_t *siso_video_vipnn          = NULL;

//--------------------------------------------
// Draw Rect
//--------------------------------------------
#include "osd_render.h"
#define LIMIT(x, lower, upper) if(x<lower) x=lower; else if(x>upper) x=upper;

static void nn_set_object(void *p, void *img_param)
{
	int i = 0;
	vipnn_out_buf_t *out = (vipnn_out_buf_t *)p;
	facedetect_res_t *face_res = (facedetect_res_t *)&out->res[0];
	nn_data_param_t *im = (nn_data_param_t *)img_param;

	if (!p || !img_param)	{
		return;
	}

	int im_h = RTSP_HEIGHT;
	int im_w = RTSP_WIDTH;

	//crop
	float ratio_w = (float)im_w / (float)im->img.width;
	float ratio_h = (float)im_h / (float)im->img.height;
	float ratio = ratio_h < ratio_w ? ratio_h : ratio_w;
	int roi_w = (int)((im->img.roi.xmax - im->img.roi.xmin) * ratio);
	int roi_h = (int)((im->img.roi.ymax - im->img.roi.ymin) * ratio);
	int roi_x = (int)(im->img.roi.xmin * ratio + (im_w - roi_w) / 2);
	int roi_y = (int)(im->img.roi.ymin * ratio + (im_h - roi_h) / 2);

	printf("object num = %d\r\n", out->res_cnt);
	canvas_create_bitmap(V1_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
	canvas_create_bitmap(V2_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
	if (out->res_cnt > 0) {
		for (i = 0; i < out->res_cnt; i++) {
			int obj_class = (int)face_res[i].result[0];
			int class_id = obj_class;
			if (class_id != -1) {
				int xmin = (int)(face_res[i].result[2] * roi_w) + roi_x;
				int ymin = (int)(face_res[i].result[3] * roi_h) + roi_y;
				int xmax = (int)(face_res[i].result[4] * roi_w) + roi_x;
				int ymax = (int)(face_res[i].result[5] * roi_h) + roi_y;
				LIMIT(xmin, 0, im_w)
				LIMIT(xmax, 0, im_w)
				LIMIT(ymin, 0, im_h)
				LIMIT(ymax, 0, im_h)
				printf("%d,c%d:%d %d %d %d\n\r", i, class_id, xmin, ymin, xmax, ymax);
				canvas_set_rect(V1_CHANNEL, 0, xmin, ymin, xmax, ymax, 3, COLOR_WHITE);
				canvas_set_rect(V2_CHANNEL, 0, xmin, ymin, xmax, ymax, 3, COLOR_WHITE);
				char text_str[20];
				snprintf(text_str, sizeof(text_str), "%s %d", "face", (int)(face_res[i].result[1] * 100));
				canvas_set_text(V1_CHANNEL, 0, xmin, ymin - 32, text_str, COLOR_CYAN);
				canvas_set_text(V2_CHANNEL, 0, xmin, ymin - 32, text_str, COLOR_CYAN);

				for (int j = 0; j < 5; j++) {
					int x = (int)(face_res[i].landmark.pos[j].x * roi_w) + roi_x;
					int y = (int)(face_res[i].landmark.pos[j].y * roi_h) + roi_y;
					canvas_set_point(V1_CHANNEL, 0, x, y, 8, COLOR_RED);
					canvas_set_point(V2_CHANNEL, 0, x, y, 8, COLOR_RED);
				}
			}
		}
	}
	canvas_update(V1_CHANNEL, 0, 1);
	canvas_update(V2_CHANNEL, 0, 1);
}

void mmf2_video_example_vipnn_facedet_sync_init(void)
{
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, RTSP_WIDTH, RTSP_HEIGHT, RTSP_BPS, 0,
						1, RTSP_WIDTH, RTSP_HEIGHT, RTSP_BPS, 0,
						0, 0, 0, 0, 0,
						1, NN_WIDTH, NN_HEIGHT);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 0, &video_v2_params, 0, NULL, 0, &video_v4_params);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, RTSP_FPS * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_vnn_facedetect_sync_fail;
	}

	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		printf("RTSP2 open fail\n\r");
		goto mmf2_example_vnn_facedetect_sync_fail;
	}

	video_v2_ctx = mm_module_open(&video_module);
	if (video_v2_ctx) {
		mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v2_params);
		mm_module_ctrl(video_v2_ctx, MM_CMD_SET_QUEUE_LEN, RTSP_FPS * 3);
		mm_module_ctrl(video_v2_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_example_vnn_facedetect_sync_fail;
	}

	rtsp2_v2_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v2_ctx) {
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v2_params);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_example_vnn_facedetect_sync_fail;
	}

	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_vnn_facedetect_sync_fail;
	}

	// VIPNN
	vipnn_ctx = mm_module_open(&vipnn_module);
	if (vipnn_ctx) {
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(facedetect_res_t));		// result size
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_DISPPOST, (int)nn_set_object);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_example_vnn_facedetect_sync_fail;
	}
	printf("VIPNN opened\n\r");

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
		goto mmf2_example_vnn_facedetect_sync_fail;
	}
	printf("siso_video_rtsp_v1 started\n\r");
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0

	siso_video_rtsp_v2 = siso_create();
	if (siso_video_rtsp_v2) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_rtsp_v2, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_rtsp_v2, MMIC_CMD_ADD_INPUT, (uint32_t)video_v2_ctx, 0);
		siso_ctrl(siso_video_rtsp_v2, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v2_ctx, 0);
		siso_start(siso_video_rtsp_v2);
	} else {
		printf("siso_video_rtsp_v2 open fail\n\r");
		goto mmf2_example_vnn_facedetect_sync_fail;
	}
	printf("siso_video_rtsp_v2 started\n\r");
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_APPLY, V2_CHANNEL);	// start channel 0

	siso_video_vipnn = siso_create();
	if (siso_video_vipnn) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_vipnn, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_ADD_OUTPUT, (uint32_t)vipnn_ctx, 0);
		siso_start(siso_video_vipnn);
	} else {
		printf("siso_video_vipnn open fail\n\r");
		goto mmf2_example_vnn_facedetect_sync_fail;
	}
	printf("siso_video_vipnn started\n\r");
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);	// start channel 4
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);

	int ch_enable[3] = {1, 1, 0};
	int char_resize_w[3] = {16, 16, 0}, char_resize_h[3] = {32, 32, 0};
	int ch_width[3] = {RTSP_WIDTH, RTSP_WIDTH, 0}, ch_height[3] = {RTSP_HEIGHT, RTSP_WIDTH, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);
	return;
mmf2_example_vnn_facedetect_sync_fail:

	return;
}
