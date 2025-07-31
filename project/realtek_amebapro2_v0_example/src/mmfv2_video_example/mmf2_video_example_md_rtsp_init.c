/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"

#include "module_eip.h"
#include "module_rtsp2.h"
#include "log_service.h"

#undef printf // undefine hal_vidoe.h printf 
#include <stdio.h>

/*****************************************************************************
* ISP channel : 4
* Video type  : RGB
*****************************************************************************/
#define RTSP_CHANNEL 0
#define RTSP_RESOLUTION VIDEO_FHD
#define RTSP_FPS 30
#define RTSP_GOP 30
#define RTSP_BPS 2*1024*1024
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
	.use_static_addr = 1
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

#define MD_INPUT USE_RGB
#define USE_RGB 0
#define USE_NV12 1
#if MD_INPUT == USE_RGB
#define MD_CHANNEL 4//only ch4 support rgb type
#define MD_TYPE VIDEO_RGB
#else
#define MD_CHANNEL 2//ch0~2 support nv12 type
#define MD_TYPE VIDEO_NV12
#endif
#define MD_RESOLUTION VIDEO_VGA //VIDEO_WVGA
#define MD_GOP MD_FPS
#define MD_BPS 1024*1024
#define MD_COL 32
#define MD_ROW 32


#if MD_RESOLUTION == VIDEO_VGA
#define MD_WIDTH	640
#define MD_HEIGHT	480
#elif MD_RESOLUTION == VIDEO_WVGA
#define MD_WIDTH	640
#define MD_HEIGHT	360
#endif

static video_params_t video_md_params = {
	.stream_id 		= MD_CHANNEL,
	.type 			= MD_TYPE,
	.resolution	 	= MD_RESOLUTION,
	.width 			= MD_WIDTH,
	.height 		= MD_HEIGHT,
	.bps 			= MD_BPS,
	.fps 			= MD_FPS,
	.gop 			= MD_GOP,
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

//eip acceleration resolution are 640x480, 640x360, 576x320, 416x416, 320x180, 128x128
static eip_param_t eip_param = {
	.image_width = MD_WIDTH,
	.image_height = MD_HEIGHT,
	.eip_row = MD_ROW,
	.eip_col = MD_COL
};

static void atcmd_userctrl_init(void);
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v1			= NULL;

static mm_context_t *video_md_ctx			= NULL;
static mm_context_t *md_ctx            = NULL;
static mm_siso_t *siso_rgb_md         = NULL;

//--------------------------------------------
// Draw Rect
//--------------------------------------------
#define MD_DRAW 1
#define MD_DRAW_ALL 0

#if MD_DRAW
#include "osd_render.h"

#endif
static void md_process(void *md_result)
{
	md_result_t *md_res = (md_result_t *) md_result;
#if MD_DRAW
#if MD_DRAW_ALL
	for (int i = 0; i < 4; i++) {
		canvas_create_bitmap(RTSP_CHANNEL, i, RTS_OSD2_BLK_FMT_1BPP);
		if (i < md_res->motion_cnt) {
			int xmin = (int)(md_res->md_pos[i].xmin * RTSP_WIDTH);
			int ymin = (int)(md_res->md_pos[i].ymin * RTSP_HEIGHT);
			int xmax = (int)(md_res->md_pos[i].xmax * RTSP_WIDTH);
			int ymax = (int)(md_res->md_pos[i].ymax * RTSP_HEIGHT);
			//printf("%d: x(%d,%d), y(%d,%d)\r\n",i,xmin,xmax,ymin,ymax);
			canvas_set_rect(RTSP_CHANNEL, i, xmin, ymin, xmax, ymax, 3, COLOR_GREEN);
		}
		int ready2update = 0;
		if (i == 3) {
			ready2update = 1;
		}
		canvas_update(RTSP_CHANNEL, i, ready2update);
	}
#else
	int motion = md_res->motion_cnt;
	canvas_create_bitmap(RTSP_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
	if (motion) {
		int xmin = (int)(md_res->md_pos[0].xmin * RTSP_WIDTH);
		int ymin = (int)(md_res->md_pos[0].ymin * RTSP_HEIGHT);
		int xmax = (int)(md_res->md_pos[0].xmax * RTSP_WIDTH);
		int ymax = (int)(md_res->md_pos[0].ymax * RTSP_HEIGHT);
		canvas_set_rect(RTSP_CHANNEL, 0, xmin, ymin, xmax, ymax, 3, COLOR_GREEN);
	}
	canvas_update(RTSP_CHANNEL, 0, 1);
#endif
#endif
}

void mmf2_video_example_md_rtsp_init(void)
{
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, RTSP_WIDTH, RTSP_HEIGHT, RTSP_BPS, 0,
						0, 0, 0, 0, 0,
						(MD_INPUT == USE_NV12 ? 1 : 0), MD_WIDTH, MD_HEIGHT, 0, 0,
						(MD_INPUT == USE_RGB ? 1 : 0), MD_WIDTH, MD_HEIGHT);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 0, (MD_INPUT == USE_NV12 ? &video_md_params : NULL), 0, NULL, 0,
						(MD_INPUT == USE_RGB ? &video_md_params : NULL));

#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_VOE_HEAP, voe_heap_size);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, RTSP_FPS * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_md_rtsp_fail;
	}

	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		printf("RTSP2 open fail\n\r");
		goto mmf2_example_md_rtsp_fail;
	}
	video_md_ctx = mm_module_open(&video_module);
	if (video_md_ctx) {
		//mm_module_ctrl(video_md_ctx, CMD_VIDEO_SET_VOE_HEAP, voe_heap_size);
		mm_module_ctrl(video_md_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_md_params);
		mm_module_ctrl(video_md_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_md_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_md_rtsp_fail;
	}

	char md_mask [MD_MASK_ROW * MD_MASK_COL] = {0};
	memset(md_mask, 1, sizeof(md_mask));
	md_ctx  = mm_module_open(&eip_module);
	if (md_ctx) {
		mm_module_ctrl(md_ctx, CMD_EIP_SET_PARAMS, (int)&eip_param);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_MASK, (int)&md_mask);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_DISPPOST, (int)md_process);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_EN, 1);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_START);
	} else {
		printf("md_ctx open fail\n\r");
		goto mmf2_example_md_rtsp_fail;
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
		printf("siso2 open fail\n\r");
		goto mmf2_example_md_rtsp_fail;
	}
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, RTSP_CHANNEL);	// start channel 0

	siso_rgb_md = siso_create();
	if (siso_rgb_md) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_rgb_md, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_rgb_md, MMIC_CMD_ADD_INPUT, (uint32_t)video_md_ctx, 0);
		siso_ctrl(siso_rgb_md, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		//siso_ctrl(siso_rgb_md, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 1024, 0);
		//siso_ctrl(siso_rgb_md, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_ctrl(siso_rgb_md, MMIC_CMD_ADD_OUTPUT, (uint32_t)md_ctx, 0);
		siso_start(siso_rgb_md);
	} else {
		printf("siso_rgb_md open fail\n\r");
		goto mmf2_example_md_rtsp_fail;
	}
	printf("siso_rgb_md started\n\r");
	mm_module_ctrl(video_md_ctx, CMD_VIDEO_APPLY, MD_CHANNEL);	// start channel 4
	mm_module_ctrl(video_md_ctx, CMD_VIDEO_YUV, 2);

#if MD_DRAW
	int ch_enable[3] = {1, 0, 0};
	int char_resize_w[3] = {16, 0, 0}, char_resize_h[3] = {32, 0, 0};
	int ch_width[3] = {RTSP_WIDTH, 0, 0}, ch_height[3] = {RTSP_HEIGHT, 0, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);
#endif

	atcmd_userctrl_init();
	return;
mmf2_example_md_rtsp_fail:

	return;
}

static const char *example = "mmf2_video_example_md_rtsp_init";
static void example_deinit(void)
{
	if (md_ctx) {
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_STOP);
	}

#if MD_DRAW
	osd_render_task_stop();
	osd_render_dev_deinit_all();
#endif
	//Pause Linker
	siso_pause(siso_rgb_md);
	siso_pause(siso_video_rtsp_v1);

	//Stop module
	mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, 0);
	mm_module_ctrl(video_md_ctx, CMD_VIDEO_STREAM_STOP, 0);

	//Delete linker
	siso_delete(siso_rgb_md);
	siso_delete(siso_video_rtsp_v1);

	//Close module
	rtsp2_v1_ctx = mm_module_close(rtsp2_v1_ctx);
	md_ctx = mm_module_close(md_ctx);
	video_md_ctx = mm_module_close(video_md_ctx);
	video_v1_ctx = mm_module_close(video_v1_ctx);

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
			printf("invalid state, can not do %s init!\r\n", example);
		}
	} else {
		printf("invalid cmd");
	}

	printf("user command 0x%x\r\n", user_cmd);
}

static log_item_t userctrl_items[] = {
	{"UC", fUC, },
};

static void atcmd_userctrl_init(void)
{
	log_service_add_table(userctrl_items, sizeof(userctrl_items) / sizeof(userctrl_items[0]));
}