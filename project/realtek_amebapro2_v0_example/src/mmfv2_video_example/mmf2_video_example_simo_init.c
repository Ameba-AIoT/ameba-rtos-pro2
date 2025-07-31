/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_simo.h"
#include "module_video.h"
#include "module_rtsp2.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "log_service.h"
#include "sensor.h"

/*****************************************************************************
* ISP channel : 1
* Video type  : H264/HEVC
*****************************************************************************/

#define V2_CHANNEL 0
#define V2_BPS 2*1024*1024
#define V2_RCMODE 2 // 1: CBR, 2: VBR
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

static void atcmd_userctrl_init(void);
static mm_context_t *video_v2_ctx			= NULL;
static mm_context_t *rtsp2_v2_ctx			= NULL;
static mm_context_t *rtsp2_v3_ctx			= NULL;

static mm_simo_t *simo_video_rtsp_v2_v3		= NULL;

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

void mmf2_video_example_simo_init(void)
{
	atcmd_userctrl_init();

	/*sensor capacity check & video parameter setting*/
	video_v2_params.resolution = VIDEO_FHD;
	video_v2_params.width = sensor_params[USE_SENSOR].sensor_width;
	video_v2_params.height = sensor_params[USE_SENSOR].sensor_height;
	video_v2_params.fps = sensor_params[USE_SENSOR].sensor_fps;
	video_v2_params.gop = sensor_params[USE_SENSOR].sensor_fps;
	/*rtsp parameter setting*/
	rtsp2_v2_params.u.v.fps = sensor_params[USE_SENSOR].sensor_fps;
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(0, 0, 0, 0, 0,
						1, video_v2_params.width, video_v2_params.height, V2_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);
#else
	int voe_heap_size = video_voe_presetting_by_params(NULL, 0, &video_v2_params, 0, NULL, 0, NULL);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_v2_ctx = mm_module_open(&video_module);
	if (video_v2_ctx) {
		mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v2_params);
		mm_module_ctrl(video_v2_ctx, MM_CMD_SET_QUEUE_LEN, video_v2_params.fps * 3);
		mm_module_ctrl(video_v2_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_exmaple_simo_fail;
	}

	rtsp2_v2_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v2_ctx) {
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v2_params);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_video_exmaple_simo_fail;
	}

	rtsp2_v3_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v3_ctx) {
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v2_params);
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_video_exmaple_simo_fail;
	}

	simo_video_rtsp_v2_v3 = simo_create();
	if (simo_video_rtsp_v2_v3) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		simo_ctrl(simo_video_rtsp_v2_v3, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		simo_ctrl(simo_video_rtsp_v2_v3, MMIC_CMD_ADD_INPUT, (uint32_t)video_v2_ctx, 0);
		simo_ctrl(simo_video_rtsp_v2_v3, MMIC_CMD_ADD_OUTPUT0, (uint32_t)rtsp2_v2_ctx, 0);
		simo_ctrl(simo_video_rtsp_v2_v3, MMIC_CMD_ADD_OUTPUT1, (uint32_t)rtsp2_v3_ctx, 0);
		simo_start(simo_video_rtsp_v2_v3);
	} else {
		rt_printf("simo open fail\n\r");
		goto mmf2_video_exmaple_simo_fail;
	}

	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_APPLY, V2_CHANNEL);

	return;
mmf2_video_exmaple_simo_fail:

	return;
}

static const char *example = "mmf2_video_example_simo";
static void example_deinit(void)
{
	//Pause Linker
	simo_pause(simo_video_rtsp_v2_v3, MM_OUTPUT0);
	simo_pause(simo_video_rtsp_v2_v3, MM_OUTPUT1);

	//Stop module
	mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_STREAM_STOP, V2_CHANNEL);

	//Delete linker
	simo_delete(simo_video_rtsp_v2_v3);

	//Close module
	mm_module_close(rtsp2_v2_ctx);
	mm_module_close(rtsp2_v3_ctx);
	mm_module_close(video_v2_ctx);

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
