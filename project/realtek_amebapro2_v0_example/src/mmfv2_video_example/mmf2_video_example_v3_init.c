/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
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

/*****************************************************************************
* ISP channel : 2
* Video type  : JPEG
*****************************************************************************/

#define V3_CHANNEL 2
#define V3_FPS 5
#define V3_GOP 5
#define V3_BPS 2*1024*1024
#define V3_RCMODE 2 // 1: CBR, 2: VBR

static void atcmd_userctrl_init(void);
static mm_context_t *video_v3_ctx			= NULL;
static mm_context_t *rtsp2_v3_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v3			= NULL;

static video_params_t video_v3_params = {
	.stream_id = V3_CHANNEL,
	.type = VIDEO_JPEG,
	.use_static_addr = 1
};


static rtsp2_params_t rtsp2_v3_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = AV_CODEC_ID_MJPEG,
		}
	}
};


void mmf2_video_example_v3_init(void)
{
	atcmd_userctrl_init();

	/*sensor capacity check & video parameter setting*/
	video_v3_params.resolution = VIDEO_FHD;

	//In RFC 2435 for rtp jpg, the width and height should be less than 2040 (8bits 255 * 8 = 2040)
	//User could ignore this checking, if there server could support
	video_v3_params.width = sensor_params[USE_SENSOR].sensor_width > 2040 ? 2040 : sensor_params[USE_SENSOR].sensor_width;
	video_v3_params.height = sensor_params[USE_SENSOR].sensor_height > 2040 ? 2040 : sensor_params[USE_SENSOR].sensor_height;
	video_v3_params.fps = V3_FPS;
	video_v3_params.gop = V3_GOP;
	/*rtsp parameter setting*/
	rtsp2_v3_params.u.v.fps = sensor_params[USE_SENSOR].sensor_fps;
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						1, video_v3_params.width, video_v3_params.height, V3_BPS, 0,
						0, 0, 0);
#else
	int voe_heap_size = video_voe_presetting_by_params(NULL, 0, NULL, 0, &video_v3_params, 0, NULL);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_v3_ctx = mm_module_open(&video_module);
	if (video_v3_ctx) {
		mm_module_ctrl(video_v3_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v3_params);
		mm_module_ctrl(video_v3_ctx, MM_CMD_SET_QUEUE_LEN, video_v3_params.fps * 3);
		mm_module_ctrl(video_v3_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_exmaple_v3_fail;
	}


	rtsp2_v3_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v3_ctx) {
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v3_params);
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_video_exmaple_v3_fail;
	}

	siso_video_rtsp_v3 = siso_create();
	if (siso_video_rtsp_v3) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_rtsp_v3, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_rtsp_v3, MMIC_CMD_ADD_INPUT, (uint32_t)video_v3_ctx, 0);
		siso_ctrl(siso_video_rtsp_v3, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v3_ctx, 0);
		siso_start(siso_video_rtsp_v3);
	} else {
		rt_printf("siso2 open fail\n\r");
		goto mmf2_video_exmaple_v3_fail;
	}

	mm_module_ctrl(video_v3_ctx, CMD_VIDEO_APPLY, V3_CHANNEL);
	mm_module_ctrl(video_v3_ctx, CMD_VIDEO_SNAPSHOT, 2);

	return;
mmf2_video_exmaple_v3_fail:

	return;
}

static const char *example = "mmf2_video_example_v3";

static void example_deinit(void)
{
	//Pause Linker
	siso_pause(siso_video_rtsp_v3);

	//Stop module
	mm_module_ctrl(rtsp2_v3_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v3_ctx, CMD_VIDEO_STREAM_STOP, V3_CHANNEL);

	//Delete linker
	siso_delete(siso_video_rtsp_v3);

	//Close module
	mm_module_close(rtsp2_v3_ctx);
	mm_module_close(video_v3_ctx);

	video_voe_release();
}

static void fUC(void *arg)
{
	static uint32_t user_cmd = 0;

	if (!strcmp(arg, "TD")) {
		if (user_cmd & USR_CMD_EXAMPLE_DEINIT) {
			printf("invalid state, can not do %s deinit!\r\n", example);
		} else {
			user_cmd = USR_CMD_EXAMPLE_DEINIT;
			example_deinit();
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
