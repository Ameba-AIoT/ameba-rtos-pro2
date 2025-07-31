/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_array.h"
#include "module_rtsp2.h"
#include "video_example_media_framework.h"
#include "log_service.h"

#define USE_H265 1

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
static mm_context_t *array_ctx            = NULL;
static mm_context_t *rtsp2_ctx            = NULL;
static mm_siso_t *siso_array_rtsp         = NULL;

static array_params_t h264_array_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.codec_id = VIDEO_CODEC,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.v = {
			.fps    = 25,
			.h264_nal_size = 4,
		}
	}
};

static rtsp2_params_t rtsp2_array_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.fps      = 25,
			.bps      = 0, // if unknown
		}
	}
};

void mmf2_video_example_array_rtsp_init(void)
{
	// Video array input (H264/H265)
	array_t array;
#if USE_H265
	array.data_addr = (uint32_t) h265_sample;
	array.data_len = (uint32_t) h265_sample_len;
#else
	array.data_addr = (uint32_t) h264_sample;
	array.data_len = (uint32_t) h264_sample_len;
#endif
	int i = 0;
	char tempchar;

	atcmd_userctrl_init();

	for (i = 0; i < 16; i++) {
		memcpy(&tempchar, (void *)(array.data_addr + i), 1);
		printf("%02X", tempchar);
	}
	printf("\r\n");


	array_ctx = mm_module_open(&array_module);
	if (array_ctx) {
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_PARAMS, (int)&h264_array_params);
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_ARRAY, (int)&array);
		mm_module_ctrl(array_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(array_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(array_ctx, CMD_ARRAY_APPLY, 0);
		mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 1);	// streamming on
	} else {
		rt_printf("ARRAY open fail\n\r");
		goto mmf2_video_example_array_rtsp_fail;
	}

	// RTSP
	rtsp2_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_ctx) {
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_array_params);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_video_example_array_rtsp_fail;
	}
	rt_printf("RTSP2 opened\n\r");


	//--------------Link---------------------------
	siso_array_rtsp = siso_create();
	if (siso_array_rtsp) {
		siso_ctrl(siso_array_rtsp, MMIC_CMD_ADD_INPUT, (uint32_t)array_ctx, 0);
		siso_ctrl(siso_array_rtsp, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_ctx, 0);
		siso_start(siso_array_rtsp);
	} else {
		rt_printf("siso_array_rtsp open fail\n\r");
		goto mmf2_video_example_array_rtsp_fail;
	}
	rt_printf("siso_array_rtsp started\n\r");

	return;
mmf2_video_example_array_rtsp_fail:

	return;
}

static const char *example = "mmf2_video_example_array_rtsp";
static void example_deinit(void)
{
	siso_pause(siso_array_rtsp);
	mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 0);
	siso_delete(siso_array_rtsp);
	mm_module_close(rtsp2_ctx);
	mm_module_close(array_ctx);
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
