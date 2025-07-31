/******************************************************************************
 *
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_rtsp2.h"
#include "module_demuxer.h"
#include "video_example_media_framework.h"
#include "log_service.h"
#include "mmf2_pro2_video_config.h"

static mm_context_t *rtsp2_ctx		= NULL;
static mm_context_t *demuxer_ctx	= NULL;

static mm_siso_t *siso_demuxer_rtsp	= NULL;

#define V1_CHANNEL 0
#define V1_RESOLUTION VIDEO_2K
#define V1_FPS 15
#define V1_GOP 15
//#define V1_RESOLUTION VIDEO_FHD
//#define V1_FPS 30
//#define V1_GOP 30
#define V1_BPS 2*1024*1024
//#define V1_RCMODE 2 // 1: CBR, 2: VBR

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

#if V1_RESOLUTION == VIDEO_VGA
#define V1_WIDTH	640
#define V1_HEIGHT	480
#elif V1_RESOLUTION == VIDEO_HD
#define V1_WIDTH	1280
#define V1_HEIGHT	720
#elif V1_RESOLUTION == VIDEO_FHD
#define V1_WIDTH	1920
#define V1_HEIGHT	1080
#elif V1_RESOLUTION == VIDEO_2K
#define V1_WIDTH	2560
#define V1_HEIGHT	1440
#else	// default value
#define V1_WIDTH	1280
#define V1_HEIGHT	720
#endif


static void atcmd_userctrl_init(void);
static rtsp2_params_t rtsp2_v_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.fps      = V1_FPS,
			.bps      = V1_BPS
		}
	}
};

static rtsp2_params_t rtsp2_a_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a = {
			.codec_id   = AV_CODEC_ID_PCMU,//AV_CODEC_ID_MP4A_LATM,
			.channel    = 1,
			.samplerate = 8000
		}
	}
};

static demuxer_params_t demuxer_params = {
	.start_time     = 10000,
	.stream_type    = STREAM_ALL,
	.loop_mode      = 1, //0 no loop, 1 loop

	.record_file_name = "AmebaPro_recording.mp4",
	.mem_total_size = 1 * 1024 * 1024,
	.mem_block_size = 128
};

void mmf2_video_example_demuxer_rtsp_init(void)
{
	atcmd_userctrl_init();

	demuxer_ctx = mm_module_open(&demuxer_module);
	if (demuxer_ctx) {
		mm_module_ctrl(demuxer_ctx, CMD_DEMUXER_SET_PARAMS, (int)&demuxer_params);
		mm_module_ctrl(demuxer_ctx, MM_CMD_SET_QUEUE_LEN, 12);
		mm_module_ctrl(demuxer_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(demuxer_ctx, CMD_DEMUXER_INIT_MEM_POOL, 0);
		mm_module_ctrl(demuxer_ctx, CMD_DEMUXER_OPEN, 0);
		printf("CMD_DEMUXER_STREAM_START\r\n");
		mm_module_ctrl(demuxer_ctx, CMD_DEMUXER_STREAM_START, 0);
	} else {
		rt_printf("DEMUXER open fail\n\r");
		goto mmf2_exmaple_demuxer_rtsp_fail;
	}

	rtsp2_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_ctx) {
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v_params);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_APPLY, 0);

		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SELECT_STREAM, 1);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_a_params);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_APPLY, 0);

		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_STREAMMING, ON);

	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_exmaple_demuxer_rtsp_fail;
	}

	siso_demuxer_rtsp = siso_create();
	if (siso_demuxer_rtsp) {
		siso_ctrl(siso_demuxer_rtsp, MMIC_CMD_ADD_INPUT, (uint32_t)demuxer_ctx, 0);
		siso_ctrl(siso_demuxer_rtsp, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_ctx, 0);
		siso_start(siso_demuxer_rtsp);
	} else {
		rt_printf("siso2 open fail\n\r");
		goto mmf2_exmaple_demuxer_rtsp_fail;
	}
	rt_printf("siso2 started\n\r");

	vTaskDelay(30000);
	printf("Set Pause for the MP4\r\n");
	mm_module_ctrl(demuxer_ctx, CMD_DEMUXER_STREAM_PAUSE, 0);

	vTaskDelay(30000);
	printf("Set RESUME for the MP4\r\n");
	mm_module_ctrl(demuxer_ctx, CMD_DEMUXER_STREAM_RESUME, 0);

	return;
mmf2_exmaple_demuxer_rtsp_fail:

	return;
}

static const char *example = "mmf2_video_example_demuxer_rtsp";
static void example_deinit(void)
{
	//Pause Linker
	siso_pause(siso_demuxer_rtsp);

	//Stop module
	mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(demuxer_ctx, CMD_DEMUXER_CLOSE, 0);

	//Delete linker
	siso_delete(siso_demuxer_rtsp);

	//Close module
	mm_module_close(rtsp2_ctx);
	mm_module_close(demuxer_ctx);

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
