/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "module_rtsp2.h"
#include "module_mp4.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "log_service.h"
#include "sensor.h"
#include <sntp/sntp.h>
#include "isp_osd_example.h"

/*****************************************************************************
* ISP channel : 0
* Video type  : H264/HEVC
*****************************************************************************/

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

static void atcmd_userctrl_init(void);
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *mp4_ctx			= NULL;
static mm_siso_t *siso_video_mp4_v1			= NULL;

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.bps = V1_BPS,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1
};

static mp4_timelapse_params_t mp4_timelapse_params = {
	.capture_interval = 2,
	.record_fps = 30,
};

static mp4_params_t mp4_v1_params = {
	.channel = 1,
	.record_length = 10,
	.record_type = STORAGE_VIDEO,
	.record_file_num = 1,
	.record_file_name = "Timelapse_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
};

static int mp4_end_cb(void *parm)
{
	printf("Record end\r\n");
	return 0;
}

void mmf2_video_example_timelapse_mp4_init(void)
{
	atcmd_userctrl_init();

	/*sensor capacity check & video parameter setting*/
	video_v1_params.resolution = VIDEO_FHD;
	video_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	video_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
	video_v1_params.fps = 1;
	video_v1_params.gop = 1;

	//mp4
	mp4_v1_params.fps = mp4_timelapse_params.record_fps;
	mp4_v1_params.gop = 1;
	mp4_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	mp4_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, video_v1_params.width, video_v1_params.height, V1_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 0, NULL, 0, NULL, 0, NULL);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_CAP_INTVL, mp4_timelapse_params.capture_interval);
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_example_timelapse_mp4_fail;
	}

	mp4_ctx = mm_module_open(&mp4_module);
	if (mp4_ctx) {
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_PARAMS, (int)&mp4_v1_params);
		mm_module_ctrl(mp4_ctx, CMD_MP4_LOOP_MODE, 0);
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_END_CB, (int)mp4_end_cb);
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_TIMELAPSE_PARAMS, (int)&mp4_timelapse_params);
	} else {
		printf("MP4 open fail\n\r");
		goto mmf2_video_example_timelapse_mp4_fail;
	}

	siso_video_mp4_v1 = siso_create();
	if (siso_video_mp4_v1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_mp4_v1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_mp4_v1, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_video_mp4_v1, MMIC_CMD_ADD_OUTPUT, (uint32_t)mp4_ctx, 0);
		siso_start(siso_video_mp4_v1);
	} else {
		rt_printf("siso2 open fail\n\r");
		goto mmf2_video_example_timelapse_mp4_fail;
	}

	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0

	//enable osd timestamp
	sntp_init();
	example_isp_osd(0, 0, 16, 32);
	mm_module_ctrl(mp4_ctx, CMD_MP4_START, mp4_v1_params.record_file_num);


	return;
mmf2_video_example_timelapse_mp4_fail:

	return;
}

static const char *example = "mmf2_video_example_v1";
static void example_deinit(void)
{
	//Pause Linker
	siso_pause(siso_video_mp4_v1);

	//Stop module
	mm_module_ctrl(mp4_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);

	//Delete linker
	siso_delete(siso_video_mp4_v1);

	//Close module
	mm_module_close(mp4_ctx);
	mm_module_close(video_v1_ctx);

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
