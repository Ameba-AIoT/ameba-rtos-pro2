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
static mm_context_t *rtsp2_v1_ctx			= NULL;

static mm_siso_t *siso_video_rtsp_v1			= NULL;

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.bps = V1_BPS,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1
};

static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.bps      = V1_BPS
		}
	}
};

enum param_change_option {
	RESOLUTION_BPS_CHANGE_TEST = 0,
	BPS_CHANGE_TEST,
	QP_CHANGE_TEST,
	FORCEI_TEST,
	SCALE_UP_TEST,
	ISP_INIT_TEST,
	SENSOR_DRIVER_CHANGE_TEST,
	TEST_NUM
};

static const char* param_change_test_item[] = {
	"RESOLUTION_BPS_CHANGE_TEST",
	"BPS_CHANGE_TEST",
	"QP_CHANGE_TEST",
	"FORCEI_TEST",
	"SCALE_UP_TEST",
	"ISP_INIT_TEST",
	"SENSOR_DRIVER_CHANGE_TEST"
};

static void change_resolution_parameter(int parm_index)
{
	if (parm_index == 0) {
		video_v1_params.resolution = VIDEO_FHD;
		video_v1_params.width = 1920;
		video_v1_params.height = 1080;
		video_v1_params.bps = 2 * 1024 * 1024;
	} else {
		video_v1_params.resolution = VIDEO_HD;
		video_v1_params.width = 1280;
		video_v1_params.height = 720;
		video_v1_params.bps = 1 * 1024 * 1024;
	}
}

static uint32_t scale_up_width = 0;
static uint32_t scale_up_height = 0;
static video_roi_t crop_roi = {0};
static int scale_up_load_demo_param(void)
{
	/*The scale up demo only demonstrates for sensor resolutions 1920x1080 and 2560x1440.
	For other resolutions, users need to add the scale up size and roi settings.*/
	if (sensor_params[USE_SENSOR].sensor_width == 1920 && sensor_params[USE_SENSOR].sensor_height == 1080) {
		scale_up_width = 2560;
		scale_up_height = 1440;
		crop_roi.xmin = 80;
		crop_roi.ymin = 44;
		crop_roi.xmax = 80 + 1760;
		crop_roi.ymax = 44 + 990;
	} else if (sensor_params[USE_SENSOR].sensor_width == 2560 && sensor_params[USE_SENSOR].sensor_height == 1440) {
		scale_up_width = 2688;
		scale_up_height = 1520;
		crop_roi.xmin = 280;
		crop_roi.ymin = 156;
		crop_roi.xmax = 280 + 2000;
		crop_roi.ymax = 156 + 1126;
	} else {
		printf("Please add the scale up size and roi settings.\r\n");
		return -1;
	}
	return 0;
}

static void scale_up_set_param(int idx)
{
	if(idx == 0) {
		printf("revert video settings\n\r");
		video_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
		video_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
		video_v1_params.use_roi = 0;
	} else if (idx == 1) {
		//only scale up
		printf("%dx%d scale up to %dx%d test\n\r", video_v1_params.width, video_v1_params.height, scale_up_width, scale_up_height);
		video_v1_params.width = scale_up_width;
		video_v1_params.height = scale_up_height;
		video_v1_params.use_roi = 0;
	} else if (idx == 2) {
		//crop + scale up
		printf("crop %dx%d scale up to %dx%d test\n\r", (crop_roi.xmax - crop_roi.xmin), (crop_roi.ymax - crop_roi.ymin), scale_up_width, scale_up_height);
		video_v1_params.width = scale_up_width;
		video_v1_params.height = scale_up_height;
		video_v1_params.use_roi = 1;
		memcpy(&video_v1_params.roi, &crop_roi, sizeof(video_roi_t));
	}
}

void mmf2_video_example_v1_param_change_init(void)
{
	int i = 0;

	atcmd_userctrl_init();

	/*sensor capacity check & video parameter setting*/
	video_v1_params.resolution = VIDEO_FHD;
	video_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	video_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
	video_v1_params.fps = sensor_params[USE_SENSOR].sensor_fps;
	video_v1_params.gop = sensor_params[USE_SENSOR].sensor_fps;
	/*rtsp parameter setting*/
	rtsp2_v1_params.u.v.fps = video_v1_params.fps;
	rtsp2_v1_params.u.v.bps = video_v1_params.bps;

	//malloc video heap with scale up resolution
	if(scale_up_load_demo_param() == 0) {
		video_v1_params.width = scale_up_width;
		video_v1_params.height = scale_up_height;
	} else {
		goto mmf2_video_exmaple_v1_param_change_fail;
	}

#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, video_v1_params.width, video_v1_params.height, V1_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 0, NULL, 0, NULL, 0, NULL);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	//revert video resolution settings
	video_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	video_v1_params.height = sensor_params[USE_SENSOR].sensor_height;

	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_exmaple_v1_param_change_fail;
	}

	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		printf("RTSP2 open fail\n\r");
		goto mmf2_video_exmaple_v1_param_change_fail;
	}

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
		goto mmf2_video_exmaple_v1_param_change_fail;
	}

	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);
	printf("PCT=[idx] or PCT=list or PCT=all\n\r");

	return;
mmf2_video_exmaple_v1_param_change_fail:

	return;
}

static void param_change_test(int idx, char *argv[MAX_ARGC])
{
	int i;
	switch (idx)
	{
	case RESOLUTION_BPS_CHANGE_TEST:
		printf("changing resolution and bps test\n\r");
		for (i = 0; i < 2; i++) {
			siso_pause(siso_video_rtsp_v1);
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
			change_resolution_parameter(i);

			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
			siso_resume(siso_video_rtsp_v1);
			// wait 3 seconds, change resolution
			vTaskDelay(3000);
		}
		break;
	case BPS_CHANGE_TEST:
		for (i = 0; i < 10; i++) {
			printf("changing bit rate test = %d\n\r", (1024 * 1024 + 1024 * (1 + i)));
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_BPS, (1024 * 1024 + 1024 * (1 + i)));
			vTaskDelay(3000);
		}
		printf("revert bitrate to %d\n\r", V1_BPS);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_BPS, V1_BPS);
		break;
	case QP_CHANGE_TEST:
		for (i = 0; i < 5; i++) {
			printf("changing QP test = %d, %d\n\r", (25 + i * 2), (35 + i * 2));
			encode_rc_parm_t rc_parm;
			memset(&rc_parm, 0, sizeof(encode_rc_parm_t));
			rc_parm.minQp = (25 + i * 2);
			rc_parm.maxQp = (35 + i * 2);

			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_RCPARAM, (int)&rc_parm);
			vTaskDelay(3000);
		}
		break;
	case FORCEI_TEST:
		for (i = 0; i < 10; i++) {
			printf("changing forcei test\n\r");
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_FORCE_IFRAME, 0);
			vTaskDelay(500);
		}
		break;
	case SCALE_UP_TEST:
		printf("scale up test\n\r");
		for(i = 0; i < 2; i++) {
			siso_pause(siso_video_rtsp_v1);
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
			scale_up_set_param(i + 1);
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
			siso_resume(siso_video_rtsp_v1);
			vTaskDelay(5000);
		}

		printf("revert to origin settings\n\r");
		siso_pause(siso_video_rtsp_v1);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
		scale_up_set_param(0);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
		siso_resume(siso_video_rtsp_v1);
		break;
	case ISP_INIT_TEST:
		printf("isp init test\n\r");
		siso_pause(siso_video_rtsp_v1);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
		video_pre_init_params_t init_params;
		memset(&init_params, 0x00, sizeof(video_pre_init_params_t));
		//enable drop frame
		init_params.video_drop_enable = 1;
		init_params.video_drop_frame = 5;
		
		//isp init settings
		init_params.isp_init_enable = 1;
		init_params.init_isp_items.init_brightness = 0x00;
		init_params.init_isp_items.init_contrast = 0x50;
		init_params.init_isp_items.init_flicker = 0x02;
		init_params.init_isp_items.init_hdr_mode = 0x00;
		init_params.init_isp_items.init_mirrorflip = 0xf0;
		init_params.init_isp_items.init_saturation = 050;
		init_params.init_isp_items.init_wdr_level = 0x50;
		init_params.init_isp_items.init_wdr_mode = 0x02;
		init_params.init_isp_items.init_mipi_mode = 0x0;

		//isp init ae, awb settings
		init_params.isp_ae_enable = 1;
		init_params.isp_ae_init_exposure = 10000;
		init_params.isp_ae_init_gain = 256;
		init_params.isp_awb_enable = 1;
		init_params.isp_awb_init_rgain = 256;
		init_params.isp_awb_init_bgain = 867;

		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&init_params);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
		siso_resume(siso_video_rtsp_v1);
		break;
	case SENSOR_DRIVER_CHANGE_TEST:
	{
		printf("sensor driver change test\n\r");
		int sensor_id = 0;
		if(argv == NULL) {
			printf("need to input sensor driver id!\r\n");
			break;
		} else {
			if(argv[2] == NULL) {
				printf("please enter sensor id with 'PCT=%d,[sensor_id]'\r\n", SENSOR_DRIVER_CHANGE_TEST);
				break;
			}
			sensor_id = strtol(argv[2], NULL, 10);
			if(sensor_id == 0 || sensor_id >= SENSOR_MAX) {
				printf("invalid sensor id %d\r\n", sensor_id);
				break;
			}
		}
		
		//close all video stream
		siso_pause(siso_video_rtsp_v1);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_SENSOR_ID, sensor_id);

		//update video and rtsp parameters
		video_v1_params.width = sensor_params[sen_id[sensor_id]].sensor_width;
		video_v1_params.height = sensor_params[sen_id[sensor_id]].sensor_height;
		video_v1_params.fps = sensor_params[sen_id[sensor_id]].sensor_fps;
		video_v1_params.gop = sensor_params[sen_id[sensor_id]].sensor_fps;
		rtsp2_v1_params.u.v.fps = video_v1_params.fps;
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);

		//if change sensor reslution, update voe heap size
		video_voe_release();
		int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 1, NULL, 0, NULL, 0, NULL);
		printf("\r\n voe heap size = %d\r\n", voe_heap_size);

		//restart video
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
		siso_resume(siso_video_rtsp_v1);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);
		break;
	}
	default:
		break;
	}
}

static void fPCT(void *arg) //param change test
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};

	if (!arg) {
		return;
	}
	argc = parse_param(arg, argv);
	if (argc == 1) {
		// get array size
		//
		printf("PCT=[idx] or PCT=list or PCT=all\n\r");
	} else {
		if (strcmp("all", argv[1]) == 0) {
			for (int i = 0; i < TEST_NUM; i++) {
				param_change_test(i, NULL);
			}
			return;
		} else if (strcmp("list", argv[1]) == 0) {
			for (int i = 0; i < TEST_NUM; i++) {
				printf("%02d : %s\n\r", i, param_change_test_item[i]);
			}
			return;
		}

		int idx = strtol(argv[1], NULL, 10);
		param_change_test(idx, argv);
	}
	return;
}

static const char *example = "mmf2_video_example_v1_param_change";
static void example_deinit(void)
{
	//Pause Linker
	siso_pause(siso_video_rtsp_v1);

	//Stop module
	mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);

	//Delete linker
	siso_delete(siso_video_rtsp_v1);

	//Close module
	mm_module_close(rtsp2_v1_ctx);
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
	{"PCT", fPCT, },
};

static void atcmd_userctrl_init(void)
{
	log_service_add_table(userctrl_items, sizeof(userctrl_items) / sizeof(userctrl_items[0]));
}
