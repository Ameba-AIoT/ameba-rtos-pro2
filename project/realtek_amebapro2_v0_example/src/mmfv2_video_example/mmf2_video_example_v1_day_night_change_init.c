/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_video.h"
#include "module_rtsp2.h"
#include "module_eip.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "log_service.h"
#include "isp_ctrl_api.h"
#include "sensor_service.h"

static void atcmd_userctrl_init(void);
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v1		= NULL;
static mm_context_t *video_rgb_ctx			= NULL;
static mm_context_t *md_ctx            		= NULL;
static mm_siso_t *siso_rgb_md         		= NULL;

/*****************************************************************************
* ISP channel : 0
* Video type  : H264/HEVC
*****************************************************************************/
#define V1_CHANNEL 0
#define DAY_FPS 24
#define DAY_BPS 1.5 * 1024 * 1024
#define NIGHT_FPS 15
#define NIGHT_BPS 512 * 1024
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

#define BPS_STABLE_CONTROL 0

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.bps = DAY_BPS,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1
};

static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.bps      = DAY_BPS
		}
	}
};

/*****************************************************************************
* ISP channel : 4
* Video type  : RGB
*****************************************************************************/
#define MD_CHANNEL 4
#define MD_RESOLUTION VIDEO_VGA //VIDEO_WVGA
#define MD_GOP MD_FPS
#define MD_BPS 1024*1024
#define MD_COL 32
#define MD_ROW 32
#define MD_TYPE VIDEO_RGB

#if MD_RESOLUTION == VIDEO_VGA
#define MD_WIDTH	640
#define MD_HEIGHT	480
#elif MD_RESOLUTION == VIDEO_WVGA
#define MD_WIDTH	640
#define MD_HEIGHT	360
#endif

static video_params_t video_v4_params = {
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
};

#define USE_MD 1
#if USE_MD
//--------------------------------------------
// Draw Rect
//--------------------------------------------
#include "osd_render.h"
static void md_process(void *md_result)
{
	md_result_t *md_res = (md_result_t *) md_result;
	int motion = md_res->motion_cnt;
	canvas_create_bitmap(V1_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
	if (motion) {
		int xmin = (int)(md_res->md_pos[0].xmin * video_v1_params.width);
		int ymin = (int)(md_res->md_pos[0].ymin * video_v1_params.height);
		int xmax = (int)(md_res->md_pos[0].xmax * video_v1_params.width);
		int ymax = (int)(md_res->md_pos[0].ymax * video_v1_params.height);
		canvas_set_rect(V1_CHANNEL, 0, xmin, ymin, xmax, ymax, 3, COLOR_GREEN);
	}
	canvas_update(V1_CHANNEL, 0, 1);
}

static void md_osd_cleanup(void)
{
	canvas_create_bitmap(V1_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
	canvas_update(V1_CHANNEL, 0, 1);
}

static eip_param_t md_param_day = {
	.image_width = MD_WIDTH,
	.image_height = MD_HEIGHT,
	.eip_row = 32,
	.eip_col = 32
};
static md_config_t md_config_day = {
	.adapt_mode = 0,
	.adapt_level = 1.1,
	.adapt_step = 30,
	.adapt_thr_max = 10,
	.bg_mode = 0,
	.detect_interval = 1,
	.his_resolution = 5,
	.his_threshold = 50,
	.his_step = 100,
	.md_obj_sensitivity = 85,
	.md_time_filter_interval = 3,
	.md_trigger_block_threshold = 0,
	.block_base_thr = 1,
	.block_lum_thr = 3,
};

static eip_param_t md_param_night = {
	.image_width = MD_WIDTH,
	.image_height = MD_HEIGHT,
	.eip_row = 32,
	.eip_col = 64
};
static md_config_t md_config_night = {
	.adapt_mode = 0,
	.adapt_level = 1.1,
	.adapt_step = 30,
	.adapt_thr_max = 10,
	.bg_mode = 0,
	.detect_interval = 1,
	.his_resolution = 6,
	.his_threshold = 190,
	.his_step = 200,
	.md_obj_sensitivity = 95,
	.md_time_filter_interval = 3,
	.md_trigger_block_threshold = 0,
	.block_base_thr = 0.5,
	.block_lum_thr = 1,
};
#endif

static rate_ctrl_s rc_ctrl_day = {
	.bps = DAY_BPS,
	.isp_fps = DAY_FPS,
	.fps = DAY_FPS,
	.gop = DAY_FPS,
};

static rate_ctrl_s rc_ctrl_night = {
	.bps = NIGHT_BPS,
	.isp_fps = NIGHT_FPS,
	.fps = NIGHT_FPS,
	.gop = NIGHT_FPS,
};

#if BPS_STABLE_CONTROL
static bps_stbl_ctrl_param_t bps_stbl_ctrl_day_params = {
	.maximun_bitrate = DAY_BPS * 1.2,
	.minimum_bitrate = DAY_BPS * 0.8,
	.target_bitrate = DAY_BPS,
	.sampling_time = 2000,
};
static bps_stbl_ctrl_param_t bps_stbl_ctrl_night_params = {
	.maximun_bitrate = NIGHT_BPS * 1.2,
	.minimum_bitrate = NIGHT_BPS * 0.8,
	.target_bitrate = NIGHT_BPS,
	.sampling_time = 2000,
};
static uint32_t bps_stbl_ctrl_day_fps_stage[BPS_STBL_CTRL_STG_CNT] = {DAY_FPS, DAY_FPS * 0.8, DAY_FPS * 0.6};
static uint32_t bps_stbl_ctrl_day_gop_stage[BPS_STBL_CTRL_STG_CNT] = {DAY_FPS * 2, DAY_FPS * 0.8 * 2, DAY_FPS * 0.6 * 2}; //default set gop = fps * 2
static uint32_t bps_stbl_ctrl_night_fps_stage[BPS_STBL_CTRL_STG_CNT] = {NIGHT_FPS, NIGHT_FPS * 0.8, NIGHT_FPS * 0.6};
static uint32_t bps_stbl_ctrl_night_gop_stage[BPS_STBL_CTRL_STG_CNT] = {NIGHT_FPS * 2, NIGHT_FPS * 0.8 * 2, NIGHT_FPS * 0.6 * 2}; //default set gop = fps * 2
#endif

typedef enum {
	DAY_MODE = 0,
	NIGHT_MODE,
} day_night_mode_change_t;

static void fps_change(day_night_mode_change_t mode, int sensor_fps, rate_ctrl_s *rc, bps_stbl_ctrl_param_t *bps_stbl_ctrl_param, uint32_t *fps_stage, uint32_t *gop_stage)
{
#if BPS_STABLE_CONTROL
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_BPS_STBL_CTRL_EN, 0);
#endif
	if (mode == DAY_MODE) {
		//Set ISP FPS. raise maxfps first. minfps cannot set larger than maxfps.
		isp_set_max_fps(sensor_fps);
		isp_set_min_fps(sensor_fps);
	} else {
		//Set ISP FPS. lower minfps first. minfps cannot set larger than maxfps.
		isp_set_min_fps(sensor_fps);
		isp_set_max_fps(sensor_fps);
	}
	if(video_wait_target_fps(V1_CHANNEL, sensor_fps, 500) == OK) {
		//Set Encode configuration
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_MULTI_RCCTRL, (int)rc);
#if BPS_STABLE_CONTROL
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_PARAMS, (int)bps_stbl_ctrl_param);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_FPS_STG, (int)fps_stage);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_GOP_STG, (int)gop_stage);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_BPS_STBL_CTRL_EN, 1);
#endif
	}
}

static void day_night_mode_change(day_night_mode_change_t mode)
{
	if (mode == DAY_MODE) {
#if CONFIG_RTK_EVB_IR_CTRL
		ir_ctrl_set_brightness_d(0); //close ir light
		ir_cut_enable(1); //enable ir cut
#endif
		//Change iq paramter
		isp_set_day_night(0);
		//Set to Color Mode, IQ table has a default minfps.
		isp_set_gray_mode(0);
#if BPS_STABLE_CONTROL
		fps_change(mode, DAY_FPS, &rc_ctrl_day, &bps_stbl_ctrl_day_params, bps_stbl_ctrl_day_fps_stage, bps_stbl_ctrl_day_gop_stage);
#else
		fps_change(mode, DAY_FPS, &rc_ctrl_day, NULL, NULL, NULL);
#endif
	} else if (mode == NIGHT_MODE) {
		//Change iq paramter
		isp_set_gray_mode(1);
#if CONFIG_RTK_EVB_IR_CTRL
		ir_cut_enable(0); //close ir cut
		ir_ctrl_set_brightness_d(100);//open ir light
#endif
		//Set to Gray Mode, IQ table has a default minfps.
		isp_set_day_night(1);
#if BPS_STABLE_CONTROL
		fps_change(mode, NIGHT_FPS, &rc_ctrl_night, &bps_stbl_ctrl_night_params, bps_stbl_ctrl_night_fps_stage, bps_stbl_ctrl_night_gop_stage);
#else
		fps_change(mode, NIGHT_FPS, &rc_ctrl_night, NULL, NULL, NULL);
#endif
	}
};

static int md_delay_start = 1000; //start md 1000ms after mode change to prevent false alarm
void mmf2_video_example_v1_day_night_change_init(void)
{

	atcmd_userctrl_init();

	/*sensor capacity check & video parameter setting*/
	video_v1_params.resolution = VIDEO_FHD;
	video_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	video_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
	video_v1_params.fps = DAY_FPS;
	video_v1_params.gop = DAY_FPS;
#if BPS_STABLE_CONTROL
	video_v1_params.fps = bps_stbl_ctrl_day_fps_stage[0];
	video_v1_params.gop = bps_stbl_ctrl_day_gop_stage[0];
	rc_ctrl_day.fps = bps_stbl_ctrl_day_fps_stage[0];
	rc_ctrl_day.gop = bps_stbl_ctrl_day_gop_stage[0];
	rc_ctrl_night.fps = bps_stbl_ctrl_night_fps_stage[0];
	rc_ctrl_night.gop = bps_stbl_ctrl_night_gop_stage[0];
#endif
#if USE_MD
	video_v4_params.use_roi = 1;
	video_v4_params.roi.xmin = 0;
	video_v4_params.roi.ymin = 0;
	video_v4_params.roi.xmax = sensor_params[USE_SENSOR].sensor_width;
	video_v4_params.roi.ymax = sensor_params[USE_SENSOR].sensor_height;
#endif
	/*rtsp parameter setting*/
	rtsp2_v1_params.u.v.fps = video_v1_params.fps;
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, video_v1_params.width, video_v1_params.height, DAY_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						USE_MD, MD_WIDTH, MD_HEIGHT);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 0, NULL, 0, NULL, 0, (USE_MD ? &video_v4_params : NULL));
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_example_v1_day_night_change_fail;
	}

	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		printf("RTSP2 open fail\n\r");
		goto mmf2_video_example_v1_day_night_change_fail;
	}

#if USE_MD
	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_example_v1_day_night_change_fail;
	}

	memset(md_config_day.md_mask, 1, sizeof(char) * MD_MASK_COL * MD_MASK_ROW);
	memset(md_config_night.md_mask, 1, sizeof(char) * MD_MASK_COL * MD_MASK_ROW);
	md_ctx  = mm_module_open(&eip_module);
	if (md_ctx) {
		mm_module_ctrl(md_ctx, CMD_EIP_SET_PARAMS, (int)&md_param_day);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_MASK, (int) & (md_config_day.md_mask));
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_DISPPOST, (int)md_process);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_EN, 1);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_START);
	} else {
		printf("md_ctx open fail\n\r");
		goto mmf2_video_example_v1_day_night_change_fail;
	}
#endif

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
		rt_printf("siso2 open fail\n\r");
		goto mmf2_video_example_v1_day_night_change_fail;
	}
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);

#if BPS_STABLE_CONTROL
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_PARAMS, (int)&bps_stbl_ctrl_day_params);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_FPS_STG, (int)bps_stbl_ctrl_day_fps_stage);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_GOP_STG, (int)bps_stbl_ctrl_day_gop_stage);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_BPS_STBL_CTRL_EN, 1);
#endif

#if USE_MD
	siso_rgb_md = siso_create();
	if (siso_rgb_md) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_rgb_md, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_rgb_md, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		siso_ctrl(siso_rgb_md, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_rgb_md, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_ctrl(siso_rgb_md, MMIC_CMD_ADD_OUTPUT, (uint32_t)md_ctx, 0);
		siso_start(siso_rgb_md);
	} else {
		printf("siso_rgb_md open fail\n\r");
		goto mmf2_video_example_v1_day_night_change_fail;
	}
	printf("siso_rgb_md started\n\r");
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, MD_CHANNEL);	// start channel 4
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);

	int ch_enable[3] = {1, 0, 0};
	int char_resize_w[3] = {16, 0, 0}, char_resize_h[3] = {32, 0, 0};
	int ch_width[3] = {video_v1_params.width, 0, 0}, ch_height[3] = {video_v1_params.height, 0, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);
#endif

	//video_show_fps(1); //show fps log

	printf("changing day mode and night mode\n\r");
	for (int i = 0; i < 10; i++) {
		vTaskDelay(10000);
		printf("change to night mode %d\r\n", i);
		//--------------------------------------------
		// DAY -> NIGHT
		// 1. clean MD OSD drawing
		// 2. stop MD
		// 3. change to gray mode
		// 4. switch night mode ircut settings
		// 5. switch night mode IQ table
		// 6. wait 1000ms after mode change to prevent md false alarm
		// 7. change MD night configuration
		// 8. start MD
		//--------------------------------------------
#if USE_MD
		md_osd_cleanup();
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_STOP); //stop eip handle before changing iq table
#endif
		day_night_mode_change(NIGHT_MODE);
#if USE_MD
		vTaskDelay(md_delay_start);
		mm_module_ctrl(md_ctx, CMD_EIP_AE_STABLE_EN, 1);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_PARAMS, (int)&md_param_night);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_CONFIG, (int)&md_config_night);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_START);
#endif

		vTaskDelay(10000);
		printf("change to day mode %d\r\n", i);
		//--------------------------------------------
		// NIGHT -> DAY
		// 1. clean MD OSD drawing
		// 2. stop MD
		// 3. switch day mode ircut settings
		// 4. switch day mode IQ table
		// 5. change to rgb mode
		// 6. wait 1000ms after mode change to prevent MD false alarm
		// 7. change MD day configuration
		// 8. start MD
		//--------------------------------------------
#if USE_MD
		md_osd_cleanup();
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_STOP); //stop eip handle before changing iq table
#endif
		day_night_mode_change(DAY_MODE);
#if USE_MD
		vTaskDelay(md_delay_start);
		mm_module_ctrl(md_ctx, CMD_EIP_AE_STABLE_EN, 1);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_PARAMS, (int)&md_param_day);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_CONFIG, (int)&md_config_day);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_START);
#endif
	}

	return;
mmf2_video_example_v1_day_night_change_fail:

	return;
}

static const char *example = "mmf2_video_example_v1_param_change";
#if USE_MD
static void example_deinit(void)
{
	if (md_ctx) {
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_STOP);
	}
	osd_render_task_stop();
	osd_render_dev_deinit_all();
	//Pause Linker
	siso_pause(siso_rgb_md);
	siso_pause(siso_video_rtsp_v1);

	//Stop module
	mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, 0);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_STREAM_STOP, 0);

	//Delete linker
	siso_delete(siso_rgb_md);
	siso_delete(siso_video_rtsp_v1);

	//Close module
	rtsp2_v1_ctx = mm_module_close(rtsp2_v1_ctx);
	md_ctx = mm_module_close(md_ctx);
	video_rgb_ctx = mm_module_close(video_rgb_ctx);
	video_v1_ctx = mm_module_close(video_v1_ctx);

	video_voe_release();
}
#else
static void example_deinit(void)
{
	//Pause Linker
	siso_pause(siso_video_rtsp_v1);

	//Stop module
	mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, 0);

	//Delete linker
	siso_delete(siso_video_rtsp_v1);

	//Close module
	rtsp2_v1_ctx = mm_module_close(rtsp2_v1_ctx);
	video_v1_ctx = mm_module_close(video_v1_ctx);

	video_voe_release();
}
#endif

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
