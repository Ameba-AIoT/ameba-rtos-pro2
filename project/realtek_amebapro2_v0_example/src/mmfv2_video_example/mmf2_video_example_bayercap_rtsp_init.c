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

#include "module_rtsp2.h"
#include "log_service.h"
#include "module_filesaver.h"
#include "ff.h"

#undef printf // undefine hal_vidoe.h printf 
#include <stdio.h>

#include "fatfs_sdcard_api.h"
char savefilepath[64];
int file_count = 0;
FIL     m_file1;


/*****************************************************************************
* ISP channel : 4
* Video type  : RGB
*****************************************************************************/
#define RTSP_CHANNEL 1
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

static video_params_t video_v2_params = {
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


static rtsp2_params_t rtsp2_v2_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = RTSP_CODEC,
			.fps      = RTSP_FPS,
			.bps      = RTSP_BPS
		}
	}
};

static void atcmd_userctrl_init(void);
static mm_context_t *video_v2_ctx			= NULL;
static mm_context_t *rtsp2_v2_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v2			= NULL;

#define V1_RAW_CHANNEL 0
#define V1_RAW_RESOLUTION VIDEO_FHD//VIDEO_HD//VIDEO_FHD 
#define V1_RAW_FPS 2
#define V1_RAW_GOP 80
#define V1_RAW_BPS 1024*1024
#define V1_RAW_RCMODE 1 // 1: CBR, 2: VBR

#define V1_RAW_TYPE VIDEO_NV16//VIDEO_JPEG//VIDEO_NV2_RAW2//VIDEO_JPEG//VIDEO_H264//VIDEO_HEVC//VIDEO_NV2_RAW6
#define VIDEO_CODEC AV_CODEC_ID_H264

static mm_context_t *video_bayercap_ctx			= NULL;

static video_params_t video_bayercap_params = {
	.stream_id = V1_RAW_CHANNEL,
	.type = V1_RAW_TYPE,
	.width = sensor_params[USE_SENSOR].sensor_width,
	.height = sensor_params[USE_SENSOR].sensor_height,
	.fps = V1_RAW_FPS,
	.gop = V1_RAW_GOP,
	.bps = V1_RAW_BPS,
	.rc_mode = V1_RAW_RCMODE,
	.use_static_addr = 1
};


static mm_context_t *filesaver_ctx         = NULL;
static mm_siso_t *siso_array_filesaver     = NULL;

void raw_reform(unsigned char *pData, int dataLen)
{
	int dim = dataLen / 2;
	unsigned char *pTmp = malloc(dataLen);
	memcpy(pTmp, pData, dataLen);
	int nIndex = 0;
	for (int j = 0; j < dim; j++) {
		int nValue = (pTmp[nIndex] << 8) | pTmp[nIndex + dim];

		pData[2 * nIndex] = nValue & 0xff;
		pData[2 * nIndex + 1] = (nValue >> 8) & 0xff;
		nIndex++;
	}
	free(pTmp);
}

void file_save(char *file_path, uint32_t data_addr, uint32_t data_size)
{
	int bw = 0;
	raw_reform((unsigned char *)data_addr, data_size);
	if (f_open(&m_file1, savefilepath, FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK) {
		if (data_size > 0) {
			printf("file_path:%s  data_addr:%d  data_size:%d \r\n", file_path, data_addr, data_size);
			f_write(&m_file1, (void *)data_addr, data_size, (u32 *)&bw);
			f_close(&m_file1);
		}
	}
	sprintf(savefilepath, "raw_img_%02d.raw", file_count++);
	mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_SET_SAVE_FILE_PATH, (int)savefilepath);

}

static void mmf2_rtsp_init(void)
{
	video_v2_ctx = mm_module_open(&video_module);
	if (video_v2_ctx) {
		//mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_VOE_HEAP, voe_heap_size);
		mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v2_params);
		mm_module_ctrl(video_v2_ctx, MM_CMD_SET_QUEUE_LEN, RTSP_FPS * 3);
		mm_module_ctrl(video_v2_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_rtsp_fail;
	}

	rtsp2_v2_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v2_ctx) {
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v2_params);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		printf("RTSP2 open fail\n\r");
		goto mmf2_example_rtsp_fail;
	}

	//--------------Link---------------------------
	siso_video_rtsp_v2 = siso_create();
	if (siso_video_rtsp_v2) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_rtsp_v2, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_rtsp_v2, MMIC_CMD_ADD_INPUT, (uint32_t)video_v2_ctx, 0);
		siso_ctrl(siso_video_rtsp_v2, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v2_ctx, 0);
		siso_start(siso_video_rtsp_v2);
	} else {
		printf("siso2 open fail\n\r");
		goto mmf2_example_rtsp_fail;
	}
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_APPLY, RTSP_CHANNEL);	// start channel 0

mmf2_example_rtsp_fail:

	return;
}
static void mmf2_rtsp_deinit(void)
{
	//Pause Linker
	siso_pause(siso_video_rtsp_v2);

	//Stop module
	mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_STREAM_STOP, 0);

	//Delete linker
	siso_delete(siso_video_rtsp_v2);

	//Close module
	rtsp2_v2_ctx = mm_module_close(rtsp2_v2_ctx);
	video_v2_ctx = mm_module_close(video_v2_ctx);
}
static void mmf2_bayercap_init(void)
{
	filesaver_ctx = mm_module_open(&filesaver_module);

	if (filesaver_ctx) {
		sprintf(savefilepath, "raw_img_%02d.raw", file_count++);
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_SET_SAVE_FILE_PATH, (int)savefilepath);
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_SET_TYPE_HANDLER, (int)file_save);
	} else {
		rt_printf("filesaver open fail\n\r");
		goto mmf2_video_exmaple_bayercap_fail;
	}


	video_bayercap_ctx = mm_module_open(&video_module);
	if (video_bayercap_ctx) {
		//mm_module_ctrl(video_bayercap_ctx, CMD_VIDEO_SET_VOE_HEAP, voe_heap_size);
		mm_module_ctrl(video_bayercap_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_bayercap_params);
		mm_module_ctrl(video_bayercap_ctx, MM_CMD_SET_QUEUE_LEN, 1);//Default 30
		mm_module_ctrl(video_bayercap_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_bayercap_ctx, CMD_VIDEO_APPLY, V1_RAW_CHANNEL);	// start channel 0
		mm_module_ctrl(video_bayercap_ctx, CMD_VIDEO_YUV, 2);
		hal_video_isp_set_rawfmt(0, 1);
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_exmaple_bayercap_fail;
	}

	//vTaskDelay(2000);

	siso_array_filesaver = siso_create();
	if (siso_array_filesaver) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_array_filesaver, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_array_filesaver, MMIC_CMD_ADD_INPUT, (uint32_t)video_bayercap_ctx, 0);
		siso_ctrl(siso_array_filesaver, MMIC_CMD_ADD_OUTPUT, (uint32_t)filesaver_ctx, 0);
		siso_start(siso_array_filesaver);
	} else {
		rt_printf("siso_array_filesaver open fail\n\r");
		goto mmf2_video_exmaple_bayercap_fail;
	}

mmf2_video_exmaple_bayercap_fail:

	return;
}
static void mmf2_bayercap_deinit(void)
{
	//Pause Linker
	siso_pause(siso_array_filesaver);

	//Stop module
	mm_module_ctrl(video_bayercap_ctx, CMD_VIDEO_STREAM_STOP, 0);

	//Delete linker
	siso_delete(siso_array_filesaver);

	//Close module
	video_bayercap_ctx = mm_module_close(video_bayercap_ctx);
	filesaver_ctx = mm_module_close(filesaver_ctx);
}
void mmf2_video_example_bayercap_rtsp_init(void)
{

	fatfs_sd_init();
	int voe_heap_size = video_voe_presetting(1, RTSP_WIDTH, RTSP_HEIGHT, RTSP_BPS, 0,
						1, sensor_params[USE_SENSOR].sensor_width, sensor_params[USE_SENSOR].sensor_height, V1_RAW_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);

	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	mmf2_rtsp_init();
	mmf2_bayercap_init();

	atcmd_userctrl_init();
	return;
}

static const char *example = "mmf2_video_example_bayercap_rtsp_init";
static void example_deinit(void)
{
	mmf2_rtsp_deinit();
	mmf2_bayercap_deinit();
	video_voe_release();
}

static void fUC(void *arg)
{
	static uint32_t user_cmd = 0;

	if (!strcmp(arg, "TD")) {
		if (user_cmd & USR_CMD_EXAMPLE_DEINIT) {
			printf("invalid state, can not do %s deinit!\r\n", example);
		} else {
			mmf2_rtsp_deinit();
			mmf2_bayercap_deinit();
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