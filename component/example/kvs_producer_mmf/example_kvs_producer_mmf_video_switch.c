/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "platform_opts.h"
#include "log_service.h"

#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_mimo.h"

#include "module_video.h"
#include "module_audio.h"
#include "module_aac.h"
#include "module_g711.h"
#include "module_rtsp2.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"

#include "example_kvs_producer_mmf.h"
#include "module_kvs_producer.h"
#include "sample_config.h"

#include "avcodec.h"

/*****************************************************************************
* ISP channel : 0
* Video type  : H264/HEVC
*****************************************************************************/

#define V1_CHANNEL 0
#define V1_RESOLUTION VIDEO_FHD
#define V1_FPS 15
#define V1_GOP 15
#define V1_BPS 1024*1024
#define V1_RCMODE 1 // 1: CBR, 2: VBR

#define V2_CHANNEL 1
#define V2_RESOLUTION VIDEO_HD
#define V2_FPS 15
#define V2_GOP 15
#define V2_BPS 1024*1024
#define V2_RCMODE 1 // 1: CBR, 2: VBR

#define USE_H265 0

#if USE_H265
#define VIDEO_TYPE VIDEO_HEVC
#define VIDEO_CODEC AV_CODEC_ID_H265
#else
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
#endif

#if V2_RESOLUTION == VIDEO_VGA
#define V2_WIDTH    640
#define V2_HEIGHT   480
#elif V2_RESOLUTION == VIDEO_HD
#define V2_WIDTH    1280
#define V2_HEIGHT   720
#elif V2_RESOLUTION == VIDEO_FHD
#define V2_WIDTH    1920
#define V2_HEIGHT   1080
#endif

#define KVS_PRODUCER_CHANGE_RES_TEST 1  /* set 1 to enable resolution change test by ATCMD */

static mm_context_t *video_v1_ctx           = NULL;
static mm_context_t *video_v2_ctx           = NULL;
static mm_context_t *audio_ctx				= NULL;
static mm_context_t *aac_ctx				= NULL;
static mm_context_t *kvs_producer_v1_ctx    = NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;

static mm_siso_t *siso_audio_aac			= NULL;
static mm_mimo_t *mimo_1v1a_aac_producer    = NULL;

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = V1_RESOLUTION,
	.width = V1_WIDTH,
	.height = V1_HEIGHT,
	.bps = V1_BPS,
	.fps = V1_FPS,
	.gop = V1_GOP,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1
};

static video_params_t video_v2_params = {
	.stream_id = V2_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = V2_RESOLUTION,
	.width = V2_WIDTH,
	.height = V2_HEIGHT,
	.bps = V2_BPS,
	.fps = V2_FPS,
	.gop = V2_GOP,
	.rc_mode = V2_RCMODE,
	.use_static_addr = 1
};

#if !USE_DEFAULT_AUDIO_SET
static audio_params_t audio_params = {
	.sample_rate = ASR_8KHZ,
	.word_length = WL_16BIT,
	.mic_gain    = MIC_40DB,
	.channel     = 1,
	.enable_aec  = 0
};
#endif

static aac_params_t aac_params = {
	.sample_rate = 8000,
	.channel = 1,
	.trans_type = AAC_TYPE_RAW,  //kvs producer require aac "RAW" type
	.object_type = AAC_AOT_LC,
	.bitrate = 32000,

	.mem_total_size = 10 * 1024,
	.mem_block_size = 128,
	.mem_frame_size = 1024
};

static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.fps      = V1_FPS,
			.bps      = V1_BPS
		}
	}
};

#include "wifi_conf.h"
#include "lwip_netconf.h"
#define wifi_wait_time 500 //Here we wait 5 second to wiat the fast connect 
static void wifi_common_init(void)
{
	uint32_t wifi_wait_count = 0;

	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(10);
		wifi_wait_count++;
		if (wifi_wait_count == wifi_wait_time) {
			printf("\r\nuse ATW0, ATW1, ATWC to make wifi connection\r\n");
			printf("wait for wifi connection...\r\n");
		}
	}
}

void atcmd_kvs_init(void);

void example_kvs_producer_mmf_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif

	if (!voe_boot_fsc_status()) {
		wifi_common_init();
	}

#if KVS_PRODUCER_CHANGE_RES_TEST
	atcmd_kvs_init();
#endif

	int voe_heap_size = video_voe_presetting(1, V1_WIDTH, V1_HEIGHT, V1_BPS, 0,
						1, V2_WIDTH, V2_HEIGHT, V2_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);

	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	// ------ Channel 1--------------
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, V1_FPS);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
	} else {
		rt_printf("video1 open fail\n\r");
		goto example_kvs_producer_mmf;
	}

	// ------ Channel 2--------------
	video_v2_ctx = mm_module_open(&video_module);
	if (video_v2_ctx) {
		mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v2_params);
		mm_module_ctrl(video_v2_ctx, MM_CMD_SET_QUEUE_LEN, V2_FPS);
		mm_module_ctrl(video_v2_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v2_ctx, CMD_VIDEO_APPLY, V2_CHANNEL);
	} else {
		rt_printf("video2 open fail\n\r");
		goto example_kvs_producer_mmf;
	}

	//-------- Audio --------------
	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
#if !USE_DEFAULT_AUDIO_SET
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
#endif
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		rt_printf("AUDIO open fail\n\r");
		goto example_kvs_producer_mmf;
	}

	aac_ctx = mm_module_open(&aac_module);
	if (aac_ctx) {
		mm_module_ctrl(aac_ctx, CMD_AAC_SET_PARAMS, (int)&aac_params);
		mm_module_ctrl(aac_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(aac_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(aac_ctx, CMD_AAC_INIT_MEM_POOL, 0);
		mm_module_ctrl(aac_ctx, CMD_AAC_APPLY, 0);
	} else {
		rt_printf("AAC open fail\n\r");
		goto example_kvs_producer_mmf;
	}

	siso_audio_aac = siso_create();
	if (siso_audio_aac) {
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_OUTPUT, (uint32_t)aac_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(siso_audio_aac);
	} else {
		rt_printf("siso_audio_aac open fail\n\r");
		goto example_kvs_producer_mmf;
	}

	kvs_producer_v1_ctx = mm_module_open(&kvs_producer_module);
	if (kvs_producer_v1_ctx) {
		mm_module_ctrl(kvs_producer_v1_ctx, CMD_KVS_PRODUCER_SET_APPLY, 0);
	} else {
		rt_printf("KVS open fail\n\r");
		goto example_kvs_producer_mmf;
	}

	//--------------RTSP---------------
	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto example_kvs_producer_mmf;
	}

	mimo_1v1a_aac_producer = mimo_create();
	if (mimo_1v1a_aac_producer) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		mimo_ctrl(mimo_1v1a_aac_producer, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		mimo_ctrl(mimo_1v1a_aac_producer, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
		mimo_ctrl(mimo_1v1a_aac_producer, MMIC_CMD_ADD_INPUT1, (uint32_t)video_v2_ctx, 0);
		mimo_ctrl(mimo_1v1a_aac_producer, MMIC_CMD_ADD_INPUT2, (uint32_t)aac_ctx, 0);
		mimo_ctrl(mimo_1v1a_aac_producer, MMIC_CMD_ADD_OUTPUT0, (uint32_t)rtsp2_v1_ctx, MMIC_DEP_INPUT0);
		mimo_ctrl(mimo_1v1a_aac_producer, MMIC_CMD_ADD_OUTPUT1, (uint32_t)kvs_producer_v1_ctx, MMIC_DEP_INPUT0 | MMIC_DEP_INPUT2);
		mimo_ctrl(mimo_1v1a_aac_producer, MMIC_CMD_ADD_OUTPUT2, (uint32_t)kvs_producer_v1_ctx, MMIC_DEP_INPUT1 | MMIC_DEP_INPUT2);
		mimo_start(mimo_1v1a_aac_producer);

		rt_printf("mimo paused video 720p to producer...\n\r");
		mimo_pause(mimo_1v1a_aac_producer, MM_OUTPUT2);
		rt_printf("video_mmf_profile 2 (1080P)...\n\r");
	} else {
		rt_printf("mimo open fail\n\r");
		goto example_kvs_producer_mmf;
	}
	rt_printf("mimo started\n\r");

example_kvs_producer_mmf:

	// TODO: exit condition or signal
	while (1) {
		vTaskDelay(1000);
	}
}

#if KVS_PRODUCER_CHANGE_RES_TEST
// changing resolution test
static int kvs_change_res_flag = 0;
void command_waiting_task(void *param)
{
	int sw = 0;
	rt_printf("KVS producer changing resolution test\n\r");
	while (1) {
		if (kvs_change_res_flag) {
			sw++;
			sw &= 1;
			rt_printf("kvs_producer_v1_ctx CMD_KVS_PRODUCER_PAUSE.\n\r");
			mm_module_ctrl(kvs_producer_v1_ctx, CMD_KVS_PRODUCER_PAUSE, 0);
			vTaskDelay(500);
			mimo_resume(mimo_1v1a_aac_producer);
			mimo_pause(mimo_1v1a_aac_producer, sw ? MM_OUTPUT1 : MM_OUTPUT2);
			rt_printf("video_mmf_profile 1 (%s)...\n\r", sw ? "720P" : "1080P");
			rt_printf("kvs_producer_v1_ctx CMD_KVS_PRODUCER_RECONNECT.\n\r");
			mm_module_ctrl(kvs_producer_v1_ctx, CMD_KVS_PRODUCER_RECONNECT, 0);

			kvs_change_res_flag = 0;
		}
		vTaskDelay(500);
	}
}

void fKVSC(void *arg)
{
	kvs_change_res_flag = 1;
}

log_item_t kvs_items[] = {
	{"KVSC", fKVSC,},
};

void atcmd_kvs_init(void)
{
	log_service_add_table(kvs_items, sizeof(kvs_items) / sizeof(kvs_items[0]));
}
#endif

void example_kvs_producer_mmf(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(example_kvs_producer_mmf_thread, ((const char *)"example_kvs_producer_mmf_thread"), 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n example_kvs_producer_mmf_thread: Create Task Error\n");
	}
#if KVS_PRODUCER_CHANGE_RES_TEST
	if (xTaskCreate(command_waiting_task, ((const char *)"command_waiting_task"), 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n command_waiting_task: Create Task Error\n");
	}
#endif
}
