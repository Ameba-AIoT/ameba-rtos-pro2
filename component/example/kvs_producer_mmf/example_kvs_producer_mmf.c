/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "platform_opts.h"
#include "log_service.h"

#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "module_video.h"
#include "module_audio.h"
#include "module_aac.h"
#include "module_g711.h"
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
#define V1_RCMODE 2 // 1: CBR, 2: VBR

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

#define KVS_PRODUCER_CHANGE_RES_TEST 0

static mm_context_t *video_v1_ctx           = NULL;
static mm_context_t *audio_ctx				= NULL;
static mm_context_t *aac_ctx				= NULL;
static mm_context_t *kvs_producer_v1_ctx    = NULL;
static mm_siso_t *siso_audio_aac			= NULL;
static mm_siso_t *siso_video_kvs_v1         = NULL;
static mm_miso_t *miso_video_aac_kvs_v1_a1  = NULL;

static mm_context_t *g711e_ctx				= NULL;
static mm_siso_t *siso_audio_g711e			= NULL;
static mm_miso_t *miso_video_g711e_kvs_v1_a1  = NULL;

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

#if ENABLE_AUDIO_TRACK
#if !USE_DEFAULT_AUDIO_SET
static audio_params_t audio_params = {
	.sample_rate = ASR_8KHZ,
	.word_length = WL_16BIT,
	.mic_gain    = MIC_40DB,
	.channel     = 1,
	.enable_aec  = 0
};
#endif
#if USE_AUDIO_AAC
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
#endif
#if USE_AUDIO_G711
static g711_params_t g711e_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_ENCODE
};
#endif
#endif

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

static void change_resolution_parameter(int parm_index)
{
	if (parm_index == 0) {
		video_v1_params.resolution = VIDEO_FHD;
		video_v1_params.width = 1920;
		video_v1_params.height = 1080;
	} else {
		video_v1_params.resolution = VIDEO_HD;
		video_v1_params.width = 1280;
		video_v1_params.height = 720;
	}
}

static void atcmd_kvs_producer_init(void);

void example_kvs_producer_mmf_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif

	atcmd_kvs_producer_init();

	if (!voe_boot_fsc_status()) {
		wifi_common_init();
	}

	int voe_heap_size = video_voe_presetting(1, V1_WIDTH, V1_HEIGHT, V1_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);

	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, 10);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
	} else {
		rt_printf("video open fail\n\r");
		goto example_kvs_producer_mmf;
	}

#if ENABLE_AUDIO_TRACK
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
#if USE_AUDIO_AAC
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
#endif
#if USE_AUDIO_G711
	g711e_ctx = mm_module_open(&g711_module);
	if (g711e_ctx) {
		mm_module_ctrl(g711e_ctx, CMD_G711_SET_PARAMS, (int)&g711e_params);
		mm_module_ctrl(g711e_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711e_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711e_ctx, CMD_G711_APPLY, 0);
	} else {
		rt_printf("G711 open fail\n\r");
		goto example_kvs_producer_mmf;
	}
#endif

#if USE_AUDIO_AAC
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
#endif
#if USE_AUDIO_G711
	siso_audio_g711e = siso_create();
	if (siso_audio_g711e) {
		siso_ctrl(siso_audio_g711e, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_g711e, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711e_ctx, 0);
		siso_start(siso_audio_g711e);
	} else {
		rt_printf("siso_audio_g711e open fail\n\r");
		goto example_kvs_producer_mmf;
	}
#endif
#endif

	kvs_producer_v1_ctx = mm_module_open(&kvs_producer_module);
	if (kvs_producer_v1_ctx) {
		mm_module_ctrl(kvs_producer_v1_ctx, CMD_KVS_PRODUCER_SET_APPLY, 0);
	} else {
		rt_printf("KVS open fail\n\r");
		goto example_kvs_producer_mmf;
	}

#if ENABLE_AUDIO_TRACK
#if USE_AUDIO_AAC
	miso_video_aac_kvs_v1_a1 = miso_create();
	if (miso_video_aac_kvs_v1_a1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		miso_ctrl(miso_video_aac_kvs_v1_a1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		miso_ctrl(miso_video_aac_kvs_v1_a1, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
		miso_ctrl(miso_video_aac_kvs_v1_a1, MMIC_CMD_ADD_INPUT1, (uint32_t)aac_ctx, 0);
		miso_ctrl(miso_video_aac_kvs_v1_a1, MMIC_CMD_ADD_OUTPUT, (uint32_t)kvs_producer_v1_ctx, 0);
		miso_start(miso_video_aac_kvs_v1_a1);
	} else {
		rt_printf("miso_video_aac_kvs_v1_a1 open fail\n\r");
		goto example_kvs_producer_mmf;
	}
	rt_printf("miso started\n\r");
#endif
#if USE_AUDIO_G711
	miso_video_g711e_kvs_v1_a1 = miso_create();
	if (miso_video_g711e_kvs_v1_a1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		miso_ctrl(miso_video_g711e_kvs_v1_a1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		miso_ctrl(miso_video_g711e_kvs_v1_a1, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
		miso_ctrl(miso_video_g711e_kvs_v1_a1, MMIC_CMD_ADD_INPUT1, (uint32_t)g711e_ctx, 0);
		miso_ctrl(miso_video_g711e_kvs_v1_a1, MMIC_CMD_ADD_OUTPUT, (uint32_t)kvs_producer_v1_ctx, 0);
		miso_start(miso_video_g711e_kvs_v1_a1);
	} else {
		rt_printf("miso_video_g711e_kvs_v1_a1 open fail\n\r");
		goto example_kvs_producer_mmf;
	}
	rt_printf("miso started\n\r");
#endif
#else
	siso_video_kvs_v1 = siso_create();
	if (siso_video_kvs_v1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_kvs_v1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_kvs_v1, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_video_kvs_v1, MMIC_CMD_ADD_OUTPUT, (uint32_t)kvs_producer_v1_ctx, 0);
		siso_start(siso_video_kvs_v1);
	} else {
		rt_printf("siso2 open fail\n\r");
		goto example_kvs_producer_mmf;
	}
	rt_printf("siso_video_kvs_v1 started\n\r");
#endif


	// changing resolution test
#if (ENABLE_AUDIO_TRACK && KVS_PRODUCER_CHANGE_RES_TEST)
	int sw = 0;
	rt_printf("KVS producer changing resolution test\n\r");
	for (int i = 0; i < 10; i++) {
		sw++;
		sw &= 1;
		// wait 30 seconds, change resolution
		vTaskDelay(30000);
		rt_printf("kvs_producer_v1_ctx CMD_KVS_PRODUCER_PAUSE.\n\r");
		mm_module_ctrl(kvs_producer_v1_ctx, CMD_KVS_PRODUCER_PAUSE, 0);
		vTaskDelay(500);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
		change_resolution_parameter(sw);

		miso_pause(miso_video_aac_kvs_v1_a1, MM_OUTPUT);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
		miso_resume(miso_video_aac_kvs_v1_a1);

		rt_printf("kvs_producer_v1_ctx CMD_KVS_PRODUCER_RECONNECT.\n\r");
		mm_module_ctrl(kvs_producer_v1_ctx, CMD_KVS_PRODUCER_RECONNECT, 0);
	}
#endif

example_kvs_producer_mmf:

	vTaskDelete(NULL);
}

static void fKVSC(void *arg)
{
	uint32_t t0 = xTaskGetTickCount();
	//Pause Linker
#if ENABLE_AUDIO_TRACK
#if USE_AUDIO_AAC
	miso_pause(miso_video_aac_kvs_v1_a1, MM_OUTPUT);
#elif USE_AUDIO_G711
	miso_pause(miso_video_g711e_kvs_v1_a1, MM_OUTPUT);
#endif
#else
	siso_pause(siso_video_kvs_v1);
#endif

	//Stop module
	mm_module_ctrl(kvs_producer_v1_ctx, CMD_KVS_PRODUCER_PAUSE, 0);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
#if ENABLE_AUDIO_TRACK
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_TRX, 0);
#if USE_AUDIO_AAC
	mm_module_ctrl(aac_ctx, CMD_AAC_STOP, 0);
#endif
#endif

	//Delete linker
#if ENABLE_AUDIO_TRACK
#if USE_AUDIO_AAC
	miso_delete(miso_video_aac_kvs_v1_a1);
#elif USE_AUDIO_G711
	miso_delete(miso_video_g711e_kvs_v1_a1);
#endif
#else
	siso_delete(siso_video_kvs_v1);
#endif
#if USE_AUDIO_AAC
	siso_delete(siso_audio_aac);
#elif USE_AUDIO_G711
	siso_delete(siso_audio_g711e);
#endif

	//Close module
	mm_module_close(video_v1_ctx);
#if ENABLE_AUDIO_TRACK
	mm_module_close(audio_ctx);
#if USE_AUDIO_AAC
	mm_module_close(aac_ctx);
#elif USE_AUDIO_G711
	mm_module_close(g711e_ctx);
#endif
#endif
	mm_module_close(kvs_producer_v1_ctx);

	video_voe_release();
	printf(">>>>>> Deinit take %d ms \r\n", xTaskGetTickCount() - t0);
}

static log_item_t kvs_items[] = {
	{"KVSC", fKVSC,},
};

static void atcmd_kvs_producer_init(void)
{
	log_service_add_table(kvs_items, sizeof(kvs_items) / sizeof(kvs_items[0]));
}

void example_kvs_producer_mmf(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(example_kvs_producer_mmf_thread, ((const char *)"example_kvs_producer_mmf_thread"), 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n example_kvs_producer_mmf_thread: Create Task Error\n");
	}
}