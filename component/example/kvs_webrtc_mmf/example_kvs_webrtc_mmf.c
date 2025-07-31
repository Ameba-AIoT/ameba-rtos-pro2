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

#include "avcodec.h"
#include "module_video.h"
#include "module_audio.h"
#include "module_g711.h"
#include "module_opusc.h"
#include "module_opusd.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"

#include "example_kvs_webrtc_mmf.h"
#include "module_kvs_webrtc.h"
#include "sample_config_webrtc.h"

/*****************************************************************************
* ISP channel : 0
* Video type  : H264/HEVC
*****************************************************************************/

#define V1_CHANNEL 0
#define V1_RESOLUTION VIDEO_HD
#define V1_FPS 30
#define V1_GOP 5
#define V1_BPS 512*1024
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

static mm_context_t *video_v1_ctx           = NULL;
static mm_context_t *audio_ctx              = NULL;
static mm_context_t *g711e_ctx              = NULL;
static mm_context_t *g711d_ctx              = NULL;
static mm_context_t *opusc_ctx		        = NULL;
static mm_context_t *opusd_ctx		        = NULL;
static mm_context_t *kvs_webrtc_v1_a1_ctx   = NULL;

static mm_siso_t *siso_audio_a1             = NULL;
static mm_siso_t *siso_webrtc_a2            = NULL;
static mm_siso_t *siso_a2_audio             = NULL;
static mm_miso_t *miso_kvs_webrtc_v1_a1     = NULL;

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

#if !USE_DEFAULT_AUDIO_SET
static audio_params_t audio_params = {
	.sample_rate = ASR_8KHZ,
	.word_length = WL_16BIT,
	.mic_gain    = MIC_0DB,
	.dmic_l_gain    = DMIC_BOOST_24DB,
	.dmic_r_gain    = DMIC_BOOST_24DB,
	.use_mic_type   = USE_AUDIO_AMIC,
	.channel     = 1,
	.mix_mode = 0,
	.enable_aec  = 0
};
#endif

static g711_params_t g711e_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_ENCODE
};

static g711_params_t g711d_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_DECODE
};

static opusc_params_t opusc_params = {
	.sample_rate = 8000,  //16000
	.channel = 1,
	.bit_length = 16,     // 16 recommand
	.complexity = 5,      // 0~10
	.bitrate = 25000,     // default 25000
	.use_framesize = 20,  // 10 // needs to the same or bigger than AUDIO_DMA_PAGE_SIZE/(sample_rate/1000)/2 but less than 60
	.enable_vbr = 1,
	.vbr_constraint = 0,
	.packetLossPercentage = 0,
	.opus_application = OPUS_APPLICATION_AUDIO

};

static opusd_params_t opusd_only_params = {
	.sample_rate = 8000,  //16000
	.channel = 1,
	.bit_length = 16,         // 16 recommand
	.frame_size_in_msec = 10, // will not be uused
	.with_opus_enc = 1,       // enable semaphore if the application with opus encoder
	.opus_application = OPUS_APPLICATION_AUDIO
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

static void atcmd_kvsWebrtc_init(void);
static SemaphoreHandle_t kvsWebrtc_stop_sem;    // wait for webrtc termination

void example_kvs_webrtc_mmf_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif

	atcmd_kvsWebrtc_init();
	kvsWebrtc_stop_sem = xSemaphoreCreateBinary();

	if (!voe_boot_fsc_status()) {
		wifi_common_init();
	}
	sntp_init();

	kvs_webrtc_v1_a1_ctx = mm_module_open(&kvs_webrtc_module);
	if (kvs_webrtc_v1_a1_ctx) {
		mm_module_ctrl(kvs_webrtc_v1_a1_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(kvs_webrtc_v1_a1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(kvs_webrtc_v1_a1_ctx, CMD_KVS_WEBRTC_SET_APPLY, 0);
	} else {
		printf("KVS open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	int voe_heap_size = video_voe_presetting(1, V1_WIDTH, V1_HEIGHT, V1_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);

	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, V1_FPS * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
	} else {
		printf("video open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
#if !USE_DEFAULT_AUDIO_SET
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
#endif
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		printf("AUDIO open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

#if (AUDIO_G711_MULAW || AUDIO_G711_ALAW)
	g711e_ctx = mm_module_open(&g711_module);
	if (g711e_ctx) {
		mm_module_ctrl(g711e_ctx, CMD_G711_SET_PARAMS, (int)&g711e_params);
		mm_module_ctrl(g711e_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711e_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711e_ctx, CMD_G711_APPLY, 0);
	} else {
		printf("G711 open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}
#elif AUDIO_OPUS
	opusc_ctx = mm_module_open(&opusc_module);
	if (opusc_ctx) {
		mm_module_ctrl(opusc_ctx, CMD_OPUSC_SET_PARAMS, (int)&opusc_params);
		mm_module_ctrl(opusc_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(opusc_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(opusc_ctx, CMD_OPUSC_APPLY, 0);
	} else {
		printf("OPUSC open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}
#endif

	siso_audio_a1 = siso_create();
	if (siso_audio_a1) {
		siso_ctrl(siso_audio_a1, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
#if (AUDIO_G711_MULAW || AUDIO_G711_ALAW)
		siso_ctrl(siso_audio_a1, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711e_ctx, 0);
#elif AUDIO_OPUS
		siso_ctrl(siso_audio_a1, MMIC_CMD_ADD_OUTPUT, (uint32_t)opusc_ctx, 0);
		siso_ctrl(siso_audio_a1, MMIC_CMD_SET_STACKSIZE, 24 * 1024, 0);
#endif
		siso_start(siso_audio_a1);
	} else {
		printf("siso_audio_a1 open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	miso_kvs_webrtc_v1_a1 = miso_create();
	if (miso_kvs_webrtc_v1_a1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		miso_ctrl(miso_kvs_webrtc_v1_a1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		miso_ctrl(miso_kvs_webrtc_v1_a1, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
#if (AUDIO_G711_MULAW || AUDIO_G711_ALAW)
		miso_ctrl(miso_kvs_webrtc_v1_a1, MMIC_CMD_ADD_INPUT1, (uint32_t)g711e_ctx, 0);
#elif AUDIO_OPUS
		miso_ctrl(miso_kvs_webrtc_v1_a1, MMIC_CMD_ADD_INPUT1, (uint32_t)opusc_ctx, 0);
#endif
		miso_ctrl(miso_kvs_webrtc_v1_a1, MMIC_CMD_ADD_OUTPUT, (uint32_t)kvs_webrtc_v1_a1_ctx, 0);
		miso_start(miso_kvs_webrtc_v1_a1);
	} else {
		printf("miso_kvs_webrtc_v1_a1 open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}
	printf("miso_kvs_webrtc_v1_a1 started\n\r");

#ifdef ENABLE_AUDIO_SENDRECV
#if (AUDIO_G711_MULAW || AUDIO_G711_ALAW)
	g711d_ctx = mm_module_open(&g711_module);
	if (g711d_ctx) {
		mm_module_ctrl(g711d_ctx, CMD_G711_SET_PARAMS, (int)&g711d_params);
		mm_module_ctrl(g711d_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711d_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711d_ctx, CMD_G711_APPLY, 0);
	} else {
		printf("G711 open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}
#elif AUDIO_OPUS
	opusd_ctx = mm_module_open(&opusd_module);
	if (opusd_ctx) {
		mm_module_ctrl(opusd_ctx, CMD_OPUSD_SET_PARAMS, (int)&opusd_only_params);
		mm_module_ctrl(opusd_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(opusd_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(opusd_ctx, CMD_OPUSD_APPLY, 0);
	} else {
		printf("OPUSD open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}
#endif

	siso_webrtc_a2 = siso_create();
	if (siso_webrtc_a2) {
		siso_ctrl(siso_webrtc_a2, MMIC_CMD_ADD_INPUT, (uint32_t)kvs_webrtc_v1_a1_ctx, 0);
#if (AUDIO_G711_MULAW || AUDIO_G711_ALAW)
		siso_ctrl(siso_webrtc_a2, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711d_ctx, 0);
#elif AUDIO_OPUS
		siso_ctrl(siso_webrtc_a2, MMIC_CMD_ADD_OUTPUT, (uint32_t)opusd_ctx, 0);
		siso_ctrl(siso_webrtc_a2, MMIC_CMD_SET_STACKSIZE, 24 * 1024, 0);
#endif
		siso_start(siso_webrtc_a2);
	} else {
		printf("siso_webrtc_a2 open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	siso_a2_audio = siso_create();
	if (siso_a2_audio) {
#if (AUDIO_G711_MULAW || AUDIO_G711_ALAW)
		siso_ctrl(siso_a2_audio, MMIC_CMD_ADD_INPUT, (uint32_t)g711d_ctx, 0);
#elif AUDIO_OPUS
		siso_ctrl(siso_a2_audio, MMIC_CMD_ADD_INPUT, (uint32_t)opusd_ctx, 0);
#endif
		siso_ctrl(siso_a2_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
		siso_start(siso_a2_audio);
	} else {
		printf("siso_a2_audio open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}
#endif

	//wait for termination
	xSemaphoreTake(kvsWebrtc_stop_sem, portMAX_DELAY);

example_kvs_webrtc_cleanup:

	//Pause Linker
	siso_pause(siso_audio_a1);
	miso_pause(miso_kvs_webrtc_v1_a1, MM_OUTPUT);
	siso_pause(siso_webrtc_a2);
	siso_pause(siso_a2_audio);

	//Stop module
	mm_module_ctrl(kvs_webrtc_v1_a1_ctx, CMD_KVS_WEBRTC_STOP, 0);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_TRX, 0);

	//Delete linker
	siso_audio_a1 = siso_delete(siso_audio_a1);
	miso_kvs_webrtc_v1_a1 = miso_delete(miso_kvs_webrtc_v1_a1);
	siso_webrtc_a2 = siso_delete(siso_webrtc_a2);
	siso_a2_audio = siso_delete(siso_a2_audio);

	//Close module
	kvs_webrtc_v1_a1_ctx = mm_module_close(kvs_webrtc_v1_a1_ctx);
	video_v1_ctx = mm_module_close(video_v1_ctx);
	audio_ctx = mm_module_close(audio_ctx);
#if (AUDIO_G711_MULAW || AUDIO_G711_ALAW)
	g711e_ctx = mm_module_close(g711e_ctx);
	g711d_ctx = mm_module_close(g711d_ctx);
#elif AUDIO_OPUS
	opusc_ctx = mm_module_close(opusc_ctx);
	opusd_ctx = mm_module_close(opusd_ctx);
#endif

	//Video Deinit
	video_deinit();

	vSemaphoreDelete(kvsWebrtc_stop_sem);
	kvsWebrtc_stop_sem = NULL;
	vTaskDelete(NULL);
}

/* KVS termination */
static void fKVST(void *arg)
{
	if (kvsWebrtc_stop_sem != NULL) {
		xSemaphoreGive(kvsWebrtc_stop_sem);
	}
}

static log_item_t kvsWebrtc_items[] = {
	{"KVST", fKVST,},
};

static void atcmd_kvsWebrtc_init(void)
{
	log_service_add_table(kvsWebrtc_items, sizeof(kvsWebrtc_items) / sizeof(kvsWebrtc_items[0]));
}

void example_kvs_webrtc_mmf(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(example_kvs_webrtc_mmf_thread, ((const char *)"example_kvs_webrtc_mmf_thread"), 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n example_kvs_webrtc_mmf_thread: Create Task Error\n");
	}
}