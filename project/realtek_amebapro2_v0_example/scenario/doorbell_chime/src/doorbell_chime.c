/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"
#include "module_video.h"
#include "module_rtsp2.h"
#include "module_audio.h"
#include "module_aac.h"
#include "module_p2p.h"
#include "module_p2p_aac.h"
#include "module_g711.h"
#include "module_rtp.h"
#include "module_array.h"
#include "sample_doorbell_pcmu.h"
#include "mmf2_pro2_video_config.h"
#include "doorbell_chime.h"
#include "log_service.h"
#include "doorbell_demo.h"

/*****************************************************************************
* ISP channel : 0
* Video type  : H264/HEVC
*****************************************************************************/

#define V1_CHANNEL 0
#define V1_RESOLUTION VIDEO_FHD
#define V1_FPS 30
#define V1_GOP 30
#define V1_BPS 2*1024*1024
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

mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *audio_ctx				= NULL;
static mm_context_t *aac_ctx				= NULL;
static mm_context_t *p2p_ctx				= NULL;
static mm_context_t *g711e_ctx				= NULL;
static mm_context_t *g711d_ctx				= NULL;
static mm_context_t *rtp_ctx				= NULL;
static mm_context_t *array_ctx            = NULL;

static mm_siso_t *siso_audio_aac			= NULL;
static mm_miso_t *miso_video_aac_p2p		= NULL;
static mm_siso_t *siso_array_g711d			= NULL;
static mm_siso_t *siso_g711d_audio			= NULL;

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

static audio_params_t audio_params = {
	.sample_rate = ASR_8KHZ,
	.word_length = WL_16BIT,
	.mic_gain    = MIC_40DB,
	.channel     = 1,
};

static aac_params_t aac_params = {
	.sample_rate = 8000,
	.channel = 1,
	.trans_type = AAC_TYPE_ADTS,
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

static rtsp2_params_t rtsp2_a_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a = {
			.codec_id   = AV_CODEC_ID_MP4A_LATM,
			.channel    = 1,
			.samplerate = 8000
		}
	}
};

static g711_params_t g711e_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_ENCODE
};

static rtp_params_t rtp_g711d_params = {
	.valid_pt = 0xFFFFFFFF,
	.port = 16384,
	.frame_size = 1500,
	.cache_depth = 6
};

static g711_params_t g711d_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_DECODE
};

static array_params_t pcmu_array_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.codec_id = AV_CODEC_ID_PCMU,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.a = {
			.channel    = 1,
			.samplerate = 8000,
			.frame_size = 160,
		}
	}
};

#define wifi_wait_time 500 //Here we wait 5 second to wiat the fast connect 
//------------------------------------------------------------------------------
// common code for network connection
//------------------------------------------------------------------------------
#include "wifi_conf.h"
#include "lwip_netconf.h"

static void wifi_common_init()
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

void start_doorbell_ring(void)
{
	int state = 0;
	int timeout = 0;
	mm_module_ctrl(array_ctx, CMD_ARRAY_GET_STATE, (int)&state);
	if (state) {
		printf("doorbell is ringing\n\r");
	} else {
#if DOORBELL_AMP_ENABLE
		amp_gpio_enable(1);
#endif

		printf("start doorbell_ring\n\r");
		siso_resume(siso_array_g711d);

		mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 1);	// doorbell ring
		timeout = 0;
		do {	// wait until doorbell_ring done
			timeout++;
			vTaskDelay(100);

			mm_module_ctrl(array_ctx, CMD_ARRAY_GET_STATE, (int)&state);
			if (timeout > 100) {
				mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 0);
				break;
			}
		} while (state == 1);

		siso_pause(siso_array_g711d);

		printf("doorbell_ring done!\n\r");
#if DOORBELL_AMP_ENABLE
		amp_gpio_enable(0);
#endif

		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_RESET, 0);
	}
}

void mmf2_video_example_p2p_av_init(void)
{
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
		goto mmf2_video_exmaple_av_fail;
	}

	encode_rc_parm_t rc_parm;
	rc_parm.minQp = 28;
	rc_parm.maxQp = 45;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_RCPARAM, (int)&rc_parm);

	HAL_WRITE32(0x40300000, 0xc0f8, 0x5);

#if 0
	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		rt_printf("AUDIO open fail\n\r");
		goto mmf2_video_exmaple_av_fail;
	}

	aac_ctx = mm_module_open(&p2p_aac_module);
	if (aac_ctx) {
		mm_module_ctrl(aac_ctx, CMD_P2P_AAC_SET_PARAMS, (int)&aac_params);
		mm_module_ctrl(aac_ctx, MM_CMD_SET_QUEUE_LEN, 16);
		mm_module_ctrl(aac_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(aac_ctx, CMD_P2P_AAC_INIT_MEM_POOL, 0);
		mm_module_ctrl(aac_ctx, CMD_P2P_AAC_APPLY, 0);
	} else {
		rt_printf("AAC open fail\n\r");
		goto mmf2_video_exmaple_av_fail;
	}

	rt_printf("aac_ctx\n");

#endif
	//--------------P2P---------------

	p2p_ctx = mm_module_open(&p2p_module);
	if (p2p_ctx) {

	} else {
		rt_printf("p2p open fail\n\r");
		goto mmf2_video_exmaple_av_fail;
	}

	rt_printf("p2p_ctx\n");

#if 0
	siso_audio_aac = siso_create();
	if (siso_audio_aac) {
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_OUTPUT, (uint32_t)aac_ctx, 0);
		siso_start(siso_audio_aac);
	} else {
		rt_printf("siso1 open fail\n\r");
		goto mmf2_video_exmaple_av_fail;
	}

	rt_printf("siso1 started\n\r");
#endif

	miso_video_aac_p2p = miso_create();
	if (miso_video_aac_p2p) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		miso_ctrl(miso_video_aac_p2p, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		miso_ctrl(miso_video_aac_p2p, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		miso_ctrl(miso_video_aac_p2p, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
		//miso_ctrl(miso_video_aac_p2p, MMIC_CMD_ADD_INPUT1, (uint32_t)aac_ctx, 0);
		miso_ctrl(miso_video_aac_p2p, MMIC_CMD_ADD_OUTPUT, (uint32_t)p2p_ctx, 0);
		miso_ctrl(miso_video_aac_p2p, MMIC_CMD_SET_TASKPRIORITY, 2, 0);
		miso_start(miso_video_aac_p2p);
	} else {
		rt_printf("miso open fail\n\r");
		goto mmf2_video_exmaple_av_fail;
	}
	rt_printf("miso started\n\r");

#if 0
	// RTP audio

	// Audio array input (doorbell)
	array_t array;
	array.data_addr = (uint32_t) doorbell_pcmu_sample;
	array.data_len = (uint32_t) doorbell_pcmu_sample_size;
	array_ctx = mm_module_open(&array_module);
	if (array_ctx) {
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_PARAMS, (int)&pcmu_array_params);
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_ARRAY, (int)&array);
		mm_module_ctrl(array_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(array_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(array_ctx, CMD_ARRAY_APPLY, 0);
	} else {
		rt_printf("ARRAY open fail\n\r");
		goto mmf2_video_exmaple_av_fail;
	}

	// G711D
	g711d_ctx = mm_module_open(&g711_module);
	if (g711d_ctx) {
		mm_module_ctrl(g711d_ctx, CMD_G711_SET_PARAMS, (int)&g711d_params);
		mm_module_ctrl(g711d_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711d_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711d_ctx, CMD_G711_APPLY, 0);
	} else {
		rt_printf("G711 open fail\n\r");
		goto mmf2_video_exmaple_av_fail;
	}

	siso_array_g711d = siso_create();
	if (siso_array_g711d) {
		siso_ctrl(siso_array_g711d, MMIC_CMD_ADD_INPUT, (uint32_t)array_ctx, 0);
		siso_ctrl(siso_array_g711d, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711d_ctx, 0);
		siso_start(siso_array_g711d);
	} else {
		rt_printf("siso_array_g711d open fail\n\r");
		goto mmf2_video_exmaple_av_fail;
	}

	siso_g711d_audio = siso_create();
	if (siso_g711d_audio) {
		siso_ctrl(siso_g711d_audio, MMIC_CMD_ADD_INPUT, (uint32_t)g711d_ctx, 0);
		siso_ctrl(siso_g711d_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
		siso_start(siso_g711d_audio);
	} else {
		rt_printf("siso_g711d_audio open fail\n\r");
		goto mmf2_video_exmaple_av_fail;
	}

	rt_printf("siso_g711d_audio started\n\r");

#endif

	return;
mmf2_video_exmaple_av_fail:

	return;
}

void doorbell_chime_main(void *param)
{

	wifi_common_init();
	mmf2_video_example_p2p_av_init();
	skynet_device_run();

	// TODO: exit condition or signal
	while (1) {
		vTaskDelay(10000);
	}
}

extern char gP2PString[41];
extern int UID_Setup;
extern int Config_Reset;
extern int Wake_On_Wlan;
extern void console_init(void);

void fATUID(void *arg)
{
	int ret = 0;
	if (!arg) {
		printf("[ATUID]Usage: ATUID=UID(Maximum length is 32)\n\r");
		ret = -2;
		goto exit;
	}
	if (strlen((char *)arg) > 32) {
		printf("[ATUID]Error: UID length can't exceed 32\n\r");
		ret = -2;
		goto exit;
	}
	printf("[ATUID]: _AT_WLAN_SET_P2P_UID_ [%s]\n\r", (char *)arg);
	memset(gP2PString, 0, 41);
	strncpy(gP2PString, (char *)arg, 40);
	UID_Setup = 1;
exit:
#if CONFIG_INIC_CMD_RSP
	inic_c2h_msg("ATUID", ret, NULL, 0);
#endif
	return;
}

log_item_t at_p2p_items[ ] = {
	{"ATUD", fATUID,},
};

void doorbell_chime_initialize(void)
{
	log_service_add_table(at_p2p_items, sizeof(at_p2p_items) / sizeof(at_p2p_items[0]));
	/*user can start their own task here*/
	if (xTaskCreate(doorbell_chime_main, ((const char *)"doorbell-chime"), 4096, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
		printf("\r\n doorbell_chime_main: Create Task Error\n");
	}
}