/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"
#include "avcodec.h"

#include "module_rtsp2.h"
#include "module_audio.h"
#include "module_g711.h"
#include "module_rtp.h"
#include "module_array.h"

#include "sample_doorbell_pcmu.h"

#define ENABLE_DOORBELL_RING 1
#define AUDIO_MIX_MODE 1    // User can choose using AUDIO_MIX_MODE for the audio ring

static mm_context_t *audio_ctx              = NULL;
static mm_context_t *rtsp2_ctx              = NULL;
static mm_context_t *g711e_ctx              = NULL;
static mm_context_t *g711d_ctx              = NULL;
static mm_context_t *ring_g711d_ctx         = NULL;
static mm_context_t *rtp_ctx                = NULL;
static mm_context_t *array_ctx              = NULL;

static mm_siso_t *siso_audio_g711e          = NULL;
static mm_siso_t *siso_g711_rtsp            = NULL;
static mm_siso_t *siso_ring_g711d           = NULL;
static mm_siso_t *siso_rtp_g711d            = NULL;
static mm_miso_t *miso_rtp_ring_g711d       = NULL;

static g711_params_t g711e_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_ENCODE
};

static rtsp2_params_t rtsp2_a_pcmu_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a = {
			.codec_id   = AV_CODEC_ID_PCMU,
			.channel    = 1,
			.samplerate = 16000
		}
	}
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

static array_params_t doorbell_pcmu_array_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.codec_id = AV_CODEC_ID_PCMU,
	.mode = ARRAY_MODE_ONCE,
	.u = {
		.a = {
			.channel    = 1,
			.samplerate = 16000,
			.frame_size = 640,
		}
	}
};

static audio_params_t audio_params;

#if ENABLE_DOORBELL_RING
static void audio_doorbell_ring(void)
{
	int state = 0;
	if (array_ctx) {
		mm_module_ctrl(array_ctx, CMD_ARRAY_GET_STATE, (int)&state);
		if (state) {
			printf("doorbell is ringing\n\r");
		} else {
			printf("start doorbell_ring\n\r");
#if !(AUDIO_MIX_MODE)
			miso_resume(miso_rtp_ring_g711d);
			miso_pause(miso_rtp_ring_g711d, MM_INPUT0);	// pause audio from rtp
#endif
			mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 1);	// doorbell ring

			do {	// wait until doorbell_ring done
				vTaskDelay(100);
				mm_module_ctrl(array_ctx, CMD_ARRAY_GET_STATE, (int)&state);
			} while (state == 1);
#if !(AUDIO_MIX_MODE)
			miso_resume(miso_rtp_ring_g711d);
			miso_pause(miso_rtp_ring_g711d, MM_INPUT1);	// pause array
#endif
			printf("doorbell_ring done!\n\r");
		}
	}
}
#endif

#if AUDIO_MIX_MODE
static void audio_params_enable_mixmode(void)
{
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_PARAMS, (int)&audio_params);
	audio_params.mix_mode = 1;  // enable audio mix mode
	audio_params.sample_rate = ASR_16KHZ;
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
}
#endif

void audio_save_log_init(void);
void mmf2_example_2way_audio_g711_doorbell_init(void)
{
	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
#if AUDIO_MIX_MODE
		audio_params_enable_mixmode();
#endif
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		rt_printf("audio open fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}


	g711e_ctx = mm_module_open(&g711_module);
	if (g711e_ctx) {
		mm_module_ctrl(g711e_ctx, CMD_G711_SET_PARAMS, (int)&g711e_params);
		mm_module_ctrl(g711e_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711e_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711e_ctx, CMD_G711_APPLY, 0);
	} else {
		rt_printf("G711 open fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}

	//--------------RTSP---------------
	rtsp2_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_ctx) {
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_a_pcmu_params);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_APPLY, 0);

		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}

	//--------------Link---------------------------
	siso_audio_g711e = siso_create();
	if (siso_audio_g711e) {
		siso_ctrl(siso_audio_g711e, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_g711e, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711e_ctx, 0);
		siso_start(siso_audio_g711e);
	} else {
		rt_printf("siso_audio_g711e open fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}

	rt_printf("siso_audio_g711e started\n\r");


	siso_g711_rtsp = siso_create();
	if (siso_g711_rtsp) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_g711_rtsp, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_g711_rtsp, MMIC_CMD_ADD_INPUT, (uint32_t)g711e_ctx, 0);
		siso_ctrl(siso_g711_rtsp, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_ctx, 0);
		siso_start(siso_g711_rtsp);
	} else {
		rt_printf("siso_g711_rtsp fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}
	rt_printf("siso_g711_rtsp started\n\r");

	// RTP audio
	rtp_ctx = mm_module_open(&rtp_module);
	if (rtp_ctx) {
		mm_module_ctrl(rtp_ctx, CMD_RTP_SET_PARAMS, (int)&rtp_g711d_params);
		mm_module_ctrl(rtp_ctx, MM_CMD_SET_QUEUE_LEN, 16);
		mm_module_ctrl(rtp_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(rtp_ctx, CMD_RTP_APPLY, 0);
		mm_module_ctrl(rtp_ctx, CMD_RTP_STREAMING, 1);	// streamming on
	} else {
		rt_printf("RTP open fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}

	// Audio array input (doorbell)
	array_t array;
	array.data_addr = (uint32_t) doorbell_pcmu_sample;
	array.data_len = (uint32_t) doorbell_pcmu_sample_size;
	array_ctx = mm_module_open(&array_module);
	if (array_ctx) {
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_PARAMS, (int)&doorbell_pcmu_array_params);
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_ARRAY, (int)&array);
		mm_module_ctrl(array_ctx, MM_CMD_SET_QUEUE_LEN, 16);
		mm_module_ctrl(array_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(array_ctx, CMD_ARRAY_APPLY, 0);
		//mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 1);	// streamming on
	} else {
		rt_printf("ARRAY open fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}


	g711d_ctx = mm_module_open(&g711_module);
	if (g711d_ctx) {
		mm_module_ctrl(g711d_ctx, CMD_G711_SET_PARAMS, (int)&g711d_params);
		mm_module_ctrl(g711d_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711d_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711d_ctx, CMD_G711_APPLY, 0);
	} else {
		rt_printf("G711 open fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}

	ring_g711d_ctx = mm_module_open(&g711_module);
	if (ring_g711d_ctx) {
		mm_module_ctrl(ring_g711d_ctx, CMD_G711_SET_PARAMS, (int)&g711d_params);
		mm_module_ctrl(ring_g711d_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(ring_g711d_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(ring_g711d_ctx, CMD_G711_APPLY, 0);
	} else {
		rt_printf("G711 open fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}

	siso_rtp_g711d = siso_create();
	if (siso_rtp_g711d) {
		siso_ctrl(siso_rtp_g711d, MMIC_CMD_ADD_INPUT, (uint32_t)rtp_ctx, 0);
		siso_ctrl(siso_rtp_g711d, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711d_ctx, 0);
		siso_start(siso_rtp_g711d);
	} else {
		rt_printf("siso_ring_g711d open fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}

	siso_ring_g711d = siso_create();
	if (siso_ring_g711d) {
		siso_ctrl(siso_ring_g711d, MMIC_CMD_ADD_INPUT, (uint32_t)array_ctx, 0);
		siso_ctrl(siso_ring_g711d, MMIC_CMD_ADD_OUTPUT, (uint32_t)ring_g711d_ctx, 0);
		siso_start(siso_ring_g711d);
	} else {
		rt_printf("siso_ring_g711d open fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}

	miso_rtp_ring_g711d = miso_create();
	if (miso_rtp_ring_g711d) {
		miso_ctrl(miso_rtp_ring_g711d, MMIC_CMD_ADD_INPUT0, (uint32_t)g711d_ctx, 0);
		miso_ctrl(miso_rtp_ring_g711d, MMIC_CMD_ADD_INPUT1, (uint32_t)ring_g711d_ctx, 0);
		miso_ctrl(miso_rtp_ring_g711d, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
		miso_start(miso_rtp_ring_g711d);
	} else {
		rt_printf("miso_rtp_ring_g711d open fail\n\r");
		goto mmf2_example_2way_audio_g711_doorbell_fail;
	}

	rt_printf("miso_rtp_ring_g711d started\n\r");

#if ENABLE_DOORBELL_RING
	while (1) {
		printf("Delay 10s and then ring the doorbell\n\r");
		vTaskDelay(10000);
		audio_doorbell_ring();
	}
#endif


	return;
mmf2_example_2way_audio_g711_doorbell_fail:

	return;
}