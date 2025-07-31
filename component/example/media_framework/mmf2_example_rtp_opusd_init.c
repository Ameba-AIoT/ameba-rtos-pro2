/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
//#include "example_media_framework.h"
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_audio.h"
#include "module_i2s.h"
#include "module_opusd.h"
#include "module_rtp.h"

#define I2S_INTERFACE   0
#define AUDIO_INTERFACE 1

#define AUDIO_SRC AUDIO_INTERFACE

#if AUDIO_SRC==AUDIO_INTERFACE
static mm_context_t *audio_ctx      = NULL;
static audio_params_t audio_params;
#elif AUDIO_SRC==I2S_INTERFACE
static mm_context_t *i2s_ctx        = NULL;
static i2s_params_t i2s_params;
#else
#error "please set correct AUDIO_SRC"
#endif
static mm_context_t *rtp_ctx		= NULL;
static mm_context_t *opusd_ctx		= NULL;

static mm_siso_t *siso_rtp_opusd    = NULL;
static mm_siso_t *siso_opusd_audio	= NULL;

static opusd_params_t opusd_only_params = {
	.sample_rate = 16000,//
	.channel = 1,
	.bit_length = 16,         //16 recommand
	.frame_size_in_msec = 10, //will not be uused
	.with_opus_enc = 0,       //enable semaphore if the application with opus encoder
	.opus_application = OPUS_APPLICATION_AUDIO
};

static rtp_params_t rtp_opusd_params = {
	.valid_pt = 0xFFFFFFFF,
	.port = 16384,
	.frame_size = 1500,
	.cache_depth = 15
};

void mmf2_example_rtp_opusd_init(void)
{
	rtp_ctx = mm_module_open(&rtp_module);
	if (rtp_ctx) {
		mm_module_ctrl(rtp_ctx, CMD_RTP_SET_PARAMS, (int)&rtp_opusd_params);
		mm_module_ctrl(rtp_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(rtp_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(rtp_ctx, CMD_RTP_APPLY, 0);
		mm_module_ctrl(rtp_ctx, CMD_RTP_STREAMING, 1);	// streamming on
	} else {
		printf("RTP open fail\n\r");
		goto mmf2_exmaple_rtp_opusd_fail;
	}

	opusd_ctx = mm_module_open(&opusd_module);
	if (opusd_ctx) {
		mm_module_ctrl(opusd_ctx, CMD_OPUSD_SET_PARAMS, (int)&opusd_only_params);
		mm_module_ctrl(opusd_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(opusd_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(opusd_ctx, CMD_OPUSD_APPLY, 0);
	} else {
		printf("OPUSD open fail\n\r");
		goto mmf2_exmaple_rtp_opusd_fail;
	}

#if AUDIO_SRC==AUDIO_INTERFACE
	//since the audio module is only a sink in this example, the output queue is not needed in this example
	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
		mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_PARAMS, (int)&audio_params);
		audio_params.sample_rate = ASR_16KHZ;
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		printf("audio open fail\n\r");
		goto mmf2_exmaple_rtp_opusd_fail;
	}
#elif AUDIO_SRC==I2S_INTERFACE
	//since the i2s module is only a sink in this example, the output queue is not needed in this example
	i2s_ctx = mm_module_open(&i2s_module);
	if (i2s_ctx) {
		mm_module_ctrl(i2s_ctx, CMD_I2S_GET_PARAMS, (int)&i2s_params);
		i2s_params.sample_rate = SR_16KHZ;
		i2s_params.i2s_direction = I2S_TX_ONLY;
		mm_module_ctrl(i2s_ctx, CMD_I2S_SET_PARAMS, (int)&i2s_params);
		mm_module_ctrl(i2s_ctx, CMD_I2S_APPLY, 0);
	} else {
		printf("i2s open fail\n\r");
		goto mmf2_exmaple_rtp_opusd_fail;
	}
#endif

	siso_rtp_opusd = siso_create();
	if (siso_rtp_opusd) {
		siso_ctrl(siso_rtp_opusd, MMIC_CMD_ADD_INPUT, (uint32_t)rtp_ctx, 0);
		siso_ctrl(siso_rtp_opusd, MMIC_CMD_ADD_OUTPUT, (uint32_t)opusd_ctx, 0);
		siso_ctrl(siso_rtp_opusd, MMIC_CMD_SET_STACKSIZE, 24 * 1024, 0);
		//siso_ctrl(siso_rtp_opusd, MMIC_CMD_SET_TASKNANE, (uint32_t)"rtp_opd", 0);
		siso_start(siso_rtp_opusd);
	} else {
		printf("siso1 open fail\n\r");
		goto mmf2_exmaple_rtp_opusd_fail;
	}

	printf("siso3 started\n\r");

	siso_opusd_audio = siso_create();
	if (siso_opusd_audio) {
		siso_ctrl(siso_opusd_audio, MMIC_CMD_ADD_INPUT, (uint32_t)opusd_ctx, 0);
#if AUDIO_SRC==AUDIO_INTERFACE
		siso_ctrl(siso_opusd_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
#elif AUDIO_SRC==I2S_INTERFACE
		siso_ctrl(siso_opusd_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)i2s_ctx, 0);
#endif
		//siso_ctrl(siso_opusd_audio, MMIC_CMD_SET_TASKNANE, (uint32_t)"opusd_audio", 0);
		siso_start(siso_opusd_audio);
	} else {
		printf("siso2 open fail\n\r");
		goto mmf2_exmaple_rtp_opusd_fail;
	}

	printf("siso2 started\n\r");

	return;
mmf2_exmaple_rtp_opusd_fail:

	return;

}