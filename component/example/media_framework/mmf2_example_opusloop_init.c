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
#include "module_opusc.h"
#include "module_opusd.h"

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
static mm_context_t *opusc_ctx      = NULL;
static mm_context_t *opusd_ctx      = NULL;
static mm_siso_t *siso_audio_opusc  = NULL;
static mm_siso_t *siso_opus_e2d     = NULL;
static mm_siso_t *siso_opusd_audio  = NULL;

static opusc_params_t opusc_params = {
	//8000/16000
	.sample_rate = 16000,
	.channel = 1,
	.bit_length = 16,			//16 recommand
	.complexity = 5,			//0~10
	.bitrate = 25000,			//default 25000
	.use_framesize = 0,			//set 0 only when using audio loop
	.enable_vbr = 1,
	.vbr_constraint = 0,
	.packetLossPercentage = 0,
	.opus_application = OPUS_APPLICATION_AUDIO

};

static opusd_params_t opusd_params = {
	.sample_rate = 16000,
	.channel = 1,
	.bit_length = 16,			//16 recommand
	.frame_size_in_msec = 10,	//will not be uused
	.with_opus_enc = 0,			//enable semaphore if the application with opus encoder
	.opus_application = OPUS_APPLICATION_AUDIO
};

void mmf2_example_opusloop_init(void)
{
#if AUDIO_SRC==AUDIO_INTERFACE
	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
		mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_PARAMS, (int)&audio_params);
		audio_params.sample_rate = ASR_16KHZ;
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		printf("audio open fail\n\r");
		goto mmf2_exmaple_opusloop_fail;
	}
#elif AUDIO_SRC==I2S_INTERFACE
	i2s_ctx = mm_module_open(&i2s_module);
	if (i2s_ctx) {
		mm_module_ctrl(i2s_ctx, CMD_I2S_GET_PARAMS, (int)&i2s_params);
		i2s_params.sample_rate = SR_16KHZ;
		i2s_params.i2s_direction = I2S_TRX_BOTH;
		mm_module_ctrl(i2s_ctx, CMD_I2S_SET_PARAMS, (int)&i2s_params);
		mm_module_ctrl(i2s_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(i2s_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(i2s_ctx, CMD_I2S_APPLY, 0);
	} else {
		printf("i2s open fail\n\r");
		goto mmf2_exmaple_opusloop_fail;
	}
#endif

	opusc_ctx = mm_module_open(&opusc_module);
	if (opusc_ctx) {
		mm_module_ctrl(opusc_ctx, CMD_OPUSC_SET_PARAMS, (int)&opusc_params);
		mm_module_ctrl(opusc_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(opusc_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(opusc_ctx, CMD_OPUSC_APPLY, 0);
	} else {
		printf("OPUSC open fail\n\r");
		goto mmf2_exmaple_opusloop_fail;
	}

	opusd_ctx = mm_module_open(&opusd_module);
	if (opusd_ctx) {
		mm_module_ctrl(opusd_ctx, CMD_OPUSD_SET_PARAMS, (int)&opusd_params);
		mm_module_ctrl(opusd_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(opusd_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(opusd_ctx, CMD_OPUSD_APPLY, 0);
	} else {
		printf("OPUSD open fail\n\r");
		goto mmf2_exmaple_opusloop_fail;
	}

	siso_audio_opusc = siso_create();
	if (siso_audio_opusc) {
#if AUDIO_SRC==AUDIO_INTERFACE
		siso_ctrl(siso_audio_opusc, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
#elif AUDIO_SRC==I2S_INTERFACE
		siso_ctrl(siso_audio_opusc, MMIC_CMD_ADD_INPUT, (uint32_t)i2s_ctx, 0);
#endif
		siso_ctrl(siso_audio_opusc, MMIC_CMD_ADD_OUTPUT, (uint32_t)opusc_ctx, 0);
		siso_ctrl(siso_audio_opusc, MMIC_CMD_SET_STACKSIZE, 24 * 1024, 0);
		siso_start(siso_audio_opusc);
	} else {
		printf("siso1 open fail\n\r");
		goto mmf2_exmaple_opusloop_fail;
	}

	printf("siso1 started\n\r");

	siso_opus_e2d = siso_create();
	if (siso_opus_e2d) {
		siso_ctrl(siso_opus_e2d, MMIC_CMD_ADD_INPUT, (uint32_t)opusc_ctx, 0);
		siso_ctrl(siso_opus_e2d, MMIC_CMD_ADD_OUTPUT, (uint32_t)opusd_ctx, 0);
		siso_ctrl(siso_opus_e2d, MMIC_CMD_SET_STACKSIZE, 24 * 1024, 0);
		siso_start(siso_opus_e2d);
	} else {
		printf("siso2 open fail\n\r");
		goto mmf2_exmaple_opusloop_fail;
	}

	printf("siso2 started\n\r");

	siso_opusd_audio = siso_create();
	if (siso_opusd_audio) {
		siso_ctrl(siso_opusd_audio, MMIC_CMD_ADD_INPUT, (uint32_t)opusd_ctx, 0);
#if AUDIO_SRC==AUDIO_INTERFACE
		siso_ctrl(siso_opusd_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
#elif AUDIO_SRC==I2S_INTERFACE
		siso_ctrl(siso_opusd_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)i2s_ctx, 0);
#endif
		siso_start(siso_opusd_audio);
	} else {
		printf("siso3 open fail\n\r");
		goto mmf2_exmaple_opusloop_fail;
	}

	printf("siso3 started\n\r");


	return;
mmf2_exmaple_opusloop_fail:

	return;
}