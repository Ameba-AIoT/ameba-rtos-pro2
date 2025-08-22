/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_audio.h"
#include "module_i2s.h"
#include "module_aad.h"
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
static mm_context_t *rtp_ctx        = NULL;
static mm_context_t *aad_ctx        = NULL;

static mm_siso_t *siso_rtp_aad      = NULL;
static mm_siso_t *siso_aad_audio    = NULL;

static aad_params_t aad_rtp_params = {
	.sample_rate = 16000,
	.channel = 1,
	.trans_type = AAD_TYPE_RTP_RAW,
	.object_type = AAD_AOT_LC
};

static rtp_params_t rtp_aad_params = {
	.valid_pt = 0xFFFFFFFF,
	.port = 16384,
	.frame_size = 1500,
	.cache_depth = 6
};

void mmf2_example_rtp_aad_init(void)
{
	rtp_ctx = mm_module_open(&rtp_module);
	if (rtp_ctx) {
		mm_module_ctrl(rtp_ctx, CMD_RTP_SET_PARAMS, (int)&rtp_aad_params);
		mm_module_ctrl(rtp_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(rtp_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(rtp_ctx, CMD_RTP_APPLY, 0);
		mm_module_ctrl(rtp_ctx, CMD_RTP_STREAMING, 1);	// streamming on
	} else {
		rt_printf("RTP open fail\n\r");
		goto mmf2_exmaple_rtp_aad_fail;
	}

	aad_ctx = mm_module_open(&aad_module);
	if (aad_ctx) {
		mm_module_ctrl(aad_ctx, CMD_AAD_SET_PARAMS, (int)&aad_rtp_params);
		mm_module_ctrl(aad_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(aad_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(aad_ctx, CMD_AAD_APPLY, 0);
	} else {
		rt_printf("AAD open fail\n\r");
		goto mmf2_exmaple_rtp_aad_fail;
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
		goto mmf2_exmaple_rtp_aad_fail;
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
		goto mmf2_exmaple_rtp_aad_fail;
	}
#endif

	siso_rtp_aad = siso_create();
	if (siso_rtp_aad) {
		siso_ctrl(siso_rtp_aad, MMIC_CMD_ADD_INPUT, (uint32_t)rtp_ctx, 0);
		siso_ctrl(siso_rtp_aad, MMIC_CMD_ADD_OUTPUT, (uint32_t)aad_ctx, 0);
		siso_ctrl(siso_rtp_aad, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(siso_rtp_aad);
	} else {
		rt_printf("siso1 open fail\n\r");
		goto mmf2_exmaple_rtp_aad_fail;
	}

	rt_printf("siso1 started\n\r");

	siso_rtp_aad = siso_create();
	if (siso_rtp_aad) {
		siso_ctrl(siso_rtp_aad, MMIC_CMD_ADD_INPUT, (uint32_t)aad_ctx, 0);
#if AUDIO_SRC==AUDIO_INTERFACE
		siso_ctrl(siso_rtp_aad, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
#elif AUDIO_SRC==I2S_INTERFACE
		siso_ctrl(siso_rtp_aad, MMIC_CMD_ADD_OUTPUT, (uint32_t)i2s_ctx, 0);
#endif
		siso_start(siso_rtp_aad);
	} else {
		rt_printf("siso2 open fail\n\r");
		goto mmf2_exmaple_rtp_aad_fail;
	}

	rt_printf("siso2 started\n\r");

	return;
mmf2_exmaple_rtp_aad_fail:

	return;

}