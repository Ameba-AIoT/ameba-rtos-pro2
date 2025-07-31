/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "avcodec.h"

#include "module_audio.h"
#include "module_i2s.h"
#include "module_g711.h"

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
static mm_context_t *g711e_ctx      = NULL;
static mm_context_t *g711d_ctx      = NULL;
static mm_siso_t *siso_audio_g711e  = NULL;
static mm_siso_t *siso_g711_e2d     = NULL;
static mm_siso_t *siso_g711d_audio  = NULL;

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

void mmf2_example_g711loop_init(void)
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
		goto mmf2_exmaple_g711loop_fail;
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
		goto mmf2_exmaple_g711loop_fail;
	}
#endif

	g711e_ctx = mm_module_open(&g711_module);
	if (g711e_ctx) {
		mm_module_ctrl(g711e_ctx, CMD_G711_SET_PARAMS, (int)&g711e_params);
		mm_module_ctrl(g711e_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711e_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711e_ctx, CMD_G711_APPLY, 0);
	} else {
		printf("G711 open fail\n\r");
		goto mmf2_exmaple_g711loop_fail;
	}


	g711d_ctx = mm_module_open(&g711_module);
	if (g711d_ctx) {
		mm_module_ctrl(g711d_ctx, CMD_G711_SET_PARAMS, (int)&g711d_params);
		mm_module_ctrl(g711d_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711d_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711d_ctx, CMD_G711_APPLY, 0);
	} else {
		printf("G711 open fail\n\r");
		goto mmf2_exmaple_g711loop_fail;
	}


	siso_audio_g711e = siso_create();
	if (siso_audio_g711e) {
#if AUDIO_SRC==AUDIO_INTERFACE
		siso_ctrl(siso_audio_g711e, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
#elif AUDIO_SRC==I2S_INTERFACE
		siso_ctrl(siso_audio_g711e, MMIC_CMD_ADD_INPUT, (uint32_t)i2s_ctx, 0);
#endif
		siso_ctrl(siso_audio_g711e, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711e_ctx, 0);
		siso_start(siso_audio_g711e);
	} else {
		printf("siso1 open fail\n\r");
		goto mmf2_exmaple_g711loop_fail;
	}


	siso_g711_e2d = siso_create();
	if (siso_g711_e2d) {
		siso_ctrl(siso_g711_e2d, MMIC_CMD_ADD_INPUT, (uint32_t)g711e_ctx, 0);
		siso_ctrl(siso_g711_e2d, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711d_ctx, 0);
		siso_start(siso_g711_e2d);
	} else {
		printf("siso2 open fail\n\r");
		goto mmf2_exmaple_g711loop_fail;
	}

	siso_g711d_audio = siso_create();
	if (siso_g711d_audio) {
		siso_ctrl(siso_g711d_audio, MMIC_CMD_ADD_INPUT, (uint32_t)g711d_ctx, 0);
#if AUDIO_SRC==AUDIO_INTERFACE
		siso_ctrl(siso_g711d_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
#elif AUDIO_SRC==I2S_INTERFACE
		siso_ctrl(siso_g711d_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)i2s_ctx, 0);
#endif
		siso_start(siso_g711d_audio);
	} else {
		printf("siso3 open fail\n\r");
		goto mmf2_exmaple_g711loop_fail;
	}

	printf("siso1 started\n\r");


	return;
mmf2_exmaple_g711loop_fail:

	return;
}