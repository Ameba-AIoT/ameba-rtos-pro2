/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_audio.h"
#include "module_i2s.h"

static mm_context_t *i2s_ctx        = NULL;
static mm_context_t *audio_ctx      = NULL;
static mm_siso_t *siso_i2s_audio    = NULL;
static mm_siso_t *siso_audio_i2s    = NULL;

static audio_params_t audio_params;

static i2s_params_t i2s_params;

void mmf2_example_i2s_audio_init(void)
{

	i2s_ctx = mm_module_open(&i2s_module);
	if (i2s_ctx) {
		mm_module_ctrl(i2s_ctx, CMD_I2S_GET_PARAMS, (int)&i2s_params);
		i2s_params.sample_rate = SR_16KHZ;
		i2s_params.i2s_direction = I2S_TRX_BOTH;
		mm_module_ctrl(i2s_ctx, CMD_I2S_SET_PARAMS, (int)&i2s_params);
		mm_module_ctrl(i2s_ctx, MM_CMD_SET_QUEUE_LEN, 120);
		mm_module_ctrl(i2s_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(i2s_ctx, CMD_I2S_APPLY, 0);
	} else {
		printf("i2s open fail\n\r");
		goto mmf2_exmaple_i2s_audio_fail;
	}

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
		goto mmf2_exmaple_i2s_audio_fail;
	}

	siso_i2s_audio = siso_create();
	if (siso_i2s_audio) {
		siso_ctrl(siso_i2s_audio, MMIC_CMD_ADD_INPUT, (uint32_t)i2s_ctx, 0);
		siso_ctrl(siso_i2s_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
		siso_start(siso_i2s_audio);
	} else {
		printf("siso_i2s_audio open fail\n\r");
		goto mmf2_exmaple_i2s_audio_fail;
	}
	printf("siso_i2s_audio started\n\r");

	siso_audio_i2s = siso_create();
	if (siso_audio_i2s) {
		siso_ctrl(siso_audio_i2s, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_i2s, MMIC_CMD_ADD_OUTPUT, (uint32_t)i2s_ctx, 0);
		siso_start(siso_audio_i2s);
	} else {
		printf("siso_audio_i2s open fail\n\r");
		goto mmf2_exmaple_i2s_audio_fail;
	}

	printf("siso_audio_i2s started\n\r");
	return;
mmf2_exmaple_i2s_audio_fail:

	return;
}