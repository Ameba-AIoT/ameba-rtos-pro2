/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_audio.h"
#include "module_i2s.h"

#define I2S_INTERFACE   0
#define AUDIO_INTERFACE 1

#define AUDIO_SRC AUDIO_INTERFACE

#if AUDIO_SRC==AUDIO_INTERFACE
static mm_context_t *audio_ctx		= NULL;
static audio_params_t audio_params;
#define AUDIO_PCM_DB 0

#if defined(AUDIO_PCM_DB) && AUDIO_PCM_DB
static float transfer_adc_gain2dB(int ADC_gain)
{
	return 30.0 - 0.375 * (0x7F - ADC_gain);
}

static void left_mic_cb(const uint8_t *data, int data_length, uint8_t bytespersample, uint32_t samplerate, audio_params_t audio_params)
{
	//User can get data, data_length(bytes), bytespersample, sample rate and all the audio parameters in this function
	//This is the fucntion is an example to transfer ADC gain into dB
	printf("The ADC Gain setting is %.04fdBFS\r\n", transfer_adc_gain2dB(audio_params.ADC_gain));
}
#endif
#elif AUDIO_SRC==I2S_INTERFACE
static mm_context_t *i2s_ctx        = NULL;
static i2s_params_t i2s_params;
#else
#error "please set correct AUDIO_SRC"
#endif

static mm_siso_t *siso_audio_loop   = NULL;

void mmf2_example_audioloop_init(void)
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
		goto mmf2_exmaple_audioloop_fail;
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
		goto mmf2_exmaple_audioloop_fail;
	}
#endif

	siso_audio_loop = siso_create();
	if (siso_audio_loop) {
#if AUDIO_SRC==AUDIO_INTERFACE
		siso_ctrl(siso_audio_loop, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_loop, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
#elif AUDIO_SRC==I2S_INTERFACE
		siso_ctrl(siso_audio_loop, MMIC_CMD_ADD_INPUT, (uint32_t)i2s_ctx, 0);
		siso_ctrl(siso_audio_loop, MMIC_CMD_ADD_OUTPUT, (uint32_t)i2s_ctx, 0);
#endif
		siso_start(siso_audio_loop);
	} else {
		printf("siso1 open fail\n\r");
		goto mmf2_exmaple_audioloop_fail;
	}

	printf("siso1 started\n\r");

	return;
mmf2_exmaple_audioloop_fail:

	return;
}