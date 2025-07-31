/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_audio.h"
#include "module_i2s.h"
#include "module_array.h"

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
#include "module_i2s.h"
#include "module_array.h"

#include "avcodec.h"

#include "sample_pcm_8k.h"
#include "sample_pcm_16k.h"
static mm_context_t *array_pcm_ctx  = NULL;
static mm_siso_t *siso_array_audio  = NULL;

static array_params_t pcm_array_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.codec_id = AV_CODEC_ID_PCM_RAW,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.a = {
			.channel    = 1,
			.samplerate = 16000, //8000,
			.frame_size = 640, //suggest set the same as audio
		}
	}
};

void mmf2_example_pcm_array_audio_init(void)
{
	array_t a_array;
	//a_array.data_addr = (uint32_t) pcm_sample_8k;
	//a_array.data_len = (uint32_t) pcm_sample_8k_size;
	a_array.data_addr = (uint32_t) pcm_sample_16k;
	a_array.data_len = (uint32_t) pcm_sample_16k_size;
	array_pcm_ctx = mm_module_open(&array_module);
	if (array_pcm_ctx) {
		mm_module_ctrl(array_pcm_ctx, CMD_ARRAY_SET_PARAMS, (int)&pcm_array_params);
		mm_module_ctrl(array_pcm_ctx, CMD_ARRAY_SET_ARRAY, (int)&a_array);
		mm_module_ctrl(array_pcm_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(array_pcm_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(array_pcm_ctx, CMD_ARRAY_APPLY, 0);
		mm_module_ctrl(array_pcm_ctx, CMD_ARRAY_STREAMING, 1);	// streamming on
	} else {
		printf("ARRAY open fail\n\r");
		goto mmf2_example_pcm_audio_init;
	}
	printf("ARRAY opened\n\r");

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
		goto mmf2_example_pcm_audio_init;
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
		goto mmf2_example_pcm_audio_init;
	}
#endif
	printf("AUDIO opened\n\r");

	//--------------Link---------------------------
	siso_array_audio = siso_create();
	if (siso_array_audio) {
		siso_ctrl(siso_array_audio, MMIC_CMD_ADD_INPUT, (uint32_t)array_pcm_ctx, 0);
#if AUDIO_SRC==AUDIO_INTERFACE
		siso_ctrl(siso_array_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
#elif AUDIO_SRC==I2S_INTERFACE
		siso_ctrl(siso_array_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)i2s_ctx, 0);
#endif
		siso_start(siso_array_audio);
	} else {
		printf("siso_array_audio open fail\n\r");
		goto mmf2_example_pcm_audio_init;
	}
	printf("siso_array_audio started\n\r");

	return;
mmf2_example_pcm_audio_init:

	return;
}