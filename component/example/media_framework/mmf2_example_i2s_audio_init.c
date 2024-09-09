/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
//TODO compile ok, not test yet
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_audio.h"
#include "module_i2s.h"

static mm_context_t *i2s_ctx 			= NULL;
static mm_context_t *audio_ctx 			= NULL;
static mm_siso_t *siso_audio_loop       = NULL;

#if !USE_DEFAULT_AUDIO_SET
static audio_params_t audio_params = {
	.sample_rate = ASR_8KHZ,
	.word_length = WL_16BIT,
	.mic_gain    = MIC_0DB,
	.channel     = 1,
	.enable_aec  = 0
};
#endif

static i2s_params_t i2s_params = {
	.sample_rate = SR_16KHZ,
	.word_length = WL_24b,
	.out_sample_rate = SR_8KHZ,
	.out_word_length = WL_16b,
	.mic_gain    = MIC_40DB,
	.channel     = 2,
	.out_channel = 1,
	.enable_aec  = 0
};

void mmf2_example_i2s_audio_init(void)
{
	i2s_ctx = mm_module_open(&i2s_module);
	if (i2s_ctx) {
		mm_module_ctrl(i2s_ctx, CMD_I2S_SET_PARAMS, (int)&i2s_params);
		mm_module_ctrl(i2s_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(i2s_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(i2s_ctx, CMD_I2S_APPLY, 0);
	} else {
		rt_printf("i2s open fail\n\r");
		goto mmf2_exmaple_i2s_audio_fail;
	}
	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
#if USE_DEFAULT_AUDIO_SET
		default_audio_params.use_mic_type = USE_AUDIO_AMIC; //since the dmic pin will conflit the i2s
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&default_audio_params);
#else
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
#endif
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		rt_printf("audio open fail\n\r");
		goto mmf2_exmaple_i2s_audio_fail;
	}
	siso_audio_loop = siso_create();
	if (siso_audio_loop) {
		siso_ctrl(siso_audio_loop, MMIC_CMD_ADD_INPUT, (uint32_t)i2s_ctx, 0);
		siso_ctrl(siso_audio_loop, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
		siso_start(siso_audio_loop);
	} else {
		rt_printf("siso1 open fail\n\r");
		goto mmf2_exmaple_i2s_audio_fail;
	}

	rt_printf("siso1 started\n\r");


	return;
mmf2_exmaple_i2s_audio_fail:

	return;
}