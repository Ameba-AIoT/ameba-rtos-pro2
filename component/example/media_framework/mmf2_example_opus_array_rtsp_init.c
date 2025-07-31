/******************************************************************************
 *
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_array.h"
#include "module_rtsp2.h"

#include "sample_opus.h"

static mm_context_t *array_ctx            = NULL;
static mm_context_t *rtsp2_ctx            = NULL;
static mm_siso_t *siso_array_rtsp         = NULL;

static array_params_t opus_array_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.codec_id = AV_CODEC_ID_OPUS,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.a = {
			.channel    = 1,
			.samplerate = 16000,
			.sample_bit_length = 16,
			.frame_size = 80
		}
	}
};

static rtsp2_params_t rtsp2_a_opus_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a = {
			.codec_id   = AV_CODEC_ID_OPUS,
			.channel    = 1,
			.samplerate = 16000,
			.frame_size = 20
		}
	}
};

void mmf2_example_opus_array_rtsp_init(void)
{
	// Audio array input (AAC)
	array_t array;
	array.data_addr = (uint32_t) opus_sample;
	array.data_len = (uint32_t) opus_sample_size;
	array_ctx = mm_module_open(&array_module);
	if (array_ctx) {
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_PARAMS, (int)&opus_array_params);
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_ARRAY, (int)&array);
		mm_module_ctrl(array_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(array_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(array_ctx, CMD_ARRAY_APPLY, 0);
		mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 1);	// streamming on
	} else {
		rt_printf("ARRAY open fail\n\r");
		goto mmf2_example_opus_array_rtsp_fail;
	}

	// RTSP
	rtsp2_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_ctx) {
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_a_opus_params);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_example_opus_array_rtsp_fail;
	}
	rt_printf("RTSP2 opened\n\r");


	//--------------Link---------------------------
	siso_array_rtsp = siso_create();
	if (siso_array_rtsp) {
		siso_ctrl(siso_array_rtsp, MMIC_CMD_ADD_INPUT, (uint32_t)array_ctx, 0);
		siso_ctrl(siso_array_rtsp, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_ctx, 0);
		siso_start(siso_array_rtsp);
	} else {
		rt_printf("siso_array_rtsp open fail\n\r");
		goto mmf2_example_opus_array_rtsp_fail;
	}
	rt_printf("siso_array_rtsp started\n\r");

	return;
mmf2_example_opus_array_rtsp_fail:

	return;
}