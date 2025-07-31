/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "module_video.h"
#include "module_audio.h"
#include "module_aac.h"
#include "module_mp4.h"
#include "module_array.h"

#include "avcodec.h"
#include "sample_h264.h"
#include "sample_h265.h"
#include "sample_pcmu.h"
#include "video_example_media_framework.h"
#include "log_service.h"

static void atcmd_userctrl_init(void);
static mm_context_t *array_ctx          = NULL;
static mm_context_t *array_a_ctx          = NULL;
static mm_context_t *mp4_ctx            = NULL;
static mm_miso_t *miso_array_mp4       = NULL;

//#define H265_ARRAY_ENABLE
static array_params_t h264_array_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.codec_id = AV_CODEC_ID_H264,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.v = {
			.fps    = 25,
			.h264_nal_size = 4,
		}
	}
};

static array_params_t h265_array_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.codec_id = AV_CODEC_ID_H265,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.v = {
			.fps    = 25,
			.h264_nal_size = 4,
		}
	}
};

static array_params_t pcmu_array_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.codec_id = AV_CODEC_ID_PCMU,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.a = {
			.channel    = 1,
			.samplerate = 8000,
			.frame_size = 160,
		}
	}
};

static mp4_params_t mp4_v1_params = {
	.width          = 0,
	.height         = 0,
	.fps            = 0,
	.gop            = 25,

	.sample_rate = 8000,
	.channel = 1,

	.record_length = 120, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 1,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
	.mp4_audio_format = AUDIO_ULAW,//0:AAC 1:PCM ULAW 2.PCM ALAW
	.mp4_audio_duration = 20,//audio duration 20ms for PCM
};

void mmf2_video_example_h264_pcmu_array_mp4_init(void)
{
	// Video array input (H264)
	array_t array;
#ifdef  H265_ARRAY_ENABLE
	array.data_addr = (uint32_t) h265_sample;
	array.data_len = (uint32_t) h265_sample_len;
#else
	array.data_addr = (uint32_t) h264_sample;
	array.data_len = (uint32_t) h264_sample_len;
#endif
	array_ctx = mm_module_open(&array_module);
	if (array_ctx) {
#ifdef  H265_ARRAY_ENABLE
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_PARAMS, (int)&h265_array_params);
#else
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_PARAMS, (int)&h264_array_params);
#endif
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_ARRAY, (int)&array);
		mm_module_ctrl(array_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(array_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(array_ctx, CMD_ARRAY_APPLY, 0);
		mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 1);	// streamming on
	} else {
		rt_printf("ARRAY open fail\n\r");
		goto mmf2_video_example_h264_pcmu_array_mp4_init_fail;
	}

	mp4_ctx = mm_module_open(&mp4_module);
#ifdef  H265_ARRAY_ENABLE
	mp4_v1_params.fps = 30;
	mp4_v1_params.record_type = STORAGE_ALL;
	mp4_v1_params.width  = 1280;
	mp4_v1_params.height = 720;
	mp4_v1_params.record_length = 20;//120;//10;
	mp4_v1_params.record_file_num = 3;//1;
	mp4_v1_params.gop = 30;
#else

	mp4_v1_params.fps = 25;
	mp4_v1_params.record_type = STORAGE_ALL;
	mp4_v1_params.width  = 854;
	mp4_v1_params.height = 480;
	mp4_v1_params.record_length = 20;//120;//10;
	mp4_v1_params.record_file_num = 3;//1;
	mp4_v1_params.gop = 25;
#endif

	array_t a_array;
	a_array.data_addr = (uint32_t) pcmu_sample;
	a_array.data_len = (uint32_t) pcmu_sample_size;
	array_a_ctx = mm_module_open(&array_module);
	if (array_a_ctx) {
		mm_module_ctrl(array_a_ctx, CMD_ARRAY_SET_PARAMS, (int)&pcmu_array_params);
		mm_module_ctrl(array_a_ctx, CMD_ARRAY_SET_ARRAY, (int)&a_array);
		mm_module_ctrl(array_a_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(array_a_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(array_a_ctx, CMD_ARRAY_APPLY, 0);
		mm_module_ctrl(array_a_ctx, CMD_ARRAY_STREAMING, 1);	// streamming on
	} else {
		rt_printf("ARRAY open fail\n\r");
		goto mmf2_video_example_h264_pcmu_array_mp4_init_fail;
	}

	if (mp4_ctx) {
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_PARAMS, (int)&mp4_v1_params);
		mm_module_ctrl(mp4_ctx, CMD_MP4_START, mp4_v1_params.record_file_num);
		//mm_module_ctrl(mp4_ctx, CMD_MP4_SET_STOP_CB,(int)mp4_stop_cb);
		//mm_module_ctrl(mp4_ctx, CMD_MP4_SET_END_CB,(int)mp4_end_cb);
		//mm_module_ctrl(mp4_ctx, MM_CMD_SET_QUEUE_LEN, 3);
		//mm_module_ctrl(mp4_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
	} else {
		rt_printf("MP4 open fail\n\r");
		goto mmf2_video_example_h264_pcmu_array_mp4_init_fail;
	}

	rt_printf("MP4 opened\n\r");


	//--------------Link---------------------------
	miso_array_mp4 = miso_create();
	if (miso_array_mp4) {
		miso_ctrl(miso_array_mp4, MMIC_CMD_ADD_INPUT0, (uint32_t)array_ctx, 0);
		miso_ctrl(miso_array_mp4, MMIC_CMD_ADD_INPUT1, (uint32_t)array_a_ctx, 0);
		miso_ctrl(miso_array_mp4, MMIC_CMD_ADD_OUTPUT, (uint32_t)mp4_ctx, 0);
		miso_start(miso_array_mp4);
	} else {
		rt_printf("miso_array_mp4 open fail\n\r");
		goto mmf2_video_example_h264_pcmu_array_mp4_init_fail;
	}
	rt_printf("miso_array_mp4 started\n\r");

	atcmd_userctrl_init();
	return;
mmf2_video_example_h264_pcmu_array_mp4_init_fail:

	return;
}

static const char *example = "mmf2_video_example_h264_pcmu_array_mp4_init";
static void example_deinit(void)
{
	//Pause Linker
	miso_pause(miso_array_mp4, MM_OUTPUT);

	//Stop module
	mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 0);
	mm_module_ctrl(array_a_ctx, CMD_ARRAY_STREAMING, 0);
	mm_module_ctrl(mp4_ctx, CMD_MP4_STOP, 0);

	//Delete linker
	miso_delete(miso_array_mp4);

	//Close module
	array_ctx = mm_module_close(array_ctx);
	array_a_ctx = mm_module_close(array_a_ctx);
	mp4_ctx = mm_module_close(mp4_ctx);

}

static void fUC(void *arg)
{
	static uint32_t user_cmd = 0;

	if (!strcmp(arg, "TD")) {
		if (user_cmd & USR_CMD_EXAMPLE_DEINIT) {
			printf("invalid state, can not do %s deinit!\r\n", example);
		} else {
			example_deinit();
			user_cmd = USR_CMD_EXAMPLE_DEINIT;
			printf("deinit %s\r\n", example);
		}
	} else if (!strcmp(arg, "TSR")) {
		if (user_cmd & USR_CMD_EXAMPLE_DEINIT) {
			printf("reinit %s\r\n", example);
			sys_reset();
		} else {
			printf("invalid state, can not do %s init!\r\n", example);
		}
	} else {
		printf("invalid cmd");
	}

	printf("user command 0x%x\r\n", user_cmd);
}

static log_item_t userctrl_items[] = {
	{"UC", fUC, },
};

static void atcmd_userctrl_init(void)
{
	log_service_add_table(userctrl_items, sizeof(userctrl_items) / sizeof(userctrl_items[0]));
}