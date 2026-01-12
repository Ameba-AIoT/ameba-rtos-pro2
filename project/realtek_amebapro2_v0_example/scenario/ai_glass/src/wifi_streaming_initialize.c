#include "platform_opts.h"
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "module_video.h"
#include "module_rtsp2.h"
#include "module_audio.h"
#include "module_i2s.h"
#include "module_opusc.h"
#include "module_aac.h"
#include "log_service.h"
#include "sensor.h"
#include "ai_glass_dbg.h"
#include "ai_glass_media.h"


//Configuration for RTSP streaming
#define AUDIO_SAMPLE_RATE       16000 // 48000
#define AUDIO_SRC               I2S_INTERFACE //AUDIO_INTERFACE
#define AUDIO_I2S_ROLE          I2S_SLAVE //I2S_MASTER

//Modules
static mm_context_t *streaming_video_ctx = NULL;
static mm_context_t *streaming_audio_ctx = NULL;
// static mm_context_t *streaming_opusc_ctx = NULL;
static mm_context_t *streaming_aac_ctx   = NULL;
static mm_context_t *streaming_rtsp2_ctx = NULL;

//Linkers
// static mm_siso_t *siso_streaming_audio_opusc = NULL;
// static mm_miso_t *miso_streaming_video_opusc_rtsp = NULL;

static mm_siso_t *siso_streaming_audio_aac = NULL;
static mm_miso_t *miso_streaming_video_aac_rtsp = NULL;

static video_params_t streaming_video_params = {
    .stream_id = MAIN_STREAM_ID,
    .use_static_addr = 1,
	.level = VCENC_H264_LEVEL_4_1,
	.profile = VCENC_H264_MAIN_PROFILE,
	.cavlc = 1
};

#if AUDIO_SRC==AUDIO_INTERFACE
static audio_params_t audio_params;
static audio_sr audio_samplerate2index(int samplerate)
{
	switch (samplerate) {
	case 8000:
		return ASR_8KHZ;
	case 16000:
		return ASR_16KHZ;
	case 32000:
		return ASR_32KHZ;
	case 44100:
		return ASR_44p1KHZ;
	case 48000:
		return ASR_48KHZ;
	case 88200:
		return ASR_88p2KHZ;
	case 96000:
		return ASR_96KHZ;
	default:
		AI_GLASS_ERR("Invalid audio samplerate %d for audio set to default value ASR_16KHZ\n\r", samplerate);
		return ASR_16KHZ;
	}
}
#elif AUDIO_SRC==I2S_INTERFACE
static i2s_params_t i2s_params;
static int i2s_samplerate2index(int samplerate)
{
	switch (samplerate) {
	case 8000:
		return SR_8KHZ;
	case 16000:
		return SR_16KHZ;
	case 32000:
		return SR_32KHZ;
	case 44100:
		return SR_44p1KHZ;
	case 48000:
		return SR_48KHZ;
	case 88200:
		return SR_88p2KHZ;
	case 96000:
		return SR_96KHZ;
	default:
		AI_GLASS_ERR("Invalid i2s samplerate %d for i2s set to default value SR_16KHZ\n\r", samplerate);
		return SR_16KHZ;
	}
}
#endif

// static opusc_params_t opusc_rtsp_params = {
// 	.sample_rate = 16000,
// 	.channel = 1,
// 	.bit_length = 16,			//16 recommand
// 	.complexity = 5,			//0~10
// 	.bitrate = 25000,			//default 25000
// 	.use_framesize = 40,		//needs to the same or bigger than AUDIO_DMA_PAGE_SIZE/(sample_rate/1000)/2 but less than 60
// 	.enable_vbr = 1,
// 	.vbr_constraint = 0,
// 	.packetLossPercentage = 0,
// 	.opus_application = OPUS_APPLICATION_AUDIO

// };

static aac_params_t aac_params = {
	.sample_rate = AUDIO_SAMPLE_RATE,
	.channel = 1,
	.trans_type = AAC_TYPE_ADTS,
	.object_type = AAC_AOT_LC,
	.bitrate = 32000,

	.mem_total_size = 10 * 1024,
	.mem_block_size = 128,
	.mem_frame_size = 1024
};

static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.bps      = DEFAULT_STREAM_BPS
		}
	}
};

static rtsp2_params_t rtsp2_a_opus_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a = {
			.codec_id   = AV_CODEC_ID_MP4A_LATM,
			.channel    = 1,
			.samplerate = AUDIO_SAMPLE_RATE,
			// .frame_size = 40	//equal to use_framesize in opusc_rtsp_params
		}
	}
};

int wifi_streaming_initialize(void)
{ 
	ai_glass_stream_param_t *ai_stream_param = NULL;

	AI_GLASS_MSG("================wifi streaming initialize========================== = %lu\r\n", mm_read_mediatime_ms());

	ai_stream_param = (ai_glass_stream_param_t *) malloc(sizeof(ai_glass_stream_param_t));
	memset(ai_stream_param, 0x00, sizeof(ai_glass_stream_param_t));
	media_get_stream_params(ai_stream_param);

	//Update rtsp video_params
	rtsp2_v1_params.u.v.fps = ai_stream_param->fps;
	rtsp2_v1_params.u.v.bps = ai_stream_param->bps;

	//Update video_channel params
	streaming_video_params.type = ai_stream_param->type;
	streaming_video_params.bps = ai_stream_param->bps;
	streaming_video_params.fps = ai_stream_param->fps;
	streaming_video_params.gop = ai_stream_param->fps;
	streaming_video_params.width = ai_stream_param->width;
	streaming_video_params.height = ai_stream_param->height;
	streaming_video_params.rc_mode = ai_stream_param->rc_mode;
	streaming_video_params.rotation = ai_stream_param->rotation;
	
	if(streaming_video_params.type == 0) {
		rtsp2_v1_params.u.v.codec_id = AV_CODEC_ID_H265;
	}
	if(streaming_video_params.type == 1) {
		rtsp2_v1_params.u.v.codec_id = AV_CODEC_ID_H264;
	}
#if AUDIO_SRC==AUDIO_INTERFACE
	streaming_audio_ctx = mm_module_open(&audio_module);
	memcpy((void *)&audio_params, (void *)&default_audio_params, sizeof(audio_params_t));
	if (streaming_audio_ctx) {
		mm_module_ctrl(streaming_audio_ctx, CMD_AUDIO_GET_PARAMS, (int)&audio_params);
		audio_params.sample_rate = audio_samplerate2index(AUDIO_SAMPLE_RATE);
		mm_module_ctrl(streaming_audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
		mm_module_ctrl(streaming_audio_ctx, MM_CMD_SET_QUEUE_LEN, 6); //queue size can be smaller 160ms
		mm_module_ctrl(streaming_audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
	} else {
		AI_GLASS_ERR("audio open fail\n\r");
		goto wifi_streaming_initialize_fail;
	}
#elif AUDIO_SRC==I2S_INTERFACE
	streaming_audio_ctx = mm_module_open(&i2s_module);
	if (streaming_audio_ctx) {
		mm_module_ctrl(streaming_audio_ctx, CMD_I2S_GET_PARAMS, (int)&i2s_params);
		i2s_params.sample_rate = i2s_samplerate2index(AUDIO_SAMPLE_RATE);
		i2s_params.i2s_direction = I2S_RX_ONLY;
		i2s_params.i2s_role = AUDIO_I2S_ROLE;
		//i2s_params.pin_group_num = 0;
		mm_module_ctrl(streaming_audio_ctx, CMD_I2S_SET_PARAMS, (int)&i2s_params);
		mm_module_ctrl(streaming_audio_ctx, MM_CMD_SET_QUEUE_LEN, 6); //queue size can be smaller 160ms
		mm_module_ctrl(streaming_audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
	} else {
		AI_GLASS_ERR("i2s open fail\n\r");
		goto wifi_streaming_initialize_fail;
	}
#endif

	// streaming_opusc_ctx = mm_module_open(&opusc_module);
	// if(streaming_opusc_ctx) {
	// 	mm_module_ctrl(streaming_opusc_ctx, CMD_OPUSC_SET_PARAMS, (int)&opusc_rtsp_params);
	// 	mm_module_ctrl(streaming_opusc_ctx, MM_CMD_SET_QUEUE_LEN, 6);
	// 	mm_module_ctrl(streaming_opusc_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	// 	mm_module_ctrl(streaming_opusc_ctx, CMD_OPUSC_APPLY, 0);
	// } else {
	// 	AI_GLASS_ERR("OPUSC  open fail\n\r");
	// 	goto wifi_streaming_initialize_fail;
	// }

	streaming_aac_ctx = mm_module_open(&aac_module);
	if(streaming_aac_ctx) {
		mm_module_ctrl(streaming_aac_ctx, CMD_AAC_SET_PARAMS, (int)&aac_params);
		mm_module_ctrl(streaming_aac_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(streaming_aac_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(streaming_aac_ctx, CMD_AAC_INIT_MEM_POOL, 0);
		mm_module_ctrl(streaming_aac_ctx, CMD_AAC_APPLY, 0);
	} else {
		AI_GLASS_ERR("aac open fail\n\r");
		goto wifi_streaming_initialize_fail;
	}

	streaming_rtsp2_ctx = mm_module_open(&rtsp2_module);
	if (streaming_rtsp2_ctx) {
		mm_module_ctrl(streaming_rtsp2_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(streaming_rtsp2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(streaming_rtsp2_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(streaming_rtsp2_ctx, CMD_RTSP2_SELECT_STREAM, 1);
		mm_module_ctrl(streaming_rtsp2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_a_opus_params);
		mm_module_ctrl(streaming_rtsp2_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(streaming_rtsp2_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		AI_GLASS_ERR("RTSP2 open fail\n\r");
		goto wifi_streaming_initialize_fail;
	}

	AI_GLASS_MSG("RTSP2 opened\n\r");

	video_pre_init_params_t ai_glass_pre_init_params = {0};
	streaming_video_ctx = mm_module_open(&video_module);
	if (streaming_video_ctx) {
		media_get_preinit_isp_data(&ai_glass_pre_init_params);
		mm_module_ctrl(streaming_video_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&ai_glass_pre_init_params);
		mm_module_ctrl(streaming_video_ctx, CMD_VIDEO_SET_PARAMS, (int)&streaming_video_params);
		mm_module_ctrl(streaming_video_ctx, MM_CMD_SET_QUEUE_LEN, streaming_video_params.fps * 10); //Add the queue buffer to avoid to lost data.
		mm_module_ctrl(streaming_video_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		AI_GLASS_ERR("video open fail\n\r");
		goto wifi_streaming_initialize_fail;
	}


	// siso_streaming_audio_opusc = siso_create();
	// if (siso_streaming_audio_opusc) {
	// 	siso_ctrl(siso_streaming_audio_opusc, MMIC_CMD_ADD_INPUT, (uint32_t)streaming_audio_ctx, 0);
	// 	siso_ctrl(siso_streaming_audio_opusc, MMIC_CMD_ADD_OUTPUT, (uint32_t)streaming_opusc_ctx, 0);
	// 	siso_ctrl(siso_streaming_audio_opusc, MMIC_CMD_SET_TASKPRIORITY, 4, 0);
	// 	siso_ctrl(siso_streaming_audio_opusc, MMIC_CMD_SET_STACKSIZE, 24 * 1024, 0);
	// 	siso_start(siso_streaming_audio_opusc);
	// } else {
	// 	AI_GLASS_ERR("lr siso audio opus open fail\n\r");
	// 	goto wifi_streaming_initialize_fail;
	// }

	siso_streaming_audio_aac = siso_create();
	if (siso_streaming_audio_aac) {
		siso_ctrl(siso_streaming_audio_aac, MMIC_CMD_ADD_INPUT, (uint32_t)streaming_audio_ctx, 0);
		siso_ctrl(siso_streaming_audio_aac, MMIC_CMD_ADD_OUTPUT, (uint32_t)streaming_aac_ctx, 0);
		siso_ctrl(siso_streaming_audio_aac, MMIC_CMD_SET_TASKPRIORITY, 4, 0);
		siso_ctrl(siso_streaming_audio_aac, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(siso_streaming_audio_aac);
	} else {
		AI_GLASS_ERR("lr siso audio aac open fail\n\r");
		goto wifi_streaming_initialize_fail;
	}

	// miso_streaming_video_opusc_rtsp = miso_create();
	// if (miso_streaming_video_opusc_rtsp) {
	// 	miso_ctrl(miso_streaming_video_opusc_rtsp, MMIC_CMD_ADD_INPUT0, (uint32_t)streaming_video_ctx, 0);
	// 	miso_ctrl(miso_streaming_video_opusc_rtsp, MMIC_CMD_ADD_INPUT1, (uint32_t)streaming_aac_ctx, 0);
	// 	miso_ctrl(miso_streaming_video_opusc_rtsp, MMIC_CMD_ADD_OUTPUT0, (uint32_t)streaming_rtsp2_ctx, 0);
	// 	miso_ctrl(miso_streaming_video_opusc_rtsp, MMIC_CMD_SET_TASKPRIORITY, 4, 0);
	// 	miso_start(miso_streaming_video_opusc_rtsp);
	// } else {
	// 	AI_GLASS_ERR("miso open fail for video streaming\n\r");
	// 	goto wifi_streaming_initialize_fail;
	// }

	miso_streaming_video_aac_rtsp = miso_create();
	if (miso_streaming_video_aac_rtsp) {
		miso_ctrl(miso_streaming_video_aac_rtsp, MMIC_CMD_ADD_INPUT0, (uint32_t)streaming_video_ctx, 0);
		miso_ctrl(miso_streaming_video_aac_rtsp, MMIC_CMD_ADD_INPUT1, (uint32_t)streaming_aac_ctx, 0);
		miso_ctrl(miso_streaming_video_aac_rtsp, MMIC_CMD_ADD_OUTPUT0, (uint32_t)streaming_rtsp2_ctx, 0);
		miso_ctrl(miso_streaming_video_aac_rtsp, MMIC_CMD_SET_TASKPRIORITY, 4, 0);
		miso_start(miso_streaming_video_aac_rtsp);
	} else {
		AI_GLASS_ERR("miso open fail for video streaming\n\r");
		goto wifi_streaming_initialize_fail;
	}

	AI_GLASS_INFO("miso(videochn1_aac_rtsp2) started\n\r");

#if AUDIO_SRC==AUDIO_INTERFACE
	mm_module_ctrl(streaming_audio_ctx, CMD_AUDIO_APPLY, 0);
#elif AUDIO_SRC==I2S_INTERFACE
	mm_module_ctrl(streaming_audio_ctx, CMD_I2S_APPLY, 0);
#endif
	mm_module_ctrl(streaming_video_ctx, CMD_VIDEO_APPLY, streaming_video_params.stream_id);

	if (ai_stream_param) {
		free(ai_stream_param);
	}
	return 0;
	wifi_streaming_initialize_fail:
	if (ai_stream_param) {
		free(ai_stream_param);
	}
	wifi_streaming_deinitialize();
	return -1;
}	


void wifi_streaming_deinitialize(void) {

	//Pause Linker
	// miso_pause(miso_streaming_video_opusc_rtsp, MM_OUTPUT);
	// siso_pause(siso_streaming_audio_opusc);

	miso_pause(miso_streaming_video_aac_rtsp, MM_OUTPUT);
	siso_pause(siso_streaming_audio_aac);

	//Stop module
	mm_module_ctrl(streaming_rtsp2_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(streaming_video_ctx, CMD_VIDEO_STREAM_STOP, streaming_video_params.stream_id);
	mm_module_ctrl(streaming_audio_ctx, CMD_AUDIO_SET_TRX, 0);
	// mm_module_ctrl(streaming_opusc_ctx, CMD_OPUSC_STOP, 0);
	mm_module_ctrl(streaming_aac_ctx, CMD_AAC_STOP, 0);

	//Delete linker
	// miso_delete(miso_streaming_video_opusc_rtsp);
	// siso_delete(siso_streaming_audio_opusc);

	miso_delete(miso_streaming_video_aac_rtsp);
	siso_delete(siso_streaming_audio_aac);

	//Close module
	mm_module_close(streaming_rtsp2_ctx);
	mm_module_close(streaming_video_ctx);
	mm_module_close(streaming_audio_ctx);
	// mm_module_close(streaming_opusc_ctx);
	mm_module_close(streaming_aac_ctx);

	video_voe_release();
}
