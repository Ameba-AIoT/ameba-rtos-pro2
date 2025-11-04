#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "module_video.h"
#include "module_streaming.h"
#include "module_audio.h"
#include "module_i2s.h"
#include "module_aac.h"

#include "log_service.h"
#include "sensor.h"
#include "ai_glass_media.h"
#include "media_filesystem.h"
#include "ai_glass_dbg.h"

#define V1_CHANNEL 1
#define V1_BPS 256*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR
#define VIDEO_TYPE VIDEO_H264
#define VIDEO_CODEC AV_CODEC_ID_H264
#define AUDIO_CODEC AV_CODEC_ID_MP4A_LATM
// // Definition of the audio interfcae
// #define I2S_INTERFACE           0
// #define AUDIO_INTERFACE         1
#define VIDEO_AUDIO             0  
// Configuration
#define AUDIO_SAMPLE_RATE       16000 // 48000
#define AUDIO_SRC               I2S_INTERFACE //AUDIO_INTERFACE
#define AUDIO_I2S_ROLE          I2S_SLAVE //I2S_MASTER

static mm_context_t *ls_video_ctx           = NULL;
static mm_context_t *lifetime_streaming_ctx = NULL;
static mm_context_t *ls_audio_ctx = NULL;
static mm_context_t *ls_aac_ctx = NULL;

//Linkers
#if VIDEO_AUDIO
static mm_siso_t *ls_siso_audio_aac = NULL;
static mm_miso_t *miso_ai_streaming = NULL;
#else 
static mm_siso_t *siso_ai_streaming = NULL;
#endif

#if VIDEO_AUDIO
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
#endif

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.bps = V1_BPS,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1,
    .level = VCENC_H264_LEVEL_4,
	.profile = VCENC_H264_BASE_PROFILE,
	.cavlc = 1
};

static int end_streaming_cb(void) {

    AI_GLASS_INFO("close streaming.....\n");
	current_state = STATE_IDLE;
    lifetime_streaming_deinitialize();
    return 0;
}

int lifetime_streaming_initialize(void) {
    ai_glass_stream_param_t *stream_param = NULL;
    stream_param = (ai_glass_stream_param_t *) malloc(sizeof(ai_glass_stream_param_t));
	memset(stream_param, 0x00, sizeof(ai_glass_stream_param_t));
	media_get_stream_params(stream_param);
    
    AI_GLASS_MSG("==================================== Width : %d, Height: %d, FPS: %d==========================\n",stream_param->width, stream_param->height, stream_param->fps);
    video_v1_params.resolution = stream_param->resolution;
    video_v1_params.bps = stream_param->bps;
	video_v1_params.width = stream_param->width;
	video_v1_params.height = stream_param->height;
	video_v1_params.fps = stream_param->fps;
	video_v1_params.gop = stream_param->fps;
	video_v1_params.rc_mode = stream_param->rc_mode;
    video_v1_params.rc_mode = stream_param->rc_mode;
    video_v1_params.rotation = stream_param->rotation;

#if VIDEO_AUDIO
#if AUDIO_SRC==AUDIO_INTERFACE
	ls_audio_ctx = mm_module_open(&audio_module);
	memcpy((void *)&audio_params, (void *)&default_audio_params, sizeof(audio_params_t));
	if (ls_audio_ctx) {
		mm_module_ctrl(ls_audio_ctx, CMD_AUDIO_GET_PARAMS, (int)&audio_params);
		audio_params.sample_rate = audio_samplerate2index(AUDIO_SAMPLE_RATE);
		mm_module_ctrl(ls_audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
		mm_module_ctrl(ls_audio_ctx, MM_CMD_SET_QUEUE_LEN, 6); //queue size can be smaller 160ms
		mm_module_ctrl(ls_audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
	} else {
		AI_GLASS_ERR("audio open fail\n\r");
		goto lifetime_streaming_initialize_fail;
	}
#elif AUDIO_SRC==I2S_INTERFACE
	ls_audio_ctx = mm_module_open(&i2s_module);
	if (ls_audio_ctx) {
		mm_module_ctrl(ls_audio_ctx, CMD_I2S_GET_PARAMS, (int)&i2s_params);
		i2s_params.sample_rate = i2s_samplerate2index(AUDIO_SAMPLE_RATE);
		i2s_params.i2s_direction = I2S_RX_ONLY;
		i2s_params.i2s_role = AUDIO_I2S_ROLE;
		//i2s_params.pin_group_num = 0;
		mm_module_ctrl(ls_audio_ctx, CMD_I2S_SET_PARAMS, (int)&i2s_params);
		mm_module_ctrl(ls_audio_ctx, MM_CMD_SET_QUEUE_LEN, 6); //queue size can be smaller 160ms
		mm_module_ctrl(ls_audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
	} else {
		AI_GLASS_ERR("i2s open fail\n\r");
		goto lifetime_streaming_initialize_fail;
	}
#endif

	ls_aac_ctx = mm_module_open(&aac_module);
	if (ls_aac_ctx) {
		mm_module_ctrl(ls_aac_ctx, CMD_AAC_SET_PARAMS, (int)&aac_params);
		mm_module_ctrl(ls_aac_ctx, MM_CMD_SET_QUEUE_LEN, 30);
		mm_module_ctrl(ls_aac_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(ls_aac_ctx, CMD_AAC_INIT_MEM_POOL, 0);
		mm_module_ctrl(ls_aac_ctx, CMD_AAC_APPLY, 0);
	} else {
		AI_GLASS_ERR("AAC open fail\n\r");
		goto lifetime_streaming_initialize_fail;
	}
#endif

    // Initialize video module
    video_pre_init_params_t ai_glass_pre_init_params = {0};
    ls_video_ctx = mm_module_open(&video_module);
    if (ls_video_ctx) {
        mm_module_ctrl(ls_video_ctx, CMD_VIDEO_GET_PRE_INIT_PARM, (int)&ai_glass_pre_init_params);
		// Init ISP parameters
		ai_glass_pre_init_params.isp_init_enable = 1;
		ai_glass_pre_init_params.init_isp_items.init_brightness = 0;
		ai_glass_pre_init_params.init_isp_items.init_contrast = 50;
		ai_glass_pre_init_params.init_isp_items.init_flicker = 1;
		ai_glass_pre_init_params.init_isp_items.init_hdr_mode = 0;
		//ai_glass_pre_init_params.init_isp_items.init_mirrorflip = 0x02; // flip and mirror
		ai_glass_pre_init_params.init_isp_items.init_saturation = 50;
		ai_glass_pre_init_params.init_isp_items.init_wdr_level = 50;
		ai_glass_pre_init_params.init_isp_items.init_wdr_mode = 2;
		ai_glass_pre_init_params.init_isp_items.init_mipi_mode = 0;
		mm_module_ctrl(ls_video_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&ai_glass_pre_init_params);
        mm_module_ctrl(ls_video_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
        mm_module_ctrl(ls_video_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps * 10);
        mm_module_ctrl(ls_video_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
    } else {
        AI_GLASS_ERR("Video module open failed\n");
        goto lifetime_streaming_initialize_fail;
    }
    
    lifetime_streaming_ctx = mm_module_open(&streaming_module);  // Your custom streaming module
    if (lifetime_streaming_ctx) {
        mm_module_ctrl(lifetime_streaming_ctx, CMD_STREAMING_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(lifetime_streaming_ctx, CMD_STREAMING_SET_APPLY, 0);
		mm_module_ctrl(lifetime_streaming_ctx, CMD_STREAMING_SET_STREAMING, ON);
        mm_module_ctrl(lifetime_streaming_ctx, CMD_STREAMING_SET_STOP_CB, (int)end_streaming_cb);
    } else {
        AI_GLASS_ERR("AI streaming module open failed\n");
        goto lifetime_streaming_initialize_fail;
    }
#if VIDEO_AUDIO
    ls_siso_audio_aac = siso_create();
	if (ls_siso_audio_aac) {
		siso_ctrl(ls_siso_audio_aac, MMIC_CMD_ADD_INPUT, (uint32_t)ls_audio_ctx, 0);
		siso_ctrl(ls_siso_audio_aac, MMIC_CMD_ADD_OUTPUT, (uint32_t)ls_aac_ctx, 0);
		siso_ctrl(ls_siso_audio_aac, MMIC_CMD_SET_TASKPRIORITY, 4, 0);
		siso_ctrl(ls_siso_audio_aac, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(ls_siso_audio_aac);
	} else {
		AI_GLASS_ERR("lr siso audio aac open fail\n\r");
		goto lifetime_streaming_initialize_fail;
	}
#endif

#if VIDEO_AUDIO   
    miso_ai_streaming = miso_create();
	if (miso_ai_streaming) {
		miso_ctrl(miso_ai_streaming, MMIC_CMD_ADD_INPUT0, (uint32_t)ls_video_ctx, 0);
		miso_ctrl(miso_ai_streaming, MMIC_CMD_ADD_INPUT1, (uint32_t)ls_aac_ctx, 0);
		miso_ctrl(miso_ai_streaming, MMIC_CMD_ADD_OUTPUT0, (uint32_t)lifetime_streaming_ctx, 0);
		miso_ctrl(miso_ai_streaming, MMIC_CMD_SET_TASKPRIORITY, 4, 0);
		miso_start(miso_ai_streaming);
	} else {
		AI_GLASS_ERR("miso open fail for video streaming\n\r");
		goto lifetime_streaming_initialize_fail;
	}

#if AUDIO_SRC==AUDIO_INTERFACE
	mm_module_ctrl(ls_audio_ctx, CMD_AUDIO_APPLY, 0);
#elif AUDIO_SRC==I2S_INTERFACE
	mm_module_ctrl(ls_audio_ctx, CMD_I2S_APPLY, 0);
#endif

#else
	siso_ai_streaming = siso_create();
	if (siso_ai_streaming) {
		siso_ctrl(siso_ai_streaming, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);

		siso_ctrl(siso_ai_streaming, MMIC_CMD_ADD_INPUT, (uint32_t)ls_video_ctx, 0);
		siso_ctrl(siso_ai_streaming, MMIC_CMD_ADD_OUTPUT, (uint32_t)lifetime_streaming_ctx, 0);
		siso_start(siso_ai_streaming);
	} else {
		AI_GLASS_ERR("siso2 open fail\n\r");
		goto lifetime_streaming_initialize_fail;
	}
#endif
	current_state = STATE_STREAMING;
	AI_GLASS_INFO("miso(videochn0_aac_streaming) started\n\r");
	mm_module_ctrl(ls_video_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 1

	return 0;
lifetime_streaming_initialize_fail:
	if (stream_param) {
		free(stream_param);
	}
	lifetime_streaming_deinitialize();
	return -1;
}

void lifetime_streaming_deinitialize(void) {
    AI_GLASS_INFO("=============lifetime_streaming_deinitialize================\n\r");

    //Pause individual audio
#if VIDEO_AUDIO       
    if (ls_siso_audio_aac) {
        siso_pause(ls_siso_audio_aac);
    }

    // Pause linker
    if (miso_ai_streaming) {
        miso_pause(miso_ai_streaming, MM_OUTPUT0);
    }

#if AUDIO_SRC==AUDIO_INTERFACE
	if (ls_audio_ctx != NULL) {
		mm_module_ctrl(ls_audio_ctx, CMD_AUDIO_SET_TRX, 0);
	}
#else
	if (ls_audio_ctx != NULL) {
		mm_module_ctrl(ls_audio_ctx, CMD_I2S_SET_TRX, 0);
	}
#endif
	if (ls_aac_ctx != NULL) {
		mm_module_ctrl(ls_aac_ctx, CMD_AAC_STOP, 0);
	}
#else
	if (siso_ai_streaming) {
        siso_pause(siso_ai_streaming);
    }
#endif
    // Stop modules
    if (ls_video_ctx) {
        mm_module_ctrl(ls_video_ctx, CMD_VIDEO_STREAM_STOP, video_v1_params.stream_id);
    }

    if (lifetime_streaming_ctx) {
        mm_module_ctrl(lifetime_streaming_ctx, CMD_STREAMING_SET_STREAMING, OFF);
    }

    // Delete linker
#if VIDEO_AUDIO
    if (ls_siso_audio_aac) {
        siso_delete(ls_siso_audio_aac);
	    ls_siso_audio_aac = NULL;
    }

    if (miso_ai_streaming) {
        miso_delete(miso_ai_streaming);
        miso_ai_streaming = NULL;
    }
#else
    if (siso_ai_streaming) {
        siso_delete(siso_ai_streaming);
        siso_ai_streaming = NULL;
    }
#endif

    // Close modules
#if VIDEO_AUDIO	
    if (ls_audio_ctx) {
        mm_module_close(ls_audio_ctx);
	    ls_audio_ctx = NULL;
    }
    
    if (ls_aac_ctx) {
        mm_module_close(ls_aac_ctx);
	    ls_aac_ctx = NULL;
    }
#endif	
    if (lifetime_streaming_ctx) {
        mm_module_close(lifetime_streaming_ctx);
        lifetime_streaming_ctx = NULL;
    }

    if (ls_video_ctx) {
        mm_module_close(ls_video_ctx);
        ls_video_ctx = NULL;
    }

	current_state = STATE_IDLE;
    AI_GLASS_INFO("AI streaming deinitialized successfully\n\r");
}
