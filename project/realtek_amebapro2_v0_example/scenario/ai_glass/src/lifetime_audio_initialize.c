#include "platform_opts.h"
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "module_audio.h"
#include "module_i2s.h"
#include "module_opusc.h"
#include "module_mp4.h"
#include "log_service.h"
#include "sensor.h"
#include "isp_ctrl_api.h"
#include "ai_glass_media.h"
#include "media_filesystem.h"
#include "ai_glass_dbg.h"
#include "ai_glass_media.h"

#define AUDIO_SAMPLE_RATE       16000
#define AUDIO_SRC               I2S_INTERFACE // or AUDIO_INTERFACE
#define AUDIO_I2S_ROLE          I2S_SLAVE

// Modules
static mm_context_t *lr_audio_ctx = NULL;
static mm_context_t *lr_opusc_ctx = NULL;
static mm_context_t *lr_mp4_ctx = NULL;

// Linker
static mm_siso_t *lr_siso_audio_opusc = NULL;
static mm_siso_t *lr_siso_opusc_mp4 = NULL;

static char life_recording_name[128] = {0};
static const char *example = "ai_glass_lifetime_audio_recording";

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

static opusc_params_t opusc_rtsp_params = {
	.sample_rate = AUDIO_SAMPLE_RATE,
	.channel = 1,
	.bit_length = 16,			//16 recommand
	.complexity = 5,			//0~10
	.bitrate = 25000,			//default 25000
	.use_framesize = 40,		//needs to the same or bigger than AUDIO_DMA_PAGE_SIZE/(sample_rate/1000)/2 but less than 60
	.enable_vbr = 1,
	.vbr_constraint = 0,
	.packetLossPercentage = 0,
	.opus_application = OPUS_APPLICATION_AUDIO

};

static mp4_params_t lr_mp4_params = {
	.sample_rate = AUDIO_SAMPLE_RATE,
	.channel     = 1,

	.record_length = 60, //seconds
	.record_type = STORAGE_AUDIO,
	.record_file_num = 1,
	.record_file_name = "AmebaPro_audio_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
	.append_header = 0,
};

// current_state = STATE_IDLE;

extern void mp4_send_response_callback(void *parm);

static int lr_mp4_end_cb(void *parm)
{
    AI_GLASS_INFO("Record end\r\n");

    // Save file count list to external disk
    extdisk_save_file_cntlist();

    // Update state machine
    current_state = STATE_END_RECORDING;

    return 0;
}

static int lr_mp4_stop_cb(void *parm)
{
    AI_GLASS_INFO("Record stop\r\n");

    // Update state machine
    current_state = STATE_IDLE;

    // If you want to get timestamp when stopped, you can keep this (optional)
    // get_mp4_video_timestamp();

    return 0;
}

static int lr_mp4_error_cb(void *parm)
{
    AI_GLASS_ERR("Lifetime recording error\r\n");

    // Update state machine
    current_state = STATE_ERROR;

    return 0;
}


int lifetime_audio_initialize(uint8_t record_filename_length, const char *filename)
{
	AI_GLASS_INFO("=== LifeTime Audio Record Start ===\n\r");
	uint8_t recording_filename_length = record_filename_length;
	// Generate recording name
	if (record_filename_length == 0) {
		char *cur_time_str = (char *)media_filesystem_get_current_time_string();
		extdisk_generate_unique_filename("AUDIO_0_0_", cur_time_str, ".mp4", life_recording_name, 128);
		free(cur_time_str);
	} else {
		extdisk_generate_unique_filename("", filename, ".mp4", life_recording_name, 128);
	}

	// Pre-create MP4 file entry
	char life_record_name[128] = {0};
	snprintf(life_record_name, sizeof(life_record_name), "%s.mp4", life_recording_name);
	FILE *life_record_mp4 = extdisk_fopen(life_record_name, "w");
	extdisk_fclose(life_record_mp4);

	// ====== Initialize Modules ======
#if AUDIO_SRC==AUDIO_INTERFACE
	lr_audio_ctx = mm_module_open(&audio_module);
	memcpy((void *)&audio_params, (void *)&default_audio_params, sizeof(audio_params_t));
	if (lr_audio_ctx) {
		mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_GET_PARAMS, (int)&audio_params);
		audio_params.sample_rate = audio_samplerate2index(AUDIO_SAMPLE_RATE);
		mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
		mm_module_ctrl(lr_audio_ctx, MM_CMD_SET_QUEUE_LEN, 6); //queue size can be smaller 160ms
		mm_module_ctrl(lr_audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
	} else {
		AI_GLASS_ERR("audio open fail\n\r");
		goto lifetime_audio_initialize_fail;
	}
#elif AUDIO_SRC==I2S_INTERFACE
	lr_audio_ctx = mm_module_open(&i2s_module);
	if (lr_audio_ctx) {
		mm_module_ctrl(lr_audio_ctx, CMD_I2S_GET_PARAMS, (int)&i2s_params);
		i2s_params.sample_rate = i2s_samplerate2index(AUDIO_SAMPLE_RATE);
		i2s_params.i2s_direction = I2S_RX_ONLY;
		i2s_params.i2s_role = AUDIO_I2S_ROLE;
		//i2s_params.pin_group_num = 0;
		mm_module_ctrl(lr_audio_ctx, CMD_I2S_SET_PARAMS, (int)&i2s_params);
		mm_module_ctrl(lr_audio_ctx, MM_CMD_SET_QUEUE_LEN, 6); //queue size can be smaller 160ms
		mm_module_ctrl(lr_audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
	} else {
		AI_GLASS_ERR("i2s open fail\n\r");
		goto lifetime_audio_initialize_fail;
	}
#endif

	lr_opusc_ctx = mm_module_open(&opusc_module);
		if (lr_opusc_ctx) {
			mm_module_ctrl(lr_opusc_ctx, CMD_OPUSC_SET_PARAMS, (int)&opusc_rtsp_params);
			mm_module_ctrl(lr_opusc_ctx, MM_CMD_SET_QUEUE_LEN, 6);
			mm_module_ctrl(lr_opusc_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
			mm_module_ctrl(lr_opusc_ctx, CMD_OPUSC_APPLY, 0);
		} else {
			printf("OPUSC open fail\n\r");
			goto lifetime_audio_initialize_fail;
	}

	lr_mp4_ctx = mm_module_open(&mp4_module);
	lr_mp4_params.mp4_audio_format = AUDIO_OPUS;
	lr_mp4_params.mp4_audio_duration = 40;
	
    if (lr_mp4_ctx) {
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_PARAMS, (int)&lr_mp4_params);
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_LOOP_MODE, 0);
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_STOP_CB, (int)lr_mp4_stop_cb);
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_END_CB, (int)lr_mp4_end_cb);
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_ERROR_CB, (int)lr_mp4_error_cb);
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_RECORD_FILE_NAME, (int)life_recording_name);
		//char *extdisk_tag = extdisk_get_filesystem_tag_name();
		//if (extdisk_tag) {
		//mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_VFS_EXDISK_TAG, (int)extdisk_tag);
		//free(extdisk_tag);
		//}
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_START, lr_mp4_params.record_file_num);
	} else {
		AI_GLASS_ERR("MP4 open fail\n\r");
		goto lifetime_audio_initialize_fail;
	}

	AI_GLASS_INFO("MP4 opened\n\r");

	lr_siso_audio_opusc = siso_create();
	if (lr_siso_audio_opusc) {

		siso_ctrl(lr_siso_audio_opusc, MMIC_CMD_ADD_INPUT, (uint32_t)lr_audio_ctx, 0);
		siso_ctrl(lr_siso_audio_opusc, MMIC_CMD_ADD_OUTPUT, (uint32_t)lr_opusc_ctx, 0);
		siso_ctrl(lr_siso_audio_opusc, MMIC_CMD_SET_STACKSIZE, 24 * 1024, 0);
		siso_start(lr_siso_audio_opusc);
	} else {
		AI_GLASS_ERR("lr siso audio opus open fail\n\r");
		goto lifetime_audio_initialize_fail;
	}

	lr_siso_opusc_mp4 = siso_create();
    if (lr_siso_opusc_mp4) {
        siso_ctrl(lr_siso_opusc_mp4, MMIC_CMD_ADD_INPUT, (uint32_t)lr_opusc_ctx, 0);
        siso_ctrl(lr_siso_opusc_mp4, MMIC_CMD_ADD_OUTPUT, (uint32_t)lr_mp4_ctx, 0);
        siso_ctrl(lr_siso_opusc_mp4, MMIC_CMD_SET_TASKPRIORITY, 4, 0);
        siso_start(lr_siso_opusc_mp4);
    } else {
		AI_GLASS_ERR("siso open fail for adui recording\n\r");
		goto lifetime_audio_initialize_fail;
	}
	AI_GLASS_INFO("Audio recording pipeline started -> %s\n\r", life_recording_name);

    current_state = STATE_RECORDING;
#if AUDIO_SRC==AUDIO_INTERFACE
	mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_APPLY, 0);
#elif AUDIO_SRC==I2S_INTERFACE
	mm_module_ctrl(lr_audio_ctx, CMD_I2S_APPLY, 0);
#endif
	return 0;

lifetime_audio_initialize_fail:
	
	lifetime_audio_deinitialize();
	return -1;
}

void lifetime_audio_deinitialize(void)
{
    siso_pause(lr_siso_audio_opusc);

	//Pause MP4 recording / RTSP Stream
	siso_pause(lr_siso_opusc_mp4);

#if AUDIO_SRC==AUDIO_INTERFACE
	if (lr_audio_ctx != NULL) {
		mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_SET_TRX, 0);
	}
#else
	if (lr_audio_ctx != NULL) {
		mm_module_ctrl(lr_audio_ctx, CMD_I2S_SET_TRX, 0);
	}
#endif
    if (lr_opusc_ctx != NULL) {
		mm_module_ctrl(lr_opusc_ctx, CMD_OPUSC_STOP, 0);
	}
    if (lr_mp4_ctx) {
        mm_module_ctrl(lr_mp4_ctx, CMD_MP4_STOP_IMMEDIATELY, 0);
    }
	siso_delete(lr_siso_opusc_mp4);
    lr_siso_opusc_mp4 = NULL;
	siso_delete(lr_siso_audio_opusc);
    lr_siso_audio_opusc = NULL;
    
    //Close module  
	mm_module_close(lr_audio_ctx);
	lr_audio_ctx = NULL;
	mm_module_close(lr_opusc_ctx);
	lr_opusc_ctx = NULL;
	mm_module_close(lr_mp4_ctx);
	lr_mp4_ctx = NULL;

    current_state = STATE_IDLE;
	AI_GLASS_INFO("Audio recording stopped.\n\r");
}
