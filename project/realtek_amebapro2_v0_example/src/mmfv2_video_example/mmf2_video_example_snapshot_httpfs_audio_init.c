/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"
#include "avcodec.h"

#include "module_video.h"
#include "module_audio.h"
#include "module_rtsp2.h"
#include "module_g711.h"
#include "module_rtp.h"
#include "module_array.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "module_httpfs.h"
#include "log_service.h"
#include "video_snapshot.h"
#include "sensor.h"

// three sound files
#include "close_reminder.h"  
#include "walk_reminder.h"
#include "medicine_reminder.h"
/*****************************************************************************
* ISP channel : 0
* Video type  : H264/HEVC + SNAPSHOT
*****************************************************************************/
#define ENABLE_REMINDER 1
#define AUDIO_MIX_MODE 1    // User can choose using AUDIO_MIX_MODE for the audio ring
#define V1_CHANNEL 0
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR
#define USE_H265 0
#if USE_H265
#include "sample_h265.h"
#define VIDEO_TYPE VIDEO_HEVC
#define VIDEO_CODEC AV_CODEC_ID_H265
#define SHAPSHOT_TYPE VIDEO_HEVC_JPEG
#else
#include "sample_h264.h"
#define VIDEO_TYPE VIDEO_H264
#define VIDEO_CODEC AV_CODEC_ID_H264
#define SHAPSHOT_TYPE VIDEO_H264_JPEG
#endif

//#define ENABLE_META_INFO  //Enable the marco to wirte the META data to frame
#define ENABLE_SD_SNAPSHOT //Enable the snapshot to sd card

static void atcmd_userctrl_init(void);
static void audio_doorbell_ring_reminder(void);
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v1			= NULL;
static mm_context_t *httpfs_ctx        		= NULL;
static mm_context_t *audio_ctx              = NULL;
static mm_context_t *rtsp2_ctx              = NULL;
static mm_context_t *g711e_ctx              = NULL;
static mm_context_t *g711d_ctx              = NULL;
static mm_context_t *ring_g711d_ctx         = NULL;
static mm_context_t *rtp_ctx                = NULL;
static mm_context_t *array_ctx              = NULL;

static mm_siso_t *siso_audio_g711e          = NULL;
static mm_siso_t *siso_g711_rtsp            = NULL;
static mm_siso_t *siso_ring_g711d           = NULL;
static mm_siso_t *siso_rtp_g711d            = NULL;
static mm_miso_t *miso_rtp_ring_g711d       = NULL;

static array_t array = {
    .data_addr = (uint32_t)close_pcmu_sample,
    .data_len  = (uint32_t)close_pcmu_sample_size
};

static g711_params_t g711e_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_ENCODE
};

static rtsp2_params_t rtsp2_a_pcmu_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a = {
			.codec_id   = AV_CODEC_ID_PCMU,
			.channel    = 1,
			.samplerate = 16000
		}
	}
};

static rtp_params_t rtp_g711d_params = {
	.valid_pt = 0xFFFFFFFF,
	.port = 16384,
	.frame_size = 1500,
	.cache_depth = 6
};

static g711_params_t g711d_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_DECODE
};

static array_params_t reminder_pcmu_array_params = { 
	.type = AVMEDIA_TYPE_AUDIO,
	.codec_id = AV_CODEC_ID_PCMU,
	.mode = 0,
	.u = {
		.a = {
			.channel    = 1,
			.samplerate = 16000,
			.frame_size = 640,
		}
	}
};

static audio_params_t audio_params;

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = SHAPSHOT_TYPE,
	.bps = V1_BPS,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1,
};


static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.bps      = V1_BPS
		}
	}
};


static httpfs_params_t httpfs_params = {
	.fileext = "jpg",
	.filedir = "",
	.request_string = "/image_get.jpg",
	.fatfs_buf_size = 1024
};

static TaskHandle_t snapshot_thread = NULL;

static int v1_snapshot_cb(uint32_t jpeg_addr, uint32_t jpeg_len)
{
	printf("snapshot size=%d\n\r", jpeg_len);
	return 0;
}

static void audio_play_cmd_handler(void *arg) {
	char *input_command = (char *)arg; // Received commands, such as "CLOSE", "WALK", "MEDICINE"

    if (!strcmp(input_command, "CLOSE")) {
	    array.data_addr = (uint32_t)close_pcmu_sample;
       	array.data_len = (uint32_t)close_pcmu_sample_size;
    	printf("Set audio to: CLOSE reminder\r\n");
  	} else if (!strcmp(input_command, "WALK")) {
	    array.data_addr = (uint32_t)walk_pcmu_sample;
    	array.data_len = (uint32_t)walk_pcmu_sample_size;
   	 	printf("Set audio to: WALK reminder\r\n");
	} else if (!strcmp(input_command, "MEDICINE")) {
	    array.data_addr = (uint32_t)medicine_pcmu_sample;
   		array.data_len = (uint32_t)medicine_pcmu_sample_size;
    	printf("Set audio to: MEDICINE reminder\r\n");
	} else {
	    printf("Unknown audio command: %s\r\n", input_command);
    	return; 
	}
	if (array_ctx) {
	   // Update the audio data of the Array module
	    mm_module_ctrl(array_ctx, CMD_ARRAY_SET_ARRAY, (int)&array);
	    // If you need to play immediately, you can add the start command here
	    mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 1);
	} else {
	    printf("Error: array_ctx is not initialized!\r\n");
	}
	#if ENABLE_REMINDER
		for (int i=0; i<3; i++) {
			printf("Delay 5s and then playing the audio\n\r");
			vTaskDelay(5000);
			audio_doorbell_ring_reminder();
		}
		printf("Finished playing reminder 3 times\n\r");
	#endif
}

static log_item_t audio_userctrl_items[] = {
    {"AUDIO", audio_play_cmd_handler, },
};
static void atcmd_audio_init(void) {
    log_service_add_table(audio_userctrl_items, sizeof(audio_userctrl_items) / sizeof(audio_userctrl_items[0]));
}	

#if ENABLE_REMINDER
static void audio_doorbell_ring_reminder(void)
{
	int state = 0;
	if (array_ctx) {
		mm_module_ctrl(array_ctx, CMD_ARRAY_GET_STATE, (int)&state);
		if (state) {
			printf("Audio reminder is playing\n\r");
		} else {
			printf("start the audio reminder\n\r");
#if !(AUDIO_MIX_MODE)
			miso_resume(miso_rtp_ring_g711d);
			miso_pause(miso_rtp_ring_g711d, MM_INPUT0);	// pause audio from rtp
#endif
			mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 1);	// doorbell ring

			do {	// wait until doorbell_ring done
				vTaskDelay(100);
				mm_module_ctrl(array_ctx, CMD_ARRAY_GET_STATE, (int)&state);
			} while (state == 1);
#if !(AUDIO_MIX_MODE)
			miso_resume(miso_rtp_ring_g711d);
			miso_pause(miso_rtp_ring_g711d, MM_INPUT1);	// pause array
#endif
			printf("Audio reminder done!\n\r");
		}
	}
}
#endif

#if AUDIO_MIX_MODE
static void audio_params_enable_mixmode(void)
{
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_PARAMS, (int)&audio_params);
	audio_params.mix_mode = 1;  // enable audio mix mode
	audio_params.sample_rate = ASR_16KHZ;
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
}
#endif

void audio_save_log_init(void);

#if defined(ENABLE_META_INFO)
static void video_meta_cb(void *parm)
{
	video_meta_t *m_parm = (video_meta_t *)parm;
	m_parm->user_buf = NULL;
	video_sei_write(m_parm);
}
#endif

static void snapshot_control_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif
	while (1) {
		vTaskDelay(10000);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SNAPSHOT, 1);
	}
}

static int httpfs_response_cb(void)
{
	rt_printf("httpfs response\r\n");
	return 0;
}

void mmf2_video_example_snapshot_httpfs_audio_init(void)
{
	atcmd_userctrl_init();

	/*sensor capacity check & video parameter setting*/
	video_v1_params.resolution = VIDEO_FHD;
	video_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	video_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
	video_v1_params.fps = sensor_params[USE_SENSOR].sensor_fps;
	video_v1_params.gop = sensor_params[USE_SENSOR].sensor_fps;
	/*rtsp parameter setting*/
	rtsp2_v1_params.u.v.fps = sensor_params[USE_SENSOR].sensor_fps;

#if defined(ENABLE_META_INFO)
	unsigned char uuid[16] = {0xc7, 0x98, 0x2c, 0x28, 0x0a, 0xfc, 0x49, 0xe6, 0xaa, 0xe4, 0x7f, 0x8f, 0x64, 0xee, 0x65, 0x01};
	video_pre_init_params_t init_params;
	memset(&init_params, 0x00, sizeof(video_pre_init_params_t));
	init_params.meta_enable = 1;
	init_params.meta_size = VIDEO_META_USER_SIZE;
	memcpy(init_params.video_meta_uuid, uuid, VIDEO_META_UUID_SIZE);
	video_pre_init_setup_parameters(&init_params);
	video_v1_params.meta_enable = 1;
#endif
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, video_v1_params.width, video_v1_params.height, V1_BPS, 1,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 1, NULL, 0, NULL, 0, NULL);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
#if AUDIO_MIX_MODE
		audio_params_enable_mixmode();
#endif
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		rt_printf("audio open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;
	}

    g711e_ctx = mm_module_open(&g711_module);
	if (g711e_ctx) {
		mm_module_ctrl(g711e_ctx, CMD_G711_SET_PARAMS, (int)&g711e_params);
		mm_module_ctrl(g711e_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711e_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711e_ctx, CMD_G711_APPLY, 0);
	} else {
		rt_printf("G711 open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;
	}

	//--------------RTSP---------------
	rtsp2_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_ctx) {
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_a_pcmu_params);
		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_APPLY, 0);

		mm_module_ctrl(rtsp2_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;
	}

	//--------------Link---------------------------
	siso_audio_g711e = siso_create();
	if (siso_audio_g711e) {
		siso_ctrl(siso_audio_g711e, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_g711e, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711e_ctx, 0);
		siso_start(siso_audio_g711e);
	} else {
		rt_printf("siso_audio_g711e open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;
	}

	rt_printf("siso_audio_g711e started\n\r");

    	siso_g711_rtsp = siso_create();
	if (siso_g711_rtsp) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_g711_rtsp, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_g711_rtsp, MMIC_CMD_ADD_INPUT, (uint32_t)g711e_ctx, 0);
		siso_ctrl(siso_g711_rtsp, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_ctx, 0);
		siso_start(siso_g711_rtsp);
	} else {
		rt_printf("siso_g711_rtsp fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;;
	}
	rt_printf("siso_g711_rtsp started\n\r");

	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SNAPSHOT, 0);
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;;
	}

	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;
	}

	//--------------HTTP File Server---------------
	httpfs_ctx = mm_module_open(&httpfs_module);
	if (httpfs_ctx) {
		mm_module_ctrl(httpfs_ctx, CMD_HTTPFS_SET_PARAMS, (int)&httpfs_params);
		mm_module_ctrl(httpfs_ctx, CMD_HTTPFS_SET_RESPONSE_CB, (int)httpfs_response_cb);
		mm_module_ctrl(httpfs_ctx, CMD_HTTPFS_APPLY, 0);
	} else {
		rt_printf("HTTPFS open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;;
	}

	siso_video_rtsp_v1 = siso_create();
	if (siso_video_rtsp_v1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v1_ctx, 0);
		siso_start(siso_video_rtsp_v1);
	} else {
		rt_printf("siso2 open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;;
	}

    rtp_ctx = mm_module_open(&rtp_module);
	if (rtp_ctx) {
		mm_module_ctrl(rtp_ctx, CMD_RTP_SET_PARAMS, (int)&rtp_g711d_params);
		mm_module_ctrl(rtp_ctx, MM_CMD_SET_QUEUE_LEN, 16);
		mm_module_ctrl(rtp_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(rtp_ctx, CMD_RTP_APPLY, 0);
		mm_module_ctrl(rtp_ctx, CMD_RTP_STREAMING, 1);	// streamming on
	} else {
		rt_printf("RTP open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;;
	}

    array_ctx = mm_module_open(&array_module);
	if (array_ctx) {
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_PARAMS, (int)&reminder_pcmu_array_params);
		mm_module_ctrl(array_ctx, CMD_ARRAY_SET_ARRAY, (int)&array);
		mm_module_ctrl(array_ctx, MM_CMD_SET_QUEUE_LEN, 16);
		mm_module_ctrl(array_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(array_ctx, CMD_ARRAY_APPLY, 0);
		//mm_module_ctrl(array_ctx, CMD_ARRAY_STREAMING, 1);	// streamming on
	} else {
		rt_printf("ARRAY open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;;
	}

    	g711d_ctx = mm_module_open(&g711_module);
	if (g711d_ctx) {
		mm_module_ctrl(g711d_ctx, CMD_G711_SET_PARAMS, (int)&g711d_params);
		mm_module_ctrl(g711d_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711d_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711d_ctx, CMD_G711_APPLY, 0);
	} else {
		rt_printf("G711 open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;;
	}

	ring_g711d_ctx = mm_module_open(&g711_module);
	if (ring_g711d_ctx) {
		mm_module_ctrl(ring_g711d_ctx, CMD_G711_SET_PARAMS, (int)&g711d_params);
		mm_module_ctrl(ring_g711d_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(ring_g711d_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(ring_g711d_ctx, CMD_G711_APPLY, 0);
	} else {
		rt_printf("G711 open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;;
	}

	siso_rtp_g711d = siso_create();
	if (siso_rtp_g711d) {
		siso_ctrl(siso_rtp_g711d, MMIC_CMD_ADD_INPUT, (uint32_t)rtp_ctx, 0);
		siso_ctrl(siso_rtp_g711d, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711d_ctx, 0);
		siso_start(siso_rtp_g711d);
	} else {
		rt_printf("siso_ring_g711d open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;;
	}

	siso_ring_g711d = siso_create();
	if (siso_ring_g711d) {
		siso_ctrl(siso_ring_g711d, MMIC_CMD_ADD_INPUT, (uint32_t)array_ctx, 0);
		siso_ctrl(siso_ring_g711d, MMIC_CMD_ADD_OUTPUT, (uint32_t)ring_g711d_ctx, 0);
		siso_start(siso_ring_g711d);
	} else {
		rt_printf("siso_ring_g711d open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;;
	}

	miso_rtp_ring_g711d = miso_create();
	if (miso_rtp_ring_g711d) {
		miso_ctrl(miso_rtp_ring_g711d, MMIC_CMD_ADD_INPUT0, (uint32_t)g711d_ctx, 0);
		miso_ctrl(miso_rtp_ring_g711d, MMIC_CMD_ADD_INPUT1, (uint32_t)ring_g711d_ctx, 0);
		miso_ctrl(miso_rtp_ring_g711d, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
		miso_start(miso_rtp_ring_g711d);
	} else {
		rt_printf("miso_rtp_ring_g711d open fail\n\r");
		goto mmf2_video_example_snapshot_httpfs_audio_init_fail;;
	}
	rt_printf("miso_rtp_ring_g711d started\n\r");
    atcmd_audio_init();
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);

    

	//--------------snapshot setting---------------------------
#if defined(ENABLE_SD_SNAPSHOT)
	extern snapshot_user_config_t snap_config;
	memset(&snap_config, 0x00, sizeof(snap_config));
	snapshot_vfs_init();
	snap_config.video_snapshot_ctx = video_v1_ctx;
	snap_config.snapshot_write = snapshot_write_picture;
	video_snapshot_init_with_streaming(&snap_config);
	atcmd_snapshot_init();//ATCMD => SNAP=SNAPS (Take picture to sdcard)
#else
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SNAPSHOT_CB, (int)v1_snapshot_cb);
	if (xTaskCreate(snapshot_control_thread, ((const char *)"snapshot_store"), 512, NULL, tskIDLE_PRIORITY + 1, &snapshot_thread) != pdPASS) {
		printf("\n\r%s xTaskCreate failed", __FUNCTION__);
	}
#endif

#if defined(ENABLE_META_INFO)
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_META_CB, (int)video_meta_cb);
#endif

	return;
mmf2_video_example_snapshot_httpfs_audio_init_fail:

	return;
}

static const char *example = "mmf2_video_example_snapshot_httpfs_audio_init";
static void example_deinit(void)
{
	//Pause Linker
	siso_pause(siso_video_rtsp_v1);

	//Stop module
	mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);

	//Delete linker
	siso_delete(siso_video_rtsp_v1);

	//Close module
	mm_module_close(rtsp2_v1_ctx);
	mm_module_close(video_v1_ctx);

	video_voe_release();
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
			printf("invalid state, can not do %s reinit!\r\n", example);
		}
	} else if (!strcmp(arg, "TSS")) {
		if (!(user_cmd & USR_CMD_EXAMPLE_DEINIT)) {
			printf("snapshot %s\r\n", example);
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
			rtw_create_secure_context(2048);
#endif
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SNAPSHOT, 1);
		} else {
			printf("invalid state, can not do %s snapshot!\r\n", example);
		}
	} else {
		printf("invalid cmd");
	}

	printf("user command 0x%lx\r\n", user_cmd);
}

static log_item_t userctrl_items[] = {
	{"UC", fUC, },
};

static void atcmd_userctrl_init(void)
{
	log_service_add_table(userctrl_items, sizeof(userctrl_items) / sizeof(userctrl_items[0]));
}

