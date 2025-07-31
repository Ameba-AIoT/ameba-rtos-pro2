/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "module_video.h"
#include "module_rtsp2.h"
#include "module_audio.h"
#include "module_g711.h"
#include "module_aac.h"
#include "module_opusc.h"
#include "mp4_muxer.h"
#include "module_mp4.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "log_service.h"
#include "sensor.h"

/*****************************************************************************
* ISP channel : 0
* Video type  : H264/HEVC
*****************************************************************************/
#define V1_CHANNEL 0
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR
#define USE_H265 0
#if USE_H265
#include "sample_h265.h"
#define VIDEO_TYPE VIDEO_HEVC
#define VIDEO_CODEC AV_CODEC_ID_H265
#else
#include "sample_h264.h"
#define VIDEO_TYPE VIDEO_H264
#define VIDEO_CODEC AV_CODEC_ID_H264
#endif
#define AAC_ENCODE_MODE
//#define G711_ULAW_MODE
//#define G711_ALAW_MODE
//#define OPUS_ENCODE_MODE

#define ENABLE_SET_GPS 1 //1 to Enable, 0 to Disable the GPS callback setup

static void atcmd_userctrl_init(void);
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *audio_ctx				= NULL;
static mm_context_t *aac_ctx				= NULL;
static mm_context_t *mp4_ctx				= NULL;
static mm_context_t *g711e_ctx				= NULL;
static mm_context_t *opusc_ctx				= NULL;

static mm_siso_t *siso_audio_aac			= NULL;
static mm_siso_t *siso_audio_g711e			= NULL;
static mm_siso_t *siso_audio_opusc			= NULL;
static mm_miso_t *miso_video_aac_mp4		= NULL;
static mm_miso_t *miso_video_g711e_mp4		= NULL;
static mm_miso_t *miso_video_opusc_mp4		= NULL;

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.bps = V1_BPS,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1
};

#if !USE_DEFAULT_AUDIO_SET
static audio_params_t audio_params = {
	.sample_rate = ASR_8KHZ,
	.word_length = WL_16BIT,
	.mic_gain    = MIC_0DB,
	.dmic_l_gain    = DMIC_BOOST_24DB,
	.dmic_r_gain    = DMIC_BOOST_24DB,
	.use_mic_type   = USE_AUDIO_AMIC,
	.channel     = 1,
	.enable_aec  = 0
};
#endif

static aac_params_t aac_params = {
	.sample_rate = 8000,
	.channel = 1,
	.trans_type = AAC_TYPE_ADTS,
	.object_type = AAC_AOT_LC,
	.bitrate = 32000,

	.mem_total_size = 10 * 1024,
	.mem_block_size = 128,
	.mem_frame_size = 1024
};

static g711_params_t g711e_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_ENCODE
};

static opusc_params_t opusc_params = {
	.sample_rate = 8000,
	.channel = 1,
	.bit_length = 16,			//16 recommand
	.complexity = 4,			//0~10
	.bitrate = 25000,			//default 25000
	.use_framesize = 40,		//needs to the same or bigger than AUDIO_DMA_PAGE_SIZE/(sample_rate/1000)/2 but less than 60
	.enable_vbr = 1,
	.vbr_constraint = 0,
	.packetLossPercentage = 0,
	.opus_application = OPUS_APPLICATION_RESTRICTED_LOWDELAY

};

static mp4_params_t mp4_v1_params = {
	.sample_rate = 8000,
	.channel = 1,

	.record_length = 10, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 1,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
	.mp4_audio_format = AUDIO_AAC,//AUDIO_OPUS,//AUDIO_ULAW
	.mp4_audio_duration = 40,//audio duration 40ms for PCM
};

int mp4_stop_cb(void *parm)
{
	printf("Record stop\r\n");
	return 0;
}
int mp4_end_cb(void *parm)
{
	printf("Record end\r\n");
	return 0;
}

#if ENABLE_SET_GPS
#define MAX_UDTA_SIZE 64
static int set_loci_box(unsigned char *udta_buf, pmp4_context mp4_ctx)
{
	//Set GPS's Latitude, Longitude and Altitude with an valid values.
	float latitude = 1.329911; // -90° to 90°
	float longitude = 103.747317; // -180° to 180°
	float altitude = 00.0000; // in metre

	// Convert latitude and longitude to Q16.16 fixed point format
	uint32_t lat_fixed = (int32_t)(latitude * 65536);  // Convert latitude to Q16.16 format
	uint32_t lon_fixed = (int32_t)(longitude * 65536); // Convert longitude to Q16.16 format

	// Standard loci box size with metadata (latitude, longitude, altitude)
	int lat_size = sizeof(lat_fixed);  // Size of latitude (4 bytes)
	int lon_size = sizeof(lon_fixed);  // Size of longitude (4 bytes)
	int alt_size = 4;  // Size of altitude (4 bytes)
	int flag_size = 8; // Size of flags and reserved (8 bytes)
	const char *earth = "earth";
	size_t earth_size = strlen(earth) + 1; // +1 to include the null terminator

	// Total size calculation for the loci box (35 bytes)
	int loci_size = 8 + flag_size + lat_size + lon_size + alt_size + earth_size + 1;

	// Create the box header: Box size (4 bytes)
	udta_buf[0] = (loci_size >> 24) & 0xFF;
	udta_buf[1] = (loci_size >> 16) & 0xFF;
	udta_buf[2] = (loci_size >> 8) & 0xFF;
	udta_buf[3] = loci_size & 0xFF;

	// ("loci", 4 bytes) in Hex values
	udta_buf[4] = 0x6C;  // 'l' = 6C
	udta_buf[5] = 0x6F;  // 'o' = 6F
	udta_buf[6] = 0x63;  // 'c' = 63
	udta_buf[7] = 0x69;  // 'i' = 69

	//Version and flags (8 bytes)
	memset(udta_buf + 8, 0, flag_size);

	// Use Save32BigEndian to copy latitude and longitude into buffer after the header
	Save32BigEndian(lon_fixed, udta_buf + 8 + flag_size);  // Longitude
	Save32BigEndian(lat_fixed, udta_buf + 8 + flag_size + lon_size);  // Latitude

	// Set altitude to zero (4 bytes)
	uint8_t zero_buffer[4] = {0, 0, 0, 0};  // Buffer of 4 bytes filled with zero
	memcpy(udta_buf + 8 + flag_size + lat_size + lon_size, zero_buffer, alt_size);  // Altitude

	// Reference system (e.g., "earth")
	memcpy(udta_buf + 8 + flag_size + lat_size + lon_size + alt_size, earth, earth_size);

	// Padding to ensure total size is correct
	uint8_t padding_buffer[1] = {0};  // 1 bytes of padding
	memcpy(udta_buf + 8 + flag_size + lat_size + lon_size + alt_size + earth_size, padding_buffer, 1);  // Padding

	// If the total size is not a multiple of 4, pad the buffer
	int padding_size = loci_size - (8 + flag_size + lat_size + lon_size + alt_size  + earth_size + 1);  // Remaining space after latitude, longitude, and altitude
	if (padding_size > 0) {
		memset(udta_buf + 8 + flag_size + lat_size + lon_size + alt_size + earth_size + 1, 0, padding_size);  // Pad with zeroes
	}

	return loci_size;
}

static int set_udta_box(unsigned char *udta_buf, pmp4_context mp4_ctx)
{
	int size = 0;
	int udta_position = 0;
	// Create udta box header (8 bytes)
	size = create_box(udta_buf + udta_position, (char *)"udta", 8);
	udta_position += size;

	// Add loci box
	size = set_loci_box(udta_buf + udta_position, mp4_ctx);
	udta_position += size;

	// Update udta box size after adding loci box
	update_udtabox_size(mp4_ctx, udta_buf, udta_position);

	return udta_position;
}
#endif

void mmf2_video_example_av_mp4_init(void)
{
	atcmd_userctrl_init();

	/*sensor capacity check & video parameter setting*/
	video_v1_params.resolution = VIDEO_FHD;
	video_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	video_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
	video_v1_params.fps = sensor_params[USE_SENSOR].sensor_fps;
	video_v1_params.gop = sensor_params[USE_SENSOR].sensor_fps;
	/*mp4 parameter setting*/
	mp4_v1_params.fps = sensor_params[USE_SENSOR].sensor_fps;
	mp4_v1_params.gop = sensor_params[USE_SENSOR].sensor_fps;
	mp4_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	mp4_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, video_v1_params.width, video_v1_params.height, V1_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 0, NULL, 0, NULL, 0, NULL);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	// ------ Channel 1--------------
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		//mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}

	mp4_ctx = mm_module_open(&mp4_module);
#ifdef AAC_ENCODE_MODE
	mp4_v1_params.mp4_audio_format = AUDIO_AAC;
#endif

#ifdef G711_ULAW_MODE
	mp4_v1_params.mp4_audio_format = AUDIO_ULAW;
#endif

#ifdef G711_ALAW_MODE
	mp4_v1_params.mp4_audio_format = AUDIO_ALAW;
#endif

#ifdef OPUS_ENCODE_MODE
	mp4_v1_params.mp4_audio_format = AUDIO_OPUS;
#endif
	if (mp4_ctx) {
#if ENABLE_SET_GPS
		/*Udta parameter setting*/
		udta_callback_t udta_params;
		mp4_v1_params.udta_buf = (unsigned char *)malloc(MAX_UDTA_SIZE); //  Allocate memory and set to udta_buf
		if (mp4_v1_params.udta_buf == NULL) {
			printf("ERROR: Allocation memory of udta_buf failed\n");
			printf("ERROR: Video init failed\n");
			goto mmf2_video_exmaple_av_mp4_fail;
		}
		mp4_v1_params.udta_buf_size = MAX_UDTA_SIZE; //Set the udta_buf_size
		udta_params.udta_box_cb = set_udta_box; // Set the callback function
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_UDAT_CALLBACK, (int)&udta_params);
#endif
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_PARAMS, (int)&mp4_v1_params);
		mm_module_ctrl(mp4_ctx, CMD_MP4_LOOP_MODE, 0);
		mm_module_ctrl(mp4_ctx, CMD_MP4_START, mp4_v1_params.record_file_num);
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_STOP_CB, (int)mp4_stop_cb);
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_END_CB, (int)mp4_end_cb);
	} else {
		rt_printf("MP4 open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}

	rt_printf("MP4 opened\n\r");

	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
#if !USE_DEFAULT_AUDIO_SET
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
#endif
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		rt_printf("AUDIO open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}
#ifdef AAC_ENCODE_MODE
	aac_ctx = mm_module_open(&aac_module);
	if (aac_ctx) {
		mm_module_ctrl(aac_ctx, CMD_AAC_SET_PARAMS, (int)&aac_params);
		mm_module_ctrl(aac_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(aac_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(aac_ctx, CMD_AAC_INIT_MEM_POOL, 0);
		mm_module_ctrl(aac_ctx, CMD_AAC_APPLY, 0);
	} else {
		rt_printf("AAC open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}
#else
#ifdef OPUS_ENCODE_MODE
	opusc_ctx = mm_module_open(&opusc_module);
	if (opusc_ctx) {
		mm_module_ctrl(opusc_ctx, CMD_OPUSC_SET_PARAMS, (int)&opusc_params);
		mm_module_ctrl(opusc_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(opusc_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(opusc_ctx, CMD_OPUSC_APPLY, 0);
	} else {
		rt_printf("OPUSC open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}
#else
	g711e_ctx = mm_module_open(&g711_module);
#ifdef G711_ULAW_MODE
	g711e_params.codec_id = AV_CODEC_ID_PCMU;
#endif

#ifdef G711_ALAW_MODE
	g711e_params.codec_id = AV_CODEC_ID_PCMA;
#endif
	if (g711e_ctx) {
		mm_module_ctrl(g711e_ctx, CMD_G711_SET_PARAMS, (int)&g711e_params);
		mm_module_ctrl(g711e_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711e_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711e_ctx, CMD_G711_APPLY, 0);
	} else {
		rt_printf("G711 open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}
#endif
#endif

#ifdef AAC_ENCODE_MODE
	siso_audio_aac = siso_create();
	if (siso_audio_aac) {
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_OUTPUT, (uint32_t)aac_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(siso_audio_aac);
	} else {
		rt_printf("siso1 open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}
#else
#ifdef OPUS_ENCODE_MODE
	siso_audio_opusc = siso_create();
	if (siso_audio_opusc) {
		siso_ctrl(siso_audio_opusc, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_opusc, MMIC_CMD_ADD_OUTPUT, (uint32_t)opusc_ctx, 0);
		siso_ctrl(siso_audio_opusc, MMIC_CMD_SET_STACKSIZE, 24 * 1024, 0);
		siso_start(siso_audio_opusc);
	} else {
		rt_printf("siso1 open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}
#else
	siso_audio_g711e = siso_create();
	if (siso_audio_g711e) {
		siso_ctrl(siso_audio_g711e, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_g711e, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711e_ctx, 0);
		siso_start(siso_audio_g711e);
	} else {
		rt_printf("siso_audio_g711e open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}
#endif
#endif

	rt_printf("siso1 started\n\r");

#ifdef AAC_ENCODE_MODE
	miso_video_aac_mp4 = miso_create();
	if (miso_video_aac_mp4) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		miso_ctrl(miso_video_aac_mp4, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		miso_ctrl(miso_video_aac_mp4, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
		miso_ctrl(miso_video_aac_mp4, MMIC_CMD_ADD_INPUT1, (uint32_t)aac_ctx, 0);
		miso_ctrl(miso_video_aac_mp4, MMIC_CMD_ADD_OUTPUT, (uint32_t)mp4_ctx, 0);
		miso_start(miso_video_aac_mp4);
	} else {
		rt_printf("miso open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}
#else
#ifdef OPUS_ENCODE_MODE
	miso_video_opusc_mp4 = miso_create();
	if (miso_video_opusc_mp4) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		miso_ctrl(miso_video_opusc_mp4, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		miso_ctrl(miso_video_opusc_mp4, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
		miso_ctrl(miso_video_opusc_mp4, MMIC_CMD_ADD_INPUT1, (uint32_t)opusc_ctx, 0);
		miso_ctrl(miso_video_opusc_mp4, MMIC_CMD_ADD_OUTPUT, (uint32_t)mp4_ctx, 0);
		miso_start(miso_video_opusc_mp4);
	} else {
		rt_printf("miso open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}
#else
	miso_video_g711e_mp4 = miso_create();
	if (miso_video_g711e_mp4) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		miso_ctrl(miso_video_g711e_mp4, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		miso_ctrl(miso_video_g711e_mp4, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
		miso_ctrl(miso_video_g711e_mp4, MMIC_CMD_ADD_INPUT1, (uint32_t)g711e_ctx, 0);
		miso_ctrl(miso_video_g711e_mp4, MMIC_CMD_ADD_OUTPUT, (uint32_t)mp4_ctx, 0);
		miso_start(miso_video_g711e_mp4);
	} else {
		rt_printf("miso open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_fail;
	}
#endif
#endif

	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);

	rt_printf("miso started\n\r");

	return;
mmf2_video_exmaple_av_mp4_fail:

	return;
}

static const char *example = "mmf2_video_example_av_mp4";
static void example_deinit(void)
{
	//Pause Linker
#ifdef AAC_ENCODE_MODE
	miso_pause(miso_video_aac_mp4, MM_OUTPUT);
	siso_pause(siso_audio_aac);
#else
	miso_pause(miso_video_g711e_mp4, MM_OUTPUT);
	siso_pause(siso_audio_g711e);
#endif

	//Stop module
	mm_module_ctrl(mp4_ctx, CMD_MP4_STOP, 0);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_TRX, 0);
#ifdef AAC_ENCODE_MODE
	mm_module_ctrl(aac_ctx, CMD_AAC_STOP, 0);
#else
	//g711 no stop cmd and g711 apply do nothing
#endif

	//Delete linker
	miso_delete(miso_video_aac_mp4);
#ifdef AAC_ENCODE_MODE
	siso_delete(siso_audio_aac);
#else
	siso_delete(siso_audio_g711e);
#endif

	//Close module
	mm_module_close(video_v1_ctx);
	mm_module_close(audio_ctx);
#ifdef AAC_ENCODE_MODE
	mm_module_close(aac_ctx);
#else
	mm_module_close(g711e_ctx);
#endif
	mm_module_close(mp4_ctx);

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
