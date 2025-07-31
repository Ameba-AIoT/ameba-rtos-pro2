/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "module_video.h"
#include "module_audio.h"
#include "module_aac.h"
#include "module_mp4.h"
#include "module_httpfs.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
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

static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *audio_ctx				= NULL;
static mm_context_t *aac_ctx				= NULL;
static mm_context_t *mp4_ctx        		= NULL;
static mm_context_t *httpfs_ctx        		= NULL;

static mm_siso_t *siso_audio_aac			= NULL;
static mm_miso_t *miso_video_aac_mp4			= NULL;

#define MP4_FILE_LENGTH      30 //30s
#define MP4_GROUP_FILE_NUM   60 //MP4_FILE_LENGTH*60 = 30m
#define STORAGE_FREE_SPACE   MP4_GROUP_FILE_NUM*MP4_FILE_LENGTH/3
#define MP4_LONG_RUN_TEST    1

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

static mp4_params_t mp4_v1_params = {
	.sample_rate = 8000,
	.channel = 1,

	.record_length = 30, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 3,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
};

static httpfs_params_t httpfs_params = {
	.fileext = "mp4",
	.filedir = "VIDEO",
	.request_string = "/video_get.mp4",
	.fatfs_buf_size = 1024
};

//static uint32_t file_serial = 0;
static char sd_filename[64];
static char sd_dirname[32];
static fatfs_sd_params_t fatfs_sd;

static void del_old_file(void)
{
	DIR m_dir;
	FILINFO m_fileinfo;
	char *filename;
	char old_filename[32] = {0};
	char old_filepath[32] = {0};
	WORD filedate = 0, filetime = 0, old_filedate = 0, old_filetime = 0;
#if _USE_LFN
	char fname_lfn[32];
	m_fileinfo.lfname = fname_lfn;
	m_fileinfo.lfsize = sizeof(fname_lfn);
#endif

	if (f_opendir(&m_dir, sd_dirname) == 0) {
		while (1) {
			if ((f_readdir(&m_dir, &m_fileinfo) != 0) || m_fileinfo.fname[0] == 0) {
				break;
			}

#if _USE_LFN
			filename = *m_fileinfo.lfname ? m_fileinfo.lfname : m_fileinfo.fname;
#else
			filename = m_fileinfo.fname;
#endif
			if (strcmp(filename, ".") == 0 || strcmp(filename, "..") == 0) {
				continue;
			}

			if (!(m_fileinfo.fattrib & AM_DIR)) {
				filedate = m_fileinfo.fdate;
				filetime = m_fileinfo.ftime;

				if ((strlen(old_filename) == 0) ||
					(filedate < old_filedate) ||
					((filedate == old_filedate) && (filetime < old_filetime))) {

					old_filedate = filedate;
					old_filetime = filetime;
					strcpy(old_filename, filename);
				}
			}
		}

		f_closedir(&m_dir);

		if (strlen(old_filename)) {
			sprintf(old_filepath, "%s/%s", sd_dirname, old_filename);
			printf("del %s\n\r", old_filepath);
			f_unlink(old_filepath);
		}
	}
}

int httpfs_response_cb(void)
{
	rt_printf("httpfs response\r\n");
	return 0;
}

void mmf2_video_example_av_mp4_httpfs_init(void)
{
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


	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_httpfs_fail;
	}

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
		goto mmf2_video_exmaple_av_mp4_httpfs_fail;
	}

	aac_ctx = mm_module_open(&aac_module);
	if (aac_ctx) {
		mm_module_ctrl(aac_ctx, CMD_AAC_SET_PARAMS, (int)&aac_params);
		mm_module_ctrl(aac_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(aac_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(aac_ctx, CMD_AAC_INIT_MEM_POOL, 0);
		mm_module_ctrl(aac_ctx, CMD_AAC_APPLY, 0);
	} else {
		rt_printf("AAC open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_httpfs_fail;
	}

	// Create video folder and check free space
	if (fatfs_sd_init() < 0) {
		goto mmf2_video_exmaple_av_mp4_httpfs_fail;
	}

	DIR m_dir;
	fatfs_sd_get_param(&fatfs_sd);
	sprintf(sd_dirname, "%s", "VIDEO");
	sprintf(sd_filename, "%s/%s", sd_dirname, "mp4_record");
	if (f_opendir(&m_dir, sd_dirname) == 0) {
		f_closedir(&m_dir);
	} else {
		f_mkdir(sd_dirname);
	}

	if (fatfs_get_free_space() < STORAGE_FREE_SPACE) {
		del_old_file();
	}

	if (fatfs_get_free_space() < STORAGE_FREE_SPACE) {
		rt_printf("ERROR: free space < 50MB\n\r");
	}

	mp4_ctx = mm_module_open(&mp4_module);
	mp4_v1_params.record_file_num = MP4_GROUP_FILE_NUM;
	mp4_v1_params.record_length = MP4_FILE_LENGTH;
	//file_serial = 1234;
	//sprintf(mp4_v1_params.record_file_name, "%s_%d", sd_filename, file_serial);
	sprintf(mp4_v1_params.record_file_name, "%s", sd_filename);

	int loopmode = MP4_LONG_RUN_TEST;
	if (mp4_ctx) {
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_PARAMS, (int)&mp4_v1_params);
		mm_module_ctrl(mp4_ctx, CMD_MP4_LOOP_MODE, loopmode);
		mm_module_ctrl(mp4_ctx, CMD_MP4_START, mp4_v1_params.record_file_num);
	} else {
		rt_printf("MP4 open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_httpfs_fail;
	}

	rt_printf("MP4 opened\n\r");

	//--------------HTTP File Server---------------
	httpfs_ctx = mm_module_open(&httpfs_module);
	if (httpfs_ctx) {
		mm_module_ctrl(httpfs_ctx, CMD_HTTPFS_SET_PARAMS, (int)&httpfs_params);
		mm_module_ctrl(httpfs_ctx, CMD_HTTPFS_SET_RESPONSE_CB, (int)httpfs_response_cb);
		mm_module_ctrl(httpfs_ctx, CMD_HTTPFS_APPLY, 0);
	} else {
		rt_printf("HTTPFS open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_httpfs_fail;
	}

	siso_audio_aac = siso_create();
	if (siso_audio_aac) {
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_OUTPUT, (uint32_t)aac_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(siso_audio_aac);
	} else {
		rt_printf("siso1 open fail\n\r");
		goto mmf2_video_exmaple_av_mp4_httpfs_fail;
	}

	rt_printf("siso1 started\n\r");

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
		goto mmf2_video_exmaple_av_mp4_httpfs_fail;
	}
	rt_printf("miso started\n\r");

	return;
mmf2_video_exmaple_av_mp4_httpfs_fail:

	return;
}
