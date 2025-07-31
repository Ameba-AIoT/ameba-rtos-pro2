/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_mimo.h"

#include "module_video.h"
#include "module_rtsp2.h"
#include "module_audio.h"
#include "module_aac.h"
#include "module_aad.h"
#include "module_rtp.h"
#include "module_mp4.h"
#include "module_vipnn.h"
#include "module_eip.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "log_service.h"
#include "fwfs.h"
#include "nn_utils/class_name.h"
#include "model_yolo.h"
#include "isp_ctrl_api.h"

/*****************************************************************************
* ISP channel : 0,1
* Video type  : H264/HEVC
*****************************************************************************/

#define V1_CHANNEL 0
#define V1_FPS sensor_params[USE_SENSOR].sensor_fps
#define V1_GOP sensor_params[USE_SENSOR].sensor_fps
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR

#define V2_CHANNEL 1
#define V2_FPS sensor_params[USE_SENSOR].sensor_fps
#define V2_GOP sensor_params[USE_SENSOR].sensor_fps
#define V2_BPS 1024*1024
#define V2_RCMODE 2 // 1: CBR, 2: VBR

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

#define V1_WIDTH	sensor_params[USE_SENSOR].sensor_width
#define V1_HEIGHT	sensor_params[USE_SENSOR].sensor_height

#define V2_WIDTH	1280
#define V2_HEIGHT	720

static void atcmd_userctrl_init(void);
static void atcmd_fcs_init(void);
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *video_v2_ctx			= NULL;
static mm_context_t *rtsp2_v2_ctx			= NULL;
static mm_context_t *audio_ctx				= NULL;
static mm_context_t *aac_ctx				= NULL;
static mm_context_t *rtp_ctx				= NULL;
static mm_context_t *aad_ctx				= NULL;
static mm_context_t *mp4_ctx				= NULL;


static mm_siso_t *siso_audio_aac			= NULL;
static mm_mimo_t *mimo_2v_1a_rtsp_mp4		= NULL;
static mm_siso_t *siso_rtp_aad				= NULL;
static mm_siso_t *siso_aad_audio			= NULL;

#include "video_boot.h"
#include "ftl_common_api.h"
#include "boot_retention.h"

//#define FCS_PARTITION //Use the FCS data to change the parameter from bootloader.If mark the marco, it will use the FTL config.

#define EXAMPLE_SAVE_TO_FLASH 0
#define EXAMPLE_SAVE_TO_RETENTION 1
#define EXAMPLE_SAVE_OPTION EXAMPLE_SAVE_TO_FLASH

#define ENA_SLEEP_TEST 0 //for testing fcs with retention data

static video_boot_stream_t video_boot_stream = {
	.video_params[STREAM_V1].stream_id = STREAM_ID_V1,
	.video_params[STREAM_V1].type = CODEC_H264,
	.video_params[STREAM_V1].resolution = 0,
	.video_params[STREAM_V1].width  = sensor_params[USE_SENSOR].sensor_width,
	.video_params[STREAM_V1].height = sensor_params[USE_SENSOR].sensor_height,
	.video_params[STREAM_V1].bps = 2 * 1024 * 1024,
	.video_params[STREAM_V1].fps = 15,
	.video_params[STREAM_V1].gop = 15,
	.video_params[STREAM_V1].rc_mode = 2,
	.video_params[STREAM_V1].minQp = 25,
	.video_params[STREAM_V1].maxQp = 48,
	.video_params[STREAM_V1].jpeg_qlevel = 0,
	.video_params[STREAM_V1].rotation = 0,
	.video_params[STREAM_V1].out_buf_size = V1_ENC_BUF_SIZE,
	.video_params[STREAM_V1].out_rsvd_size = 0,
	.video_params[STREAM_V1].direct_output = 0,
	.video_params[STREAM_V1].use_static_addr = 0,
	.video_snapshot[STREAM_V1] = 0,
	.video_drop_frame[STREAM_V1] = 0,
	.video_params[STREAM_V1].fcs = 1,//Enable the fcs for channel 1
	.bps_stbl_ctrl_params[STREAM_V1].sampling_time = 1000,
	.bps_stbl_ctrl_params[STREAM_V1].maximun_bitrate = 2 * 1024 * 1024 * 1.2,
	.bps_stbl_ctrl_params[STREAM_V1].minimum_bitrate = 2 * 1024 * 1024 * 0.8,
	.bps_stbl_ctrl_params[STREAM_V1].target_bitrate = 2 * 1024 * 1024,
	.video_params[STREAM_V2].stream_id = STREAM_ID_V2,
	.video_params[STREAM_V2].type = CODEC_H264,
	.video_params[STREAM_V2].resolution = 0,
	.video_params[STREAM_V2].width = 1280,
	.video_params[STREAM_V2].height = 720,
	.video_params[STREAM_V2].bps = 1 * 1024 * 1024,
	.video_params[STREAM_V2].fps = 15,
	.video_params[STREAM_V2].gop = 15,
	.video_params[STREAM_V2].rc_mode = 0,
	.video_params[STREAM_V2].minQp = 25,
	.video_params[STREAM_V2].maxQp = 48,
	.video_params[STREAM_V2].jpeg_qlevel = 0,
	.video_params[STREAM_V2].rotation = 0,
	.video_params[STREAM_V2].out_buf_size = V2_ENC_BUF_SIZE,
	.video_params[STREAM_V2].out_rsvd_size = 0,
	.video_params[STREAM_V2].direct_output = 0,
	.video_params[STREAM_V2].use_static_addr = 0,
	.video_params[STREAM_V2].fcs = 0,
	.video_snapshot[STREAM_V2] = 0,
	.video_drop_frame[STREAM_V2] = 0,
	.bps_stbl_ctrl_params[STREAM_V2].sampling_time = 0,
	.bps_stbl_ctrl_params[STREAM_V2].maximun_bitrate = 0,
	.bps_stbl_ctrl_params[STREAM_V2].minimum_bitrate = 0,
	.bps_stbl_ctrl_params[STREAM_V2].target_bitrate = 0,
	.video_params[STREAM_V3].stream_id = STREAM_ID_V3,
	.video_params[STREAM_V3].type = CODEC_NV12,
	.video_params[STREAM_V3].resolution = 0,
	.video_params[STREAM_V3].width = 640,
	.video_params[STREAM_V3].height = 480,
	.video_params[STREAM_V3].bps = 0,
	.video_params[STREAM_V3].fps = 10,
	.video_params[STREAM_V3].gop = 0,
	.video_params[STREAM_V3].rc_mode = 0,
	.video_params[STREAM_V3].minQp = 0,
	.video_params[STREAM_V3].maxQp = 0,
	.video_params[STREAM_V3].jpeg_qlevel = 0,
	.video_params[STREAM_V3].rotation = 0,
	.video_params[STREAM_V3].out_buf_size = V3_ENC_BUF_SIZE,
	.video_params[STREAM_V3].out_rsvd_size = 0,
	.video_params[STREAM_V3].direct_output = 0,
	.video_params[STREAM_V3].use_static_addr = 0,
	.video_params[STREAM_V3].fcs = 0,
	.video_snapshot[STREAM_V3] = 0,
	.video_drop_frame[STREAM_V3] = 0,
	.video_params[STREAM_V4].stream_id = STREAM_ID_V4,
	.video_params[STREAM_V4].type = CODEC_RGB,
	.video_params[STREAM_V4].resolution = 0,
	.video_params[STREAM_V4].width = 640,
	.video_params[STREAM_V4].height = 480,
	.video_params[STREAM_V4].bps = 0,
	.video_params[STREAM_V4].fps = 10,
	.video_params[STREAM_V4].gop = 0,
	.video_params[STREAM_V4].rc_mode = 0,
	.video_params[STREAM_V4].minQp = 0,
	.video_params[STREAM_V4].maxQp = 0,
	.video_params[STREAM_V4].jpeg_qlevel = 0,
	.video_params[STREAM_V4].rotation = 0,
	.video_params[STREAM_V4].out_buf_size = 0,
	.video_params[STREAM_V4].out_rsvd_size = 0,
	.video_params[STREAM_V4].direct_output = 0,
	.video_params[STREAM_V4].use_static_addr = 0,
	.video_params[STREAM_V4].fcs = 0,
	.video_enable[STREAM_V1] = 1,
	.video_enable[STREAM_V2] = 1,
	.video_enable[STREAM_V3] = 0,
	.video_enable[STREAM_V4] = 1,
	.fcs_isp_ae_enable = 0,
	.fcs_isp_ae_init_exposure = 0,
	.fcs_isp_ae_init_gain = 0,
	.fcs_isp_awb_enable = 0,
	.fcs_isp_awb_init_rgain = 0,
	.fcs_isp_awb_init_bgain = 0,
	.fcs_isp_init_daynight_mode = 0,
	.voe_heap_size = 0,
	.voe_heap_addr = 0,
	.isp_info.sensor_width = sensor_params[USE_SENSOR].sensor_width,
	.isp_info.sensor_height = sensor_params[USE_SENSOR].sensor_height,
	.isp_info.sensor_fps = sensor_params[USE_SENSOR].sensor_fps,
	.isp_info.md_enable = 1,
	.isp_info.hdr_enable = 1,
	.isp_info.osd_enable = 1,
	.fcs_channel = 1,//FCS_TOTAL_NUMBER
	.fcs_status = 0,
	.fcs_setting_done = 0,
	.fcs_isp_iq_id = 0,
};

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.width = V1_WIDTH,
	.height = V1_HEIGHT,
	.bps = V1_BPS,
	.fps = V1_FPS,
	.gop = V1_GOP,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1
};

static video_params_t video_v2_params = {
	.stream_id = V2_CHANNEL,
	.type = VIDEO_TYPE,
	.width = V2_WIDTH,
	.height = V2_HEIGHT,
	.bps = V2_BPS,
	.fps = V2_FPS,
	.gop = V2_GOP,
	.rc_mode = V2_RCMODE,
	.use_static_addr = 1
};

static audio_params_t audio_params;

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

static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.fps      = V1_FPS,
			.bps      = V1_BPS
		}
	}
};

static rtsp2_params_t rtsp2_v2_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.fps      = V2_FPS,
			.bps      = V2_BPS
		}
	}
};

static rtsp2_params_t rtsp2_a_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a = {
			.codec_id   = AV_CODEC_ID_MP4A_LATM,
			.channel    = 1,
			.samplerate = 8000
		}
	}
};

static aad_params_t aad_rtp_params = {
	.sample_rate = 8000,
	.channel = 1,
	.trans_type = AAD_TYPE_RTP_RAW,
	.object_type = AAD_AOT_LC
};

static rtp_params_t rtp_aad_params = {
	.valid_pt = 0xFFFFFFFF,
	.port = 16384,
	.frame_size = 1500,
	.cache_depth = 6
};

static mp4_params_t mp4_v1_params = {
	.fps            = V1_FPS,
	.gop            = V1_GOP,
	.width = V1_WIDTH,
	.height = V1_HEIGHT,
	.sample_rate = 8000,
	.channel = 1,

	.record_length = 10, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 2,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
};

static bps_stbl_ctrl_param_t bps_stbl_ctrl_v1_params;
static bps_stbl_ctrl_param_t bps_stbl_ctrl_v2_params;
static uint32_t bps_stbl_ctrl_v1_fps_stage[BPS_STBL_CTRL_STG_CNT] = {0};
static uint32_t bps_stbl_ctrl_v1_gop_stage[BPS_STBL_CTRL_STG_CNT] = {0};
static uint32_t bps_stbl_ctrl_v2_fps_stage[BPS_STBL_CTRL_STG_CNT] = {0};
static uint32_t bps_stbl_ctrl_v2_gop_stage[BPS_STBL_CTRL_STG_CNT] = {0};
static uint8_t bps_stbl_ctrl_en[2] = {0, 0};

#ifdef FCS_PARTITION
void voe_fcs_change_parameters(int ch, int width, int height, int iq_id, int video_pre_init)
{
	pfw_init();

	void *fp = pfw_open("FCSDATA", M_RAW | M_CREATE);
	if (fp == NULL) {
		printf("Cannot open FCSDATA\r\n");
		return;
	}

	unsigned char *ptr = (unsigned char *)&video_boot_stream;
	unsigned char *fcs_buf = malloc(4096);
	unsigned char *fcs_verify = malloc(4096);
	unsigned int checksum_tag = 0;
	int i = 0;
	if (fcs_buf == NULL || fcs_verify == NULL) {
		printf("It can't allocate buffer\r\n");
		return;
	}
	video_boot_stream.video_params[ch].width  = width;
	video_boot_stream.video_params[ch].height = height;
	video_boot_stream.fcs_isp_iq_id = iq_id;
	if (video_pre_init) {
		//AE value
		int ae_exposure_time, ae_gain;
		isp_get_exposure_time(&ae_exposure_time);
		isp_get_ae_gain(&ae_gain);
		printf("ae exposure time %d, ae gain %d\r\n", ae_exposure_time, ae_gain);
		video_boot_stream.fcs_isp_ae_enable = 1;
		video_boot_stream.fcs_isp_ae_init_exposure = ae_exposure_time;
		video_boot_stream.fcs_isp_ae_init_gain = ae_gain;
		//AWB value
		int awb_rgain, awb_bgain;
		isp_get_red_balance(&awb_rgain);
		isp_get_blue_balance(&awb_bgain);
		printf("awb rgain %d, bgain %d\r\n", awb_rgain, awb_bgain);
		video_boot_stream.fcs_isp_awb_enable = 1;
		video_boot_stream.fcs_isp_awb_init_rgain = awb_rgain;
		video_boot_stream.fcs_isp_awb_init_bgain = awb_bgain;
	} else {
		printf("disable isp init\r\n");
		video_boot_stream.fcs_isp_ae_enable = 0;
		video_boot_stream.fcs_isp_awb_enable = 0;
	}
	printf("ch %d width %d height %d iq_id %d\r\n", ch, width, height, iq_id);
	memset(fcs_buf, 0x00, 4096);
	memset(fcs_verify, 0x00, 4096);
	fcs_buf[0] = 'F';
	fcs_buf[1] = 'C';
	fcs_buf[2] = 'S';
	fcs_buf[3] = 'D';
	for (i = 0; i < sizeof(video_boot_stream_t); i++) {
		checksum_tag += ptr[i];
	}
	fcs_buf[4] = (checksum_tag) & 0xff;
	fcs_buf[5] = (checksum_tag >> 8) & 0xff;
	fcs_buf[6] = (checksum_tag >> 16) & 0xff;
	fcs_buf[7] = (checksum_tag >> 24) & 0xff;
	memcpy(fcs_buf + 8, &video_boot_stream, sizeof(video_boot_stream_t));
	pfw_seek(fp, 0, SEEK_SET);
	pfw_write(fp, fcs_buf, 4096);
	pfw_seek(fp, 0, SEEK_SET);
	pfw_read(fp, fcs_verify, 4096);
	for (i = 0; i < 4096; i++) {
		if (fcs_buf[i] != fcs_verify[i]) {
			printf("wrong %d %x %x\r\n", i, fcs_buf[i], fcs_verify[i]);
		}
	}
	pfw_seek(fp, 0, SEEK_SET);
	pfw_close(fp);
	if (fcs_buf) {
		free(fcs_buf);
	}
	if (fcs_verify) {
		free(fcs_verify);
	}
}
#else

#if EXAMPLE_SAVE_OPTION == EXAMPLE_SAVE_TO_RETENTION
extern boot_retention_table_t retention_table;
__attribute__((section(".retention.data"))) video_boot_stream_t video_boot_retention_data __attribute__((aligned(32)));
#endif
void voe_fcs_change_parameters(int ch, int width, int height, int iq_id, int video_pre_init) //Setup the tag and modify the parameters.
{
	video_boot_stream_t *fcs_data = NULL;// = (video_boot_stream_t *)malloc(sizeof(video_boot_stream_t));
	int i = 0;
	unsigned char *ptr = (unsigned char *)&video_boot_stream;
#if EXAMPLE_SAVE_OPTION == EXAMPLE_SAVE_TO_FLASH
	unsigned char *fcs_buf = malloc(2048);
	unsigned int flash_addr = 0;
	if (sys_get_boot_sel() == 0) {
		flash_addr = NOR_FLASH_FCS;
	} else {
		flash_addr = NAND_FLASH_FCS;
	}
	if (fcs_buf == NULL) {
		printf("It can't get the buffer\r\n");
		return;
	}
#endif

	video_boot_stream.video_params[ch].width  = width;
	video_boot_stream.video_params[ch].height = height;
	video_boot_stream.fcs_isp_iq_id = iq_id;
	if (video_pre_init) {
		//AE value
		int ae_exposure_time, ae_gain;
		isp_get_exposure_time(&ae_exposure_time);
		isp_get_ae_gain(&ae_gain);
		printf("ae exposure time %d, ae gain %d\r\n", ae_exposure_time, ae_gain);
		video_boot_stream.fcs_isp_ae_enable = 1;
		video_boot_stream.fcs_isp_ae_init_exposure = ae_exposure_time;
		video_boot_stream.fcs_isp_ae_init_gain = ae_gain;
		//AWB value
		int awb_rgain, awb_bgain;
		isp_get_red_balance(&awb_rgain);
		isp_get_blue_balance(&awb_bgain);
		printf("awb rgain %d, bgain %d\r\n", awb_rgain, awb_bgain);
		video_boot_stream.fcs_isp_awb_enable = 1;
		video_boot_stream.fcs_isp_awb_init_rgain = awb_rgain;
		video_boot_stream.fcs_isp_awb_init_bgain = awb_bgain;
	} else {
		printf("disable isp init\r\n");
		video_boot_stream.fcs_isp_ae_enable = 0;
		video_boot_stream.fcs_isp_awb_enable = 0;
	}
	printf("ch %d width %d height %d iq_id %d\r\n", ch, width, height, iq_id);

#if EXAMPLE_SAVE_OPTION == EXAMPLE_SAVE_TO_FLASH
	memset(fcs_buf, 0x00, 2048);
	fcs_buf[0] = 'F';
	fcs_buf[1] = 'C';
	fcs_buf[2] = 'S';
	fcs_buf[3] = 'D';
	unsigned int checksum_tag = 0;
	for (i = 0; i < sizeof(video_boot_stream_t); i++) {
		checksum_tag += ptr[i];
	}
	fcs_buf[4] = (checksum_tag) & 0xff;
	fcs_buf[5] = (checksum_tag >> 8) & 0xff;
	fcs_buf[6] = (checksum_tag >> 16) & 0xff;
	fcs_buf[7] = (checksum_tag >> 24) & 0xff;
	memcpy(fcs_buf + 8, &video_boot_stream, sizeof(video_boot_stream_t));
	ftl_common_write(flash_addr, fcs_buf, 2048);
	memset(fcs_buf, 0xff, 2048);
	ftl_common_read(flash_addr, fcs_buf, 2048);
	fcs_data = (video_boot_stream_t *)(fcs_buf + 8);
	printf("ch %d ->width %d ->height %d iq_id %d\r\n", ch, fcs_data->video_params[ch].width, fcs_data->video_params[ch].height, fcs_data->fcs_isp_iq_id);
	if (fcs_buf) {
		free(fcs_buf);
	}
#elif EXAMPLE_SAVE_OPTION == EXAMPLE_SAVE_TO_RETENTION
	uint32_t checksum_value = 0;
	uint8_t *checksum_array = (uint8_t *) &video_boot_stream;
	for (int i = 0; i < sizeof(video_boot_stream); i++) {
		checksum_value += checksum_array[i];
	}
	memcpy(&video_boot_retention_data, &video_boot_stream, sizeof(video_boot_retention_data));
	dcache_clean_invalidate_by_addr((uint32_t *)&video_boot_retention_data, sizeof(video_boot_retention_data));

	uint8_t *tag = (uint8_t *)&retention_table.reserve_data[0].tag;
	tag[0] = 'F';
	tag[1] = 'C';
	tag[2] = 'S';
	tag[3] = 'D';
	retention_table.reserve_data[0].data_len = sizeof(video_boot_stream_t);
	retention_table.reserve_data[0].address = (uint32_t)&video_boot_retention_data;
	retention_table.reserve_data[0].checksum = checksum_value;
	dcache_clean_invalidate_by_addr((uint32_t *)&retention_table, sizeof(retention_table));
#endif
}
#endif

static inline unsigned int str_to_value(const unsigned char *str)
{
	if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X')) {
		return strtol((char const *)str, NULL, 16);
	} else {
		return atoi((char const *)str);
	}
}

void fcs_change(void *arg)
{
	int argc;
	int ret = 0;
	char *argv[MAX_ARGC] = {0};
	unsigned char *ptr = NULL;
	int ch, width, height, iq_id = 0, video_pre_init = 0;

	argc = parse_param(arg, argv);
	if (argc != 6) {
		printf("FCST=ch,width,height,iq_id,video_pre_init\r\n");//FCST=0,1280,720,IQ_ID,1
		return;
	}
	ch = str_to_value((unsigned char const *)argv[1]);
	width = str_to_value((unsigned char const *)argv[2]);
	height = str_to_value((unsigned char const *)argv[3]);
	iq_id = str_to_value((unsigned char const *)argv[4]);
	video_pre_init = str_to_value((unsigned char const *)argv[5]);
	printf("ch %d width %d height %d iq_id %d video_pre_init %d\r\n", ch, width, height, iq_id, video_pre_init);
	voe_fcs_change_parameters(ch, width, height, iq_id, video_pre_init);
}

void fcs_info(void *arg)
{
	video_boot_stream_t *isp_fcs_info;
	video_get_fcs_info(&isp_fcs_info);
	printf("ch 0 -> width %d height %d\r\n", isp_fcs_info->video_params[STREAM_V1].width, isp_fcs_info->video_params[STREAM_V1].height);
	printf("ch 1 -> width %d height %d\r\n", isp_fcs_info->video_params[STREAM_V2].width, isp_fcs_info->video_params[STREAM_V2].height);
}

int fcs_snapshot_cb(uint32_t jpeg_addr, uint32_t jpeg_len)
{
	printf("snapshot size=%d\n\r", jpeg_len);
	return 0;
}

#define FCS_AV_SYNC 1
void fcs_avsync(bool enable)
{
	if (enable) {
		//get the fcs time need to what video first frame
		int fcs_video_starttime = 0;
		int fcs_video_endtime = 0;
		while (!fcs_video_starttime) {
			vTaskDelay(1);
			video_get_fcs_queue_info(&fcs_video_starttime, &fcs_video_endtime);
		}
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_AVSYNC_TIMESTAMP, fcs_video_starttime);
	}
}

#define ENA_MD_NN   0  //enable MD trigger NN in ch4

/*****************************************************************************
* ISP channel : 4
* Video type  : RGB
*****************************************************************************/
#define V4_CHANNEL 4
#define V4_WIDTH 416
#define V4_HEIGHT 416
#define V4_FPS 10
#define V4_TYPE VIDEO_RGB

#define SENSOR_MAX_WIDTH sensor_params[USE_SENSOR].sensor_width
#define SENSOR_MAX_HEIGHT sensor_params[USE_SENSOR].sensor_height

static video_params_t video_v4_params = {
	.stream_id 		= V4_CHANNEL,
	.type 			= V4_TYPE,
	.width 			= V4_WIDTH,
	.height 		= V4_HEIGHT,
	.fps 			= V4_FPS,
	.direct_output 	= 0,
	.use_static_addr = 1,
	.use_roi = 1,
	.roi = {
		.xmin = 0,
		.ymin = 0,
		.xmax = SENSOR_MAX_WIDTH,
		.ymax = SENSOR_MAX_HEIGHT,
	}
};

#if ENA_MD_NN
#define MD_COL 32
#define MD_ROW 32

static float nn_confidence_thresh = 0.5;
static float nn_nms_thresh = 0.3;

//eip acceleration resolution are 640x480, 640x360, 576x320, 416x416, 320x180, 128x128
static eip_param_t eip_param = {
	.image_width = V4_WIDTH,
	.image_height = V4_HEIGHT,
	.eip_row = MD_ROW,
	.eip_col = MD_COL,
};

static nn_data_param_t roi_nn = {
	.img = {
		.width = V4_WIDTH,
		.height = V4_HEIGHT,
		.roi = {
			.xmin = 0,
			.ymin = 0,
			.xmax = V4_WIDTH,
			.ymax = V4_HEIGHT,
		}
	},
	.codec_type = AV_CODEC_ID_RGB888
};

static mm_context_t *video_rgb_ctx  = NULL;
static mm_context_t *vipnn_ctx      = NULL;
static mm_context_t *md_ctx         = NULL;
static mm_siso_t *siso_rgb_md       = NULL;
static mm_siso_t *siso_md_nn        = NULL;

//--------------------------------------------
// Draw Rect
//--------------------------------------------
#include "osd_render.h"
#define LIMIT(x, lower, upper) if(x<lower) x=lower; else if(x>upper) x=upper;

static void nn_set_object(void *p, void *img_param)
{
	int i = 0;
	vipnn_out_buf_t *out = (vipnn_out_buf_t *)p;
	objdetect_res_t *res = (objdetect_res_t *)&out->res[0];

	int obj_num = out->res_cnt;

	nn_data_param_t *im = (nn_data_param_t *)img_param;

	if (!p || !img_param)	{
		return;
	}

	int im_h = V2_HEIGHT;
	int im_w = V2_WIDTH;
	float ratio_h = (float)im_h / (float)im->img.height;
	float ratio_w = (float)im_w / (float)im->img.width;
	int roi_h = (int)((im->img.roi.ymax - im->img.roi.ymin) * ratio_h);
	int roi_w = (int)((im->img.roi.xmax - im->img.roi.xmin) * ratio_w);
	int roi_x = (int)(im->img.roi.xmin * ratio_w);
	int roi_y = (int)(im->img.roi.ymin * ratio_h);

	//printf("object num = %d\r\n", obj_num);
	canvas_create_bitmap(V2_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
	if (obj_num > 0) {
		for (i = 0; i < obj_num; i++) {
			int obj_class = (int)res[i].result[0];
			//printf("obj_class = %d\r\n",obj_class);

			int class_id = obj_class; //coco label
			if (class_id != -1) {
				int xmin = (int)(res[i].result[2] * roi_w) + roi_x;
				int ymin = (int)(res[i].result[3] * roi_h) + roi_y;
				int xmax = (int)(res[i].result[4] * roi_w) + roi_x;
				int ymax = (int)(res[i].result[5] * roi_h) + roi_y;
				LIMIT(xmin, 0, im_w)
				LIMIT(xmax, 0, im_w)
				LIMIT(ymin, 0, im_h)
				LIMIT(ymax, 0, im_h)
				//printf("%d,c%d:%d %d %d %d\n\r", i, class_id, xmin, ymin, xmax, ymax);
				canvas_set_rect(V2_CHANNEL, 0, xmin, ymin, xmax, ymax, 3, COLOR_WHITE);
				char text_str[20];
				snprintf(text_str, sizeof(text_str), "%s %d", coco_name_get_by_id(class_id), (int)(res[i].result[1] * 100));
				canvas_set_text(V2_CHANNEL, 0, xmin, ymin - 32, text_str, COLOR_CYAN);
			}
		}
	}
	canvas_update(V2_CHANNEL, 0, 1);
}

static int no_motion_count = 0;
static void md_process(void *md_result)
{
	md_result_t *md_res = (md_result_t *) md_result;
	int motion = md_res->motion_cnt;

	if (motion) {
		printf("Motion Detected\r\n");
		no_motion_count = 0;
	} else {
		no_motion_count++;
	}

	//clear nn result when no motion
	if (no_motion_count > 2) {
		canvas_create_bitmap(V2_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
		canvas_update(V2_CHANNEL, 0, 1);
	}

}
#endif  /* ENA_MD_NN */

extern uint32_t initial_tick_count;
void mmf2_video_example_joint_test_rtsp_mp4_init_fcs(void)
{
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, V1_WIDTH, V1_HEIGHT, V1_BPS, 1,
						1, V2_WIDTH, V2_HEIGHT, V2_BPS, 1,
						0, 0, 0, 0, 0,
						ENA_MD_NN, V4_WIDTH, V4_HEIGHT);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 1, &video_v2_params, 1, NULL, 0, (ENA_MD_NN ? &video_v4_params : NULL));
#endif
	atcmd_fcs_init();
	//printf("\r\n voe heap size = %d\r\n", voe_heap_size);
	video_boot_stream_t *isp_fcs_info;
	video_get_fcs_info(&isp_fcs_info);//Get the fcs info
	printf("initial_tick_count = %d, fcs start time = %d\r\n", initial_tick_count, isp_fcs_info->fcs_start_time);

	//--------------Audio --------------
	uint32_t audio_start = mm_read_mediatime_ms();
	audio_ctx = mm_module_open(&audio_module);
	memcpy((void *)&audio_params, (void *)&default_audio_params, sizeof(audio_params_t));
	audio_params.avsync_en = 1;
	if (audio_ctx) {
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
#if FCS_AV_SYNC
		uint32_t audio_expected_queue = 800; //set 800 length as the maximum value
		uint32_t audio_apply_time = mm_read_mediatime_ms();
		uint32_t audio_frame_ms;
		mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_FRAMESIZE_MS, (int)&audio_frame_ms);
		if (audio_frame_ms) {
			if (audio_expected_queue > (audio_apply_time - isp_fcs_info->fcs_start_time) / audio_frame_ms) {
				audio_expected_queue = (audio_apply_time - isp_fcs_info->fcs_start_time) / audio_frame_ms;
			}
		}
		printf("audio length = %d\r\n", audio_expected_queue);
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, audio_expected_queue); //Add the queue buffer to avoid to lost data.
#else
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6); //queue size can be smaller 160ms
#endif
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		rt_printf("audio open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}

	int fcs_start_ch = -1;//Get the first start fcs channel
	if (isp_fcs_info->fcs_status) {
		for (int i = 0; i < 2; i++) { //Maximum two channel
			if (fcs_start_ch == -1 && isp_fcs_info->video_params[i].fcs == 1) {
				fcs_start_ch = i;
				printf("fcs_start_ch %d\r\n", fcs_start_ch);
			}
		}
		//sync video settings from fcs data
		if ((video_boot_stream.isp_info.sensor_width == isp_fcs_info->isp_info.sensor_width) &&
			(video_boot_stream.isp_info.sensor_height == isp_fcs_info->isp_info.sensor_height)) {
			memcpy(&video_boot_stream, isp_fcs_info, sizeof(video_boot_stream));
			video_v1_params.width = isp_fcs_info->video_params[STREAM_V1].width;
			video_v1_params.height = isp_fcs_info->video_params[STREAM_V1].height;
			video_v1_params.fps = isp_fcs_info->video_params[STREAM_V1].fps;
			video_v1_params.gop = isp_fcs_info->video_params[STREAM_V1].gop;
			mp4_v1_params.width = video_v1_params.width;
			mp4_v1_params.height = video_v1_params.height;
			//printf("ch 0 w %d h %d\r\n",mp4_v1_params.width,mp4_v1_params.height);
			video_v2_params.width = isp_fcs_info->video_params[STREAM_V2].width;
			video_v2_params.height = isp_fcs_info->video_params[STREAM_V2].height;
			video_v2_params.fps = isp_fcs_info->video_params[STREAM_V2].fps;
			video_v2_params.gop = isp_fcs_info->video_params[STREAM_V2].gop;
			//printf("ch 1 w %d h %d\r\n",video_v2_params.width,video_v2_params.height);
		}
	}
	if (isp_fcs_info->fcs_status == 1 && fcs_start_ch == 1) { //It need to change the order if the fcs channel is not zero
		if (isp_fcs_info->bps_stbl_ctrl_params[STREAM_V2].target_bitrate != 0) {
			bps_stbl_ctrl_en[STREAM_V2] = 1;
			bps_stbl_ctrl_v2_params.sampling_time = isp_fcs_info->bps_stbl_ctrl_params[STREAM_V2].sampling_time;
			bps_stbl_ctrl_v2_params.maximun_bitrate = isp_fcs_info->bps_stbl_ctrl_params[STREAM_V2].maximun_bitrate;
			bps_stbl_ctrl_v2_params.minimum_bitrate = isp_fcs_info->bps_stbl_ctrl_params[STREAM_V2].minimum_bitrate;
			bps_stbl_ctrl_v2_params.target_bitrate = isp_fcs_info->bps_stbl_ctrl_params[STREAM_V2].target_bitrate;
			bps_stbl_ctrl_v2_fps_stage[0] = video_v2_params.fps;
			bps_stbl_ctrl_v2_fps_stage[1] = (uint32_t)(video_v2_params.fps * 0.8);
			bps_stbl_ctrl_v2_fps_stage[2] = (uint32_t)(video_v2_params.fps * 0.6);
			bps_stbl_ctrl_v2_gop_stage[0] = video_v2_params.gop;
			bps_stbl_ctrl_v2_gop_stage[1] = (uint32_t)(video_v2_params.gop * 0.8);
			bps_stbl_ctrl_v2_gop_stage[2] = (uint32_t)(video_v2_params.gop * 0.6);
		}
		// ------ Channel 2--------------
		video_v2_ctx = mm_module_open(&video_module);
		if (video_v2_ctx) {
			video_v2_params.type = SHAPSHOT_TYPE;
			mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v2_params);
			mm_module_ctrl(video_v2_ctx, MM_CMD_SET_QUEUE_LEN, V2_FPS * 10); //Add the queue buffer to avoid to lost data.
			mm_module_ctrl(video_v2_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
			mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SNAPSHOT_CB, (int)fcs_snapshot_cb);
			mm_module_ctrl(video_v2_ctx, CMD_VIDEO_APPLY, V2_CHANNEL);	// start channel 1
			if (bps_stbl_ctrl_en[STREAM_V2]) {
				mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_FPS_STG, (int)bps_stbl_ctrl_v2_fps_stage);
				mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_GOP_STG, (int)bps_stbl_ctrl_v2_gop_stage);
				mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_PARAMS, (int)&bps_stbl_ctrl_v2_params);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_BPS_STBL_CTRL_EN, 1);
			}
		} else {
			rt_printf("video open fail\n\r");
			goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
		}
		// ------ Channel 1--------------
		video_v1_ctx = mm_module_open(&video_module);
		if (video_v1_ctx) {
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
			mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, V1_FPS * 10); //Add the queue buffer to avoid to lost data.
			mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
		} else {
			rt_printf("video open fail\n\r");
			goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
		}
	} else {
		if (isp_fcs_info->bps_stbl_ctrl_params[STREAM_V1].target_bitrate != 0) {
			bps_stbl_ctrl_en[STREAM_V1] = 1;
			bps_stbl_ctrl_v1_params.sampling_time = isp_fcs_info->bps_stbl_ctrl_params[STREAM_V1].sampling_time;
			bps_stbl_ctrl_v1_params.maximun_bitrate = isp_fcs_info->bps_stbl_ctrl_params[STREAM_V1].maximun_bitrate;
			bps_stbl_ctrl_v1_params.minimum_bitrate = isp_fcs_info->bps_stbl_ctrl_params[STREAM_V1].minimum_bitrate;
			bps_stbl_ctrl_v1_params.target_bitrate = isp_fcs_info->bps_stbl_ctrl_params[STREAM_V1].target_bitrate;
			bps_stbl_ctrl_v1_fps_stage[0] = video_v1_params.fps;
			bps_stbl_ctrl_v1_fps_stage[1] = (uint32_t)(video_v1_params.fps * 0.8);
			bps_stbl_ctrl_v1_fps_stage[2] = (uint32_t)(video_v1_params.fps * 0.6);
			bps_stbl_ctrl_v1_gop_stage[0] = video_v1_params.gop;
			bps_stbl_ctrl_v1_gop_stage[1] = (uint32_t)(video_v1_params.gop * 0.8);
			bps_stbl_ctrl_v1_gop_stage[2] = (uint32_t)(video_v1_params.gop * 0.6);
		}
		// ------ Channel 1--------------
		video_v1_ctx = mm_module_open(&video_module);
		if (video_v1_ctx) {
			video_v1_params.type = SHAPSHOT_TYPE;
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
			mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, V1_FPS * 10); //Add the queue buffer to avoid to lost data.
			mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SNAPSHOT_CB, (int)fcs_snapshot_cb);
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
			if (bps_stbl_ctrl_en[STREAM_V1]) {
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_FPS_STG, (int)bps_stbl_ctrl_v1_fps_stage);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_GOP_STG, (int)bps_stbl_ctrl_v1_gop_stage);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_BPS_STBL_CTRL_PARAMS, (int)&bps_stbl_ctrl_v1_params);
				mm_module_ctrl(video_v1_ctx, CMD_VIDEO_BPS_STBL_CTRL_EN, 1);
				mp4_v1_params.append_header = 1;//enable appending header, since the auto rate control may update the video header
			}
		} else {
			rt_printf("video open fail\n\r");
			goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
		}
		// ------ Channel 2--------------
		video_v2_ctx = mm_module_open(&video_module);
		if (video_v2_ctx) {
			mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v2_params);
			mm_module_ctrl(video_v2_ctx, MM_CMD_SET_QUEUE_LEN, V2_FPS * 10); //Add the queue buffer to avoid to lost data.
			mm_module_ctrl(video_v2_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
			mm_module_ctrl(video_v2_ctx, CMD_VIDEO_APPLY, V2_CHANNEL);	// start channel 1
		} else {
			rt_printf("video open fail\n\r");
			goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
		}
	}

	if (isp_fcs_info->fcs_status) {
		//enable the setting of fcs avsync
		fcs_avsync(FCS_AV_SYNC);
	}

	aac_ctx = mm_module_open(&aac_module);
	if (aac_ctx) {
		mm_module_ctrl(aac_ctx, CMD_AAC_SET_PARAMS, (int)&aac_params);
		mm_module_ctrl(aac_ctx, MM_CMD_SET_QUEUE_LEN, 30);
		mm_module_ctrl(aac_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(aac_ctx, CMD_AAC_INIT_MEM_POOL, 0);
		mm_module_ctrl(aac_ctx, CMD_AAC_APPLY, 0);
	} else {
		rt_printf("AAC open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}

	//--------------MP4---------------
	mp4_ctx = mm_module_open(&mp4_module);
	if (mp4_ctx) {
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_PARAMS, (int)&mp4_v1_params);
		mm_module_ctrl(mp4_ctx, CMD_MP4_LOOP_MODE, 0);
		mm_module_ctrl(mp4_ctx, CMD_MP4_START, mp4_v1_params.record_file_num);
	} else {
		rt_printf("MP4 open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}

	//--------------RTSP---------------
	rtsp2_v2_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v2_ctx) {
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v2_params);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_APPLY, 0);

		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SELECT_STREAM, 1);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_a_params);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}

	//--------------Link---------------------------
	siso_audio_aac = siso_create();
	if (siso_audio_aac) {
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_OUTPUT, (uint32_t)aac_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(siso_audio_aac);
	} else {
		rt_printf("siso1 open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}
	//vTaskDelay(300);
	//rt_printf("siso started\n\r");

	mimo_2v_1a_rtsp_mp4 = mimo_create();
	if (mimo_2v_1a_rtsp_mp4) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_ADD_INPUT1, (uint32_t)video_v2_ctx, 0);
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_ADD_INPUT2, (uint32_t)aac_ctx, 0);
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_ADD_OUTPUT0, (uint32_t)mp4_ctx, MMIC_DEP_INPUT0 | MMIC_DEP_INPUT2);
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_ADD_OUTPUT1, (uint32_t)rtsp2_v2_ctx, MMIC_DEP_INPUT1 | MMIC_DEP_INPUT2);
		mimo_start(mimo_2v_1a_rtsp_mp4);
	} else {
		rt_printf("mimo open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}
	//rt_printf("mimo started\n\r");

	// RTP audio

	rtp_ctx = mm_module_open(&rtp_module);
	if (rtp_ctx) {
		mm_module_ctrl(rtp_ctx, CMD_RTP_SET_PARAMS, (int)&rtp_aad_params);
		mm_module_ctrl(rtp_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(rtp_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(rtp_ctx, CMD_RTP_APPLY, 0);
		mm_module_ctrl(rtp_ctx, CMD_RTP_STREAMING, 1);	// streamming on
	} else {
		rt_printf("RTP open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}

	aad_ctx = mm_module_open(&aad_module);
	if (aad_ctx) {
		mm_module_ctrl(aad_ctx, CMD_AAD_SET_PARAMS, (int)&aad_rtp_params);
		mm_module_ctrl(aad_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(aad_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(aad_ctx, CMD_AAD_APPLY, 0);
	} else {
		rt_printf("AAD open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}

	siso_rtp_aad = siso_create();
	if (siso_rtp_aad) {
		siso_ctrl(siso_rtp_aad, MMIC_CMD_ADD_INPUT, (uint32_t)rtp_ctx, 0);
		siso_ctrl(siso_rtp_aad, MMIC_CMD_ADD_OUTPUT, (uint32_t)aad_ctx, 0);
		siso_ctrl(siso_rtp_aad, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(siso_rtp_aad);
	} else {
		rt_printf("siso1 open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}

	//rt_printf("siso3 started\n\r");

	siso_aad_audio = siso_create();
	if (siso_aad_audio) {
		siso_ctrl(siso_aad_audio, MMIC_CMD_ADD_INPUT, (uint32_t)aad_ctx, 0);
		siso_ctrl(siso_aad_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
		siso_start(siso_aad_audio);
	} else {
		rt_printf("siso2 open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}

#if ENA_MD_NN
	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}

	char md_mask [MD_MASK_ROW * MD_MASK_COL] = {0};
	memset(md_mask, 1, sizeof(md_mask));
	md_ctx  = mm_module_open(&eip_module);
	if (md_ctx) {
		md_config_t md_config;
		mm_module_ctrl(md_ctx, CMD_EIP_GET_MD_CONFIG, (int)&md_config); //get default md config
		md_config.md_trigger_block_threshold = 3; //md triggered when at least 3 motion block triggered
		memset(md_config.md_mask, 1, sizeof(md_config.md_mask));
		mm_module_ctrl(md_ctx, CMD_EIP_SET_PARAMS, (int)&eip_param);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_DISPPOST, (int)md_process);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_CONFIG, (int)&md_config);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_OUTPUT, 1);  //enable module output
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_EN, 1);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_START);

		mm_module_ctrl(md_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(md_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("md_ctx open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}

	vipnn_ctx = mm_module_open(&vipnn_module);
	if (vipnn_ctx) {
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_MODEL, (int)&yolov4_tiny);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_DISPPOST, (int)nn_set_object);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_CONFIDENCE_THRES, (int)&nn_confidence_thresh);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_NMS_THRES, (int)&nn_nms_thresh);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(objdetect_res_t));		// result size
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}
	printf("VIPNN opened\n\r");

	siso_rgb_md = siso_create();
	if (siso_rgb_md) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_rgb_md, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_rgb_md, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		siso_ctrl(siso_rgb_md, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_rgb_md, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_ctrl(siso_rgb_md, MMIC_CMD_ADD_OUTPUT, (uint32_t)md_ctx, 0);
		siso_start(siso_rgb_md);
	} else {
		printf("siso_rgb_md open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}
	printf("siso_rgb_md started\n\r");
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, V4_CHANNEL);	// start channel 4
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);

	siso_md_nn = siso_create();
	if (siso_md_nn) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_md_nn, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_md_nn, MMIC_CMD_ADD_INPUT, (uint32_t)md_ctx, 0);
		siso_ctrl(siso_md_nn, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_md_nn, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_ctrl(siso_md_nn, MMIC_CMD_ADD_OUTPUT, (uint32_t)vipnn_ctx, 0);
		siso_start(siso_md_nn);
	} else {
		printf("siso_md_nn open fail\n\r");
		goto mmf2_video_exmaple_joint_test_rtsp_mp4_fail;
	}
	printf("siso_md_nn started\n\r");

	int ch_enable[3] = {0, 1, 0};
	int char_resize_w[3] = {0, 16, 0}, char_resize_h[3] = {0, 32, 0};
	int ch_width[3] = {0, V2_WIDTH, 0}, ch_height[3] = {0, V2_HEIGHT, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);
#endif  /* ENA_MD_NN */


	atcmd_userctrl_init();

	vTaskDelay(5000);
	//print the fcs firsy frame message
	int fcs_video_starttime = 0;
	int fcs_video_endtime = 0;
	int fcs_audio_data_starttime = 0;
	int fcs_audio_dummy_starttime = 0;
	video_get_fcs_queue_info(&fcs_video_starttime, &fcs_video_endtime);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_FIRST_DATA_TS, (int)&fcs_audio_data_starttime);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_FIRST_DUMMY_TS, (int)&fcs_audio_dummy_starttime);
	printf("fcs vi: %d, v: %d, a_start: %d, a_dummy: %d, a_data: %d\r\n", fcs_video_endtime, fcs_video_starttime, audio_start, fcs_audio_dummy_starttime,
		   fcs_audio_data_starttime);
	return;
mmf2_video_exmaple_joint_test_rtsp_mp4_fail:

	return;
}

static const char *example = "mmf2_video_example_joint_test_rtsp_mp4_init_fcs";
static void example_deinit(int need_pause)
{
#if ENA_MD_NN
	if (md_ctx) {
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_STOP);
	}
	osd_render_task_stop();
	osd_render_dev_deinit_all();
#endif

	//Pause Linker
	siso_pause(siso_audio_aac);
	mimo_pause(mimo_2v_1a_rtsp_mp4, MM_OUTPUT0 | MM_OUTPUT1);
	siso_pause(siso_rtp_aad);
	siso_pause(siso_aad_audio);
#if ENA_MD_NN
	siso_pause(siso_md_nn);
	siso_pause(siso_rgb_md);
#endif

	//Stop module
	mm_module_ctrl(rtp_ctx, CMD_RTP_STREAMING, 0);
	mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(aac_ctx, CMD_AAC_STOP, 0);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_TRX, 0);
	mm_module_ctrl(mp4_ctx, CMD_MP4_STOP, 0);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, 0);
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_STREAM_STOP, 0);
#if ENA_MD_NN
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_STREAM_STOP, 0);
#endif

	//Delete linker
	siso_delete(siso_aad_audio);
	siso_delete(siso_rtp_aad);
	mimo_delete(mimo_2v_1a_rtsp_mp4);
	siso_delete(siso_audio_aac);
#if ENA_MD_NN
	siso_delete(siso_md_nn);
	siso_delete(siso_rgb_md);
#endif

	//Close module
	aad_ctx = mm_module_close(aad_ctx);
	rtp_ctx = mm_module_close(rtp_ctx);
	rtsp2_v2_ctx = mm_module_close(rtsp2_v2_ctx);
	aac_ctx = mm_module_close(aac_ctx);
	audio_ctx = mm_module_close(audio_ctx);
	mp4_ctx = mm_module_close(mp4_ctx);
	video_v1_ctx = mm_module_close(video_v1_ctx);
	video_v2_ctx = mm_module_close(video_v2_ctx);
#if ENA_MD_NN
	md_ctx = mm_module_close(md_ctx);
	vipnn_ctx = mm_module_close(vipnn_ctx);
	video_rgb_ctx = mm_module_close(video_rgb_ctx);
#endif

	video_voe_release();
}

#if ENA_SLEEP_TEST
#include "power_mode_api.h"
#include "wifi_conf.h"
#include "lwip_netconf.h"
extern int rtl8735b_suspend(int mode);
extern void rtl8735b_set_lps_pg(void);
#endif

static void fUC(void *arg)
{
	static uint32_t user_cmd = 0;

	if (!strcmp(arg, "TD")) {
		if (user_cmd & USR_CMD_EXAMPLE_DEINIT) {
			printf("invalid state, can not do %s deinit!\r\n", example);
		} else {
			example_deinit(user_cmd);
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
#if ENA_SLEEP_TEST
	} else if (!strcmp(arg, "SLEEP")) {
		//test fcs with retention data
		if ((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) || (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID)) {
			// sleep
			rtl8735b_set_lps_pg();
			rtw_enter_critical(NULL, NULL);
			if (rtl8735b_suspend(0) == 0) { // should stop wifi application before doing rtl8735b_suspend
				rtw_exit_critical(NULL, NULL);
				dbg_printf("wakeup after 3 sec\r\n");
				Standby(SLP_AON_TIMER | SLP_GTIMER, 3000000 /* 3s */, 0 /* CLOCK */, 1 /* SRAM retention */);
			} else {
				rtw_exit_critical(NULL, NULL);
				printf("rtl8735b_suspend fail\r\n");
				sys_reset();
			}
		} else {
			printf("wakeup after 3 sec\r\n"); //printf cannot show
			Standby(SLP_AON_TIMER | SLP_GTIMER, 3000000 /* 3s */, 0 /* CLOCK */, 1 /* SRAM retention */);
		}
#endif
	} else {
		printf("invalid cmd");
	}

	printf("user command 0x%x\r\n", user_cmd);
}

static log_item_t userctrl_items[] = {
	{"UC", fUC, },
};

static log_item_t at_fcs_change_items[ ] = {
	{"FCST", fcs_change,},
	{"FCSG", fcs_info,},
};

static void atcmd_userctrl_init(void)
{
	log_service_add_table(userctrl_items, sizeof(userctrl_items) / sizeof(userctrl_items[0]));
}
static void atcmd_fcs_init(void)
{
	log_service_add_table(at_fcs_change_items, sizeof(at_fcs_change_items) / sizeof(at_fcs_change_items[0]));
	printf("FCST=ch,width,height,iq_id,video_pre_init -> change the fcs resolution; FCSG get the fcs resolution info\r\n");
}