/******************************************************************************
*
* Copyright(c) 2007 - 2024 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "fast_inf_example.h"
#include "sensor.h"
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_simo.h"
#include "mmf2_mimo.h"
#include "video_boot.h"
#include "module_video.h"
#include "module_vipnn.h"
#include "module_rtsp2.h"
#include "module_audio.h"
#include "module_aac.h"
#include "module_aad.h"
#include "module_rtp.h"
#include "module_eip.h"
#include "module_mp4.h"
#include "video_snapshot.h"
#include "log_service.h"
#include "avcodec.h"
#include "sys_api.h"
#include "mmf2_pro2_video_config.h"
#include "nn_utils/class_name.h"
#include "model_yolo.h"
#include "us_ticker_api.h"

static void atcmd_userctrl_init(void);
static mm_context_t *video_v1_ctx           = NULL;
static mm_context_t *video_v2_ctx           = NULL;
static mm_context_t *video_nv12_ctx         = NULL;
static mm_context_t *video_vExt_ctx			= NULL;
static mm_context_t *rtsp2_v2_ctx           = NULL;
static mm_context_t *audio_ctx              = NULL;
static mm_context_t *aac_ctx                = NULL;
static mm_context_t *rtp_ctx                = NULL;
static mm_context_t *aad_ctx                = NULL;
static mm_context_t *mp4_ctx                = NULL;
static mm_context_t *md_ctx                 = NULL;
static mm_context_t *video_rgb_ctx          = NULL;
static mm_context_t *objdet_ctx             = NULL;

static mm_siso_t *siso_audio_aac            = NULL;
static mm_mimo_t *mimo_2v_1a_rtsp_mp4       = NULL;
static mm_siso_t *siso_rtp_aad              = NULL;
static mm_siso_t *siso_aad_audio            = NULL;
static mm_siso_t *siso_rgb_yolo   			= NULL;
static mm_siso_t *siso_nv12_md              = NULL;

/*****************************************************************************
* Audio
*****************************************************************************/
#define ENABLE_AUDIO       		1
#if ENABLE_AUDIO
static audio_params_t audio_params;
static TX_cfg_t audio_tx_cfg;
static RX_cfg_t audio_rx_cfg;
static void audio_params_customized_setting(void)
{
	memcpy(&audio_params, &default_audio_params, sizeof(audio_params_t));
	audio_params.sample_rate = ASR_16KHZ;  // NN audio classification require 16K
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_TXASP_PARAM, (int)&audio_tx_cfg);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_RXASP_PARAM, (int)&audio_rx_cfg);
	audio_rx_cfg.aec_cfg.AEC_EN = 1; 	// enable AEC, NS, AGC for Ameba-->Peer
	audio_rx_cfg.ns_cfg.NS_EN = 1;      // enable AEC, NS, AGC for Ameba-->Peer
	audio_tx_cfg.agc_cfg.AGC_EN = 1;    // enable AGC/DRC for Peer-->Ameba
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_TXASP_PARAM, (int)&audio_tx_cfg);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_RXASP_PARAM, (int)&audio_rx_cfg);
	//audio_params.enable_aec = 1;  // enable AEC, NS, AGC for Ameba-->Peer
	//audio_params.enable_ns = 0;   // disable MS for Peer-->Ameba
	//audio_params.enable_agc = 1;  // enable AGC/DRC for Peer-->Ameba
}
#endif

/*****************************************************************************
* ISP channel : 0,1
* Video type  : H264/HEVC,H264/HEVC
*****************************************************************************/
//For mp4
#define V1_CHANNEL 0
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR
#define EN_META_DATA 0 // if enable, need to define META_DATA_TEST in video_user_boot.c
//For rtsp
#define RTSP_CHANNEL 1
#define RTSP_BPS 1*1024*1024
#define VIDEO_RCMODE 2 // 1: CBR, 2: VBR
#define USE_H265 0
#if USE_H265
#include "sample_h265.h"
#define RTSP_TYPE VIDEO_HEVC
#define VIDEO_TYPE VIDEO_HEVC
#define RTSP_CODEC AV_CODEC_ID_H265
#else
#include "sample_h264.h"
#define RTSP_TYPE VIDEO_H264
#define VIDEO_TYPE VIDEO_H264
#define RTSP_CODEC AV_CODEC_ID_H264
#endif
#define RTSP_WIDTH	1280
#define RTSP_HEIGHT	720

static video_params_t video_v1_params = {
	.stream_id 		= V1_CHANNEL,
	.type 			= VIDEO_TYPE,
	.bps            = V1_BPS,
	.rc_mode        = V1_RCMODE,
	.resolution 	= VIDEO_FHD,
	.width 			= 1920,
	.height 		= 1080,
	.use_static_addr = 1,
	.fps = 20,
	.gop = 20,
};

static video_params_t video_v2_params = {
	.stream_id 		= RTSP_CHANNEL,
	.type 			= RTSP_TYPE,
	.bps            = RTSP_BPS,
	.rc_mode        = VIDEO_RCMODE,
	.resolution 	= VIDEO_HD,
	.width 			= RTSP_WIDTH,
	.height 		= RTSP_HEIGHT,
	.use_static_addr = 1,
	.fps = 20,
	.gop = 20,
};

static rtsp2_params_t rtsp2_v2_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = RTSP_CODEC,
			.bps      = RTSP_BPS
		}
	}
};

static rtsp2_params_t rtsp2_a_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a = {
			.codec_id   = AV_CODEC_ID_MP4A_LATM,
			.channel    = 1,
			.samplerate = 16000
		}
	}
};

static aac_params_t aac_params = {
	.sample_rate = 16000,
	.channel = 1,
	.trans_type = AAC_TYPE_ADTS,
	.object_type = AAC_AOT_LC,
	.bitrate = 32000,

	.mem_total_size = 10 * 1024,
	.mem_block_size = 128,
	.mem_frame_size = 1024
};

static aad_params_t aad_rtp_params = {
	.sample_rate = 16000,
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

static rate_ctrl_t rate_ctrl_v1_params;

static mp4_params_t mp4_v1_params = {
	.sample_rate = 16000,
	.channel = 1,
	.record_length = 30, //seconds
#if ENABLE_AUDIO
	.record_type = STORAGE_ALL,
#else
	.record_type = STORAGE_VIDEO,
#endif
	.record_file_num = 3000,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
};

/*****************************************************************************
* ISP channel : 2
* Video type  : NV12
*****************************************************************************/
#define ENABLE_MD 1
#define WAIT_MD_RESULT 1
#define MD_CHANNEL 2
#define MD_WIDTH 128
#define MD_HEIGHT 128
#define MD_FPS 10
#define MD_TYPE VIDEO_NV12
#define MD_COL 32
#define MD_ROW 32

static video_params_t video_v3_params = {
	.stream_id 		= MD_CHANNEL,
	.type 			= MD_TYPE,
	.fps 			= MD_FPS,
	.width 			= MD_WIDTH,
	.height 		= MD_HEIGHT,
	.direct_output 	= 0,
	.use_static_addr = 0,
	.use_roi = 1,
	.roi = {
		.xmin = 0,
		.ymin = 0,
		.xmax = sensor_params[USE_SENSOR].sensor_width,
		.ymax = sensor_params[USE_SENSOR].sensor_height,
	}
};

static eip_param_t md_param = {
	.eip_row = MD_ROW,
	.eip_col = MD_COL
};

static int start_tick = 0;
char time_message[1024] = "";
char md_time_message[1024] = "";
char nn_time_message[1024] = "";
static char text[64] = "";
static int md_framecount = 0;

static void md_process(void *md_result)
{
	md_result_t *md_res = (md_result_t *) md_result;
	int motion = md_res->motion_cnt;
	static int first_md_trigger = 0;
	char md_text[64] = "";
	md_framecount++;
	if (md_framecount == 1) {
		snprintf(md_text, sizeof(md_text), "md first callback %d\r\n", us_ticker_read()/1000);
		strcat(md_time_message, md_text);
	}
	if (md_framecount == 4) {
		printf("first motion result\r\n");
		snprintf(md_text, sizeof(md_text), "md four callback %d\r\n", us_ticker_read()/1000);
		strcat(md_time_message, md_text);
	}
	if (motion && first_md_trigger == 0) {
		first_md_trigger = 1;
		snprintf(md_text, sizeof(md_text), "md first trigger %d frame %d\r\n", us_ticker_read()/1000, md_framecount);
		strcat(md_time_message, md_text);
	}
}

/*****************************************************************************
* ISP channel : 3
* Video type  : JPEG
*****************************************************************************/
#ifndef ENABLE_V_EXT
#define ENABLE_V_EXT       		1   /* fix here: enable ENABLE_V_EXT */
#endif

#if ENABLE_V_EXT
//#define ENABLE_SD_SNAPSHOT //Enable the snapshot to sd card
#define JPEG_CHANNEL 3
#define JPEG_WIDTH	320
#define JPEG_HEIGHT	240
#define JPEG_FPS	5
#define SHAPSHOT_TYPE VIDEO_JPEG

static video_params_t video_vExt_params = {
	.stream_id 	= JPEG_CHANNEL,
	.type 		= SHAPSHOT_TYPE,
	.width 		= JPEG_WIDTH,
	.height 	= JPEG_HEIGHT,
	.fps 		= JPEG_FPS,
	.use_static_addr = 1,
};

static int vExt_snapshot_cb(uint32_t jpeg_addr, uint32_t jpeg_len)
{
	//printf(">>>> snapshot jpeg_addr = 0x%x, size=%d\n\r", jpeg_addr, jpeg_len);
	return 0;
}

static TaskHandle_t snapshot_thread = NULL;
static void snapshot_control_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif
	while (1) {
		vTaskDelay(200);
		mm_module_ctrl(video_vExt_ctx, CMD_VIDEO_SNAPSHOT, 1);
	}
}
#endif

/*****************************************************************************
* ISP channel : 4
* Video type  : RGB
*****************************************************************************/
#define NN_MODEL_OBJ   yolo_fastest

#define NN_WIDTH	320
#define NN_HEIGHT	320
#define NN_CHANNEL 4
#define NN_FPS 10
#define NN_GOP NN_FPS
#define NN_TYPE VIDEO_RGB
#define NN_DRAW 0

static video_params_t video_v4_params = {
	.stream_id 		= NN_CHANNEL,
	.type 			= NN_TYPE,
	.width 			= NN_WIDTH,
	.height 		= NN_HEIGHT,
	.fps 			= NN_FPS,
	.gop			= NN_GOP,
	.direct_output 	= 0,
	.use_static_addr = 1,
	.use_roi = 1,
	.roi = {
		.xmin = 0,
		.ymin = 0,
		.xmax = sensor_params[USE_SENSOR].sensor_width,
		.ymax = sensor_params[USE_SENSOR].sensor_height,
	}
};

static nn_data_param_t roi_nn = {
	.codec_type = AV_CODEC_ID_RGB888
};

//--------------------------------------------
// Draw Rect
//--------------------------------------------
#include "osd_render.h"
#define LIMIT(x, lower, upper) if(x<lower) x=lower; else if(x>upper) x=upper;
static int first_nn_callback = 0;
static int nn_framecount = 0;
static void nn_set_object(void *p, void *img_param)
{
	int i = 0;
	char nn_text[64] = "";
	vipnn_out_buf_t *out = (vipnn_out_buf_t *)p;
	objdetect_res_t *res = (objdetect_res_t *)&out->res[0];

	int obj_num = out->res_cnt;
	nn_data_param_t *im = (nn_data_param_t *)img_param;

	if (!p || !img_param)	{
		return;
	}

	if (!first_nn_callback) {
		first_nn_callback = 1;
		printf("first nn result\r\n");
		snprintf(nn_text, sizeof(nn_text), "nn first callback %d\r\n", us_ticker_read()/1000);
		strcat(nn_time_message, nn_text);
	}

	nn_framecount++;
	if(nn_framecount == 3) {
		snprintf(nn_text, sizeof(nn_text), "nn third detect %d\r\n", us_ticker_read()/1000);
		strcat(nn_time_message, nn_text);
	}

#if NN_DRAW
	int im_h = RTSP_HEIGHT;
	int im_w = RTSP_WIDTH;

	float ratio_w = (float)im_w / (float)im->img.width;
	float ratio_h = (float)im_h / (float)im->img.height;
	int roi_h, roi_w, roi_x, roi_y;
	if (video_v4_params.use_roi == 1) { //resize
		roi_w = (int)((im->img.roi.xmax - im->img.roi.xmin) * ratio_w);
		roi_h = (int)((im->img.roi.ymax - im->img.roi.ymin) * ratio_h);
		roi_x = (int)(im->img.roi.xmin * ratio_w);
		roi_y = (int)(im->img.roi.ymin * ratio_h);
	} else {  //crop
		float ratio = ratio_h < ratio_w ? ratio_h : ratio_w;
		roi_w = (int)((im->img.roi.xmax - im->img.roi.xmin) * ratio);
		roi_h = (int)((im->img.roi.ymax - im->img.roi.ymin) * ratio);
		roi_x = (int)(im->img.roi.xmin * ratio + (im_w - roi_w) / 2);
		roi_y = (int)(im->img.roi.ymin * ratio + (im_h - roi_h) / 2);
	}

	// printf("object num = %d\r\n", obj_num);
	canvas_create_bitmap(RTSP_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
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
			canvas_set_rect(RTSP_CHANNEL, 0, xmin, ymin, xmax, ymax, 3, COLOR_WHITE);
			char text_str[20];
			snprintf(text_str, sizeof(text_str), "%s %d", coco_name_get_by_id(class_id), (int)(res[i].result[1] * 100));
			canvas_set_text(RTSP_CHANNEL, 0, xmin, ymin - 32, text_str, COLOR_CYAN);
		}
	}
	canvas_update(RTSP_CHANNEL, 0, 1);
#endif
}

extern int get_inf_result;
void mmf2_video_example_joint_test_fast_inf_init(void)
{

	start_tick = us_ticker_read()/1000;
	snprintf(text, sizeof(text), "media init start %d\r\n", start_tick);
	strcat(time_message, text);

	/*rtsp parameter setting*/
	rtsp2_v2_params.u.v.fps = video_v2_params.fps;
	/*mp4 parameter setting*/
	mp4_v1_params.fps = video_v1_params.fps;
	mp4_v1_params.gop = video_v1_params.gop;
	mp4_v1_params.width = video_v1_params.width;
	mp4_v1_params.height = video_v1_params.height;
	/*md parameter setting*/
	md_param.image_width = video_v3_params.width;
	md_param.image_height = video_v3_params.height;
	/*nn parameter setting*/
	roi_nn.img.width = video_v4_params.width;
	roi_nn.img.height = video_v4_params.height;
	roi_nn.img.roi.xmin = 0,
	roi_nn.img.roi.ymin = 0,
	roi_nn.img.roi.xmax = video_v4_params.width;
	roi_nn.img.roi.ymax = video_v4_params.height;

	int voe_heap_size = video_voe_presetting(1, video_v1_params.width, video_v1_params.height, V1_BPS, 0,
						1, video_v2_params.width, video_v2_params.height, RTSP_BPS, 0,
						1, video_v3_params.width, video_v3_params.height, 0, 0,
						1, video_v4_params.width, video_v4_params.height);
#if ENABLE_V_EXT
	voe_heap_size = video_extra_voe_presetting(voe_heap_size, 1, video_vExt_params.width, video_vExt_params.height, 0, 1);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_boot_stream_t *isp_fcs_info;
	video_get_fcs_info(&isp_fcs_info); //Get the fcs info

	// ------ Channel 1--------------
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
#if EN_META_DATA
		unsigned char uuid[16] = {0xc7, 0x98, 0x2c, 0x28, 0x0a, 0xfc, 0x49, 0xe6, 0xaa, 0xe4, 0x7f, 0x8f, 0x64, 0xee, 0x65, 0x01};
		video_pre_init_params_t init_params;
		memset(&init_params, 0x00, sizeof(video_pre_init_params_t));
		init_params.meta_enable = 1;
		init_params.meta_size = VIDEO_META_USER_SIZE;
		memcpy(init_params.video_meta_uuid, uuid, VIDEO_META_UUID_SIZE);
		video_pre_init_setup_parameters(&init_params);
		video_v1_params.meta_enable = 1;
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_META_CB, MMF_VIDEO_DEFAULT_META_CB);
#endif
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_EN_DBG_TS_INFO, 1);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps * 8);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		if (isp_fcs_info->auto_rate_control[STREAM_V1].sampling_time != 0) {
			rate_ctrl_v1_params.sampling_time = isp_fcs_info->auto_rate_control[STREAM_V1].sampling_time;
			rate_ctrl_v1_params.maximun_bitrate = isp_fcs_info->auto_rate_control[STREAM_V1].maximun_bitrate;
			rate_ctrl_v1_params.minimum_bitrate = isp_fcs_info->auto_rate_control[STREAM_V1].minimum_bitrate;
			rate_ctrl_v1_params.target_bitrate = isp_fcs_info->auto_rate_control[STREAM_V1].target_bitrate;
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_RATE_CONTROL, (int)&rate_ctrl_v1_params);
			mp4_v1_params.append_header = 1;//enable appending header, since the auto rate control may update the video header
		}
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	start_tick = us_ticker_read()/1000;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
	snprintf(text, sizeof(text), "ch0 apply %d %d\r\n", start_tick, us_ticker_read()/1000);
	strcat(time_message, text);

	//--------------Motion Detection---------------
	// V3 (NV12) + MD
#if ENABLE_MD
	video_nv12_ctx = mm_module_open(&video_module);
	if (video_nv12_ctx) {
		mm_module_ctrl(video_nv12_ctx, CMD_VIDEO_EN_DBG_TS_INFO, 1);
		mm_module_ctrl(video_nv12_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v3_params);
		mm_module_ctrl(video_nv12_ctx, MM_CMD_SET_QUEUE_LEN, video_v3_params.fps * 2);
		mm_module_ctrl(video_nv12_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video nv12 open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	start_tick = us_ticker_read()/1000;
	mm_module_ctrl(video_nv12_ctx, CMD_VIDEO_APPLY, MD_CHANNEL);	// start channel 3
	mm_module_ctrl(video_nv12_ctx, CMD_VIDEO_YUV, 2);
	snprintf(text, sizeof(text), "ch2 apply %d %d\r\n", start_tick, us_ticker_read()/1000);
	strcat(time_message, text);
#endif
	// ------ Channel 4--------------
	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_EN_DBG_TS_INFO, 1);
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 3);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	start_tick = us_ticker_read()/1000;
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);
	snprintf(text, sizeof(text), "ch4 apply %d %d\r\n", start_tick, us_ticker_read()/1000);
	strcat(time_message, text);

#if ENABLE_MD
	//--------------Motion Detection---------------
	// V3 (NV12) + MD
	start_tick = us_ticker_read()/1000;
	char md_mask [MD_MASK_ROW * MD_MASK_COL] = {0};
	memset(md_mask, 1, sizeof(md_mask));
	md_ctx = mm_module_open(&eip_module);
	if (md_ctx) {
		mm_module_ctrl(md_ctx, CMD_EIP_SET_PARAMS, (int)&md_param);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_MASK, (int)&md_mask);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_DISPPOST, (int)md_process);
		mm_module_ctrl(md_ctx, CMD_EIP_AE_STABLE_EN, 0);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_EN, 1);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_START);
	} else {
		printf("md_ctx open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	snprintf(text, sizeof(text), "md init %d %d\r\n", start_tick, us_ticker_read()/1000);
	strcat(time_message, text);

	siso_nv12_md = siso_create();
	if (siso_nv12_md) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_nv12_md, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_nv12_md, MMIC_CMD_ADD_INPUT, (uint32_t)video_nv12_ctx, 0);
		siso_ctrl(siso_nv12_md, MMIC_CMD_ADD_OUTPUT, (uint32_t)md_ctx, 0);
		siso_ctrl(siso_nv12_md, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_nv12_md, MMIC_CMD_SET_TASKNANE, (uint32_t)"ss_nv12_md", 0);
		siso_ctrl(siso_nv12_md, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_start(siso_nv12_md);
	} else {
		printf("siso_nv12_md open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	printf("siso_nv12_md started\n\r");
#endif
	//--------------Object Detection ---------------
	// VIPNN - YOLO
	start_tick = us_ticker_read()/1000;
	objdet_ctx = mm_module_open(&vipnn_module);
	if (objdet_ctx) {
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_DISPPOST, (int)nn_set_object);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(objdetect_res_t));		// result size
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count
		//mm_module_ctrl(objdet_ctx, MM_CMD_SET_DATAGROUP, MM_GROUP_END);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN_OBJDET open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	printf("VIPNN_OBJDET opened\n\r");
	snprintf(text, sizeof(text), "yolo init %d %d\r\n", start_tick, us_ticker_read()/1000);
	strcat(time_message, text);

	siso_rgb_yolo = siso_create();
	if (siso_rgb_yolo) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_rgb_yolo, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_rgb_yolo, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		siso_ctrl(siso_rgb_yolo, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_rgb_yolo, MMIC_CMD_ADD_OUTPUT0, (uint32_t)objdet_ctx, 0);
		siso_ctrl(siso_rgb_yolo, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_start(siso_rgb_yolo);
	} else {
		printf("siso_rgb_yolo open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	printf("siso_rgb_yolo started\n\r");

#if ENABLE_MD && WAIT_MD_RESULT
	int md_wait_time = 0;
	int md_timeout = 500;
	while (md_framecount < 4) {
		vTaskDelay(1);
		md_wait_time++;
		if (md_wait_time > md_timeout) {
			printf("md wait timeout\r\n");
			break;
		}
	}
	get_inf_result = 1;
#endif

	//change MD, NN FPS back
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_ISPFPS, NN_FPS);
#if ENABLE_MD
	mm_module_ctrl(video_nv12_ctx, CMD_VIDEO_ISPFPS, MD_FPS);
#endif

	// ------ Channel 2--------------
	video_v2_ctx = mm_module_open(&video_module);
	if (video_v2_ctx) {
		mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v2_params);
		mm_module_ctrl(video_v2_ctx, MM_CMD_SET_QUEUE_LEN, video_v2_params.fps * 8);
		mm_module_ctrl(video_v2_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	start_tick = us_ticker_read()/1000;
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_APPLY, RTSP_CHANNEL);// start channel 1
	snprintf(text, sizeof(text), "ch1 apply %d %d\r\n", start_tick, us_ticker_read()/1000);
	strcat(time_message, text);

#if ENABLE_AUDIO
	//--------------Audio --------------
	start_tick = us_ticker_read()/1000;
	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
		audio_params_customized_setting();
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_RUN_AEC, 1);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		printf("audio open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	snprintf(text, sizeof(text), "audio init %d %d\r\n", start_tick, us_ticker_read()/1000);
	strcat(time_message, text);

	//--------------ACC --------------
	aac_ctx = mm_module_open(&aac_module);
	if (aac_ctx) {
		mm_module_ctrl(aac_ctx, CMD_AAC_SET_PARAMS, (int)&aac_params);
		mm_module_ctrl(aac_ctx, MM_CMD_SET_QUEUE_LEN, 16);
		mm_module_ctrl(aac_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(aac_ctx, CMD_AAC_INIT_MEM_POOL, 0);
		mm_module_ctrl(aac_ctx, CMD_AAC_APPLY, 0);
	} else {
		printf("AAC open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
#endif
	//--------------MP4---------------
	start_tick = us_ticker_read()/1000;
	mp4_ctx = mm_module_open(&mp4_module);
	if (mp4_ctx) {
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_PARAMS, (int)&mp4_v1_params);
		mm_module_ctrl(mp4_ctx, CMD_MP4_LOOP_MODE, 0);
		mm_module_ctrl(mp4_ctx, CMD_MP4_START, mp4_v1_params.record_file_num);
	} else {
		printf("MP4 open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	snprintf(text, sizeof(text), "mp4 init %d %d\r\n", start_tick, us_ticker_read()/1000);
	strcat(time_message, text);

	//--------------RTSP---------------
	start_tick = us_ticker_read()/1000;
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
		printf("RTSP2 open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	snprintf(text, sizeof(text), "rtsp init %d %d\r\n", start_tick, us_ticker_read()/1000);
	strcat(time_message, text);

	//--------------Link---------------------------
	mimo_2v_1a_rtsp_mp4 = mimo_create();
	if (mimo_2v_1a_rtsp_mp4) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_ADD_INPUT1, (uint32_t)video_v2_ctx, 0);
#if ENABLE_AUDIO
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_ADD_INPUT2, (uint32_t)aac_ctx, 0);
#endif
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_ADD_OUTPUT0, (uint32_t)mp4_ctx, MMIC_DEP_INPUT0 | MMIC_DEP_INPUT2);
		mimo_ctrl(mimo_2v_1a_rtsp_mp4, MMIC_CMD_ADD_OUTPUT1, (uint32_t)rtsp2_v2_ctx, MMIC_DEP_INPUT1 | MMIC_DEP_INPUT2);
		mimo_start(mimo_2v_1a_rtsp_mp4);
	} else {
		printf("mimo open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	printf("mimo started\n\r");

#if ENABLE_AUDIO
	// RTP audio
	rtp_ctx = mm_module_open(&rtp_module);
	if (rtp_ctx) {
		mm_module_ctrl(rtp_ctx, CMD_RTP_SET_PARAMS, (int)&rtp_aad_params);
		mm_module_ctrl(rtp_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(rtp_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(rtp_ctx, CMD_RTP_APPLY, 0);
		mm_module_ctrl(rtp_ctx, CMD_RTP_STREAMING, 1);	// streamming on
	} else {
		printf("RTP open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}

	aad_ctx = mm_module_open(&aad_module);
	if (aad_ctx) {
		mm_module_ctrl(aad_ctx, CMD_AAD_SET_PARAMS, (int)&aad_rtp_params);
		mm_module_ctrl(aad_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(aad_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(aad_ctx, CMD_AAD_APPLY, 0);
	} else {
		printf("AAD open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}

	siso_rtp_aad = siso_create();
	if (siso_rtp_aad) {
		siso_ctrl(siso_rtp_aad, MMIC_CMD_SET_TASKNANE, (uint32_t)"ss_rtp_aad", 0);
		siso_ctrl(siso_rtp_aad, MMIC_CMD_ADD_INPUT, (uint32_t)rtp_ctx, 0);
		siso_ctrl(siso_rtp_aad, MMIC_CMD_ADD_OUTPUT, (uint32_t)aad_ctx, 0);
		siso_ctrl(siso_rtp_aad, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(siso_rtp_aad);
	} else {
		printf("siso1 open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}

	printf("siso3 started\n\r");

	siso_aad_audio = siso_create();
	if (siso_aad_audio) {
		siso_ctrl(siso_aad_audio, MMIC_CMD_SET_TASKNANE, (uint32_t)"ss_aad_audio", 0);
		siso_ctrl(siso_aad_audio, MMIC_CMD_ADD_INPUT, (uint32_t)aad_ctx, 0);
		siso_ctrl(siso_aad_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
		siso_start(siso_aad_audio);
	} else {
		printf("siso2 open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}

	siso_audio_aac = siso_create();
	if (siso_audio_aac) {
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_OUTPUT, (uint32_t)aac_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(siso_audio_aac);
	} else {
		printf("siso_audio_aac open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	printf("siso_audio_aac started\n\r");
#endif

#if ENABLE_V_EXT
	//--------------Ext video channel for jpeg snapshot---------------
	start_tick = us_ticker_read()/1000;
	video_vExt_ctx = mm_module_open(&video_module);
	if (video_vExt_ctx) {
		mm_module_ctrl(video_vExt_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_vExt_params);
		mm_module_ctrl(video_vExt_ctx, CMD_VIDEO_SNAPSHOT_CB, (int)vExt_snapshot_cb);
	} else {
		rt_printf("video_vExt_ctx open fail\n\r");
		goto mmf2_video_example_joint_test_fast_inf_init_fail;
	}
	mm_module_ctrl(video_vExt_ctx, CMD_VIDEO_APPLY, JPEG_CHANNEL);
	snprintf(text, sizeof(text), "ch3 init %d %d\r\n", start_tick, us_ticker_read()/1000);
	strcat(time_message, text);
	if (xTaskCreate(snapshot_control_thread, ((const char *)"snapshot_store"), 512, NULL, tskIDLE_PRIORITY + 3, &snapshot_thread) != pdPASS) {
		printf("\n\r%s xTaskCreate failed", __FUNCTION__);
	}
#endif

#if NN_DRAW
	int ch_enable[3] = {0, 1, 0};
	int char_resize_w[3] = {0, 16, 0}, char_resize_h[3] = {0, 32, 0};
	int ch_width[3] = {0, RTSP_WIDTH, 0}, ch_height[3] = {0, RTSP_HEIGHT, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);
#endif

	atcmd_userctrl_init();

	return;
mmf2_video_example_joint_test_fast_inf_init_fail:

	return;
}

static const char *example = "mmf2_video_example_joint_test_vipnn_rtsp_mp4";
static void example_deinit(void)
{
	//OSD Deinit
	osd_render_task_stop();
	osd_render_dev_deinit_all();
	//snapshot task delete
#if ENABLE_V_EXT
	printf("Snapshot_thread Stop\r\n");
	vTaskDelete(snapshot_thread);
#endif
	printf("Pause Linker\r\n");
	//Pause Linker
#if ENABLE_AUDIO
	siso_pause(siso_rtp_aad);
	siso_pause(siso_aad_audio);
	siso_pause(siso_audio_aac);
#endif
	mimo_pause(mimo_2v_1a_rtsp_mp4, MM_OUTPUT0 | MM_OUTPUT1);

	mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_STOP);
	siso_pause(siso_nv12_md);

	siso_pause(siso_rgb_yolo);

	//Stop module
#if ENABLE_AUDIO
	mm_module_ctrl(rtp_ctx, CMD_RTP_STREAMING, 0);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_TRX, 0);
#endif
	mm_module_ctrl(mp4_ctx, CMD_MP4_STOP, 0);
	mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_STREAM_STOP, RTSP_CHANNEL);
	mm_module_ctrl(video_nv12_ctx, CMD_VIDEO_STREAM_STOP, MD_CHANNEL);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_STREAM_STOP, NN_CHANNEL);
#if ENABLE_V_EXT
	mm_module_ctrl(video_vExt_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);
#endif

	//Delete linker
#if ENABLE_AUDIO
	siso_delete(siso_rtp_aad);
	siso_delete(siso_aad_audio);
	siso_delete(siso_audio_aac);
#endif
	mimo_delete(mimo_2v_1a_rtsp_mp4);

	siso_delete(siso_nv12_md);
	siso_delete(siso_rgb_yolo);

	//Close module
#if ENABLE_AUDIO
	mm_module_close(aad_ctx);
	mm_module_close(rtp_ctx);
	mm_module_close(aac_ctx);
	mm_module_close(audio_ctx);
#endif
	mm_module_close(mp4_ctx);
	mm_module_close(video_v1_ctx);
	mm_module_close(rtsp2_v2_ctx);
	mm_module_close(video_v2_ctx);

#if ENABLE_MD
	mm_module_close(md_ctx);
	mm_module_close(video_nv12_ctx);
#endif

	mm_module_close(objdet_ctx);
	mm_module_close(video_rgb_ctx);

	video_voe_release();
}

static uint32_t wlan_init_start_tick = 0, wlan_init_end_tick = 0;
static uint32_t ssl_start_tick = 0, ssl_end_tick = 0;
static uint32_t first_tx_pkt_tick;
void wlan_init_start_time(void) {
	wlan_init_start_tick = us_ticker_read() / 1000;
}
void wlan_init_end_time(void) {
	wlan_init_end_tick = us_ticker_read() / 1000;
}
void ssl_connect_start_time(void) {
	ssl_start_tick = us_ticker_read() / 1000;
}
void ssl_connect_end_time(void) {
	ssl_end_tick = us_ticker_read() / 1000;
}
void rtl8735b_tx_pkt_callback(void) {
	static int count = 0;
	if(count == 0) {
		first_tx_pkt_tick = us_ticker_read() / 1000;
		count++;
	}	
}

static void wlan_time_message(void)
{
	char wlan_time_message[1024] = "";
	char text[64] = "";

	snprintf(text, sizeof(text), "wifi init %d %d\r\n", wlan_init_start_tick, wlan_init_end_tick);
	strcat(wlan_time_message, text);

	snprintf(text, sizeof(text), "wifi tx %d\r\n", first_tx_pkt_tick);
	strcat(wlan_time_message, text);

	snprintf(text, sizeof(text), "ssl init %d %d\r\n", ssl_start_tick, ssl_end_tick);
	strcat(wlan_time_message, text);

	printf("%s\r\n", wlan_time_message);
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
	} else if (!strcmp(arg, "TIME")) {
		printf("%s\r\n", time_message);
		wlan_time_message();
		printf("%s\r\n", md_time_message);
		printf("%s\r\n", nn_time_message);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SHOW_DBG_TS_INFO, 0);
		mm_module_ctrl(video_nv12_ctx, CMD_VIDEO_SHOW_DBG_TS_INFO, 0);
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SHOW_DBG_TS_INFO, 0);
	} else {
		printf("invalid cmd\r\n");
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

void fast_inf_video_example_task(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif
#if(CONFIG_RTK_EVB_IR_CTRL == 1)
	sensor_board_init();
#endif

	mmf2_video_example_joint_test_fast_inf_init();

	// TODO: exit condition or signal
	while (1) {
		vTaskDelay(10000);
		// extern mm_context_t *video_v1_ctx;
		// mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRINT_INFO, 0);
		// check video frame here
		if (hal_video_get_no_video_time() > 1000 * 30) {
			printf("no video frame time > %d ms", hal_video_get_no_video_time());
			//reopen video or system reset

			sys_reset();
		}
	}
}

void fast_inf_video_example(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(fast_inf_video_example_task, ((const char *)"fast_inf_task"), 4096, NULL, tskIDLE_PRIORITY + 6, NULL) != pdPASS) {
		printf("\r\n video_example_main: Create Task Error\n");
	}
}