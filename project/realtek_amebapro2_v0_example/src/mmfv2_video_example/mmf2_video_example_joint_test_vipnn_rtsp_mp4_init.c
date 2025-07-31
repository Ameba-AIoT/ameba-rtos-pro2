/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_simo.h"
#include "mmf2_mimo.h"
#include "module_video.h"
#include "module_vipnn.h"
#include "module_rtsp2.h"
#include "module_audio.h"
#include "module_aac.h"
#include "module_aad.h"
#include "module_rtp.h"
#include "module_eip.h"
#include "module_mp4.h"
#include "module_facerecog.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "log_service.h"
#include "avcodec.h"

#include "nn_utils/class_name.h"
#include "model_yolo.h"
#include "model_yamnet_s.h"
#include "model_yamnet.h"
#include "model_mobilefacenet.h"
#include "model_scrfd.h"

#include "hal_video.h"
#include "hal_isp.h"

/*****************************************************************************
* ISP channel : 0,1,4
* Video type  : H264/HEVC,H264/HEVC,RGB
*****************************************************************************/
//For mp4
#define V1_CHANNEL 0
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR
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
	.use_static_addr = 1,
	//.fcs = 1
};

static video_params_t video_v2_params = {
	.stream_id 		= RTSP_CHANNEL,
	.type 			= RTSP_TYPE,
	.bps            = RTSP_BPS,
	.rc_mode        = VIDEO_RCMODE,
	.use_static_addr = 1,
	//.fcs = 1
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

static mp4_params_t mp4_v1_params = {
	.sample_rate = 16000,
	.channel = 1,

	.record_length = 30, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 3000,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
};

// NN model config //
#ifndef ENABLE_NN_FACERECOG
#define ENABLE_NN_FACERECOG   	0   /* fix here: enable NN face detection/recognition */
#endif
#ifndef ENABLE_NN_YOLO
#define ENABLE_NN_YOLO       	1   /* fix here: enable NN yolo */
#endif
#ifndef ENABLE_NN_YAMNET
#define ENABLE_NN_YAMNET       	1   /* fix here: enable NN audio classification */
#endif
#ifndef ENABLE_MD
#define ENABLE_MD       		0   /* fix here: enable MD */
#endif

#if (ENABLE_MD && ENABLE_NN_YOLO)
#define ENABLE_MD_TRIGGER_YOLO	1
#else
#define ENABLE_MD_TRIGGER_YOLO	0
#endif

#define NN_MODEL_OBJ   	scrfd_fwfs
#define NN_MODEL2_OBJ   mbfacenet_fwfs
#define NN_MODEL3_OBJ   yolov4_tiny

#if ENABLE_NN_FACERECOG
#define NN_WIDTH	576
#define NN_HEIGHT	320
#else
#define NN_WIDTH	416
#define NN_HEIGHT	416
#endif

#define NN_CHANNEL 4
#define NN_FPS 10
#define NN_GOP NN_FPS
#define NN_BPS 1024*1024 //don't care for NN
#define NN_TYPE VIDEO_RGB

#if ENABLE_NN_YOLO==1
define_model(yolov4_tiny_320p)
#define USE_OBJDET_MODEL use_model(yolov4_tiny_320p)
#else
#define USE_OBJDET_MODEL
#endif

#if ENABLE_NN_FACERECOG==1
define_model(scrfd320p)
#define USE_FACEDET_MODEL use_model(scrfd320p)
define_model(mobilefacenet_i8)
#define USE_FACENET_MODEL use_model(mobilefacenet_i8)
#else
#define USE_FACEDET_MODEL
#define USE_FACENET_MODEL
#endif

#if ENABLE_NN_YAMNET==1
define_model(yamnet_s_hybrid)
#define USE_AUDCLS_MODEL use_model(yamnet_s_hybrid)
#else
#define USE_AUDCLS_MODEL
#endif


static float nn_confidence_thresh = 0.5;
static float nn_nms_thresh = 0.3;
static int desired_class_list[] = {0, 2, 5, 7};

static video_params_t video_v4_params = {
	.stream_id 		= NN_CHANNEL,
	.type 			= NN_TYPE,
	.bps 			= NN_BPS,
	.direct_output 	= 0,
	.use_static_addr = 1,
#if !ENABLE_NN_FACERECOG
	.use_roi = 1,
	.roi = {
		.xmin = 0,
		.ymin = 0,
	}
#endif
};

#define MD_COL 32
#define MD_ROW 32
static eip_param_t eip_param = {
	.eip_row = MD_ROW,
	.eip_col = MD_COL
};

static nn_data_param_t roi_nn = {
	.img = {
		.rgb = 0, // set to 1 if want RGB->BGR or BGR->RGB
		.roi = {
			.xmin = 0,
			.ymin = 0,
		}
	},
	.codec_type = AV_CODEC_ID_RGB888
};

static nn_data_param_t fr_param_nn = {
	.codec_type = AV_CODEC_ID_NN_RAW
};

static nn_data_param_t aud_info = {
	.aud = {
		.bit_pre_sample = 16,
		.channel = 1,
		.sample_rate = 16000
	},
	.codec_type = AV_CODEC_ID_PCM_RAW
};

static void atcmd_userctrl_init(void);
static void atcmd_frc_init(void *ctx);
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *video_v2_ctx			= NULL;
static mm_context_t *rtsp2_v2_ctx			= NULL;
static mm_context_t *audio_ctx				= NULL;
static mm_context_t *aac_ctx				= NULL;
static mm_context_t *rtp_ctx				= NULL;
static mm_context_t *aad_ctx				= NULL;
static mm_context_t *mp4_ctx				= NULL;
static mm_context_t *facedet_ctx            = NULL;
static mm_context_t *facenet_ctx            = NULL;
static mm_context_t *facerecog_ctx          = NULL;

static mm_siso_t *siso_audio_aac            = NULL;
static mm_simo_t *simo_audio_aac_audclas    = NULL;
static mm_mimo_t *mimo_2v_1a_rtsp_mp4		= NULL;
static mm_siso_t *siso_rtp_aad				= NULL;
static mm_siso_t *siso_aad_audio			= NULL;

static mm_context_t *video_rgb_ctx			= NULL;
static mm_context_t *objdet_ctx            	= NULL;
static mm_context_t *md_ctx            		= NULL;
static mm_context_t *audclas_ctx           	= NULL;

static mm_siso_t *siso_video_vipnn         	= NULL;
static mm_siso_t *siso_md_nn         		= NULL;
static mm_siso_t *siso_facedet_facenet      = NULL;
static mm_siso_t *siso_facenet_facerecog    = NULL;
static mm_simo_t *simo_video_yolo_facedet   = NULL;

static audio_params_t audio_params;
static TX_cfg_t audio_tx_cfg;
static RX_cfg_t audio_rx_cfg;
static void audio_params_customized_setting(void)
{
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_PARAMS, (int)&audio_params); // get the audio default setting parameters
	audio_params.sample_rate = ASR_16KHZ;  // NN audio classification require 16K
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_TXASP_PARAM, (int)&audio_tx_cfg);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_RXASP_PARAM, (int)&audio_rx_cfg);
	audio_rx_cfg.aec_cfg.AEC_EN = ENABLE_NN_YAMNET ? 0 : 1;    // enable AEC, NS, AGC for Ameba-->Peer
	audio_rx_cfg.ns_cfg.NS_EN = 0;      // enable AEC, NS, AGC for Ameba-->Peer
	audio_tx_cfg.agc_cfg.AGC_EN = 1;    // enable AGC/DRC for Peer-->Ameba
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_TXASP_PARAM, (int)&audio_tx_cfg);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_RXASP_PARAM, (int)&audio_rx_cfg);
}

//--------------------------------------------
// Draw Rect
//--------------------------------------------
#include "osd_render.h"
#define LIMIT(x, lower, upper) if(x<lower) x=lower; else if(x>upper) x=upper;

static TimerHandle_t osd_cleanup_timer = NULL;
static void face_cleanup_callback(TimerHandle_t xTimer)
{
	(void)xTimer;
	canvas_create_bitmap(RTSP_CHANNEL, 1, RTS_OSD2_BLK_FMT_1BPP);
	canvas_create_bitmap(RTSP_CHANNEL, 2, RTS_OSD2_BLK_FMT_1BPP);
	canvas_update(RTSP_CHANNEL, 1, 0);
	canvas_update(RTSP_CHANNEL, 2, 1);
}

static void face_draw_object(void *p, void *img_param)
{
	int i = 0;
	frc_draw_t *fdraw = (frc_draw_t *)p;

	if (!p)	{
		return;
	}

	int im_h = RTSP_HEIGHT;
	int im_w = RTSP_WIDTH;


	//printf("face num = %d\r\n", fdraw->obj_cnt);
	canvas_create_bitmap(RTSP_CHANNEL, 1, RTS_OSD2_BLK_FMT_1BPP);//draw red for unknown
	canvas_create_bitmap(RTSP_CHANNEL, 2, RTS_OSD2_BLK_FMT_1BPP);
	if (fdraw->obj_cnt > 0) {
		for (i = 0; i < fdraw->obj_cnt; i++) {
			frc_bbox_t *bbox = &fdraw->bbox[i];
			int x_offset = 0, y_offset = 0;
			float ratio;
			if ((float)im_w / (float)im_h > (float)fdraw->pic_width / (float)fdraw->pic_height) {
				ratio = (float)im_h / (float)fdraw->pic_height;
				x_offset = (im_w - (float)fdraw->pic_width * ratio) / 2;
			} else {
				ratio = (float)im_w / (float)fdraw->pic_width;
				y_offset = (im_h - (float)fdraw->pic_height * ratio) / 2;
			}

			int xmin = (int)(bbox->xmin * ratio * (float)fdraw->pic_width) + x_offset;
			int ymin = (int)(bbox->ymin * ratio * (float)fdraw->pic_height) + y_offset;
			int xmax = (int)(bbox->xmax * ratio * (float)fdraw->pic_width) + x_offset;
			int ymax = (int)(bbox->ymax * ratio * (float)fdraw->pic_height) + y_offset;
			LIMIT(xmin, 0, im_w)
			LIMIT(xmax, 0, im_w)
			LIMIT(ymin, 0, im_h)
			LIMIT(ymax, 0, im_h)
			//printf("%d,c%s:%d %d %d %d\n\r", i, fdraw->obj_name[i], xmin, ymin, xmax, ymax);

			if (!strcmp(fdraw->obj_name[i], "unknown")) {
				canvas_set_rect(RTSP_CHANNEL, 1, xmin, ymin, xmax, ymax, 3, COLOR_RED);
				canvas_set_text(RTSP_CHANNEL, 1, xmin, ymin - 40, fdraw->obj_name[i], COLOR_RED);
			} else {
				canvas_set_rect(RTSP_CHANNEL, 2, xmin, ymin, xmax, ymax, 3, COLOR_GREEN);
				canvas_set_text(RTSP_CHANNEL, 2, xmin, ymin - 40, fdraw->obj_name[i], COLOR_GREEN);
			}
		}
		if (osd_cleanup_timer) {
			xTimerReset(osd_cleanup_timer, 10);
		}
	}
	canvas_update(RTSP_CHANNEL, 1, 0);
	canvas_update(RTSP_CHANNEL, 2, 1);
}

static int check_in_list(int class_indx)
{
	for (int i = 0; i < (sizeof(desired_class_list) / sizeof(int)); i++) {
		if (class_indx == desired_class_list[i]) {
			return class_indx;
		}
	}
	return -1;
}

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

	int im_h = RTSP_HEIGHT;
	int im_w = RTSP_WIDTH;

#if !ENABLE_NN_FACERECOG
	float ratio_h = (float)im_h / (float)im->img.height;
	float ratio_w = (float)im_w / (float)im->img.width;
	int roi_h = (int)((im->img.roi.ymax - im->img.roi.ymin) * ratio_h);
	int roi_w = (int)((im->img.roi.xmax - im->img.roi.xmin) * ratio_w);
	int roi_x = (int)(im->img.roi.xmin * ratio_w);
	int roi_y = (int)(im->img.roi.ymin * ratio_h);
#else
	float ratio_w = (float)im_w / (float)im->img.width;
	int roi_h = (int)((im->img.roi.ymax - im->img.roi.ymin) * ratio_w);
	int roi_w = (int)((im->img.roi.xmax - im->img.roi.xmin) * ratio_w);
	int roi_x = (int)(im->img.roi.xmin * ratio_w);
	int roi_y = (int)(im->img.roi.ymin * ratio_w + (im_h - roi_h) / 2);
#endif

	// printf("object num = %d\r\n", obj_num);
	canvas_create_bitmap(RTSP_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
	for (i = 0; i < obj_num; i++) {
		int obj_class = (int)res[i].result[0];

		//printf("obj_class = %d\r\n",obj_class);

		int class_id = check_in_list(obj_class); //show class in desired_class_list
		//int class_id = obj_class; //coco label
		if (class_id != -1) {
			int xmin = (int)(res[i].result[2] * roi_w) + roi_x;
			int ymin = (int)(res[i].result[3] * roi_h) + roi_y;
			int xmax = (int)(res[i].result[4] * roi_w) + roi_x;
			int ymax = (int)(res[i].result[5] * roi_h) + roi_y;
			LIMIT(xmin, 0, im_w)
			LIMIT(xmax, 0, im_w)
			LIMIT(ymin, 0, im_h)
			LIMIT(ymax, 0, im_h)
			printf("%d,c%d:%d %d %d %d\n\r", i, class_id, xmin, ymin, xmax, ymax);
			canvas_set_rect(RTSP_CHANNEL, 0, xmin, ymin, xmax, ymax, 3, COLOR_WHITE);
			char text_str[20];
			snprintf(text_str, sizeof(text_str), "%s %d", coco_name_get_by_id(class_id), (int)(res[i].result[1] * 100));
			canvas_set_text(RTSP_CHANNEL, 0, xmin, ymin - 32, text_str, COLOR_CYAN);
		}
	}
	canvas_update(RTSP_CHANNEL, 0, 1);

}
#if ENABLE_MD_TRIGGER_YOLO
static int no_motion_count = 0;
static void md_process(void *md_result)
{
	return;
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
		canvas_create_bitmap(RTSP_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
		canvas_update(RTSP_CHANNEL, 0, 1);
	}

}
#endif

void mmf2_video_example_joint_test_vipnn_rtsp_mp4_init(void)
{
	USE_OBJDET_MODEL;
	USE_AUDCLS_MODEL;
	USE_FACEDET_MODEL;
	USE_FACENET_MODEL;

	atcmd_userctrl_init();

	/*sensor capacity check & video parameter setting*/
	video_v1_params.resolution = VIDEO_FHD;
	video_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	video_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
	video_v1_params.fps = sensor_params[USE_SENSOR].sensor_fps / 2;
	video_v1_params.gop = sensor_params[USE_SENSOR].sensor_fps / 2;
	video_v2_params.resolution = VIDEO_HD;
	video_v2_params.width = RTSP_WIDTH;
	video_v2_params.height = RTSP_HEIGHT;
	video_v2_params.fps = sensor_params[USE_SENSOR].sensor_fps;
	video_v2_params.gop = sensor_params[USE_SENSOR].sensor_fps;
	video_v4_params.resolution = VIDEO_VGA;
	video_v4_params.width = NN_WIDTH;
	video_v4_params.height = NN_HEIGHT;
	video_v4_params.fps = NN_FPS;
	video_v4_params.gop = NN_GOP;
#if !ENABLE_NN_FACERECOG
	video_v4_params.roi.xmax = sensor_params[USE_SENSOR].sensor_width;
	video_v4_params.roi.ymax = sensor_params[USE_SENSOR].sensor_height;
#endif
	/*rtsp parameter setting*/
	rtsp2_v2_params.u.v.fps = sensor_params[USE_SENSOR].sensor_fps;
	/*mp4 parameter setting*/
	mp4_v1_params.fps = sensor_params[USE_SENSOR].sensor_fps / 2;
	mp4_v1_params.gop = sensor_params[USE_SENSOR].sensor_fps / 2;
	mp4_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	mp4_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
	/*md parameter setting*/
	eip_param.image_width = NN_WIDTH;
	eip_param.image_height = NN_HEIGHT;
	/*nn parameter setting*/
	roi_nn.img.width = NN_WIDTH;
	roi_nn.img.height = NN_HEIGHT;
	roi_nn.img.roi.xmax = NN_WIDTH;
	roi_nn.img.roi.ymax = NN_HEIGHT;
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, video_v1_params.width, video_v1_params.height, V1_BPS, 0,
						1, video_v2_params.width, video_v2_params.height, RTSP_BPS, 0,
						0, 0, 0, 0, 0,
						1, NN_WIDTH, NN_HEIGHT);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 0, &video_v2_params, 0, NULL, 0, &video_v4_params);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	// ------ Channel 1--------------
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps * 8);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}

	// ------ Channel 2--------------
	video_v2_ctx = mm_module_open(&video_module);
	if (video_v2_ctx) {
		mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v2_params);
		mm_module_ctrl(video_v2_ctx, MM_CMD_SET_QUEUE_LEN, video_v2_params.fps * 8);
		mm_module_ctrl(video_v2_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}

	//--------------MP4---------------
	mp4_ctx = mm_module_open(&mp4_module);
	if (mp4_ctx) {
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_PARAMS, (int)&mp4_v1_params);
		mm_module_ctrl(mp4_ctx, CMD_MP4_LOOP_MODE, 0);
		mm_module_ctrl(mp4_ctx, CMD_MP4_START, mp4_v1_params.record_file_num);
	} else {
		printf("MP4 open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}


	//--------------Audio --------------
	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
		audio_params_customized_setting();
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		printf("audio open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}

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
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
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
		printf("RTSP2 open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}

	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
#if ENABLE_NN_YOLO
	// VIPNN - YOLO
	objdet_ctx = mm_module_open(&vipnn_module);
	if (objdet_ctx) {
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL3_OBJ);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_DISPPOST, (int)nn_set_object);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_CONFIDENCE_THRES, (int)&nn_confidence_thresh);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_NMS_THRES, (int)&nn_nms_thresh);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(objdetect_res_t));		// result size
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count
		//mm_module_ctrl(objdet_ctx, MM_CMD_SET_DATAGROUP, MM_GROUP_END);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN_OBJDET open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("VIPNN_OBJDET opened\n\r");
#endif

#if ENABLE_MD
	md_ctx  = mm_module_open(&eip_module);
	if (md_ctx) {
		mm_module_ctrl(md_ctx, CMD_EIP_SET_PARAMS, (int)&eip_param);
#if ENABLE_MD_TRIGGER_YOLO
		md_config_t md_config;
		mm_module_ctrl(md_ctx, CMD_EIP_GET_MD_CONFIG, (int)&md_config); //get default md config
		md_config.md_trigger_block_threshold = 3; //md triggered when at least 3 motion block triggered
		memset(md_config.md_mask, 1, sizeof(md_config.md_mask));
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_DISPPOST, (int)md_process);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_CONFIG, (int)&md_config);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_OUTPUT, 1);  //enable module output
		mm_module_ctrl(md_ctx, CMD_EIP_SET_MD_EN, 1);
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_START);

		mm_module_ctrl(md_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(md_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
#endif
	} else {
		printf("md_ctx open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
#endif

#if ENABLE_NN_YAMNET
	// VIPNN - YAMNET
	audclas_ctx = mm_module_open(&vipnn_module);
	if (audclas_ctx) {
		mm_module_ctrl(audclas_ctx, CMD_VIPNN_SET_MODEL, (int)&yamnet_s);
		mm_module_ctrl(audclas_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&aud_info);
		mm_module_ctrl(audclas_ctx, CMD_VIPNN_SET_DISPPOST, (int)0);
		mm_module_ctrl(audclas_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN_AUD open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("VIPNN_AUD opened\n\r");
#endif

#if ENABLE_NN_FACERECOG
	// VIPNN - FACE DETECTION MODEL
	facedet_ctx = mm_module_open(&vipnn_module);
	if (facedet_ctx) {
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_OUTPUT, 1);		// output
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(facedetect_res_t));		// result size
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max coun
		mm_module_ctrl(facedet_ctx, MM_CMD_SET_QUEUE_LEN, 1);
		mm_module_ctrl(facedet_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);

		mm_module_ctrl(facedet_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("VIPNN opened\n\r");

	// VIPNN - FACE RECOGNITION MODEL
	facenet_ctx = mm_module_open(&vipnn_module);
	if (facenet_ctx) {
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL2_OBJ);
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&fr_param_nn);
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_CASCADE, 2);		// this module is cascade mode

		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_OUTPUT, 1);		// output
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(face_feature_res_t));		// result size
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max coun
		mm_module_ctrl(facenet_ctx, MM_CMD_SET_QUEUE_LEN, 1);
		mm_module_ctrl(facenet_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);

		mm_module_ctrl(facenet_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN2 open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("VIPNN2 opened\n\r");

	// FACERECOG
	facerecog_ctx = mm_module_open(&facerecog_module);
	if (facerecog_ctx) {
		mm_module_ctrl(facerecog_ctx, CMD_FRC_SET_THRES100, 99);  // 99/100 = 0.99 --> set a value to get lowest FP rate
		mm_module_ctrl(facerecog_ctx, CMD_FRC_SET_OSD_DRAW, (int)face_draw_object);  //face_draw_object
	} else {
		printf("FACERECOG open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("FACERECOG opened\n\r");

	atcmd_frc_init(facerecog_ctx);
#endif

	//--------------Link---------------------------
#if ENABLE_NN_YAMNET
	simo_audio_aac_audclas = simo_create();
	if (simo_audio_aac_audclas) {
		simo_ctrl(simo_audio_aac_audclas, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		simo_ctrl(simo_audio_aac_audclas, MMIC_CMD_ADD_OUTPUT0, (uint32_t)aac_ctx, 0);
		simo_ctrl(simo_audio_aac_audclas, MMIC_CMD_ADD_OUTPUT1, (uint32_t)audclas_ctx, 0);
		simo_ctrl(simo_audio_aac_audclas, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		simo_start(simo_audio_aac_audclas);
	} else {
		printf("simo_audio_aac_audclas open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("simo_audio_aac_audclas started\n\r");
#else
	siso_audio_aac = siso_create();
	if (siso_audio_aac) {
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_OUTPUT, (uint32_t)aac_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(siso_audio_aac);
	} else {
		printf("siso_audio_aac open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("siso_audio_aac started\n\r");
#endif

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
		printf("mimo open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("mimo started\n\r");
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_APPLY, RTSP_CHANNEL);// start channel 1

#if (ENABLE_NN_FACERECOG && (ENABLE_NN_YOLO || ENABLE_MD))
	siso_facenet_facerecog = siso_create();
	if (siso_facenet_facerecog) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_facenet_facerecog, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_facenet_facerecog, MMIC_CMD_ADD_INPUT, (uint32_t)facenet_ctx, 0);
		siso_ctrl(siso_facenet_facerecog, MMIC_CMD_ADD_OUTPUT, (uint32_t)facerecog_ctx, 0);
		siso_ctrl(siso_facenet_facerecog, MMIC_CMD_SET_TASKNANE, (uint32_t)"ss_fn_fr", 0);
		siso_start(siso_facenet_facerecog);
	} else {
		printf("siso_facenet_facerecog open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("siso_facenet_facerecog started\n\r");

	siso_facedet_facenet = siso_create();
	if (siso_facedet_facenet) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_facedet_facenet, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_facedet_facenet, MMIC_CMD_ADD_INPUT, (uint32_t)facedet_ctx, 0);
		siso_ctrl(siso_facedet_facenet, MMIC_CMD_ADD_OUTPUT, (uint32_t)facenet_ctx, 0);
		siso_ctrl(siso_facedet_facenet, MMIC_CMD_SET_TASKNANE, (uint32_t)"ss_fd_fn", 0);
		siso_start(siso_facedet_facenet);
	} else {
		printf("siso_facedet_facenet open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("siso_facedet_facenet started\n\r");

	simo_video_yolo_facedet = simo_create();
	if (simo_video_yolo_facedet) {

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_ADD_OUTPUT0, (uint32_t)facedet_ctx, 0);
#if ENABLE_MD
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_ADD_OUTPUT1, (uint32_t)md_ctx, 0);
#else
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_ADD_OUTPUT1, (uint32_t)objdet_ctx, 0);
#endif
		simo_start(simo_video_yolo_facedet);
	} else {
		printf("simo_video_yolo_facedet open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("simo_video_yolo_facedet started\n\r");

#elif (ENABLE_NN_YOLO || ENABLE_MD)

	siso_video_vipnn = siso_create();
	if (siso_video_vipnn) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_vipnn, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
#if ENABLE_MD
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_TASKNANE, (uint32_t)"ss_v4_md", 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_ADD_OUTPUT, (uint32_t)md_ctx, 0);
#else
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_TASKNANE, (uint32_t)"ss_v4_yolo", 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_ADD_OUTPUT, (uint32_t)objdet_ctx, 0);
#endif
		siso_start(siso_video_vipnn);
	} else {
		printf("siso_video_vipnn open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("siso_video_vipnn started\n\r");
#endif //(ENABLE_NN_FACERECOG && (ENABLE_NN_YOLO || ENABLE_MD))

	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);

#if ENABLE_MD_TRIGGER_YOLO
	siso_md_nn = siso_create();
	if (siso_md_nn) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_md_nn, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_md_nn, MMIC_CMD_SET_TASKNANE, (uint32_t)"ss_md_yolo", 0);
		siso_ctrl(siso_md_nn, MMIC_CMD_ADD_INPUT, (uint32_t)md_ctx, 0);
		siso_ctrl(siso_md_nn, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_md_nn, MMIC_CMD_ADD_OUTPUT, (uint32_t)objdet_ctx, 0);
		siso_start(siso_md_nn);
	} else {
		printf("siso_md_nn open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}
	printf("siso_md_nn started\n\r");
#endif
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
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}

	aad_ctx = mm_module_open(&aad_module);
	if (aad_ctx) {
		mm_module_ctrl(aad_ctx, CMD_AAD_SET_PARAMS, (int)&aad_rtp_params);
		mm_module_ctrl(aad_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(aad_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(aad_ctx, CMD_AAD_APPLY, 0);
	} else {
		printf("AAD open fail\n\r");
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
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
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
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
		goto mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail;
	}

	int ch_enable[3] = {0, 1, 0};
	int char_resize_w[3] = {0, 16, 0}, char_resize_h[3] = {0, 32, 0};
	int ch_width[3] = {0, RTSP_WIDTH, 0}, ch_height[3] = {0, RTSP_HEIGHT, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);
#if ENABLE_NN_FACERECOG
	osd_cleanup_timer = xTimerCreate("OSD clean timer", 1000 / portTICK_PERIOD_MS, pdTRUE, NULL, face_cleanup_callback);
#endif

	return;
mmf2_video_example_joint_test_vipnn_rtsp_mp4_fail:

	return;
}

static const char *example = "mmf2_video_example_joint_test_vipnn_rtsp_mp4";
static void example_deinit(void)
{
	if (md_ctx) {
		mm_module_ctrl(md_ctx, CMD_EIP_SET_STATUS, EIP_STATUS_STOP);
	}

	osd_render_task_stop();
	osd_render_dev_deinit_all();

	//Pause Linker
	siso_pause(siso_rtp_aad);
	siso_pause(siso_aad_audio);
	mimo_pause(mimo_2v_1a_rtsp_mp4, MM_OUTPUT0 | MM_OUTPUT1);
#if !ENABLE_NN_YAMNET
	siso_pause(siso_audio_aac);
#else
	simo_pause(simo_audio_aac_audclas, MM_OUTPUT0 | MM_OUTPUT1);
#endif
#if (ENABLE_NN_FACERECOG && (ENABLE_NN_YOLO || ENABLE_MD))
	simo_pause(simo_video_yolo_facedet, MM_OUTPUT0 | MM_OUTPUT1);
	siso_pause(siso_facedet_facenet);
	siso_pause(siso_facenet_facerecog);
#elif (ENABLE_NN_YOLO || ENABLE_MD)
	siso_pause(siso_video_vipnn);
#endif
#if ENABLE_MD_TRIGGER_YOLO
	siso_pause(siso_md_nn);
#endif


	//Stop module
	mm_module_ctrl(rtp_ctx, CMD_RTP_STREAMING, 0);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_TRX, 0);
	mm_module_ctrl(mp4_ctx, CMD_MP4_STOP, 0);
	mm_module_ctrl(rtsp2_v2_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
	mm_module_ctrl(video_v2_ctx, CMD_VIDEO_STREAM_STOP, RTSP_CHANNEL);
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_STREAM_STOP, NN_CHANNEL);


	//Delete linker
	siso_delete(siso_rtp_aad);
	siso_delete(siso_aad_audio);
	mimo_delete(mimo_2v_1a_rtsp_mp4);
#if !ENABLE_NN_YAMNET
	siso_delete(siso_audio_aac);
#else
	simo_delete(simo_audio_aac_audclas);
#endif
#if (ENABLE_NN_FACERECOG && (ENABLE_NN_YOLO || ENABLE_MD))
	simo_delete(simo_video_yolo_facedet);
	siso_delete(siso_facedet_facenet);
	siso_delete(siso_facenet_facerecog);
#elif (ENABLE_NN_YOLO || ENABLE_MD)
	siso_delete(siso_video_vipnn);
#endif

#if ENABLE_MD_TRIGGER_YOLO
	siso_delete(siso_md_nn);
#endif
	//Close module
	mm_module_close(aad_ctx);
	mm_module_close(rtp_ctx);
	mm_module_close(aac_ctx);
	mm_module_close(audio_ctx);
	mm_module_close(mp4_ctx);
#if ENABLE_NN_YOLO
	mm_module_close(objdet_ctx);
#endif
#if ENABLE_MD
	mm_module_close(md_ctx);
#endif
#if ENABLE_NN_YAMNET
	mm_module_close(audclas_ctx);
#endif
	mm_module_close(rtsp2_v2_ctx);
	mm_module_close(video_v1_ctx);
	mm_module_close(video_v2_ctx);
	mm_module_close(video_rgb_ctx);
#if ENABLE_NN_FACERECOG
	mm_module_close(facedet_ctx);
	mm_module_close(facenet_ctx);
	mm_module_close(facerecog_ctx);
#endif

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

//-------- Face recognition command --------------------------------------------------------

static void *g_frc_ctx = NULL;
static void fFREG(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};
	argc = parse_param(arg, argv);

	if (!g_frc_ctx)	{
		return;
	}
	printf("enter register mode\n\r");
	mm_module_ctrl(g_frc_ctx, CMD_FRC_REGISTER_MODE, (int)argv[1]);
}

static void fFRRM(void *arg)
{
	if (!g_frc_ctx)	{
		return;
	}
	printf("enter recognition mode\n\r");
	mm_module_ctrl(g_frc_ctx, CMD_FRC_RECOGNITION_MODE, 0);
}


static void fFRFL(void *arg)
{
	if (!g_frc_ctx)	{
		return;
	}
	printf("load feature\n\r");
	mm_module_ctrl(g_frc_ctx, CMD_FRC_LOAD_FEATURES, 0);
}


static void fFRFS(void *arg)
{
	if (!g_frc_ctx)	{
		return;
	}
	printf("save feature\n\r");
	mm_module_ctrl(g_frc_ctx, CMD_FRC_SAVE_FEATURES, 0);
}


static void fFRFR(void *arg)
{
	if (!g_frc_ctx)	{
		return;
	}
	printf("reset features\n\r");
	mm_module_ctrl(g_frc_ctx, CMD_FRC_RESET_FEATURES, 0);
}


static log_item_t nn_frc_items[] = {
	{"FREG", fFREG,},
	{"FRRM", fFRRM,},
	{"FRFL", fFRFL,},
	{"FRFS", fFRFS,},
	{"FRFR", fFRFR,}
};

static void atcmd_frc_init(void *ctx)
{
	g_frc_ctx = ctx;
	log_service_add_table(nn_frc_items, sizeof(nn_frc_items) / sizeof(nn_frc_items[0]));
}