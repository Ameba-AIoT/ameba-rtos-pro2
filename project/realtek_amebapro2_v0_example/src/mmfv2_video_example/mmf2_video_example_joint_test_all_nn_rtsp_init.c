/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_simo.h"
#include "module_video.h"
#include "module_vipnn.h"
#include "module_rtsp2.h"
#include "module_facerecog.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "avcodec.h"

#include "nn_utils/class_name.h"
#include "model_yolo.h"
#include "model_mobilefacenet.h"
#include "model_scrfd.h"

#include "module_audio.h"
//#include "model_yamnet.h"
#include "model_yamnet_s.h"

#include "hal_video.h"
#include "hal_isp.h"

#define V1_ENA 1
#define V4_ENA 1
#define AUD_ENA 1   //enable audio process in joint test
#if AUD_ENA
#define ASP_ENA 0   //enable audio signal process in joint test
#endif

/*****************************************************************************
* ISP channel : 4
* Video type  : RGB
*****************************************************************************/
#define RTSP_CHANNEL 0
#define RTSP_RESOLUTION VIDEO_FHD
#define RTSP_FPS 30
#define RTSP_GOP 30
#define RTSP_BPS 1*1024*1024
#define VIDEO_RCMODE 2 // 1: CBR, 2: VBR

#define USE_H265 0

#if USE_H265
#include "sample_h265.h"
#define RTSP_TYPE VIDEO_HEVC
#define RTSP_CODEC AV_CODEC_ID_H265
#else
#include "sample_h264.h"
#define RTSP_TYPE VIDEO_H264
#define RTSP_CODEC AV_CODEC_ID_H264
#endif

#if RTSP_RESOLUTION == VIDEO_VGA
#define RTSP_WIDTH	640
#define RTSP_HEIGHT	480
#elif RTSP_RESOLUTION == VIDEO_HD
#define RTSP_WIDTH	1280
#define RTSP_HEIGHT	720
#elif RTSP_RESOLUTION == VIDEO_FHD
#define RTSP_WIDTH	1920
#define RTSP_HEIGHT	1080
#endif

static video_params_t video_v1_params = {
	.stream_id 		= RTSP_CHANNEL,
	.type 			= RTSP_TYPE,
	.resolution 	= RTSP_RESOLUTION,
	.width 			= RTSP_WIDTH,
	.height 		= RTSP_HEIGHT,
	.bps            = RTSP_BPS,
	.fps 			= RTSP_FPS,
	.gop 			= RTSP_GOP,
	.rc_mode        = VIDEO_RCMODE,
	.use_static_addr = 1,
	//.fcs = 1
};

static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = RTSP_CODEC,
			.fps      = RTSP_FPS,
			.bps      = RTSP_BPS
		}
	}
};

static audio_params_t audio_params;

static nn_data_param_t aud_info = {
	.aud = {
		.bit_pre_sample = 16,
		.channel = 1,
		.sample_rate = 16000
	},
	.codec_type = AV_CODEC_ID_PCM_RAW
};

// NN model config //
#define NN_CHANNEL 4
#define NN_RESOLUTION VIDEO_VGA //don't care for NN
#define NN_FPS 10
#define NN_GOP NN_FPS
#define NN_BPS 1024*1024 //don't care for NN
#define NN_TYPE VIDEO_RGB

#define NN_MODEL_OBJ   	scrfd_fwfs
#define NN_MODEL2_OBJ   mbfacenet_fwfs
#define NN_WIDTH	576     //640
#define NN_HEIGHT	320     //480

#define NN_MODEL3_OBJ   yolov4_tiny

define_model(yolov4_tiny_320p)
define_model(scrfd320p)
define_model(mobilefacenet_i8)
define_model(yamnet_s_hybrid)
#define USE_OBJDET_MODEL use_model(yolov4_tiny_320p)
#define USE_FACEDET_MODEL use_model(scrfd320p)
#define USE_FACENET_MODEL use_model(mobilefacenet_i8)
#define USE_AUDCLS_MODEL use_model(yamnet_s_hybrid)

static float nn_confidence_thresh = 0.5;
static float nn_nms_thresh = 0.3;
static int desired_class_list[] = {0, 2, 5, 7};


static video_params_t video_v4_params = {
	.stream_id 		= NN_CHANNEL,
	.type 			= NN_TYPE,
	.resolution	 	= NN_RESOLUTION,
	.width 			= NN_WIDTH,
	.height 		= NN_HEIGHT,
	.bps 			= NN_BPS,
	.fps 			= NN_FPS,
	.gop 			= NN_GOP,
	.direct_output 	= 0,
	.use_static_addr = 1
};

static nn_data_param_t roi_nn = {
	.img = {
		.width = NN_WIDTH,
		.height = NN_HEIGHT,
		.rgb = 0, // set to 1 if want RGB->BGR or BGR->RGB
		.roi = {
			.xmin = 0,
			.ymin = 0,
			.xmax = NN_WIDTH,
			.ymax = NN_HEIGHT,
		}
	},
	.codec_type = AV_CODEC_ID_RGB888
};

static nn_data_param_t fr_param_nn = {
	.codec_type = AV_CODEC_ID_NN_RAW
};

static mm_context_t *video_v1_ctx           = NULL;
static mm_context_t *rtsp2_v1_ctx           = NULL;
static mm_context_t *audio_ctx              = NULL;
static mm_context_t *video_rgb_ctx          = NULL;
static mm_context_t *facedet_ctx            = NULL;
static mm_context_t *facenet_ctx            = NULL;
static mm_context_t *objdet_ctx             = NULL;
static mm_context_t *audclas_ctx            = NULL;
static mm_context_t *facerecog_ctx          = NULL;

static mm_siso_t *siso_video_rtsp_v1        = NULL;
static mm_siso_t *siso_audio_audcls         = NULL;
static mm_siso_t *siso_facedet_facenet      = NULL;
static mm_siso_t *siso_facenet_facerecog    = NULL;
static mm_simo_t *simo_video_yolo_facedet   = NULL;

static void atcmd_frc_init(void *ctx);

#if AUD_ENA
#if ASP_ENA
static RX_cfg_t rx_asp_params;
static TX_cfg_t tx_asp_params;
#endif

static void audio_params_customized_setting(void)
{
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_PARAMS, (int)&audio_params); // get the default audio codec setting parameters
	audio_params.sample_rate = ASR_16KHZ;  // NN audio classification require 16K
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
#if ASP_ENA
	// get the default audio signal setting parameters
	// TX => Peer-->device signal, RX => Ameba-->Peer
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_TXASP_PARAM, (int)&audio_tx_cfg);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_GET_RXASP_PARAM, (int)&audio_rx_cfg);
	// set up the enable flag and the related ASP will be initialed when audio module apply
	audio_rx_cfg.aec_cfg.AEC_EN = 1;
	audio_tx_cfg.agc_cfg.AGC_EN = 1;
	audio_rx_cfg.ns_cfg.NS_EN = 1;
	audio_tx_cfg.agc_cfg.AGC_EN = 1;
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_TXASP_PARAM, (int)&audio_tx_cfg);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_RXASP_PARAM, (int)&audio_rx_cfg);
	// If setting ASP run, the ASP will be adopted by the audio module, except that the ASP is not initialed

#endif
}
#endif

//--------------------------------------------
// Draw Rect
//--------------------------------------------
#include "osd_render.h"
static TimerHandle_t osd_cleanup_timer = NULL;

#define LIMIT(x, lower, upper) if(x<lower) x=lower; else if(x>upper) x=upper;
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

static void objdet_draw_object(void *p, void *img_param)
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

	//crop
	float ratio_w = (float)im_w / (float)im->img.width;
	float ratio_h = (float)im_h / (float)im->img.height;
	float ratio = ratio_h < ratio_w ? ratio_h : ratio_w;
	int roi_w = (int)((im->img.roi.xmax - im->img.roi.xmin) * ratio);
	int roi_h = (int)((im->img.roi.ymax - im->img.roi.ymin) * ratio);
	int roi_x = (int)(im->img.roi.xmin * ratio + (im_w - roi_w) / 2);
	int roi_y = (int)(im->img.roi.ymin * ratio + (im_h - roi_h) / 2);

	//printf("object num = %d\r\n", obj_num);
	canvas_create_bitmap(RTSP_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
	if (obj_num > 0) {
		for (i = 0; i < obj_num; i++) {
			int obj_class = (int)res[i].result[0];

			int class_id = check_in_list(obj_class); //show class in desired_class_list

			if (class_id != -1) {
				int xmin = (int)(res[i].result[2] * roi_w) + roi_x;
				int ymin = (int)(res[i].result[3] * roi_h) + roi_y;
				int xmax = (int)(res[i].result[4] * roi_w) + roi_x;
				int ymax = (int)(res[i].result[5] * roi_h) + roi_y;
				LIMIT(xmin, 0, im_w)
				LIMIT(xmax, 0, im_w)
				LIMIT(ymin, 0, im_h)
				LIMIT(ymax, 0, im_h)

				char text_str[20];
				snprintf(text_str, sizeof(text_str), "%s", coco_name_get_by_id(class_id));
				canvas_set_rect(RTSP_CHANNEL, 0, xmin, ymin, xmax, ymax, 3, COLOR_CYAN);
				canvas_set_text(RTSP_CHANNEL, 0, xmin, ymin - 40, text_str, COLOR_CYAN);
			}
		}

		int human_cnt = 0;
		for (i = 0; i < obj_num; i++) {
			int obj_class = (int)res[i].result[0];

			int class_id = check_in_list(obj_class); //show class in desired_class_list
			if (class_id == 0)	{
				human_cnt++;
			}
		}

		if (human_cnt > 0) {  // if human detected
			simo_resume(simo_video_yolo_facedet);
		} else {
			simo_pause(simo_video_yolo_facedet, MM_OUTPUT0);
		}
	}
	canvas_update(RTSP_CHANNEL, 0, 1);
}


void mmf2_video_example_joint_test_all_nn_rtsp_init(void)
{
	USE_OBJDET_MODEL;
	USE_AUDCLS_MODEL;
	USE_FACEDET_MODEL;
	USE_FACENET_MODEL;
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(V1_ENA, RTSP_WIDTH, RTSP_HEIGHT, RTSP_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						V4_ENA, NN_WIDTH, NN_HEIGHT);
#else
	int voe_heap_size = video_voe_presetting_by_params((V1_ENA ? &video_v1_params : NULL), 0, NULL, 0, NULL, 0, (V4_ENA ? &video_v4_params : NULL));
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

#if V1_ENA
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, RTSP_FPS);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_fail;
	}

	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		printf("RTSP2 open fail\n\r");
		goto mmf2_example_fail;
	}
#endif

#if V4_ENA
	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_fail;
	}

	// VIPNN
	facedet_ctx = mm_module_open(&vipnn_module);
	if (facedet_ctx) {
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);

		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_OUTPUT, 1);		// output
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(facedetect_res_t));		// result size
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count

		mm_module_ctrl(facedet_ctx, MM_CMD_SET_DATAGROUP, MM_GROUP_START);
		mm_module_ctrl(facedet_ctx, MM_CMD_SET_QUEUE_LEN, 1);
		mm_module_ctrl(facedet_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);

		mm_module_ctrl(facedet_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_example_fail;
	}
	printf("VIPNN opened\n\r");

	// VIPNN
	facenet_ctx = mm_module_open(&vipnn_module);
	if (facenet_ctx) {
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL2_OBJ);
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&fr_param_nn);
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_CASCADE, 2);		// this module is cascade mode

		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_OUTPUT, 1);		// output
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(face_feature_res_t));		// result size
		mm_module_ctrl(facenet_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count
		mm_module_ctrl(facenet_ctx, MM_CMD_SET_DATAGROUP, MM_GROUP_END);
		mm_module_ctrl(facenet_ctx, MM_CMD_SET_QUEUE_LEN, 1);
		mm_module_ctrl(facenet_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);

		mm_module_ctrl(facenet_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN2 open fail\n\r");
		goto mmf2_example_fail;
	}
	printf("VIPNN2 opened\n\r");

	// FACERECOG
	facerecog_ctx = mm_module_open(&facerecog_module);
	if (facerecog_ctx) {
		mm_module_ctrl(facerecog_ctx, CMD_FRC_SET_THRES100, 99);  // 99/100 = 0.99 --> set a value to get lowest FP rate
		mm_module_ctrl(facerecog_ctx, CMD_FRC_SET_OSD_DRAW, (int)face_draw_object);
	} else {
		printf("FACERECOG open fail\n\r");
		goto mmf2_example_fail;
	}
	printf("FACERECOG opened\n\r");

	atcmd_frc_init(facerecog_ctx);


	// OBJDETECT
	objdet_ctx = mm_module_open(&vipnn_module);
	if (objdet_ctx) {
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL3_OBJ);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_DISPPOST, (int)objdet_draw_object);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_CONFIDENCE_THRES, (int)&nn_confidence_thresh);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_SET_NMS_THRES, (int)&nn_nms_thresh);
		mm_module_ctrl(objdet_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_example_fail;
	}
	printf("VIPNN opened\n\r");
#endif

#if AUD_ENA
	//--------------Audio --------------
	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
		audio_params_customized_setting();
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		printf("audio open fail\n\r");
		goto mmf2_example_fail;
	}

	// YAMNET
	audclas_ctx = mm_module_open(&vipnn_module);
	if (audclas_ctx) {
		//mm_module_ctrl(audclas_ctx, CMD_VIPNN_SET_MODEL, (int)&yamnet);
		mm_module_ctrl(audclas_ctx, CMD_VIPNN_SET_MODEL, (int)&yamnet_s);
		mm_module_ctrl(audclas_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&aud_info);
		mm_module_ctrl(audclas_ctx, CMD_VIPNN_SET_DISPPOST, (int)0);
		mm_module_ctrl(audclas_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_example_fail;
	}
	printf("VIPNN opened\n\r");
#endif

	//--------------Link---------------------------
#if V1_ENA
	siso_video_rtsp_v1 = siso_create();
	if (siso_video_rtsp_v1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v1_ctx, 0);
		siso_start(siso_video_rtsp_v1);
	} else {
		printf("siso_video_rtsp_v1 open fail\n\r");
		goto mmf2_example_fail;
	}
	printf("siso_video_rtsp_v1 started\n\r");
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, RTSP_CHANNEL);	// start channel 0
#endif
#if V4_ENA
	siso_facenet_facerecog = siso_create();
	if (siso_facenet_facerecog) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_facenet_facerecog, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_facenet_facerecog, MMIC_CMD_ADD_INPUT, (uint32_t)facenet_ctx, 0);
		siso_ctrl(siso_facenet_facerecog, MMIC_CMD_ADD_OUTPUT, (uint32_t)facerecog_ctx, 0);
		siso_start(siso_facenet_facerecog);
	} else {
		printf("siso_facenet_facerecog open fail\n\r");
		goto mmf2_example_fail;
	}
	printf("siso_facenet_facerecog started\n\r");

	siso_facedet_facenet = siso_create();
	if (siso_facedet_facenet) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_facedet_facenet, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_facedet_facenet, MMIC_CMD_ADD_INPUT, (uint32_t)facedet_ctx, 0);
		siso_ctrl(siso_facedet_facenet, MMIC_CMD_ADD_OUTPUT, (uint32_t)facenet_ctx, 0);
		siso_start(siso_facedet_facenet);
	} else {
		printf("siso_facedet_facenet open fail\n\r");
		goto mmf2_example_fail;
	}
	printf("siso_facedet_facenet started\n\r");

	simo_video_yolo_facedet = simo_create();
	if (simo_video_yolo_facedet) {

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_SET_TASKPRIORITY, 3, 0);

		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_ADD_OUTPUT0, (uint32_t)facedet_ctx, 0);
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_ADD_OUTPUT1, (uint32_t)objdet_ctx, 0);
		simo_ctrl(simo_video_yolo_facedet, MMIC_CMD_SET_OUT_STAT0, MMIC_STAT_PAUSE, 0);
		simo_start(simo_video_yolo_facedet);
		//simo_pause(simo_video_yolo_facedet, MM_OUTPUT0);
	} else {
		printf("simo_video_yolo_facedet open fail\n\r");
		goto mmf2_example_fail;
	}
	printf("simo_video_yolo_facedet started\n\r");

	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);	// start channel 4
	mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);
#endif

#if V1_ENA && V4_ENA
	int ch_enable[3] = {1, 0, 0};
	int char_resize_w[3] = {24, 0, 0}, char_resize_h[3] = {40, 0, 0};
	int ch_width[3] = {RTSP_WIDTH, 0, 0}, ch_height[3] = {RTSP_HEIGHT, 0, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);
	osd_cleanup_timer = xTimerCreate("OSD clean timer", 1000 / portTICK_PERIOD_MS, pdTRUE, NULL, face_cleanup_callback);
#endif

#if AUD_ENA
	siso_audio_audcls = siso_create();
	if (siso_audio_audcls) {
		siso_ctrl(siso_audio_audcls, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_audcls, MMIC_CMD_ADD_OUTPUT, (uint32_t)audclas_ctx, 0);
		siso_start(siso_audio_audcls);
	} else {
		printf("siso_audio_audcls open fail\n\r");
		goto mmf2_example_fail;
	}
	printf("siso_audio_audcls started\n\r");
#endif

	return;
mmf2_example_fail:
	return;
}

//-----------------------------------------------------------------------------------------------

#include "log_service.h"
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