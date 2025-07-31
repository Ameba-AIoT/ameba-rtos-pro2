/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "platform_opts.h"
#include "log_service.h"

#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"
#include "mmf2_mimo.h"

#include "avcodec.h"
#include "module_video.h"
#include "module_vipnn.h"
#include "module_audio.h"
#include "module_g711.h"
#include "module_mp4.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"

#include "model_yolo.h"
#include "nn_utils/class_name.h"

#include "example_kvs_webrtc_mmf.h"
#include "module_kvs_webrtc.h"
#include "sample_config_webrtc.h"

/*****************************************************************************
* ISP channel : 0
* Video type  : H264/HEVC
*****************************************************************************/
// for mp4
#define V1_CHANNEL 0
#define V1_RESOLUTION VIDEO_FHD
#define V1_FPS 30
#define V1_GOP 30
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR

// for webrtc
#define WEBRTC_CHANNEL 1
#define WEBRTC_RESOLUTION VIDEO_HD
#define WEBRTC_FPS 30
#define WEBRTC_GOP 30
#define WEBRTC_BPS 1*1024*1024
#define VIDEO_RCMODE 2 // 1: CBR, 2: VBR

#define USE_H265 0

#if USE_H265
#define VIDEO_TYPE VIDEO_HEVC
#define VIDEO_CODEC AV_CODEC_ID_H265
#else
#define VIDEO_TYPE VIDEO_H264
#define VIDEO_CODEC AV_CODEC_ID_H264
#endif

#if V1_RESOLUTION == VIDEO_VGA
#define V1_WIDTH	640
#define V1_HEIGHT	480
#elif V1_RESOLUTION == VIDEO_HD
#define V1_WIDTH	1280
#define V1_HEIGHT	720
#elif V1_RESOLUTION == VIDEO_FHD
#define V1_WIDTH	1920
#define V1_HEIGHT	1080
#endif

#if WEBRTC_RESOLUTION == VIDEO_VGA
#define WEBRTC_WIDTH	640
#define WEBRTC_HEIGHT	480
#elif WEBRTC_RESOLUTION == VIDEO_HD
#define WEBRTC_WIDTH	1280
#define WEBRTC_HEIGHT	720
#elif WEBRTC_RESOLUTION == VIDEO_FHD
#define WEBRTC_WIDTH	1920
#define WEBRTC_HEIGHT	1080
#endif

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = V1_RESOLUTION,
	.width = V1_WIDTH,
	.height = V1_HEIGHT,
	.bps = V1_BPS,
	.fps = V1_FPS,
	.gop = V1_GOP,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1
};

static video_params_t video_v2_params = {
	.stream_id 		= WEBRTC_CHANNEL,
	.type 			= VIDEO_TYPE,
	.resolution 	= WEBRTC_RESOLUTION,
	.width 			= WEBRTC_WIDTH,
	.height 		= WEBRTC_HEIGHT,
	.bps            = WEBRTC_BPS,
	.fps 			= WEBRTC_FPS,
	.gop 			= WEBRTC_GOP,
	.rc_mode        = VIDEO_RCMODE,
	.use_static_addr = 1,
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
	.mix_mode = 0,
	.enable_aec  = 1
};
#endif

static g711_params_t g711e_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_ENCODE
};

static g711_params_t g711d_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_DECODE
};

static mp4_params_t mp4_v1_params = {
	.fps            = V1_FPS,
	.gop            = V1_GOP,
	.width = V1_WIDTH,
	.height = V1_HEIGHT,
	.sample_rate = 8000,
	.channel = 1,

	.record_length = 30, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 3000,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
	.mp4_audio_format = AUDIO_ULAW,  // AUDIO_AAC
	.mp4_audio_duration = 20,  // audio duration 20ms for PCM
};


#define NN_CHANNEL 4
#define NN_RESOLUTION VIDEO_VGA //don't care for NN
#define NN_FPS 10
#define NN_GOP NN_FPS
#define NN_BPS 1024*1024 //don't care for NN
#define NN_TYPE VIDEO_RGB

#define NN_MODEL_OBJ   yolov4_tiny
#define NN_WIDTH	416
#define NN_HEIGHT	416
static float nn_confidence_thresh = 0.4;
static float nn_nms_thresh = 0.3;

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
	}
};

#include "wifi_conf.h"
#include "lwip_netconf.h"
#define wifi_wait_time 500 //Here we wait 5 second to wiat the fast connect 
static void wifi_common_init(void)
{
	uint32_t wifi_wait_count = 0;

	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(10);
		wifi_wait_count++;
		if (wifi_wait_count == wifi_wait_time) {
			printf("\r\nuse ATW0, ATW1, ATWC to make wifi connection\r\n");
			printf("wait for wifi connection...\r\n");
		}
	}
}

static mm_context_t *video_v1_ctx           = NULL;
static mm_context_t *video_v2_ctx           = NULL;
static mm_context_t *audio_ctx              = NULL;
static mm_context_t *g711e_ctx              = NULL;
static mm_context_t *g711d_ctx              = NULL;
static mm_context_t *kvs_webrtc_ctx         = NULL;
static mm_context_t *mp4_ctx				= NULL;

static mm_context_t *video_rgb_ctx			= NULL;
static mm_context_t *vipnn_ctx            	= NULL;

static mm_siso_t *siso_audio_a1             = NULL;
static mm_siso_t *siso_webrtc_a2            = NULL;
static mm_siso_t *siso_a2_audio             = NULL;
static mm_siso_t *siso_video_vipnn         	= NULL;
static mm_mimo_t *mimo_kvs_webrtc_v3_a1     = NULL;

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

	int im_h = V1_HEIGHT;
	int im_w = V1_WIDTH;

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

	printf("object num = %d\r\n", obj_num);
	if (obj_num > 0) {
		for (i = 0; i < obj_num; i++) {
			int obj_class = (int)res[i].result[0];
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
				printf("%d,c%d:%d %d %d %d\n\r", i, class_id, xmin, ymin, xmax, ymax);

				char text_str[20];
				snprintf(text_str, sizeof(text_str), "%s %d", coco_name_get_by_id(class_id), (int)(res[i].result[1] * 100));
			}
		}
	}
}

void example_kvs_webrtc_joint_test_mmf_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif

	if (!voe_boot_fsc_status()) {
		wifi_common_init();
	}

	int voe_heap_size = video_voe_presetting(1, V1_WIDTH, V1_HEIGHT, V1_BPS, 0,
						1, WEBRTC_WIDTH, WEBRTC_HEIGHT, WEBRTC_BPS, 0,
						0, 0, 0, 0, 0,
						1, NN_WIDTH, NN_HEIGHT);

	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	// ------ Channel 1--------------
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, V1_FPS * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
	} else {
		printf("video open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	// ------ Channel 2--------------
	video_v2_ctx = mm_module_open(&video_module);
	if (video_v2_ctx) {
		mm_module_ctrl(video_v2_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v2_params);
		mm_module_ctrl(video_v2_ctx, MM_CMD_SET_QUEUE_LEN, WEBRTC_FPS * 3);
		mm_module_ctrl(video_v2_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v2_ctx, CMD_VIDEO_APPLY, WEBRTC_CHANNEL);	// start webrtc channel
	} else {
		printf("video open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	//--------------MP4---------------
	mp4_ctx = mm_module_open(&mp4_module);
	if (mp4_ctx) {
		mm_module_ctrl(mp4_ctx, CMD_MP4_SET_PARAMS, (int)&mp4_v1_params);
		mm_module_ctrl(mp4_ctx, CMD_MP4_LOOP_MODE, 0);
		mm_module_ctrl(mp4_ctx, CMD_MP4_START, mp4_v1_params.record_file_num);
	} else {
		printf("MP4 open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	//--------------Audio --------------
	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
#if !USE_DEFAULT_AUDIO_SET
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
#endif
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		printf("AUDIO open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	//--------------G711 --------------
	g711e_ctx = mm_module_open(&g711_module);
	if (g711e_ctx) {
		mm_module_ctrl(g711e_ctx, CMD_G711_SET_PARAMS, (int)&g711e_params);
		mm_module_ctrl(g711e_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711e_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711e_ctx, CMD_G711_APPLY, 0);
	} else {
		printf("G711 open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	//--------------WebRTC --------------
	kvs_webrtc_ctx = mm_module_open(&kvs_webrtc_module);
	if (kvs_webrtc_ctx) {
		mm_module_ctrl(kvs_webrtc_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(kvs_webrtc_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(kvs_webrtc_ctx, CMD_KVS_WEBRTC_SET_APPLY, 0);
	} else {
		printf("KVS open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	// ------ Channel 3--------------
	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 2);
	} else {
		printf("video open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	//--------------VIPNN --------------
	vipnn_ctx = mm_module_open(&vipnn_module);
	if (vipnn_ctx) {
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_DISPPOST, (int)nn_set_object);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_CONFIDENCE_THRES, (int)&nn_confidence_thresh);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_NMS_THRES, (int)&nn_nms_thresh);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	siso_audio_a1 = siso_create();
	if (siso_audio_a1) {
		siso_ctrl(siso_audio_a1, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_a1, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711e_ctx, 0);
		siso_start(siso_audio_a1);
	} else {
		printf("siso_audio_a1 open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	mimo_kvs_webrtc_v3_a1 = mimo_create();
	if (mimo_kvs_webrtc_v3_a1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		mimo_ctrl(mimo_kvs_webrtc_v3_a1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		mimo_ctrl(mimo_kvs_webrtc_v3_a1, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);  // mp4
		mimo_ctrl(mimo_kvs_webrtc_v3_a1, MMIC_CMD_ADD_INPUT1, (uint32_t)video_v2_ctx, 0);  // webrtc
		mimo_ctrl(mimo_kvs_webrtc_v3_a1, MMIC_CMD_ADD_INPUT2, (uint32_t)g711e_ctx, 0);
		mimo_ctrl(mimo_kvs_webrtc_v3_a1, MMIC_CMD_ADD_OUTPUT0, (uint32_t)mp4_ctx, MMIC_DEP_INPUT0 | MMIC_DEP_INPUT2);
		mimo_ctrl(mimo_kvs_webrtc_v3_a1, MMIC_CMD_ADD_OUTPUT1, (uint32_t)kvs_webrtc_ctx, MMIC_DEP_INPUT1 | MMIC_DEP_INPUT2);
		mimo_start(mimo_kvs_webrtc_v3_a1);
	} else {
		printf("mimo_kvs_webrtc_v3_a1 open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}
	printf("mimo_kvs_webrtc_v3_a1 started\n\r");

#ifdef ENABLE_AUDIO_SENDRECV
	g711d_ctx = mm_module_open(&g711_module);
	if (g711d_ctx) {
		mm_module_ctrl(g711d_ctx, CMD_G711_SET_PARAMS, (int)&g711d_params);
		mm_module_ctrl(g711d_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(g711d_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(g711d_ctx, CMD_G711_APPLY, 0);
	} else {
		printf("G711 open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	siso_webrtc_a2 = siso_create();
	if (siso_webrtc_a2) {
		siso_ctrl(siso_webrtc_a2, MMIC_CMD_ADD_INPUT, (uint32_t)kvs_webrtc_ctx, 0);
		siso_ctrl(siso_webrtc_a2, MMIC_CMD_ADD_OUTPUT, (uint32_t)g711d_ctx, 0);
		siso_start(siso_webrtc_a2);
	} else {
		printf("siso_webrtc_a2 open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

	siso_a2_audio = siso_create();
	if (siso_a2_audio) {
		siso_ctrl(siso_a2_audio, MMIC_CMD_ADD_INPUT, (uint32_t)g711d_ctx, 0);
		siso_ctrl(siso_a2_audio, MMIC_CMD_ADD_OUTPUT, (uint32_t)audio_ctx, 0);
		siso_start(siso_a2_audio);
	} else {
		printf("siso_a2_audio open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}
#endif

	//--------------NN Link---------------------------
	siso_video_vipnn = siso_create();
	if (siso_video_vipnn) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_vipnn, MMIC_CMD_ADD_INPUT, (uint32_t)video_rgb_ctx, 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_ctrl(siso_video_vipnn, MMIC_CMD_ADD_OUTPUT, (uint32_t)vipnn_ctx, 0);
		siso_start(siso_video_vipnn);
	} else {
		printf("siso_video_vipnn open fail\n\r");
		goto example_kvs_webrtc_cleanup;
	}

example_kvs_webrtc_cleanup:

	vTaskDelete(NULL);
}

void example_kvs_webrtc_joint_test_mmf(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(example_kvs_webrtc_joint_test_mmf_thread, ((const char *)"example_kvs_webrtc_joint_test_mmf_thread"), 4096, NULL, tskIDLE_PRIORITY + 1,
					NULL) != pdPASS) {
		printf("\r\n example_kvs_webrtc_joint_test_mmf_thread: Create Task Error\n");
	}
}