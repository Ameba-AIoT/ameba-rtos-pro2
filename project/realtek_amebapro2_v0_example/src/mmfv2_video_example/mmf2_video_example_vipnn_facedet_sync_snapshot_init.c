/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "module_vipnn.h"
#include "module_rtsp2.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "avcodec.h"
#include "log_service.h"
#include "vfs.h"

#include "model_scrfd.h"

#include "hal_video.h"
#include "hal_isp.h"

/*****************************************************************************
* ISP channel : 0
* Video type  : JPEG + SNAPSHOT
*****************************************************************************/

#define V1_CHANNEL 0
#define V1_RESOLUTION VIDEO_FHD
#define V1_FPS 5
#define V1_BPS 1*1024*1024

#define VIDEO_TYPE VIDEO_JPEG

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

#define SYNC_FPS 5

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = V1_RESOLUTION,
	.width = V1_WIDTH,
	.height = V1_HEIGHT,
	.use_static_addr = 1,

	/* Enable sync mode. The video frame will not be encoded untill calling "osd update", which is included in canvas_update()
	 * In other words, OSD can draw the bounding box from NN to the video frame without latency
	 * However, the restriction is that the fps of this video channel should be same with NN channel */
	.out_mode = MODE_SYNC,
	.fps = SYNC_FPS,
};

/*****************************************************************************
* ISP channel : 4
* Video type  : RGB
*****************************************************************************/

#define NN_CHANNEL 4
#define NN_RESOLUTION VIDEO_VGA //don't care for NN
#define NN_FPS SYNC_FPS
#define NN_GOP NN_FPS //don't care for NN
#define NN_BPS 1024*1024 //don't care for NN
#define NN_TYPE VIDEO_RGB

// SCRFD
#define NN_MODEL_OBJ   scrfd_fwfs
#define NN_WIDTH    576
#define NN_HEIGHT   320

static video_params_t video_v4_params = {
	.stream_id = NN_CHANNEL,
	.type = NN_TYPE,
	.resolution = NN_RESOLUTION,
	.width = NN_WIDTH,
	.height = NN_HEIGHT,
	.bps = NN_BPS,
	.fps = NN_FPS,
	.gop = NN_GOP,
	.direct_output = 0,
	.use_static_addr = 1
};

static nn_data_param_t roi_nn = {
	.img = {
		.width = NN_WIDTH,
		.height = NN_HEIGHT,
		.roi = {
			.xmin = 0,
			.ymin = 0,
			.xmax = NN_WIDTH,
			.ymax = NN_HEIGHT,
		}
	},
	.codec_type = AV_CODEC_ID_RGB888
};

static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *video_rgb_ctx			= NULL;
static mm_context_t *vipnn_ctx              = NULL;

static mm_siso_t *siso_video_vipnn          = NULL;

//--------------------------------------------
// Draw Rect
//--------------------------------------------
#include "osd_render.h"
#define LIMIT(x, lower, upper) if(x<lower) x=lower; else if(x>upper) x=upper;

static void nn_set_object(void *p, void *img_param)
{
	int i = 0;
	vipnn_out_buf_t *out = (vipnn_out_buf_t *)p;
	facedetect_res_t *face_res = (facedetect_res_t *)&out->res[0];

	nn_data_param_t *im = (nn_data_param_t *)img_param;

	if (!p || !img_param)	{
		return;
	}

	int im_h = V1_HEIGHT;
	int im_w = V1_WIDTH;

	//crop
	float ratio_w = (float)im_w / (float)im->img.width;
	float ratio_h = (float)im_h / (float)im->img.height;
	float ratio = ratio_h < ratio_w ? ratio_h : ratio_w;
	int roi_w = (int)((im->img.roi.xmax - im->img.roi.xmin) * ratio);
	int roi_h = (int)((im->img.roi.ymax - im->img.roi.ymin) * ratio);
	int roi_x = (int)(im->img.roi.xmin * ratio + (im_w - roi_w) / 2);
	int roi_y = (int)(im->img.roi.ymin * ratio + (im_h - roi_h) / 2);

	printf("object num = %d\r\n", out->res_cnt);
	canvas_create_bitmap(V1_CHANNEL, 0, RTS_OSD2_BLK_FMT_1BPP);
	if (out->res_cnt > 0) {
		for (i = 0; i < out->res_cnt; i++) {
			int obj_class = (int)face_res[i].result[0];
			int class_id = obj_class;
			if (class_id != -1) {
				int xmin = (int)(face_res[i].result[2] * roi_w) + roi_x;
				int ymin = (int)(face_res[i].result[3] * roi_h) + roi_y;
				int xmax = (int)(face_res[i].result[4] * roi_w) + roi_x;
				int ymax = (int)(face_res[i].result[5] * roi_h) + roi_y;
				LIMIT(xmin, 0, im_w)
				LIMIT(xmax, 0, im_w)
				LIMIT(ymin, 0, im_h)
				LIMIT(ymax, 0, im_h)
				printf("%d,c%d:%d %d %d %d\n\r", i, class_id, xmin, ymin, xmax, ymax);
				canvas_set_rect(V1_CHANNEL, 0, xmin, ymin, xmax, ymax, 3, COLOR_WHITE);
				char text_str[20];
				snprintf(text_str, sizeof(text_str), "%s %d", "face", (int)(face_res[i].result[1] * 100));
				canvas_set_text(V1_CHANNEL, 0, xmin, ymin - 32, text_str, COLOR_CYAN);

				for (int j = 0; j < 5; j++) {
					int x = (int)(face_res[i].landmark.pos[j].x * roi_w) + roi_x;
					int y = (int)(face_res[i].landmark.pos[j].y * roi_h) + roi_y;
					canvas_set_point(V1_CHANNEL, 0, x, y, 8, COLOR_RED);
				}
			}
		}
	}
	canvas_update(V1_CHANNEL, 0, 1);
}

static _mutex snapshot_mutex;
static _sema snapshot_sema;

static uint32_t g_jpeg_addr;
static uint32_t g_jpeg_len;

static int v1_snapshot_cb(uint32_t jpeg_addr, uint32_t jpeg_len)
{
	printf(">>> snapshot size = %ld\n\r", jpeg_len);

	rtw_mutex_get(&snapshot_mutex);
	g_jpeg_addr = jpeg_addr;
	g_jpeg_len = jpeg_len;
	rtw_mutex_put(&snapshot_mutex);

	rtw_up_sema(&snapshot_sema);

	return 0;
}

static void jpeg_sd_save(void)
{
	printf("saving jpg data... \r\n");
	static int im_cnt = 0;
	char filename[32];
	snprintf(filename, sizeof(filename), "sd:/test_%04d.jpg", im_cnt);
	rtw_mutex_get(&snapshot_mutex);
	FILE *fp = fopen(filename, "w+");
	fwrite((void *)g_jpeg_addr, g_jpeg_len, 1, fp);
	fclose(fp);
	rtw_mutex_put(&snapshot_mutex);
	printf("save jpg %s done \r\n", filename);
	im_cnt++;
}

static void capture_nn_snapshot_and_save(void)
{
	if (video_v1_ctx && video_rgb_ctx) {
		//start video channel
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_APPLY, NN_CHANNEL);
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_YUV, 1);  //one shot mode
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);

		//wait jpeg buffer
		rtw_down_timeout_sema(&snapshot_sema, portMAX_DELAY);

		//save jpeg to sd card
		jpeg_sd_save();

		//stop video channel
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_STREAM_STOP, NN_CHANNEL);
	}
}

static void example_deinit(void);

void mmf2_video_example_vipnn_facedet_sync_snapshot_init(void)
{
	rtw_init_sema(&snapshot_sema, 0);
	rtw_mutex_init(&snapshot_mutex);

	vfs_init(NULL);
	if (vfs_user_register("sd", VFS_FATFS, VFS_INF_SD) != 0) {
		printf("fail to register SD vfs\r\n");
		return;
	}
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, V1_WIDTH, V1_HEIGHT, V1_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						1, NN_WIDTH, NN_HEIGHT);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 0, NULL, 0, NULL, 0, &video_v4_params);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SNAPSHOT_CB, (int)v1_snapshot_cb);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_vnn_facedetect_sync_fail;
	}

	video_rgb_ctx = mm_module_open(&video_module);
	if (video_rgb_ctx) {
		mm_module_ctrl(video_rgb_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v4_params);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_SET_QUEUE_LEN, 2);
		mm_module_ctrl(video_rgb_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_example_vnn_facedetect_sync_fail;
	}

	// VIPNN
	vipnn_ctx = mm_module_open(&vipnn_module);
	if (vipnn_ctx) {
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(facedetect_res_t));		// result size
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_DISPPOST, (int)nn_set_object);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_example_vnn_facedetect_sync_fail;
	}
	printf("VIPNN opened\n\r");

	//--------------Link---------------------------

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
		goto mmf2_example_vnn_facedetect_sync_fail;
	}
	printf("siso_video_vipnn started\n\r");

	//OSD init
	int ch_enable[3] = {1, 0, 0};
	int char_resize_w[3] = {16, 0, 0}, char_resize_h[3] = {32, 0, 0};
	int ch_width[3] = {V1_WIDTH, 0, 0}, ch_height[3] = {V1_HEIGHT, 0, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);

	//take snapshot with nn result
	for (int i = 0; i < 10; i++) {
		capture_nn_snapshot_and_save();
		vTaskDelay(5000);
	}

	//example deinit
	printf("example deinit\n\r");
	example_deinit();

	return;
mmf2_example_vnn_facedetect_sync_fail:

	return;
}

static void example_deinit(void)
{
	osd_render_task_stop();
	osd_render_dev_deinit_all();
	//Pause Linker
	siso_pause(siso_video_vipnn);

	//Delete linker
	siso_delete(siso_video_vipnn);

	//Close module
	video_v1_ctx = mm_module_close(video_v1_ctx);
	video_rgb_ctx = mm_module_close(video_rgb_ctx);
	vipnn_ctx = mm_module_close(vipnn_ctx);

	video_voe_release();
}
