/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "module_rtsp2.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "log_service.h"
#include "vfs.h"

/*****************************************************************************
* ISP channel : 2
* Video type  : JPEG
*****************************************************************************/
#define V1_CHANNEL 0
#define V1_FPS 5
#define V1_WIDTH	640
#define V1_HEIGHT	480

static mm_context_t *video_v1_ctx = NULL;

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_JPEG,
	.use_static_addr = 1,

	/* Enable external input mode. Any data address can be assigned as the input of encoder
	 * The source data format should be specified. 0:I420 1:NV12 2:NV21 11:RGB888 12:BGR888
	 * After opening video module, use command CMD_VIDEO_SET_EXT_INPUT to set the src data address and trigger the encoder */
	.out_mode = MODE_EXT,
	.ext_fmt = 1  //NV12
};

/* configure RAW data buffer for jpeg encoding */
#include "img_sample/person_640x480_nv12.c"
#define RAW_DATA_BUF    person_640x480_nv12

static int jpeg_encode_done_cb(uint32_t jpeg_addr, uint32_t jpeg_len)
{
	printf("[%s] snapshot size = %ld \r\n", __func__, jpeg_len);
	static int im_cnt = 0;
	printf("save jpg data... \r\n");
	char filename[32];
	snprintf(filename, sizeof(filename), "sd:/test_%04d.jpg", im_cnt);
	FILE *fp = fopen(filename, "w+");
	fwrite((void *)jpeg_addr, jpeg_len, 1, fp);
	fclose(fp);
	printf("save jpg data %s done \r\n", filename);

	im_cnt++;

	return 0;
}

static void example_deinit(void);

void mmf2_video_example_jpeg_external_init(void)
{
	vfs_init(NULL);
	if (vfs_user_register("sd", VFS_FATFS, VFS_INF_SD) != 0) {
		printf("fail to register SD vfs\r\n");
		return;
	}

	/*sensor capacity check & video parameter setting*/
	video_v1_params.resolution = VIDEO_VGA;
	video_v1_params.width = V1_WIDTH;
	video_v1_params.height = V1_HEIGHT;
	video_v1_params.fps = V1_FPS;
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, video_v1_params.width, video_v1_params.height, 0, 1,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 1, NULL, 0, NULL, 0, NULL);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SNAPSHOT_CB, (int)jpeg_encode_done_cb);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);
	} else {
		printf("video open fail\n\r");
		goto mmf2_jpeg_external_fail;
	}

	for (int i = 0; i < 10; i++) {
		//clean data cache to make sure the HW encoder can access latest data
		dcache_clean_by_addr((uint32_t *)RAW_DATA_BUF, sizeof(RAW_DATA_BUF));

		//set external buffer & trigger HW jpeg encode
		if (mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_EXT_INPUT, (int)RAW_DATA_BUF) < 0) {
			printf("fail to set hal_video_ext_in \r\n");
			goto mmf2_jpeg_external_fail;
		}
		vTaskDelay(5000);
	}

	//example deinit
	printf("example deinit\n\r");
	example_deinit();

	return;
mmf2_jpeg_external_fail:

	return;
}

static void example_deinit(void)
{
	//Stop module
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);

	//Close module
	video_v1_ctx = mm_module_close(video_v1_ctx);
	
	video_voe_release();
}
