/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "example_media_uvcd.h"
#include "uvc/inc/usbd_uvc_desc.h"
#include "mmf2_pro2_video_config.h"
#include "sensor.h"
#include "module_video.h"
#include "module_uvcd.h"
#include "log_service.h"
static unsigned char *snapshot_buf = NULL;
static uint32_t snapshot_len = 0;
static TaskHandle_t UvcMainHandle = NULL;
static int uvc_cmd_status = 0;
#define UVCD_CMD_START  0x00
#define UVCD_CMD_DEINIT 0X01
#define V1_CHANNEL 0
#define V1_FPS 5
#define V1_GOP 5
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2

#define VIDEO_TYPE VIDEO_JPEG

#define V1_RESOLUTION VIDEO_FHD
#define V1_WIDTH	1920
#define V1_HEIGHT	1080

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = V1_RESOLUTION,
	.width = V1_WIDTH,
	.height = V1_HEIGHT,
	.bps = V1_BPS,
	.fps = V1_FPS,
	.gop = V1_GOP,
	.jpeg_qlevel = 5,
	.rc_mode = V1_RCMODE,
	.direct_output = 1
};

struct video_snapshot_context {
	uint8_t snap_flag;
	_sema snapshot_sema;
	uint32_t isp_len;
	uint32_t isp_addr;
	hal_video_adapter_t  *v_adp;
	int type;
};

struct video_snapshot_context *video_ctx = NULL;
static void video_output_cb(void *param1, void  *param2, uint32_t arg)
{
	enc2out_t *enc2out = (enc2out_t *)param1;
	struct video_snapshot_context *video_ctx = (struct video_snapshot_context *)arg;
	if ((enc2out->codec & (CODEC_H264 | CODEC_HEVC | CODEC_JPEG)) != 0) {
		if (video_ctx->snap_flag) {
			video_ctx->isp_addr = (uint32_t)enc2out->jpg_addr;
			video_ctx->isp_len = enc2out->jpg_len;
			snapshot_buf = (unsigned char *)enc2out->jpg_addr;
			snapshot_len = video_ctx->isp_len;
			rtw_up_sema(&video_ctx->snapshot_sema);
			video_ctx->snap_flag = 0;
			video_encbuf_release(enc2out->ch, CODEC_JPEG, enc2out->jpg_len);
		}
	} else if ((enc2out->codec & (CODEC_NV12 | CODEC_RGB | CODEC_NV16)) != 0) {
		if (video_ctx->snap_flag) {
			video_ctx->isp_addr = (uint32_t)enc2out->isp_addr;
			video_ctx->isp_len = enc2out->width * enc2out->height * 3 / 2;
			snapshot_buf = (unsigned char *)video_ctx->isp_addr;
			snapshot_len = video_ctx->isp_len;
			rtw_up_sema(&video_ctx->snapshot_sema);
			video_ctx->snap_flag = 0;
			video_ispbuf_release(enc2out->ch, (uint32_t)enc2out->isp_addr);
		}
	}

}

static void video_snapshot_init(void *ctx)
{
	struct video_snapshot_context *video_ctx = (struct video_snapshot_context *)ctx;
	int iq_addr, sensor_addr;
	isp_info_t info;

	int voe_heap_size = video_buf_calc(1, V1_WIDTH, V1_HEIGHT, V1_BPS, 1,
									   0, 0, 0, 0, 0,
									   0, 0, 0, 0, 0,
									   0, 0, 0);

	int sensor_id_value = 0;
	for (int i = 0; i < SENSOR_MAX; i++) {
		if (sen_id[i] == USE_SENSOR) {
			sensor_id_value = i;
			break;
		}
	}

	voe_get_sensor_info(sensor_id_value, &iq_addr, &sensor_addr);
	//voe_get_sensor_info(USE_SENSOR, &iq_addr, &sensor_addr);

	video_ctx->v_adp = video_init(iq_addr, sensor_addr);

	rtw_init_sema(&video_ctx->snapshot_sema, 0);
}

static void video_snapshot_deinit(void *ctx)
{
	struct video_snapshot_context *video_ctx = (struct video_snapshot_context *)ctx;
	rtw_free_sema(&video_ctx->snapshot_sema);
	if (video_ctx) {
		free(video_ctx);
		video_ctx = NULL;
	}
	video_deinit();
}

static void video_snapshot_open(void *ctx)
{
	int ret = 0;
	struct video_snapshot_context *video_ctx = (struct video_snapshot_context *)ctx;
	ret = video_open(&video_v1_params, video_output_cb, (void *)ctx);
	if (ret < 0) {
		printf("Please check sensor fisrt, the ID is %d\r\n", USE_SENSOR);
		while (1) {
			vTaskDelay(100);
		}
	}
	if (video_v1_params.type == VIDEO_JPEG) {
		video_ctrl(video_v1_params.stream_id, VIDEO_JPEG_OUTPUT, 0);
	} else {
		video_ctrl(video_v1_params.stream_id, VIDEO_NV12_OUTPUT, 0);
	}
}

static unsigned char *video_snapshot_get(void *ctx)
{
	struct video_snapshot_context *video_ctx = (struct video_snapshot_context *)ctx;
	video_ctx->snap_flag = 1;
	if (video_v1_params.type == VIDEO_JPEG) {
		video_ctrl(video_v1_params.stream_id, VIDEO_JPEG_OUTPUT, 2);
	} else {
		video_ctrl(video_v1_params.stream_id, VIDEO_NV12_OUTPUT, 2);
	}
	if (rtw_down_timeout_sema(&video_ctx->snapshot_sema, 1000)) {
		if (video_v1_params.type == VIDEO_JPEG) {
			video_ctrl(video_v1_params.stream_id, VIDEO_JPEG_OUTPUT, 0);
		} else {
			video_ctrl(video_v1_params.stream_id, VIDEO_NV12_OUTPUT, 0);
		}
		return (unsigned char *)video_ctx->isp_addr;
	} else {
		printf("Can't get the buffer\r\n");
		return 0;
	}
}

static void video_snapshot_close(void *ctx)
{
	struct video_snapshot_context *video_ctx = (struct video_snapshot_context *)ctx;
	video_close(video_v1_params.stream_id);
}

#define num 1

static struct uvc_format *uvc_format_ptr = NULL;
static struct usbd_uvc_buffer uvc_payload[num];
struct uvc_format *uvc_format_local = NULL;

static void uvcd_change_format_resolution(void *ctx)
{
	rtw_up_sema_from_isr(&uvc_format_ptr->uvcd_change_sema);
}

static int uvcd_create(void)
{
	int i = 0;
	int status = 0;
	_usb_init();
	status = wait_usb_ready();
	if (status != USBD_INIT_OK) {
		if (status == USBD_NOT_ATTACHED) {
			printf("\r\n NO USB device attached\n");
		} else {
			printf("\r\n USB init fail\n");
		}
		goto exit;
	}

	if (usbd_uvc_init() < 0) {

		printf("usbd uvc init error\r\n");
	}


	for (i = 0; i < num; i++) {
		uvc_video_put_out_stream_queue(&uvc_payload[i]);
	}
	//uvc_ctx->change_parm_cb = uvcd_change_format_resolution;
	usbd_uvc_set_change_parm_cb((int)uvcd_change_format_resolution);
	return 0;
exit:
	return -1;
}
void atcmd_usb_uvc_init(void);
void example_media_dual_uvcd_init(void)
{
	video_ctx = malloc(sizeof(struct video_snapshot_context));
	memset(video_ctx, 0, sizeof(struct video_snapshot_context));
	video_snapshot_init(video_ctx);
	video_snapshot_open(video_ctx);

	uvc_format_ptr = (struct uvc_format *)malloc(sizeof(struct uvc_format));
	memset(uvc_format_ptr, 0, sizeof(struct uvc_format));

	uvc_format_local = (struct uvc_format *)malloc(sizeof(struct uvc_format));
	memset(uvc_format_local, 0, sizeof(struct uvc_format));

	rtw_init_sema(&uvc_format_ptr->uvcd_change_sema, 0);

	uvcd_create();

	struct usbd_uvc_buffer *payload = NULL;
	int count  = 0;
	video_snapshot_get(video_ctx);
	atcmd_usb_uvc_init();
	while (1) {
		vTaskDelay(50);
		if (usbd_uvc_get_status() == 0) { //(uvc_ctx->common->running == 0) {
			continue;
		}
		do {
			payload = uvc_video_out_stream_queue();
			if (payload == NULL) {
				printf("NULL\r\n");
				vTaskDelay(1);
			}
		} while (payload == NULL);
		count++;
		if (count % 10 == 0) {
			video_snapshot_get(video_ctx);
			printf("change the picture\r\n");
		}
		payload->mem = snapshot_buf;
		payload->bytesused = snapshot_len;
		uvc_video_put_in_stream_queue(payload);
		usbd_wait_frame_down();
	}
	vTaskDelete(NULL);
}
void atcmd_usb_uvc_init(void);
void example_media_uvcd_main(void *param)
{
	example_media_dual_uvcd_init();
}

void example_media_uvcd(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(example_media_uvcd_main, ((const char *)"example_media_dual_uvcd_main"), 512 * 10, NULL, tskIDLE_PRIORITY + 1,  &UvcMainHandle) != pdPASS) {
		printf("\r\n example_media_uvcd_main: Create Task Error\n");
	}
}

void example_media_uvcd_deinit(void)
{
	usbd_uvc_stop();
	vTaskDelay(100);
	video_snapshot_close(video_ctx);
	vTaskDelay(100);
	video_snapshot_deinit(video_ctx);

	usbd_uvc_deinit();
	extern void _usb_deinit(void);
	_usb_deinit();

	if (uvc_format_ptr) {
		free(uvc_format_ptr);
		uvc_format_ptr = NULL;
	}

	if (uvc_format_local) {
		free(uvc_format_local);
		uvc_format_local = NULL;
	}

	rtw_free_sema(&uvc_format_ptr->uvcd_change_sema);

	vTaskDelete(UvcMainHandle);
}

void AUVCD(void *arg)//uvc finish
{
	if (uvc_cmd_status == UVCD_CMD_START) {
		printf("stop uvc procedure\r\n");
		example_media_uvcd_deinit();
		uvc_cmd_status = UVCD_CMD_DEINIT;
	} else {
		printf("It has already deinit\r\n");
	}
}

void AUVCE(void *arg)//uvc finish
{
	if (uvc_cmd_status == UVCD_CMD_DEINIT) {
		example_media_uvcd();
		uvc_cmd_status = UVCD_CMD_START;
	} else {
		printf("The example is running\r\n");
	}
}


log_item_t usb_uvc_items[] = {
	{"UVCD", AUVCD,},
	{"UVCE", AUVCE,},
};

void atcmd_usb_uvc_init(void)
{
	log_service_add_table(usb_uvc_items, sizeof(usb_uvc_items) / sizeof(usb_uvc_items[0]));
}