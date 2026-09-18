/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Uses the selected ported stack's standalone UVC API and streams a looping
 * 1920x1080@30fps H.264 sample clip
 * (example_usbd_uvc_new_sample_h264.h) so a PC UVC viewer (e.g. PotPlayer)
 * can display it once the board enumerates as a UVC camera.
 *
 * component/os/os_wrapper (rtos_task_t/rtos_sema_t/rtos_*) does not exist in
 * this SDK generation, so task/semaphore handling below calls
 * FreeRTOS directly instead, the same way component/example/usbd_hid_new
 * already does.
 *
 * No hotplug thread here (unlike usbd_hid_new/usbd_msc_new): the UVC class
 * driver's usbd_class_driver_t (usbd_uvc_driver in usbd_uvc.c) never wires a
 * .status_changed callback, so there is no ISR-driven attach/detach event to
 * react to - matching the reference example, which also has no hotplug
 * handling.
 *
 * The usbd_config_t / FIFO depth values below are configured for the
 * amebapro2 (rtl8735b) USB OTG controller (EP3 IN ISOC, see
 * USBD_UVC_ISO_IN_EP in usbd_uvc.h).
 */

/* Includes ------------------------------------------------------------------ */

#include "usbd.h"
#include "usbd_uvc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "example_usbd_uvc_new_sample_h264.h"

/* Private defines -----------------------------------------------------------*/
static const char *const TAG = "UVC";

// Thread priorities
#define UVC_INIT_THREAD_PRIORITY                        (tskIDLE_PRIORITY + 5)
#define UVC_VIDEO_THREAD_PRIORITY                       (tskIDLE_PRIORITY + 5)

// Thread stack sizes
#define UVC_INIT_THREAD_STACK_SIZE                     1024U
#define UVC_VIDEO_THREAD_STACK_SIZE                    1024U

// Video parameters
#define USBD_UVC_VIDEO_BUF_NUM                          1U
#define USBD_UVC_VIDEO_FPS                              30U
#define USBD_UVC_H264_NAL_SIZE                          4U

#define USBD_UVC_FRAME_INTERVAL_MS                      (1000U / USBD_UVC_VIDEO_FPS)

/* Private types -------------------------------------------------------------*/

typedef struct video_array_s {
	const u8 *data;
	u32 data_len;
	u32 data_offset;
	u32 fps;
	u32 h264_nal_size;
	u32 size;
} video_array_t;

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static video_array_t video;
static usbd_uvc_buffer_t uvc_payload[USBD_UVC_VIDEO_BUF_NUM];

static const usbd_config_t uvc_cfg = {
	.speed = USB_SPEED_HIGH,
	/* 4U matches component/example/usbd_hid_new's validated value. */
	.isr_priority = 4U,
	/* Enable SOF interrupt: drives ISOC IN arming/recovery and the SCR SOF counter. */
	.ext_intr_enable = USBD_SOF_INTR,
	/* DFIFO total 2232 DWORD, resv 8 DWORD for DMA addr and EP0 fixed 256 DWORD.
	 * ptx_fifo_depth[1] (not [2]) backs EP3
	 * IN: usbd_hal_get_tx_fifo_num() remaps EP3 -> dedicated TX FIFO #2, see
	 * USBD_UVC_ISO_IN_EP's comment in usbd_uvc.h. */
	.rx_fifo_depth = 1168U,
	.ptx_fifo_depth = {16U, 768U, 16U, },
};

/* Private functions ---------------------------------------------------------*/

static u32 array_get_h264_frame_size(const u8 *ptr_start, const u8 *ptr_end, u8 nal_len)
{
	const u8 *ptr = ptr_start;
	u8 skip_flag = 1U;

	if ((ptr_start >= ptr_end) || ((nal_len != 3U) && (nal_len != 4U))) {
		return 0U;
	}

	while ((ptr + nal_len) < ptr_end) {
		if ((ptr[0] == 0U) && (ptr[1] == 0U)) {
			if (((nal_len == 4U) && (ptr[2] == 0U) && (ptr[3] == 1U))
				|| ((nal_len == 3U) && (ptr[2] == 1U))) {
				if (((ptr[nal_len] & 0x1fU) != 0x07U) && ((ptr[nal_len] & 0x1fU) != 0x08U)) {
					if (skip_flag == 0U) {
						return (u32)(ptr - ptr_start);
					} else {
						skip_flag = 0U;
					}
				} else if ((ptr[nal_len] & 0x1fU) == 0x08U) {
					skip_flag = 1U;
				}
			}
		}
		ptr++;
	}

	return (u32)(ptr_end - ptr_start);
}

static void example_usbd_uvc_new_video_thread(void *param)
{
	video_array_t *video_data = &video;
	usbd_uvc_buffer_t *payload = NULL;

	UNUSED(param);

	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(USBD_UVC_FRAME_INTERVAL_MS));
		if (usbd_uvc_get_status() == 0) {
			continue;
		}

		if (video_data->data_offset >= video_data->data_len) {
			video_data->data_offset = 0;
		}

		video_data->size = array_get_h264_frame_size(video_data->data + video_data->data_offset,
						   video_data->data + video_data->data_len, video_data->h264_nal_size);
		if (video_data->size == 0U) {
			video_data->data_offset = 0;
			continue;
		}

		do {
			payload = usbd_uvc_video_out_stream_queue();
			if (payload == NULL) {
				vTaskDelay(pdMS_TO_TICKS(1));
			}
		} while (payload == NULL);

		payload->mem = video_data->data + video_data->data_offset;
		payload->bytesused = video_data->size;
		video_data->data_offset += video_data->size;

		usbd_uvc_video_put_in_stream_queue(payload);
		usbd_uvc_wait_frame_down();
	}
}

static void example_usbd_uvc_new_thread(void *param)
{
	int ret = HAL_OK;
	TaskHandle_t video_task;
	u32 i = 0U;

	UNUSED(param);

	ret = usbd_init(&uvc_cfg);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_init fail\n");
		goto exit_usbd_init_fail;
	}

	ret = usbd_uvc_init();
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_uvc_init fail\n");
		goto exit_usbd_uvc_init_fail;
	}

	video.data = h264_sample;
	video.data_len = (u32)h264_sample_len;
	video.data_offset = 0;
	video.fps = USBD_UVC_VIDEO_FPS;
	video.h264_nal_size = USBD_UVC_H264_NAL_SIZE;

	for (i = 0U; i < USBD_UVC_VIDEO_BUF_NUM; i++) {
		INIT_LIST_HEAD(&uvc_payload[i].buffer_list);
		usbd_uvc_video_put_out_stream_queue(&uvc_payload[i]);
	}

	if (xTaskCreate(example_usbd_uvc_new_video_thread, "usbd_uvc_new_video", UVC_VIDEO_THREAD_STACK_SIZE,
					NULL, UVC_VIDEO_THREAD_PRIORITY, &video_task) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create video thread fail\n");
		goto exit_create_video_task_fail;
	}

	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD UVC demo start\n");
	vTaskDelete(NULL);
	return;

exit_create_video_task_fail:
	usbd_uvc_deinit();
exit_usbd_uvc_init_fail:
	usbd_deinit();
exit_usbd_init_fail:
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD UVC demo stop\n");
	vTaskDelete(NULL);
}

/* Exported functions --------------------------------------------------------*/

void example_usbd_uvc_new(void)
{
	if (xTaskCreate(example_usbd_uvc_new_thread, "usbd_uvc_new_init", UVC_INIT_THREAD_STACK_SIZE,
					NULL, UVC_INIT_THREAD_PRIORITY, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create USBD UVC thread fail\n");
	}
}
