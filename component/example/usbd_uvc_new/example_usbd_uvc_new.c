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
 *
 * Console USBUVC=<0|1|2[,rounds]>: full stop / start of the camera (video thread, UVC class,
 * USB core), and a stop/start cycle printing heap and task count, same as the sensor modes.
 */

/* Includes ------------------------------------------------------------------ */

#include "usbd.h"
#include "usbd_uvc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "example_usbd_uvc_new.h"
#if USBD_UVC_NEW_DFU
#include "example_usbd_dfu_new.h"
#endif

/* Sensor modes live in example_usbd_uvc_new_media.c, the clip is only needed here */
#if (USBD_UVC_NEW_MODE == USBD_UVC_NEW_MODE_SAMPLE)
#include "semphr.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include "log_service.h"
#include "example_usbd_uvc_new_sample_h264.h"

/* Private defines -----------------------------------------------------------*/
static const char *const TAG = "UVC";

// Thread priorities
#define UVC_CTRL_THREAD_PRIORITY                        (tskIDLE_PRIORITY + 5)
#define UVC_VIDEO_THREAD_PRIORITY                       (tskIDLE_PRIORITY + 5)

// Thread stack sizes
#define UVC_CTRL_THREAD_STACK_SIZE                     1024U
#define UVC_VIDEO_THREAD_STACK_SIZE                    1024U

// Video parameters
#define USBD_UVC_VIDEO_BUF_NUM                          1U
#define USBD_UVC_VIDEO_FPS                              30U
#define USBD_UVC_H264_NAL_SIZE                          4U

#define USBD_UVC_FRAME_INTERVAL_MS                      (1000U / USBD_UVC_VIDEO_FPS)

// Stop / cycle test
#define UVC_STOP_WAIT_MS                                2000U
#define UVC_CYCLE_DEFAULT                               10U
#define UVC_CYCLE_RUN_MS                                3000U

#define UVC_EVT_START                                   1U
#define UVC_EVT_STOP                                    2U
#define UVC_EVT_CYCLE                                   3U

/* Private types -------------------------------------------------------------*/

typedef struct video_array_s {
	const u8 *data;
	u32 data_len;
	u32 data_offset;
	u32 fps;
	u32 h264_nal_size;
	u32 size;
} video_array_t;

typedef struct {
	u32 evt;
	u32 rounds;
} uvc_ctrl_msg_t;

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static video_array_t video;
static usbd_uvc_buffer_t uvc_payload[USBD_UVC_VIDEO_BUF_NUM];
/* USB hand-over with the built-in DFU: video thread parked while uvc_sample_paused is set */
static volatile u8 uvc_sample_pause;
static volatile u8 uvc_sample_paused;
/* Full stop: the video thread exits on uvc_stop_req */
static volatile u8 uvc_stop_req;
static volatile u8 uvc_video_alive;
/* uvc_lock serializes start / stop (ctrl thread) with the DFU hand-over (DFU control thread) */
static SemaphoreHandle_t uvc_lock;
static QueueHandle_t uvc_ctrl_queue;
static u8 uvc_running;
static u8 uvc_dfu_owns_usb;

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

	while (uvc_stop_req == 0U) {
		vTaskDelay(pdMS_TO_TICKS(USBD_UVC_FRAME_INTERVAL_MS));
		/* USB handed to DFU: stay out of the UVC API until it is back */
		uvc_sample_paused = uvc_sample_pause;
		if (uvc_sample_pause != 0U) {
			continue;
		}
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
		} while ((payload == NULL) && (uvc_stop_req == 0U) && (usbd_uvc_get_status() != 0));
		if (payload == NULL) {
			continue;
		}

		payload->mem = video_data->data + video_data->data_offset;
		payload->bytesused = video_data->size;
		video_data->data_offset += video_data->size;

		usbd_uvc_video_put_in_stream_queue(payload);
		usbd_uvc_wait_frame_down();
	}
	uvc_video_alive = 0U;
	vTaskDelete(NULL);
}

static void uvc_payload_init(void)
{
	u32 i;

	for (i = 0U; i < USBD_UVC_VIDEO_BUF_NUM; i++) {
		INIT_LIST_HEAD(&uvc_payload[i].buffer_list);
		usbd_uvc_video_put_out_stream_queue(&uvc_payload[i]);
	}
}

static void uvc_print_res(const char *what)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "%s: heap %u (min %u) tasks %u\n", what, (u32)xPortGetFreeHeapSize(),
			 (u32)xPortGetMinimumEverFreeHeapSize(), (u32)uxTaskGetNumberOfTasks());
}

/* USB core + UVC class + video thread. uvc_lock held. */
static int uvc_sample_start(void)
{
	if (uvc_running != 0U) {
		RTK_LOGS(TAG, RTK_LOG_INFO, "Already started\n");
		return 0;
	}
	if (uvc_dfu_owns_usb != 0U) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "USB port is used by DFU, USBDFU=0 first\n");
		return -1;
	}
	if (usbd_init(&uvc_cfg) != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_init fail\n");
		return -1;
	}
	if (usbd_uvc_init() != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_uvc_init fail\n");
		usbd_deinit();
		return -1;
	}

	video.data = h264_sample;
	video.data_len = (u32)h264_sample_len;
	video.data_offset = 0;
	video.fps = USBD_UVC_VIDEO_FPS;
	video.h264_nal_size = USBD_UVC_H264_NAL_SIZE;
	uvc_payload_init();

	uvc_stop_req = 0U;
	uvc_sample_pause = 0U;
	uvc_video_alive = 1U;
	if (xTaskCreate(example_usbd_uvc_new_video_thread, "usbd_uvc_new_video", UVC_VIDEO_THREAD_STACK_SIZE,
					NULL, UVC_VIDEO_THREAD_PRIORITY, NULL) != pdPASS) {
		uvc_video_alive = 0U;
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create video thread fail\n");
		usbd_uvc_deinit();
		usbd_deinit();
		return -1;
	}

	uvc_running = 1U;
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD UVC demo start\n");
	return 0;
}

/* Undo uvc_sample_start(). uvc_lock held. */
static int uvc_sample_stop(void)
{
	u32 wait_ms;

	if (uvc_running == 0U) {
		RTK_LOGS(TAG, RTK_LOG_INFO, "Already stopped\n");
		return 0;
	}
	/* The video thread checks once per frame interval; a frame in flight finishes first */
	uvc_stop_req = 1U;
	for (wait_ms = 0U; (uvc_video_alive != 0U) && (wait_ms < UVC_STOP_WAIT_MS); wait_ms += 10U) {
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	if (uvc_video_alive != 0U) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Video thread still running\n");
		return -1;
	}
	/* USB class before the core; skipped when DFU already took the port */
	if (usbd_uvc_get_inst_num() != 0U) {
		usbd_uvc_deinit();
		usbd_deinit();
	}
	uvc_sample_pause = 0U;
	uvc_running = 0U;
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD UVC demo stop\n");
	return 0;
}

#if USBD_UVC_NEW_DFU
/*
 * USB hand-over with the built-in USB DFU (example_usbd_dfu_new.c): USBDFU=1 parks the video
 * thread and releases the USB port, USBDFU=0 brings the camera back if it was running.
 * DFU control thread context.
 */
int usbd_dfu_new_usb_release(void)
{
	u32 wait_ms;
	int ret = 0;

	if (uvc_lock == NULL) {
		return 0;
	}
	xSemaphoreTake(uvc_lock, portMAX_DELAY);
	/* Also while stopped: USBUVC=1 must not take the port from DFU */
	uvc_dfu_owns_usb = 1U;
	if ((uvc_running != 0U) && (usbd_uvc_get_inst_num() != 0U)) {
		uvc_sample_pause = 1U;
		for (wait_ms = 0U; (uvc_sample_paused == 0U) && (wait_ms < 1000U); wait_ms += 10U) {
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		if (uvc_sample_paused == 0U) {
			uvc_sample_pause = 0U;
			uvc_dfu_owns_usb = 0U;
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Video thread busy, USB not released\n");
			ret = -1;
		} else {
			usbd_uvc_deinit();
			usbd_deinit();
			RTK_LOGS(TAG, RTK_LOG_INFO, "UVC released the USB port for DFU\n");
		}
	}
	xSemaphoreGive(uvc_lock);
	return ret;
}

void usbd_dfu_new_usb_restore(void)
{
	if (uvc_lock == NULL) {
		return;
	}
	xSemaphoreTake(uvc_lock, portMAX_DELAY);
	uvc_dfu_owns_usb = 0U;
	if ((uvc_running == 0U) || (usbd_uvc_get_inst_num() != 0U)) {
		/* Stopped (USBUVC=0) while DFU had the port, or never released */
		goto exit;
	}
	if (usbd_init(&uvc_cfg) != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "UVC restore: usbd_init fail\n");
		goto exit;
	}
	if (usbd_uvc_init() != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "UVC restore: usbd_uvc_init fail\n");
		usbd_deinit();
		goto exit;
	}
	uvc_payload_init();
	uvc_sample_pause = 0U;
	RTK_LOGS(TAG, RTK_LOG_INFO, "UVC restored\n");
exit:
	xSemaphoreGive(uvc_lock);
}
#endif

/* N stop/start rounds, resources printed after each; a leak shows as heap going down round by round */
static void uvc_sample_cycle(u32 rounds)
{
	u32 i;

	uvc_print_res("Cycle begin");
	for (i = 1U; i <= rounds; i++) {
		xSemaphoreTake(uvc_lock, portMAX_DELAY);
		if (uvc_sample_stop() != 0) {
			xSemaphoreGive(uvc_lock);
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Cycle %u: stop fail\n", i);
			return;
		}
		uvc_print_res("Cycle stopped");
		if (uvc_sample_start() != 0) {
			xSemaphoreGive(uvc_lock);
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Cycle %u: start fail\n", i);
			return;
		}
		xSemaphoreGive(uvc_lock);
		/* Let the host enumerate and open the camera before the next stop */
		vTaskDelay(pdMS_TO_TICKS(UVC_CYCLE_RUN_MS));
		RTK_LOGS(TAG, RTK_LOG_INFO, "Cycle %u/%u done\n", i, rounds);
		uvc_print_res("Cycle running");
	}
	RTK_LOGS(TAG, RTK_LOG_INFO, "Cycle test done, %u rounds\n", rounds);
}

/* Console: USBUVC=<0|1|2[,rounds]>  0: stop, 1: start, 2: stop/start rounds; no argument: status */
static void USBUVC(void *arg)
{
	int argc;
	char *argv[MAX_ARGC] = {0};
	uvc_ctrl_msg_t msg = {0};

	argc = parse_param((char *)arg, argv);
	if ((argc >= 2) && (strcmp(argv[1], "0") == 0)) {
		msg.evt = UVC_EVT_STOP;
	} else if ((argc >= 2) && (strcmp(argv[1], "1") == 0)) {
		msg.evt = UVC_EVT_START;
	} else if ((argc >= 2) && (strcmp(argv[1], "2") == 0)) {
		msg.evt = UVC_EVT_CYCLE;
		msg.rounds = (argc >= 3) ? (u32)atoi(argv[2]) : UVC_CYCLE_DEFAULT;
		if (msg.rounds == 0U) {
			msg.rounds = UVC_CYCLE_DEFAULT;
		}
	} else {
		RTK_LOGS(TAG, RTK_LOG_INFO, "Usage: USBUVC=<0|1|2[,rounds]>  0: stop, 1: start, 2: stop/start rounds\n");
		RTK_LOGS(TAG, RTK_LOG_INFO, "UVC %s%s\n", (uvc_running != 0U) ? "running" : "stopped",
				 (uvc_dfu_owns_usb != 0U) ? ", USB used by DFU" : "");
		uvc_print_res("Now");
		return;
	}
	if (xQueueSend(uvc_ctrl_queue, &msg, 0) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_WARN, "Busy, try again\n");
	}
}

static log_item_t uvc_cmd_items[] = {
	{"USBUVC", USBUVC,},
};

/* Starts the camera at boot, then runs USBUVC requests */
static void example_usbd_uvc_new_thread(void *param)
{
	uvc_ctrl_msg_t msg;

	UNUSED(param);

	xSemaphoreTake(uvc_lock, portMAX_DELAY);
	(void)uvc_sample_start();
	xSemaphoreGive(uvc_lock);
	uvc_print_res("Boot");

	for (;;) {
		if (xQueueReceive(uvc_ctrl_queue, &msg, portMAX_DELAY) != pdPASS) {
			continue;
		}
		if (msg.evt == UVC_EVT_CYCLE) {
			uvc_sample_cycle(msg.rounds);
			continue;
		}
		xSemaphoreTake(uvc_lock, portMAX_DELAY);
		if (msg.evt == UVC_EVT_START) {
			(void)uvc_sample_start();
		} else {
			(void)uvc_sample_stop();
		}
		xSemaphoreGive(uvc_lock);
		uvc_print_res((msg.evt == UVC_EVT_START) ? "Start" : "Stop");
	}
}

#endif /* USBD_UVC_NEW_MODE == USBD_UVC_NEW_MODE_SAMPLE */

/* Exported functions --------------------------------------------------------*/

void example_usbd_uvc_new(void)
{
#if (USBD_UVC_NEW_MODE != USBD_UVC_NEW_MODE_SAMPLE)
	/* Sensor modes, see example_usbd_uvc_new_media.c */
	example_usbd_uvc_new_media();
#else
	/* Before example_usbd_dfu_new(): the DFU hooks take uvc_lock */
	uvc_lock = xSemaphoreCreateMutex();
	uvc_ctrl_queue = xQueueCreate(4, sizeof(uvc_ctrl_msg_t));
	if ((uvc_lock == NULL) || (uvc_ctrl_queue == NULL)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "No mem\n");
		return;
	}
	log_service_add_table(uvc_cmd_items, sizeof(uvc_cmd_items) / sizeof(uvc_cmd_items[0]));
	if (xTaskCreate(example_usbd_uvc_new_thread, "usbd_uvc_new_ctrl", UVC_CTRL_THREAD_STACK_SIZE,
					NULL, UVC_CTRL_THREAD_PRIORITY, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create USBD UVC thread fail\n");
	}
#endif

#if USBD_UVC_NEW_DFU
	/* USB DFU firmware update stays available: USBDFU=1 hands the USB port to DFU */
	example_usbd_dfu_new();
#endif
}
