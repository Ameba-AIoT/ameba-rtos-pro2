/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Uses the selected ported stack's standalone HID API in mouse-only mode
 * (CONFIG_USBD_HID_KEYBOARD is not defined), matching the boot-mouse HID
 * interface in usbd_composite_msc_hid_new.
 *
 * component/os/os_wrapper (rtos_task_t/rtos_sema_t/rtos_*) does not exist in
 * this SDK generation, so task/semaphore handling below calls FreeRTOS
 * directly. The MOUSE command uses this project's
 * log_service_add_table()/log_item_t registration mechanism.
 *
 * The usb_hal_driver / FIFO depth values below are configured for the
 * amebapro2 (rtl8735b) USB OTG controller.
 */

/* Includes ------------------------------------------------------------------ */

#include "usbd_hid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log_service.h"
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/

/* Re-init the USB device stack on hot-plug (detach/attach), matching the
 * upstream example - avoids leaking the previous session's driver state. */
#define CONFIG_USBD_HID_HOTPLUG                        1

#define HID_TX_BUF_LEN                                  USBD_HID_INTR_IN_BUF_SIZE

#define HID_INIT_THREAD_STACK_SIZE                     1024U
#define HID_INIT_THREAD_PRIORITY                        (tskIDLE_PRIORITY + 5)
#define HID_HOTPLUG_THREAD_STACK_SIZE                  1024U
#define HID_HOTPLUG_THREAD_PRIORITY                     (tskIDLE_PRIORITY + 6)

/* Private function prototypes -----------------------------------------------*/

static void hid_cb_init(void);
static void hid_cb_deinit(void);
static void hid_cb_setup(void);
static void hid_cb_transmitted(u8 status);
static void hid_cb_status_changed(u8 old_status, u8 status);

/* Private variables ---------------------------------------------------------*/

static const char *const TAG = "HID";

static const usbd_config_t hid_cfg = {
	.speed = USB_SPEED_HIGH,
	/* 4U matches component/example/usbd_cdc_acm_new's validated value. */
	.isr_priority = 4U,
	/* DFIFO total 2232 DWORD, resv 8 DWORD for DMA addr and EP0 fixed 256 DWORD. */
	.rx_fifo_depth = 1680U,
	.ptx_fifo_depth = {256U, 16U, 16U},
};

static const usbd_hid_usr_cb_t hid_usr_cb = {
	.init = hid_cb_init,
	.deinit = hid_cb_deinit,
	.setup = hid_cb_setup,
	.transmitted = hid_cb_transmitted,
	.status_changed = hid_cb_status_changed,
};

#if CONFIG_USBD_HID_HOTPLUG
static u8 hid_attach_status;
static SemaphoreHandle_t hid_attach_status_changed_sema;
#endif

/* Private functions ---------------------------------------------------------*/

static void hid_cb_init(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "INIT\n");
}

static void hid_cb_deinit(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "DEINIT\n");
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static void hid_cb_setup(void)
{
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static void hid_cb_transmitted(u8 status)
{
	UNUSED(status);
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static void hid_cb_status_changed(u8 old_status, u8 status)
{
	UNUSED(old_status);
#if CONFIG_USBD_HID_HOTPLUG
	BaseType_t woken = pdFALSE;

	hid_attach_status = status;
	xSemaphoreGiveFromISR(hid_attach_status_changed_sema, &woken);
	portYIELD_FROM_ISR(woken);
#else
	UNUSED(status);
#endif
}

/**
  * @brief  Console command to drive the boot-mouse HID interrupt-IN report:
  *         "MOUSE=<left>[,<right>,<middle>,<x_axis>,<y_axis>,<wheel>]"
  *         left/right/middle: 0=release, non-zero=press.
  *         x_axis/y_axis/wheel: signed, -127..127.
  */
static void MOUSE(void *arg)
{
	int argc;
	char *argv[MAX_ARGC] = {0};
	u8 report[4] = {0};
	int ret;

	argc = parse_param((char *)arg, argv);

	if (argc < 2) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Usage: MOUSE=<left>[,<right>,<middle>,<x_axis>,<y_axis>,<wheel>]\n");
		return;
	}

	if (strtoul(argv[1], NULL, 10) != 0) {
		report[0] |= USBD_HID_MOUSE_BUTTON_LEFT;
	}
	if ((argc > 2) && (strtoul(argv[2], NULL, 10) != 0)) {
		report[0] |= USBD_HID_MOUSE_BUTTON_RIGHT;
	}
	if ((argc > 3) && (strtoul(argv[3], NULL, 10) != 0)) {
		report[0] |= USBD_HID_MOUSE_BUTTON_MIDDLE;
	}
	report[0] |= USBD_HID_MOUSE_BUTTON_RESERVED;
	if (argc > 4) {
		report[1] = (u8)strtol(argv[4], NULL, 10);
	}
	if (argc > 5) {
		report[2] = (u8)strtol(argv[5], NULL, 10);
	}
	if (argc > 6) {
		report[3] = (u8)strtol(argv[6], NULL, 10);
	}

	ret = usbd_hid_send_data(report, sizeof(report));
	RTK_LOGS(TAG, RTK_LOG_INFO, "MOUSE: buttons=%02x x=%d y=%d wheel=%d, ret=%d\n",
			 report[0], (s8)report[1], (s8)report[2], (s8)report[3], ret);
}

static log_item_t hid_cmd_items[] = {
	{"MOUSE", MOUSE,},
};

#if CONFIG_USBD_HID_HOTPLUG
static void hid_hotplug_thread(void *param)
{
	int ret;

	UNUSED(param);

	for (;;) {
		if (xSemaphoreTake(hid_attach_status_changed_sema, portMAX_DELAY) == pdTRUE) {
			if (hid_attach_status == USBD_ATTACH_STATUS_DETACHED) {
				RTK_LOGS(TAG, RTK_LOG_INFO, "DETACHED\n");
				usbd_hid_deinit();
				ret = usbd_deinit();
				if (ret != HAL_OK) {
					break;
				}
				ret = usbd_init(&hid_cfg);
				if (ret != HAL_OK) {
					break;
				}
				ret = usbd_hid_init(HID_TX_BUF_LEN, &hid_usr_cb);
				if (ret != HAL_OK) {
					usbd_deinit();
					break;
				}
			} else if (hid_attach_status == USBD_ATTACH_STATUS_ATTACHED) {
				RTK_LOGS(TAG, RTK_LOG_INFO, "ATTACHED\n");
			} else {
				RTK_LOGS(TAG, RTK_LOG_INFO, "INIT\n");
			}
		}
	}

	RTK_LOGS(TAG, RTK_LOG_INFO, "Hotplug thread exit\n");
	vTaskDelete(NULL);
}
#endif

static void example_usbd_hid_new_thread(void *param)
{
	int ret;
#if CONFIG_USBD_HID_HOTPLUG
	TaskHandle_t check_task;
#endif

	UNUSED(param);

#if CONFIG_USBD_HID_HOTPLUG
	hid_attach_status_changed_sema = xSemaphoreCreateBinary();
#endif

	ret = usbd_init(&hid_cfg);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_init fail\n");
		goto exit_usbd_init_fail;
	}

	ret = usbd_hid_init(HID_TX_BUF_LEN, &hid_usr_cb);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_hid_init fail\n");
		goto exit_usbd_hid_init_fail;
	}

#if CONFIG_USBD_HID_HOTPLUG
	if (xTaskCreate(hid_hotplug_thread, "hid_hotplug", HID_HOTPLUG_THREAD_STACK_SIZE, NULL,
					HID_HOTPLUG_THREAD_PRIORITY, &check_task) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create hotplug thread fail\n");
		goto exit_create_check_task_fail;
	}
#endif

	log_service_add_table(hid_cmd_items, sizeof(hid_cmd_items) / sizeof(hid_cmd_items[0]));

	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD HID demo start\n");
	vTaskDelete(NULL);
	return;

#if CONFIG_USBD_HID_HOTPLUG
exit_create_check_task_fail:
	usbd_hid_deinit();
#endif
exit_usbd_hid_init_fail:
	usbd_deinit();
exit_usbd_init_fail:
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD HID demo stop\n");
#if CONFIG_USBD_HID_HOTPLUG
	vSemaphoreDelete(hid_attach_status_changed_sema);
#endif
	vTaskDelete(NULL);
}

/* Exported functions --------------------------------------------------------*/

void example_usbd_hid_new(void)
{
	if (xTaskCreate(example_usbd_hid_new_thread, "usbd_hid_new_init", HID_INIT_THREAD_STACK_SIZE,
					NULL, HID_INIT_THREAD_PRIORITY, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create USBD HID thread fail\n");
	}
}
