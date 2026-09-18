/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Uses the selected ported stack's standalone MSC API.
 * component/os/os_wrapper (rtos_task_t/rtos_sema_t/rtos_*) does not exist in
 * this SDK generation, so task/semaphore handling below calls FreeRTOS
 * directly, the same way component/example/usbd_cdc_acm_new already does.
 *
 * SD-card hotplug (CONFIG_USBD_MSC_SD_HOTPLUG) and second-flash
 * (CONFIG_USBD_MSC_SECOND_FLASH) support are dropped - neither "ameba_sd.h"
 * nor "vfs_second_nor_flash.h" exist on this SDK, matching the same scope
 * decision already made for usbd_msc.c's own storage backend (RAM disk by
 * default, or a real SD card via CONFIG_USBD_MSC_SD_MODE - see
 * component/usb/usb_stack/<version>/device/msc/usbd_msc.c's disk_init/disk_deinit, which
 * this example just calls through). USB hotplug (re-init the USB stack on
 * detach/attach) is kept, matching example_usbd_cdc_acm_new's own hotplug
 * thread pattern.
 *
 * The usb_hal_driver / FIFO depth values below are configured for the
 * amebapro2 (rtl8735b) USB OTG controller.
 */

/* Includes ------------------------------------------------------------------ */

#include "usbd_msc.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private defines -----------------------------------------------------------*/

/* Re-init the USB device stack on hot-plug (detach/attach), matching the
 * upstream example - avoids leaking the previous session's driver state. */
#define CONFIG_USBD_MSC_HOTPLUG                        1

#define MSC_INIT_THREAD_STACK_SIZE                     1024U
#define MSC_INIT_THREAD_PRIORITY                        (tskIDLE_PRIORITY + 5)
#define MSC_HOTPLUG_THREAD_STACK_SIZE                  1024U
#define MSC_HOTPLUG_THREAD_PRIORITY                     (tskIDLE_PRIORITY + 6)

/* Private function prototypes -----------------------------------------------*/

static void msc_cb_status_changed(u8 old_status, u8 status);

/* Private variables ---------------------------------------------------------*/

static const char *const TAG = "MSC";

static const usbd_config_t msc_cfg = {
	.speed = USB_SPEED_HIGH,
	/* 4U matches component/example/usbd_cdc_acm_new's validated value. */
	.isr_priority = 4U,
	/* DFIFO total 2232 DWORD, resv 8 DWORD for DMA addr and EP0 fixed 256 DWORD. */
	.rx_fifo_depth = 1680U,
	.ptx_fifo_depth = {256U, 16U, 16U},
};

static const usbd_msc_cb_t msc_cb = {
	.status_changed = msc_cb_status_changed,
};

#if CONFIG_USBD_MSC_HOTPLUG
static u8 msc_attach_status;
static SemaphoreHandle_t msc_attach_status_changed_sema;
#endif

/* Private functions ---------------------------------------------------------*/

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static void msc_cb_status_changed(u8 old_status, u8 status)
{
	UNUSED(old_status);
#if CONFIG_USBD_MSC_HOTPLUG
	BaseType_t woken = pdFALSE;

	msc_attach_status = status;
	xSemaphoreGiveFromISR(msc_attach_status_changed_sema, &woken);
	portYIELD_FROM_ISR(woken);
#else
	UNUSED(status);
#endif
}

#if CONFIG_USBD_MSC_HOTPLUG
static void msc_hotplug_thread(void *param)
{
	int ret;

	UNUSED(param);

	for (;;) {
		if (xSemaphoreTake(msc_attach_status_changed_sema, portMAX_DELAY) == pdTRUE) {
			if (msc_attach_status == USBD_ATTACH_STATUS_DETACHED) {
				RTK_LOGS(TAG, RTK_LOG_INFO, "DETACHED\n");
				usbd_msc_deinit();
				ret = usbd_deinit();
				if (ret != HAL_OK) {
					break;
				}
				usbd_msc_disk_deinit();
				usbd_msc_disk_init();
				ret = usbd_init(&msc_cfg);
				if (ret != HAL_OK) {
					break;
				}
				ret = usbd_msc_init(&msc_cb);
				if (ret != HAL_OK) {
					usbd_deinit();
					break;
				}
			} else if (msc_attach_status == USBD_ATTACH_STATUS_ATTACHED) {
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

static void example_usbd_msc_new_thread(void *param)
{
	int ret;
#if CONFIG_USBD_MSC_HOTPLUG
	TaskHandle_t check_task;
#endif

	UNUSED(param);

#if CONFIG_USBD_MSC_HOTPLUG
	msc_attach_status_changed_sema = xSemaphoreCreateBinary();
#endif

	ret = usbd_msc_disk_init();
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Init disk fail\n");
		goto exit_disk_init_fail;
	}

	ret = usbd_init(&msc_cfg);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_init fail\n");
		goto exit_usbd_init_fail;
	}

	ret = usbd_msc_init(&msc_cb);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_msc_init fail\n");
		goto exit_usbd_msc_init_fail;
	}

#if CONFIG_USBD_MSC_HOTPLUG
	if (xTaskCreate(msc_hotplug_thread, "msc_hotplug", MSC_HOTPLUG_THREAD_STACK_SIZE, NULL,
					MSC_HOTPLUG_THREAD_PRIORITY, &check_task) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create hotplug thread fail\n");
		goto exit_create_check_task_fail;
	}
#endif

	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD MSC demo start\n");
	vTaskDelete(NULL);
	return;

#if CONFIG_USBD_MSC_HOTPLUG
exit_create_check_task_fail:
	usbd_msc_deinit();
#endif
exit_usbd_msc_init_fail:
	usbd_deinit();
exit_usbd_init_fail:
	usbd_msc_disk_deinit();
exit_disk_init_fail:
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD MSC demo stop\n");
#if CONFIG_USBD_MSC_HOTPLUG
	vSemaphoreDelete(msc_attach_status_changed_sema);
#endif
	vTaskDelete(NULL);
}

/* Exported functions --------------------------------------------------------*/

void example_usbd_msc_new(void)
{
	if (xTaskCreate(example_usbd_msc_new_thread, "usbd_msc_new_init", MSC_INIT_THREAD_STACK_SIZE,
					NULL, MSC_INIT_THREAD_PRIORITY, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create USBD MSC thread fail\n");
	}
}
