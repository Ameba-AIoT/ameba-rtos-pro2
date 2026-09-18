/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * component/os/os_wrapper (rtos_task_t/rtos_sema_t/rtos_*) does not exist in
 * this SDK generation, so task/semaphore handling below calls FreeRTOS
 * directly. The USB driver itself still goes through usb_os_*
 * (component/usb/usb_stack/<version>/common/usb_os.c) for its FreeRTOS access.
 *
 * The usb_hal_driver / FIFO depth values below are configured for the
 * amebapro2 (rtl8735b) USB OTG controller.
 */

/* Includes ------------------------------------------------------------------ */

#include "usbd_cdc_acm.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private defines -----------------------------------------------------------*/

/* Re-init the USB device stack on hot-plug (detach/attach), matching the
 * upstream example - avoids leaking the previous session's driver state. */
#define CONFIG_USBD_CDC_ACM_HOTPLUG                    1

#define CDC_ACM_BULK_OUT_XFER_SIZE                     2048U
#define CDC_ACM_BULK_IN_XFER_SIZE                      2048U

#define CDC_ACM_INIT_THREAD_STACK_SIZE                 1024U
#define CDC_ACM_INIT_THREAD_PRIORITY                   (tskIDLE_PRIORITY + 5)
#define CDC_ACM_HOTPLUG_THREAD_STACK_SIZE              1024U
#define CDC_ACM_HOTPLUG_THREAD_PRIORITY                (tskIDLE_PRIORITY + 6)

/* Private function prototypes -----------------------------------------------*/

static int cdc_acm_cb_init(void);
static int cdc_acm_cb_deinit(void);
static int cdc_acm_cb_setup(usb_setup_req_t *req, u8 *buf);
static int cdc_acm_cb_received(u8 *buf, u32 len);
static void cdc_acm_cb_status_changed(u8 old_status, u8 status);

/* Private variables ---------------------------------------------------------*/

static const char *const TAG = "ACM";

static const usbd_cdc_acm_cb_t cdc_acm_cb = {
	.init = cdc_acm_cb_init,
	.deinit = cdc_acm_cb_deinit,
	.setup = cdc_acm_cb_setup,
	.received = cdc_acm_cb_received,
	.status_changed = cdc_acm_cb_status_changed,
};

static usb_cdc_line_coding_t cdc_acm_line_coding;
static u16 cdc_acm_ctrl_line_state;

static const usbd_config_t cdc_acm_cfg = {
	.speed = USB_SPEED_HIGH,
	/* 4U is the validated interrupt priority for this rtl8735b example. */
	.isr_priority = 4U,
	/* DFIFO total 2232 DWORD, resv 8 DWORD for DMA addr and EP0 fixed 256 DWORD. */
	.rx_fifo_depth = 1664U,
	.ptx_fifo_depth = {256U, 32U, 16U},
};

#if CONFIG_USBD_CDC_ACM_HOTPLUG
static u8 cdc_acm_attach_status;
static SemaphoreHandle_t cdc_acm_attach_status_changed_sema;
#endif

/* Private functions ---------------------------------------------------------*/

static int cdc_acm_cb_init(void)
{
	usb_cdc_line_coding_t *lc = &cdc_acm_line_coding;

	lc->b.dwDteRate = 150000;
	lc->b.bCharFormat = 0x00;
	lc->b.bParityType = 0x00;
	lc->b.bDataBits = 0x08;

	return HAL_OK;
}

static int cdc_acm_cb_deinit(void)
{
	return HAL_OK;
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static int cdc_acm_cb_received(u8 *buf, u32 len)
{
	/* Simple loopback: echo whatever the host sent straight back. */
	return usbd_cdc_acm_transmit(buf, len);
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static int cdc_acm_cb_setup(usb_setup_req_t *req, u8 *buf)
{
	usb_cdc_line_coding_t *lc = &cdc_acm_line_coding;

	switch (req->bRequest) {
	case USB_CDC_ACM_SET_LINE_CODING:
		if (req->wLength == USB_CDC_ACM_LINE_CODING_SIZE) {
			lc->b.dwDteRate = (u32)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
			lc->b.bCharFormat = buf[4];
			lc->b.bParityType = buf[5];
			lc->b.bDataBits = buf[6];
		}
		break;

	case USB_CDC_ACM_GET_LINE_CODING:
		buf[0] = (u8)(lc->b.dwDteRate & 0xFF);
		buf[1] = (u8)((lc->b.dwDteRate >> 8) & 0xFF);
		buf[2] = (u8)((lc->b.dwDteRate >> 16) & 0xFF);
		buf[3] = (u8)((lc->b.dwDteRate >> 24) & 0xFF);
		buf[4] = lc->b.bCharFormat;
		buf[5] = lc->b.bParityType;
		buf[6] = lc->b.bDataBits;
		break;

	case USB_CDC_ACM_SET_CONTROL_LINE_STATE:
		cdc_acm_ctrl_line_state = req->wValue;
		if (cdc_acm_ctrl_line_state & 0x01) {
			RTK_LOGS(TAG, RTK_LOG_INFO, "VCOM port activated\n");
		}
		break;

	default:
		break;
	}

	return HAL_OK;
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static void cdc_acm_cb_status_changed(u8 old_status, u8 status)
{
	UNUSED(old_status);
#if CONFIG_USBD_CDC_ACM_HOTPLUG
	BaseType_t woken = pdFALSE;

	cdc_acm_attach_status = status;
	xSemaphoreGiveFromISR(cdc_acm_attach_status_changed_sema, &woken);
	portYIELD_FROM_ISR(woken);
#else
	UNUSED(status);
#endif
}

#if CONFIG_USBD_CDC_ACM_HOTPLUG
static void cdc_acm_hotplug_thread(void *param)
{
	int ret;

	UNUSED(param);

	for (;;) {
		if (xSemaphoreTake(cdc_acm_attach_status_changed_sema, portMAX_DELAY) == pdTRUE) {
			if (cdc_acm_attach_status == USBD_ATTACH_STATUS_DETACHED) {
				RTK_LOGS(TAG, RTK_LOG_INFO, "DETACHED\n");
				usbd_cdc_acm_deinit();
				ret = usbd_deinit();
				if (ret != HAL_OK) {
					break;
				}
				ret = usbd_init(&cdc_acm_cfg);
				if (ret != HAL_OK) {
					break;
				}
				ret = usbd_cdc_acm_init(CDC_ACM_BULK_OUT_XFER_SIZE, CDC_ACM_BULK_IN_XFER_SIZE, &cdc_acm_cb);
				if (ret != HAL_OK) {
					usbd_deinit();
					break;
				}
			} else if (cdc_acm_attach_status == USBD_ATTACH_STATUS_ATTACHED) {
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

static void example_usbd_cdc_acm_new_thread(void *param)
{
	int ret;
#if CONFIG_USBD_CDC_ACM_HOTPLUG
	TaskHandle_t check_task;
#endif

	UNUSED(param);

#if CONFIG_USBD_CDC_ACM_HOTPLUG
	cdc_acm_attach_status_changed_sema = xSemaphoreCreateBinary();
#endif

	ret = usbd_init(&cdc_acm_cfg);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_init fail\n");
		goto exit_usbd_init_fail;
	}

	ret = usbd_cdc_acm_init(CDC_ACM_BULK_OUT_XFER_SIZE, CDC_ACM_BULK_IN_XFER_SIZE, &cdc_acm_cb);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_cdc_acm_init fail\n");
		goto exit_usbd_cdc_acm_init_fail;
	}

#if CONFIG_USBD_CDC_ACM_HOTPLUG
	if (xTaskCreate(cdc_acm_hotplug_thread, "cdc_acm_hotplug", CDC_ACM_HOTPLUG_THREAD_STACK_SIZE, NULL,
					CDC_ACM_HOTPLUG_THREAD_PRIORITY, &check_task) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create hotplug thread fail\n");
		goto exit_create_check_task_fail;
	}
#endif

	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD CDC ACM demo start\n");
	vTaskDelete(NULL);
	return;

#if CONFIG_USBD_CDC_ACM_HOTPLUG
exit_create_check_task_fail:
	usbd_cdc_acm_deinit();
#endif
exit_usbd_cdc_acm_init_fail:
	usbd_deinit();
exit_usbd_init_fail:
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD CDC ACM demo stop\n");
#if CONFIG_USBD_CDC_ACM_HOTPLUG
	vSemaphoreDelete(cdc_acm_attach_status_changed_sema);
#endif
	vTaskDelete(NULL);
}

/* Exported functions --------------------------------------------------------*/

void example_usbd_cdc_acm_new(void)
{
	if (xTaskCreate(example_usbd_cdc_acm_new_thread, "usbd_cdc_acm_new_init", CDC_ACM_INIT_THREAD_STACK_SIZE,
					NULL, CDC_ACM_INIT_THREAD_PRIORITY, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create USBD CDC ACM thread fail\n");
	}
}
