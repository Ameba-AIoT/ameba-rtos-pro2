/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Uses the selected ported stack's composite CDC ACM + MSC API.
 * component/os/os_wrapper (rtos_task_t/rtos_sema_t/rtos_*) does not exist in
 * this SDK generation, so task/semaphore handling below calls FreeRTOS
 * directly, the same way component/example/usbd_cdc_acm_new already does.
 *
 * MSC storage uses the configured RAM-disk or SD-card backend. SD-card
 * hotplug and second-flash handling are omitted because their supporting
 * headers are unavailable in this SDK.
 *
 * The usb_hal_driver / FIFO depth values below are configured for the
 * amebapro2 (rtl8735b) USB OTG controller.
 */

/* Includes ------------------------------------------------------------------ */

#include "usbd_composite_cdc_acm_msc.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private defines -----------------------------------------------------------*/

#define COMP_BULK_IN_XFER_SIZE                         2048U
#define COMP_BULK_OUT_XFER_SIZE                         2048U

#define COMP_INIT_THREAD_STACK_SIZE                     1024U
#define COMP_INIT_THREAD_PRIORITY                       (tskIDLE_PRIORITY + 5)

/* Private function prototypes -----------------------------------------------*/

static int composite_cdc_acm_cb_init(void);
static int composite_cdc_acm_cb_deinit(void);
static int composite_cdc_acm_cb_setup(usb_setup_req_t *req, u8 *buf);
static int composite_cdc_acm_cb_received(u8 *buf, u32 len);
static void composite_cdc_acm_cb_transmitted(u8 status);

static void composite_cb_status_changed(u8 old_status, u8 status);

/* Private variables ---------------------------------------------------------*/

static const char *const TAG = "COMP";

static const usbd_config_t composite_cfg = {
	.speed = USB_SPEED_HIGH,
	/* 4U matches component/example/usbd_cdc_acm_new's validated value. */
	.isr_priority = 4U,
	/* DFIFO total 2232 DWORD, resv 8 DWORD for DMA addr and EP0 fixed 256 DWORD. */
	.rx_fifo_depth = 1424U,
	.ptx_fifo_depth = {256U, 32U, 256U},
};

static const usbd_composite_cdc_acm_usr_cb_t composite_cdc_acm_usr_cb = {
	.init = composite_cdc_acm_cb_init,
	.deinit = composite_cdc_acm_cb_deinit,
	.setup = composite_cdc_acm_cb_setup,
	.received = composite_cdc_acm_cb_received,
	.transmitted = composite_cdc_acm_cb_transmitted,
};

static usb_cdc_line_coding_t composite_cdc_acm_line_coding;

static const usbd_composite_cb_t composite_cb = {
	.status_changed = composite_cb_status_changed,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media layer
  * @retval Status
  */
static int composite_cdc_acm_cb_init(void)
{
	usb_cdc_line_coding_t *lc = &composite_cdc_acm_line_coding;

	lc->b.dwDteRate = 150000;
	lc->b.bCharFormat = 0x00;
	lc->b.bParityType = 0x00;
	lc->b.bDataBits = 0x08;

	return HAL_OK;
}

/**
  * @brief  De-initializes the CDC media layer
  * @retval Status
  */
static int composite_cdc_acm_cb_deinit(void)
{
	return HAL_OK;
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static int composite_cdc_acm_cb_received(u8 *buf, u32 len)
{
	/* CDC ACM loopback test */
	return usbd_composite_cdc_acm_transmit(buf, len);
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static void composite_cdc_acm_cb_transmitted(u8 status)
{
	UNUSED(status);
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static int composite_cdc_acm_cb_setup(usb_setup_req_t *req, u8 *buf)
{
	int ret = HAL_OK;
	usb_cdc_line_coding_t *lc = &composite_cdc_acm_line_coding;
	u16 ctrl_line_state;

	switch (req->bRequest) {
	case USB_CDC_ACM_SET_LINE_CODING:
		if (req->wLength == USB_CDC_ACM_LINE_CODING_SIZE) {
			lc->b.dwDteRate = (u32)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
			lc->b.bCharFormat = buf[4];
			lc->b.bParityType = buf[5];
			lc->b.bDataBits = buf[6];
		} else {
			USB_DIAG(USB_LAYER_APP, USB_EVT_ERR_SETUP, 0);
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
		/*
		wValue: Control Signal Bitmap
			D2-15: Reserved, 0
			D1:    RTS, 0 - Deactivate, 1 - Activate
			D0:    DTR, 0 - Not Present, 1 - Present
		*/
		ctrl_line_state = req->wValue;
		if (ctrl_line_state & 0x01) {
			RTK_LOGS(TAG, RTK_LOG_INFO, "VCOM port activated\n");
		}
		break;

	case USB_CDC_ACM_SEND_ENCAPSULATED_COMMAND:
	case USB_CDC_ACM_GET_ENCAPSULATED_RESPONSE:
	case USB_CDC_ACM_SET_COMM_FEATURE:
	case USB_CDC_ACM_GET_COMM_FEATURE:
	case USB_CDC_ACM_CLEAR_COMM_FEATURE:
	case USB_CDC_ACM_SEND_BREAK:
		/* Do nothing */
		break;

	default:
		USB_DIAG(USB_LAYER_APP, USB_EVT_ERR_SETUP, 1);
		ret = HAL_ERR_PARA;
		break;
	}

	return ret;
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static void composite_cb_status_changed(u8 old_status, u8 status)
{
	UNUSED(old_status);

	if (status == USBD_ATTACH_STATUS_DETACHED) {
		RTK_LOGS(TAG, RTK_LOG_INFO, "DETACHED\n");
	} else if (status == USBD_ATTACH_STATUS_ATTACHED) {
		RTK_LOGS(TAG, RTK_LOG_INFO, "ATTACHED\n");
	} else {
		RTK_LOGS(TAG, RTK_LOG_INFO, "INIT\n");
	}
}

static void example_usbd_composite_cdc_acm_msc_new_thread(void *param)
{
	int ret;

	UNUSED(param);

	ret = usbd_composite_msc_disk_init();
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Init disk fail\n");
		goto exit_disk_init_fail;
	}

	ret = usbd_init(&composite_cfg);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_init fail\n");
		goto exit_usbd_init_fail;
	}

	ret = usbd_composite_init(COMP_BULK_OUT_XFER_SIZE, COMP_BULK_IN_XFER_SIZE, &composite_cdc_acm_usr_cb, &composite_cb);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_composite_init fail\n");
		goto exit_composite_init_fail;
	}

	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD COMP demo start\n");
	vTaskDelete(NULL);
	return;

exit_composite_init_fail:
	usbd_deinit();
exit_usbd_init_fail:
	usbd_composite_msc_disk_deinit();
exit_disk_init_fail:
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD COMP demo stop\n");
	vTaskDelete(NULL);
}

/* Exported functions --------------------------------------------------------*/

void example_usbd_composite_cdc_acm_msc_new(void)
{
	if (xTaskCreate(example_usbd_composite_cdc_acm_msc_new_thread, "usbd_composite_cdc_acm_msc_new_init", COMP_INIT_THREAD_STACK_SIZE,
					NULL, COMP_INIT_THREAD_PRIORITY, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create USBD COMP thread fail\n");
	}
}
