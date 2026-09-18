/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Composite MSC + boot-mouse HID demo, mirroring the structure of
 * component/example/usbd_composite_cdc_acm_msc_new's
 * example_usbd_composite_cdc_acm_msc_new.c (task/semaphore handling via
 * FreeRTOS directly, no component/os/os_wrapper) with the CDC ACM leg
 * swapped for HID and calling this port's
 * component/usb/usb_stack/<version>/device/composite/usbd_composite_msc_hid.h dispatcher
 * instead of usbd_composite_cdc_acm_msc.h.
 *
 * MSC still uses the RAM disk backend (usbd_composite_msc_disk_init/deinit,
 * same as the CDC ACM + MSC example) - the actual mass-storage interface
 * needs a live disk_ops backend to answer SCSI READ/WRITE, so disk_init is
 * kept despite HID itself having no storage of its own.
 *
 * DFIFO total 2232 DWORD, resv 8 DWORD for DMA addr and EP0 fixed 256 DWORD:
 * rx_fifo_depth 1680 + ptx_fifo_depth (32 + 256) + 256 (EP0) + 8 (DMA) = 2232.
 * ptx_fifo_depth[0]=32 backs TxFIFO1 (HID interrupt IN, EP1, 4-byte mouse
 * report), ptx_fifo_depth[1]=256 backs TxFIFO2 (MSC bulk IN, EP3, matching
 * the CDC ACM + MSC example's MSC allotment).
 *
 * The MOUSE console command packs the boot-mouse report and registers it
 * through this project's log_service_add_table()/log_item_t mechanism (see
 * component/example/usb_mass_storage's atcmd_usb_msc_init() for the command
 * registration pattern used in this project).
 */

/* Includes ------------------------------------------------------------------ */

#include "usbd_composite_msc_hid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log_service.h"
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/

#define COMP_HID_TX_BUF_LEN                            COMP_HID_INTR_IN_PACKET_SIZE

#define COMP_INIT_THREAD_STACK_SIZE                     1024U
#define COMP_INIT_THREAD_PRIORITY                       (tskIDLE_PRIORITY + 5)

/* Private function prototypes -----------------------------------------------*/

static int composite_hid_cb_init(void);
static void composite_hid_cb_deinit(void);
static int composite_hid_cb_setup(usb_setup_req_t *req, u8 *buf);
static void composite_hid_cb_transmitted(u8 status);

static void composite_cb_status_changed(u8 old_status, u8 status);

/* Private variables ---------------------------------------------------------*/

static const char *const TAG = "COMP";

static const usbd_config_t composite_cfg = {
	.speed = USB_SPEED_HIGH,
	/* 4U matches component/example/usbd_cdc_acm_new's validated value. */
	.isr_priority = 4U,
	/* DFIFO total 2232 DWORD, resv 8 DWORD for DMA addr and EP0 fixed 256 DWORD.
	 * rx_fifo_depth is given all the remaining budget: 1968 - 32 - 256 = 1680. */
	.rx_fifo_depth = 1680U,
	.ptx_fifo_depth = {32U, 256U},
};

static const usbd_composite_hid_usr_cb_t composite_hid_usr_cb = {
	.init = composite_hid_cb_init,
	.deinit = composite_hid_cb_deinit,
	.setup = composite_hid_cb_setup,
	.transmitted = composite_hid_cb_transmitted,
};

static const usbd_composite_cb_t composite_cb = {
	.status_changed = composite_cb_status_changed,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the HID media layer
  * @retval Status
  */
static int composite_hid_cb_init(void)
{
	return HAL_OK;
}

/**
  * @brief  De-initializes the HID media layer
  * @retval Status
  */
static void composite_hid_cb_deinit(void)
{
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  * @brief  Minimal demo handling of the standard HID class requests -
  *         GET_REPORT/GET_IDLE/GET_PROTOCOL just answer with zeroed data of
  *         the requested length (this demo doesn't track idle rate/protocol
  *         state), and SET_REPORT/SET_IDLE/SET_PROTOCOL are accepted and
  *         ignored; actual mouse movement only goes out via the interrupt-IN
  *         reports sent from the MOUSE console command below.
  */
static int composite_hid_cb_setup(usb_setup_req_t *req, u8 *buf)
{
	int ret = HAL_OK;

	switch (req->bRequest) {
	case COMP_HID_GET_REPORT:
	case COMP_HID_GET_IDLE:
	case COMP_HID_GET_PROTOCOL:
		if (buf != NULL) {
			usb_os_memset((void *)buf, 0x00, req->wLength);
		}
		break;

	case COMP_HID_SET_REPORT:
	case COMP_HID_SET_IDLE:
	case COMP_HID_SET_PROTOCOL:
		/* Demo: accept but ignore host-provided report/idle/protocol data */
		break;

	default:
		USB_DIAG(USB_LAYER_APP, USB_EVT_ERR_SETUP, 0);
		ret = HAL_ERR_PARA;
		break;
	}

	return ret;
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static void composite_hid_cb_transmitted(u8 status)
{
	UNUSED(status);
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

/**
  * @brief  Console command to drive the boot-mouse HID interrupt-IN report:
  *         "MOUSE=<left>[,<right>,<middle>,<x_axis>,<y_axis>,<wheel>]"
  *         left/right/middle: 0=release, non-zero=press.
  *         x_axis/y_axis/wheel: signed, -127..127 (see
  *         composite_hid_mouse_report_desc in usbd_composite_hid.c for the
  *         report field layout this packs into byte[0..3]).
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
		report[0] |= COMP_HID_MOUSE_BUTTON_LEFT;
	}
	if ((argc > 2) && (strtoul(argv[2], NULL, 10) != 0)) {
		report[0] |= COMP_HID_MOUSE_BUTTON_RIGHT;
	}
	if ((argc > 3) && (strtoul(argv[3], NULL, 10) != 0)) {
		report[0] |= COMP_HID_MOUSE_BUTTON_MIDDLE;
	}
	report[0] |= COMP_HID_MOUSE_BUTTON_RESERVED;
	if (argc > 4) {
		report[1] = (u8)strtol(argv[4], NULL, 10);
	}
	if (argc > 5) {
		report[2] = (u8)strtol(argv[5], NULL, 10);
	}
	if (argc > 6) {
		report[3] = (u8)strtol(argv[6], NULL, 10);
	}

	ret = usbd_composite_hid_send_data(report, sizeof(report));
	RTK_LOGS(TAG, RTK_LOG_INFO, "MOUSE: buttons=%02x x=%d y=%d wheel=%d, ret=%d\n",
			 report[0], (s8)report[1], (s8)report[2], (s8)report[3], ret);
}

static log_item_t composite_hid_cmd_items[] = {
	{"MOUSE", MOUSE,},
};

static void example_usbd_composite_msc_hid_new_thread(void *param)
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

	ret = usbd_composite_msc_hid_init(COMP_HID_TX_BUF_LEN, &composite_hid_usr_cb, &composite_cb);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_composite_msc_hid_init fail\n");
		goto exit_composite_init_fail;
	}

	log_service_add_table(composite_hid_cmd_items, sizeof(composite_hid_cmd_items) / sizeof(composite_hid_cmd_items[0]));

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

void example_usbd_composite_msc_hid_new(void)
{
	if (xTaskCreate(example_usbd_composite_msc_hid_new_thread, "usbd_composite_msc_hid_new_init", COMP_INIT_THREAD_STACK_SIZE,
					NULL, COMP_INIT_THREAD_PRIORITY, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create USBD COMP thread fail\n");
	}
}
