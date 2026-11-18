/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * USB DFU 1.1 device class for the usb_stack/v1.0.0 device core (AmebaPro2).
 *
 * Ported from the AmebaPro3 usbd_dfu class. Differences against it, all forced by this core/SDK:
 * - get_descriptor() has no buffer length and the core has no string table (usbd_add_string), so
 *   the class answers its own string descriptors and patches VID/PID from usbd_dfu_config_t.
 * - No Kconfig: the bmAttributes feature switches are compile definitions, see usbd_dfu.h.
 * - No RTOS timers: the manifest "status poll timeout" is driven by the host GETSTATUS polls, and
 *   the non-WillDetach detach timeout is a timed wait in the reconf task.
 * - cb->manifest() returns a result, so a failed verification reaches the host as errVERIFY.
 * - Worker tasks get a secure context under TrustZone, since the callbacks program flash.
 *
 * DFU is EP0-only: DNLOAD/UPLOAD blocks are moved by temporarily pointing the core's EP0 buffer at
 * the class block buffer, and restoring it on completion or on bus reset.
 */

/* Includes ------------------------------------------------------------------*/

#include "usbd_dfu.h"

/* Private defines -----------------------------------------------------------*/

#define USBD_DFU_ITF_NUM                0x00U   /* The only interface of this device */
#define USBD_DFU_STATE_PKT_SIZE         1U      /* wLength of GETSTATE, DFU 1.1 6.1.5 */

/* Values of write_state */
#define USBD_DFU_WRITE_BUSY             0U
#define USBD_DFU_WRITE_OK               1U
#define USBD_DFU_WRITE_FAIL             2U

#define USBD_DFU_STOP_TASK_WARN_MS      1000U

/* Private function prototypes -----------------------------------------------*/

static int usbd_dfu_set_config(usb_dev_t *dev, u8 config);
static int usbd_dfu_clear_config(usb_dev_t *dev, u8 config);
static int usbd_dfu_setup(usb_dev_t *dev, usb_setup_req_t *req);
static u16 usbd_dfu_get_descriptor(usb_dev_t *dev, usb_setup_req_t *req, u8 *buf);
static int usbd_dfu_ep0_data_in(usb_dev_t *dev, u8 status);
static int usbd_dfu_ep0_data_out(usb_dev_t *dev);
static void usbd_dfu_status_changed(usb_dev_t *dev, u8 old_status, u8 status);

/* Private variables ---------------------------------------------------------*/

static const char *const TAG = "DFU";

/* USB Standard Device Descriptor, idVendor/idProduct patched from usbd_dfu_config_t */
static const u8 usbd_dfu_dev_desc[USB_LEN_DEV_DESC] = {
	USB_LEN_DEV_DESC,                               /* bLength */
	USB_DESC_TYPE_DEVICE,                           /* bDescriptorType */
	0x00, 0x02,                                     /* bcdUSB 2.00 */
	0x00,                                           /* bDeviceClass: at interface level */
	0x00,                                           /* bDeviceSubClass */
	0x00,                                           /* bDeviceProtocol */
	USB_MAX_EP0_SIZE,                               /* bMaxPacketSize0 */
	USB_LOW_BYTE(USBD_DFU_DEFAULT_VID),             /* idVendor */
	USB_HIGH_BYTE(USBD_DFU_DEFAULT_VID),
	USB_LOW_BYTE(USBD_DFU_DEFAULT_PID),             /* idProduct */
	USB_HIGH_BYTE(USBD_DFU_DEFAULT_PID),
	0x00, 0x02,                                     /* bcdDevice 2.00 */
	USBD_DFU_STRING_MANUFACTURER,                   /* iManufacturer */
	USBD_DFU_STRING_PRODUCT,                        /* iProduct */
	USBD_DFU_STRING_SERIALNUMBER,                   /* iSerialNumber */
	0x01,                                           /* bNumConfigurations */
};

/* USB Standard String Descriptor 0 */
static const u8 usbd_dfu_lang_id_desc[USB_LEN_LANGID_STR_DESC] = {
	USB_LEN_LANGID_STR_DESC,
	USB_DESC_TYPE_STRING,
	USB_LOW_BYTE(USBD_DFU_LANGID_STRING),
	USB_HIGH_BYTE(USBD_DFU_LANGID_STRING),
};

#ifndef CONFIG_USB_FS
/* USB Standard Device Qualifier Descriptor */
static const u8 usbd_dfu_device_qualifier_desc[USB_LEN_DEV_QUALIFIER_DESC] = {
	USB_LEN_DEV_QUALIFIER_DESC,                     /* bLength */
	USB_DESC_TYPE_DEVICE_QUALIFIER,                 /* bDescriptorType */
	0x00, 0x02,                                     /* bcdUSB 2.00 */
	0x00,                                           /* bDeviceClass */
	0x00,                                           /* bDeviceSubClass */
	0x00,                                           /* bDeviceProtocol */
	USB_MAX_EP0_SIZE,                               /* bMaxPacketSize0 */
	0x01,                                           /* bNumConfigurations */
	0x00,                                           /* Reserved */
};
#endif

/* Configuration Descriptor: Config(9) + Interface(9) + DFU Functional(9), no endpoints */
static const u8 usbd_dfu_config_desc[] = {
	/* Configuration Descriptor */
	USB_LEN_CFG_DESC,                               /* bLength */
	USB_DESC_TYPE_CONFIGURATION,                    /* bDescriptorType */
	0x00, 0x00,                                     /* wTotalLength, patched */
	0x01,                                           /* bNumInterfaces */
	0x01,                                           /* bConfigurationValue */
	0x00,                                           /* iConfiguration */
	0x80,                                           /* bmAttributes: bus powered */
	0x32,                                           /* bMaxPower: 100 mA */

	/* Interface Descriptor */
	USB_LEN_IF_DESC,                                /* bLength */
	USB_DESC_TYPE_INTERFACE,                        /* bDescriptorType */
	USBD_DFU_ITF_NUM,                               /* bInterfaceNumber */
	0x00,                                           /* bAlternateSetting */
	0x00,                                           /* bNumEndpoints: EP0 only */
	USB_DFU_CLASS_CODE,                             /* bInterfaceClass: 0xFE */
	USB_DFU_SUBCLASS_CODE,                          /* bInterfaceSubClass: 0x01 */
	USB_DFU_PROTOCOL_DFU,                           /* bInterfaceProtocol, patched to 0x01 in Run-Time mode */
	USBD_DFU_STRING_INTERFACE,                      /* iInterface */

	/* DFU Functional Descriptor */
	USB_DFU_FUNC_DESC_SIZE,                         /* bLength */
	USB_DFU_FUNC_DESC_TYPE,                         /* bDescriptorType: 0x21 */
#if CONFIG_USBD_DFU_WILL_DETACH
	USB_DFU_ATTR_WILL_DETACH |
#endif
#if CONFIG_USBD_DFU_MANIFESTATION_TOLERANT
	USB_DFU_ATTR_MANIFEST_TOL |
#endif
#if CONFIG_USBD_DFU_CAN_UPLOAD
	USB_DFU_ATTR_CAN_UPLOAD |
#endif
	USB_DFU_ATTR_CAN_DNLOAD,                        /* bmAttributes */
	USB_LOW_BYTE(USBD_DFU_DEFAULT_DETACH_TIMEOUT),  /* wDetachTimeOut, patched */
	USB_HIGH_BYTE(USBD_DFU_DEFAULT_DETACH_TIMEOUT),
	USB_LOW_BYTE(USBD_DFU_DEFAULT_XFER_SIZE),       /* wTransferSize, patched */
	USB_HIGH_BYTE(USBD_DFU_DEFAULT_XFER_SIZE),
	USB_LOW_BYTE(USB_DFU_BCD_VERSION),              /* bcdDFUVersion */
	USB_HIGH_BYTE(USB_DFU_BCD_VERSION),
};

static const usbd_class_driver_t usbd_dfu_driver = {
	.get_descriptor = usbd_dfu_get_descriptor,
	.set_config     = usbd_dfu_set_config,
	.clear_config   = usbd_dfu_clear_config,
	.setup          = usbd_dfu_setup,
	.ep0_data_in    = usbd_dfu_ep0_data_in,
	.ep0_data_out   = usbd_dfu_ep0_data_out,
	.status_changed = usbd_dfu_status_changed,
};

static usbd_dfu_dev_t usbd_dfu_dev;

/* Private functions ---------------------------------------------------------*/

/* Point both EP0 directions back to the core buffer if they still use the class block buffer */
static void usbd_dfu_restore_ep0(usb_dev_t *dev, usbd_dfu_dev_t *dfu)
{
	if ((dev == NULL) || (dfu->xfer_buf == NULL) || (dfu->ep0_default_buf == NULL)) {
		return;
	}
	if (dev->ep0_in.xfer_buf == dfu->xfer_buf) {
		dev->ep0_in.xfer_buf     = dfu->ep0_default_buf;
		dev->ep0_in.xfer_buf_len = dfu->ep0_default_buf_len;
	}
	if (dev->ep0_out.xfer_buf == dfu->xfer_buf) {
		dev->ep0_out.xfer_buf     = dfu->ep0_default_buf;
		dev->ep0_out.xfer_buf_len = dfu->ep0_default_buf_len;
	}
}

static inline u8 usbd_dfu_idle_state(const usbd_dfu_dev_t *dfu)
{
	return (dfu->mode == USB_DFU_PROTOCOL_RUNTIME) ? USB_DFU_STATE_APP_IDLE : USB_DFU_STATE_DFU_IDLE;
}

/**
 * @brief  SET_CONFIGURATION (ISR context).
 */
static int usbd_dfu_set_config(usb_dev_t *dev, u8 config)
{
	usbd_dfu_dev_t *dfu = &usbd_dfu_dev;

	if (config != 1U) {
		return HAL_ERR_PARA;
	}

	dfu->dev = dev;
	dfu->alt_setting = 0U;
	/* Save the core EP0 buffer (shared by IN/OUT), unless it is still redirected */
	if (dev->ep0_in.xfer_buf != dfu->xfer_buf) {
		dfu->ep0_default_buf     = dev->ep0_in.xfer_buf;
		dfu->ep0_default_buf_len = dev->ep0_out.xfer_buf_len;
	}
	dfu->manifest_done = 0U;
	dfu->state  = (usb_dfu_state_t)usbd_dfu_idle_state(dfu);
	dfu->status = USBD_DFU_STATUS_OK;

	return HAL_OK;
}

/**
 * @brief  Configuration cleared on bus reset, disconnect or deinit (ISR context).
 */
static int usbd_dfu_clear_config(usb_dev_t *dev, u8 config)
{
	usbd_dfu_dev_t *dfu = &usbd_dfu_dev;

	UNUSED(config);

	/* USB 2.0 9.1.1.5: a bus reset voids the control transfer in progress, so the EP0 completion
	 * that would restore a redirected buffer never arrives. */
	usbd_dfu_restore_ep0(dev, dfu);
#if CONFIG_USBD_DFU_CAN_UPLOAD
	dfu->upload_last = 0U;
#endif
	dfu->ep0_in_owned = 0U;

	/* During the intentional detach the mode is already DFU, so the re-enumeration comes up in
	 * dfuIDLE; a plain unplug in Run-Time mode stays in appIDLE. */
	dfu->state = (usb_dfu_state_t)usbd_dfu_idle_state(dfu);

	return HAL_OK;
}

/**
 * @brief  Send the 6-byte GETSTATUS response (ISR context).
 */
static int usbd_dfu_send_status(usb_dev_t *dev, const usbd_dfu_dev_t *dfu, u32 poll_ms)
{
	usbd_ep_t *ep0_in = &dev->ep0_in;

	ep0_in->xfer_buf[0] = (u8)dfu->status;
	ep0_in->xfer_buf[1] = (u8)(poll_ms & 0xFFU);
	ep0_in->xfer_buf[2] = (u8)((poll_ms >> 8U) & 0xFFU);
	ep0_in->xfer_buf[3] = (u8)((poll_ms >> 16U) & 0xFFU);
	ep0_in->xfer_buf[4] = (u8)dfu->state;
	ep0_in->xfer_buf[5] = 0x00U;  /* iString */
	ep0_in->xfer_len    = USB_DFU_STATUS_PKT_SIZE;
	return usbd_ep_transmit(dev, ep0_in);
}

/* bwPollTimeout while write_task programs the block: at least 1 ms so the task can post its result */
static u32 usbd_dfu_get_busy_poll_ms(const usbd_dfu_dev_t *dfu)
{
	u32 pm = (dfu->cb->get_write_poll_ms != NULL) ? dfu->cb->get_write_poll_ms(dfu->block_num) : dfu->write_poll_ms;

	return (pm == 0U) ? 1U : pm;
}

/**
 * @brief  Leave dfuMANIFEST once manifest_task has finished (ISR context).
 * @retval 1 if the state advanced.
 */
static u8 usbd_dfu_manifest_advance(usbd_dfu_dev_t *dfu)
{
	if ((dfu->state != USB_DFU_STATE_MANIFEST) || (dfu->manifest_done == 0U)) {
		return 0U;
	}

	if (dfu->manifest_ret != HAL_OK) {
		/* DFU 1.1 6.1.2: a failed verification is reported as errVERIFY in dfuERROR */
		dfu->manifest_done = 0U;
		dfu->state  = USB_DFU_STATE_ERROR;
		dfu->status = USBD_DFU_STATUS_ERR_VERIFY;
		return 1U;
	}

#if CONFIG_USBD_DFU_MANIFESTATION_TOLERANT
	/* manifest_done stays set, the next GETSTATUS in dfuMANIFEST-SYNC moves to dfuIDLE */
	dfu->state = USB_DFU_STATE_MANIFEST_SYNC;
#else
	dfu->manifest_done = 0U;
	dfu->state = USB_DFU_STATE_MANIFEST_WAIT_RESET;
#if CONFIG_USBD_DFU_WILL_DETACH
	/* DFU 1.1 §7: the device generates the detach-attach itself, back in Run-Time mode */
	dfu->mode = USB_DFU_PROTOCOL_RUNTIME;
	dfu->reconf_pending = 1U;
	usb_os_sema_give(dfu->reconf_sema);
#endif
#endif
	dfu->status = USBD_DFU_STATUS_OK;

	return 1U;
}

/**
 * @brief  Apply the block result posted by write_task (ISR context).
 * @retval 1 if the block has finished, 0 if still busy.
 */
static u8 usbd_dfu_apply_write_state(usbd_dfu_dev_t *dfu)
{
	u8 result = dfu->write_state;

	if (result == USBD_DFU_WRITE_BUSY) {
		return 0U;
	}

	dfu->write_state = USBD_DFU_WRITE_BUSY;
	if (result == USBD_DFU_WRITE_OK) {
		dfu->state = USB_DFU_STATE_DNLOAD_IDLE;
	} else {
		dfu->state  = USB_DFU_STATE_ERROR;
		dfu->status = USBD_DFU_STATUS_ERR_WRITE;
	}

	return 1U;
}

/* Enter dfuERROR; the non-OK return makes the core stall EP0 */
static int usbd_dfu_stall_error(usbd_dfu_dev_t *dfu, usbd_dfu_status_t err)
{
	dfu->state  = USB_DFU_STATE_ERROR;
	dfu->status = err;
	return HAL_ERR_HW;
}

/**
 * @brief  Validate direction and wLength of a DFU class request (DFU 1.1 Table 3.2).
 *         Requests without a data stage must have wLength 0, otherwise the core would not send the
 *         status stage and the control transfer would hang.
 */
static u8 usbd_dfu_req_is_valid(const usb_setup_req_t *req)
{
	u8 dir = req->bmRequestType & USB_REQ_DIR_MASK;

	switch (req->bRequest) {
	case USB_DFU_REQ_DETACH:
	case USB_DFU_REQ_CLRSTATUS:
	case USB_DFU_REQ_ABORT:
		return ((dir == USB_H2D) && (req->wLength == 0U)) ? 1U : 0U;
	case USB_DFU_REQ_DNLOAD:
		return (dir == USB_H2D) ? 1U : 0U;
	case USB_DFU_REQ_UPLOAD:
		return (dir == USB_D2H) ? 1U : 0U;
	case USB_DFU_REQ_GETSTATUS:
		return ((dir == USB_D2H) && (req->wLength == USB_DFU_STATUS_PKT_SIZE)) ? 1U : 0U;
	case USB_DFU_REQ_GETSTATE:
		return ((dir == USB_D2H) && (req->wLength == USBD_DFU_STATE_PKT_SIZE)) ? 1U : 0U;
	default:
		return 0U;
	}
}

static int usbd_dfu_handle_dnload(usb_dev_t *dev, usbd_dfu_dev_t *dfu, usb_setup_req_t *req)
{
	usbd_ep_t *ep0_out = &dev->ep0_out;

	if (req->wLength == 0U) {
		/* Zero-length DNLOAD ends the download; manifest starts on the next GETSTATUS.
		 * In any other state, including dfuIDLE, it is a protocol error (DFU 1.1 Table A.2). */
		if (dfu->state == USB_DFU_STATE_DNLOAD_IDLE) {
			dfu->state = USB_DFU_STATE_MANIFEST_SYNC;
			return HAL_OK;
		}
		return usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
	}

	if ((req->wLength > dfu->xfer_size) ||
		((dfu->state != USB_DFU_STATE_DFU_IDLE) && (dfu->state != USB_DFU_STATE_DNLOAD_IDLE))) {
		return usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
	}

	if (dfu->state == USB_DFU_STATE_DNLOAD_IDLE) {
		u16 expected = (u16)(dfu->block_num + 1U);
		if (req->wValue != expected) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Block num mismatch: got %u, expected %u\n", req->wValue, expected);
			return usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
		}
	} else if (req->wValue != 0U) {
		/* The first block of a download is always wBlockNum 0, it starts a new session */
		RTK_LOGS(TAG, RTK_LOG_ERROR, "First block must be 0, got %u\n", req->wValue);
		return usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
	}

	dfu->block_num = req->wValue;
	dfu->block_len = req->wLength;

	/* Receive the block straight into the class buffer */
	ep0_out->xfer_buf     = dfu->xfer_buf;
	ep0_out->xfer_buf_len = dfu->xfer_size;
	ep0_out->xfer_len     = req->wLength;
	if (usbd_ep_receive(dev, ep0_out) != HAL_OK) {
		ep0_out->xfer_buf     = dfu->ep0_default_buf;
		ep0_out->xfer_buf_len = dfu->ep0_default_buf_len;
		return usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
	}
	dfu->state = USB_DFU_STATE_DNLOAD_SYNC;

	return HAL_OK;
}

#if CONFIG_USBD_DFU_CAN_UPLOAD
static int usbd_dfu_handle_upload(usb_dev_t *dev, usbd_dfu_dev_t *dfu, usb_setup_req_t *req)
{
	usbd_ep_t *ep0_in = &dev->ep0_in;
	int len;

	if ((req->wLength > dfu->xfer_size) ||
		((dfu->state != USB_DFU_STATE_DFU_IDLE) && (dfu->state != USB_DFU_STATE_UPLOAD_IDLE))) {
		return usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
	}

	len = dfu->cb->read(req->wValue, dfu->xfer_buf, req->wLength);
	if (len < 0) {
		return usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_FILE);
	}

	ep0_in->xfer_buf     = dfu->xfer_buf;
	ep0_in->xfer_buf_len = dfu->xfer_size;
	ep0_in->xfer_len     = MIN((u32)len, (u32)req->wLength);
	dfu->ep0_in_owned = 1U;
	if (usbd_ep_transmit(dev, ep0_in) != HAL_OK) {
		ep0_in->xfer_buf     = dfu->ep0_default_buf;
		ep0_in->xfer_buf_len = dfu->ep0_default_buf_len;
		dfu->ep0_in_owned = 0U;
		return usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
	}
	/* A short block ends the upload (DFU 1.1 6.2) */
	dfu->upload_last = ((u32)len < req->wLength) ? 1U : 0U;
	dfu->state = USB_DFU_STATE_UPLOAD_IDLE;

	return HAL_OK;
}
#endif

static int usbd_dfu_handle_getstatus(usb_dev_t *dev, usbd_dfu_dev_t *dfu)
{
	switch (dfu->state) {
	case USB_DFU_STATE_MANIFEST_SYNC:
		if (dfu->manifest_done != 0U) {
			/* Manifestation-tolerant end: back to dfuIDLE */
			dfu->manifest_done = 0U;
			dfu->state  = USB_DFU_STATE_DFU_IDLE;
			dfu->status = USBD_DFU_STATUS_OK;
			return usbd_dfu_send_status(dev, dfu, 0U);
		}
		/* First GETSTATUS after the zero-length DNLOAD: start manifest_task */
		dfu->state = USB_DFU_STATE_MANIFEST;
		usb_os_sema_give(dfu->manifest_sema);
		return usbd_dfu_send_status(dev, dfu, dfu->manifest_poll_ms);

	case USB_DFU_STATE_MANIFEST:
		if (usbd_dfu_manifest_advance(dfu) != 0U) {
			return usbd_dfu_send_status(dev, dfu, 0U);
		}
		return usbd_dfu_send_status(dev, dfu, dfu->manifest_poll_ms);

	case USB_DFU_STATE_DNLOAD_SYNC:
		/* DFU 1.1 A.2.4: report the block result if write_task has already posted it */
		if (usbd_dfu_apply_write_state(dfu) != 0U) {
			return usbd_dfu_send_status(dev, dfu, 0U);
		}
		dfu->state = USB_DFU_STATE_DNBUSY;
		return usbd_dfu_send_status(dev, dfu, usbd_dfu_get_busy_poll_ms(dfu));

	case USB_DFU_STATE_DNBUSY:
		if (usbd_dfu_apply_write_state(dfu) != 0U) {
			return usbd_dfu_send_status(dev, dfu, 0U);
		}
		return usbd_dfu_send_status(dev, dfu, usbd_dfu_get_busy_poll_ms(dfu));

	default:
		return usbd_dfu_send_status(dev, dfu, 0U);
	}
}

static int usbd_dfu_class_request(usb_dev_t *dev, usbd_dfu_dev_t *dfu, usb_setup_req_t *req)
{
	usbd_ep_t *ep0_in = &dev->ep0_in;
	int ret = HAL_OK;

	/* DFU 1.1 A.1: Run-Time states accept only a few requests; a wrong one stalls without dfuERROR */
	if (dfu->state == USB_DFU_STATE_APP_IDLE) {
		if ((req->bRequest != USB_DFU_REQ_DETACH) && (req->bRequest != USB_DFU_REQ_GETSTATUS) &&
			(req->bRequest != USB_DFU_REQ_GETSTATE)) {
			return HAL_ERR_HW;
		}
	} else if (dfu->state == USB_DFU_STATE_APP_DETACH) {
		if ((req->bRequest != USB_DFU_REQ_GETSTATUS) && (req->bRequest != USB_DFU_REQ_GETSTATE)) {
			dfu->state = USB_DFU_STATE_APP_IDLE;
			return HAL_ERR_HW;
		}
	}

	switch (req->bRequest) {
	case USB_DFU_REQ_DETACH:
		if (dfu->state != USB_DFU_STATE_APP_IDLE) {
			ret = usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
			break;
		}
		dfu->state = USB_DFU_STATE_APP_DETACH;
		if (dfu->cb->detach != NULL) {
			dfu->cb->detach();
		}
#if !CONFIG_USBD_DFU_WILL_DETACH
		/* DFU 1.1 6.1.1: wValue is how long the host waits, never above wDetachTimeOut */
		dfu->detach_wait_ms = (u16)MIN((u32)req->wValue, dfu->detach_timeout_ms);
		if (dfu->detach_wait_ms == 0U) {
			dfu->detach_wait_ms = 1U;
		}
#endif
		/* DFU 1.1 §5: switch the descriptor set to DFU mode, re-enumerate off ISR */
		dfu->mode = USB_DFU_PROTOCOL_DFU;
		dfu->reconf_pending = 1U;
		usb_os_sema_give(dfu->reconf_sema);
		break;

	case USB_DFU_REQ_DNLOAD:
		ret = usbd_dfu_handle_dnload(dev, dfu, req);
		break;

	case USB_DFU_REQ_UPLOAD:
#if CONFIG_USBD_DFU_CAN_UPLOAD
		ret = usbd_dfu_handle_upload(dev, dfu, req);
#else
		ret = usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
#endif
		break;

	case USB_DFU_REQ_GETSTATUS:
		ret = usbd_dfu_handle_getstatus(dev, dfu);
		break;

	case USB_DFU_REQ_CLRSTATUS:
		if (dfu->state == USB_DFU_STATE_ERROR) {
			dfu->state  = USB_DFU_STATE_DFU_IDLE;
			dfu->status = USBD_DFU_STATUS_OK;
		} else {
			ret = usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
		}
		break;

	case USB_DFU_REQ_GETSTATE:
		/* DFU 1.1 A.2.5: dfuDNBUSY answers nothing but GETSTATUS */
		if (dfu->state == USB_DFU_STATE_DNBUSY) {
			ret = usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
		} else {
			ep0_in->xfer_buf[0] = (u8)dfu->state;
			ep0_in->xfer_len    = USBD_DFU_STATE_PKT_SIZE;
			ret = usbd_ep_transmit(dev, ep0_in);
		}
		break;

	case USB_DFU_REQ_ABORT:
		if ((dfu->state == USB_DFU_STATE_DFU_IDLE) || (dfu->state == USB_DFU_STATE_DNLOAD_IDLE) ||
			(dfu->state == USB_DFU_STATE_UPLOAD_IDLE)) {
			dfu->state  = USB_DFU_STATE_DFU_IDLE;
			dfu->status = USBD_DFU_STATUS_OK;
		} else {
			ret = usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
		}
		break;

	default:
		ret = usbd_dfu_stall_error(dfu, USBD_DFU_STATUS_ERR_STALLEDPKT);
		break;
	}

	return ret;
}

/**
 * @brief  Class and standard interface SETUP requests (ISR context).
 */
static int usbd_dfu_setup(usb_dev_t *dev, usb_setup_req_t *req)
{
	usbd_dfu_dev_t *dfu = &usbd_dfu_dev;
	usbd_ep_t *ep0_in = &dev->ep0_in;
	int ret = HAL_ERR_HW;

	/* The core also dispatches interface requests in DEFAULT/ADDRESSED state, but this class only
	 * owns EP0 buffers once configured. */
	if (dev->dev_state != USBD_STATE_CONFIGURED) {
		return HAL_ERR_HW;
	}

	switch (req->bmRequestType & USB_REQ_TYPE_MASK) {
	case USB_REQ_TYPE_STANDARD:
		switch (req->bRequest) {
		case USB_REQ_SET_INTERFACE:
			/* One interface with alternate setting 0 only (DFU 1.1 4.2.3) */
			if ((req->wIndex == USBD_DFU_ITF_NUM) && (USB_LOW_BYTE(req->wValue) == 0U)) {
				dfu->alt_setting = 0U;
				ret = HAL_OK;
			}
			break;
		case USB_REQ_GET_INTERFACE:
			if ((req->wIndex == USBD_DFU_ITF_NUM) && (req->wLength == 1U)) {
				ep0_in->xfer_buf[0] = dfu->alt_setting;
				ep0_in->xfer_len = 1U;
				ret = usbd_ep_transmit(dev, ep0_in);
			}
			break;
		case USB_REQ_GET_STATUS:
			if ((req->wIndex == USBD_DFU_ITF_NUM) && (req->wLength == 2U)) {
				ep0_in->xfer_buf[0] = 0U;
				ep0_in->xfer_buf[1] = 0U;
				ep0_in->xfer_len = 2U;
				ret = usbd_ep_transmit(dev, ep0_in);
			}
			break;
		default:
			break;
		}
		break;

	case USB_REQ_TYPE_CLASS:
		/* DFU 1.1 3.1: every DFU request targets the DFU interface. A malformed request stalls and
		 * leaves the DFU state untouched (USB 2.0 9.2.7). */
		if (((req->bmRequestType & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_INTERFACE) &&
			(req->wIndex == USBD_DFU_ITF_NUM) && (usbd_dfu_req_is_valid(req) != 0U)) {
			ret = usbd_dfu_class_request(dev, dfu, req);
		}
		break;

	default:
		break;
	}

	return ret;
}

/**
 * @brief  EP0 IN data stage complete (ISR context). Only the UPLOAD data stage belongs to DFU.
 */
static int usbd_dfu_ep0_data_in(usb_dev_t *dev, u8 status)
{
	usbd_dfu_dev_t *dfu = &usbd_dfu_dev;

	if (dfu->ep0_in_owned == 0U) {
		return HAL_ERR_PARA;
	}
	dfu->ep0_in_owned = 0U;

	dev->ep0_in.xfer_buf     = dfu->ep0_default_buf;
	dev->ep0_in.xfer_buf_len = dfu->ep0_default_buf_len;

	if (status != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "EP0 IN err: %d\n", status);
		dfu->state  = USB_DFU_STATE_ERROR;
		dfu->status = USBD_DFU_STATUS_ERR_STALLEDPKT;
		return HAL_OK;
	}

#if CONFIG_USBD_DFU_CAN_UPLOAD
	if (dfu->upload_last != 0U) {
		dfu->upload_last = 0U;
		dfu->state = USB_DFU_STATE_DFU_IDLE;
	}
#endif

	return HAL_OK;
}

/**
 * @brief  EP0 OUT data stage complete: a DNLOAD block was received (ISR context).
 *         write_task programs it; GETSTATUS reports dfuDNBUSY until the result is posted.
 */
static int usbd_dfu_ep0_data_out(usb_dev_t *dev)
{
	usbd_dfu_dev_t *dfu = &usbd_dfu_dev;

	dev->ep0_out.xfer_buf     = dfu->ep0_default_buf;
	dev->ep0_out.xfer_buf_len = dfu->ep0_default_buf_len;

	if (dfu->state != USB_DFU_STATE_DNLOAD_SYNC) {
		return HAL_ERR_HW;
	}

	dfu->write_state = USBD_DFU_WRITE_BUSY;
	usb_os_sema_give(dfu->write_sema);

	return HAL_OK;
}

/**
 * @brief  Descriptor requests (ISR context).
 */
static u16 usbd_dfu_get_descriptor(usb_dev_t *dev, usb_setup_req_t *req, u8 *buf)
{
	usbd_dfu_dev_t *dfu = &usbd_dfu_dev;
	const usbd_dfu_config_t *cfg = dfu->cfg;
	u8 type = USB_HIGH_BYTE(req->wValue);
	u16 len = 0U;
	u16 id;

	UNUSED(dev);

	switch (type) {
	case USB_DESC_TYPE_DEVICE:
		len = sizeof(usbd_dfu_dev_desc);
		usb_os_memcpy((void *)buf, (const void *)usbd_dfu_dev_desc, len);
		id = (cfg->vid != 0U) ? cfg->vid : (u16)USBD_DFU_DEFAULT_VID;
		buf[8] = USB_LOW_BYTE(id);
		buf[9] = USB_HIGH_BYTE(id);
		id = (cfg->pid != 0U) ? cfg->pid : (u16)USBD_DFU_DEFAULT_PID;
		buf[10] = USB_LOW_BYTE(id);
		buf[11] = USB_HIGH_BYTE(id);
		break;

	case USB_DESC_TYPE_CONFIGURATION:
#ifndef CONFIG_USB_FS
	case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
#endif
		/* No endpoints, so the other-speed configuration is the same blob */
		len = sizeof(usbd_dfu_config_desc);
		usb_os_memcpy((void *)buf, (const void *)usbd_dfu_config_desc, len);
		buf[USB_CFG_DESC_OFFSET_TYPE]          = type;
		buf[USB_CFG_DESC_OFFSET_TOTAL_LEN]     = USB_LOW_BYTE(len);
		buf[USB_CFG_DESC_OFFSET_TOTAL_LEN + 1] = USB_HIGH_BYTE(len);
		if (dfu->mode == USB_DFU_PROTOCOL_RUNTIME) {
			buf[USBD_DFU_CFG_IF_PROTOCOL_OFFSET] = USB_DFU_PROTOCOL_RUNTIME;
		}
		buf[USBD_DFU_CFG_DETACH_TIMEOUT_OFFSET]     = USB_LOW_BYTE(dfu->detach_timeout_ms);
		buf[USBD_DFU_CFG_DETACH_TIMEOUT_OFFSET + 1] = USB_HIGH_BYTE(dfu->detach_timeout_ms);
		buf[USBD_DFU_CFG_XFER_SIZE_OFFSET]          = USB_LOW_BYTE(dfu->xfer_size);
		buf[USBD_DFU_CFG_XFER_SIZE_OFFSET + 1]      = USB_HIGH_BYTE(dfu->xfer_size);
		break;

#ifndef CONFIG_USB_FS
	case USB_DESC_TYPE_DEVICE_QUALIFIER:
		len = sizeof(usbd_dfu_device_qualifier_desc);
		usb_os_memcpy((void *)buf, (const void *)usbd_dfu_device_qualifier_desc, len);
		break;
#endif

	case USB_DESC_TYPE_STRING:
		switch (USB_LOW_BYTE(req->wValue)) {
		case USBD_IDX_LANGID_STR:
			len = sizeof(usbd_dfu_lang_id_desc);
			usb_os_memcpy((void *)buf, (const void *)usbd_dfu_lang_id_desc, len);
			break;
		case USBD_DFU_STRING_MANUFACTURER:
			len = usbd_get_str_desc((cfg->mfg_str != NULL) ? cfg->mfg_str : USBD_DFU_DEFAULT_MFG_STRING, buf);
			break;
		case USBD_DFU_STRING_PRODUCT:
			len = usbd_get_str_desc((cfg->prod_str != NULL) ? cfg->prod_str : USBD_DFU_DEFAULT_PROD_STRING, buf);
			break;
		case USBD_DFU_STRING_SERIALNUMBER:
			len = usbd_get_str_desc((cfg->sn_str != NULL) ? cfg->sn_str : USBD_DFU_DEFAULT_SN_STRING, buf);
			break;
		case USBD_DFU_STRING_INTERFACE:
			len = usbd_get_str_desc((cfg->itf_str != NULL) ? cfg->itf_str : USBD_DFU_DEFAULT_IFACE_STRING, buf);
			break;
		default:
			USB_DIAG(USB_LAYER_CLASS, USB_EVT_ERR_GET_DESC, 0);
			break;
		}
		break;

	default:
		break;
	}

	return len;
}

static void usbd_dfu_status_changed(usb_dev_t *dev, u8 old_status, u8 status)
{
	usbd_dfu_dev_t *dfu = &usbd_dfu_dev;

	UNUSED(dev);

	if ((dfu->cb != NULL) && (dfu->cb->status_changed != NULL)) {
		dfu->cb->status_changed(old_status, status);
	}
}

/* The callbacks program flash and may reset the SoC, which need the secure world under TrustZone */
static void usbd_dfu_task_prologue(void)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
}

/**
 * @brief  Runs cb->write() for every received DNLOAD block. Only posts the result in write_state:
 *         state/status belong to the ISR, which applies the result on the next GETSTATUS.
 */
static void usbd_dfu_write_task(void *param)
{
	usbd_dfu_dev_t *dfu = (usbd_dfu_dev_t *)param;
	int ret;

	usbd_dfu_task_prologue();

	for (;;) {
		if (usb_os_sema_take(dfu->write_sema, USB_OS_SEMA_TIMEOUT) != HAL_OK) {
			continue;
		}
		if (dfu->write_stop != 0U) {
			break;
		}
		ret = dfu->cb->write(dfu->block_num, dfu->xfer_buf, dfu->block_len);
		if (ret != HAL_OK) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Write blk %u err: %d\n", dfu->block_num, ret);
			dfu->write_state = USBD_DFU_WRITE_FAIL;
		} else {
			dfu->write_state = USBD_DFU_WRITE_OK;
		}
	}

	/* Publish the exit last, usbd_dfu_stop_task() waits for it */
	dfu->write_task = NULL;
	rtos_task_delete(NULL);
}

/**
 * @brief  Runs cb->manifest() once the device enters dfuMANIFEST.
 */
static void usbd_dfu_manifest_task(void *param)
{
	usbd_dfu_dev_t *dfu = (usbd_dfu_dev_t *)param;

	usbd_dfu_task_prologue();

	for (;;) {
		if (usb_os_sema_take(dfu->manifest_sema, USB_OS_SEMA_TIMEOUT) != HAL_OK) {
			continue;
		}
		if (dfu->manifest_stop != 0U) {
			break;
		}
		dfu->manifest_ret = (dfu->cb->manifest != NULL) ? dfu->cb->manifest() : HAL_OK;
		dfu->manifest_done = 1U;
		RTK_LOGS(TAG, RTK_LOG_INFO, "Manifest %s\n", (dfu->manifest_ret == HAL_OK) ? "done" : "fail");
	}

	dfu->manifest_task = NULL;
	rtos_task_delete(NULL);
}

/**
 * @brief  Handles the Run-Time -> DFU mode switch off ISR context.
 *   WILL_DETACH=1: asks the application to re-enumerate (cb->reconf).
 *   WILL_DETACH=0: waits for the host USB reset up to detach_wait_ms, then reverts to Run-Time mode.
 */
static void usbd_dfu_reconf_task(void *param)
{
	usbd_dfu_dev_t *dfu = (usbd_dfu_dev_t *)param;
#if !CONFIG_USBD_DFU_WILL_DETACH
	int ret;
#endif

	for (;;) {
		if (usb_os_sema_take(dfu->reconf_sema, USB_OS_SEMA_TIMEOUT) != HAL_OK) {
			continue;
		}
		if (dfu->reconf_stop != 0U) {
			break;
		}
		if (dfu->reconf_pending == 0U) {
			continue;
		}

#if CONFIG_USBD_DFU_WILL_DETACH
		dfu->reconf_pending = 0U;
		/* Let the DFU_DETACH status stage complete before the application tears down the stack */
		usb_os_sleep_ms(dfu->detach_status_delay_ms);
		RTK_LOGS(TAG, RTK_LOG_INFO, "Re-enumerate in %s mode\n",
				 (dfu->mode == USB_DFU_PROTOCOL_RUNTIME) ? "Run-Time" : "DFU");
		if (dfu->cb->reconf != NULL) {
			dfu->cb->reconf();
		}
#else
		RTK_LOGS(TAG, RTK_LOG_INFO, "Wait host USB reset (%u ms)\n", dfu->detach_wait_ms);
		do {
			dfu->reconf_pending = 0U;
			ret = usb_os_sema_take(dfu->reconf_sema, dfu->detach_wait_ms);
			if (dfu->reconf_stop != 0U) {
				goto exit;
			}
		} while ((ret == HAL_OK) && (dfu->reconf_pending != 0U));

		if (ret != HAL_OK) {
			/* No USB reset arrived within the timeout: clear_config() would have left appDETACH */
			usb_os_enter_critical(1U);
			if (dfu->state == USB_DFU_STATE_APP_DETACH) {
				dfu->mode  = USB_DFU_PROTOCOL_RUNTIME;
				dfu->state = USB_DFU_STATE_APP_IDLE;
				ret = HAL_OK;
			}
			usb_os_exit_critical(1U);
			if (ret == HAL_OK) {
				RTK_LOGS(TAG, RTK_LOG_WARN, "No USB reset, back to Run-Time mode\n");
			}
		}
#endif
	}

#if !CONFIG_USBD_DFU_WILL_DETACH
exit:
#endif
	dfu->reconf_task = NULL;
	rtos_task_delete(NULL);
}

/**
 * @brief  Ask a worker task to exit and wait until it has (it clears its own handle last).
 *         Unbounded on purpose: releasing the buffer while a callback still runs would be a
 *         use-after-free, so a hung callback is reported instead.
 */
static void usbd_dfu_stop_task(usb_os_task_t *task, __IO u8 *stop, usb_os_sema_t sema)
{
	u32 wait_ms = 0U;

	if (*task == NULL) {
		return;
	}

	*stop = 1U;
	usb_os_sema_give(sema);
	while (*(volatile usb_os_task_t *)task != NULL) {
		usb_os_sleep_ms(1U);
		if (++wait_ms >= USBD_DFU_STOP_TASK_WARN_MS) {
			RTK_LOGS(TAG, RTK_LOG_WARN, "Wait task exit\n");
			wait_ms = 0U;
		}
	}
}

static void usbd_dfu_free_resources(usbd_dfu_dev_t *dfu)
{
	usbd_dfu_stop_task(&dfu->reconf_task, &dfu->reconf_stop, dfu->reconf_sema);
	usbd_dfu_stop_task(&dfu->manifest_task, &dfu->manifest_stop, dfu->manifest_sema);
	usbd_dfu_stop_task(&dfu->write_task, &dfu->write_stop, dfu->write_sema);

	if (dfu->reconf_sema != NULL) {
		usb_os_sema_delete(dfu->reconf_sema);
		dfu->reconf_sema = NULL;
	}
	if (dfu->manifest_sema != NULL) {
		usb_os_sema_delete(dfu->manifest_sema);
		dfu->manifest_sema = NULL;
	}
	if (dfu->write_sema != NULL) {
		usb_os_sema_delete(dfu->write_sema);
		dfu->write_sema = NULL;
	}
	if (dfu->xfer_buf != NULL) {
		usbd_dfu_restore_ep0(dfu->dev, dfu);
		usb_os_mfree((void *)dfu->xfer_buf);
		dfu->xfer_buf = NULL;
	}
}

static int usbd_dfu_create_task(usb_os_task_t *task, const char *name, void (*func)(void *), usbd_dfu_dev_t *dfu,
								u16 stack, u16 def_stack, u8 prio, u32 def_prio)
{
	if (rtos_task_create(task, name, func, dfu, (stack != 0U) ? stack : def_stack,
						 (prio != 0U) ? prio : def_prio) != RTK_SUCCESS) {
		*task = NULL;
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create %s fail\n", name);
		return HAL_ERR_MEM;
	}
	return HAL_OK;
}

/* Exported functions --------------------------------------------------------*/

int usbd_dfu_init(const usbd_dfu_cb_t *cb, const usbd_dfu_config_t *cfg)
{
	usbd_dfu_dev_t *dfu = &usbd_dfu_dev;
	u8 saved_mode = dfu->mode;
	int ret;

	if ((cb == NULL) || (cfg == NULL) || (cb->write == NULL)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Invalid param\n");
		return HAL_ERR_PARA;
	}
#if CONFIG_USBD_DFU_CAN_UPLOAD
	if (cb->read == NULL) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "read cb required (CAN_UPLOAD=1)\n");
		return HAL_ERR_PARA;
	}
#endif

	/* mode survives a reconf re-init so the device comes back in DFU mode */
	usb_os_memset((void *)dfu, 0, sizeof(usbd_dfu_dev_t));

	dfu->cb  = cb;
	dfu->cfg = cfg;
	dfu->xfer_size = (cfg->xfer_size != 0U) ? cfg->xfer_size : USBD_DFU_DEFAULT_XFER_SIZE;
	dfu->detach_timeout_ms = (cfg->detach_timeout_ms != 0U) ? cfg->detach_timeout_ms : USBD_DFU_DEFAULT_DETACH_TIMEOUT;
	dfu->detach_status_delay_ms = (cfg->detach_status_delay_ms != 0U) ? cfg->detach_status_delay_ms : USBD_DFU_DEFAULT_DETACH_STATUS_DELAY_MS;
	dfu->write_poll_ms = (cfg->write_poll_ms != 0U) ? cfg->write_poll_ms : USBD_DFU_DEFAULT_WRITE_POLL_MS;
	dfu->manifest_poll_ms = (cfg->manifest_poll_ms != 0U) ? cfg->manifest_poll_ms : USBD_DFU_DEFAULT_MANIFEST_POLL_MS;

	/* 16-bit descriptor fields; bwPollTimeout is 24-bit */
	if ((dfu->xfer_size > 0xFFFFU) || (dfu->detach_timeout_ms > 0xFFFFU) ||
		(dfu->write_poll_ms > 0xFFFFFFU) || (dfu->manifest_poll_ms > 0xFFFFFFU)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Invalid cfg\n");
		return HAL_ERR_PARA;
	}

	if ((cfg->dfu_mode_only != 0U) || (saved_mode == USB_DFU_PROTOCOL_DFU)) {
		dfu->mode = USB_DFU_PROTOCOL_DFU;
	} else {
		dfu->mode = USB_DFU_PROTOCOL_RUNTIME;
	}

	/* Whole cache lines, so the DMA invalidate never touches a neighbouring allocation */
	dfu->xfer_buf = (u8 *)usb_os_malloc(CACHE_LINE_ALIGNMENT(dfu->xfer_size));
	if ((dfu->xfer_buf == NULL) || !USB_IS_MEM_DMA_ALIGNED(dfu->xfer_buf)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "No mem for xfer_buf\n");
		ret = HAL_ERR_MEM;
		goto fail;
	}

	if ((usb_os_sema_create(&dfu->write_sema) != HAL_OK) ||
		(usb_os_sema_create(&dfu->manifest_sema) != HAL_OK) ||
		(usb_os_sema_create(&dfu->reconf_sema) != HAL_OK)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "No mem for sema\n");
		ret = HAL_ERR_MEM;
		goto fail;
	}

	ret = usbd_dfu_create_task(&dfu->write_task, "usbd_dfu_write", usbd_dfu_write_task, dfu,
							   cfg->write_task_stack_size, USBD_DFU_DEFAULT_WRITE_TASK_STACK,
							   cfg->write_task_priority, USBD_DFU_DEFAULT_WRITE_TASK_PRIORITY);
	if (ret == HAL_OK) {
		ret = usbd_dfu_create_task(&dfu->manifest_task, "usbd_dfu_manifest", usbd_dfu_manifest_task, dfu,
								   cfg->manifest_task_stack_size, USBD_DFU_DEFAULT_MANIFEST_TASK_STACK,
								   cfg->manifest_task_priority, USBD_DFU_DEFAULT_MANIFEST_TASK_PRIORITY);
	}
	if (ret == HAL_OK) {
		ret = usbd_dfu_create_task(&dfu->reconf_task, "usbd_dfu_reconf", usbd_dfu_reconf_task, dfu,
								   cfg->reenum_task_stack_size, USBD_DFU_DEFAULT_REENUM_TASK_STACK,
								   cfg->reenum_task_priority, USBD_DFU_DEFAULT_REENUM_TASK_PRIORITY);
	}
	if (ret != HAL_OK) {
		goto fail;
	}

	if (cb->init != NULL) {
		ret = cb->init();
		if (ret != HAL_OK) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "CB init fail: %d\n", ret);
			goto fail;
		}
	}

	dfu->state  = (usb_dfu_state_t)usbd_dfu_idle_state(dfu);
	dfu->status = USBD_DFU_STATUS_OK;

	ret = usbd_register_class(&usbd_dfu_driver);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Register class fail: %d\n", ret);
		if (cb->deinit != NULL) {
			cb->deinit();
		}
		goto fail;
	}

	RTK_LOGS(TAG, RTK_LOG_INFO, "Init in %s mode, xfer %u\n",
			 (dfu->mode == USB_DFU_PROTOCOL_RUNTIME) ? "Run-Time" : "DFU", dfu->xfer_size);
	return HAL_OK;

fail:
	usbd_dfu_free_resources(dfu);
	dfu->cb = NULL;
	return ret;
}

void usbd_dfu_deinit(void)
{
	usbd_dfu_dev_t *dfu = &usbd_dfu_dev;

	if (dfu->cb == NULL) {
		return;
	}

	/* Leave the core first, so no class callback runs during the teardown below */
	usbd_unregister_class();

	usbd_dfu_free_resources(dfu);

	if (dfu->cb->deinit != NULL) {
		dfu->cb->deinit();
	}
	dfu->cb = NULL;
}

void usbd_dfu_set_write_poll_ms(u32 poll_ms)
{
	usbd_dfu_dev.write_poll_ms = MIN(poll_ms, 0xFFFFFFU);
}

void usbd_dfu_set_manifest_poll_ms(u32 poll_ms)
{
	usbd_dfu_dev.manifest_poll_ms = MIN(poll_ms, 0xFFFFFFU);
}

u8 usbd_dfu_get_state(void)
{
	return (u8)usbd_dfu_dev.state;
}

void usbd_dfu_reset_mode(void)
{
	usbd_dfu_dev.mode = USB_DFU_PROTOCOL_RUNTIME;
}
