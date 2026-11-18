/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef USBD_DFU_H
#define USBD_DFU_H

/* Includes ------------------------------------------------------------------*/

#include "usbd.h"
#include "usb_dfu.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup USB_Device_API USB Device API
 *  @{
 */

/* Exported defines ----------------------------------------------------------*/

/** @addtogroup USB_Device_Constants USB Device Constants
 *  @{
 */
/** @defgroup Device_DFU_Constants Device DFU Constants
 *  @brief Feature switches, descriptor layout and defaults of the DFU class.
 *  @{
 */

/*
 * Feature switches (bmAttributes of the DFU Functional Descriptor). This SDK has no Kconfig, so
 * they default here and can be overridden with a compile definition (e.g. in the example cmake).
 * They select which code is compiled, so they are not usbd_dfu_config_t fields.
 *
 * CONFIG_USBD_DFU_WILL_DETACH
 *   1: bitWillDetach=1. On DFU_DETACH the class asks the application (reconf callback) to run
 *      usbd_dfu_deinit/usbd_deinit/usbd_init/usbd_dfu_init, so the device re-enumerates in DFU mode
 *      by itself. After a non-tolerant manifest it re-enumerates back in Run-Time mode.
 *   0: The host has to issue a USB reset within wDetachTimeOut after DFU_DETACH, otherwise the
 *      device reverts to Run-Time mode.
 * CONFIG_USBD_DFU_MANIFESTATION_TOLERANT
 *   1: After manifestation the device goes back to dfuIDLE and keeps answering GETSTATUS.
 *   0: After manifestation the device waits in dfuMANIFEST-WAIT-RESET.
 * CONFIG_USBD_DFU_CAN_UPLOAD
 *   1: DFU_UPLOAD is supported, usbd_dfu_cb_t::read is required.
 */
#ifndef CONFIG_USBD_DFU_WILL_DETACH
#define CONFIG_USBD_DFU_WILL_DETACH                 1
#endif
#ifndef CONFIG_USBD_DFU_MANIFESTATION_TOLERANT
#define CONFIG_USBD_DFU_MANIFESTATION_TOLERANT      1
#endif
#ifndef CONFIG_USBD_DFU_CAN_UPLOAD
#define CONFIG_USBD_DFU_CAN_UPLOAD                  0
#endif

/* Defaults applied when the matching usbd_dfu_config_t field is left 0/NULL. */
#define USBD_DFU_DEFAULT_VID                        0x0BDAU             /**< Default idVendor, same as AmebaPro3 DFU. */
#define USBD_DFU_DEFAULT_PID                        0x8D00U             /**< Default idProduct, same as AmebaPro3 DFU. */
#define USBD_DFU_DEFAULT_MFG_STRING                 "Realtek"           /**< Default manufacturer string. */
#define USBD_DFU_DEFAULT_PROD_STRING                "Realtek DFU Device" /**< Default product string. */
#define USBD_DFU_DEFAULT_SN_STRING                  "1234567890"        /**< Default serial number string. */
#define USBD_DFU_DEFAULT_IFACE_STRING               "DFU Interface"     /**< Default iInterface string. */
#define USBD_DFU_LANGID_STRING                      0x0409U             /**< English (United States). */

/** Default maximum block size per DNLOAD/UPLOAD request (wTransferSize). */
#define USBD_DFU_DEFAULT_XFER_SIZE                  4096U
/** Default DFU detach timeout (ms), reported as wDetachTimeOut. */
#define USBD_DFU_DEFAULT_DETACH_TIMEOUT             1000U
/** Default delay (ms) after DFU_DETACH so its status stage completes before the bus detach. */
#define USBD_DFU_DEFAULT_DETACH_STATUS_DELAY_MS     5U
/** Default bwPollTimeout (ms) reported while a block is programmed. */
#define USBD_DFU_DEFAULT_WRITE_POLL_MS              10U
/** Default bwPollTimeout (ms) reported during dfuMANIFEST. */
#define USBD_DFU_DEFAULT_MANIFEST_POLL_MS           100U

/* Worker task defaults. Stack sizes are in FreeRTOS stack words (rtos_task_create shim). */
#define USBD_DFU_DEFAULT_WRITE_TASK_STACK           1024U
#define USBD_DFU_DEFAULT_WRITE_TASK_PRIORITY        (tskIDLE_PRIORITY + 3U)
#define USBD_DFU_DEFAULT_MANIFEST_TASK_STACK        1024U
#define USBD_DFU_DEFAULT_MANIFEST_TASK_PRIORITY     (tskIDLE_PRIORITY + 3U)
#define USBD_DFU_DEFAULT_REENUM_TASK_STACK          512U
#define USBD_DFU_DEFAULT_REENUM_TASK_PRIORITY       (tskIDLE_PRIORITY + 3U)

/* String descriptor indexes */
#define USBD_DFU_STRING_MANUFACTURER                USBD_IDX_MFC_STR
#define USBD_DFU_STRING_PRODUCT                     USBD_IDX_PRODUCT_STR
#define USBD_DFU_STRING_SERIALNUMBER                USBD_IDX_SERIAL_STR
#define USBD_DFU_STRING_INTERFACE                   0x04U

/* Byte offsets inside the configuration descriptor (Config(9) + Interface(9) + DFU Func(9)) */
#define USBD_DFU_CFG_IF_PROTOCOL_OFFSET             16U     /* Config(9) + 7: bInterfaceProtocol */
#define USBD_DFU_CFG_DETACH_TIMEOUT_OFFSET          21U     /* Config(9) + Interface(9) + 3 */
#define USBD_DFU_CFG_XFER_SIZE_OFFSET               23U     /* Config(9) + Interface(9) + 5 */

/** @} End of Device_DFU_Constants group */
/** @} End of USB_Device_Constants group */

/* Exported types ------------------------------------------------------------*/

/** @addtogroup USB_Device_Types USB Device Types
 *  @{
 */
/** @defgroup Device_DFU_Types Device DFU Types
 *  @{
 */

/**
 * @brief DFU status codes (DFU 1.1 Table 6.1.2).
 */
typedef enum {
	USBD_DFU_STATUS_OK                    = 0x00U,
	USBD_DFU_STATUS_ERR_TARGET            = 0x01U,
	USBD_DFU_STATUS_ERR_FILE              = 0x02U,
	USBD_DFU_STATUS_ERR_WRITE             = 0x03U,
	USBD_DFU_STATUS_ERR_ERASE             = 0x04U,
	USBD_DFU_STATUS_ERR_CHECK_ERASED      = 0x05U,
	USBD_DFU_STATUS_ERR_PROG              = 0x06U,
	USBD_DFU_STATUS_ERR_VERIFY            = 0x07U,
	USBD_DFU_STATUS_ERR_ADDRESS           = 0x08U,
	USBD_DFU_STATUS_ERR_NOTDONE           = 0x09U,
	USBD_DFU_STATUS_ERR_FIRMWARE          = 0x0AU,
	USBD_DFU_STATUS_ERR_VENDOR            = 0x0BU,
	USBD_DFU_STATUS_ERR_USBR              = 0x0CU,
	USBD_DFU_STATUS_ERR_POR               = 0x0DU,
	USBD_DFU_STATUS_ERR_UNKNOWN           = 0x0EU,
	USBD_DFU_STATUS_ERR_STALLEDPKT        = 0x0FU,
} usbd_dfu_status_t;

/**
 * @brief Configuration of the DFU class. Every field left 0/NULL takes its default.
 * @note  Stored by pointer, so it shall stay valid until @ref usbd_dfu_deinit.
 */
typedef struct {
	u32 xfer_size;              /**< wTransferSize, also the size of the class block buffer. 0: @ref USBD_DFU_DEFAULT_XFER_SIZE. */
	u32 detach_timeout_ms;      /**< wDetachTimeOut, 0: @ref USBD_DFU_DEFAULT_DETACH_TIMEOUT. */
	u32 write_poll_ms;          /**< bwPollTimeout while a block is programmed, 0: @ref USBD_DFU_DEFAULT_WRITE_POLL_MS. */
	u32 manifest_poll_ms;       /**< bwPollTimeout during dfuMANIFEST, 0: @ref USBD_DFU_DEFAULT_MANIFEST_POLL_MS. */
	const char *mfg_str;        /**< iManufacturer string, NULL: @ref USBD_DFU_DEFAULT_MFG_STRING. */
	const char *prod_str;       /**< iProduct string, NULL: @ref USBD_DFU_DEFAULT_PROD_STRING. */
	const char *sn_str;         /**< iSerialNumber string, NULL: @ref USBD_DFU_DEFAULT_SN_STRING. */
	const char *itf_str;        /**< iInterface string, NULL: @ref USBD_DFU_DEFAULT_IFACE_STRING. */
	u16 vid;                    /**< idVendor, 0: @ref USBD_DFU_DEFAULT_VID. */
	u16 pid;                    /**< idProduct, 0: @ref USBD_DFU_DEFAULT_PID. */
	u16 write_task_stack_size;  /**< 0: @ref USBD_DFU_DEFAULT_WRITE_TASK_STACK. */
	u16 manifest_task_stack_size; /**< 0: @ref USBD_DFU_DEFAULT_MANIFEST_TASK_STACK. */
	u16 reenum_task_stack_size; /**< 0: @ref USBD_DFU_DEFAULT_REENUM_TASK_STACK. */
	u16 detach_status_delay_ms; /**< 0: @ref USBD_DFU_DEFAULT_DETACH_STATUS_DELAY_MS. */
	u8  write_task_priority;    /**< 0: @ref USBD_DFU_DEFAULT_WRITE_TASK_PRIORITY. */
	u8  manifest_task_priority; /**< 0: @ref USBD_DFU_DEFAULT_MANIFEST_TASK_PRIORITY. */
	u8  reenum_task_priority;   /**< 0: @ref USBD_DFU_DEFAULT_REENUM_TASK_PRIORITY. */
	/**
	 * @brief Enumerate directly in DFU mode (dfuIDLE), skipping Run-Time mode and DFU_DETACH.
	 * @details This is the legacy AmebaPro2 usb_dfu_ota behaviour: a single enumeration, so the
	 *          host only binds one driver. 0 starts in Run-Time mode (appIDLE) as DFU 1.1 §4.1.
	 */
	u8  dfu_mode_only;
} usbd_dfu_config_t;

/**
 * @brief Application callbacks.
 * @note  write() and manifest() run in dedicated task contexts and may block (flash erase/program).
 *        read(), detach(), get_write_poll_ms() and status_changed() run in ISR context and must not
 *        block. init(), deinit() and reconf() run in task context.
 */
typedef struct {
	int (*init)(void);          /**< Called once by usbd_dfu_init(). Optional. */
	int (*deinit)(void);        /**< Called once by usbd_dfu_deinit(). Optional. */

	/**
	 * @brief  Program one DNLOAD block (task context). Required.
	 * @param  block_num: wBlockNum, 0 starts a new download session.
	 * @param  buf: Block data, valid until the callback returns.
	 * @param  len: Block length, at most usbd_dfu_config_t::xfer_size.
	 * @retval 0 on success, non-zero reports dfuERROR / errWRITE to the host.
	 */
	int (*write)(u16 block_num, u8 *buf, u32 len);

	/**
	 * @brief  Read one UPLOAD block (ISR context). Required with CONFIG_USBD_DFU_CAN_UPLOAD.
	 * @retval Bytes placed in buf (< max_len ends the upload), or < 0 on error.
	 */
	int (*read)(u16 block_num, u8 *buf, u32 max_len);

	/**
	 * @brief  Manifest the downloaded image after the zero-length DNLOAD (task context).
	 * @retval 0 on success, non-zero reports dfuERROR / errVERIFY to the host.
	 */
	int (*manifest)(void);

	void (*detach)(void);       /**< DFU_DETACH received in appIDLE (ISR context). Optional. */

	/**
	 * @brief  Re-enumeration request (CONFIG_USBD_DFU_WILL_DETACH=1, task context).
	 * @details The application shall, from ANOTHER task, call usbd_dfu_deinit(), usbd_deinit(),
	 *          usbd_init() and usbd_dfu_init(). usbd_dfu_deinit() waits for the task that runs this
	 *          callback, so calling it from here would never return.
	 */
	void (*reconf)(void);

	/** Expected programming time (ms) of a block (ISR context). Optional, NULL uses write_poll_ms. */
	u32(*get_write_poll_ms)(u16 block_num);

	/** USB attach status changed (ISR context). Optional. */
	void (*status_changed)(u8 old_status, u8 status);
} usbd_dfu_cb_t;

/**
 * @brief DFU device instance (internal).
 */
typedef struct {
	usb_dev_t *dev;
	const usbd_dfu_cb_t *cb;
	const usbd_dfu_config_t *cfg;

	usb_dfu_state_t state;
	usbd_dfu_status_t status;
	u8 mode;                         /* USB_DFU_PROTOCOL_RUNTIME or USB_DFU_PROTOCOL_DFU */
	u8 alt_setting;

	__IO u8 write_state;             /* Result of the block programmed by write_task */
	__IO u8 manifest_done;           /* Set by manifest_task when cb->manifest() returned */
	__IO int manifest_ret;           /* Return value of cb->manifest() */
	__IO u8 reconf_pending;
	__IO u8 write_stop;
	__IO u8 manifest_stop;
	__IO u8 reconf_stop;
	u8 ep0_in_owned;                 /* The in-flight EP0 IN data stage was armed by DFU */
#if CONFIG_USBD_DFU_CAN_UPLOAD
	u8 upload_last;
#endif

	u8 *xfer_buf;                    /* DNLOAD/UPLOAD block buffer */
	u32 xfer_size;
	u8 *ep0_default_buf;             /* Core EP0 buffer saved on set_config */
	u32 ep0_default_buf_len;
	u16 block_num;
	u16 block_len;

	usb_os_sema_t write_sema;
	usb_os_task_t write_task;
	u32 write_poll_ms;

	usb_os_sema_t manifest_sema;
	usb_os_task_t manifest_task;
	u32 manifest_poll_ms;

	usb_os_sema_t reconf_sema;
	usb_os_task_t reconf_task;
	u32 detach_timeout_ms;
	u16 detach_status_delay_ms;
#if !CONFIG_USBD_DFU_WILL_DETACH
	u16 detach_wait_ms;              /* min(DFU_DETACH wValue, wDetachTimeOut) */
#endif
} usbd_dfu_dev_t;

/** @} End of Device_DFU_Types group */
/** @} End of USB_Device_Types group */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup USB_Device_Functions USB Device Functions
 *  @{
 */
/** @defgroup Device_DFU_Functions Device DFU Functions
 *  @{
 */

/**
 * @brief  Initialize and register the DFU class. Call after usbd_init().
 * @param  cb: Application callbacks.
 * @param  cfg: Class configuration.
 * @retval 0 on success, non-zero on failure.
 */
int usbd_dfu_init(const usbd_dfu_cb_t *cb, const usbd_dfu_config_t *cfg);

/**
 * @brief  Unregister the DFU class and release its resources. Call before usbd_deinit().
 * @note   Task context only, never from a DFU callback.
 */
void usbd_dfu_deinit(void);

/**
 * @brief  Set the bwPollTimeout (ms) reported while a block is programmed.
 */
void usbd_dfu_set_write_poll_ms(u32 poll_ms);

/**
 * @brief  Set the bwPollTimeout (ms) reported during dfuMANIFEST.
 */
void usbd_dfu_set_manifest_poll_ms(u32 poll_ms);

/**
 * @brief  Get the current DFU state, see @ref usb_dfu_state_t.
 */
u8 usbd_dfu_get_state(void);

/**
 * @brief  Force the next usbd_dfu_init() to enumerate in Run-Time mode.
 * @details The DFU/Run-Time mode survives usbd_dfu_deinit() so the reconf re-enumeration comes up
 *          in DFU mode. Call this before a user-initiated restart to start over from Run-Time mode.
 */
void usbd_dfu_reset_mode(void);

/** @} End of Device_DFU_Functions group */
/** @} End of USB_Device_Functions group */
/** @} End of USB_Device_API group */

#ifdef __cplusplus
}
#endif

#endif /* USBD_DFU_H */
