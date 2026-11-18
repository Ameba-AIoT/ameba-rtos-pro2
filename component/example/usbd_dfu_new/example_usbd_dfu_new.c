/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * USB DFU firmware update on the usb_stack/v1.0.0 device stack.
 *
 * Flow: the AmebaPro3 usbd_dfu class/AT flow (Run-Time mode -> DFU_DETACH -> re-enumerate in DFU
 * mode -> DNLOAD -> manifest -> reboot) on top of the AmebaPro2 usb_dfu_ota image handling:
 * - ota.bin is streamed into the firmware slot that is NOT running (FW1 <-> FW2) through fwfs.
 * - The last 4 bytes of ota.bin are the byte-sum checksum of everything before them.
 * - Power-loss safety: the first block of the image (manifest header with label and timestamp) is
 *   held back in RAM and its place written as 0xFF, so until manifest the slot never carries a
 *   bootable image. This is stronger than ota_flash_NOR(), which holds back only the 8-byte label:
 *   on NAND the bootloader does not check the label (verified on board), and fwfs may commit the
 *   partition table early if another module closes an fwfs file during the download.
 *   Manifest checks the checksum of the received data first and only then writes the header. On
 *   NAND fwfs commits the partition table at pfw_close(), after the header is in.
 * - A slot that once failed to load is recorded in an AON register and skipped by the bootloader
 *   until power-off, even after it is rewritten. Manifest clears that record once the new image is
 *   verified (hal_sys_clear_prefw_boot_fail_sts), so DFU can recover such a slot.
 *   The slot is then read back as a second check; a mismatch at either step voids the slot header
 *   and the host gets errVERIFY.
 * - On success the board reboots once the host has read the final DFU_GETSTATUS.
 *
 * Unlike the legacy example, flash is programmed from the class write task (not the USB ISR), and a
 * failure is reported to dfu-util (errWRITE / errVERIFY) instead of only being printed.
 */

/* Includes ------------------------------------------------------------------*/

#include "usbd_dfu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "log_service.h"
#include "fwfs.h"
#include "sys_api.h"
#include "flash_api.h"
#include "device_lock.h"
#include <stdlib.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/* 1: enumerate directly in DFU mode like the legacy usb_dfu_ota example (no DFU_DETACH step).
 * 0: start in Run-Time mode, dfu-util detaches the device into DFU mode (AmebaPro3 flow). */
#define DFU_NEW_DFU_MODE_ONLY           0

/* 1: start USB DFU at boot (standalone example). 0: only on USBDFU=1, used when another example
 * (e.g. usbd_uvc_new) owns the USB device and builds this file in, see usbd_dfu_new_usb_release(). */
#ifndef DFU_NEW_AUTO_START
#define DFU_NEW_AUTO_START              1
#endif

#define DFU_NEW_XFER_SIZE               4096U   /* wTransferSize, one NOR sector per block */
#define DFU_NEW_WRITE_POLL_MS           10U     /* bwPollTimeout while a block is programmed */
#define DFU_NEW_MANIFEST_POLL_MS        200U    /* bwPollTimeout while the slot is verified */
#define DFU_NEW_RESET_WAIT_MS           2000U   /* Max wait for the host to read the final GETSTATUS */
#define DFU_NEW_RESET_DELAY_MS          100U

#define DFU_NEW_CHKSUM_LEN              4U
#define DFU_NEW_VERIFY_BUF_SIZE         4096U
#define DFU_NEW_INVALIDATE_LEN          4096U

/* Image label at offset 0 of an AmebaPro2 firmware, checked on block 0 */
#define DFU_NEW_LABEL                   "RTL8735B"
#define DFU_NEW_LABEL_LEN               8U

/* NOR partition table: FW1/FW2 length words (see ota_flash_NOR). NAND is bounded by fwfs itself. */
#define DFU_NEW_NOR_FW1_LEN_ADDR        0x2064U
#define DFU_NEW_NOR_FW2_LEN_ADDR        0x2084U

#define DFU_NEW_CTRL_THREAD_STACK       1024U   /* words */
#define DFU_NEW_CTRL_THREAD_PRIORITY    (tskIDLE_PRIORITY + 4)
#define DFU_NEW_RESET_THREAD_STACK      512U
#define DFU_NEW_RESET_THREAD_PRIORITY   (tskIDLE_PRIORITY + 2)

/* Events of the control thread, the only context that runs usbd_init/deinit */
#define DFU_NEW_EVT_START               1U
#define DFU_NEW_EVT_STOP                2U
#define DFU_NEW_EVT_RECONF              3U

/* Private types -------------------------------------------------------------*/

typedef struct {
	void *fp;                           /* Target slot, open while a download session runs */
	const char *slot;                   /* "FW1" / "FW2" */
	u32 size;                           /* Bytes written in this session */
	u32 max_size;                       /* Slot size on NOR, 0 on NAND (fwfs bounds it) */
	u32 sum;                            /* Byte-sum of every byte received */
	u8 tail[DFU_NEW_CHKSUM_LEN];        /* Last 4 bytes received: the file checksum */
	u8 *head;                           /* Block 0 held back, written after verification */
	u32 head_len;
} dfu_new_ota_t;

/* Private function prototypes -----------------------------------------------*/

static int dfu_cb_init(void);
static int dfu_cb_deinit(void);
static int dfu_cb_write(u16 block_num, u8 *buf, u32 len);
static int dfu_cb_manifest(void);
static void dfu_cb_detach(void);
static void dfu_cb_reconf(void);

extern uint8_t hal_sys_get_ld_fw_idx(void);
extern void hal_sys_clear_prefw_boot_fail_sts(void);
extern void ota_platform_reset(void);

/* Private variables ---------------------------------------------------------*/

static const char *const TAG = "DFU";

static const usbd_config_t dfu_usb_cfg = {
	.speed = USB_SPEED_HIGH,
	.isr_priority = 4U,
	/* DFIFO total 2232 DWORD, resv 8 DWORD for DMA addr and EP0 fixed 256 DWORD; EP0 only here */
	.rx_fifo_depth = 1680U,
	.ptx_fifo_depth = {256U, 16U, 16U},
};

static const usbd_dfu_cb_t dfu_cb = {
	.init     = dfu_cb_init,
	.deinit   = dfu_cb_deinit,
	.write    = dfu_cb_write,
	.manifest = dfu_cb_manifest,
	.detach   = dfu_cb_detach,
	.reconf   = dfu_cb_reconf,
};

static const usbd_dfu_config_t dfu_class_cfg = {
	/* VID/PID/strings left 0/NULL: 0BDA:8D00 "Realtek DFU Device", same as AmebaPro3 */
	.xfer_size        = DFU_NEW_XFER_SIZE,
	.write_poll_ms    = DFU_NEW_WRITE_POLL_MS,
	.manifest_poll_ms = DFU_NEW_MANIFEST_POLL_MS,
	.dfu_mode_only    = DFU_NEW_DFU_MODE_ONLY,
};

static dfu_new_ota_t dfu_ota;
static QueueHandle_t dfu_ctrl_queue;
static SemaphoreHandle_t dfu_reset_sema;
static u8 dfu_usb_on;

/* Private functions ---------------------------------------------------------*/

/*
 * USB hand-over hooks. The USB core runs one class driver, so an application that keeps another
 * class on the USB port (e.g. usbd_uvc_new) overrides these: release() tears its class and
 * usbd_deinit() down before DFU calls usbd_init(), restore() brings it back after USBDFU=0.
 * Control thread context. A non-zero release() return keeps DFU stopped.
 */
__weak int usbd_dfu_new_usb_release(void)
{
	return 0;
}

__weak void usbd_dfu_new_usb_restore(void)
{
}

/* Zero the header of an open slot, so it is never selected by the bootloader */
static void dfu_ota_void_header(void *fp, const char *slot)
{
	u8 *buf = (u8 *)malloc(DFU_NEW_INVALIDATE_LEN);

	if (buf == NULL) {
		return;
	}
	memset(buf, 0, DFU_NEW_INVALIDATE_LEN);
	pfw_seek(fp, 0, SEEK_SET);
	pfw_write(fp, buf, DFU_NEW_INVALIDATE_LEN);
	free(buf);
	RTK_LOGS(TAG, RTK_LOG_WARN, "%s invalidated\n", slot);
}

/* Void a slot that has already been committed (closed) */
static void dfu_ota_invalidate(const char *slot)
{
	void *fp = pfw_open(slot, M_RAW | M_RDWR);

	if (fp != NULL) {
		dfu_ota_void_header(fp, slot);
		pfw_close(fp);
	}
}

/* End the running session. Its header was never written; it is also zeroed before the close
 * commits anything, so the slot cannot hold a bootable partial image. */
static void dfu_ota_abort(void)
{
	dfu_new_ota_t *ota = &dfu_ota;

	if (ota->fp != NULL) {
		dfu_ota_void_header(ota->fp, ota->slot);
		pfw_close(ota->fp);
		ota->fp = NULL;
		RTK_LOGS(TAG, RTK_LOG_WARN, "Session aborted after %u bytes\n", ota->size);
	}
}

/* Slot size from the NOR partition table, 0 when it cannot be bounded here (NAND) */
static u32 dfu_ota_slot_size(const char *slot)
{
	flash_t flash;
	u32 len = 0U;

	if (sys_get_boot_sel() != 0U) {
		return 0U;
	}
	device_mutex_lock(RT_DEV_LOCK_FLASH);
	flash_read_word(&flash, (strcmp(slot, "FW1") == 0) ? DFU_NEW_NOR_FW1_LEN_ADDR : DFU_NEW_NOR_FW2_LEN_ADDR, &len);
	device_mutex_unlock(RT_DEV_LOCK_FLASH);
	return len;
}

/* Block 0: open the slot that is not running */
static int dfu_ota_start(void)
{
	dfu_new_ota_t *ota = &dfu_ota;
	u8 cur = hal_sys_get_ld_fw_idx();

	dfu_ota_abort();

	if (cur == 1U) {
		ota->slot = "FW2";
	} else if (cur == 2U) {
		ota->slot = "FW1";
	} else {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Unknown running fw idx %u\n", cur);
		return HAL_ERR_PARA;
	}

	/* Block 0 is at most wTransferSize long */
	if (ota->head == NULL) {
		ota->head = (u8 *)malloc(DFU_NEW_XFER_SIZE);
		if (ota->head == NULL) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "No mem for header\n");
			return HAL_ERR_MEM;
		}
	}
	ota->head_len = 0U;

	ota->fp = pfw_open(ota->slot, M_RAW | M_CREATE);
	if (ota->fp == NULL) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Open %s fail\n", ota->slot);
		return HAL_ERR_HW;
	}
	ota->size = 0U;
	ota->sum = 0U;
	ota->max_size = dfu_ota_slot_size(ota->slot);
	memset(ota->tail, 0, sizeof(ota->tail));
	RTK_LOGS(TAG, RTK_LOG_INFO, "Running FW%u, update %s\n", cur, ota->slot);

	return HAL_OK;
}

/* Keep the last 4 bytes of the stream, they may straddle two blocks */
static void dfu_ota_track_tail(dfu_new_ota_t *ota, const u8 *buf, u32 len)
{
	u32 keep;

	if (len >= DFU_NEW_CHKSUM_LEN) {
		memcpy(ota->tail, buf + len - DFU_NEW_CHKSUM_LEN, DFU_NEW_CHKSUM_LEN);
	} else {
		keep = DFU_NEW_CHKSUM_LEN - len;
		memmove(ota->tail, ota->tail + len, keep);
		memcpy(ota->tail + keep, buf, len);
	}
}

/* Byte-sum of the slot content without the trailing checksum, read back from flash */
static int dfu_ota_verify(dfu_new_ota_t *ota)
{
	u32 expected = (u32)ota->tail[0] | ((u32)ota->tail[1] << 8) | ((u32)ota->tail[2] << 16) | ((u32)ota->tail[3] << 24);
	u32 remain = ota->size - DFU_NEW_CHKSUM_LEN;
	u32 sum = 0U;
	u32 rd;
	u32 i;
	void *fp;
	u8 *buf;

	buf = (u8 *)malloc(DFU_NEW_VERIFY_BUF_SIZE);
	if (buf == NULL) {
		return HAL_ERR_MEM;
	}
	fp = pfw_open(ota->slot, M_RAW | M_RDWR);
	if (fp == NULL) {
		free(buf);
		return HAL_ERR_HW;
	}

	while (remain > 0U) {
		rd = MIN(remain, DFU_NEW_VERIFY_BUF_SIZE);
		if (pfw_read(fp, buf, (int)rd) != (int)rd) {
			break;
		}
		for (i = 0U; i < rd; i++) {
			sum += buf[i];
		}
		remain -= rd;
	}
	pfw_close(fp);
	free(buf);

	RTK_LOGS(TAG, RTK_LOG_INFO, "Checksum file 0x%08x, flash 0x%08x\n", expected, sum);
	return ((remain == 0U) && (sum == expected)) ? HAL_OK : HAL_ERR_HW;
}

static int dfu_cb_init(void)
{
	return HAL_OK;
}

/* Task context: USB DFU stopped or re-enumerating */
static int dfu_cb_deinit(void)
{
	dfu_ota_abort();
	if (dfu_ota.head != NULL) {
		free(dfu_ota.head);
		dfu_ota.head = NULL;
	}
	return HAL_OK;
}

/* Class write task context: may block on flash erase/program */
static int dfu_cb_write(u16 block_num, u8 *buf, u32 len)
{
	dfu_new_ota_t *ota = &dfu_ota;
	int wr;

	u32 i;

	if (block_num == 0U) {
		if (dfu_ota_start() != HAL_OK) {
			return HAL_ERR_HW;
		}
		/* Reject anything that is not an AmebaPro2 firmware before touching the slot content */
		if ((len < DFU_NEW_LABEL_LEN) || (memcmp(buf, DFU_NEW_LABEL, DFU_NEW_LABEL_LEN) != 0)) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Not an AmebaPro2 firmware image (no %s label)\n", DFU_NEW_LABEL);
			dfu_ota_abort();
			return HAL_ERR_PARA;
		}
	}
	if (ota->fp == NULL) {
		return HAL_ERR_HW;
	}
	if ((ota->max_size != 0U) && ((ota->size + len) > ota->max_size)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Image exceeds %s size %u\n", ota->slot, ota->max_size);
		dfu_ota_abort();
		return HAL_ERR_PARA;
	}

	dfu_ota_track_tail(ota, buf, len);
	for (i = 0U; i < len; i++) {
		ota->sum += buf[i];
	}
	if (block_num == 0U) {
		/* Hold the header block back: the slot stays unbootable until manifest writes it */
		memcpy(ota->head, buf, len);
		ota->head_len = len;
		memset(buf, 0xFF, len);
	}

	/* NOR returns the written size, NAND returns 0 on success: only a negative value is a failure */
	wr = pfw_write(ota->fp, buf, (int)len);
	if (wr < 0) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Flash write fail at %u: %d\n", ota->size, wr);
		dfu_ota_abort();
		return HAL_ERR_HW;
	}
	ota->size += len;

	if ((block_num & 0xFFU) == 0U) {
		RTK_LOGS(TAG, RTK_LOG_INFO, "Written %u bytes\n", ota->size);
	}

	return HAL_OK;
}

/* Class manifest task context, after the zero-length DNLOAD */
static int dfu_cb_manifest(void)
{
	dfu_new_ota_t *ota = &dfu_ota;
	u32 expected = (u32)ota->tail[0] | ((u32)ota->tail[1] << 8) | ((u32)ota->tail[2] << 16) | ((u32)ota->tail[3] << 24);
	u32 received;
	int ret;

	if (ota->fp == NULL) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Manifest: no download session\n");
		return HAL_ERR_PARA;
	}

	/* 1. Checksum of the received data, before anything bootable reaches the slot */
	received = ota->sum - ota->tail[0] - ota->tail[1] - ota->tail[2] - ota->tail[3];
	RTK_LOGS(TAG, RTK_LOG_INFO, "Received %u bytes, checksum file 0x%08x, received 0x%08x\n", ota->size, expected, received);
	if ((ota->size <= (DFU_NEW_LABEL_LEN + DFU_NEW_CHKSUM_LEN)) || (received != expected)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Verify fail (received data), image not updated\n");
		dfu_ota_abort();
		return HAL_ERR_HW;
	}

	/* 2. Write the header block last, then close: on NAND the close commits the partition table */
	pfw_seek(ota->fp, 0, SEEK_SET);
	if (pfw_write(ota->fp, ota->head, (int)ota->head_len) < 0) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Write header fail, image not updated\n");
		dfu_ota_abort();
		return HAL_ERR_HW;
	}
	pfw_close(ota->fp);
	ota->fp = NULL;
	RTK_LOGS(TAG, RTK_LOG_INFO, "Header written, verify %s\n", ota->slot);

	/* 3. Read the committed slot back */
	ret = dfu_ota_verify(ota);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Verify fail, image not updated\n");
		dfu_ota_invalidate(ota->slot);
		return ret;
	}

	/* A slot that failed to load is recorded in an AON register (kept over warm/WDT reset) and the
	 * bootloader skips it until power-off. It now holds a verified image, so drop that record;
	 * should the new image fail to load, the bootloader records it again and falls back. */
	hal_sys_clear_prefw_boot_fail_sts();

	RTK_LOGS(TAG, RTK_LOG_INFO, "Verify OK, ready to reboot\n");
	xSemaphoreGive(dfu_reset_sema);
	return HAL_OK;
}

/* ISR context */
static void dfu_cb_detach(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "DFU_DETACH\n");
}

/* Class reconf task context: hand the re-enumeration to the control thread */
static void dfu_cb_reconf(void)
{
	u8 evt = DFU_NEW_EVT_RECONF;

	xQueueSend(dfu_ctrl_queue, &evt, 0);
}

static int dfu_usb_init(void)
{
	int ret;

	ret = usbd_init(&dfu_usb_cfg);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_init fail: %d\n", ret);
		return ret;
	}
	ret = usbd_dfu_init(&dfu_cb, &dfu_class_cfg);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_dfu_init fail: %d\n", ret);
		usbd_deinit();
	}
	return ret;
}

static void dfu_usb_deinit(void)
{
	usbd_dfu_deinit();
	usbd_deinit();
}

/* Serializes start/stop and the DFU_DETACH re-enumeration */
static void dfu_ctrl_thread(void *param)
{
	u8 evt;

	UNUSED(param);
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	/* usbd_dfu_deinit() runs dfu_cb_deinit(), which may void a slot in flash */
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	for (;;) {
		if (xQueueReceive(dfu_ctrl_queue, &evt, portMAX_DELAY) != pdTRUE) {
			continue;
		}
		switch (evt) {
		case DFU_NEW_EVT_START:
			if (dfu_usb_on != 0U) {
				RTK_LOGS(TAG, RTK_LOG_WARN, "Already started\n");
			} else if (usbd_dfu_new_usb_release() != 0) {
				RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_init fail: USB busy\n");
			} else if (dfu_usb_init() == HAL_OK) {
				dfu_usb_on = 1U;
				RTK_LOGS(TAG, RTK_LOG_INFO, "Started, run: dfu-util -d 0bda:8d00 -D ota.bin\n");
			} else {
				usbd_dfu_new_usb_restore();
			}
			break;
		case DFU_NEW_EVT_STOP:
			if (dfu_usb_on == 0U) {
				RTK_LOGS(TAG, RTK_LOG_WARN, "Already stopped\n");
			} else {
				dfu_usb_deinit();
				usbd_dfu_reset_mode();   /* Next start begins in Run-Time mode */
				dfu_usb_on = 0U;
				RTK_LOGS(TAG, RTK_LOG_INFO, "Stopped\n");
				usbd_dfu_new_usb_restore();
			}
			break;
		case DFU_NEW_EVT_RECONF:
			if (dfu_usb_on != 0U) {
				RTK_LOGS(TAG, RTK_LOG_INFO, "Re-enumerating\n");
				dfu_usb_deinit();
				if (dfu_usb_init() != HAL_OK) {
					dfu_usb_on = 0U;
				}
			}
			break;
		default:
			break;
		}
	}
}

/* Reboots into the new image once the host has read the final GETSTATUS */
static void dfu_reset_thread(void *param)
{
	u32 wait_ms;
	u8 st;

	UNUSED(param);
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	for (;;) {
		if (xSemaphoreTake(dfu_reset_sema, portMAX_DELAY) != pdTRUE) {
			continue;
		}
		for (wait_ms = 0U; wait_ms < DFU_NEW_RESET_WAIT_MS; wait_ms += 10U) {
			st = usbd_dfu_get_state();
			if ((st != USB_DFU_STATE_MANIFEST) && (st != USB_DFU_STATE_MANIFEST_SYNC)) {
				break;
			}
			vTaskDelay(pdMS_TO_TICKS(10U));
		}
		RTK_LOGS(TAG, RTK_LOG_INFO, "Reboot\n");
		vTaskDelay(pdMS_TO_TICKS(DFU_NEW_RESET_DELAY_MS));
		ota_platform_reset();
	}
}

/* Console: USBDFU=<1|0>  1: start USB DFU, 0: stop USB DFU */
static void USBDFU(void *arg)
{
	int argc;
	char *argv[MAX_ARGC] = {0};
	u8 evt;

	argc = parse_param((char *)arg, argv);
	if ((argc != 2) || ((strcmp(argv[1], "0") != 0) && (strcmp(argv[1], "1") != 0))) {
		RTK_LOGS(TAG, RTK_LOG_INFO, "Usage: USBDFU=<1|0>  1: start USB DFU, 0: stop USB DFU\n");
		return;
	}
	evt = (argv[1][0] == '1') ? DFU_NEW_EVT_START : DFU_NEW_EVT_STOP;
	xQueueSend(dfu_ctrl_queue, &evt, 0);
}

static log_item_t dfu_cmd_items[] = {
	{"USBDFU", USBDFU,},
};

static void example_usbd_dfu_new_thread(void *param)
{
	u8 evt = DFU_NEW_EVT_START;

	UNUSED(param);

	dfu_ctrl_queue = xQueueCreate(4, sizeof(u8));
	dfu_reset_sema = xSemaphoreCreateBinary();
	if ((dfu_ctrl_queue == NULL) || (dfu_reset_sema == NULL)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "No mem\n");
		goto exit;
	}

	if ((xTaskCreate(dfu_ctrl_thread, "usbd_dfu_ctrl", DFU_NEW_CTRL_THREAD_STACK, NULL,
					 DFU_NEW_CTRL_THREAD_PRIORITY, NULL) != pdPASS) ||
		(xTaskCreate(dfu_reset_thread, "usbd_dfu_reset", DFU_NEW_RESET_THREAD_STACK, NULL,
					 DFU_NEW_RESET_THREAD_PRIORITY, NULL) != pdPASS)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create thread fail\n");
		goto exit;
	}

	log_service_add_table(dfu_cmd_items, sizeof(dfu_cmd_items) / sizeof(dfu_cmd_items[0]));
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD DFU demo start, USBDFU=0/1 to stop/start\n");

#if DFU_NEW_AUTO_START
	/* Start USB DFU right away, like the legacy usb_dfu_ota example */
	xQueueSend(dfu_ctrl_queue, &evt, 0);
#else
	(void)evt;
#endif

exit:
	vTaskDelete(NULL);
}

/* Exported functions --------------------------------------------------------*/

void example_usbd_dfu_new(void)
{
	if (xTaskCreate(example_usbd_dfu_new_thread, "usbd_dfu_new_init", 1024, NULL,
					tskIDLE_PRIORITY + 5, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create USBD DFU thread fail\n");
	}
}
