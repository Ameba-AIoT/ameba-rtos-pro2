/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Uses the selected ported stack's host MSC API.
 *
 * task/semaphore handling below calls FreeRTOS directly instead of
 * component/os/os_wrapper (rtos_*), the same way component/example/
 * the other ported host examples already do.
 *
 * Disk registration uses this project's own
 * component/file_system/fatfs/fatfs_ext/inc/ff_driver.h
 * (FATFS_RegisterDiskDriver/FATFS_UnRegisterDiskDriver + ll_diskio_drv) -
 * the upstream example's "vfs_fatfs.h" does not exist here; see
 * component/usb/usb_stack/<version>/host/msc/usbh_msc_disk.h.
 *
 * The usbh_config_t / FIFO depth values below are configured for the
 * amebapro2 (rtl8735b) USB OTG controller.
 */

/* Includes ------------------------------------------------------------------ */

#include <string.h>
#include <stdlib.h>
#include "ff.h"
#include "usbh_msc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Private defines -----------------------------------------------------------*/
static const char *const TAG = "MSC";

/* Hotplug switch
 *   0 = replug loop:  test files -> stop -> wait for replug -> repeat
 *                      USB stack stays alive; reconnect is handled by the
 *                      core without tearing down the stack
 *   1 = continuous:   test files -> repeat immediately; on detach the
 *                      stack is fully torn down and rebuilt to guarantee
 *                      clean transfer state across plug/unplug cycles
 */
#define CONFIG_USBH_MSC_HOTPLUG              0

// Thread priorities
#define USBH_MSC_INIT_THREAD_PRIORITY        (tskIDLE_PRIORITY + 2)
#define USBH_MSC_MAIN_TASK_PRIORITY          (tskIDLE_PRIORITY + 3)
#define USBH_MSC_HOTPLUG_THREAD_PRIORITY     (tskIDLE_PRIORITY + 2)

// Thread stack sizes
#define USBH_MSC_INIT_THREAD_STACK_SIZE      2048U
#define USBH_MSC_HOTPLUG_THREAD_STACK_SIZE   1024U

#define USBH_MSC_TEST_BUF_SIZE               4096
#define USBH_MSC_TEST_ROUNDS                 20
#define USBH_MSC_CHECK_DATA                  0
#define USBH_MSC_MAX_FILES                   10

/* Private types -------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

static int msc_cb_attach(void);
static int msc_cb_detach(void);
static int msc_cb_setup(void);
static int msc_cb_process(usb_host_t *host, u8 msg);

/* Private variables ---------------------------------------------------------*/

static SemaphoreHandle_t msc_attach_sema;
static volatile int msc_is_connected = 0;
static volatile int msc_is_ready = 0;
static u32 filenum = 0;
static u8 *msc_wt_buf;
static u8 *msc_rd_buf;

#if CONFIG_USBH_MSC_HOTPLUG
static SemaphoreHandle_t msc_detach_sema;
#endif

static const usbh_config_t usbh_cfg = {
	.speed = USB_SPEED_HIGH,
	.ext_intr_enable = USBH_SOF_INTR,
	/* 4U matches component/example/usbh_cdc_ecm_new's validated value. */
	.isr_priority = 4U,
	/* Left unset (0) originally, which the core clamps to its
	 * USBH_MAIN_TASK_MIN_STACK_SIZE (1024) - too small for MSC's attach/setup
	 * path (pipe open + GetMaxLUN + the Inquiry/TestUnitReady/ReadCapacity
	 * SCSI state machine), causing a stack-overflow UsageFault right after
	 * "Device connect". 1280 matches usbh_cdc_ecm_new's already-working value. */
	.main_task_stack_size = 1280U,
	.main_task_priority = USBH_MSC_MAIN_TASK_PRIORITY,
	.tick_source = USBH_SOF_TICK,
	.class_num = 1U,   /* MSC only */
	/* DFIFO total 2232 DWORD, resv 8 DWORD for DMA addr. */
	.rx_fifo_depth = 1712U,
	.nptx_fifo_depth = 256U,
	.ptx_fifo_depth = 256U,
};

static const usbh_msc_cb_t msc_usr_cb = {
	.attach = msc_cb_attach,
	.detach = msc_cb_detach,
	.setup = msc_cb_setup,
};

static const usbh_user_cb_t usbh_usr_cb = {
	.process = msc_cb_process
};

/* Private functions ---------------------------------------------------------*/

static int msc_cb_attach(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "ATTACH\n");
	msc_is_connected = 1;
	xSemaphoreGive(msc_attach_sema);
	return HAL_OK;
}

static int msc_cb_detach(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "DETACH\n");
#if CONFIG_USBH_MSC_HOTPLUG
	xSemaphoreGive(msc_detach_sema);
#endif
	return HAL_OK;
}

static int msc_cb_setup(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "SETUP\n");
	msc_is_ready = 1;
	return HAL_OK;
}

static int msc_cb_process(usb_host_t *host, u8 msg)
{
	UNUSED(host);

	switch (msg) {
	case USBH_MSG_DISCONNECTED:
		msc_is_connected = 0;
		msc_is_ready = 0;
		break;
	case USBH_MSG_CONNECTED:
		break;
	default:
		break;
	}

	return HAL_OK;
}

/*  I/O test routine (10 files, each with W/R of multiple sizes) */
static int usbh_msc_file_test(void)
{
	/* FATFS/FIL each embed a FF_MAX_SS (4096-byte) buffer (win[]/buf[]) - as
	 * locals these two alone need >8KB of stack, blowing well past this
	 * function's caller's task stack (example_usbh_msc_thread,
	 * USBH_MSC_INIT_THREAD_STACK_SIZE = 2048 words = 8KB) the moment this
	 * function is entered. This was the actual cause of a stack-overflow
	 * UsageFault seen right after usbh_start() - static moves them off the
	 * task stack entirely (safe: this function only ever runs sequentially
	 * on one task, never reentered/concurrent). */
	static FATFS fs;
	static FIL f;
	int drv_num = -1;
	FRESULT res;
	char logical_drv[4];
	char path[64] = {0};
	int ret = HAL_OK;
	UINT br;
	UINT bw;
	u32 round = 0;
	u32 start;
	u32 elapse;
	u32 perf = 0;
	u32 test_sizes[] = {512, 1024, 2048, 4096};
	u32 test_size;
	u32 i;
	u8 data;

	/* Wait for device attach + MSC ready */
	RTK_LOGS(TAG, RTK_LOG_INFO, "Wait for device attach...\n");
	if (xSemaphoreTake(msc_attach_sema, portMAX_DELAY) != pdTRUE) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Take attach sema fail\n");
		return HAL_ERR_UNKNOWN;
	}

	while (!msc_is_ready) {
		if (!msc_is_connected) {
			RTK_LOGS(TAG, RTK_LOG_WARN, "Device disconnected before ready\n");
			return HAL_ERR_UNKNOWN;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	RTK_LOGS(TAG, RTK_LOG_INFO, "Device ready\n");

	/* Allocate test buffers lazily (after attach) so no device attached
	 * means no heap held for them. */
	msc_wt_buf = (u8 *)pvPortMalloc(USBH_MSC_TEST_BUF_SIZE);
	msc_rd_buf = (u8 *)pvPortMalloc(USBH_MSC_TEST_BUF_SIZE);
	if (!msc_wt_buf || !msc_rd_buf) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Fail to alloc test buf\n");
		if (msc_wt_buf) {
			vPortFree(msc_wt_buf);
			msc_wt_buf = NULL;
		}
		if (msc_rd_buf) {
			vPortFree(msc_rd_buf);
			msc_rd_buf = NULL;
		}
		return HAL_ERR_MEM;
	}

	/* Register disk driver and mount */
	drv_num = FATFS_RegisterDiskDriver(&USB_disk_Driver);
	if (drv_num < 0) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Fail to register disk driver\n");
		return HAL_ERR_UNKNOWN;
	}

	logical_drv[0] = (char)(drv_num + '0');
	logical_drv[1] = ':';
	logical_drv[2] = '/';
	logical_drv[3] = 0;
	strcpy(path, logical_drv);

	if (f_mount(&fs, logical_drv, 1) != FR_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Fail to mount logical drive\n");
		FATFS_UnRegisterDiskDriver((unsigned char)drv_num);
		return HAL_ERR_UNKNOWN;
	}

	RTK_LOGS(TAG, RTK_LOG_INFO, "FatFS USB W/R performance test start, free heap: 0x%08x\n",
			 (unsigned int)xPortGetFreeHeapSize());

	/* Write / read up to USBH_MSC_MAX_FILES files */
	for (filenum = 0; filenum < USBH_MSC_MAX_FILES; filenum++) {
#if CONFIG_USBH_MSC_HOTPLUG
		if (!msc_is_ready) {
			RTK_LOGS(TAG, RTK_LOG_WARN, "Device detached during test\n");
			ret = HAL_ERR_UNKNOWN;
			break;
		}
#endif
		sprintf(&path[3], "TEST%u.DAT", (unsigned int)filenum);
		RTK_LOGS(TAG, RTK_LOG_INFO, "Open file: %s\n", path);

		res = f_open(&f, path, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if (res) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Open file fail: %s, rc=%d\n", path, res);
			ret = HAL_ERR_UNKNOWN;
			break;
		}

		data = (u8)(rand() % 0xFF);
		memset(msc_wt_buf, data, USBH_MSC_TEST_BUF_SIZE);

		for (i = 0; i < sizeof(test_sizes) / sizeof(test_sizes[0]); ++i) {
			test_size = test_sizes[i];
			if (test_size > USBH_MSC_TEST_BUF_SIZE) {
				break;
			}

			/* Write */
			RTK_LOGS(TAG, RTK_LOG_INFO, "W test: size %u, round %d...\n", test_size, USBH_MSC_TEST_ROUNDS);
			start = xTaskGetTickCount();
			for (round = 0; round < USBH_MSC_TEST_ROUNDS; ++round) {
				res = f_write(&f, (void *)msc_wt_buf, test_size, &bw);
#if CONFIG_USBH_MSC_HOTPLUG
				if (res == FR_DISK_ERR) {
					RTK_LOGS(TAG, RTK_LOG_WARN, "W err: device disconnected (rc=%d)\n", res);
					break;
				}
#endif
				if (res || (bw < test_size)) {
					f_lseek(&f, 0);
					RTK_LOGS(TAG, RTK_LOG_ERROR, "W err bw=%u, rc=%d\n", bw, res);
					ret = HAL_ERR_UNKNOWN;
					break;
				}
			}
#if CONFIG_USBH_MSC_HOTPLUG
			if (res == FR_DISK_ERR) {
				break;
			}
#endif
			if (ret) {
				break;
			}

			elapse = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
			if (elapse == 0) {
				elapse = 1;
			}
			perf = (round * test_size * 10000 / 1024) / elapse;
			RTK_LOGS(TAG, RTK_LOG_INFO, "W rate %u.%u KB/s for %u round @ %u ms\n",
					 perf / 10, perf % 10, round, elapse);

			/* Read */
			f_lseek(&f, 0);
			RTK_LOGS(TAG, RTK_LOG_INFO, "R test: size = %u round = %d...\n", test_size, USBH_MSC_TEST_ROUNDS);
			start = xTaskGetTickCount();
			for (round = 0; round < USBH_MSC_TEST_ROUNDS; ++round) {
				res = f_read(&f, (void *)msc_rd_buf, test_size, &br);
#if CONFIG_USBH_MSC_HOTPLUG
				if (res == FR_DISK_ERR) {
					RTK_LOGS(TAG, RTK_LOG_WARN, "R err: device disconnected (rc=%d)\n", res);
					break;
				}
#endif
				if (res || (br < test_size)) {
					f_lseek(&f, 0);
					RTK_LOGS(TAG, RTK_LOG_ERROR, "R err br=%u, rc=%d\n", br, res);
					ret = HAL_ERR_UNKNOWN;
					break;
				}
			}
#if CONFIG_USBH_MSC_HOTPLUG
			if (res == FR_DISK_ERR) {
				break;
			}
#endif
			if (ret) {
				break;
			}

			elapse = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
			if (elapse == 0) {
				elapse = 1;
			}
			perf = (round * test_size * 10000 / 1024) / elapse;
			RTK_LOGS(TAG, RTK_LOG_INFO, "R rate %u.%u KB/s for %u round @ %u ms\n",
					 perf / 10, perf % 10, round, elapse);
			f_lseek(&f, 0);

#if USBH_MSC_CHECK_DATA
			if (!(memcmp(msc_wt_buf, msc_rd_buf, test_size) == 0)) {
				RTK_LOGS(TAG, RTK_LOG_ERROR, "WR%u check err: %u-%u-%u-%u vs %u-%u-%u-%u\n", test_size,
						 msc_wt_buf[0], msc_wt_buf[1], msc_wt_buf[test_size - 2], msc_wt_buf[test_size - 1],
						 msc_rd_buf[0], msc_rd_buf[1], msc_rd_buf[test_size - 2], msc_rd_buf[test_size - 1]);
				ret = HAL_ERR_UNKNOWN;
				break;
			}
#endif
		}

#if CONFIG_USBH_MSC_HOTPLUG
		if (res == FR_DISK_ERR) {
			f_close(&f);
			ret = HAL_ERR_UNKNOWN;
			break;
		}
#endif

		f_close(&f);
		if (ret) {
			break;
		}

		RTK_LOGS(TAG, RTK_LOG_INFO, "File %s done\n", path);
		vTaskDelay(pdMS_TO_TICKS(20));
	}

	/* Unmount & unregister */
	if (f_unmount(logical_drv) != FR_OK) {
		RTK_LOGS(TAG, RTK_LOG_WARN, "Unmount fail\n");
	}
	if (FATFS_UnRegisterDiskDriver((unsigned char)drv_num)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Unregister disk driver fail\n");
	}

	if (msc_wt_buf) {
		vPortFree(msc_wt_buf);
		msc_wt_buf = NULL;
	}
	if (msc_rd_buf) {
		vPortFree(msc_rd_buf);
		msc_rd_buf = NULL;
	}

	RTK_LOGS(TAG, RTK_LOG_INFO, "FatFS USB W/R performance test done, free heap: 0x%08x\n",
			 (unsigned int)xPortGetFreeHeapSize());
	return ret;
}

#if CONFIG_USBH_MSC_HOTPLUG
/* Hotplug thread: re-init USB stack after detach */
static void example_usbh_msc_hotplug_thread(void *param)
{
	int ret;

	UNUSED(param);

	for (;;) {
		if (xSemaphoreTake(msc_detach_sema, portMAX_DELAY) == pdTRUE) {
			RTK_LOGS(TAG, RTK_LOG_INFO, "Hotplug: deinit USB stack\n");
			vTaskDelay(pdMS_TO_TICKS(100));

			usbh_stop();
			usbh_msc_deinit();
			vTaskDelay(pdMS_TO_TICKS(20)); /* let USB main task flush its queue before deletion */
			usbh_deinit();

			/* Reset state and drain any transient attach event fired during
			 * the USB controller reset (power cycle causes a brief connect). */
			msc_is_connected = 0;
			msc_is_ready = 0;
			xSemaphoreTake(msc_attach_sema, 0);

			vTaskDelay(pdMS_TO_TICKS(10));

			RTK_LOGS(TAG, RTK_LOG_INFO, "Free heap: 0x%08x\n", (unsigned int)xPortGetFreeHeapSize());

			ret = usbh_init(&usbh_cfg, &usbh_usr_cb);
			if (ret != HAL_OK) {
				RTK_LOGS(TAG, RTK_LOG_ERROR, "Hotplug: USBH init fail\n");
				break;
			}

			ret = usbh_msc_init(&msc_usr_cb);
			if (ret != HAL_OK) {
				RTK_LOGS(TAG, RTK_LOG_ERROR, "Hotplug: MSC init fail\n");
				usbh_deinit();
				break;
			}

			/* Re-arm USB TRX after the re-init. */
			usbh_start();

			RTK_LOGS(TAG, RTK_LOG_INFO, "Hotplug: USB stack re-initialized\n");
		}
	}

	vTaskDelete(NULL);
}
#endif

/* Main test thread */
static void example_usbh_msc_thread(void *param)
{
	int ret;
#if CONFIG_USBH_MSC_HOTPLUG
	TaskHandle_t hotplug_task;
#endif

	UNUSED(param);

	msc_attach_sema = xSemaphoreCreateBinary();
#if CONFIG_USBH_MSC_HOTPLUG
	msc_detach_sema = xSemaphoreCreateBinary();
#endif

	/* Init USB host + MSC class driver */
	ret = usbh_init(&usbh_cfg, &usbh_usr_cb);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "USBH init fail\n");
		goto exit_free;
	}

	ret = usbh_msc_init(&msc_usr_cb);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "MSC init fail\n");
		usbh_deinit();
		goto exit_free;
	}

	/* All class drivers registered; start USB TRX so enumeration can run. */
	usbh_start();

#if CONFIG_USBH_MSC_HOTPLUG
	/* Start hotplug thread (waits on detach_sema, re-inits USB stack) */
	if (xTaskCreate(example_usbh_msc_hotplug_thread, "usbh_msc_hotplug", USBH_MSC_HOTPLUG_THREAD_STACK_SIZE,
					NULL, USBH_MSC_HOTPLUG_THREAD_PRIORITY, &hotplug_task) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create hotplug thread fail\n");
		usbh_msc_deinit();
		usbh_deinit();
		goto exit_free;
	}
#endif

	/* I/O test loop */
#if CONFIG_USBH_MSC_HOTPLUG
	for (;;) {
		ret = usbh_msc_file_test();
		if (ret == HAL_OK) {
			RTK_LOGS(TAG, RTK_LOG_INFO, "All files done, repeat from 0\n");
			/* Device still attached -- give sema so the next call does not
			 * block at xSemaphoreTake waiting for a new attach event. */
			xSemaphoreGive(msc_attach_sema);
		} else {
			RTK_LOGS(TAG, RTK_LOG_WARN, "I/O interrupted, wait for next attach\n");
			/* Hotplug thread tears down + rebuilds the stack and gives
			 * attach_sema after enumeration completes. */
		}
	}
#else
	/* Replug loop: test files, then wait for the next physical replug
	 * event before running again.  usbh_msc_file_test() blocks at
	 * xSemaphoreTake(msc_attach_sema) until msc_cb_attach fires. */
	for (;;) {
		ret = usbh_msc_file_test();
		RTK_LOGS(TAG, RTK_LOG_INFO, "Test %s, unplug and replug to repeat\n",
				 ret == HAL_OK ? "done" : "interrupted");
	}
#endif

exit_free:
	if (msc_wt_buf) {
		vPortFree(msc_wt_buf);
	}
	if (msc_rd_buf) {
		vPortFree(msc_rd_buf);
	}
	vTaskDelete(NULL);
}

/* Exported functions --------------------------------------------------------*/

void example_usbh_msc_new(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBH MSC demo start\n");

	if (xTaskCreate(example_usbh_msc_thread, "usbh_msc_new_init", USBH_MSC_INIT_THREAD_STACK_SIZE,
					NULL, USBH_MSC_INIT_THREAD_PRIORITY, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create thread fail\n");
	}
}
