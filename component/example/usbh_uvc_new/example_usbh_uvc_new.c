/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Uses the selected ported stack's host UVC API.
 *
 * Three application modes (select via CONFIG_USBH_UVC_APP):
 *   USBH_UVC_APP_EMPTY  — capture frames and discard, report throughput
 *   USBH_UVC_APP_FATFS  — capture raw JPEG/YUY2 frames and save to SD card
 *   USBH_UVC_APP_RGB    — capture frames, convert to RGB888, save as PPM to SD card
 *
 * task/semaphore handling calls FreeRTOS directly (no rtos_* / rtw_* wrappers),
 * consistent with the rest of the new-stack examples in this project.
 */

/* Includes ------------------------------------------------------------------ */

#include "usbh_uvc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Private defines -----------------------------------------------------------*/

/* Application mode selection */
#define USBH_UVC_APP_EMPTY   1   /* capture and discard, measure throughput */
#define USBH_UVC_APP_FATFS   2   /* save raw JPEG/YUY2 to SD card */
#define USBH_UVC_APP_RGB     3   /* convert to RGB888, save PPM to SD card */

/* Default mode when not set by cmake (-DUSBH_UVC_APP_MODE=EMPTY/FATFS/RGB). */
#ifndef CONFIG_USBH_UVC_APP
#define CONFIG_USBH_UVC_APP  USBH_UVC_APP_RGB
#endif

/* Max images to save before stopping (FATFS / RGB modes) */
#define USBH_UVC_MAX_IMAGES  500

/* Warmup frames to discard before saving (FATFS / RGB modes) */
#define USBH_UVC_WARMUP_FRAMES  30

/* Save every N-th captured frame (FATFS / RGB modes) */
#define USBH_UVC_SAVE_EVERY_N   15

/* Supported formats: USBH_UVC_FORMAT_MJPEG, USBH_UVC_FORMAT_YUV,
 * USBH_UVC_FORMAT_H264, USBH_UVC_FORMAT_H265. Verify which formats the
 * attached camera actually supports and adjust below accordingly. */
#define CONFIG_USBH_UVC_FORMAT_TYPE                USBH_UVC_FORMAT_MJPEG

/* Target resolution and compression ratio. If the camera does not support
 * these values, the host stack automatically selects the closest match -
 * always check the log output to confirm the actual parameters applied. */
#define CONFIG_USBH_UVC_WIDTH                      640U
#define CONFIG_USBH_UVC_HEIGHT                     480U
#define CONFIG_USBH_UVC_FRAME_RATE                 15U

/* Frame buffer size in bytes; increase if a "truncated" error appears in the
 * log (see usbh_uvc_stream.c: bytes = MIN(maxlen, payload_len)).
 * 640x480 YUY2 = 614400; 640x480 MJPEG typically well under 640 KB. */
#define CONFIG_USBH_UVC_FRAME_BUF_SIZE             (640U * 1024U)

/* Most cameras have a single video stream interface. */
#define CONFIG_USBH_UVC_STREAM_INDEX               0U

/* Re-init the USB host stack on hot-plug (detach/attach). */
#define CONFIG_USBH_UVC_HOTPLUG                    1

/* Check MJPEG SOI/EOI markers. */
#if (CONFIG_USBH_UVC_FORMAT_TYPE == USBH_UVC_FORMAT_MJPEG)
#define CONFIG_USBH_UVC_CHECK_MJPEG_DATA           1
#else
#define CONFIG_USBH_UVC_CHECK_MJPEG_DATA           0
#endif

/* EMPTY mode: number of frames to capture per round */
#define CONFIG_USBH_UVC_LOOP                       200U

/* EMPTY mode: max consecutive get_frame() failures before stopping */
#define CONFIG_USBH_UVC_MAX_FAIL_COUNT             5U

#define UVC_INIT_THREAD_STACK_SIZE                 768U
#define UVC_INIT_THREAD_PRIORITY                   (tskIDLE_PRIORITY + 1)
#define UVC_MAIN_TASK_STACK_SIZE                   1024U
#define UVC_MAIN_TASK_PRIORITY                     (tskIDLE_PRIORITY + 3)
#define UVC_TEST_THREAD_STACK_SIZE                 1024U
#define UVC_TEST_THREAD_PRIORITY                   (tskIDLE_PRIORITY + 4)
#define UVC_HOTPLUG_THREAD_STACK_SIZE              768U
#define UVC_HOTPLUG_THREAD_PRIORITY                (tskIDLE_PRIORITY + 6)
#define UVC_FATFS_THREAD_STACK_SIZE                2048U
#define UVC_FATFS_THREAD_PRIORITY                  (tskIDLE_PRIORITY + 2)

/* Private conditional includes ----------------------------------------------*/

#if ((CONFIG_USBH_UVC_APP == USBH_UVC_APP_FATFS) || (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB))
#include "ff.h"
#include "fatfs_sdcard_api.h"
#endif

#if (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB)
#include "uvc_to_rgb.h"
#endif

/* Private function prototypes -----------------------------------------------*/

static int uvc_cb_init(void);
static int uvc_cb_deinit(void);
static int uvc_cb_attach(void);
static int uvc_cb_detach(void);
static int uvc_cb_setup(void);
static int uvc_cb_setparam(int status);

/* Private variables ---------------------------------------------------------*/

static const char *const TAG = "UVC";

static SemaphoreHandle_t uvc_attach_sema;
static SemaphoreHandle_t uvc_start_sema;
static SemaphoreHandle_t uvc_setparam_sema;
#if CONFIG_USBH_UVC_HOTPLUG
static SemaphoreHandle_t uvc_detach_sema;
#endif

static usbh_uvc_s_ctx_t uvc_s_ctx;
static TaskHandle_t uvc_task;
static u8 uvc_setparam_status;
static volatile u8 uvc_task_exiting;

/* EMPTY mode: throughput counters */
static u64 rx_total_bytes;
static u32 rx_start;

/* FATFS / RGB mode: shared frame buffer and synchronisation */
#if ((CONFIG_USBH_UVC_APP == USBH_UVC_APP_FATFS) || (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB))
PSRAM_BSS_SECTION static u8 uvc_save_buf[CONFIG_USBH_UVC_FRAME_BUF_SIZE];
static u32 uvc_save_buf_size;
static SemaphoreHandle_t uvc_buf_mutex;
static SemaphoreHandle_t uvc_save_sema;
static volatile int uvc_fatfs_is_init;
static volatile int uvc_save_busy;
static int uvc_img_file_no;
#endif

/* RGB mode: converted RGB888 output buffer */
#if (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB)
PSRAM_BSS_SECTION static u8 uvc_rgb_buf[CONFIG_USBH_UVC_WIDTH * CONFIG_USBH_UVC_HEIGHT * 3];
#endif

static const usbh_config_t usbh_cfg = {
	.speed = USB_SPEED_HIGH,
	.ext_intr_enable = USBH_SOF_INTR,
	.isr_priority = 4U,
	.main_task_stack_size = UVC_MAIN_TASK_STACK_SIZE,
	.main_task_priority = tskIDLE_PRIORITY + 3,
	.tick_source = USBH_SOF_TICK,
	.class_num = 1U,
	/* DFIFO total 2232 DWORD, resv 8 DWORD for DMA addr. */
	.rx_fifo_depth = 1712U,
	.nptx_fifo_depth = 256U,
	.ptx_fifo_depth = 256U,
};

static const usbh_uvc_ctx_t uvc_cfg = {
	.frame_buf_size = CONFIG_USBH_UVC_FRAME_BUF_SIZE,
};

static const usbh_uvc_cb_t uvc_cb = {
	.init = uvc_cb_init,
	.deinit = uvc_cb_deinit,
	.attach = uvc_cb_attach,
	.detach = uvc_cb_detach,
	.setup = uvc_cb_setup,
	.set_param = uvc_cb_setparam,
};

/* Private functions ---------------------------------------------------------*/

static int uvc_cb_init(void)
{
	return HAL_OK;
}

static int uvc_cb_deinit(void)
{
	return HAL_OK;
}

/** @note Called in ISR context - no malloc/blocking calls. */
static int uvc_cb_attach(void)
{
	BaseType_t woken = pdFALSE;

	xSemaphoreGiveFromISR(uvc_attach_sema, &woken);
	portYIELD_FROM_ISR(woken);
	return HAL_OK;
}

/** @note Called in ISR context - no malloc/blocking calls. */
static int uvc_cb_detach(void)
{
#if CONFIG_USBH_UVC_HOTPLUG
	BaseType_t woken = pdFALSE;

	xSemaphoreGiveFromISR(uvc_detach_sema, &woken);
	portYIELD_FROM_ISR(woken);
#endif
	return HAL_OK;
}

/** @note Called in ISR context - no malloc/blocking calls. */
static int uvc_cb_setup(void)
{
	BaseType_t woken = pdFALSE;

	xSemaphoreGiveFromISR(uvc_start_sema, &woken);
	portYIELD_FROM_ISR(woken);
	return HAL_OK;
}

/** @note Called in ISR context - no malloc/blocking calls. */
static int uvc_cb_setparam(int status)
{
	BaseType_t woken = pdFALSE;

	uvc_setparam_status = (u8)status;
	xSemaphoreGiveFromISR(uvc_setparam_sema, &woken);
	portYIELD_FROM_ISR(woken);
	return HAL_OK;
}

/* EMPTY mode helpers --------------------------------------------------------*/

static void uvc_calculate_tp(u32 loop)
{
	u32 rx_elapse = usb_os_get_timestamp_ms() - rx_start;
	u32 rx_fps, rx_kbps, rx_mbps_x100;

	if (rx_elapse == 0U) {
		rx_elapse = 1U;
	}

	rx_fps      = loop * 1000U / rx_elapse;
	rx_kbps     = (u32)(rx_total_bytes * 1000U / 1024U / rx_elapse);
	rx_mbps_x100 = (u32)(rx_total_bytes * 100000U / (1024U * 1024U) / rx_elapse);

	RTK_LOGS(TAG, RTK_LOG_INFO, "TP %u KB/s @ %u ms, fps %u/s\n", rx_kbps, rx_elapse, rx_fps);
	RTK_LOGS(TAG, RTK_LOG_INFO, "TP %u.%02u MB/s (%u bytes)\n",
			 rx_mbps_x100 / 100U, rx_mbps_x100 % 100U, (u32)rx_total_bytes);

	rx_total_bytes = 0U;
}

static void usbh_uvc_img_check(usbh_uvc_frame_t *frame)
{
#if CONFIG_USBH_UVC_CHECK_MJPEG_DATA
	u32 len = frame->byteused;

	if (len == 0U) {
		return;
	}
	while ((len > 2U) && (frame->buf[len - 1U] == 0U)) {
		len--;
	}
	if ((frame->buf[0] != 0xffU) || (frame->buf[1] != 0xd8U) ||
		(frame->buf[len - 2U] != 0xffU) || (frame->buf[len - 1U] != 0xd9U)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "[mjpeg] image error: %x %x %x %x\n",
				 frame->buf[0], frame->buf[1], frame->buf[2], frame->buf[3]);
	}
#else
	UNUSED(frame);
#endif
}

/* FATFS / RGB mode helpers --------------------------------------------------*/

#if ((CONFIG_USBH_UVC_APP == USBH_UVC_APP_FATFS) || (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB))

static void uvc_fatfs_thread(void *param)
{
	FRESULT res;
	static fatfs_sd_params_t fatfs_sd;
	static FIL m_file;
	char filename[64];
	char path[128];
	UINT bw;

	UNUSED(param);

	RTK_LOGS(TAG, RTK_LOG_INFO, "Init FATFS SD...\n");

	res = fatfs_sd_init();
	if (res < 0) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "fatfs_sd_init fail: %d\n", res);
		goto fail;
	}

	fatfs_sd_get_param(&fatfs_sd);
	RTK_LOGS(TAG, RTK_LOG_INFO, "SD mounted at: %s\n", fatfs_sd.drv);

	uvc_fatfs_is_init = 1;

	while (uvc_fatfs_is_init) {
		if (xSemaphoreTake(uvc_save_sema, portMAX_DELAY) != pdTRUE) {
			continue;
		}
		if (!uvc_fatfs_is_init) {
			break;
		}

#if (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB)
		{
			int w = (int)CONFIG_USBH_UVC_WIDTH;
			int h = (int)CONFIG_USBH_UVC_HEIGHT;
			uint8_t *rgb;

			if (xSemaphoreTake(uvc_buf_mutex, portMAX_DELAY) != pdTRUE) {
				uvc_save_busy = 0;
				continue;
			}

			if (CONFIG_USBH_UVC_FORMAT_TYPE == USBH_UVC_FORMAT_MJPEG) {
				rgb = uvc_jpeg_to_rgb(uvc_save_buf, (int)uvc_save_buf_size, &w, &h,
									  uvc_rgb_buf, UVC_RGB_FORMAT_RGB888);
				if (!rgb) {
					RTK_LOGS(TAG, RTK_LOG_ERROR, "JPEG decode fail (len=%u)\n", uvc_save_buf_size);
					xSemaphoreGive(uvc_buf_mutex);
					uvc_save_busy = 0;
					continue;
				}
				RTK_LOGS(TAG, RTK_LOG_INFO, "JPEG decoded: %dx%d\n", w, h);
			} else {
				rgb = uvc_yuy2_to_rgb(uvc_save_buf, w, h,
									  uvc_rgb_buf, UVC_RGB_FORMAT_RGB888);
				if (!rgb) {
					RTK_LOGS(TAG, RTK_LOG_ERROR, "YUY2 convert fail (len=%u)\n", uvc_save_buf_size);
					xSemaphoreGive(uvc_buf_mutex);
					uvc_save_busy = 0;
					continue;
				}
				RTK_LOGS(TAG, RTK_LOG_INFO, "YUY2 converted: %dx%d\n", w, h);
			}

			xSemaphoreGive(uvc_buf_mutex);

			snprintf(filename, sizeof(filename), "img%d.ppm", uvc_img_file_no);
			snprintf(path, sizeof(path), "%s%s", fatfs_sd.drv, filename);

			res = f_open(&m_file, path, FA_CREATE_ALWAYS | FA_WRITE);
			if (res) {
				RTK_LOGS(TAG, RTK_LOG_ERROR, "f_open fail (%s): %d\n", path, res);
				uvc_save_busy = 0;
				continue;
			}

			{
				char header[64];
				int hdr_len = snprintf(header, sizeof(header), "P6\n%d %d\n255\n", w, h);
				f_write(&m_file, header, (UINT)hdr_len, &bw);
			}
			res = f_write(&m_file, uvc_rgb_buf, (UINT)(w * h * 3), &bw);
			if (res) {
				RTK_LOGS(TAG, RTK_LOG_ERROR, "f_write PPM fail: %d\n", res);
			} else {
				RTK_LOGS(TAG, RTK_LOG_INFO, "Saved %s (%u bytes RGB)\n", filename, bw);
			}
			f_close(&m_file);
		}
#else /* USBH_UVC_APP_FATFS */
		{
			if (CONFIG_USBH_UVC_FORMAT_TYPE == USBH_UVC_FORMAT_YUV) {
				snprintf(filename, sizeof(filename), "img%d.yuy2", uvc_img_file_no);
			} else {
				snprintf(filename, sizeof(filename), "img%d.jpg", uvc_img_file_no);
			}
			snprintf(path, sizeof(path), "%s%s", fatfs_sd.drv, filename);

			res = f_open(&m_file, path, FA_CREATE_ALWAYS | FA_WRITE);
			if (res) {
				RTK_LOGS(TAG, RTK_LOG_ERROR, "f_open fail (%s): %d\n", path, res);
				uvc_save_busy = 0;
				continue;
			}

			if (xSemaphoreTake(uvc_buf_mutex, portMAX_DELAY) == pdTRUE) {
				res = f_write(&m_file, uvc_save_buf, (UINT)uvc_save_buf_size, &bw);
				xSemaphoreGive(uvc_buf_mutex);
			}
			f_close(&m_file);

			if (res) {
				RTK_LOGS(TAG, RTK_LOG_ERROR, "f_write fail: %d\n", res);
			} else {
				RTK_LOGS(TAG, RTK_LOG_INFO, "Saved %s (%u bytes)\n", filename, bw);
			}
		}
#endif /* USBH_UVC_APP_FATFS */

		uvc_img_file_no++;
		uvc_save_busy = 0;

		if (uvc_img_file_no >= USBH_UVC_MAX_IMAGES) {
			RTK_LOGS(TAG, RTK_LOG_INFO, "Reached %d images, stopping\n", USBH_UVC_MAX_IMAGES);
			break;
		}
	}

	fatfs_sd_close();

fail:
	uvc_fatfs_is_init = 0;
	vTaskDelete(NULL);
}

/**
  * @brief  Copy frame to shared buffer and wake the FATFS save thread.
  *         Called from the capture loop; must not block for long.
  */
static void uvc_prepare_save(usbh_uvc_frame_t *frame)
{
	static int warmup_cnt;
	static int frame_cnt;

	if (!frame || frame->byteused == 0U) {
		return;
	}
	if (frame->byteused > CONFIG_USBH_UVC_FRAME_BUF_SIZE) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Frame too large: %u\n", frame->byteused);
		return;
	}

	warmup_cnt++;
	if (warmup_cnt <= USBH_UVC_WARMUP_FRAMES) {
		return;
	}

	frame_cnt++;
	if ((frame_cnt % USBH_UVC_SAVE_EVERY_N) != 0) {
		return;
	}

	if (!uvc_fatfs_is_init || uvc_save_busy) {
		return;
	}

	if (xSemaphoreTake(uvc_buf_mutex, 0) == pdTRUE) {
		memcpy(uvc_save_buf, frame->buf, frame->byteused);
		uvc_save_buf_size = frame->byteused;
		xSemaphoreGive(uvc_buf_mutex);

		uvc_save_busy = 1;
		xSemaphoreGive(uvc_save_sema);
	}
}

#endif /* FATFS || RGB */

/* Main capture test task ----------------------------------------------------*/

static void example_usbh_uvc_test(void *param)
{
	usbh_uvc_frame_t *buf;
	const char *fmt_name = NULL;
	int ret;

	UNUSED(param);

#if ((CONFIG_USBH_UVC_APP == USBH_UVC_APP_FATFS) || (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB))
	/* Start FATFS save thread once. */
	if (xTaskCreate(uvc_fatfs_thread, "uvc_fatfs", UVC_FATFS_THREAD_STACK_SIZE,
					NULL, UVC_FATFS_THREAD_PRIORITY, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create FATFS thread fail\n");
		goto exit;
	}
	/* Wait for SD card initialisation to finish before streaming. */
	while (!uvc_fatfs_is_init) {
		vTaskDelay(pdMS_TO_TICKS(100));
	}
#endif

	while (uvc_task_exiting == 0U) {
		if (xSemaphoreTake(uvc_start_sema, portMAX_DELAY) != pdTRUE) {
			continue;
		}
		if (uvc_task_exiting != 0U) {
			goto exit;
		}

		uvc_s_ctx.fmt_type  = CONFIG_USBH_UVC_FORMAT_TYPE;
		uvc_s_ctx.width     = CONFIG_USBH_UVC_WIDTH;
		uvc_s_ctx.height    = CONFIG_USBH_UVC_HEIGHT;
		uvc_s_ctx.frame_rate = CONFIG_USBH_UVC_FRAME_RATE;

		switch (uvc_s_ctx.fmt_type) {
		case USBH_UVC_FORMAT_MJPEG:
			fmt_name = "MJPEG";
			break;
		case USBH_UVC_FORMAT_H264:
			fmt_name = "H264";
			break;
		case USBH_UVC_FORMAT_YUV:
			fmt_name = "YUV";
			break;
		case USBH_UVC_FORMAT_H265:
			fmt_name = "H265";
			break;
		default:
			fmt_name = "OTHER";
			break;
		}

		ret = usbh_uvc_set_param(&uvc_s_ctx, CONFIG_USBH_UVC_STREAM_INDEX);
		if (ret != HAL_OK) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Set param req: %d\n", ret);
			goto exit;
		}

		if (xSemaphoreTake(uvc_setparam_sema, pdMS_TO_TICKS(5000)) == pdTRUE) {
			if (uvc_setparam_status != HAL_OK) {
				RTK_LOGS(TAG, RTK_LOG_ERROR, "Set paras err: %s %ux%u@%ufps status=%d\n",
						 fmt_name, uvc_s_ctx.width, uvc_s_ctx.height,
						 uvc_s_ctx.frame_rate, uvc_setparam_status);
				goto exit;
			}
			RTK_LOGS(TAG, RTK_LOG_INFO, "Set paras ok: %s %ux%u@%ufps\n",
					 fmt_name, uvc_s_ctx.width, uvc_s_ctx.height, uvc_s_ctx.frame_rate);
		} else {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Set paras timeout\n");
			goto exit;
		}

		usbh_uvc_start(CONFIG_USBH_UVC_STREAM_INDEX);

#if (CONFIG_USBH_UVC_APP == USBH_UVC_APP_EMPTY)
		{
			u32 img_cnt = 0U;
			u32 fail_cnt = 0U;

			rx_start = usb_os_get_timestamp_ms();

			while ((img_cnt < CONFIG_USBH_UVC_LOOP) && (uvc_task_exiting == 0U)) {
				buf = usbh_uvc_get_frame(CONFIG_USBH_UVC_STREAM_INDEX);
				if (buf == NULL) {
					if (++fail_cnt >= CONFIG_USBH_UVC_MAX_FAIL_COUNT) {
						RTK_LOGS(TAG, RTK_LOG_INFO, "Stop (fail:%u)\n", fail_cnt);
						break;
					}
					vTaskDelay(pdMS_TO_TICKS(1));
					continue;
				}
				if (buf->byteused >= CONFIG_USBH_UVC_FRAME_BUF_SIZE) {
					usbh_uvc_put_frame(buf, CONFIG_USBH_UVC_STREAM_INDEX);
					RTK_LOGS(TAG, RTK_LOG_ERROR,
							 "Frame %u truncated: len %u >= buf %u, increase CONFIG_USBH_UVC_FRAME_BUF_SIZE\n",
							 img_cnt, buf->byteused, CONFIG_USBH_UVC_FRAME_BUF_SIZE);
					goto exit;
				}
				if (buf->byteused > 0U) {
					rx_total_bytes += buf->byteused;
					RTK_LOGS(TAG, RTK_LOG_INFO, "Frame %u, len=%u\n", img_cnt, buf->byteused);
					usbh_uvc_img_check(buf);
				}
				usbh_uvc_put_frame(buf, CONFIG_USBH_UVC_STREAM_INDEX);
				img_cnt++;
			}

			usbh_uvc_stop(CONFIG_USBH_UVC_STREAM_INDEX);
			uvc_calculate_tp(img_cnt);
		}
#else /* FATFS / RGB */
		{
			while ((uvc_img_file_no < USBH_UVC_MAX_IMAGES) && (uvc_task_exiting == 0U)) {
				buf = usbh_uvc_get_frame(CONFIG_USBH_UVC_STREAM_INDEX);
				if (buf == NULL) {
					vTaskDelay(pdMS_TO_TICKS(1));
					continue;
				}
				uvc_prepare_save(buf);
				usbh_uvc_put_frame(buf, CONFIG_USBH_UVC_STREAM_INDEX);
			}

			usbh_uvc_stop(CONFIG_USBH_UVC_STREAM_INDEX);

			if (uvc_img_file_no >= USBH_UVC_MAX_IMAGES) {
				RTK_LOGS(TAG, RTK_LOG_INFO, "All %d images saved\n", USBH_UVC_MAX_IMAGES);
				goto exit;
			}
		}
#endif /* APP mode */

		if (uvc_task_exiting != 0U) {
			goto exit;
		}
		/* Normal completion: wait for next start. */
		continue;
	}

exit:
	usbh_uvc_stop(CONFIG_USBH_UVC_STREAM_INDEX);
	uvc_task = NULL;
	vTaskDelete(NULL);
}

/* Hotplug task --------------------------------------------------------------*/

#if CONFIG_USBH_UVC_HOTPLUG
static void example_usbh_uvc_hotplug_thread(void *param)
{
	int ret;

	UNUSED(param);

	for (;;) {
		if (xSemaphoreTake(uvc_detach_sema, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		uvc_task_exiting = 1U;

		vTaskDelay(pdMS_TO_TICKS(200));

		usbh_uvc_stop(CONFIG_USBH_UVC_STREAM_INDEX);
		xSemaphoreGive(uvc_start_sema);

		while (uvc_task != NULL) {
			vTaskDelay(pdMS_TO_TICKS(100));
		}

		RTK_LOGS(TAG, RTK_LOG_INFO, "Hotplug: uvc_test exited\n");

		usbh_stop();
		usbh_uvc_deinit();
		usbh_deinit();
		vTaskDelay(pdMS_TO_TICKS(10));

		uvc_task_exiting = 0U;

		ret = usbh_init(&usbh_cfg, NULL);
		if (ret != HAL_OK) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Init USBH fail\n");
			break;
		}

		ret = usbh_uvc_init(&uvc_cfg, &uvc_cb);
		if (ret != HAL_OK) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Init UVC fail\n");
			usbh_deinit();
			break;
		}

		usbh_start();
	}

	RTK_LOGS(TAG, RTK_LOG_ERROR, "Hotplug thread fail\n");
	vTaskDelete(NULL);
}
#endif /* CONFIG_USBH_UVC_HOTPLUG */

/* Init thread ---------------------------------------------------------------*/

static void example_usbh_uvc_new_thread(void *param)
{
#if CONFIG_USBH_UVC_HOTPLUG
	TaskHandle_t hotplug_task;
#endif
	int ret;

	UNUSED(param);

	uvc_attach_sema    = xSemaphoreCreateBinary();
	uvc_start_sema     = xSemaphoreCreateBinary();
	uvc_setparam_sema  = xSemaphoreCreateBinary();
#if CONFIG_USBH_UVC_HOTPLUG
	uvc_detach_sema    = xSemaphoreCreateBinary();
#endif

#if ((CONFIG_USBH_UVC_APP == USBH_UVC_APP_FATFS) || (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB))
	uvc_buf_mutex = xSemaphoreCreateMutex();
	uvc_save_sema = xSemaphoreCreateBinary();
#endif

	ret = usbh_init(&usbh_cfg, NULL);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Init USBH fail\n");
		goto exit;
	}

	ret = usbh_uvc_init(&uvc_cfg, &uvc_cb);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Init UVC fail\n");
		usbh_deinit();
		goto exit;
	}

	usbh_start();

#if CONFIG_USBH_UVC_HOTPLUG
	if (xTaskCreate(example_usbh_uvc_hotplug_thread, "usbh_uvc_hotplug", UVC_HOTPLUG_THREAD_STACK_SIZE,
					NULL, UVC_HOTPLUG_THREAD_PRIORITY, &hotplug_task) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create hotplug thread fail\n");
		usbh_uvc_deinit();
		usbh_deinit();
		goto exit;
	}
#endif

	RTK_LOGS(TAG, RTK_LOG_INFO, "USBH UVC demo start\n");

	for (;;) {
		if (xSemaphoreTake(uvc_attach_sema, portMAX_DELAY) == pdTRUE) {
			if (xTaskCreate(example_usbh_uvc_test, "usbh_uvc_test", UVC_TEST_THREAD_STACK_SIZE,
							NULL, UVC_TEST_THREAD_PRIORITY, &uvc_task) != pdPASS) {
				RTK_LOGS(TAG, RTK_LOG_ERROR, "Create test thread fail\n");
			}
		}
	}

exit:
	vTaskDelete(NULL);
}

/* Exported functions --------------------------------------------------------*/

void example_usbh_uvc(void)
{
	if (xTaskCreate(example_usbh_uvc_new_thread, "usbh_uvc_new_init", UVC_INIT_THREAD_STACK_SIZE,
					NULL, UVC_INIT_THREAD_PRIORITY, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create USBH UVC thread fail\n");
	}
}
