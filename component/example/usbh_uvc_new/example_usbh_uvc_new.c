/**
  ******************************************************************************
  * @file    example_usbh_uvc_new.c
  * @author  Realsil WLAN5 Team
  * @version V1.0.0
  * @date    2021-11-16
  * @brief   This file provides the demo code for USB UVC host class
  *
  * Modes:
  *   USBH_UVC_APP_EMPTY  — capture and discard
  *   USBH_UVC_APP_FATFS  — capture raw JPEG/YUY2 and save to SD card
  *   USBH_UVC_APP_RGB    — capture, convert to RGB888, save as PPM to SD card
  ******************************************************************************
  * @attention
  *
  * This module is a confidential and proprietary property of RealTek and
  * possession or use of this module requires written permission of RealTek.
  *
  * Modified: Realtek SG SD3, 2026-08-21
  * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------ */
#define CONFIG_EXAMPLE_USBH_UVC 1
#include <platform_opts.h>
#include "example_usbh_uvc_new.h"
#if defined(CONFIG_EXAMPLE_USBH_UVC) && CONFIG_EXAMPLE_USBH_UVC
#include <platform_stdlib.h>
#include "osdep_service.h"
#include "usbh_uvc_intf.h"
#include "usbh.h"

/* Private defines -----------------------------------------------------------*/

/*Just capture and abandon frame*/
#define USBH_UVC_APP_EMPTY	1

/*Capture frame and write it to SD card through fatfs*/
#define USBH_UVC_APP_FATFS	2

/*Capture frame, convert to RGB, and write to SD card as PPM*/
#define USBH_UVC_APP_RGB	3


/*Choose which application example*/
#define CONFIG_USBH_UVC_APP  USBH_UVC_APP_RGB
#define USBH_UVC_MAX_IMAGES  500


#define CONFIG_USBH_UVC_CHECK_IMAGE_DATA  1

extern int otg_select_usb_mode(int value);

/* Private includes -------------------------------------------------------------*/

#if ((CONFIG_USBH_UVC_APP == USBH_UVC_APP_FATFS) || (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB))
#include "ff.h"
#include "fatfs_sdcard_api.h"
#endif

#if (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB)
#include "uvc_to_rgb.h"
#endif

/* cmsis.h included in uvc_to_rgb.c for D-cache maintenance.
 * uvcbuf cache maintenance is in uvc_jpeg_to_rgb(). */

/* Private macros ------------------------------------------------------------*/

#define USBH_UVC_BUF_SIZE       (640*1024U) // 640x480 YUY2 = 614400 bytes; 640x480 RGB888 = 921600 bytes
#define USBH_UVC_FORMAT_TYPE    UVC_FORMAT_MJPEG // UVC_FORMAT_UNCOMPRESSED or UVC_FORMAT_MJPEG
#define USBH_UVC_WIDTH          640
#define USBH_UVC_HEIGHT         480
#define USBH_UVC_FRAME_RATE     15


#define USBH_UVC_JFIF_TAG       0xFF
#define USBH_UVC_JFIF_SOI       0xD8
#define USBH_UVC_JFIF_EOI       0xD9


/* Private function prototypes -----------------------------------------------*/

static int uvc_cb_init(void);
static int uvc_cb_deinit(void);
static int uvc_cb_attach(void);
static int uvc_cb_detach(void);



/* Private variables ---------------------------------------------------------*/

static _sema UvcSema;
static _mutex uvc_buf_mutex = NULL;
static int save_cnt = 0;
static int warmup_cnt = 0;

#if ((CONFIG_USBH_UVC_APP == USBH_UVC_APP_FATFS) || (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB))
static _sema uvc_fatfs_save_img_sema = NULL;
static _sema uvc_fatfs_exit_sema = NULL;
static int uvc_fatfs_is_init = 0;
static int uvc_fatfs_img_file_no = 0;
static volatile int uvc_save_busy = 0;
#endif

/* Buffer for raw captured frame data */
PSRAM_BSS_SECTION u8 uvc_buf[USBH_UVC_BUF_SIZE];
static int uvc_buf_size = 0;

#if (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB)
/* Buffer for converted RGB888 data (max 640x480 RGB888 = 921600 bytes) */
PSRAM_BSS_SECTION u8 uvc_rgb_buf[USBH_UVC_WIDTH * USBH_UVC_HEIGHT * 3];
static int uvc_rgb_buf_size = 0;
#endif


static usbh_config_t usbh_cfg = {
	.host_channels = 5U,
	.speed = USB_SPEED_HIGH,
	.dma_enable = FALSE,
	.main_task_priority = 3U,
	.isr_task_priority = 4U,
	.rx_fifo_size   = 0x330U,
	.nptx_fifo_size = 0x060U,
	.ptx_fifo_size  = 0x070U,
};


static usbh_uvc_cb_t uvc_cb = {
	.init = uvc_cb_init,
	.deinit = uvc_cb_deinit,
	.attach = uvc_cb_attach,
	.detach = uvc_cb_detach,
};

static u8 user_process(usb_host_t *host, u8 id)
{
	if (id == USBH_MSG_CONNECTED) {
		printf("USB physical connected\n");
	} else if (id == USBH_MSG_DISCONNECTED) {
		printf("USB physical disconnected\n");
	}
	return HAL_OK;
}

static usbh_user_cb_t user_cb = {
	.process = user_process,
};

/* Private functions ---------------------------------------------------------*/

static int uvc_cb_init(void)
{
	printf("UVC init\n");
	return HAL_OK;
}


static int uvc_cb_deinit(void)
{
	printf("UVC deinit\n");
	return HAL_OK;
}


static int uvc_cb_attach(void)
{
	printf("UVC attach\n");
	rtw_up_sema(&UvcSema);
	return HAL_OK;
}


static int uvc_cb_detach(void)
{
	printf("UVC detach\n");
	return HAL_OK;
}


static void uvc_img_prepare(uvc_frame *frame)
{
	u32 len = 0;
	u32 start = 0;

	rtw_enter_critical(NULL, NULL);

	if (frame != NULL) {
#if ((CONFIG_USBH_UVC_CHECK_IMAGE_DATA == 1) && (USBH_UVC_FORMAT_TYPE == UVC_FORMAT_MJPEG))
		u8 *ptr = frame->buf;
		len = frame->byteused;
		u32 end = frame->byteused - 2;

		if (len < 4) {
			printf("\nToo short uvc data, len=%d\n", len);
		}

		while (start < len - 1) {
			if ((ptr[start] == USBH_UVC_JFIF_TAG) && (ptr[start + 1] == USBH_UVC_JFIF_SOI)) {
				while (end > 0) {
					if (ptr[end + 1] != 0) {
						if ((ptr[end] == USBH_UVC_JFIF_TAG) && (ptr[end + 1] == USBH_UVC_JFIF_EOI)) {
							break;
						} else {
							printf("\nInvalid uvc data\n");
							rtw_exit_critical(NULL, NULL);
							return;
						}
					}
					end--;
				}
				break;
			}
			start++;
		}

		if (start == len - 1) {
			printf("\nInvalid uvc data, start\n");
			rtw_exit_critical(NULL, NULL);
			return;
		}

		len = end + 2 - start;

		warmup_cnt++;
		if (warmup_cnt < 30) {
			rtw_exit_critical(NULL, NULL);
			return;
		}

		save_cnt++;
		if ((save_cnt % 15) != 0) {
			rtw_exit_critical(NULL, NULL);
			return;
		}

		if (len == frame->byteused) {
			printf("\nCapture uvc data, len=%d\n", len);
		} else {
			printf("\nCapture uvc data, start=%d, end=%d, actul_len=%d\n", start, end + 2, len);
		}

#else
		len = frame->byteused;

		warmup_cnt++;
		if (warmup_cnt < 30) {
			rtw_exit_critical(NULL, NULL);
			return;
		}

		save_cnt++;
		if ((save_cnt % 15) != 0) {
			rtw_exit_critical(NULL, NULL);
			return;
		}
#endif
		if (len > USBH_UVC_BUF_SIZE) {
			printf("\nImage len overflow!\n");
			rtw_exit_critical(NULL, NULL);
			return;
		}

		rtw_mutex_get(&uvc_buf_mutex);
		memcpy(uvc_buf, (void *)(frame->buf + start), len);
		uvc_buf_size = len;
		/*
		 * NOTE: do NOT decode JPEG here. uvc_img_prepare() runs inside a
		 * critical section (interrupts disabled). A JPEG decode takes tens of
		 * ms; running it here drops USB isochronous packets for the next
		 * frame, corrupting its entropy stream and producing a horizontally
		 * shifted image after restart-marker resync. Decode is done in the
		 * FATFS thread (uvc_fatfs_thread) instead.
		 */
		rtw_mutex_put(&uvc_buf_mutex);

		if (uvc_buf_size > 0) {
#if ((CONFIG_USBH_UVC_APP == USBH_UVC_APP_FATFS) || (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB))
			if (uvc_fatfs_is_init && !uvc_save_busy) {
				uvc_save_busy = 1;
				rtw_up_sema(&uvc_fatfs_save_img_sema);
			}
#endif
		}
	}

	rtw_exit_critical(NULL, NULL);
}


#if ((CONFIG_USBH_UVC_APP == USBH_UVC_APP_FATFS) || (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB))

static fatfs_sd_params_t fatfs_sd;
static FIL m_file;

static void uvc_fatfs_thread(void *param)
{
	FRESULT res;
	char filename[64] = {0};
	char path[128] = {0};
	UINT bw = 0;

	UNUSED(param);

	printf("Init FATFS SD...\n");

	res = fatfs_sd_init();
	if (res < 0) {
		printf("fatfs_sd_init fail: %d\n", res);
		goto fail;
	}

	fatfs_sd_get_param(&fatfs_sd);
	printf("SD mounted at: %s\n", fatfs_sd.drv);

	rtw_init_sema(&uvc_fatfs_save_img_sema, 0);
	uvc_fatfs_is_init = 1;

	while (uvc_fatfs_is_init) {
		rtw_down_sema(&uvc_fatfs_save_img_sema);

		if (!uvc_fatfs_is_init) {
			break;
		}

#if (CONFIG_USBH_UVC_APP == USBH_UVC_APP_RGB)
		/*
		 * Convert captured frame to RGB888 before saving.
		 *
		 * MJPEG → RGB: use stb_image decoder via uvc_jpeg_to_rgb().
		 * YUY2  → RGB: use inline YUV-to-RGB via uvc_yuy2_to_rgb().
		 */
		{
			int w = USBH_UVC_WIDTH;
			int h = USBH_UVC_HEIGHT;
			uint8_t *rgb;

			/* Read uvc_buf under mutex (written by uvc_img_prepare) */
			rtw_mutex_get(&uvc_buf_mutex);

			if (USBH_UVC_FORMAT_TYPE == UVC_FORMAT_MJPEG) {
				rgb = uvc_jpeg_to_rgb(uvc_buf, uvc_buf_size, &w, &h,
									  uvc_rgb_buf, UVC_RGB_FORMAT_RGB888);
				if (!rgb) {
					printf("\nJPEG decode failed (len=%d)\n", uvc_buf_size);
					rtw_mutex_put(&uvc_buf_mutex);
					uvc_save_busy = 0;
					continue;
				}
				uvc_rgb_buf_size = w * h * 3;
				printf("\nJPEG decoded: %dx%d (%d bytes)\n", w, h, uvc_rgb_buf_size);
			} else {
				rgb = uvc_yuy2_to_rgb(uvc_buf, w, h,
									  uvc_rgb_buf, UVC_RGB_FORMAT_RGB888);
				if (!rgb) {
					printf("\nYUY2 conversion failed (len=%d)\n", uvc_buf_size);
					rtw_mutex_put(&uvc_buf_mutex);
					uvc_save_busy = 0;
					continue;
				}
				uvc_rgb_buf_size = w * h * 3;
				printf("\nYUY2 converted: %dx%d (%d bytes)\n", w, h, uvc_rgb_buf_size);
			}

			rtw_mutex_put(&uvc_buf_mutex);

			snprintf(filename, sizeof(filename), "img%d.ppm", uvc_fatfs_img_file_no);
			snprintf(path, sizeof(path), "%s%s", fatfs_sd.drv, filename);

			res = f_open(&m_file, path, FA_CREATE_ALWAYS | FA_WRITE);
			if (res) {
				printf("\nFail to open file (%s): %d\n", path, res);
				uvc_save_busy = 0;
				continue;
			}

			printf("\nCreate PPM image file: %s (%d bytes)\n", path, uvc_rgb_buf_size);

			/*
			 * Write PPM P6 header: "P6\nwidth height\n255\n" + raw RGB.
			 * This format is viewable in any image viewer (GIMP, ImageMagick, etc.).
			 */
			{
				char header[64];
				int hdr_len = snprintf(header, sizeof(header), "P6\n%d %d\n255\n", w, h);
				f_write(&m_file, header, hdr_len, &bw);
			}
			res = f_write(&m_file, uvc_rgb_buf, uvc_rgb_buf_size, &bw);

			if (res) {
				printf("\nFail to write PPM file: res=%d\n", res);
			} else {
				printf("\nWrote %d bytes RGB data\n", bw);
			}

			res = f_close(&m_file);
			if (res) {
				printf("\nFail to close file (%s): %d\n", path, res);
			}
		}
#else
		/* Original FATFS path: save raw data (JPEG or YUY2) */
		if (USBH_UVC_FORMAT_TYPE == UVC_FORMAT_UNCOMPRESSED) {
			snprintf(filename, sizeof(filename), "img%d.yuy2", uvc_fatfs_img_file_no);
		} else {
			snprintf(filename, sizeof(filename), "img%d.jpg", uvc_fatfs_img_file_no);
		}
		snprintf(path, sizeof(path), "%s%s", fatfs_sd.drv, filename);

		res = f_open(&m_file, path, FA_CREATE_ALWAYS | FA_WRITE);
		if (res) {
			printf("\nFail to open file (%s): %d\n", path, res);
			uvc_save_busy = 0;
			continue;
		}

		printf("\nCreate image file: %s\n", path);

		rtw_mutex_get(&uvc_buf_mutex);
		res = f_write(&m_file, uvc_buf, uvc_buf_size, &bw);
		rtw_mutex_put(&uvc_buf_mutex);

		if (res || bw != uvc_buf_size) {
			printf("\nFail to write file: res=%d, bw=%d, size=%d\n", res, bw, uvc_buf_size);
		} else {
			printf("\nWrite %d bytes\n", bw);
		}

		res = f_close(&m_file);
		if (res) {
			printf("\nFail to close file (%s): %d\n", path, res);
		}
#endif

		uvc_fatfs_img_file_no++;
		uvc_save_busy = 0;

		if (uvc_fatfs_img_file_no >= USBH_UVC_MAX_IMAGES) {
			printf("\nReached %d images, stopping capture\n", USBH_UVC_MAX_IMAGES);
			break;
		}
	}

	rtw_free_sema(&uvc_fatfs_save_img_sema);
	fatfs_sd_close();
	rtw_up_sema(&uvc_fatfs_exit_sema);

fail:
	rtw_thread_exit();
}

static int uvc_fatfs_start(void)
{
	int ret;
	struct task_struct task;
	ret = rtw_create_task(&task, "uvc_fatfs_thread", 8192, tskIDLE_PRIORITY + 2, uvc_fatfs_thread, NULL);
	if (ret != pdPASS) {
		printf("\nFail to create USB host UVC fatfs thread\n");
		ret = 1;
	} else {
		ret = 0;
	}

	return ret;
}

static void example_usbh_uvc_task(void *param)
{
	int ret = 0;
	uvc_frame *buf;
	struct uvc_config uvc_ctx;
	int uvc_service_started = 0;
	UNUSED(param);

	otg_select_usb_mode(1);

	ret = usbh_init(&usbh_cfg, &user_cb);
	if (ret != HAL_OK) {
		printf("\nFail to init USB\n");
		goto exit;
	}

	if (usbh_get_status()) {
		printf("Device is connected!\n");
	} else {
		printf("No device connected\n");
	}

	rtw_init_sema(&UvcSema, 0);
	rtw_mutex_init(&uvc_buf_mutex);
	rtw_init_sema(&uvc_fatfs_exit_sema, 0);

	while (1) {

		ret = usbh_uvc_init(&uvc_cb);
		if (ret) {
			printf("\nFail to init UVC driver\n");
			break;
		}

		uvc_ctx.fmt_type = USBH_UVC_FORMAT_TYPE;
		uvc_ctx.width = USBH_UVC_WIDTH;
		uvc_ctx.height = USBH_UVC_HEIGHT;
		uvc_ctx.frame_rate = USBH_UVC_FRAME_RATE;

		rtw_down_sema(&UvcSema);

		printf("\nSet UVC parameters\n");
		ret = usbh_uvc_set_param(&uvc_ctx);
		if (ret) {
			printf("\nFail to set UVC parameters: %d\n", ret);
			goto cleanup;
		} else {
			printf("\nUVC para: %d*%d@%dfps\n", uvc_ctx.width, uvc_ctx.height, uvc_ctx.frame_rate);
		}

		if (!uvc_service_started) {
			printf("\nStart FATFS service\n");
			ret = uvc_fatfs_start();
			rtw_mdelay_os(1000);
			if (ret != 0) {
				printf("\nFail to start fatfs: %d\n", ret);
				continue;
			}

			uvc_service_started = 1;
			rtw_mdelay_os(1000);
		}

		if (!usbh_get_status()) {
			goto cleanup;
		}

		printf("\nUVC stream on\n");
		ret = usbh_uvc_stream_on();
		if (ret) {
			printf("\nFail to turn on UVC stream: %d\n", ret);
			goto cleanup;
		}

		rtw_mdelay_os(1500);

		while (1) {
			if (!usbh_get_status()) {
				break;
			}

			buf = usbh_uvc_get_frame();
			if (buf == NULL) {
				printf("\nWaiting for UVC frame...\n");
				rtw_mdelay_os(100);
				continue;
			}

			uvc_img_prepare(buf);
			usbh_uvc_put_frame(buf);
			buf = NULL;
			rtw_mdelay_os(5);
		}

cleanup:
		printf("Disconnected, cleaning up...\n");
		usbh_uvc_stream_off();
		usbh_uvc_deinit();
		printf("Ready for next attach...\n");
	}

	rtw_free_sema(&UvcSema);
	rtw_free_sema(&uvc_fatfs_exit_sema);
exit:
	rtw_thread_exit();
}


#endif /* FATFS || RGB */

#if (CONFIG_USBH_UVC_APP == USBH_UVC_APP_EMPTY)
static void example_usbh_uvc_task(void *param)
{
	int ret = 0;
	uvc_frame *buf;
	struct uvc_config uvc_ctx;
	UNUSED(param);

	otg_select_usb_mode(1);

	ret = usbh_init(&usbh_cfg, &user_cb);
	if (ret != HAL_OK) {
		printf("\nFail to init USB\n");
		goto exit;
	}

	rtw_init_sema(&UvcSema, 0);
	rtw_mutex_init(&uvc_buf_mutex);

	if (usbh_get_status()) {
		printf("Device is connected!\n");
	} else {
		printf("No device connected\n");
	}

	while (1) {
		ret = usbh_uvc_init(&uvc_cb);
		if (ret) {
			printf("\nFail to init UVC driver\n");
			goto exit1;
		}

		rtw_down_sema(&UvcSema);

		printf("\nSet UVC parameters\n");
		uvc_ctx.fmt_type = USBH_UVC_FORMAT_TYPE;
		uvc_ctx.width = USBH_UVC_WIDTH;
		uvc_ctx.height = USBH_UVC_HEIGHT;
		uvc_ctx.frame_rate = USBH_UVC_FRAME_RATE;
		ret = usbh_uvc_set_param(&uvc_ctx);
		if (ret) {
			printf("\nFail to set UVC parameters: %d\n", ret);
			goto exit1;
		} else {
			printf("\nUVC para: %d*%d@%dfps\n", uvc_ctx.width, uvc_ctx.height, uvc_ctx.frame_rate);
		}

		if (!usbh_get_status()) {
			break;
		}

		printf("\nUVC stream on\n");
		ret = usbh_uvc_stream_on();
		if (ret) {
			printf("\nFail to turn on UVC stream: %d\n", ret);
			goto exit2;
		}

		printf("\nGet frame buffer\n");
		buf = usbh_uvc_get_frame();
		if (buf == NULL) {
			printf("\nFail to dequeue UVC buffer\n");
			ret = 1;
		} else {
			ret = 0;
		}

		if (ret == 0) {
			uvc_img_prepare(buf);
		}

		printf("\nPut frame buffer\n");
		usbh_uvc_put_frame(buf);

		if (!usbh_get_status()) {
			break;
		}

		printf("\nStop capturing images\n");
		usbh_uvc_stream_off();

		rtw_mdelay_os(1000);
	}

exit2:
	usbh_uvc_stream_off();
	usbh_uvc_deinit();
exit1:
	rtw_mutex_free(&uvc_buf_mutex);
	rtw_free_sema(&UvcSema);
	usbh_deinit();
exit:
	rtw_thread_exit();
}


#endif


/* Exported functions --------------------------------------------------------*/


void example_usbh_uvc(void)
{
	int status;
	struct task_struct task;

	printf("\n[UVC] USB host uvc demo started...\n");

	status = rtw_create_task(&task, "example_usbh_uvc_thread", 1024 * 6, tskIDLE_PRIORITY + 1, example_usbh_uvc_task, NULL);
	if (status != pdPASS) {
		printf("\n[UVC] Fail to create USB host uvc thread: %d\n", status);
	}

}

#endif
