/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Sensor modes of example_usbd_uvc_new, built on the usb_stack/v1.0.0 UVC class and the new
 * usbd_uvc_module (module_usbd_uvc.c in this example). The legacy module_uvcd is not used.
 *
 * USBD_UVC_NEW_MODE_SINGLE: one camera, the host picks the format like the legacy media_uvcd
 *   example: H.264 / HEVC / MJPEG 1920x1080 30/15/10 fps, NV12 / NV16 1920x1080 5 fps.
 *   V1 is restarted with the committed format every time the host starts streaming.
 *   NV12/NV16 1080p is limited to 5 fps by USB: 3 x 1024 B per microframe = 24.5 MB/s.
 *
 * USBD_UVC_NEW_MODE_DUAL: two cameras streaming at the same time, one USB device:
 *   UVC 0: V1 H.264 1920x1080 30 fps on EP1 IN (1 x 1024 B per microframe, 8 MB/s)
 *   UVC 1: V2 NV12  640x360   15 fps on EP3 IN (3 x 1024 B per microframe, 24.5 MB/s, raw 5.2 MB/s)
 *   Both video channels always run; a camera the host does not read just drops its frames.
 *
 * TX FIFO: ptx_fifo_depth[i] backs dedicated TX FIFO #(i+1); usbd_hal_get_tx_fifo_num() maps
 * EP1 -> FIFO#1 ([0]), EP3 -> FIFO#2 ([1], up to 768 DWORD), EP5 -> FIFO#3 ([2]).
 * Total 2232 DWORD = RX + EP0 256 + PTX + 8 reserved.
 */

/* Includes ------------------------------------------------------------------ */

#include "usbd.h"
#include "usbd_uvc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#undef MAX		/* the media headers below define it again; hal_video_common.h has no #ifndef guard */
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "module_usbd_uvc.h"
#include "mmf2_pro2_video_config.h"
#include "sensor.h"
#include "log_service.h"
#include "example_usbd_uvc_new.h"

#if (USBD_UVC_NEW_MODE != USBD_UVC_NEW_MODE_SAMPLE)

/* Private defines -----------------------------------------------------------*/
static const char *const TAG = "UVC";

#define UVC_CTRL_THREAD_PRIORITY         (tskIDLE_PRIORITY + 4)
#define UVC_CTRL_THREAD_STACK_SIZE       2048U
#define UVC_LOOP_THREAD_PRIORITY         (tskIDLE_PRIORITY + 4)
#define UVC_LOOP_THREAD_STACK_SIZE       2048U
#define UVC_STAT_THREAD_PRIORITY         (tskIDLE_PRIORITY + 1)
#define UVC_STAT_THREAD_STACK_SIZE       512U
#define UVC_STAT_INTERVAL_MS             5000U
#define UVC_POLL_MS                      100U
#define UVC_STOP_WAIT_MS                 3000U   /* loop thread may be switching the V1 format */
#define UVC_CYCLE_DEFAULT                10U
#define UVC_CYCLE_RUN_MS                 3000U

/* USBUVC console requests, run by uvc_ctrl_thread */
#define UVC_EVT_START                    1U
#define UVC_EVT_STOP                     2U
#define UVC_EVT_CYCLE                    3U

#define UVC_V1_CHANNEL                   0
#define UVC_V2_CHANNEL                   1
#define UVC_V1_BPS                       (2 * 1024 * 1024)
#define UVC_V1_RCMODE                    2      /* 1: CBR, 2: VBR */

#define UVC_FHD_W                        1920U
#define UVC_FHD_H                        1080U
#define UVC_WVGA_W                       640U
#define UVC_WVGA_H                       360U
#define UVC_RAW_FHD_FPS                  5U     /* NV12/NV16 1080p over 3 x 1024 B per microframe */

/* Private variables ---------------------------------------------------------*/

static mm_context_t *uvc_video_ctx[2];
static mm_context_t *uvc_sink_ctx[2];
static mm_siso_t *uvc_siso[2];
static u8 uvc_cam_num;

typedef struct {
	u32 evt;
	u32 rounds;
} uvc_ctrl_msg_t;

/* uvc_lock serializes start / stop (uvc_ctrl_thread) with the DFU hand-over (DFU control thread) */
static SemaphoreHandle_t uvc_lock;
static QueueHandle_t uvc_ctrl_queue;
static u8 uvc_running;
static u8 uvc_dfu_owns_usb;
static volatile u8 uvc_stop_req;
static volatile u8 uvc_loop_alive;
static volatile u8 uvc_stat_alive;

static video_params_t uvc_v1_params = {
	.stream_id = UVC_V1_CHANNEL,
	.type = VIDEO_H264,
	.resolution = VIDEO_FHD,
	.width = UVC_FHD_W,
	.height = UVC_FHD_H,
	.bps = UVC_V1_BPS,
	.fps = 30,
	.gop = 30,
	.rc_mode = UVC_V1_RCMODE,
	.use_static_addr = 1,
};

#if (USBD_UVC_NEW_MODE == USBD_UVC_NEW_MODE_SINGLE)

/* Same formats as the legacy media_uvcd example, H.264 first so it is the default */
static usbd_uvc_frame_info_t uvc_frames_enc[] = {
	{ UVC_FHD_W, UVC_FHD_H, { USBD_UVC_FPS(30), USBD_UVC_FPS(15), USBD_UVC_FPS(10), 0 } },
	{ 0, 0, { 0 } },
};
static usbd_uvc_frame_info_t uvc_frames_raw[] = {
	{ UVC_FHD_W, UVC_FHD_H, { USBD_UVC_FPS(UVC_RAW_FHD_FPS), 0 } },
	{ 0, 0, { 0 } },
};
static usbd_uvc_format_info_t uvc_single_formats[] = {
	{ USBD_UVC_FORMAT_TYPE_H264, uvc_frames_enc },
	{ USBD_UVC_FORMAT_TYPE_H265, uvc_frames_enc },
	{ USBD_UVC_FORMAT_TYPE_MJPEG, uvc_frames_enc },
	{ USBD_UVC_FORMAT_TYPE_NV12, uvc_frames_raw },
	{ USBD_UVC_FORMAT_TYPE_YUY2, uvc_frames_raw },     /* sent as NV16 */
};
static const usbd_uvc_inst_cfg_t uvc_inst_cfg[] = {
	{ .iso_in_addr = 0x83U, .iso_in_xfer_size = 3072U, .formats = uvc_single_formats, .nformats = 5U },
};

static const usbd_config_t uvc_usb_cfg = {
	.speed = USB_SPEED_HIGH,
	.isr_priority = 4U,
	.ext_intr_enable = USBD_SOF_INTR,
	/* RX 1168 + EP0 256 + {16, 768, 16} + 8 = 2232 */
	.rx_fifo_depth = 1168U,
	.ptx_fifo_depth = {16U, 768U, 16U, },
};

#else /* USBD_UVC_NEW_MODE_DUAL */

static usbd_uvc_frame_info_t uvc_frames_h264[] = {
	{ UVC_FHD_W, UVC_FHD_H, { USBD_UVC_FPS(30), 0 } },
	{ 0, 0, { 0 } },
};
static usbd_uvc_frame_info_t uvc_frames_nv12[] = {
	{ UVC_WVGA_W, UVC_WVGA_H, { USBD_UVC_FPS(15), 0 } },
	{ 0, 0, { 0 } },
};
static usbd_uvc_format_info_t uvc_dual_h264[] = {
	{ USBD_UVC_FORMAT_TYPE_H264, uvc_frames_h264 },
};
static usbd_uvc_format_info_t uvc_dual_nv12[] = {
	{ USBD_UVC_FORMAT_TYPE_NV12, uvc_frames_nv12 },
};
static const usbd_uvc_inst_cfg_t uvc_inst_cfg[] = {
	/* H.264 2 Mbps averages 8 KB a frame, an I frame of ~70 KB takes ~9 ms at 1 x 1024 B */
	{ .iso_in_addr = 0x81U, .iso_in_xfer_size = 1024U, .formats = uvc_dual_h264, .nformats = 1U },
	{ .iso_in_addr = 0x83U, .iso_in_xfer_size = 3072U, .formats = uvc_dual_nv12, .nformats = 1U },
};

static video_params_t uvc_v2_params = {
	.stream_id = UVC_V2_CHANNEL,
	.type = VIDEO_NV12,
	.resolution = VIDEO_WVGA,
	.width = UVC_WVGA_W,
	.height = UVC_WVGA_H,
	.fps = 15,
	.use_static_addr = 1,
};

static const usbd_config_t uvc_usb_cfg = {
	.speed = USB_SPEED_HIGH,
	.isr_priority = 4U,
	.ext_intr_enable = USBD_SOF_INTR,
	/* RX 912 + EP0 256 + {EP1 256, EP3 768, EP5 16} + 8 = 2216 <= 2232. UVC has no OUT EP. */
	.rx_fifo_depth = 912U,
	.ptx_fifo_depth = {256U, 768U, 16U, },
};

#endif

/* Private functions ---------------------------------------------------------*/

static const char *uvc_fmt_name(u32 fmt)
{
	switch (fmt) {
	case USBD_UVC_MOD_FMT_NV16:
		return "NV16";
	case USBD_UVC_MOD_FMT_NV12:
		return "NV12";
	case USBD_UVC_MOD_FMT_MJPEG:
		return "MJPEG";
	case USBD_UVC_MOD_FMT_H264:
		return "H264";
	case USBD_UVC_MOD_FMT_H265:
		return "HEVC";
	default:
		return "?";
	}
}

/* Start the raw / JPEG output of a channel that was just applied */
static void uvc_video_start_output(mm_context_t *ctx, u32 type)
{
	if ((type == VIDEO_NV12) || (type == VIDEO_NV16)) {
		mm_module_ctrl(ctx, CMD_VIDEO_YUV, 2);
	} else if (type == VIDEO_JPEG) {
		mm_module_ctrl(ctx, CMD_VIDEO_SNAPSHOT, 2);
	}
}

static mm_context_t *uvc_video_open(video_params_t *params, int voe_heap_size, int queue_len)
{
	mm_context_t *ctx = mm_module_open(&video_module);

	if (ctx == NULL) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Video open fail\n");
		return NULL;
	}
	if (voe_heap_size > 0) {
		mm_module_ctrl(ctx, CMD_VIDEO_SET_VOE_HEAP, voe_heap_size);
	}
	mm_module_ctrl(ctx, CMD_VIDEO_SET_PARAMS, (int)params);
	mm_module_ctrl(ctx, MM_CMD_SET_QUEUE_LEN, queue_len);
	mm_module_ctrl(ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	return ctx;
}

static mm_context_t *uvc_sink_open(u8 inst)
{
	mm_context_t *ctx = mm_module_open(&usbd_uvc_module);

	if (ctx == NULL) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "UVC module open fail\n");
		return NULL;
	}
	if (mm_module_ctrl(ctx, CMD_USBD_UVC_SET_INSTANCE, inst) != 0) {
		mm_module_close(ctx);
		return NULL;
	}
	return ctx;
}

static mm_siso_t *uvc_link(mm_context_t *src, mm_context_t *sink)
{
	mm_siso_t *siso = siso_create();

	if (siso == NULL) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "siso create fail\n");
		return NULL;
	}
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	siso_ctrl(siso, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
	siso_ctrl(siso, MMIC_CMD_ADD_INPUT, (uint32_t)src, 0);
	siso_ctrl(siso, MMIC_CMD_ADD_OUTPUT, (uint32_t)sink, 0);
	siso_start(siso);
	return siso;
}

/* Frames per second sent to each camera, printed while the host streams. Exits on uvc_stop_req. */
static void uvc_stat_thread(void *param)
{
	usbd_uvc_module_stat_t last[2] = {{0}};
	usbd_uvc_module_stat_t cur;
	u32 fps[2];
	u32 slept_ms = 0U;
	u8 i;
	u8 active;

	UNUSED(param);

	while (uvc_stop_req == 0U) {
		/* Short sleeps so a stop does not wait a whole interval */
		vTaskDelay(pdMS_TO_TICKS(UVC_POLL_MS));
		slept_ms += UVC_POLL_MS;
		if (slept_ms < UVC_STAT_INTERVAL_MS) {
			continue;
		}
		slept_ms = 0U;
		active = 0U;
		for (i = 0U; i < uvc_cam_num; i++) {
			mm_module_ctrl(uvc_sink_ctx[i], CMD_USBD_UVC_GET_STAT, (int)&cur);
			fps[i] = ((cur.sent - last[i].sent) * 1000U) / UVC_STAT_INTERVAL_MS;
			last[i] = cur;
			if (usbd_uvc_get_status_inst(i) != 0) {
				active = 1U;
			}
		}
		if (active == 0U) {
			continue;
		}
		if (uvc_cam_num == 1U) {
			RTK_LOGS(TAG, RTK_LOG_INFO, "UVC0 %s fps=%u drop=%u\n", usbd_uvc_get_status_inst(0) ? "on" : "off",
					 fps[0], last[0].dropped);
		} else {
			RTK_LOGS(TAG, RTK_LOG_INFO, "UVC0(H264) %s fps=%u drop=%u | UVC1(NV12) %s fps=%u drop=%u\n",
					 usbd_uvc_get_status_inst(0) ? "on" : "off", fps[0], last[0].dropped,
					 usbd_uvc_get_status_inst(1) ? "on" : "off", fps[1], last[1].dropped);
		}
	}
	uvc_stat_alive = 0U;
	vTaskDelete(NULL);
}

/* V1 back to its boot format, a restart must not inherit the last format the host picked */
static void uvc_v1_params_reset(void)
{
	uvc_v1_params.type = VIDEO_H264;
	uvc_v1_params.width = UVC_FHD_W;
	uvc_v1_params.height = UVC_FHD_H;
	uvc_v1_params.fps = 30;
	uvc_v1_params.gop = 30;
}

#if (USBD_UVC_NEW_MODE == USBD_UVC_NEW_MODE_SINGLE)

/* Restart V1 with the format the host committed (legacy media_uvcd behaviour) */
static void uvc_single_apply(const usbd_uvc_module_format_t *fmt)
{
	u32 type;

	switch (fmt->format) {
	case USBD_UVC_MOD_FMT_NV16:
		type = VIDEO_NV16;
		break;
	case USBD_UVC_MOD_FMT_NV12:
		type = VIDEO_NV12;
		break;
	case USBD_UVC_MOD_FMT_MJPEG:
		type = VIDEO_JPEG;
		break;
	case USBD_UVC_MOD_FMT_H265:
		type = VIDEO_HEVC;
		break;
	default:
		type = VIDEO_H264;
		break;
	}

	if ((type == uvc_v1_params.type) && (fmt->width == uvc_v1_params.width) &&
		(fmt->height == uvc_v1_params.height) && (fmt->fps == uvc_v1_params.fps)) {
		return;
	}
	RTK_LOGS(TAG, RTK_LOG_INFO, "Switch V1 to %s %ux%u %u fps\n", uvc_fmt_name(fmt->format), fmt->width, fmt->height, fmt->fps);

	/* Pause the link first so no frame of the old format is in flight while the channel stops */
	siso_pause(uvc_siso[0]);
	mm_module_ctrl(uvc_video_ctx[0], CMD_VIDEO_STREAM_STOP, UVC_V1_CHANNEL);

	uvc_v1_params.type = type;
	uvc_v1_params.width = fmt->width;
	uvc_v1_params.height = fmt->height;
	uvc_v1_params.fps = fmt->fps;
	uvc_v1_params.gop = fmt->fps;
	mm_module_ctrl(uvc_video_ctx[0], CMD_VIDEO_SET_PARAMS, (int)&uvc_v1_params);
	mm_module_ctrl(uvc_video_ctx[0], CMD_VIDEO_APPLY, UVC_V1_CHANNEL);
	uvc_video_start_output(uvc_video_ctx[0], type);

	siso_resume(uvc_siso[0]);
}

/* Open the video channel and the UVC sink and link them; uvc_media_teardown() undoes a partial open */
static int uvc_media_open(void)
{
	/* V1 may switch to NV16 (2 B/pixel): size the ISP buffers for it, plus a JPEG buffer for MJPEG */
	int voe_heap_size = video_voe_presetting(1, UVC_FHD_W, (UVC_FHD_H * 4U) / 3U, UVC_V1_BPS, 1,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);

	RTK_LOGS(TAG, RTK_LOG_INFO, "VOE heap %d\n", voe_heap_size);
	uvc_cam_num = 1U;
	uvc_video_ctx[0] = uvc_video_open(&uvc_v1_params, voe_heap_size, 1);
	uvc_sink_ctx[0] = uvc_sink_open(0U);
	if ((uvc_video_ctx[0] == NULL) || (uvc_sink_ctx[0] == NULL)) {
		return -1;
	}
	uvc_siso[0] = uvc_link(uvc_video_ctx[0], uvc_sink_ctx[0]);
	if (uvc_siso[0] == NULL) {
		return -1;
	}
	mm_module_ctrl(uvc_video_ctx[0], CMD_VIDEO_APPLY, UVC_V1_CHANNEL);
	return 0;
}

/* Host opened the camera: make V1 produce what it committed */
static void uvc_media_on_stream(const usbd_uvc_module_format_t *fmt)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "Host streams %s %ux%u %u fps\n", uvc_fmt_name(fmt->format), fmt->width, fmt->height, fmt->fps);
	uvc_single_apply(fmt);
}

#else /* USBD_UVC_NEW_MODE_DUAL */

static int uvc_media_open(void)
{
	int voe_heap_size = video_voe_presetting(1, UVC_FHD_W, UVC_FHD_H, UVC_V1_BPS, 0,
						1, UVC_WVGA_W, UVC_WVGA_H, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);

	RTK_LOGS(TAG, RTK_LOG_INFO, "VOE heap %d\n", voe_heap_size);
	uvc_cam_num = 2U;

	uvc_video_ctx[0] = uvc_video_open(&uvc_v1_params, voe_heap_size, 3);
	uvc_video_ctx[1] = uvc_video_open(&uvc_v2_params, 0, 2);
	uvc_sink_ctx[0] = uvc_sink_open(0U);
	uvc_sink_ctx[1] = uvc_sink_open(1U);
	if ((uvc_video_ctx[0] == NULL) || (uvc_video_ctx[1] == NULL) || (uvc_sink_ctx[0] == NULL) || (uvc_sink_ctx[1] == NULL)) {
		return -1;
	}
	uvc_siso[0] = uvc_link(uvc_video_ctx[0], uvc_sink_ctx[0]);
	uvc_siso[1] = uvc_link(uvc_video_ctx[1], uvc_sink_ctx[1]);
	if ((uvc_siso[0] == NULL) || (uvc_siso[1] == NULL)) {
		return -1;
	}
	mm_module_ctrl(uvc_video_ctx[0], CMD_VIDEO_APPLY, UVC_V1_CHANNEL);
	mm_module_ctrl(uvc_video_ctx[1], CMD_VIDEO_APPLY, UVC_V2_CHANNEL);
	uvc_video_start_output(uvc_video_ctx[1], VIDEO_NV12);
	return 0;
}

/* Formats are fixed, only log when the host opens camera 0 */
static void uvc_media_on_stream(const usbd_uvc_module_format_t *fmt)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "Host streams UVC0 %s %ux%u %u fps\n", uvc_fmt_name(fmt->format), fmt->width, fmt->height, fmt->fps);
}

#endif

/* Waits for the host to start streaming camera 0. Exits on uvc_stop_req (woken by CMD_USBD_UVC_WAKE). */
static void uvc_loop_thread(void *param)
{
	usbd_uvc_module_format_t fmt;

	UNUSED(param);
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	/* uvc_single_apply() reconfigures the video channel from this task */
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	while (uvc_stop_req == 0U) {
		if (mm_module_ctrl(uvc_sink_ctx[0], CMD_USBD_UVC_WAIT_STREAM, (int)&fmt) != 0) {
			/* UVC class down (USB handed to DFU) */
			vTaskDelay(pdMS_TO_TICKS(UVC_POLL_MS));
			continue;
		}
		if (uvc_stop_req != 0U) {
			break;
		}
		uvc_media_on_stream(&fmt);
	}
	uvc_loop_alive = 0U;
	vTaskDelete(NULL);
}

/*
 * Tear down everything uvc_media_start() set up, in reverse order. Handles a partial start
 * (NULL handles are skipped) and a USB port held by DFU (the UVC class is already down).
 * The worker threads must have exited (uvc_threads_stop()).
 */
static void uvc_media_teardown(void)
{
	u8 i;

	/* Links idle first: siso_pause() returns once the sink finished the frame it was sending */
	for (i = 0U; i < 2U; i++) {
		if (uvc_siso[i] != NULL) {
			siso_pause(uvc_siso[i]);
		}
	}
	for (i = 0U; i < 2U; i++) {
		if (uvc_video_ctx[i] != NULL) {
			mm_module_ctrl(uvc_video_ctx[i], CMD_VIDEO_STREAM_STOP, i);
		}
	}
	/* USB class before the core: usbd_uvc_deinit() hands the EP0 buffer back to the core */
	if (usbd_uvc_get_inst_num() != 0U) {
		usbd_uvc_deinit();
		usbd_deinit();
	}
	for (i = 0U; i < 2U; i++) {
		if (uvc_siso[i] != NULL) {
			siso_delete(uvc_siso[i]);
			uvc_siso[i] = NULL;
		}
	}
	for (i = 0U; i < 2U; i++) {
		if (uvc_video_ctx[i] != NULL) {
			mm_module_close(uvc_video_ctx[i]);
			uvc_video_ctx[i] = NULL;
		}
		if (uvc_sink_ctx[i] != NULL) {
			mm_module_close(uvc_sink_ctx[i]);
			uvc_sink_ctx[i] = NULL;
		}
	}
	/* uvc_media_open() always runs video_voe_presetting() */
	video_voe_release();
	uvc_cam_num = 0U;
}

/* Ask the loop and stat threads to exit and wait for them. Returns -1 if one is still running. */
static int uvc_threads_stop(void)
{
	u32 wait_ms;

	uvc_stop_req = 1U;
	for (wait_ms = 0U; ((uvc_loop_alive != 0U) || (uvc_stat_alive != 0U)) && (wait_ms < UVC_STOP_WAIT_MS);
		 wait_ms += 10U) {
		/* Repeat the wake-up: the loop thread may not have reached WAIT_STREAM the first time */
		if (((wait_ms % UVC_POLL_MS) == 0U) && (uvc_sink_ctx[0] != NULL)) {
			mm_module_ctrl(uvc_sink_ctx[0], CMD_USBD_UVC_WAKE, 0);
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	if ((uvc_loop_alive != 0U) || (uvc_stat_alive != 0U)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Worker thread still running (loop %u stat %u)\n", uvc_loop_alive, uvc_stat_alive);
		return -1;
	}
	return 0;
}

static void uvc_print_res(const char *what)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "%s: heap %u (min %u) tasks %u\n", what, (u32)xPortGetFreeHeapSize(),
			 (u32)xPortGetMinimumEverFreeHeapSize(), (u32)uxTaskGetNumberOfTasks());
}

/* USB + UVC class + video + links + worker threads. uvc_lock held. */
static int uvc_media_start(void)
{
	int ret;

	if (uvc_running != 0U) {
		RTK_LOGS(TAG, RTK_LOG_INFO, "Already started\n");
		return 0;
	}
	if (uvc_dfu_owns_usb != 0U) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "USB port is used by DFU, USBDFU=0 first\n");
		return -1;
	}

	ret = usbd_init(&uvc_usb_cfg);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_init fail %d\n", ret);
		return -1;
	}
	ret = usbd_uvc_init_multi(uvc_inst_cfg, (u8)(sizeof(uvc_inst_cfg) / sizeof(uvc_inst_cfg[0])));
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "usbd_uvc_init_multi fail %d\n", ret);
		usbd_deinit();
		return -1;
	}

	uvc_stop_req = 0U;
	uvc_v1_params_reset();
	if (uvc_media_open() != 0) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Media open fail\n");
		goto fail;
	}

	uvc_loop_alive = 1U;
	if (xTaskCreate(uvc_loop_thread, "usbd_uvc_new_loop", UVC_LOOP_THREAD_STACK_SIZE, NULL,
					UVC_LOOP_THREAD_PRIORITY, NULL) != pdPASS) {
		uvc_loop_alive = 0U;
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create loop thread fail\n");
		goto fail;
	}
	uvc_stat_alive = 1U;
	if (xTaskCreate(uvc_stat_thread, "usbd_uvc_new_stat", UVC_STAT_THREAD_STACK_SIZE, NULL,
					UVC_STAT_THREAD_PRIORITY, NULL) != pdPASS) {
		uvc_stat_alive = 0U;
		RTK_LOGS(TAG, RTK_LOG_WARN, "Create stat thread fail\n");
	}

	uvc_running = 1U;
#if (USBD_UVC_NEW_MODE == USBD_UVC_NEW_MODE_SINGLE)
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD UVC demo start: single camera, H264/HEVC/MJPEG/NV12/NV16\n");
#else
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD UVC demo start: dual camera, UVC0 H264 1080p30 + UVC1 NV12 640x360 15fps\n");
#endif
	return 0;

fail:
	if (uvc_threads_stop() == 0) {
		uvc_media_teardown();
	}
	return -1;
}

/* Undo uvc_media_start(). uvc_lock held. */
static int uvc_media_stop(void)
{
	if (uvc_running == 0U) {
		RTK_LOGS(TAG, RTK_LOG_INFO, "Already stopped\n");
		return 0;
	}
	if (uvc_threads_stop() != 0) {
		return -1;
	}
	uvc_media_teardown();
	uvc_running = 0U;
	RTK_LOGS(TAG, RTK_LOG_INFO, "USBD UVC demo stop\n");
	return 0;
}

#if USBD_UVC_NEW_DFU
/*
 * USB hand-over with the built-in USB DFU (usbd_dfu_new, see example_usbd_dfu_new.c). USBDFU=1
 * releases the USB port: pause the links (siso_pause() returns once the sink is idle), then tear
 * the UVC class and the USB core down. The video channels keep running, their frames are dropped.
 * USBDFU=0 brings the cameras back if they were running. DFU control thread context.
 */
int usbd_dfu_new_usb_release(void)
{
	u8 i;

	if (uvc_lock == NULL) {
		return 0;
	}
	xSemaphoreTake(uvc_lock, portMAX_DELAY);
	/* Also while stopped: USBUVC=1 must not take the port from DFU */
	uvc_dfu_owns_usb = 1U;
	if ((uvc_running != 0U) && (usbd_uvc_get_inst_num() != 0U)) {
		for (i = 0U; i < uvc_cam_num; i++) {
			if (uvc_siso[i] != NULL) {
				siso_pause(uvc_siso[i]);
			}
		}
		usbd_uvc_deinit();
		usbd_deinit();
		RTK_LOGS(TAG, RTK_LOG_INFO, "UVC released the USB port for DFU\n");
	}
	xSemaphoreGive(uvc_lock);
	return 0;
}

void usbd_dfu_new_usb_restore(void)
{
	u8 i;

	if (uvc_lock == NULL) {
		return;
	}
	xSemaphoreTake(uvc_lock, portMAX_DELAY);
	uvc_dfu_owns_usb = 0U;
	if ((uvc_running == 0U) || (usbd_uvc_get_inst_num() != 0U)) {
		/* Stopped (USBUVC=0) while DFU had the port, or never released */
		goto exit;
	}
	if (usbd_init(&uvc_usb_cfg) != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "UVC restore: usbd_init fail\n");
		goto exit;
	}
	if (usbd_uvc_init_multi(uvc_inst_cfg, (u8)(sizeof(uvc_inst_cfg) / sizeof(uvc_inst_cfg[0]))) != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "UVC restore: usbd_uvc_init_multi fail\n");
		usbd_deinit();
		goto exit;
	}
	for (i = 0U; i < uvc_cam_num; i++) {
		if (uvc_sink_ctx[i] != NULL) {
			mm_module_ctrl(uvc_sink_ctx[i], CMD_USBD_UVC_SET_INSTANCE, i);
		}
		if (uvc_siso[i] != NULL) {
			siso_resume(uvc_siso[i]);
		}
	}
	RTK_LOGS(TAG, RTK_LOG_INFO, "UVC restored\n");
exit:
	xSemaphoreGive(uvc_lock);
}
#endif

/* N stop/start rounds, resources printed after each; a leak shows as heap going down round by round */
static void uvc_media_cycle(u32 rounds)
{
	u32 i;

	uvc_print_res("Cycle begin");
	for (i = 1U; i <= rounds; i++) {
		xSemaphoreTake(uvc_lock, portMAX_DELAY);
		if (uvc_media_stop() != 0) {
			xSemaphoreGive(uvc_lock);
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Cycle %u: stop fail\n", i);
			return;
		}
		uvc_print_res("Cycle stopped");
		if (uvc_media_start() != 0) {
			xSemaphoreGive(uvc_lock);
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Cycle %u: start fail\n", i);
			return;
		}
		xSemaphoreGive(uvc_lock);
		/* Let the host enumerate and open the cameras before the next stop */
		vTaskDelay(pdMS_TO_TICKS(UVC_CYCLE_RUN_MS));
		RTK_LOGS(TAG, RTK_LOG_INFO, "Cycle %u/%u done\n", i, rounds);
		uvc_print_res("Cycle running");
	}
	RTK_LOGS(TAG, RTK_LOG_INFO, "Cycle test done, %u rounds\n", rounds);
}

/* Console: USBUVC=<0|1|2[,rounds]>  0: stop, 1: start, 2: stop/start rounds; no argument: status */
static void USBUVC(void *arg)
{
	int argc;
	char *argv[MAX_ARGC] = {0};
	uvc_ctrl_msg_t msg = {0};

	argc = parse_param((char *)arg, argv);
	if ((argc >= 2) && (strcmp(argv[1], "0") == 0)) {
		msg.evt = UVC_EVT_STOP;
	} else if ((argc >= 2) && (strcmp(argv[1], "1") == 0)) {
		msg.evt = UVC_EVT_START;
	} else if ((argc >= 2) && (strcmp(argv[1], "2") == 0)) {
		msg.evt = UVC_EVT_CYCLE;
		msg.rounds = (argc >= 3) ? (u32)atoi(argv[2]) : UVC_CYCLE_DEFAULT;
		if (msg.rounds == 0U) {
			msg.rounds = UVC_CYCLE_DEFAULT;
		}
	} else {
		RTK_LOGS(TAG, RTK_LOG_INFO, "Usage: USBUVC=<0|1|2[,rounds]>  0: stop, 1: start, 2: stop/start rounds\n");
		RTK_LOGS(TAG, RTK_LOG_INFO, "UVC %s%s\n", (uvc_running != 0U) ? "running" : "stopped",
				 (uvc_dfu_owns_usb != 0U) ? ", USB used by DFU" : "");
		uvc_print_res("Now");
		return;
	}
	if (xQueueSend(uvc_ctrl_queue, &msg, 0) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_WARN, "Busy, try again\n");
	}
}

static log_item_t uvc_cmd_items[] = {
	{"USBUVC", USBUVC,},
};

/* Starts the cameras at boot, then runs USBUVC requests (the video API needs this task's secure context) */
static void uvc_ctrl_thread(void *param)
{
	uvc_ctrl_msg_t msg;

	UNUSED(param);
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	xSemaphoreTake(uvc_lock, portMAX_DELAY);
	(void)uvc_media_start();
	xSemaphoreGive(uvc_lock);
	uvc_print_res("Boot");

	for (;;) {
		if (xQueueReceive(uvc_ctrl_queue, &msg, portMAX_DELAY) != pdPASS) {
			continue;
		}
		if (msg.evt == UVC_EVT_CYCLE) {
			uvc_media_cycle(msg.rounds);
			continue;
		}
		xSemaphoreTake(uvc_lock, portMAX_DELAY);
		if (msg.evt == UVC_EVT_START) {
			(void)uvc_media_start();
		} else {
			(void)uvc_media_stop();
		}
		xSemaphoreGive(uvc_lock);
		uvc_print_res((msg.evt == UVC_EVT_START) ? "Start" : "Stop");
	}
}

/* Exported functions --------------------------------------------------------*/

void example_usbd_uvc_new_media(void)
{
	/* Before example_usbd_dfu_new(): the DFU hooks take uvc_lock */
	uvc_lock = xSemaphoreCreateMutex();
	uvc_ctrl_queue = xQueueCreate(4, sizeof(uvc_ctrl_msg_t));
	if ((uvc_lock == NULL) || (uvc_ctrl_queue == NULL)) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "No mem\n");
		return;
	}
	log_service_add_table(uvc_cmd_items, sizeof(uvc_cmd_items) / sizeof(uvc_cmd_items[0]));
	if (xTaskCreate(uvc_ctrl_thread, "usbd_uvc_new_ctrl", UVC_CTRL_THREAD_STACK_SIZE, NULL,
					UVC_CTRL_THREAD_PRIORITY, NULL) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create USBD UVC ctrl thread fail\n");
	}
}

#endif /* USBD_UVC_NEW_MODE != USBD_UVC_NEW_MODE_SAMPLE */
