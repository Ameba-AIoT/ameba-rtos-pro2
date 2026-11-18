/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MMF sink for one UVC instance of the usb_stack/v1.0.0 UVC class, see module_usbd_uvc.h.
 * Independent of the legacy module_uvcd / usb_class UVC driver.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "usbd.h"
#include "usbd_uvc.h"
#include "mmf2_module.h"
#include "module_usbd_uvc.h"
#include "avcodec.h"

/* NAL unit types, Ref H.264 Table 7-1 and H.265 Table 7-1 */
#define USBD_UVC_MOD_H264_NAL_TYPE(b)       ((b) & 0x1FU)
#define USBD_UVC_MOD_H264_NAL_SLICE         1U
#define USBD_UVC_MOD_H264_NAL_IDR           5U
#define USBD_UVC_MOD_HEVC_NAL_TYPE(b)       (((b) >> 1) & 0x3FU)
#define USBD_UVC_MOD_HEVC_NAL_VCL_MAX       31U
#define USBD_UVC_MOD_HEVC_NAL_IRAP_MIN      16U   /* BLA_W_LP */
#define USBD_UVC_MOD_HEVC_NAL_IRAP_MAX      21U   /* CRA_NUT */

/* Upper bound on frames dropped while waiting for an I frame (~5 s at 30 fps). Open-GOP or
   intra-refresh streams may never carry an IDR/IRAP, the host would stay on a black screen. */
#define USBD_UVC_MOD_IFRAME_MAX_DROP        150U

typedef struct {
	usbd_uvc_buffer_t payload;      /* The one buffer handed back and forth with the class */
	volatile u8 wait_iframe;        /* Drop H.264/HEVC until an I frame, set at every stream start */
	volatile u8 bound;
	u8 inst;
	u16 iframe_drop;
	u32 drop_cnt;
	u32 sent_cnt;
	u8 mismatch_logged;
	usb_os_sema_t wait_sema;        /* Semaphore of the last WAIT_STREAM, kept by the class across deinit */
} usbd_uvc_mod_ctx_t;

static const char *const TAG = "USBD_UVC_MOD";

/* Context bound to each UVC instance, looked up from the ISR stream-start callback */
static usbd_uvc_mod_ctx_t *usbd_uvc_mod_bound[USBD_UVC_INST_NUM];

/* Return 1 if the first slice of the H.264/HEVC access unit is an I frame (H.264 IDR, HEVC IRAP) */
static int usbd_uvc_mod_is_iframe(const u8 *buf, u32 size, u32 codec)
{
	u32 i;
	u8 type;

	for (i = 0U; (i + 3U) < size; i++) {
		if ((buf[i] != 0U) || (buf[i + 1U] != 0U) || (buf[i + 2U] != 1U)) {
			continue;
		}
		/* SPS/PPS/VPS/SEI come before the first slice, only that slice tells the frame type */
		if (codec == AV_CODEC_ID_H264) {
			type = USBD_UVC_MOD_H264_NAL_TYPE(buf[i + 3U]);
			if ((type >= USBD_UVC_MOD_H264_NAL_SLICE) && (type <= USBD_UVC_MOD_H264_NAL_IDR)) {
				return (type == USBD_UVC_MOD_H264_NAL_IDR) ? 1 : 0;
			}
		} else {
			type = USBD_UVC_MOD_HEVC_NAL_TYPE(buf[i + 3U]);
			if (type <= USBD_UVC_MOD_HEVC_NAL_VCL_MAX) {
				return ((type >= USBD_UVC_MOD_HEVC_NAL_IRAP_MIN) && (type <= USBD_UVC_MOD_HEVC_NAL_IRAP_MAX)) ? 1 : 0;
			}
		}
		i += 2U;
	}
	return 0;
}

/* UVC format the codec of an MMF item is sent as, -1 if it cannot be sent */
static int usbd_uvc_mod_codec_to_fmt(u32 codec)
{
	switch (codec) {
	case AV_CODEC_ID_H264:
		return USBD_UVC_FORMAT_TYPE_H264;
	case AV_CODEC_ID_H265:
		return USBD_UVC_FORMAT_TYPE_H265;
	case AV_CODEC_ID_MJPEG:
		return USBD_UVC_FORMAT_TYPE_MJPEG;
	case AV_CODEC_ID_NV12:
		return USBD_UVC_FORMAT_TYPE_NV12;
	case AV_CODEC_ID_NV16:
		return USBD_UVC_FORMAT_TYPE_YUY2;
	default:
		return -1;
	}
}

/* Host started streaming on one instance (SET_INTERFACE alt 1). ISR context. */
static void usbd_uvc_mod_stream_start(void *ctx)
{
	usbd_uvc_format_t *fmt = (usbd_uvc_format_t *)ctx;
	usbd_uvc_mod_ctx_t *mod;
	u8 i;

	for (i = 0U; i < USBD_UVC_INST_NUM; i++) {
		mod = usbd_uvc_mod_bound[i];
		if ((fmt != NULL) && (mod != NULL) && (usbd_uvc_get_format_inst(i) == fmt)) {
			mod->wait_iframe = 1U;
			mod->iframe_drop = 0U;
			mod->mismatch_logged = 0U;
			usb_os_sema_give(fmt->uvcd_change_sema);
			return;
		}
	}
}

static void usbd_uvc_mod_copy_format(u8 inst, usbd_uvc_module_format_t *out)
{
	usbd_uvc_format_t *fmt = usbd_uvc_get_format_inst(inst);

	if ((fmt == NULL) || (out == NULL)) {
		return;
	}
	out->format = fmt->format;
	out->width = fmt->width;
	out->height = fmt->height;
	out->fps = fmt->fps;
}

/* Push one frame to the bound instance and wait until it is copied into the payload ring */
static void usbd_uvc_mod_send(usbd_uvc_mod_ctx_t *mod, const mm_queue_item_t *item)
{
	usbd_uvc_format_t *fmt = usbd_uvc_get_format_inst(mod->inst);
	usbd_uvc_buffer_t *payload = NULL;
	int item_fmt = usbd_uvc_mod_codec_to_fmt(item->type);

	if ((fmt == NULL) || (usbd_uvc_get_status_inst(mod->inst) == 0)) {
		mod->wait_iframe = 1U;
		mod->iframe_drop = 0U;
		return;
	}

	/* While the application switches the video channel, frames of the old format still arrive */
	if ((item_fmt < 0) || ((u32)item_fmt != fmt->format)) {
		mod->drop_cnt++;
		if (mod->mismatch_logged == 0U) {
			mod->mismatch_logged = 1U;
			RTK_LOGS(TAG, RTK_LOG_INFO, "UVC%d drop codec %d, host wants format %d\n", mod->inst, item->type, fmt->format);
		}
		return;
	}

	if ((mod->wait_iframe != 0U) && ((item->type == AV_CODEC_ID_H264) || (item->type == AV_CODEC_ID_H265))) {
		if (usbd_uvc_mod_is_iframe((const u8 *)item->data_addr, item->size, item->type) == 0) {
			mod->iframe_drop++;
			mod->drop_cnt++;
			if (mod->iframe_drop < USBD_UVC_MOD_IFRAME_MAX_DROP) {
				return;
			}
			RTK_LOGS(TAG, RTK_LOG_WARN, "UVC%d no I frame after %d drops, pass through\n", mod->inst, mod->iframe_drop);
		}
		mod->wait_iframe = 0U;
		mod->iframe_drop = 0U;
		RTK_LOGS(TAG, RTK_LOG_INFO, "UVC%d first I frame\n", mod->inst);
	}

	do {
		payload = usbd_uvc_video_out_stream_queue_inst(mod->inst);
		if (payload == NULL) {
			/* Streaming stopped (alt 0) or the class was deinitialized: the buffer may never
			   come back, do not spin on it */
			if (usbd_uvc_get_status_inst(mod->inst) == 0) {
				mod->wait_iframe = 1U;
				mod->iframe_drop = 0U;
				return;
			}
			vTaskDelay(1);
		}
	} while (payload == NULL);

	payload->mem = (const u8 *)item->data_addr;
	payload->bytesused = item->size;
	usbd_uvc_video_put_in_stream_queue_inst(mod->inst, payload);
	usbd_uvc_wait_frame_down_inst(mod->inst);
	mod->sent_cnt++;
}

static void *usbd_uvc_mod_create(void *parent)
{
	usbd_uvc_mod_ctx_t *mod = (usbd_uvc_mod_ctx_t *)malloc(sizeof(usbd_uvc_mod_ctx_t));

	(void)parent;
	if (mod != NULL) {
		memset(mod, 0, sizeof(*mod));
	}
	return mod;
}

static int usbd_uvc_mod_control(void *p, int cmd, int arg)
{
	usbd_uvc_mod_ctx_t *mod = (usbd_uvc_mod_ctx_t *)p;
	usbd_uvc_format_t *fmt;

	switch (cmd) {
	case CMD_USBD_UVC_SET_INSTANCE:
		if (((u32)arg >= USBD_UVC_INST_NUM) || (usbd_uvc_get_format_inst((u8)arg) == NULL)) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "UVC instance %d not initialized\n", arg);
			return -1;
		}
		if ((usbd_uvc_mod_bound[arg] != NULL) && (usbd_uvc_mod_bound[arg] != mod)) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "UVC instance %d already bound\n", arg);
			return -1;
		}
		/* The class keeps the stream-start semaphore across deinit: drop a wake-up left by the
		   previous run (CMD_USBD_UVC_WAKE at stop) so WAIT_STREAM does not return at once */
		(void)usb_os_sema_take(usbd_uvc_get_format_inst((u8)arg)->uvcd_change_sema, 0U);
		mod->inst = (u8)arg;
		mod->wait_iframe = 1U;
		mod->iframe_drop = 0U;
		mod->drop_cnt = 0U;
		mod->sent_cnt = 0U;
		INIT_LIST_HEAD(&mod->payload.buffer_list);
		usbd_uvc_video_put_out_stream_queue_inst(mod->inst, &mod->payload);
		usbd_uvc_mod_bound[arg] = mod;
		usbd_uvc_set_change_parm_cb_inst(mod->inst, usbd_uvc_mod_stream_start);
		mod->bound = 1U;
		break;
	case CMD_USBD_UVC_GET_FORMAT:
		if (mod->bound == 0U) {
			return -1;
		}
		usbd_uvc_mod_copy_format(mod->inst, (usbd_uvc_module_format_t *)arg);
		break;
	case CMD_USBD_UVC_WAIT_STREAM:
		fmt = (mod->bound != 0U) ? usbd_uvc_get_format_inst(mod->inst) : NULL;
		if (fmt == NULL) {
			return -1;
		}
		mod->wait_sema = fmt->uvcd_change_sema;
		usb_os_sema_take(fmt->uvcd_change_sema, USB_OS_SEMA_TIMEOUT);
		usbd_uvc_mod_copy_format(mod->inst, (usbd_uvc_module_format_t *)arg);
		break;
	case CMD_USBD_UVC_WAKE:
		/* The waiter may be blocked while the class is down (USB handed to DFU): use the semaphore
		   it waits on, the class never deletes it */
		if (mod->wait_sema == NULL) {
			return -1;
		}
		usb_os_sema_give(mod->wait_sema);
		break;
	case CMD_USBD_UVC_GET_STAT:
		if (arg != 0) {
			((usbd_uvc_module_stat_t *)arg)->sent = mod->sent_cnt;
			((usbd_uvc_module_stat_t *)arg)->dropped = mod->drop_cnt;
		}
		break;
	default:
		break;
	}
	return 0;
}

static int usbd_uvc_mod_handle(void *p, void *input, void *output)
{
	usbd_uvc_mod_ctx_t *mod = (usbd_uvc_mod_ctx_t *)p;

	(void)output;
	if (mod->bound != 0U) {
		usbd_uvc_mod_send(mod, (const mm_queue_item_t *)input);
	}
	return 0;
}

static void *usbd_uvc_mod_destroy(void *p)
{
	usbd_uvc_mod_ctx_t *mod = (usbd_uvc_mod_ctx_t *)p;

	if (mod != NULL) {
		if ((mod->bound != 0U) && (usbd_uvc_mod_bound[mod->inst] == mod)) {
			usbd_uvc_set_change_parm_cb_inst(mod->inst, NULL);
			usbd_uvc_mod_bound[mod->inst] = NULL;
		}
		/* The USB stack belongs to the application, which tears it down */
		free(mod);
	}
	return NULL;
}

mm_module_t usbd_uvc_module = {
	.create = usbd_uvc_mod_create,
	.destroy = usbd_uvc_mod_destroy,
	.control = usbd_uvc_mod_control,
	.handle = usbd_uvc_mod_handle,

	.new_item = NULL,
	.del_item = NULL,

	.output_type = MM_TYPE_NONE,     /* no output */
	.module_type = MM_TYPE_VSINK,    /* video sink */
	.name = "USBD_UVC"
};
