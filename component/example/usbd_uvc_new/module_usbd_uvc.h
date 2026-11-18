/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _MODULE_USBD_UVC_H
#define _MODULE_USBD_UVC_H

/*
 * usbd_uvc_module: MMF video sink for one camera (instance) of the usb_stack/v1.0.0 UVC class
 * (component/usb/usb_stack/v1.0.0/device/uvc). Replaces the legacy module_uvcd for usb_stack=new.
 *
 * The application brings up the USB stack itself (usbd_init() + usbd_uvc_init_multi()), then opens
 * one usbd_uvc_module per camera and binds it with CMD_USBD_UVC_SET_INSTANCE. Frames that arrive
 * while the host is not streaming, or whose codec does not match the format the host committed,
 * are dropped; H.264/HEVC is held back until the first I frame after every stream start.
 *
 * Part of the usbd_uvc_new example (built by usbd_uvc_new.cmake with the usb_stack/v1.0.0
 * -iquote paths), requires usb_stack=new.
 */

#include <stdint.h>
#include "mmf2_module.h"

/* Bind to UVC instance arg (0..USBD_UVC_INST_NUM-1); usbd_uvc_init_multi() must have run */
#define CMD_USBD_UVC_SET_INSTANCE       MM_MODULE_CMD(0x00)
/* Copy the format the host committed into (usbd_uvc_module_format_t *)arg */
#define CMD_USBD_UVC_GET_FORMAT         MM_MODULE_CMD(0x01)
/* Block until the host starts streaming (SET_INTERFACE alt 1), then copy its format into
   (usbd_uvc_module_format_t *)arg. Call from the application task that reconfigures video. */
#define CMD_USBD_UVC_WAIT_STREAM        MM_MODULE_CMD(0x02)
/* Frames sent / dropped since the bind (arg: usbd_uvc_module_stat_t *) */
#define CMD_USBD_UVC_GET_STAT           MM_MODULE_CMD(0x03)
/* Wake a task blocked in CMD_USBD_UVC_WAIT_STREAM (application stop); returns -1 if not bound */
#define CMD_USBD_UVC_WAKE               MM_MODULE_CMD(0x04)

/* usbd_uvc_module_format_t.format, same values as USBD_UVC_FORMAT_TYPE_* in usbd_video.h */
#define USBD_UVC_MOD_FMT_NV16           0   /* advertised as NV16 (YUV422 semi-planar) */
#define USBD_UVC_MOD_FMT_NV12           1
#define USBD_UVC_MOD_FMT_MJPEG          2
#define USBD_UVC_MOD_FMT_H264           3
#define USBD_UVC_MOD_FMT_H265           4

typedef struct {
	uint32_t format;    /* USBD_UVC_MOD_FMT_* */
	uint32_t width;
	uint32_t height;
	uint32_t fps;
} usbd_uvc_module_format_t;

typedef struct {
	uint32_t sent;      /* frames handed to the UVC class */
	uint32_t dropped;   /* not streaming excluded: wrong codec for the committed format, waiting for an I frame */
} usbd_uvc_module_stat_t;

extern mm_module_t usbd_uvc_module;

#endif /* _MODULE_USBD_UVC_H */
