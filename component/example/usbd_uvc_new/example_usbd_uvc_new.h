/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EXAMPLE_USBD_UVC_NEW_H
#define EXAMPLE_USBD_UVC_NEW_H

/*
 * Example mode, set here or with cmake -DUSBD_UVC_NEW_MODE=<n>:
 * 0 SAMPLE: built-in 1920x1080@30 H.264 clip, no sensor (original example)
 * 1 SINGLE: sensor, one camera, the host switches H264 / HEVC / MJPEG / NV12 / NV16
 * 2 DUAL:   sensor, two cameras at once, UVC0 H264 1920x1080 30 fps + UVC1 NV12 640x360 15 fps
 * Modes 1 and 2 use the new usbd_uvc_module (module_usbd_uvc.c in this example).
 */
#define USBD_UVC_NEW_MODE_SAMPLE    0
#define USBD_UVC_NEW_MODE_SINGLE    1
#define USBD_UVC_NEW_MODE_DUAL      2

#ifndef USBD_UVC_NEW_MODE
#define USBD_UVC_NEW_MODE           USBD_UVC_NEW_MODE_SAMPLE
#endif

/*
 * 1: build the USB DFU firmware update of usbd_uvc_new in (component/example/usbd_dfu_new).
 * The cameras own the USB port; console USBDFU=1 hands it to DFU (0BDA:8D00, dfu-util -D ota.bin),
 * the board reboots into the new firmware, USBDFU=0 gives the port back to the cameras.
 * Set with cmake -DUSBD_UVC_NEW_DFU=OFF to leave it out.
 */
#ifndef USBD_UVC_NEW_DFU
#define USBD_UVC_NEW_DFU            1
#endif

void example_usbd_uvc_new(void);
void example_usbd_uvc_new_media(void);

#endif /* EXAMPLE_USBD_UVC_NEW_H */
