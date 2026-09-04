/**
  ******************************************************************************
  * @file    uvc_to_rgb.h
  * @author  Realtek WLAN5 Team
  * @brief   Convert UVC captured frames (JPEG/YUY2) to RGB888
  ******************************************************************************
  * @attention
  *
  * This module is a confidential and proprietary property of RealTek and
  * possession or use of this module requires written permission of RealTek.
  *
  * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
  ******************************************************************************
  */

#ifndef __UVC_TO_RGB_H
#define __UVC_TO_RGB_H

#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/

/* Output RGB format selection */
#define UVC_RGB_FORMAT_RGB888    0   /* RGBRGBRGB...  (3 bytes per pixel) */
#define UVC_RGB_FORMAT_BGR888    1   /* BGRBGRBGR...  (3 bytes per pixel) */
#define UVC_RGB_FORMAT_RGB565    2   /* RGB565 little-endian (2 bytes per pixel) */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Convert YUY2 (YUYV) packed frame to RGB888 or BGR888
  * @param  src:   input YUY2 buffer (width * height * 2 bytes)
  * @param  width: frame width in pixels
  * @param  height: frame height in pixels
  * @param  dst:   output RGB buffer
  *                If NULL, memory is allocated with rtw_malloc (caller must free).
  * @param  format: UVC_RGB_FORMAT_RGB888 or UVC_RGB_FORMAT_BGR888
  * @retval Pointer to output RGB buffer (same as dst if non-NULL, or newly allocated)
  */
uint8_t *uvc_yuy2_to_rgb(uint8_t *src, int width, int height,
						 uint8_t *dst, int format);

/**
  * @brief  Decode JPEG buffer to RGB888 or BGR888 using stb_image
  * @param  src:    input JPEG data buffer
  * @param  src_len: length of JPEG data
  * @param  width:  output: decoded image width
  * @param  height: output: decoded image height
  * @param  dst:    output RGB buffer.
  *                 If NULL, memory is allocated via stb_image (caller must free
  *                 with uvc_rgb_free()).
  * @param  format: UVC_RGB_FORMAT_RGB888 or UVC_RGB_FORMAT_BGR888
  * @retval Pointer to output RGB buffer, or NULL on failure
  */
uint8_t *uvc_jpeg_to_rgb(uint8_t *src, int src_len,
						 int *width, int *height,
						 uint8_t *dst, int format);

/**
  * @brief  Free RGB buffer allocated by uvc_jpeg_to_rgb
  */
void uvc_rgb_free(uint8_t *buf);

#endif /* __UVC_TO_RGB_H */
