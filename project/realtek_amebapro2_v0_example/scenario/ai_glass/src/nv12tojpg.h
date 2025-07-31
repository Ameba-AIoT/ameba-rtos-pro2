/******************************************************************************
*
* Copyright(c) 2007 - 2023 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#ifndef _NV12TOJPEG_H_
#define _NV12TOJPEG_H_

#include <stdint.h>
#include <stdlib.h>

/******************************************************************************
* Customized Function
******************************************************************************/

/*
 * brief: jpeg encode from NV12
 *
 * [in] nv12_data : NV12 data to the destination
 * [in] nv12_width : input width
 * [in] nv12_height : input height
 * [out] p_jpeg_buf : pointer to jpeg output buffer
 * [in] jpeg_buf_size : jpeg output buffer size
 * [out] p_jpeg_size : pointer to jpeg size
 */
void custom_jpegEnc_from_nv12(uint8_t *nv12_data, uint32_t nv12_width, uint32_t nv12_height, uint8_t *p_jpeg_buf, uint32_t jpeg_quality, uint32_t jpeg_buf_size,
							  uint32_t *p_jpeg_size);

/*
 * brief: resize for NV12
 *
 * [in] input_data : input NV12 data to the destination
 * [in] input_width : input width
 * [in] input_height : input height
 * [out] output_data_data : output_data NV12 data to the destination
 * [in] output_data_width : output_data width
 * [in] output_data_height : output_data height
 */
void custom_resize_for_nv12(uint8_t *input_data, uint32_t input_width, uint32_t input_height, uint8_t *output_data, uint32_t output_width,
							uint32_t output_height);

#endif /* _NV12TOJPEG_H_ */