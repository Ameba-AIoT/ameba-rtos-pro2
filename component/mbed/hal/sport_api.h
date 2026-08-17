/**************************************************************************//**
 * @file     sport_api.h
 * @brief    This file defines the SPORT Mbed HAL API functions.
 *
 * @version  V1.00
 * @date     2017-05-03
 *
 * @note
 *
 ******************************************************************************
 * @attention
 *
 * Copyright(c) 2007 - 2022 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */
#ifndef MBED_EXT_SPORT_API_EXT_H
#define MBED_EXT_SPORT_API_EXT_H

#include "device.h"
#include "hal_sport.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup sport SPORT
 *  @ingroup    hal
 *  @brief      sport functions
 *  @{
 */

///@name AmebaPro2
///@{

typedef struct sport_s sport_t;

typedef void (*sport_irq_cb_t)(u32 *arg, u8 *pbuf);

// Type aliases matching the HAL types (without _t suffix as used in sport_api.c)
typedef sport_dev_mode_t sport_ms_mode;
typedef sport_sel_ch_t   sport_sel_chan;
typedef sport_format_t   sport_format;
typedef sport_ml_t       sport_ml;
typedef sport_mclk_t     sport_mclk;
typedef sport_ch_t       sport_ch;
typedef sport_cl_t       sport_cl;
typedef sport_dl_t       sport_dl;
typedef sport_rate_t     sport_rate;

/**
 * @brief Initialize the SPORT device.
 * @param obj The pointer of the SPORT device.
 */
void sport_init(sport_t *obj);

/**
 * @brief Deinitialize the SPORT device.
 * @param obj The pointer of the SPORT device.
 */
void sport_deinit(sport_t *obj);

/**
 * @brief Reset the SPORT device.
 * @param obj The pointer of the SPORT device.
 */
void sport_reset(sport_t *obj);

/**
 * @brief Set the SPORT device as master or slave.
 * @param obj The pointer of the SPORT device.
 * @param ms_mode The master/slave mode.
 */
void sport_set_master(sport_t *obj, sport_ms_mode ms_mode);

/**
 * @brief Enable or disable loopback mode.
 * @param obj The pointer of the SPORT device.
 * @param loopback_en Enable or disable loopback.
 */
void sport_set_loopback(sport_t *obj, BOOL loopback_en);

/**
 * @brief Set the TX and RX channel selection.
 * @param obj The pointer of the SPORT device.
 * @param tx_sel The TX channel selection.
 * @param rx_sel The RX channel selection.
 */
void sport_set_lr_channel(sport_t *obj, sport_sel_chan tx_sel, sport_sel_chan rx_sel);

/**
 * @brief Set the SPORT format (I2S, left-justified, etc.).
 * @param obj The pointer of the SPORT device.
 * @param format The SPORT format.
 */
void sport_set_format(sport_t *obj, sport_format format);

/**
 * @brief Set the data direction (MSB first or LSB first).
 * @param obj The pointer of the SPORT device.
 * @param tx0_ml TX0 data direction.
 * @param tx1_ml TX1 data direction.
 * @param rx0_ml RX0 data direction.
 * @param rx1_ml RX1 data direction.
 */
void sport_data_dir(sport_t *obj, sport_ml tx0_ml, sport_ml tx1_ml, sport_ml rx0_ml, sport_ml rx1_ml);

/**
 * @brief Enable or disable left/right channel swap.
 * @param obj The pointer of the SPORT device.
 * @param tx0_en TX0 swap enable.
 * @param tx1_en TX1 swap enable.
 * @param rx0_en RX0 swap enable.
 * @param rx1_en RX1 swap enable.
 */
void sport_lr_swap(sport_t *obj, BOOL tx0_en, BOOL tx1_en, BOOL rx0_en, BOOL rx1_en);

/**
 * @brief Enable or disable byte swap.
 * @param obj The pointer of the SPORT device.
 * @param tx0_en TX0 byte swap enable.
 * @param tx1_en TX1 byte swap enable.
 * @param rx0_en RX0 byte swap enable.
 * @param rx1_en RX1 byte swap enable.
 */
void sport_byte_swap(sport_t *obj, BOOL tx0_en, BOOL tx1_en, BOOL rx0_en, BOOL rx1_en);

/**
 * @brief Set the SCK inverse.
 * @param obj The pointer of the SPORT device.
 * @param sck_inv_en Enable or disable SCK inverse.
 */
void sport_set_sck_inv(sport_t *obj, BOOL sck_inv_en);

/**
 * @brief Set the MCLK.
 * @param obj The pointer of the SPORT device.
 * @param m_speed The MCLK speed.
 * @param mclk_en Enable or disable MCLK.
 */
void sport_set_mclk(sport_t *obj, sport_mclk m_speed, BOOL mclk_en);

/**
 * @brief Configure TX parameters.
 * @param obj The pointer of the SPORT device.
 * @param ch_num The channel number.
 * @param ch_len The channel length.
 * @param tx0_data_len The TX0 data length.
 * @param tx1_data_len The TX1 data length.
 * @param rate The sample rate.
 */
void sport_tx_params(sport_t *obj, sport_ch ch_num, sport_cl ch_len, sport_dl tx0_data_len, sport_dl tx1_data_len, sport_rate rate);

/**
 * @brief Configure RX parameters.
 * @param obj The pointer of the SPORT device.
 * @param ch_num The channel number.
 * @param ch_len The channel length.
 * @param rx0_data_len The RX0 data length.
 * @param rx1_data_len The RX1 data length.
 * @param rate The sample rate.
 */
void sport_rx_params(sport_t *obj, sport_ch ch_num, sport_cl ch_len, sport_dl rx0_data_len, sport_dl rx1_data_len, sport_rate rate);

/**
 * @brief Set the DMA buffer.
 * @param obj The pointer of the SPORT device.
 * @param ptx0_buf The TX0 buffer.
 * @param ptx1_buf The TX1 buffer.
 * @param prx0_buf The RX0 buffer.
 * @param prx1_buf The RX1 buffer.
 * @param page_num The page number.
 * @param page_size The page size.
 */
void sport_dma_buffer(sport_t *obj, u8 *ptx0_buf, u8 *ptx1_buf, u8 *prx0_buf, u8 *prx1_buf, u32 page_num, u32 page_size);

/**
 * @brief Set the FIFO threshold.
 * @param obj The pointer of the SPORT device.
 * @param tx_fifo_th The TX FIFO threshold.
 * @param rx_fifo_th The RX FIFO threshold.
 */
void sport_fifo_th(sport_t *obj, u8 tx_fifo_th, u8 rx_fifo_th);

/**
 * @brief Register the FIFO callback handler.
 * @param obj The pointer of the SPORT device.
 * @param handler The callback function.
 * @param parg The callback argument.
 */
void sport_fifo_cb_handler(sport_t *obj, sport_irq_cb_t handler, void *parg);

/**
 * @brief Register the TX0 DMA callback handler.
 * @param obj The pointer of the SPORT device.
 * @param handler The callback function.
 * @param parg The callback argument.
 */
void sport_tx0_dma_cb_handler(sport_t *obj, sport_irq_cb_t handler, void *parg);

/**
 * @brief Register the TX1 DMA callback handler.
 * @param obj The pointer of the SPORT device.
 * @param handler The callback function.
 * @param parg The callback argument.
 */
void sport_tx1_dma_cb_handler(sport_t *obj, sport_irq_cb_t handler, void *parg);

/**
 * @brief Register the RX0 DMA callback handler.
 * @param obj The pointer of the SPORT device.
 * @param handler The callback function.
 * @param parg The callback argument.
 */
void sport_rx0_dma_cb_handler(sport_t *obj, sport_irq_cb_t handler, void *parg);

/**
 * @brief Register the RX1 DMA callback handler.
 * @param obj The pointer of the SPORT device.
 * @param handler The callback function.
 * @param parg The callback argument.
 */
void sport_rx1_dma_cb_handler(sport_t *obj, sport_irq_cb_t handler, void *parg);

/**
 * @brief Get the TX0 page buffer.
 * @param obj The pointer of the SPORT device.
 * @return The pointer to the TX0 page buffer.
 */
int *sport_get_tx0_page(sport_t *obj);

/**
 * @brief Get the TX1 page buffer.
 * @param obj The pointer of the SPORT device.
 * @return The pointer to the TX1 page buffer.
 */
int *sport_get_tx1_page(sport_t *obj);

/**
 * @brief Send the TX0 page.
 * @param obj The pointer of the SPORT device.
 * @param pbuf The pointer to the page buffer.
 */
void sport_tx0_page_send(sport_t *obj, u32 *pbuf);

/**
 * @brief Send the TX1 page.
 * @param obj The pointer of the SPORT device.
 * @param pbuf The pointer to the page buffer.
 */
void sport_tx1_page_send(sport_t *obj, u32 *pbuf);

/**
 * @brief Get the TX0 error count.
 * @param obj The pointer of the SPORT device.
 * @return The error count.
 */
u8 sport_get_tx0_error_cnt(sport_t *obj);

/**
 * @brief Get the TX1 error count.
 * @param obj The pointer of the SPORT device.
 * @return The error count.
 */
u8 sport_get_tx1_error_cnt(sport_t *obj);

/**
 * @brief Get the RX0 error count.
 * @param obj The pointer of the SPORT device.
 * @return The error count.
 */
u8 sport_get_rx0_error_cnt(sport_t *obj);

/**
 * @brief Get the RX1 error count.
 * @param obj The pointer of the SPORT device.
 * @return The error count.
 */
u8 sport_get_rx1_error_cnt(sport_t *obj);

/**
 * @brief Clean the error count.
 * @param obj The pointer of the SPORT device.
 */
void sport_clean_error_cnt(sport_t *obj);

/**
 * @brief Start the TX DMA.
 * @param obj The pointer of the SPORT device.
 */
void sport_tx_start(sport_t *obj);

/**
 * @brief Start the RX DMA.
 * @param obj The pointer of the SPORT device.
 */
void sport_rx_start(sport_t *obj);

/**
 * @brief Start both TX and RX DMA.
 * @param obj The pointer of the SPORT device.
 */
void sport_trx_start(sport_t *obj);

/**
 * @brief Stop the TX DMA.
 * @param obj The pointer of the SPORT device.
 */
void sport_tx_stop(sport_t *obj);

/**
 * @brief Stop the RX DMA.
 * @param obj The pointer of the SPORT device.
 */
void sport_rx_stop(sport_t *obj);

/**
 * @brief Stop both TX and RX DMA.
 * @param obj The pointer of the SPORT device.
 */
void sport_trx_stop(sport_t *obj);

///@}

#ifdef __cplusplus
}
#endif

#endif  // end of "#define MBED_EXT_SPORT_API_EXT_H"