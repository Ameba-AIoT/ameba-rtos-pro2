#ifndef __SPI_INIT_H__
#define __SPI_INIT_H__

/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 * SPDX Slave module for AI Glass
 *
 ******************************************************************************/

/**
 * @brief Start the SPI slave test task.
 *
 * Creates a FreeRTOS task that initializes SPI0 in slave mode,
 * performs a loopback test (TX 4MB, verify RX matches TX),
 * and reports results via dbg_printf.
 *
 * This function is intended to be called from an AT command handler.
 */
void spi_slave_start(void);

#endif //#ifndef __SPI_INIT_H__