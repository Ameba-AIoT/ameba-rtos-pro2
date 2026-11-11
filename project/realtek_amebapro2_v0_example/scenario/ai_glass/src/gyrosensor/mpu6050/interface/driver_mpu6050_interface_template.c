/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_mpu6050_interface_template.c
 * @brief     driver mpu6050 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-06-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/06/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */
#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "hal.h"
#include "log_service.h"
#include <platform_opts.h>
#include "i2c_api.h"
#include "ex_api.h"

#include "driver_mpu6050_interface.h"

static i2c_t i2cmaster;
static int mpu6050_inited = 0;
static int i2c_bus_inited = 0;
static volatile int i2c_err_flag = 0;
#define MPU6050_I2C_MTR_SDA PF_2 //PE_4
#define MPU6050_I2C_MTR_SCL PF_1 //PE_3
#define MPU6050_I2C_BUS_CLK 400000 //一定要 400KHz, 否則無法跟上 FIFO 1000Hz 的速度

static xSemaphoreHandle i2c_rx_done_sema = NULL;
static xSemaphoreHandle i2c_tx_done_sema = NULL;

static void i2c_master_err_callback(void *userdata)
{
	i2c_err_flag = 1;
	xSemaphoreGiveFromISR(i2c_tx_done_sema, NULL);
	xSemaphoreGiveFromISR(i2c_rx_done_sema, NULL);
}

static void i2c_master_rxc_callback(void *userdata)
{
	xSemaphoreGiveFromISR(i2c_rx_done_sema, NULL);
}

static void i2c_master_txc_callback(void *userdata)
{
	xSemaphoreGiveFromISR(i2c_tx_done_sema, NULL);
}

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
extern void i2c_set_user_callback(i2c_t *obj, I2CCallback i2ccb, void(*i2c_callback)(void *));
uint8_t mpu6050_interface_iic_init(void)
{
	if (mpu6050_inited == 1) {
		return 0;
	}
	if (i2c_bus_inited == 0) {
		i2c_rx_done_sema = xSemaphoreCreateBinary();
		i2c_tx_done_sema = xSemaphoreCreateBinary();
		i2c_init(&i2cmaster, MPU6050_I2C_MTR_SDA, MPU6050_I2C_MTR_SCL);
		if (i2cmaster.i2c_adp.status == I2CStatusUninitial) {
			printf("MPU6050: i2c_init failed (pin conflict or init error)\r\n");
			return 1;
		}
		i2c_frequency(&i2cmaster, MPU6050_I2C_BUS_CLK);
		i2c_set_user_callback(&i2cmaster, I2C_RX_COMPLETE, i2c_master_rxc_callback);
		i2c_set_user_callback(&i2cmaster, I2C_TX_COMPLETE, i2c_master_txc_callback);
		i2c_set_user_callback(&i2cmaster, I2C_ERR_OCCURRED, i2c_master_err_callback);
		i2c_bus_inited = 1;
	}
	mpu6050_inited = 1;
	return 0;
}

uint8_t mpu6050_interface_iic_init_with_pin(uint32_t sda, uint32_t scl)
{
	if (mpu6050_inited == 1) {
		return 0;
	}
	if (i2c_bus_inited == 0) {
		i2c_rx_done_sema = xSemaphoreCreateBinary();
		i2c_tx_done_sema = xSemaphoreCreateBinary();
		i2c_init(&i2cmaster, sda, scl);
		if (i2cmaster.i2c_adp.status == I2CStatusUninitial) {
			printf("MPU6050: i2c_init failed (pin conflict or init error)\r\n");
			return 1;
		}
		i2c_frequency(&i2cmaster, MPU6050_I2C_BUS_CLK);
		i2c_set_user_callback(&i2cmaster, I2C_RX_COMPLETE, i2c_master_rxc_callback);
		i2c_set_user_callback(&i2cmaster, I2C_TX_COMPLETE, i2c_master_txc_callback);
		i2c_set_user_callback(&i2cmaster, I2C_ERR_OCCURRED, i2c_master_err_callback);
		i2c_bus_inited = 1;
	}
	mpu6050_inited = 1;
	return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t mpu6050_interface_iic_deinit(void)
{
	if (mpu6050_inited == 0) {
		return 0;
	}
	i2c_reset(&i2cmaster);
	mpu6050_inited = 0;
	i2c_bus_inited = 0;
	return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr is the iic device write address
 * @param[in]  reg is the iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu6050_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
	hal_i2c_adapter_t *phal_i2c_adapter = (hal_i2c_adapter_t *)(&i2cmaster.i2c_adp);

	i2c_err_flag = 0;
	xSemaphoreTake(i2c_tx_done_sema, 0);

	phal_i2c_adapter->tx_dat.addr = addr;
	phal_i2c_adapter->tx_dat.buf = (const uint8_t *)&reg;
	phal_i2c_adapter->tx_dat.len = 1;
	phal_i2c_adapter->tx_dat.mst_stop = 1;
	if (hal_i2c_send(phal_i2c_adapter) != HAL_OK) {
		return 1;
	}
	if (xSemaphoreTake(i2c_tx_done_sema, pdMS_TO_TICKS(50)) != pdTRUE || i2c_err_flag) {
		return 1;
	}

	i2c_err_flag = 0;
	xSemaphoreTake(i2c_rx_done_sema, 0);

	phal_i2c_adapter->rx_dat.addr = addr;
	phal_i2c_adapter->rx_dat.buf = buf;
	phal_i2c_adapter->rx_dat.len = len;
	phal_i2c_adapter->rx_dat.mst_stop = 1;
	if (hal_i2c_receive(phal_i2c_adapter) != HAL_OK) {
		return 1;
	}
	if (xSemaphoreTake(i2c_rx_done_sema, pdMS_TO_TICKS(50)) != pdTRUE || i2c_err_flag) {
		return 1;
	}

	return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr is the iic device write address
 * @param[in] reg is the iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mpu6050_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
	hal_i2c_adapter_t *phal_i2c_adapter = (hal_i2c_adapter_t *)(&i2cmaster.i2c_adp);
	uint8_t data[64] = {0};
	data[0] = reg;
	if (len > 63) {
		mpu6050_interface_debug_print("mpu6050: write length is over 63 bytes.\n");
		return 1;
	}
	for (int i = 0; i < len; i++) {
		data[i + 1] = buf[i];
	}

	i2c_err_flag = 0;
	xSemaphoreTake(i2c_tx_done_sema, 0);

	phal_i2c_adapter->tx_dat.addr = addr;
	phal_i2c_adapter->tx_dat.buf = (const uint8_t *)data;
	phal_i2c_adapter->tx_dat.len = len + 1;
	phal_i2c_adapter->tx_dat.mst_stop = 1;
	if (hal_i2c_send(phal_i2c_adapter) != HAL_OK) {
		return 1;
	}
	if (xSemaphoreTake(i2c_tx_done_sema, pdMS_TO_TICKS(50)) != pdTRUE || i2c_err_flag) {
		return 1;
	}
	return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void mpu6050_interface_delay_ms(uint32_t ms)
{
	vTaskDelay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void mpu6050_interface_debug_print(const char *const fmt, ...)
{
	printf(fmt);
}

/**
 * @brief     interface receive callback
 * @param[in] type is the irq type
 * @note      none
 */
void mpu6050_interface_receive_callback(uint8_t type)
{
	switch (type) {
	case MPU6050_INTERRUPT_MOTION : {
		mpu6050_interface_debug_print("mpu6050: irq motion.\n");

		break;
	}
	case MPU6050_INTERRUPT_FIFO_OVERFLOW : {
		mpu6050_interface_debug_print("mpu6050: irq fifo overflow.\n");

		break;
	}
	case MPU6050_INTERRUPT_I2C_MAST : {
		mpu6050_interface_debug_print("mpu6050: irq i2c master.\n");

		break;
	}
	case MPU6050_INTERRUPT_DMP : {
		mpu6050_interface_debug_print("mpu6050: irq dmp\n");

		break;
	}
	case MPU6050_INTERRUPT_DATA_READY : {
		mpu6050_interface_debug_print("mpu6050: irq data ready\n");

		break;
	}
	default : {
		mpu6050_interface_debug_print("mpu6050: irq unknown code.\n");

		break;
	}
	}
}

/**
 * @brief     interface dmp tap callback
 * @param[in] count is the tap count
 * @param[in] direction is the tap direction
 * @note      none
 */
void mpu6050_interface_dmp_tap_callback(uint8_t count, uint8_t direction)
{
	switch (direction) {
	case MPU6050_DMP_TAP_X_UP : {
		mpu6050_interface_debug_print("mpu6050: tap irq x up with %d.\n", count);

		break;
	}
	case MPU6050_DMP_TAP_X_DOWN : {
		mpu6050_interface_debug_print("mpu6050: tap irq x down with %d.\n", count);

		break;
	}
	case MPU6050_DMP_TAP_Y_UP : {
		mpu6050_interface_debug_print("mpu6050: tap irq y up with %d.\n", count);

		break;
	}
	case MPU6050_DMP_TAP_Y_DOWN : {
		mpu6050_interface_debug_print("mpu6050: tap irq y down with %d.\n", count);

		break;
	}
	case MPU6050_DMP_TAP_Z_UP : {
		mpu6050_interface_debug_print("mpu6050: tap irq z up with %d.\n", count);

		break;
	}
	case MPU6050_DMP_TAP_Z_DOWN : {
		mpu6050_interface_debug_print("mpu6050: tap irq z down with %d.\n", count);

		break;
	}
	default : {
		mpu6050_interface_debug_print("mpu6050: tap irq unknown code.\n");

		break;
	}
	}
}

/**
 * @brief     interface dmp orient callback
 * @param[in] orientation is the dmp orientation
 * @note      none
 */
void mpu6050_interface_dmp_orient_callback(uint8_t orientation)
{
	switch (orientation) {
	case MPU6050_DMP_ORIENT_PORTRAIT : {
		mpu6050_interface_debug_print("mpu6050: orient irq portrait.\n");

		break;
	}
	case MPU6050_DMP_ORIENT_LANDSCAPE : {
		mpu6050_interface_debug_print("mpu6050: orient irq landscape.\n");

		break;
	}
	case MPU6050_DMP_ORIENT_REVERSE_PORTRAIT : {
		mpu6050_interface_debug_print("mpu6050: orient irq reverse portrait.\n");

		break;
	}
	case MPU6050_DMP_ORIENT_REVERSE_LANDSCAPE : {
		mpu6050_interface_debug_print("mpu6050: orient irq reverse landscape.\n");

		break;
	}
	default : {
		mpu6050_interface_debug_print("mpu6050: orient irq unknown code.\n");

		break;
	}
	}
}
