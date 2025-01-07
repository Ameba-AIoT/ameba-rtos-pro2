/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "hal.h"
#include "log_service.h"
#include <stdio.h>
#include <stdlib.h>
#include "gyrosensor_api.h"

#if defined(AI_DEMO_MPU6050)
#include <driver_mpu6050_basic.h>
#include <driver_mpu6050_fifo.h>
#define MPU6050_FIFO_READ
#elif defined(AI_DEMO_ICM42670P)
#include <icm42670p_hal.h>
#endif

static int gyroscope_inited = 0;

int gyroscope_is_inited(void)
{
	return gyroscope_inited;
}

int gyroscope_fifo_init(void)
{
#if defined(AI_DEMO_MPU6050) && defined(MPU6050_FIFO_READ)
	mpu6050_address_t addr = MPU6050_ADDRESS_AD0_LOW;
	int ret = mpu6050_fifo_init_with_pin(addr, GYROSENSOR_I2C_MTR_SDA, GYROSENSOR_I2C_MTR_SCL);
	if (ret != 0) {
		printf("mpu6050 fifo init failed\r\n");
		return -1;
	} else {
		printf("mpu6050 fifo init success\r\n");
	}
	mpu6050_interface_delay_ms(100);
	mpu6050_fifo_reset2();
	gyroscope_inited = 1;
	return 0;
#elif defined(AI_DEMO_ICM42670P)
	int ret = icm42670p_i2c_init_with_pin(GYROSENSOR_I2C_MTR_SDA, GYROSENSOR_I2C_MTR_SCL);
	if (ret != 0) {
		printf("icm42670p or icm42607p init failed\r\n");
		return -1;
	} else {
		printf("icm42670p or icm42607p init success\r\n");
	}
	gyroscope_inited = 1;
	return 0;
#endif
	return -1;
}


#if defined(AI_DEMO_ICM42670P)
#define GYRO_FS 2000 //must sync with inv_imu_set_gyro_fsr() of icm42670p_hal.c
#define GYRO_ODR 800 //must sync with inv_imu_set_gyro_frequency() of icm42670p_hal.c
static float convert_gyro(int16_t raw, uint16_t fs)
{
	return ((float)raw * fs) / (INT16_MAX);
}
static gyro_data_t *_gyro_data = NULL;
static uint16_t _gyro_data_len = 0;
static uint16_t _gyro_data_widx = 0;
static float gyroUpdateInterval = 1000.0 / GYRO_ODR; //milliseconds
static float gyrots = 0.0;
static void event_cb(inv_imu_sensor_event_t *evt)
{
	if (evt && _gyro_data && _gyro_data_widx < _gyro_data_len) {
		_gyro_data[_gyro_data_widx].dps[0] = convert_gyro(evt->gyro[0], GYRO_FS);
		_gyro_data[_gyro_data_widx].dps[1] = convert_gyro(evt->gyro[1], GYRO_FS);
		_gyro_data[_gyro_data_widx].dps[2] = convert_gyro(evt->gyro[2], GYRO_FS);

		//TODO: every gyro data must sync with video's timestamp
		_gyro_data[_gyro_data_widx].timestamp = (uint32_t) gyrots;
		gyrots += gyroUpdateInterval;
		_gyro_data_widx++;
		if (_gyro_data_widx >= _gyro_data_len) {
			printf("icm42670p or icm42607p event_cb: buffer full\r\n");
		}
	}
}
#endif

int gyroscope_fifo_read(gyro_data_t *data, uint16_t len)
{
#if defined(AI_DEMO_MPU6050) && defined(MPU6050_FIFO_READ)
	if (len > 128) {
		return -1;
	}
	static int16_t gs_accel_raw[128][3] = {0};
	static float gs_accel_g[128][3] = {0};
	static int16_t gs_gyro_raw[128][3] = {0};
	static float gs_gyro_dps[128][3] = {0};

	if (data == NULL) {
		return -2;
	}
	if (gyroscope_inited == 0) {
		if (gyroscope_fifo_init() != 0) {
			return -3;
		}
	}
	if (mpu6050_fifo_read(gs_accel_raw, gs_accel_g,
						  gs_gyro_raw, gs_gyro_dps, &len) != 0) {
		printf("mpu6050 fifo read failed\r\n");
		return -4;
	}

	float gyroUpdateInterval = 1000.0 / MPU6050_FIFO_DEFAULT_RATE;
	float timestamp = 0;
	for (int i = 0; i < len ; i++) {
#if !IGN_ACC_DATA
		data[i].g[0] = gs_accel_g[i][0];
		data[i].g[1] = gs_accel_g[i][1];
		data[i].g[2] = gs_accel_g[i][2];
#endif
		//data[i].g_raw[0] = gs_accel_raw[i][0];
		//data[i].g_raw[1] = gs_accel_raw[i][1];
		//data[i].g_raw[2] = gs_accel_raw[i][2];
		data[i].dps[0] = gs_gyro_dps[i][0];
		data[i].dps[1] = gs_gyro_dps[i][1];
		data[i].dps[2] = gs_gyro_dps[i][2];
		//data[i].dps_raw[0] = gs_gyro_raw[i][0];
		//data[i].dps_raw[1] = gs_gyro_raw[i][1];
		//data[i].dps_raw[2] = gs_gyro_raw[i][2];

		//TODO: every gyro data must sync with video's timestamp
		data[i].timestamp = (uint32_t) timestamp;
		timestamp += gyroUpdateInterval;
	}

	return len;
#elif defined(AI_DEMO_ICM42670P)
	_gyro_data = data;
	_gyro_data_len = len;
	_gyro_data_widx = 0;
	gyrots = 0.0;
	int ret = icm42670p_read_fifo(event_cb);
	if (ret >= 0) {
		//printf("icm42670p_read_fifo: %d\r\n", ret);
	} else {
		printf("icm42670p_read_fifo failed: %d\r\n", ret);
	}

	_gyro_data = NULL;
	_gyro_data_len = 0;
	_gyro_data_widx = 0;
	gyrots = 0.0;
	return ret;
#endif
	return -5;
}

int gyroscope_reset_fifo(void)
{
#if defined(AI_DEMO_MPU6050) && defined(MPU6050_FIFO_READ)
	return mpu6050_fifo_reset2();
#elif defined(AI_DEMO_ICM42670P)
	return icm42670p_reset_fifo();
#endif
	return -1;
}
