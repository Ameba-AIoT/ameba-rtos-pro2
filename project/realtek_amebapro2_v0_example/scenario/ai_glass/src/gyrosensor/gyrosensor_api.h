#ifndef __GYROSENSOR_API_H__
#define __GYROSENSOR_API_H__
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
#include "i2c_api.h"
#include "ex_api.h"

#undef AI_DEMO_MPU6050
#define AI_DEMO_ICM42670P

// ignore the accelerometer data from g-sensor
#define IGN_ACC_DATA    1

// Whole g-sensor data
// we do not need raw data of the g-sensor
//typedef struct gyro_data_s {
//    uint32_t timestamp;
//	float dps[3];       // Gyroscope
//	float g[3];         // Accelerometer
//	int16_t dps_raw[3];
//	int16_t g_raw[3];
//} gyro_data_t;

#if IGN_ACC_DATA
typedef struct gyro_data_s {
	uint32_t timestamp;
	float dps[3];       // Gyroscope
} gyro_data_t;
#else
typedef struct gyro_data_s {
	uint32_t timestamp;
	float dps[3];       // Gyroscope
	float g[3];         // Accelerometer
} gyro_data_t;
#endif

int gyroscope_is_inited(void);

int gyroscope_fifo_init(void);

int gyroscope_fifo_read(gyro_data_t *data, uint16_t len);

int gyroscope_reset_fifo(void);

#define GYROSENSOR_I2C_MTR_SDA  PF_2
#define GYROSENSOR_I2C_MTR_SCL  PF_1

#endif //#ifndef __GYROSENSOR_API_H__