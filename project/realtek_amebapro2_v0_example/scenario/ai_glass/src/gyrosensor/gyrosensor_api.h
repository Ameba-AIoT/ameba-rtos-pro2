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
#define AI_DEMO_MPU6050//AI_DEMO_ICM42670P

// ignore the accelerometer data from g-sensor
#define IGN_ACC_DATA    1

// Definition of gyro save status
#define GYRO_SAVE_IDLE          0x00
#define GYRO_SAVE_START         0x01
#define GYRO_SAVE_STOP          0x02
#define GYRO_SAVE_SET_START     0x10
#define GYRO_SAVE_SET_STOP      0x20

// Configuration
#define ENABLE_GET_GSENSOR_INFO 1

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

typedef struct {
    double rms_error;
    double camera_matrix[3][3];
    double distortion_coeffs[4];
    double radial_distortion_limit;
} FisheyeParams;

typedef struct {
    FisheyeParams fisheye_params;
} LdcParams;

typedef struct {
    int32_t imu_rate_hz;
    bool enable_stabilization;
    float stabilization_alpha;
    float crop_ratio_min;
    float crop_ratio_max;
    float rs_readout_time_ms;
    LdcParams ldc_params;
} CameraConfig;

// 10^6 scaling factor
#define FLOAT_SCALE 1000000

#define DEFAULT_IMU_RATE_HZ          1000
#define DEFAULT_ENABLE_STABILIZATION TRUE
#define DEFAULT_STABILIZATION_ALPHA  0.05f
#define DEFAULT_CROP_RATIO_MIN       0.85f
#define DEFAULT_CROP_RATIO_MAX       0.95f
#define DEFAULT_RS_READOUT_TIME_MS   20.0f

#define DEFAULT_RMS_ERROR            0.6267216266238868
#define DEFAULT_CAMERA_MATRIX_00     1076.5431903029578
#define DEFAULT_CAMERA_MATRIX_01     0.0
#define DEFAULT_CAMERA_MATRIX_02     1063.96279664821
#define DEFAULT_CAMERA_MATRIX_10     0.0
#define DEFAULT_CAMERA_MATRIX_11     1078.2081025787843
#define DEFAULT_CAMERA_MATRIX_12     781.8767747969237
#define DEFAULT_CAMERA_MATRIX_20     0.0
#define DEFAULT_CAMERA_MATRIX_21     0.0
#define DEFAULT_CAMERA_MATRIX_22     1.0

#define DEFAULT_DIST_COEFF_0         0.03418760590556284
#define DEFAULT_DIST_COEFF_1         1.611113613393632
#define DEFAULT_DIST_COEFF_2        -2.3397135097121797
#define DEFAULT_DIST_COEFF_3         0.8642283928538785

#define DEFAULT_RADIAL_DIST_LIMIT    0.0   // or -1.0 to mean "no limit"

extern CameraConfig g_camera_cfg;

int gyroscope_is_inited(void);

int gyroscope_fifo_init(void);

int gyroscope_fifo_read(gyro_data_t *data, uint16_t len);

int gyroscope_reset_fifo(void);

#define GYROSENSOR_I2C_MTR_SDA  PF_2
#define GYROSENSOR_I2C_MTR_SCL  PF_1

#endif //#ifndef __GYROSENSOR_API_H__