#include <stdio.h>
#include <arm_math.h> //for INT16_MAX and PI
#include "icm42670p_api.h"
#include "icm42670p_hal.h"

static int gyroscope_inited = 0;

int gyroscope_fifo_init(void)
{
#if defined(AI_DEMO_ICM42670P)
	int ret = icm42670p_i2c_init();
	if (ret != 0) {
		printf("icm42670p or icm42607p init failed\r\n");
		return -1;
	}
	gyroscope_inited = 1;
	return 0;
#endif
	return -1;
}

int gyroscope_is_inited(void)
{
	return gyroscope_inited;
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
	if (data == NULL || len == 0) {
		printf("gyroscope_fifo_read: invalid parameter\r\n");
		return -1;
	}

#if defined(AI_DEMO_ICM42670P)
	//TBD TODO: It's not thread safe!
	_gyro_data = data;
	_gyro_data_len = len;
	_gyro_data_widx = 0;
	gyrots = 0.0;
	int ret = icm42670p_read_fifo(event_cb);
	if (ret > 0) {
		printf("icm42670p or icm42607p read_fifo: %d\r\n", ret);
	}

	_gyro_data = NULL;
	_gyro_data_len = 0;
	_gyro_data_widx = 0;
	gyrots = 0.0;
	return ret;
#endif
	return 0;
}

int gyroscope_reset_fifo(void)
{
#if defined(AI_DEMO_ICM42670P)
	return icm42670p_reset_fifo();
#endif
}
