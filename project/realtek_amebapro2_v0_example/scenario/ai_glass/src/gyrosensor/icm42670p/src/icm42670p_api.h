#ifndef _ICM42670P_API_H_

#include <stdint.h>

#define AI_DEMO_ICM42670P 1

typedef struct gyro_data_s {
	float g[3];
	float dps[3];
	uint32_t timestamp;
} gyro_data_t;

int gyroscope_fifo_init(void);
int gyroscope_is_inited(void);
int gyroscope_fifo_read(gyro_data_t *data, uint16_t len);
int gyroscope_reset_fifo(void);

#define _ICM42670P_API_H_
#endif