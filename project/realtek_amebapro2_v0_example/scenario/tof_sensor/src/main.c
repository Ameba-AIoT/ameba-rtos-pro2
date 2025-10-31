#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "basic_types.h"
#include "diag.h"
#include "i2c_api.h"
#include "ex_api.h"
#include "freertos_service.h"
#include "FreeRTOS.h"
#include "task.h"
#include "vl53l5cx_api.h"
#include "tof_sens_ctrl_api.h"

#define MBED_I2C_MTR_SDA 		PE_4
#define MBED_I2C_MTR_SCL 		PE_3

#define MBED_I2C_BUS_CLK		400000
#define I2C_DATA_LENGTH			128
#define I2C_DEVICE_ADDRESS		0x29
#define I2C_BUFFER_SIZE			32

tof_sens_ctx_t tof_sens_ctx = {
	.tof_i2c_cfg.bus_clk_hz = MBED_I2C_BUS_CLK,
	.tof_i2c_cfg.i2c_addr = I2C_DEVICE_ADDRESS,
	.tof_i2c_cfg.i2c_data_length = I2C_DATA_LENGTH,
	.tof_i2c_cfg.i2c_buffer_size = I2C_BUFFER_SIZE,
	.tof_i2c_cfg.i2c_max_packet_size = I2C_DATA_LENGTH,
	.tof_i2c_cfg.sda = MBED_I2C_MTR_SDA,
	.tof_i2c_cfg.scl = MBED_I2C_MTR_SCL,
	.image_res = TOF_SENS_RESOLUTION_8X8,
};

void tof_sens_task(void *param)
{
	bool status;
	int image_width = 0;
	int image_height = 0;

	printf("Initializing sensor board. This can take up to 10s. Please wait.\r\n");
	if (tof_sens_init(&tof_sens_ctx, &tof_sens_ctx.vl53l5cx_sensor) == false) {
		printf("Sensor not found - check your wiring. Freezing\r\n");
		while (1)
			;
	}
	vTaskDelay(100);

	image_width = sqrt(tof_sens_ctx.image_res);
	image_height = image_width;

	status = tof_sens_start_ranging(&tof_sens_ctx.vl53l5cx_sensor);
	if (!status) {
		printf("Start ranging failed\r\n");
		goto tof_sens_fail;
	}
	printf("Start ranging\r\n");

	while (1) {
		// Poll sensor for new data
		int data_is_ready = 0;
		data_is_ready = tof_sens_data_ready(&tof_sens_ctx.vl53l5cx_sensor);
		if (data_is_ready) {
			if (tof_sens_get_ranging_data(&tof_sens_ctx.vl53l5cx_sensor, &tof_sens_ctx.vl53l5cx_data)) { // Read distance data into array
				printf("ToF ranging data: \r\n");
				for (int y = 0; y < image_height; y++) {
					for (int x = image_width - 1; x >= 0; x--) {
						printf("\t%d", tof_sens_ctx.vl53l5cx_data.distance_mm[y * image_width + x]);
					}
					printf("\n");
				}
				printf("\n");
			}
		}
		vTaskDelay(5);
	}
	vTaskDelete(NULL);

tof_sens_fail:
	return;
}

void main(void)
{
	if (xTaskCreate(tof_sens_task, "ToF_sensor_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate failed", __FUNCTION__);
	}
	vTaskStartScheduler();

	while (1);
}