#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "PinNames.h"
#include "basic_types.h"
#include "diag.h"
#include "i2c_api.h"
#include "pinmap.h"
#include "ex_api.h"
#include "freertos_service.h"
#include "FreeRTOS.h"
#include "task.h"
#include "vl53l5cx_api.h"
#include "vl53l5cx_rtl8735b_i2c.h"

VL53L5CX_Configuration			Dev;		/* Sensor configuration */
static VL53L5CX_ResultsData 	data;		/* Results data from VL53L5CX */

bool vl53l5cx_begin(void) {
	int ready;

	uint8_t result = 0;
	uint8_t deviceId = 0;
	uint8_t revisionId = 0;
	
	begin_transmission(I2C_DEVICE_ADDRESS);
	
	if (end_transmission() != 0) {
		ready = false;
	} else {
		ready = true;
	}

	if (!ready) {
		return false;
	}

	write_single_byte(0x7fff, 0x00);
	deviceId = read_single_byte(0x00);
	revisionId = read_single_byte(0x01);
	write_single_byte(0x7fff, 0x02);
	//printf("Read Device ID: 0x%02X (expected 0x%02X)\n", deviceId, DEVICE_ID);
	//printf("Read Revision ID: 0x%02X (expected 0x%02X)\n", revisionId, REVISION_ID);

	if ((revisionId != REVISION_ID) && (deviceId != DEVICE_ID)) {
		printf("revisionId != REVISION_ID/ deviceId != DEVICE_ID \r\n");
		return false;
	}
	result = vl53l5cx_init(&Dev);

	if (result == 0) {
		return true;
	}

	return false;
}

bool set_resolution(uint8_t resolution)
{
	uint8_t result = vl53l5cx_set_resolution(&Dev, resolution);

	if (result == 0) {
		printf("Resolution set\r\n");
		return true;
	} else {
		printf("Resolution not set\r\n");
	}
	return false;
}
uint8_t get_resolution()
{
	uint8_t resolution = 0;
	uint8_t result = vl53l5cx_get_resolution(&Dev, &resolution);
	if (result == 0) {
		if (resolution == 64) {
			return (uint8_t)64;
		} else {
			return (uint8_t)16;
		}
	}
	return (uint8_t)-1;
}

bool start_ranging()
{
	uint8_t result = vl53l5cx_start_ranging(&Dev);

	if (result == 0) {
		return true;
	}

	return false;
}

bool is_data_ready()
{
	uint8_t dataReady = 0;

	uint8_t result = vl53l5cx_check_data_ready(&Dev, &dataReady);
	if (result == 0) {
		return dataReady != 0;
	}

	return false;
}

bool get_ranging_data(VL53L5CX_ResultsData *pRangingData)
{
	uint8_t result = vl53l5cx_get_ranging_data(&Dev, pRangingData);
	if (result == 0) {
		return true;
	}

	return false;
}

void vl53l5cx_task(void *param)
{
	printf("Initializing sensor board. This can take up to 10s. Please wait.\r\n");
	if (vl53l5cx_begin() == false) {
		printf("Sensor not found - check your wiring. Freezing\r\n");
		while (1)
			;
	}

	vTaskDelay(100);

	set_resolution(8 * 8);
	int image_resolution = get_resolution();
	int image_width = sqrt(image_resolution);

	start_ranging();

	while (1) {
		// Poll sensor for new data
		if (is_data_ready() == true) {
			if (get_ranging_data(&data)) // Read distance data into array
			{
				// Pretty-print data with increasing y, decreasing x
				printf("TOF ranging data: \r\n");
				for (int y = 0; y <= image_width * (image_width - 1); y += image_width) {
					for (int x = image_width - 1; x >= 0; x--) {
						printf("\t%d", data.distance_mm[x + y]);
					}
					printf("\n");
				}
				printf("\n");
			}
		}
		vTaskDelay(5);
	}
	vTaskDelete(NULL);
	}

void main(void)
{
 	i2c_init(&i2cmaster, MBED_I2C_MTR_SDA, MBED_I2C_MTR_SCL);
	i2c_frequency(&i2cmaster, MBED_I2C_BUS_CLK);

	if (xTaskCreate(vl53l5cx_task, "vl53l5cx_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate failed", __FUNCTION__);
	}
	vTaskStartScheduler();	

	while (1);
}
