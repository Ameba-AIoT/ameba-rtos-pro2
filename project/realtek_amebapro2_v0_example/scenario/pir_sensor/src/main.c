/**
 * I2C driver implementation for the target platform, based on the Nicera D4-469-AA specification.
 */

#include <string.h>
#include <stdio.h>
#include "platform_opts.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "i2c_api.h"
#include "ex_api.h"
#include "i2c_ex_api.h"

// --- Sensor Constant Definitions ---

// According to the D4-469-AA specification, this is a 10-bit I2C slave address.
// The example address 0xAA (hexadecimal) for D4-469-AA-1 is used here.
// Your i2c_read function needs to support the 10-bit address format.
// Please check the stamping on top of the sensor for the actual address.
//#define PIR_I2C_SLAVE_ADDR      0xAA
#define PIR_I2C_SLAVE_ADDR      0x301

// According to section 5.6 of the specification, a stabilization time of up to 30 seconds is required after power-on.
#define PIR_STABILITY_TIME_MS   30000

// According to section 14.4 of the specification, a read interval of > 10ms is recommended. It is set to 10ms here.
#define PIR_READ_INTERVAL_MS    10

#define MBED_I2C_MTR_SDA              PE_4
#define MBED_I2C_MTR_SCL              PE_3

// --- I2C Platform-related Variables and Callback Functions ---

static i2c_t   i2c_pir;
static xSemaphoreHandle i2c_pir_rx_done_sema = NULL;

static void i2c_pir_rxc_callback(void *userdata)
{
	xSemaphoreGiveFromISR(i2c_pir_rx_done_sema, NULL);
}

extern void i2c_set_user_callback(i2c_t *obj, I2CCallback i2ccb, void(*i2c_callback)(void *));

// --- Driver Implementation ---

// overwirte i2c default config, modify to i2c 10bit address
void i2c_user_config(i2c_t *i2c_config) {
	i2c_config->i2c_adp.init_dat.addr_mod = I2CAddress10bit;
}

int pir_sensor_init(void)
{
	printf("PIR Sensor: Initializing...\r\n");

	i2c_pir_rx_done_sema = xSemaphoreCreateBinary();

	if (!i2c_pir_rx_done_sema) {
		printf("PIR Sensor: Failed to create semaphores.\r\n");
		return -1;
	}

	memset(&i2c_pir, 0, sizeof(i2c_t));
	i2c_init(&i2c_pir, MBED_I2C_MTR_SDA, MBED_I2C_MTR_SCL);
	i2c_frequency(&i2c_pir, 100000); // 100kHz (Standard-mode)

	i2c_set_user_callback(&i2c_pir, I2C_RX_COMPLETE, i2c_pir_rxc_callback);

	printf("PIR Sensor: Waiting for stability (%lu ms)...\r\n", (unsigned long)PIR_STABILITY_TIME_MS);
	vTaskDelay(pdMS_TO_TICKS(PIR_STABILITY_TIME_MS));
	printf("PIR Sensor: Initialization complete.\r\n");

	return 0;
}

int16_t pir_sensor_read_peak_value(void)
{
	uint8_t i2c_data[2];

	i2c_read(&i2c_pir, PIR_I2C_SLAVE_ADDR, (char *)i2c_data, 2, 1);

	// Wait for I2C read to complete
	if (xSemaphoreTake(i2c_pir_rx_done_sema, pdMS_TO_TICKS(500)) != pdTRUE) {
		printf("PIR Sensor: I2C read timeout!\r\n");
		return -1;
	}

	int16_t peak_value = ((i2c_data[0] & 0x0F) << 8) | i2c_data[1];
	if (peak_value & 0x0800) {
		peak_value |= 0xF000;
	}
	return peak_value;
}

void pir_sensor_task(void *params)
{
	// Initialize the sensor
	if (pir_sensor_init() != 0) {
		printf("PIR Sensor: Initialization failed. Task aborting.\r\n");
		vTaskDelete(NULL);
		return;
	}

	printf("PIR Sensor: Starting continuous reading...\r\n");

	// Infinite loop to continuously read values
	for (;;) {
		int16_t value = pir_sensor_read_peak_value();

		// Print the read value to the console
		printf("%d\r\n", value);

		// Delay for a period before the next read, as recommended by the specification
		vTaskDelay(pdMS_TO_TICKS(PIR_READ_INTERVAL_MS));
	}
}

void main(void)
{
	if (xTaskCreate(pir_sensor_task, ((const char *)"pir_example_main"), 8192, NULL, tskIDLE_PRIORITY + 3, NULL) != pdPASS) {
		printf("pir_sensor_task xTaskCreate failed\r\n");
	}
	vTaskStartScheduler();

	while (1);
}