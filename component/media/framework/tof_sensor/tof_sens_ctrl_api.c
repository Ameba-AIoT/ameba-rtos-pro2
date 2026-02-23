#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "basic_types.h"
#include "diag.h"
#include "vl53l5cx_api.h"
#include "tof_sens_ctrl_api.h"

static bool tof_sens_i2c_init(tof_sens_ctx_t *tof_sens_ctx)
{
	uint8_t deviceId = 0;
	uint8_t revisionId = 0;

	tof_i2c_init(&tof_sens_ctx->tof_i2c_cfg, &tof_sens_ctx->tof_i2c_ctx);

	tof_sens_ctx->tof_i2c_ctx.tx_address = 0;
	tof_sens_ctx->tof_i2c_ctx.tx_buffer_length = 0;

	tof_sens_ctx->tof_i2c_ctx.rx_buffer_index = 0;
	tof_sens_ctx->tof_i2c_ctx.rx_buffer_length = 0;

	tof_i2c_begin_transmission(tof_sens_ctx->tof_i2c_cfg.i2c_addr);

	if (tof_i2c_end_transmission() != 0) {
		printf("tof sensor end transmission failed\r\n");
		return false;
	}

	tof_i2c_write_single_byte(0x7fff, 0x00);
	deviceId = tof_i2c_read_single_byte(0x00);
	revisionId = tof_i2c_read_single_byte(0x01);
	tof_i2c_write_single_byte(0x7fff, 0x02);

	if ((revisionId != REVISION_ID) && (deviceId != DEVICE_ID)) {
		printf("revisionId != REVISION_ID/ deviceId != DEVICE_ID \r\n");
		return false;
	}

	return true;
}

static void tof_sens_i2c_deinit(tof_sens_ctx_t *tof_sens_ctx)
{
	tof_i2c_deinit(&tof_sens_ctx->tof_i2c_cfg);
}

bool tof_sens_init(tof_sens_ctx_t *tof_sens_ctx, VL53L5CX_Configuration *p_dev)
{
	uint8_t status = 0;
	uint8_t set_res_status = 0;
	bool tof_i2c_status;

	tof_i2c_status = tof_sens_i2c_init(tof_sens_ctx);
	if (!tof_i2c_status) {
		printf("i2c init for ToF sensor failed \r\n");
		return false;
	}

	status = vl53l5cx_init(p_dev);

	if (status == 0) {
		if (tof_sens_ctx->image_res != TOF_SENS_RESOLUTION_4X4 && tof_sens_ctx->image_res != TOF_SENS_RESOLUTION_8X8) {
			printf("Invalid resolution set\r\n");
			return false;
		}

		set_res_status = tof_sens_set_resolution(&tof_sens_ctx->vl53l5cx_sensor, tof_sens_ctx->image_res);
		if (!set_res_status) {
			printf("Resolution not set\r\n");
			return false;
		}
		printf("Resolution set\r\n");

		return true;
	}

	return false;
}

bool tof_sens_deinit(tof_sens_ctx_t *tof_sens_ctx, VL53L5CX_Configuration *p_dev)
{
	bool status;

	printf("Deinitializing sensor board. This can take up to 10s. Please wait.\r\n");
	status = tof_sens_stop_ranging(p_dev);
	if (!status) {
		printf("Stop ranging failed.\r\n");
		return false;
	}
	printf("i2c deinitialized\r\n");
	tof_sens_i2c_deinit(tof_sens_ctx);

	return true;
}

bool tof_sens_set_resolution(VL53L5CX_Configuration *p_dev, uint8_t resolution)
{
	uint8_t status = vl53l5cx_set_resolution(p_dev, resolution);

	if (status == 0) {
		return true;
	}
	return false;
}

uint8_t tof_sens_get_resolution(VL53L5CX_Configuration *p_dev, uint8_t *resolution)
{
	uint8_t status = vl53l5cx_get_resolution(p_dev, resolution);
	if (status == 0) {
		if (*resolution == 64) {
			return (uint8_t)64;
		} else {
			return (uint8_t)16;
		}
	}
	return (uint8_t) FAIL; //error
}

bool tof_sens_start_ranging(VL53L5CX_Configuration *p_dev)
{
	uint8_t status = vl53l5cx_start_ranging(p_dev);

	if (status == 0) {
		return true;
	}
	return false;
}

bool tof_sens_stop_ranging(VL53L5CX_Configuration *p_dev)
{
	uint8_t status = vl53l5cx_stop_ranging(p_dev);

	if (status == 0) {
		return true;
	}
	return false;
}

int tof_sens_data_ready(VL53L5CX_Configuration *p_dev)
{
	uint8_t dataReady = 0;
	uint8_t status = vl53l5cx_check_data_ready(p_dev, &dataReady);
	if (status != 0) {
		printf("check_data_ready failed\n");
		return FAIL; // error
	}
	return dataReady;
}

bool tof_sens_get_ranging_data(VL53L5CX_Configuration *p_dev, VL53L5CX_ResultsData *pRangingData)
{
	uint8_t status = vl53l5cx_get_ranging_data(p_dev, pRangingData);
	if (status == 0) {
		return true;
	}

	return false;
}