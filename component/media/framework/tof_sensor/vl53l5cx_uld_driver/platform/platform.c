/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "platform.h"
#include <string.h>
#include "PinNames.h"
#include "basic_types.h"
#include "diag.h"
#include "pinmap.h"
#include "amebapro2/amebapro2_i2c_wrapper.h"

uint8_t VL53L5CX_RdByte(
	VL53L5CX_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t *p_value)
{
	tof_i2c_read_single_byte(RegisterAdress);

	return 0;
}

uint8_t VL53L5CX_WrByte(
	VL53L5CX_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t value)
{

	uint8_t status = tof_i2c_write_single_byte(RegisterAdress, value);

	return status;
}

uint8_t VL53L5CX_WrMulti(
	VL53L5CX_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t *p_values,
	uint32_t size)
{

	uint8_t status = tof_i2c_write_multiple_bytes(RegisterAdress, p_values, size);

	return status;
}

uint8_t VL53L5CX_RdMulti(
	VL53L5CX_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t *p_values,
	uint32_t size)
{
	uint8_t status = tof_i2c_read_multiple_bytes(RegisterAdress, p_values, size);

	return status;
}

uint8_t VL53L5CX_Reset_Sensor(
	VL53L5CX_Platform *p_platform)
{
	uint8_t status = 0;

	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */

	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	VL53L5CX_WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	VL53L5CX_WaitMs(p_platform, 100);

	return status;
}

void VL53L5CX_SwapBuffer(
	uint8_t 		*buffer,
	uint16_t 	 	 size)
{
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for (i = 0; i < size; i = i + 4) {
		tmp = (
				  buffer[i] << 24)
			  | (buffer[i + 1] << 16)
			  | (buffer[i + 2] << 8)
			  | (buffer[i + 3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t VL53L5CX_WaitMs(
	VL53L5CX_Platform *p_platform,
	uint32_t TimeMs)
{
	osStatus ret;

	ret = osDelay(TimeMs);
	if ((ret != osEventTimeout) && (ret != osOK)) {
		// printf("\r\n[ERROR] %s. 0x%x \n", __FUNCTION__, ret);
	}

	return 0;
}
