/*
 *******************************************************************************
 * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 */

#if defined(CONFIG_BUILD_SECURE)

#include <cmsis.h>
#include "hal_api.h"

static uint32_t cal_bit_shift(uint32_t Mask)
{
	uint32_t i;
	for (i = 0; i < 31; i++) {
		if (((Mask >> i) & 0x1) == 1) {
			break;
		}
	}
	return (i);
}

static void set_reg_value(uint32_t reg_address, uint32_t Mask, uint32_t val)
{
	uint32_t shift = 0;
	uint32_t data = 0;
	data = HAL_READ32(reg_address, 0);
	shift = cal_bit_shift(Mask);
	data = ((data & (~Mask)) | (val << shift));
	HAL_WRITE32(reg_address, 0, data);
	data = HAL_READ32(reg_address, 0);
}

static void bt_power_on(void)
{
	set_reg_value(0x50000848, BIT14, 1);
	set_reg_value(0x5000092C, BIT28, 1);
	HAL_WRITE32(0x40009830, 0, 0x7);
}

void NS_ENTRY bt_power_on_nsc(void)
{
	bt_power_on();
}

#endif