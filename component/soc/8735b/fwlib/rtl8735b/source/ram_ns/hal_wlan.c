/**************************************************************************//**
* @file         hal_wlan.c
* @brief       This file is for WLAN HAL API functions.
*
* @version    V1.00
* @date        2017-02-15
*
* @note
*
******************************************************************************
*
* Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the License); you may
* not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an AS IS BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************/

#include "hal_wlan.h"
#include "rtl8735b.h"
#include "rtl8735b_aon_type.h"
#include "hal.h"


#if CONFIG_WLAN_EN

/* =========================================================================================================================== */
/* ================                                        WLAN_PAGE0                                         ================ */
/* =========================================================================================================================== */


/* =========================================================================================================================== */
/* ================                          Device Specific Peripheral Address Map                           ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_peripheralAddr
  * @{
  */

#define WLAN_PAGE0_BASE             0x40080000UL

/** @} */ /* End of group Device_Peripheral_peripheralAddr */


/* =========================================================================================================================== */
/* ================                                  Peripheral declaration                                   ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_declaration
  * @{
  */

#define AON                         ((AON_TypeDef*)            AON_BASE)

/** @} */ /* End of group Device_Peripheral_declaration */


hal_status_t hal_wlan_pwr_on(void)
{
	AON_TypeDef *aon_wlan = AON;
#if !(CONFIG_FPGA || CONFIG_PXP)
	uint32_t loop_time = 10000;
#endif
	aon_wlan->AON_REG_AON_PWR_CTRL |= AON_BIT_WLON_SPC_EN;

	hal_delay_us(10);   //wait power stable

	aon_wlan->AON_REG_AON_PWR_CTRL |= AON_BIT_WLON_LPC_EN;

	hal_delay_us(10);   //wait power stable

	aon_wlan->AON_REG_AON_ISO_CTRL &= (~AON_BIT_SYS_ISO_WLON);

	aon_wlan->AON_REG_AON_WL_CTRL |= AON_BIT_SYS_WLON_EN;

	aon_wlan->AON_REG_AON_WL_CTRL |= AON_BIT_SYS_WLAXI_CLK_EN;

	aon_wlan->AON_REG_AON_WL_CTRL |= AON_BIT_SYS_WL_AXI_EN;

	aon_wlan->AON_REG_AON_WL_CTRL |= AON_BIT_SYS_WLON_CLK_EN;

	aon_wlan->AON_REG_AON_WL_CTRL |= AON_BIT_SYS_WLAFE_POD125;

	aon_wlan->AON_REG_AON_WL_CTRL |= AON_BIT_SYS_WLAFE_POD33;

	hal_delay_us(100);
	//  For req2act, rdy2act
	aon_wlan->AON_REG_AON_SNF_WAKE_EVENT_MSK0 |= AON_BIT_WLAN_IMR_MSK;

	aon_wlan->AON_REG_AON_SNF_WAKE_EVENT_MSK0 |= AON_BIT_PON_SNFEVT_WLON_PON_MSK;

	// RF1 enable clock source control
	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL1 |= AON_BIT_EN_XTAL_DRV_RF1;
	// RF2 enable clock source control
	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL1 |= AON_BIT_EN_XTAL_DRV_RF2;
	// AFE enable clock source control
	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL1 |= AON_BIT_EN_XTAL_DRV_AFE;
	// DIGI enable clock source control
	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL1 |= AON_BIT_EN_XTAL_DRV_DIGI;
	// LPS enable clock source control
	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL1 |= AON_BIT_EN_XTAL_DRV_LPS;

	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL2 &= (~AON_BIT_XTAL_DRV_RF_LATCH);  // RF Finn tune xtal

	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL2 &= (~AON_MASK_XTAL_VREF_SEL);   // RF Finn tune xtal

	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL2 |= 0x1B;   // RF Finn tune xtal

	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL3 &= (~AON_BIT_EN_XTAL_AAC_GM);   // RDC request, will effect xtal efficiency

	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL2 &= (~(AON_MASK_XTAL_DRV_USB));  // RDC request, spur issue

	aon_wlan->AON_REG_SWR_SRC_CTRL0 |= AON_BIT_SRC_POW_SW;

	aon_wlan->AON_REG_SWR_SRC_CTRL0 |= AON_BIT_FPWM_L1;

#if CONFIG_FPGA || CONFIG_PXP
	hal_delay_ms(2);
#else
	do {
		if ((loop_time--) == 0) {
			return HAL_TIMEOUT;
		}
		hal_delay_us(1);
	} while (!(aon_wlan->AON_REG_SWR_SRC_CTRL1 & AON_BIT_SRC_SSOVER_L));
#endif
	return HAL_OK;

}
#if 1
hal_status_t hal_wlan_pwr_off(void)
{

	AON_TypeDef *aon_wlan = AON;

	// RF1 enable clock source control
	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL1 &= (~AON_BIT_EN_XTAL_DRV_RF1);
	// RF2 enable clock source control
	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL1 &= (~AON_BIT_EN_XTAL_DRV_RF2);
	// AFE enable clock source control
	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL1 &= (~AON_BIT_EN_XTAL_DRV_AFE);
	// DIGI enable clock source control
	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL1 &= (~AON_BIT_EN_XTAL_DRV_DIGI);
	// LPS enable clock source control
	aon_wlan->AON_REG_AON_XTAL_CLK_CTRL1 &= (~AON_BIT_EN_XTAL_DRV_LPS);

	aon_wlan->AON_REG_AON_WL_CTRL &= (~AON_BIT_SYS_WLAFE_POD33);

	aon_wlan->AON_REG_AON_WL_CTRL &= (~AON_BIT_SYS_WLAFE_POD125);

	aon_wlan->AON_REG_AON_WL_CTRL &= (~AON_BIT_SYS_WLON_CLK_EN);

	aon_wlan->AON_REG_AON_WL_CTRL &= (~AON_BIT_SYS_WL_AXI_EN);

	aon_wlan->AON_REG_AON_WL_CTRL &= (~AON_BIT_SYS_WLAXI_CLK_EN);

	aon_wlan->AON_REG_AON_WL_CTRL &= (~AON_BIT_SYS_WLON_EN);

	aon_wlan->AON_REG_AON_ISO_CTRL |= AON_BIT_SYS_ISO_WLON;

	aon_wlan->AON_REG_AON_PWR_CTRL &= (~AON_BIT_WLON_LPC_EN);

	aon_wlan->AON_REG_AON_PWR_CTRL &= (~AON_BIT_WLON_SPC_EN);

	return HAL_OK;

}

#endif
#endif //#if CONFIG_WLAN_EN 

