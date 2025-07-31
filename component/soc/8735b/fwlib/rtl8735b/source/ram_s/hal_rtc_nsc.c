/**************************************************************************//**
  * @file     hal_rtc_nsc.c
  * @brief    Execute flash commands in non-secure mode through non-secure callable functions.
  * @version  1.00
  * @date     2021-2-4
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
#include "hal_rtc.h"
#include "hal_rtc_nsc.h"

/// @cond DOXYGEN_ROM_HAL_API
#if !defined(CONFIG_BUILD_SECURE) && !defined(CONFIG_BUILD_NONSECURE)
#undef SECTION_NS_ENTRY_FUNC
#undef NS_ENTRY
#define SECTION_NS_ENTRY_FUNC
#define NS_ENTRY
#endif



#if defined(CONFIG_BUILD_SECURE)

/**
 *  @brief To initial a RTC devices adapter.
 *
 *  @param[in] ppwm_adp The RTC devices adapter.
 *
 *  @returns HAL_OK: Initial succeed.
 */
SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_rtc_init_nsc(hal_rtc_adapter_t *prtc_adp)
{

	return hal_rtc_init(prtc_adp);
}
SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_rtc_deinit_nsc(hal_rtc_adapter_t *prtc_adp)
{

	return hal_rtc_deinit(prtc_adp);
}
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_rtc_enable_nsc(hal_rtc_adapter_t *prtc_adp)
{

	hal_rtc_enable(prtc_adp);
}
SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_rtc_disable_nsc(hal_rtc_adapter_t *prtc_adp)
{

	hal_rtc_disable(prtc_adp);
}

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_rtc_set_time_nsc(hal_rtc_adapter_t *prtc_adp, hal_tm_reg_t *ptm_reg, u16 leap_year)
{

	return hal_rtc_set_time(prtc_adp, ptm_reg, leap_year);
}
SECTION_NS_ENTRY_FUNC
u32 NS_ENTRY hal_rtc_read_nsc(hal_rtc_adapter_t *prtc_adp, u16 addr)
{

	return  hal_rtc_read(prtc_adp, addr);
}
SECTION_NS_ENTRY_FUNC
u32 NS_ENTRY hal_rtc_read_time_nsc(hal_rtc_adapter_t *prtc_adp)
{

	return  hal_rtc_read_time(prtc_adp);
}

SECTION_NS_ENTRY_FUNC

hal_status_t NS_ENTRY hal_rtc_set_alarm_nsc(hal_rtc_adapter_t *prtc_adp, hal_rtc_alarm_t *prtc_alarm)
{

	return  hal_rtc_set_alarm(prtc_adp, prtc_alarm);
}
SECTION_NS_ENTRY_FUNC
u32 NS_ENTRY hal_rtc_read_alarm_nsc(hal_rtc_adapter_t *prtc_adp)
{

	return  hal_rtc_read_alarm(prtc_adp);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_rtc_reg_alarm_irq_nsc(hal_rtc_adapter_t *prtc_adp, rtc_alarm_callback_t callback, void *arg)
{

	hal_rtc_reg_alarm_irq(prtc_adp, callback, arg);
}

SECTION_NS_ENTRY_FUNC
void NS_ENTRY hal_rtc_unreg_alarm_irq_nsc(hal_rtc_adapter_t *prtc_adp)
{

	hal_rtc_unreg_alarm_irq(prtc_adp);
}

SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_rtc_set_cnt_alarm_nsc(hal_rtc_adapter_t *prtc_adp, u8 cnt_alarm)
{

	return  hal_rtc_set_cnt_alarm(prtc_adp, cnt_alarm);
}
SECTION_NS_ENTRY_FUNC
hal_status_t NS_ENTRY hal_rtc_set_comp_nsc(hal_rtc_adapter_t *prtc_adp, u8 func_en)
{

	return hal_rtc_set_comp(prtc_adp, func_en);
}

SECTION_NS_ENTRY_FUNC
void hal_rtc_set_leap_year_nsc(hal_rtc_adapter_t *prtc_adp)
{

	hal_rtc_set_leap_year(prtc_adp);
}

#endif

