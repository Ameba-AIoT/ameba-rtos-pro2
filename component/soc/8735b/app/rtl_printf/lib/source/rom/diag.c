/**************************************************************************//**
 * @file     diag.c
 * @brief    This file just declare the variable for debug message on/off control.
 *
 * @version  V1.00
 * @date     2016-05-31
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2023 Realtek Corporation. All rights reserved.
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
#include "diag.h"

#define SECTION_DIAG_BSS            SECTION(".diag.bss")

#if defined(ROM_REGION)
SECTION_DIAG_BSS uint32_t ConfigDebugErr;
SECTION_DIAG_BSS uint32_t ConfigDebugInfo;
SECTION_DIAG_BSS uint32_t ConfigDebugWarn;
#endif  // end of "#if defined(ROM_REGION)"

