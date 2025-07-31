/**************************************************************************//**
 * @file     arith.c
 * @brief    Implement the memory move function.
 * @version  V1.00
 * @date     2016-05-30
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

#include "basic_types.h"
#include "section_config.h"

/**
 * div_u64_rem - unsigned 64bit divide with 32bit divisor with remainder
 *
 * This is commonly provided by 32bit archs to provide an optimized 64bit
 * divide.
 */
INFRA_ROM_TEXT_SECTION
u64 _div_u64_rem(u64 dividend, u64 divisor, u64 *remainder)
{
	*remainder = dividend % divisor;
	return dividend / divisor;
}

/**
 * div_s64_rem - signed 64bit divide with 32bit divisor with remainder
 */
INFRA_ROM_TEXT_SECTION
s64 _div_s64_rem(s64 dividend, s64 divisor, s64 *remainder)
{
	*remainder = dividend % divisor;
	return dividend / divisor;
}

/**
 * div_u64 - unsigned 64bit divide with 32bit divisor
 *
 * This is the most common 64bit divide and should be used if possible,
 * as many 32bit archs can optimize this variant better than a full 64bit
 * divide.
 */
INFRA_ROM_TEXT_SECTION
u64 _div_u64(u64 dividend, u64 divisor)
{
	u64 remainder;
	return _div_u64_rem(dividend, divisor, &remainder);
}

/**
 * div_s64 - signed 64bit divide with 32bit divisor
 */
INFRA_ROM_TEXT_SECTION
s64 _div_s64(s64 dividend, s64 divisor)
{
	s64 remainder;
	return _div_s64_rem(dividend, divisor, &remainder);
}


INFRA_ROM_TEXT_SECTION
u64 _mul_u64(u64 mulr, u64 mulp)
{
	return (mulr * mulp);
}

INFRA_ROM_TEXT_SECTION
s64 _mul_s64(s64 mulr, s64 mulp)
{
	return (mulr * mulp);
}

INFRA_ROM_TEXT_SECTION
float _div_float(float dividend, float divisor)
{
	return dividend / divisor;
}

INFRA_ROM_TEXT_SECTION
double _div_double(double dividend, double divisor)
{
	return dividend / divisor;
}

INFRA_ROM_TEXT_SECTION
float _mul_float(float mulr, float mulp)
{
	return (mulr * mulp);
}

INFRA_ROM_TEXT_SECTION
double _mul_double(double mulr, double mulp)
{
	return (mulr * mulp);
}

INFRA_ROM_TEXT_SECTION
float _add_float(float a, float b)
{
	return (a + b);
}

INFRA_ROM_TEXT_SECTION
double _add_double(double a, double b)
{
	return (a + b);
}

INFRA_ROM_TEXT_SECTION
float _sub_float(float a, float b)
{
	return (a - b);
}

INFRA_ROM_TEXT_SECTION
double _sub_double(double a, double b)
{
	return (a - b);
}

