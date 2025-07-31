/**************************************************************************//**
 * @file     strtod.c
 * @brief    Implement the string to a double covnersion function.
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
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include "rtl_ctype.h"

INFRA_ROM_TEXT_SECTION
double _strtod(const char *str, char **endptr)
{
	double number;
	int exponent;
	int negative;
	char *p = (char *) str;
	double p10;
	int n;
	int num_digits;
	int num_decimals;

	// Skip leading whitespace
	while (isspace(*p)) {
		p++;
	}

	// Handle optional sign
	negative = 0;
	switch (*p) {
	case '-':
		negative = 1; // Fall through to increment position
	case '+':
		p++;
	}

	number = 0.;
	exponent = 0;
	num_digits = 0;
	num_decimals = 0;

	// Process string of digits
	while (isdigit(*p)) {
		number = number * 10. + (*p - '0');
		p++;
		num_digits++;
	}

	// Process decimal part
	if (*p == '.') {
		p++;

		while (isdigit(*p)) {
			number = number * 10. + (*p - '0');
			p++;
			num_digits++;
			num_decimals++;
		}

		exponent -= num_decimals;
	}

	if (num_digits == 0) {
		//    errno = ERANGE;
		return 0.0;
	}

	// Correct for sign
	if (negative) {
		number = -number;
	}

	// Process an exponent string
	if (*p == 'e' || *p == 'E') {
		// Handle optional sign
		negative = 0;
		switch (*++p) {
		case '-':
			negative = 1;   // Fall through to increment pos
		case '+':
			p++;
		}

		// Process string of digits
		n = 0;
		while (isdigit(*p)) {
			n = n * 10 + (*p - '0');
			p++;
		}

		if (negative) {
			exponent -= n;
		} else {
			exponent += n;
		}
	}

	if (exponent < DBL_MIN_EXP  || exponent > DBL_MAX_EXP) {
		//    errno = ERANGE;
		return HUGE_VAL;
	}

	// Scale the result
	p10 = 10.;
	n = exponent;
	if (n < 0) {
		n = -n;
	}

	while (n) {
		if (n & 1) {
			if (exponent < 0) {
				number /= p10;
			} else {
				number *= p10;
			}
		}
		n >>= 1;
		p10 *= p10;
	}

	//  if (number == HUGE_VAL) errno = ERANGE;
	if (endptr) {
		*endptr = p;
	}

	return number;
}

INFRA_ROM_TEXT_SECTION
float _strtof(const char *str, char **endptr)
{
	return (float) _strtod(str, endptr);
}

INFRA_ROM_TEXT_SECTION
long double _strtold(const char *str, char **endptr)
{
	return _strtod(str, endptr);
}

INFRA_ROM_TEXT_SECTION
double _atof(const char *str)
{
	return _strtod(str, NULL);
}


