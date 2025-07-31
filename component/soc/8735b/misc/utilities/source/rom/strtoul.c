/**************************************************************************//**
 * @file     strtoul.c
 * @brief    Implement the string to unsigned long covnersion function.
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
#include "rtl_ctype.h"
#include "section_config.h"
#if defined ( __GNUC__ )
#include <limits.h>
#else
#include "__limits.h"
#endif

/**
 *  @brief To conver a ASCII string to a unsigned long value.
 *
 *  @param nptr The start of the string buffer.
 *  @param endptr A pointer to the end of the parsed string will be placed here
 *  @param base The carry bits of the converted integer, Hex or Dec.
 *
 *  @returns The converted value.
 */
INFRA_ROM_TEXT_SECTION
unsigned long _strtoul(const char *nptr, char **endptr, int base)
{
	register const unsigned char *s = (const unsigned char *)nptr;
	register unsigned long acc;
	register int c;
	register unsigned long cutoff;
	register int neg = 0, any, cutlim;

	/*
	 * See strtol for comments as to the logic used.
	 */
	do {
		c = *s++;
	} while (isspace(c));
	if (c == '-') {
		neg = 1;
		c = *s++;
	} else if (c == '+') {
		c = *s++;
	}
	if ((base == 0 || base == 16) &&
		c == '0' && (*s == 'x' || *s == 'X')) {
		c = s[1];
		s += 2;
		base = 16;
	}
	if (base == 0) {
		base = c == '0' ? 8 : 10;
	}
	cutoff = (unsigned long)ULONG_MAX / (unsigned long)base;
	cutlim = (unsigned long)ULONG_MAX % (unsigned long)base;
	for (acc = 0, any = 0;; c = *s++) {
		if (isdigit(c)) {
			c -= '0';
		} else if (isalpha(c)) {
			c -= isupper(c) ? 'A' - 10 : 'a' - 10;
		} else {
			break;
		}
		if (c >= base) {
			break;
		}
		if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim)) {
			any = -1;
		} else {
			any = 1;
			acc *= base;
			acc += c;
		}
	}
	if (any < 0) {
		acc = ULONG_MAX;
//      rptr->_errno = ERANGE;
	} else if (neg) {
		acc = -acc;
	}
	if (endptr != 0) {
		*endptr = (char *)(any ? (char *)s - 1 : nptr);
	}
	return (acc);
}


