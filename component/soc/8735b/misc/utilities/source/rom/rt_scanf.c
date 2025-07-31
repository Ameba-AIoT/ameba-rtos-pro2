/**************************************************************************//**
 * @file     memmove.c.c
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
#include <stdarg.h>
#include <stddef.h>             /* Compiler defns such as size_t, NULL etc. */
#include "strproc.h"
#include "__limits.h"
#include "arith.h"
#include "section_config.h"
#include "rtl_ctype.h"

#if !defined(CONFIG_BUILD_SECURE)

INFRA_ROM_TEXT_SECTION
char *skip_spaces(const char *str)
{
	while (isspace(*str)) {
		++str;
	}
	return (char *)str;
}

INFRA_ROM_TEXT_SECTION
int skip_atoi(const char **s)
{
	int i = 0;

	while (isdigit(**s)) {
		i = i * 10 + *((*s)++) - '0';
	}

	return i;
}

INFRA_ROM_TEXT_SECTION
const char *_parse_integer_fixup_radix(const char *s, unsigned int *base)
{
	if (*base == 0) {
		if (s[0] == '0') {
			if (tolower(s[1]) == 'x' && isxdigit(s[2])) {
				*base = 16;
			} else {
				*base = 8;
			}
		} else {
			*base = 10;
		}
	}

	if (*base == 16 && s[0] == '0' && tolower(s[1]) == 'x') {
		s += 2;
	}
	return s;
}


INFRA_ROM_TEXT_SECTION
int _vsscanf(const char *buf, const char *fmt, va_list args)
{
	const char *str = buf;
	char *next;
	char digit;
	int num = 0;
	char qualifier;
	unsigned int base;
	union {
		long long s;
		unsigned long long u;
	} val;
	s16 field_width;
	bool is_sign;

	while (*fmt) {
		/* skip any white space in format */
		/* white space in format matchs any amount of
		    * white space, including none, in the input.
		    */
		if (isspace(*fmt)) {
			fmt = skip_spaces(++fmt);
			str = skip_spaces(str);
		}

		/* anything that is not a conversion must match exactly */
		if (*fmt != '%' && *fmt) {
			if (*fmt++ != *str++) {
				break;
			}
			continue;
		}

		if (!*fmt) {
			break;
		}
		++fmt;

		/* skip this conversion.
		    * advance both strings to next white space
		    */
		if (*fmt == '*') {
			if (!*str) {
				break;
			}
			while (!isspace(*fmt) && *fmt != '%' && *fmt) {
				fmt++;
			}
			while (!isspace(*str) && *str) {
				str++;
			}
			continue;
		}

		/* get field width */
		field_width = -1;
		if (isdigit(*fmt)) {

			field_width = skip_atoi(&fmt);
			if (field_width <= 0) {
				break;
			}
		}

		/* get conversion qualifier */
		qualifier = (char) -1;
		if (*fmt == 'h' || tolower(*fmt) == 'l' ||
			tolower(*fmt) == 'z') {
			qualifier = *fmt++;

			if (qualifier == *fmt) {
				if (qualifier == 'h') {
					qualifier = 'H';
					fmt++;
				} else if (qualifier == 'l') {
					qualifier = 'L';
					fmt++;
				}
			}
		}

		if (!*fmt) {
			break;
		}

		if (*fmt == 'n') {
			/* return number of characters read so far */
			*va_arg(args, int *) = str - buf;
			++fmt;
			continue;
		}

		if (!*str) {
			break;
		}

		base = 10;
		is_sign = 0;

		switch (*fmt++) {
		case 'c': {
			char *s = (char *)va_arg(args, char *);
			if (field_width == -1) {
				field_width = 1;
			}
			do {
				*s++ = *str++;
			} while (--field_width > 0 && *str);
			num++;
		}
		continue;
		case 's': {
			char *s = (char *)va_arg(args, char *);
			if (field_width == -1) {
				field_width = SHRT_MAX;
			}
			/* first, skip leading white space in buffer */
			str = skip_spaces(str);

			/* now copy until next white space */
			while (*str && !isspace(*str) && field_width--) {
				*s++ = *str++;
			}
			*s = '\0';
			num++;
		}
		continue;
		case 'o':
			base = 8;
			break;
		case 'x':
		case 'X':
			base = 16;
			break;
		case 'i':
			base = 0;
		case 'd':
			is_sign = 1;
		case 'u':
			break;
		case '%':
			/* looking for '%' in str */
			if (*str++ != '%') {
				return num;
			}
			continue;
		default:
			/* invalid format; stop here */
			return num;
		}

		/* have some sort of integer conversion.
		    * first, skip white space in buffer.
		*/
		str = skip_spaces(str);

		digit = *str;
		if (is_sign && digit == '-') {
			digit = *(str + 1);
		}

		if (!digit
			|| (base == 16 && !isxdigit(digit))
			|| (base == 10 && !isdigit(digit))
			|| (base == 8 && (!isdigit(digit) || digit > '7'))
			|| (base == 0 && !isdigit(digit))) {
			break;
		}

		if (is_sign) {
			val.s = qualifier != 'L' ?
					_strtol(str, &next, base) :
					_strtoll(str, &next, base);
		} else {
			val.u = qualifier != 'L' ?
					_strtoul(str, &next, base) :
					_strtoull(str, &next, base);
		}

		if (field_width > 0 && next - str > field_width) {
			if (base == 0) {
				_parse_integer_fixup_radix(str, &base);
			}

			while (next - str > field_width) {
				if (is_sign) {
					val.s = _div_s64(val.s, base);
				} else {
					val.u = _div_u64(val.u, base);
				}
				--next;
			}
		}

		switch (qualifier) {
		case 'H':       /* that's 'hh' in format */
			if (is_sign) {
				*va_arg(args, signed char *) = val.s;
			} else {
				*va_arg(args, unsigned char *) = val.u;
			}
			break;
		case 'h':
			if (is_sign) {
				*va_arg(args, short *) = val.s;
			} else {
				*va_arg(args, unsigned short *) = val.u;
			}
			break;
		case 'l':
			if (is_sign) {
				*va_arg(args, long *) = val.s;
			} else {
				*va_arg(args, unsigned long *) = val.u;
			}
			break;
		case 'L':
			if (is_sign) {
				*va_arg(args, long long *) = val.s;
			} else {
				*va_arg(args, unsigned long long *) = val.u;
			}
			break;
		case 'Z':
		case 'z':
			*va_arg(args, size_t *) = val.u;
			break;
		default:
			if (is_sign) {
				*va_arg(args, int *) = val.s;
			} else {
				*va_arg(args, unsigned int *) = val.u;
			}
			break;
		}
		num++;

		if (!next) {
			break;
		}
		str = next;
	}

	return num;
}

/**
* sscanf - Unformat a buffer into a list of arguments
* @buf:        input buffer
* @fmt:        formatting of buffer
* @...:        resulting arguments
*/
int _sscanf(const char *buf, const char *fmt, ...)
{
	va_list args;
	int i;

	va_start(args, fmt);
	i = _vsscanf(buf, fmt, args);
	va_end(args);

	return i;
}

#endif  // end of "#if !defined(CONFIG_BUILD_SECURE)"

