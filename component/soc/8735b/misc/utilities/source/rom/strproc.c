/**************************************************************************//**
 * @file     strproc.c
 * @brief    Implement the string processing functions.
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
#include "memory.h"
#include "section_config.h"
#include "atoi.h"
#include "strproc.h"
#include "rtl_ctype.h"

#define SECTION_STRPROC_STUBS          SECTION(".rom.strproc.stubs")

SECTION_STRPROC_STUBS const strproc_func_stubs_t strproc_stubs = {
	.strcat     = _strcat,
	.strchr     = _strchr,
	.strcmp     = _strcmp,
	.strncmp    = _strncmp,
	.strnicmp   = _strnicmp,
	.strcpy     = _strcpy,
	.strncpy    = _strncpy,
	.strlcpy    = _strlcpy,
	.strlen     = _strlen,
	.strnlen    = _strnlen,
	.strncat    = _strncat,
	.strpbrk    = _strpbrk,
	.strspn     = _strspn,
	.strstr     = _strstr,
	.strtok     = _strtok,
	.strxfrm    = _strxfrm,
	.strsep     = _strsep,
	.strtod     = _strtod,
	.strtof     = _strtof,
	.strtold    = _strtold,
	.strtol     = _strtol,
	.strtoll    = _strtoll,
	.strtoul    = _strtoul,
	.strtoull   = _strtoull,
	.atoi       = _atoi,
	.atoui      = _atoui,
	.atol       = _atol,
	.atoul      = _atoul,
	.atoull     = _atoull,
	.atof       = _atof
};

static char *___strtok;

// String casecade
char *_strcat(char *dest,  char const *src)
{
	char *tmp = dest;

	while (*dest) {
		dest++;
	}
	while ((*dest++ = *src++) != '\0')
		;

	return tmp;
}

/**
 * strchr - Find the first occurrence of a character in a string
 * @s: The string to be searched
 * @c: The character to search for
 */
char *_strchr(const char *s, int c)
{
	for (; *s != (char) c; ++s)
		if (*s == '\0') {
			return NULL;
		}
	return (char *) s;
}

int _strcmp(char const *cs, char const *ct)
{
	signed char __res;

	while (1) {
		if ((__res = *cs - *ct++) != 0 || !*cs++) {
			break;
		}
	}

	return __res;
}

int _strncmp(char const *cs, char const *ct, size_t count)
{
	signed char __res = 0;

	while (count > 0) {
		if ((__res = *cs - *ct++) != 0 || !*cs++) {
			break;
		}
		count--;
	}

	return __res;
}

int _strnicmp(char const *s1, char const *s2, size_t len)
{
	unsigned char c1 = '\0';
	unsigned char c2 = '\0';

	if (len > 0) {
		do {
			c1 = *s1;
			c2 = *s2;
			s1++;
			s2++;
			if (!c1) {
				break;
			}
			if (!c2) {
				break;
			}
			if (c1 == c2) {
				continue;
			}
			c1 = tolower(c1);
			c2 = tolower(c2);
			if (c1 != c2) {
				break;
			}
		} while (--len);
	}
	return (int)c1 - (int)c2;
}

char *_strcpy(char *dest, char const *src)
{
	char *tmp = dest;

	while ((*dest++ = *src++) != '\0')
		;
	return tmp;
}

char *_strncpy(char *dest, char const *src, size_t count)
{
	char *tmp = dest;

	while (count-- && (*dest++ = *src++) != '\0')
		;

	return tmp;
}


size_t _strlcpy(char *dst, char const *src, size_t s)
{
	size_t i = 0;

	if (!s) {
		return _strlen(src);
	}

	for (i = 0; ((i < s - 1) && src[i]); i++) {
		dst[i] = src[i];
	}

	dst[i] = 0;

	return i + _strlen(src + i);
}

size_t _strlcat(char *dst, char const *src, size_t s)
{
	size_t i;
	size_t j = _strnlen(dst, s);

	if (!s) {
		return j + _strlen(src);
	}

	dst += j;

	for (i = 0; ((i < s - 1) && src[i]); i++) {
		dst[i] = src[i];
	}

	dst[i] = 0;

	return j + i + _strlen(src + i);
}

size_t _strlen(char const *s)
{
	size_t i;

	i = 0;
	while (s[i]) {
		i += 1;
	}

	return i;
}

size_t _strnlen(char const *s, size_t count)
{
	const char *sc;

	for (sc = s; count-- && *sc != '\0'; ++sc)
		;
	return sc - s;
}

char *_strncat(char *dest, char const *src, size_t count)
{
	char *tmp = dest;

	if (count > 0) {
		while (*dest) {
			dest++;
		}
		while ((*dest++ = *src++) != 0) {
			if (--count == 0) {
				*dest = '\0';
				break;
			}
		}
	}

	return tmp;
}

/**
 * strpbrk - Find the first occurrence of a set of characters
 * @cs: The string to be searched
 * @ct: The characters to search for
 */
char *_strpbrk(char const *cs, char const *ct)
{
	const char *sc1;
	const char *sc2;

	for (sc1 = cs; *sc1 != '\0'; ++sc1) {
		for (sc2 = ct; *sc2 != '\0'; ++sc2) {
			if (*sc1 == *sc2) {
				return (char *)sc1;
			}
		}
	}

	return NULL;
}

size_t _strspn(char const *s, char const *accept)
{
	const char *p;
	const char *a;
	size_t count = 0;

	for (p = s; *p != '\0'; ++p) {
		for (a = accept; *a != '\0'; ++a) {
			if (*p == *a) {
				break;
			}
		}
		if (*a == '\0') {
			return count;
		}
		++count;
	}

	return count;
}

char *_strstr(char const *s1, char const *s2)
{
	int l1, l2;

	l2 = _strlen(s2);
	if (!l2) {
		return (char *)s1;
	}
	l1 = _strlen(s1);
	while (l1 >= l2) {
		l1--;
		if (!_memcmp(s1, s2, l2)) {
			return (char *)s1;
		}
		s1++;
	}
	return NULL;
}

char *_strtok(char *s, char const *ct)
{
	char *sbegin, *send;

	sbegin  = s ? s : ___strtok;
	if (!sbegin) {
		return NULL;
	}
	sbegin += _strspn(sbegin, ct);
	if (*sbegin == '\0') {
		___strtok = NULL;
		return (NULL);
	}
	send = _strpbrk(sbegin, ct);
	if (send && *send != '\0') {
		*send++ = '\0';
	}
	___strtok = send;
	return (sbegin);
}

size_t _strxfrm(char *dest, const char *src, size_t n)
{
	size_t len = _strlen(src);

	if (n) {
		size_t copy_len = len < n ? len : n - 1;
		_memcpy(dest, src, copy_len);
		dest[copy_len] = 0;
	}
	return len;
}

/**
 * strsep - Split a string into tokens
 * @s: The string to be searched
 * @ct: The characters to search for
 *
 * strsep() updates @s to point after the token, ready for the next call.
 *
 * It returns empty tokens, too, behaving exactly like the libc function
 * of that name. In fact, it was stolen from glibc2 and de-fancy-fied.
 * Same semantics, slimmer shape. ;)
 */
char *_strsep(char **s, const char *ct)
{
	char *sbegin = *s;
	char *end;

	if (sbegin == NULL) {
		return NULL;
	}

	end = _strpbrk(sbegin, ct);
	if (end) {
		*end++ = '\0';
	}
	*s = end;
	return sbegin;
}

