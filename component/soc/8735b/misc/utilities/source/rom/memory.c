/**************************************************************************//**
 * @file     memory.c
 * @brief    Implement the memory operation functions.
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
#include <stddef.h>             /* Compiler defns such as size_t, NULL etc. */
#include "section_config.h"
#include "memory.h"

/* MACROS */

/* Nonzero if either X or Y is not aligned on a word boundary. */
#define CHECK_STR_UNALIGNED(X, Y) \
     (((u32)(X) & (sizeof (u32) - 1)) | \
      ((u32)(Y) & (sizeof (u32) - 1)))

/* How many bytes are copied each iteration of the 4X unrolled loop in the
 * optimised string implementation
 */
#define STR_OPT_BIGBLOCKSIZE     (sizeof(u32) << 2)


/* How many bytes are copied each iteration of the word copy loop in the
 * optimised string implementation
 */
#define STR_OPT_LITTLEBLOCKSIZE (sizeof (u32))

/* EXPORTED SYMBOLS */

INFRA_ROM_TEXT_SECTION
int _memcmp(const void *av, const void *bv, size_t len)
{
	const unsigned char *a = av;
	const unsigned char *b = bv;
	size_t i;

	for (i = 0; i < len; i++) {
		if (a[i] != b[i]) {
			return (int)(a[i] - b[i]);
		}
	}
	return 0;
}

/* FUNCTIONS */
INFRA_ROM_TEXT_SECTION
void *_memcpy(void *s1, const void *s2, size_t n)
{
	char *dst = (char *) s1;
	const char *src = (const char *) s2;

	u32 *aligned_dst;
	const u32 *aligned_src;


	/* If the size is small, or either SRC or DST is unaligned,
	 * then punt into the byte copy loop.  This should be rare.
	 */
	if (n < sizeof(u32) || CHECK_STR_UNALIGNED(src, dst)) {
		while (n--) {
			*dst++ = *src++;
		}

		return s1;
	} /* if */

	aligned_dst = (u32 *)dst;
	aligned_src = (const u32 *)src;

	/* Copy 4X long words at a time if possible.  */
	while (n >= STR_OPT_BIGBLOCKSIZE) {
		*aligned_dst++ = *aligned_src++;
		*aligned_dst++ = *aligned_src++;
		*aligned_dst++ = *aligned_src++;
		*aligned_dst++ = *aligned_src++;
		n -= STR_OPT_BIGBLOCKSIZE;
	} /* while */

	/* Copy one long word at a time if possible.  */
	while (n >= STR_OPT_LITTLEBLOCKSIZE) {
		*aligned_dst++ = *aligned_src++;
		n -= STR_OPT_LITTLEBLOCKSIZE;
	} /* while */

	/* Pick up any residual with a byte copier.  */
	dst = (char *)aligned_dst;
	src = (const char *)aligned_src;
	while (n--) {
		*dst++ = *src++;
	}

	return s1;
} /* _memcpy() */

INFRA_ROM_TEXT_SECTION
void *_memmove(void *destaddr, const void *sourceaddr, unsigned length)
{
	char *dest = destaddr;
	const char *source = sourceaddr;

	if (source < dest) {
		/* Moving from low mem to hi mem; start at end.  */
		for (source += length, dest += length; length; --length) {
			*--dest = *--source;
		}
	} else if (source != dest) {
		/* Moving from hi mem to low mem; start at beginning.  */
		for (; length; --length) {
			*dest++ = *source++;
		}
	}
	return destaddr;
}

#define	wsize	sizeof(u32)
#define	wmask	(wsize - 1)

INFRA_ROM_TEXT_SECTION
void *_memset(void *dst0, int Val,	size_t length)
{
	SIZE_T t;
	u32 Wideval;
	u8 *dst;

	dst = dst0;
	/*
	 * If not enough words, just fill bytes.  A length >= 2 words
	 * guarantees that at least one of them is `complete' after
	 * any necessary alignment.  For instance:
	 *
	 *	|-----------|-----------|-----------|
	 *	|00|01|02|03|04|05|06|07|08|09|0A|00|
	 *	          ^---------------------^
	 *		 dst		 dst+length-1
	 *
	 * but we use a minimum of 3 here since the overhead of the code
	 * to do word writes is substantial.
	 */
	if (length < 3 * wsize) {
		while (length != 0) {
			*dst++ = Val;
			--length;
		}
		return (dst0);
	}

	if ((Wideval = (u32)Val) != 0) {	/* Fill the word. */
		Wideval = ((Wideval << 24) | (Wideval << 16) | (Wideval << 8) | Wideval);	/* u_int is 32 bits. */
	}

	/* Align destination by filling in bytes. */
	if ((t = (u32)dst & wmask) != 0) {
		t = wsize - t;
		length -= t;
		do {
			*dst++ = Val;
		} while (--t != 0);
	}

	/* Fill words.  Length was >= 2*words so we know t >= 1 here. */
	t = length / wsize;
	do {
		*(u32 *)dst = Wideval;
		dst += wsize;
	} while (--t != 0);

	/* Mop up trailing bytes, if any. */
	t = length & wmask;
	if (t != 0)
		do {
			*dst++ = Val;
		} while (--t != 0);

	return (dst0);
}

/* secure version of memory compare */
INFRA_ROM_TEXT_SECTION
int _memcmp_s(const void *av, const void *bv, size_t len)
{
	const uint8_t *a = av;
	const uint8_t *b = bv;
	int ret = 0;
	uint32_t i;

	for (i = 0; i < len; i++) {
		ret |= a[i] - b[i];
	}

	return ret;
}

