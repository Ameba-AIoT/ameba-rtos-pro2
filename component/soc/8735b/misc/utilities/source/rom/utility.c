/**************************************************************************//**
 * @file     utility.c
 * @brief    Implement the misc utility functions.
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
#include "utility.h"
#include "diag.h"

#define SECTION_UTILITY_TEXT            SECTION(".rom.utility.text")
#define SECTION_UTILITY_STUBS           SECTION(".rom.utility.stubs")

#if defined(ROM_REGION)

#if defined ( __CC_ARM ) || (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
// _memset function name conflict with ARMCC Lib. So rename it
#define _memset         __memset
#endif

extern uint32_t ConfigDebugErr;
extern uint32_t ConfigDebugInfo;
extern uint32_t ConfigDebugWarn;

extern int _memcmp(const void *av, const void *bv, size_t len);
extern void *_memcpy(void *s1, const void *s2, size_t n);
extern void *_memmove(void *destaddr, const void *sourceaddr, unsigned length);
extern void *_memset(void *dst0, int val, size_t length);
extern int _memcmp_s(const void *av, const void *bv, size_t len);

void dump_for_one_bytes(u8 *pdata, u32 len);
void dump_for_one_words(u8 *src, u32 len);

SECTION_UTILITY_STUBS
const utility_func_stubs_t utility_stubs = {
	.config_debug_err = &ConfigDebugErr,
	.config_debug_warn = &ConfigDebugWarn,
	.config_debug_info = &ConfigDebugInfo,

	.memcmp = _memcmp,
	.memcpy = _memcpy,
	.memmove = _memmove,
	.memset = _memset,

	.dump_bytes = dump_for_one_bytes,
	.dump_words = dump_for_one_words,
	.memcmp_s = _memcmp_s
};

SECTION_UTILITY_TEXT
void dump_for_one_bytes(u8 *pdata, u32 len)
{
	u8 *pbuf = pdata;
	u32 length = len;
	u32 line_idx = 0;
	u32 byte_idx;
	u32 offset;

	dbg_printf("\r\n [Addr]   .0 .1 .2 .3 .4 .5 .6 .7 .8 .9 .A .B .C .D .E .F\r\n");

	while (line_idx < length) {
		dbg_printf("%08X: ", (pbuf + line_idx));

		if ((line_idx + 16) < length) {
			offset = 16;
		} else {
			offset = length - line_idx;
		}

		for (byte_idx = 0; byte_idx < offset; byte_idx++) {
			dbg_printf("%02x ", pbuf[line_idx + byte_idx]);
		}

		for (byte_idx = 0; byte_idx < (16 - offset); byte_idx++) { //a last line
			dbg_printf("   ");
		}

		dbg_printf("    ");        //between byte and char

		for (byte_idx = 0;  byte_idx < offset; byte_idx++) {
			if (' ' <= pbuf[line_idx + byte_idx]  && pbuf[line_idx + byte_idx] <= '~') {
				dbg_printf("%c", pbuf[line_idx + byte_idx]);
			} else {
				dbg_printf(".");
			}
		}

		dbg_printf("\n\r");
		line_idx += 16;
	}

}

SECTION_UTILITY_TEXT
void dump_for_one_words(u8 *src, u32 len)
{
	u32 i;

	src = (u8 *)(((u32)src) & (~(0x03)));
	dbg_printf("\r\n");
	for (i = 0; i < len; i += 16, src += 16) {
		dbg_printf("%08X:    %08x", src, *(u32 *)(src));
		dbg_printf("    %08x", *(u32 *)(src + 4));
		dbg_printf("    %08x", *(u32 *)(src + 8));
		dbg_printf("    %08x\r\n", *(u32 *)(src + 12));
	}
}


#endif      // end of "#if defined(ROM_REGION)"

