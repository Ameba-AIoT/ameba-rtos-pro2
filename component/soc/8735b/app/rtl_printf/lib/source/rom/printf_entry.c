/**************************************************************************//**
 * @file     printf_entry.c
 * @brief    This file defines the stdio port and printf API entry functions table.
 *
 * @version  V1.00
 * @date     2016-09-28
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
#include "printf_entry.h"

#define SECTION_PRINTF_STUBS            SECTION(".rom.printf.stubs")

extern log_buf_type_t *pglog_buf;

extern int _rtl_printf(const char *fmt, ...);
extern int _rtl_sprintf(char *buf, const char *fmt, ...);
extern int _rtl_snprintf(char *buf, size_t size, const char *fmt, ...);

extern int _xprintf(const char *fmt, ...);
extern int _xsprintf(char *buf, const char *fmt, ...);
extern int _xsnprintf(char *buf, size_t size, const char *fmt, ...);
extern unsigned __xsprintf(printf_putc_t outchar, void *arg, const char *fmt, va_list _args);
extern unsigned _printf_engine(printf_putc_t putc, void *arg, const char *fmt, va_list args);

extern int _sscanf(const char *buf, const char *fmt, ...);

SECTION_PRINTF_STUBS const stdio_printf_func_stubs_t stdio_printf_stubs = {
	.stdio_port_init = _stdio_port_init,
	.stdio_port_deinit = _stdio_port_deinit,
	.stdio_port_putc = _stdio_port_putc,
	.stdio_port_sputc = _stdio_port_sputc,
	.stdio_port_bufputc = _stdio_port_bufputc,
	.stdio_port_getc = _stdio_port_getc,

	.printf_corel = _printf_engine,

	.rt_printfl = _rtl_printf,
	.rt_sprintfl = _rtl_sprintf,
	.rt_snprintfl = _rtl_snprintf,

#if !defined(CONFIG_BUILD_SECURE)
	.printf_core = __xsprintf,

	.rt_printf = _xprintf,
	.rt_sprintf = _xsprintf,
	.rt_snprintf = _xsnprintf,

	.log_buf_init = _log_buf_init,
	.log_buf_putc = _log_buf_putc,
	.log_buf_set_msg_buf = _log_buf_set_msg_buf,
	.log_buf_show = _log_buf_show,
	.log_buf_printf = _log_buf_printf,

	.rt_sscanf = _sscanf
#endif
};

