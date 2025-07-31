/**************************************************************************//**
 * @file     stdio_port.c
 * @brief    Implement the Standard IO of the UART port. Other module,
 *           like printf, shell command will use these IO function to put
 *           or read to/from UART port.
 * @version  V1.00
 * @date     2016-09-22
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
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

#include "cmsis.h"
#include "stdio_port.h"

#define SECTION_STDIOPORT_TEXT                 SECTION(".stdioport.text")
#define SECTION_STDIOPORT_BSS                  SECTION(".stdioport.bss")
#define SECTION_STDIOPORT_RODATA               SECTION(".stdioport.rodata")

SECTION_STDIOPORT_BSS stdio_port_t _stdio_port;

SECTION_STDIOPORT_TEXT
void _stdio_port_init(void *adapter, stdio_putc_t putc, stdio_getc_t getc)
{
	if (adapter != NULL) {
		_stdio_port.adapter = adapter;
	}

	if (putc != NULL) {
		_stdio_port.putc = putc;
	}

	if (getc != NULL) {
		_stdio_port.getc = getc;
	}
}

SECTION_STDIOPORT_TEXT
void _stdio_port_deinit(void)
{
	_stdio_port.adapter = NULL;
	_stdio_port.putc = NULL;
	_stdio_port.getc = NULL;
}

SECTION_STDIOPORT_TEXT
int _stdio_port_putc(char c)
{
	_stdio_port.putc(_stdio_port.adapter, c);
	return (int)c;
}

SECTION_STDIOPORT_TEXT
int _stdio_port_sputc(void *arg, char c)
{
	_stdio_port.putc(_stdio_port.adapter, c);
	return (int)c;
}

SECTION_STDIOPORT_TEXT
int _stdio_port_bufputc(void *buf, char c)
{
	stdio_buf_t *pstd_buf = buf;

	char **s = (char **) & (pstd_buf->pbuf);
	if (pstd_buf->pbuf_lim != 0) {
		if (*s < pstd_buf->pbuf_lim) {
			*(*s)++ = c;
		} else {
			return -1;
		}
	} else {
		*(*s)++ = c;
	}

	return ((int)c);
}

SECTION_STDIOPORT_TEXT
int _stdio_port_getc(char *data)
{
	return _stdio_port.getc(_stdio_port.adapter, data);
}

