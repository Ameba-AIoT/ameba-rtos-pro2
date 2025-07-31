/**************************************************************************//**
 * @file     log_buf.c
 * @brief    This file implements the log buffer functions. User can put
 *           formated debugging message in the log buffer and print them out
 *           to the UART port when system is idle.
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
#include "cmsis.h"
#include "log_buf.h"
#include "stdio_port.h"
#include "cmsis_os.h"

#define SECTION_LOGBUF_TEXT                 SECTION(".logbuf.text")
#define SECTION_LOGBUF_BSS                  SECTION(".logbuf.bss")

unsigned __xsprintf(printf_putc_t outchar, void *arg, const char *fmt, va_list _args);
unsigned _printf_engine(printf_putc_t putc, void *arg, const char *fmt, va_list args);

extern void *_memset(void *dst0, int Val, SIZE_T length);
extern osMutexId PrintLock_id;

//SECTION_LOGBUF_BSS log_buf_type_t *pglog_buf;

/**
 *  @brief To put a character to the log buffer
 *  @param plog The log buffer entity.
 *  @param ch The character to be put into the log buffer.
 *
 *  @returns void
 */
SECTION_LOGBUF_TEXT
int _log_buf_putc(log_buf_type_t *plog, char ch)
{
	plog->log_buf[plog->buf_w] = ch;
	plog->buf_w++;
	if (plog->buf_w >= plog->buf_sz) {
		plog->buf_w = 0;
	}

	return (int)ch;
}

/**
 *  @brief To print out message in the log buffer to the real debug port.
 *  @param plog The log buffer entity.
 *
 *  @returns void
 */
SECTION_LOGBUF_TEXT
void _log_buf_show(log_buf_type_t *plog)
{
#if defined (CONFIG_VRF_MODE) && (CONFIG_VRF_MODE == 1)
	osMutexWait(PrintLock_id, osWaitForever);
#endif
	uint32_t buf_w = plog->buf_w;

	while (plog->buf_r != buf_w) {
//        plog->port_putc (plog->port_adapter, plog->log_buf[plog->buf_r]);
		_stdio_port_putc(plog->log_buf[plog->buf_r]);
		plog->buf_r++;
		if (plog->buf_r >= plog->buf_sz) {
			plog->buf_r = 0;
		}
	}
#if defined (CONFIG_VRF_MODE) && (CONFIG_VRF_MODE == 1)
	osMutexRelease(PrintLock_id);
#endif
}

/**
 *  @brief To initial a log buffer entity. It will register a function to dump the message in
 *         the log buffer to the physical port, like a UART port.
 *  @param plog The log buffer entity.
 *
 *  @returns void
 */
SECTION_LOGBUF_TEXT
void _log_buf_init(log_buf_type_t *plog)
{
	_memset((void *)plog, 0, sizeof(log_buf_type_t));

//    pglog_buf = plog;
}

/**
 *  @brief To set the start address of the message buffer of a log buffer entity.
 *  @param plog The log buffer entity.
 *  @param pbuf The start address of the mesage buffer.
 *  @param buf_size The size of the log message buffer.
 *
 *  @returns void
 */
SECTION_LOGBUF_TEXT
void _log_buf_set_msg_buf(log_buf_type_t *plog, char *pbuf, uint32_t buf_size)
{
	plog->log_buf = pbuf;
	plog->buf_sz = buf_size;

//    if (plog->port_adapter != NULL) {
	plog->initialed = TRUE;
//    }
}

/**
 *  @brief The function to print a format message to the log buffer.
 *  @param fmt The arguments list, just like the arguments of printf.
 *
 *  @returns The length of the printed string.
 */
SECTION_LOGBUF_TEXT
int _log_buf_printf(log_buf_type_t *pglog, const char *fmt, ...)
{
	int count = 0;
	va_list list;

	if (pglog->initialed) {
		va_start(list, fmt);
#if CONFIG_LIGHT_PRINTF
		count = _printf_engine((printf_putc_t)&_log_buf_putc, (void *)pglog, fmt, list);
#else
		count = __xsprintf((printf_putc_t)&_log_buf_putc, (void *)pglog, fmt, list);
#endif
		va_end(list);
		(void)list;
	}

	return count;
}

