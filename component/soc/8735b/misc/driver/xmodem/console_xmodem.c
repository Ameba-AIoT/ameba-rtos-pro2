#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xmodem.h"
#include "FreeRTOS.h"
#include "task.h"
#include "vfs.h"

#define XMODEM_STDIN_SIZE 2048
static unsigned char _stdin[XMODEM_STDIN_SIZE];
static volatile int _stdin_wcnt = 0;
static volatile int _stdin_rcnt = 0;

extern void console_stdio_init(void *read_cb, void *write_cb);
extern void remote_stdio_init(void *read_cb, void *write_cb);
extern void console_stdio_get(void **read_cb, void **write_cb);
extern void remote_stdio_get(void **read_cb, void **write_cb);

static unsigned(*c_read)(unsigned fd, void *buf, unsigned len);
static unsigned(*r_read)(unsigned fd, void *buf, unsigned len);
static unsigned(*c_write)(unsigned fd, const void *buf, unsigned len);
static unsigned(*r_write)(unsigned fd, const void *buf, unsigned len);


void xmodem_stdin_handling(unsigned char *buf, int len)
{

	// free buffer < len
	while (_stdin_wcnt - _stdin_rcnt >= XMODEM_STDIN_SIZE - len) {
		vTaskDelay(10);
	}

	int ridx = _stdin_rcnt % XMODEM_STDIN_SIZE;
	int widx = _stdin_wcnt % XMODEM_STDIN_SIZE;

	// here, free space >= len
	int tail_len = XMODEM_STDIN_SIZE - widx;
	if (tail_len > len) {
		memcpy(&_stdin[widx], buf, len);
	} else {
		memcpy(&_stdin[widx], buf, tail_len);
		memcpy(&_stdin[0], &buf[tail_len], len - tail_len);
	}

	_stdin_wcnt += len;
}

#define TIMEOUT_UNIT 50

int xmodem_getc(unsigned short timeout)
{
	int timeout_cnt = timeout / TIMEOUT_UNIT;

	while ((_stdin_rcnt >= _stdin_wcnt) && (timeout_cnt > 0)) {
		// wait timeout then return eof
		vTaskDelay(TIMEOUT_UNIT);
		timeout_cnt--;
	}

	// still empty
	if (_stdin_rcnt >= _stdin_wcnt) {
		asm(" nop");
		return -1;
	}

	int ridx = _stdin_rcnt % XMODEM_STDIN_SIZE;
	int widx = _stdin_wcnt % XMODEM_STDIN_SIZE;

	int ret = _stdin[ridx];
	_stdin_rcnt++;

	return ret;
}

void xmodem_putc(int c)
{
	int tmp = c;
	if (c_write) {
		c_write(0, (void *)&tmp, 1);
	}
	if (r_write) {
		r_write(0, (void *)&tmp, 1);
	}

}

#define UART_BUF_SIZE	576*2 // 115200/10/1000*50, baud rate/8bit+1bit start + 1bit stop/1000ms*50ms
static unsigned char recv_buf[UART_BUF_SIZE];
static volatile int rx_task_stop = 0;
void xmodem_rx_task(void *dummy)
{
	(void)dummy;
	//char buf[UART_BUF_SIZE];
	int rd_len;
	while (!rx_task_stop) {
		if (c_read && (rd_len = c_read(0, (char *)recv_buf, UART_BUF_SIZE)) != 0) {
			xmodem_stdin_handling(recv_buf, rd_len);
		} else if (r_read && (rd_len = r_read(0, (char *)recv_buf, UART_BUF_SIZE)) != 0) {
			xmodem_stdin_handling(recv_buf, rd_len);
		} else {
			vTaskDelay(TIMEOUT_UNIT);
		}
	}
	vTaskDelete(NULL);
}

int console_xmodem_init(void)
{
	// flush stdout
	printf("\n");
	console_stdio_get((void *)&c_read, (void *)&c_write);
	remote_stdio_get((void *)&r_read, (void *)&r_write);

	console_stdio_init(NULL, NULL);
	remote_stdio_init(NULL, NULL);

	_stdin_wcnt = _stdin_rcnt = 0;

	rx_task_stop = 0;
	if (xTaskCreate(xmodem_rx_task, "xm_rx", 512, NULL, 2, NULL) != pdPASS) {
		return -1;
	}

	xmodemSetInterface(xmodem_getc, xmodem_putc);
	return 0;
}

int console_xmodem_deinit(void)
{
	rx_task_stop = 1;

	console_stdio_init(c_read, c_write);
	remote_stdio_init(r_read, r_write);

	return 0;
}

int console_xmodem_tx_buffer(uint8_t *buf, int len)
{
	int ret = -1;

	if (console_xmodem_init() < 0) {
		goto tx_fail;
	}

	ret = xmodemTransmit(buf, len, M_START | M_MIDDLE | M_FINAL);

	// test seperate tx
	//ret = xmodemTransmit(buf, 1024, M_START);
	//ret = xmodemTransmit(&buf[1024], len-1024, M_FINAL);

tx_fail:
	console_xmodem_deinit();

	return ret;
}

void console_dump_memory(uint8_t *buf, int len)
{
	printf("-------------------------------------------\r\n");

	for (int i = 0; i < len; i += 16) {
		for (int x = 0; x < 16; x++) {
			printf("%02x ", buf[i + x]);
		}
		printf("\r\n");
	}

	printf("-------------------------------------------\r\n");
}

int console_xmodem_rx_buffer(uint8_t *buf, int len)
{
	int ret = -1, eot = 0;

	if (console_xmodem_init() < 0) {
		goto tx_fail;
	}

	ret = xmodemReceive(buf, len, M_START | M_MIDDLE | M_FINAL, &eot);

tx_fail:
	console_xmodem_deinit();

	return ret;
}


int console_xmodem_tx_file(char *file)
{
	int ret = -1;
	int data_len = 0;
	uint8_t *data_buf = NULL;
	FILE *m_file = NULL;
#define FILE_TX_BUF_SIZE 4096
	// open file or read ftl

	m_file = fopen(file, "a+");
	if (!m_file) {
		printf("open file (%s) fail.\n", file);
		goto end_tx;
	}
	fseek(m_file, 0, SEEK_END);
	data_len = ftell(m_file);
	fseek(m_file, 0, SEEK_SET);

	printf("file size %d\r\n", data_len);
	if (console_xmodem_init() < 0) {
		goto end_tx;
	}

	data_buf = malloc(FILE_TX_BUF_SIZE);
	if (!data_buf) {
		goto end_tx;
	}

	int rd_len = 0;
	while (rd_len < data_len) {
		int proc_len = data_len - rd_len > FILE_TX_BUF_SIZE ? FILE_TX_BUF_SIZE : data_len - rd_len;
		int curr_len = 0;
		while (curr_len < proc_len) {
			curr_len += fread(&data_buf[curr_len], 1, proc_len - curr_len, m_file);
		}

		int mode = M_MIDDLE;
		if (rd_len == 0) {
			mode |= M_START;
		}
		if (data_len - rd_len <= FILE_TX_BUF_SIZE) {
			mode |= M_FINAL;
		}

		ret = xmodemTransmit(data_buf, proc_len, mode);
		if (ret < 0) {
			printf("xmodem tx error %d\r\n", ret);
			goto end_tx;
		}

		rd_len += proc_len;
	}
	ret = 0;
end_tx:

	if (data_buf) {
		free(data_buf);
	}
	if (m_file) {
		fclose(m_file);
	}

	console_xmodem_deinit();

	return ret;
}

int console_xmodem_rx_file(char *file)
{
	int ret = -1;
	int data_len = 0;
	uint8_t *data_buf = NULL;
	FILE *m_file = NULL;

	int eot = 0;
	int start = 1;
	int xres;	
	
#define FILE_RX_BUF_SIZE 1024
	// open file or read ftl

	if (console_xmodem_init() < 0) {
		goto end_rx;
	}

	m_file = fopen(file, "w");
	if (!m_file) {
		printf("open file (%s) fail.\n", file);
		goto end_rx;
	}

	data_buf = malloc(FILE_RX_BUF_SIZE);
	if (!data_buf) {
		goto end_rx;
	}

	while (eot == 0) {
		if (start == 1) {
			xres = xmodemReceive(data_buf, FILE_RX_BUF_SIZE, M_START, &eot);
		} else {
			xres = xmodemReceive(data_buf, FILE_RX_BUF_SIZE, M_MIDDLE, &eot);
		}
		start = 0;

		if (xres < 0) {
			goto end_rx;
		}


		if (eot == 1) {
			// remove 0x1A padding
			for (int i = FILE_RX_BUF_SIZE - 1; i >= 0; i--) {
				if (data_buf[i] != 0x1A) {
					break;
				}
				xres--;
			}
		}

		if (xres > 0 && fwrite(data_buf, 1, xres, m_file) < 0) {
			goto end_rx;
		}
		data_len += xres;

	}
	ret = 0;
end_rx:

	if (data_buf) {
		free(data_buf);
	}
	if (m_file) {
		fclose(m_file);
	}

	console_xmodem_deinit();

	printf("xmodem file recived %d bytes, eot %d\n\r", data_len, eot);
	return ret;
}
