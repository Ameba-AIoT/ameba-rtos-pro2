#include <string.h>
#include "serial_api.h"
#include "serial_ex_api.h"

#define UART_TX    PE_1
#define UART_RX    PE_2

#define SRX_BUF_SZ      16

char rx_buf[SRX_BUF_SZ] = {0};
volatile uint32_t tx_busy = 0;
volatile uint32_t rx_done = 0;

void uart_send_string_done(uint32_t id)
{
//    serial_t    *sobj = (void*)id;
	tx_busy = 0;
}

void uart_recv_string_done(uint32_t id)
{
//    serial_t    *sobj = (void*)id;
	rx_done = 1;
}

static void uart_send_string(serial_t *sobj, char *pstr)
{
	int32_t ret = 0;

	if (tx_busy) {
		return;
	}

	tx_busy = 1;
	ret = serial_send_stream(sobj, pstr, strlen(pstr));
	if (ret != 0) {
		dbg_printf("%s Error(%d) \r\n", __FUNCTION__, ret);
		tx_busy = 0;
	}
}

int main(void)
{
	serial_t sobj;
	int ret;

	dbg_printf("\r\n   UART Stream IRQ Demo   \r\n");

	serial_init(&sobj, UART_TX, UART_RX);
	serial_baud(&sobj, 38400);
	serial_format(&sobj, 8, ParityNone, 1);

	serial_send_comp_handler(&sobj, (void *)uart_send_string_done, (uint32_t)&sobj);
	serial_recv_comp_handler(&sobj, (void *)uart_recv_string_done, (uint32_t)&sobj);

	ret = serial_recv_stream(&sobj, rx_buf, 8);
	if (ret) {
		dbg_printf(" %s: Recv Error(%d) \r\n", __FUNCTION__, ret);
		rx_done = 1;
	}

	while (1) {
#if 0
		if (!tx_busy) {
			uart_send_string(&sobj, "Hello! World!! :) \r\n");
		}
#endif
		if (rx_done) {
			uart_send_string(&sobj, rx_buf);
			rx_done = 0;
			ret = serial_recv_stream(&sobj, rx_buf, 8);
			if (ret) {
				dbg_printf(" %s: Recv Error(%d) \r\n", __FUNCTION__, ret);
				rx_done = 1;
			}
		}
	}
}
