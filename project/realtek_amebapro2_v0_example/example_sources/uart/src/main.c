#include "serial_api.h"

#define UART_TX    PE_1
#define UART_RX    PE_2

static void uart_send_string(serial_t *sobj, char *pstr)
{
	unsigned int i = 0;

	while (*(pstr + i) != 0) {
		serial_putc(sobj, *(pstr + i));
		i++;
	}
}

int main(void)
{
	// sample text
	char rc;
	serial_t sobj;

	dbg_printf("\r\n   UART DEMO   \r\n");

	// mbed uart test
	serial_init(&sobj, UART_TX, UART_RX);
	serial_baud(&sobj, 38400);
	serial_format(&sobj, 8, ParityNone, 1);

	uart_send_string(&sobj, (char *)"UART API Demo... \r\n");
	uart_send_string(&sobj, (char *)"Hello World!! \r\n");
	while (1) {
		uart_send_string(&sobj, (char *)"\r\n8735B$");
		rc = serial_getc(&sobj);
		serial_putc(&sobj, rc);
	}
}
