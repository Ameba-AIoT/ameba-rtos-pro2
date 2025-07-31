#include "device.h"
#include "serial_api.h"
#include "main.h"

#define UART_TX    PE_1
#define UART_RX    PE_2

volatile char rc = 0;

void uart_send_str(serial_t *sobj, char *pstr)
{
	unsigned int i = 0;

	while (*(pstr + i) != 0) {
		serial_putc(sobj, *(pstr + i));
		i++;
	}
}

void uart_irq_example(uint32_t id, SerialIrq event)
{
	serial_t    *sobj = (void *)id;

	if (event == RxIrq) {
		rc = serial_getc(sobj);
		serial_putc(sobj, rc);
	}

	if (event == TxIrq && rc != 0) {
		uart_send_str(sobj, (char *)"\r\n8735b$");
		rc = 0;
	}
}

int main(void)
{
	// sample text
	serial_t    sobj;

	// mbed uart test
	serial_init(&sobj, UART_TX, UART_RX);
	serial_baud(&sobj, 38400);
	serial_format(&sobj, 8, ParityNone, 1);

	uart_send_str(&sobj, (char *)"UART IRQ API Demo...\r\n");
	uart_send_str(&sobj, (char *)"Hello World!!\n");
	uart_send_str(&sobj, (char *)"\r\n8735b$");
	serial_irq_handler(&sobj, uart_irq_example, (uint32_t)&sobj);
	serial_irq_set(&sobj, RxIrq, 1);
	serial_irq_set(&sobj, TxIrq, 1);

	while (1);

}


