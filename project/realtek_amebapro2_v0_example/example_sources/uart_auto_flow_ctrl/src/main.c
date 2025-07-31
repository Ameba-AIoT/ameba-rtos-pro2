

// This example demo the function of Auto Flow control
// Please connect 2 board to run this example.
// Board1   <----->     Board2
// PE1      <----->     PE1
// PE2      <----->     PE2
// PE3      <----->     PE3
// PE4      <----->     PE4
// GND      <----->     GND

// The first started board will be the TX side, the othse one will be the RX side
// The RX side will make some delay every 16-bytes received,
// by this way we can trigger the flow control mechanism.
#include <string.h>
#include "serial_api.h"
#include "wait_api.h"
#include "main.h"

#define UART_TX    PE_1
#define UART_RX    PE_2
#define UART_RTS   PE_4
#define UART_CTS   PE_3


extern void serial_clear_rx(serial_t *obj);
/*
void uart_send_string(serial_t *sobj, char *pstr)
{
    unsigned int i=0;

    while (*(pstr+i) != 0) {
        serial_putc(sobj, *(pstr+i));
        i++;
    }
}*/
#define UART_BUF_SIZE   1000

serial_t sobj;
unsigned char buffer[UART_BUF_SIZE];

void main(void)
{
	// sample text
	char rc;
	char sent;
	int i, j;
	int rx_side = 0;

	dbg_printf("\r\n   UART Auto Flow Ctrl DEMO   \r\n");


	// mbed uart test
	serial_init(&sobj, UART_TX, UART_RX);
	serial_baud(&sobj, 38400);
	serial_format(&sobj, 8, ParityNone, 1);
	serial_set_flow_control(&sobj, FlowControlNone, (PinName)0, (PinName)0);// Pin assignment can be ignored when autoflow control function is disabled

	wait_ms(10000);
	serial_clear_rx(&sobj);
	for (sent = 0; sent < 126; sent++) {
		dbg_printf("Wait peer ready... \r\n");
		serial_putc(&sobj, sent);
		if (serial_readable(&sobj)) {
			rc = serial_getc(&sobj);
			if (rc > sent) {
				rx_side = 1;
				serial_putc(&sobj, 0);
			} else {
				rx_side = 0;
			}
			break;
		}
		wait_ms(100);
	}

	// Enable flow control
	serial_set_flow_control(&sobj, FlowControlRTSCTS, UART_RTS, UART_CTS);
	serial_clear_rx(&sobj);
	wait_ms(5000);

	if (rx_side) {
		dbg_printf("UART Flow Control: RX ==> \r\n");
		memset(buffer, 0, UART_BUF_SIZE);

		i = 0;
		j = 0;
		while (1) {
			if (serial_readable(&sobj)) {
				buffer[i] = serial_getc(&sobj);
				i++;
				if (i == UART_BUF_SIZE) {
					break;
				}

				if ((i & 0xf) == 0) {
					// Make some delay to cause the RX FIFO full and then trigger flow controll
					wait_ms(100);
					dbg_printf("UART RX got %d bytes\r\n", i);
				}
				j = 0;
			} else {
				wait_ms(10);
				j++;
				if (j == 1000) {
					dbg_printf("UART RX Failed, Got %d bytes\r\n", i);
					break;
				}
			}

		}
	} else {
		dbg_printf("UART Flow Control: TX ==>\r\n");
		wait_ms(500);
		for (i = 0; i < UART_BUF_SIZE; i++) {
			buffer[i] = 0x30 + (i % 10);
		}

		for (i = 0; i < UART_BUF_SIZE; i++) {
			serial_putc(&sobj, buffer[i]);
		}
	}

	dbg_printf("UART Flow Control Test Done!\r\n");
	while (1);
}

