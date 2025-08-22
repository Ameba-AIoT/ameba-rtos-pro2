#include "device.h"
#include "sgpio_api.h"

#define SGPIO_TX_PIN    PF_15
#define SGPIO_RX_PIN    PF_14

#define RX_CAPTURING_BIT_CNT 32
#define TX_TRANSMIT_BIT_CNT 32

static sgpio_t sgpio_obj;

volatile u32 monitor_done = 0;
volatile u32 tx_done = 0;
volatile u32 tx_cnt = 0;

u32 txdata[1];

void sgpio_txdata_end_handler(void *data)
{
	tx_done = 1;
	tx_cnt++;
}

void sgpio_rxdata_monitor_handler(void *data)
{
	monitor_done = 1;
}

void main(void)
{
	// Init SGPIO
	sgpio_init(&sgpio_obj, SGPIO_TX_PIN, SGPIO_RX_PIN);

	// Init SGPIO RX
	sgpio_capture_compare_rxdata(&sgpio_obj, ENABLE, RXTC_FALL_EDGE, CAP_RISE_EDGE, 100, UNIT_US, 35, GET_BIT0, RX_CAPTURING_BIT_CNT, FIRST_LSB, NULL, NULL);

	// Initialize RX Monitor
	sgpio_set_rxdata_monitor(&sgpio_obj, ENABLE, 0xBBAA, 0xFFFF, sgpio_rxdata_monitor_handler, NULL);

	// Set tx data
	txdata[0] = 0xDDCCBBEE;

	// Init the TX encoded formats of bit 0 and bit 1
	sgpio_set_bit_symbol_of_txdata(&sgpio_obj, OUTPUT_HIGH, UNIT_US, 60, 70, 10, 70);

	// Init SGPIO TX
	sgpio_set_txdata(&sgpio_obj, ENABLE, DISABLE, TX_TRANSMIT_BIT_CNT, txdata, sgpio_txdata_end_handler, NULL);

	// Start to transmit TX data
	sgpio_start_send_txdata(&sgpio_obj);

	while (1) {
		if (tx_done) {
			tx_done = 0;
			dbg_printf(" 0x%8x TX END.. \r\n", txdata[0]);
			if (tx_cnt == 0x01) {
				// Set tx data
				txdata[0] = 0xDDCCBBAA;
				// Start to transmit TX data
				sgpio_start_send_txdata(&sgpio_obj);
			}
		}

		if (monitor_done) {
			monitor_done = 0;
			dbg_printf(" Match the monitor data \r\n");
		}
	}


}


