#include "device.h"
#include "sgpio_api.h"

#define SGPIO_TX_PIN    PF_15
#define SGPIO_RX_PIN    PF_14

#define RX_SAMPLING_BIT_CNT 32
#define TX_TRANSMIT_BIT_CNT 64

static sgpio_t sgpio_obj;

volatile u32 txdata[2];
volatile u32 rxdata[2];
volatile u32 rxdata_ptr = 0;

volatile u32 rx_done = 0;
volatile u32 tx_done = 0;

void sgpio_txdata_end_handler(void *data)
{
	tx_done = 1;
}

void sgpio_rxdata_end_handler(void *data)
{
	sgpio_t *psgpio_obj = (sgpio_t *)data;
	rxdata[rxdata_ptr] = sgpio_get_input_rxdata(psgpio_obj);
	rxdata_ptr++;
	if (rxdata_ptr == 2) {
		rxdata_ptr = 0;
		rx_done = 1;
	}
}

void main(void)
{
	// Init SGPIO
	sgpio_init(&sgpio_obj, SGPIO_TX_PIN, SGPIO_RX_PIN);

	// Init SGPIO RX
	sgpio_sampling_rxdata(&sgpio_obj, ENABLE, RXTC_FALL_EDGE, UNIT_US, 0, 0, 35, RX_SAMPLING_BIT_CNT, FIRST_LSB, sgpio_rxdata_end_handler, &sgpio_obj);

	// Set tx data
	txdata[0] = 0xDDCCBBAA;
	txdata[1] = 0x44332211;

	// Init the TX encoded formats of bit 0 and bit 1
	sgpio_set_bit_symbol_of_txdata(&sgpio_obj, OUTPUT_HIGH, UNIT_US, 60, 70, 10, 70);

	// Init SGPIO TX
	sgpio_set_txdata(&sgpio_obj, ENABLE, DISABLE, TX_TRANSMIT_BIT_CNT, (uint32_t *)txdata, sgpio_txdata_end_handler, NULL);

	// Start to transmit TX data
	sgpio_start_send_txdata(&sgpio_obj);

	while (1) {
		if (tx_done) {
			tx_done = 0;
			dbg_printf("TX END... \r\n");
		}

		if (rx_done) {
			rx_done = 0;
			dbg_printf("RX END... \r\n");
			dbg_printf("rxdata[0]: 0x%8x \r\n", rxdata[0]);
			dbg_printf("rxdata[1]: 0x%8x \r\n", rxdata[1]);
		}
	}


}


