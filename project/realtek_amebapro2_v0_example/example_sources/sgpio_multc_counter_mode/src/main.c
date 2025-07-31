#include "device.h"
#include "sgpio_api.h"

#define SGPIO_TX_PIN    PF_15
#define SGPIO_RX_PIN    PF_14

static sgpio_t sgpio_obj;

volatile u32 counter_match = 0;

void sgpio_multc_counter_match_handler(void *data)
{
	counter_match++;
}

void main(void)
{
	// Init SGPIO
	sgpio_init(&sgpio_obj, SGPIO_TX_PIN, SGPIO_RX_PIN);

	// Make SGPIO become the counter mode
	sgpio_multc_counter_mode(&sgpio_obj, ENABLE, INPUT_RISE_EDGE, 5, sgpio_multc_counter_match_handler, &sgpio_obj, MULTC_RESET,
							 UNIT_US, 0, NULL, NULL);

	// TX Start, multc_value is 0
	sgpio_set_inverse_output(&sgpio_obj);  //multc_value is 1.
	sgpio_set_inverse_output(&sgpio_obj);
	sgpio_set_inverse_output(&sgpio_obj);  //multc_value is 2.
	sgpio_set_inverse_output(&sgpio_obj);
	sgpio_set_inverse_output(&sgpio_obj);  //multc_value is 3.
	sgpio_set_inverse_output(&sgpio_obj);
	sgpio_set_inverse_output(&sgpio_obj);  //multc_value is 4.
	sgpio_set_inverse_output(&sgpio_obj);
	sgpio_set_inverse_output(&sgpio_obj);  //multc_value is 5. Happen the match interrupt, and make the multc value 0.
	sgpio_set_inverse_output(&sgpio_obj);
	sgpio_set_inverse_output(&sgpio_obj);  //multc_value is 1.
	sgpio_set_inverse_output(&sgpio_obj);
	sgpio_set_inverse_output(&sgpio_obj);  //multc_value is 2.
	sgpio_set_inverse_output(&sgpio_obj);
	sgpio_set_inverse_output(&sgpio_obj);  //multc_value is 3.
	sgpio_set_inverse_output(&sgpio_obj);

	while (1) {
		if (counter_match == 1) {
			dbg_printf("multc match event, multc is 5.  \r\n ");
			break;
		}
	}

	dbg_printf("Because the match event happened, the count value is %d. \r\n ", sgpio_get_multc_value(&sgpio_obj));

	while (1);

}

