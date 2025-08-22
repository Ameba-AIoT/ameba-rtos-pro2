
#include "gpio_api.h"

#define GPIO_LED_PIN       PE_0
#define GPIO_PUSHBT_PIN    PA_3

int main(void)
{
	gpio_t gpio_led;
	gpio_t gpio_btn;

	dbg_printf("\r\n   GPIO DEMO   \r\n");

	// Init LED control pin
	gpio_init(&gpio_led, GPIO_LED_PIN);
	gpio_dir(&gpio_led, PIN_OUTPUT);        // Direction: Output
	gpio_mode(&gpio_led, PullNone);         // No pull

	// Initial Push Button pin
	gpio_init(&gpio_btn, GPIO_PUSHBT_PIN);
	gpio_dir(&gpio_btn, PIN_INPUT);         // Direction: Input
	gpio_mode(&gpio_btn, PullUp);           // Pull-High

	while (1) {
		if (gpio_read(&gpio_btn)) {
			// turn off LED
			gpio_write(&gpio_led, 0);
		} else {
			// turn on LED
			gpio_write(&gpio_led, 1);
		}
	}
}
