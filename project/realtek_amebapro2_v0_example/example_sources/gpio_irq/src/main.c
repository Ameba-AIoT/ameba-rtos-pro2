
#include "gpio_api.h"
#include "gpio_irq_api.h"

#define GPIO_LED_PIN       PE_0
#define GPIO_IRQ_PIN       PA_3

int led_ctrl;
gpio_t gpio_led;

void gpio_demo_irq_handler(uint32_t id, gpio_irq_event event)
{
	gpio_t *gpio_led;

	dbg_printf("%s==> \r\n", __FUNCTION__);
	gpio_led = (gpio_t *)id;

	led_ctrl = !led_ctrl;
	gpio_write(gpio_led, led_ctrl);
}

int main(void)
{
	gpio_irq_t gpio_btn;

	dbg_printf("\r\n   GPIO IRQ DEMO   \r\n");

	// Init LED control pin
	gpio_init(&gpio_led, GPIO_LED_PIN);
	gpio_dir(&gpio_led, PIN_OUTPUT);    // Direction: Output
	gpio_mode(&gpio_led, PullNone);     // No pull

	// Initial Push Button pin as interrupt source
	gpio_irq_init(&gpio_btn, GPIO_IRQ_PIN, gpio_demo_irq_handler, (uint32_t)(&gpio_led));
	gpio_irq_set(&gpio_btn, IRQ_FALL, 1);    // Falling Edge Trigger
	gpio_irq_enable(&gpio_btn);

	led_ctrl = 1;
	gpio_write(&gpio_led, led_ctrl);

	while (1);
}
