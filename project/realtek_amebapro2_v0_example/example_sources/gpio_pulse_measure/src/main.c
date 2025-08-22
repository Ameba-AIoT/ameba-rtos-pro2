#include "gpio_api.h"
#include "gpio_irq_api.h"
#include "gpio_irq_ex_api.h"
#include "us_ticker_api.h"
#include "wait_api.h"

#define GPIO_OUT_PIN        PE_0
#define GPIO_IRQ_PIN        PA_3

gpio_t gpio_out;
gpio_irq_t gpio_irq;
volatile char irq_rise;

void gpio_demo_irq_handler(uint32_t id, gpio_irq_event event)
{
	static unsigned int rise_time;
	static unsigned int fall_time;

	if (irq_rise) {
		rise_time = us_ticker_read();
		// Changed as Falling Edge Trigger
		gpio_irq_set_event(&gpio_irq, IRQ_FALL);
		irq_rise = 0;
	} else {
		fall_time = us_ticker_read();
		// Changed as Rising Edge Trigger
		gpio_irq_set_event(&gpio_irq, IRQ_RISE);
		irq_rise = 1;

		dbg_printf("%d \r\n", (fall_time - rise_time));
	}
}

int main(void)
{
	dbg_printf("\r\n   GPIO Pulse Measure DEMO   \r\n");

	// Init LED control pin
	gpio_init(&gpio_out, GPIO_OUT_PIN);
	gpio_dir(&gpio_out, PIN_OUTPUT);    // Direction: Output
	gpio_mode(&gpio_out, PullNone);    // No pull
	gpio_write(&gpio_out, 0);

	// Initial Push Button pin as interrupt source
	gpio_irq_init(&gpio_irq, GPIO_IRQ_PIN, gpio_demo_irq_handler, (uint32_t)(&gpio_irq));
	gpio_irq_set(&gpio_irq, IRQ_RISE, 1);    // Falling Edge Trigger
	irq_rise = 1;
	gpio_irq_pull_ctrl(&gpio_irq, PullNone);
	gpio_irq_enable(&gpio_irq);

	while (1) {
		wait_ms(500);
		gpio_write(&gpio_out, 1);
		wait_us(1000);
		gpio_write(&gpio_out, 0);
	}
}
