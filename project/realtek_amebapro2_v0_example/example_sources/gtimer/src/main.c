#include "gpio_api.h"
#include "timer_api.h"

#define GPIO_LED_PIN1       PE_0
#define GPIO_LED_PIN2       PE_1

#define PERIODICAL_TIMER    TIMER2
#define ONE_SHOUT_TIMER     TIMER3

gtimer_t my_timer1;
gtimer_t my_timer2;
gpio_t gpio_led1;
gpio_t gpio_led2;

volatile uint32_t time2_expired = 0;
static hal_timer_group_adapter_t _timer_group0;

static void timer1_timeout_handler(uint32_t id)
{
	gpio_t *gpio_led = (gpio_t *)id;
	gpio_write(gpio_led, !gpio_read(gpio_led));
}

static void timer2_timeout_handler(uint32_t id)
{
	time2_expired = 1;
}

int main(void)
{
	dbg_printf("\r\n   Gtimer DEMO   \r\n");

	// Init LED control pin
	gpio_init(&gpio_led1, GPIO_LED_PIN1);
	gpio_dir(&gpio_led1, PIN_OUTPUT);// Direction: Output
	gpio_mode(&gpio_led1, PullNone);// No pull

	gpio_init(&gpio_led2, GPIO_LED_PIN2);
	gpio_dir(&gpio_led2, PIN_OUTPUT);// Direction: Output
	gpio_mode(&gpio_led2, PullNone);// No pull

	// Initial a periodical timer
	gtimer_init(&my_timer1, PERIODICAL_TIMER);
	//Check timer had been initialisated before starting timer
	if (my_timer1.timer_adp.tid != 0xFF) {
		gtimer_start_periodical(&my_timer1, 1000000, (void *)timer1_timeout_handler, (uint32_t)&gpio_led1);
	}

	// Initial a one-shout timer and re-trigger it in while loop
	gtimer_init(&my_timer2, ONE_SHOUT_TIMER);
	time2_expired = 0;
	if (my_timer2.timer_adp.tid != 0xFF) {
		gtimer_start_one_shout(&my_timer2, 500000, (void *)timer2_timeout_handler, (uint32_t)NULL);
	}

	while (1) {
		if (time2_expired) {
			gpio_write(&gpio_led2, !gpio_read(&gpio_led2));
			time2_expired = 0;
			if (my_timer2.timer_adp.tid != 0xFF) {
				gtimer_start_one_shout(&my_timer2, 500000, (void *)timer2_timeout_handler, (uint32_t)NULL);
			}
		}
	}
}
