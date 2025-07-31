#include "power_mode_api.h"
#include "gpio_api.h"
#include "gpio_ex_api.h"
#include "gpio_irq_api.h"
#include "gpio_irq_ex_api.h"

//wake up by Stimer : 0
//wake up by GPIO   : 1
#define WAKEUP_SOURCE 0
//Clock, 1: 4MHz, 0: 100kHz
#define CLOCK 0
//SLEEP_DURATION, 5s
#define SLEEP_DURATION (5 * 1000 * 1000)

#if (WAKEUP_SOURCE == 1)
#define WAKUPE_GPIO_PIN PA_2
static gpio_irq_t my_GPIO_IRQ;
#endif

int main(void)
{
	dbg_printf("\r\n   PM_DeepSleep DEMO   \r\n");

	//dbg_printf("Wait 10s to enter DeepSleep\r\n");
	//hal_delay_us(10 * 1000 * 1000);

#if (WAKEUP_SOURCE == 0)
	dbg_printf("Enter DeepSleep, wake up by Stimer \r\n");

	//PA_2&PA_3 default pullup, need to be set according to external circuit
	gpio_t my_GPIO1;
	gpio_t my_GPIO2;
	gpio_init(&my_GPIO1, PA_2);
	gpio_pull_ctrl(&my_GPIO1, PullDown);
	gpio_init(&my_GPIO2, PA_3);
	gpio_pull_ctrl(&my_GPIO2, PullDown);

	for (int i = 5; i > 0; i--) {
		dbg_printf("Enter DeepSleep by %d seconds \r\n", i);
		hal_delay_us(1 * 1000 * 1000);
	}

	DeepSleep(DS_AON_TIMER, SLEEP_DURATION, CLOCK);

#elif (WAKEUP_SOURCE == 1)
	dbg_printf("Enter DeepSleep, wake up by GPIO");

//if there is no GPIO wakeup source please set a GPIO pin for wake up
	gpio_t my_GPIO2;
	gpio_init(&my_GPIO2, PA_3);
	gpio_pull_ctrl(&my_GPIO2, PullDown);
	gpio_irq_init(&my_GPIO_IRQ, WAKUPE_GPIO_PIN, NULL, (uint32_t)&my_GPIO_IRQ);
	gpio_irq_pull_ctrl(&my_GPIO_IRQ, PullDown);
	gpio_irq_set(&my_GPIO_IRQ, IRQ_RISE, 1);
	dbg_printf("_A%d \r\n", WAKUPE_GPIO_PIN);

	for (int i = 5; i > 0; i--) {
		dbg_printf("Enter DeepSleep by %d seconds \r\n", i);
		hal_delay_us(1 * 1000 * 1000);
	}
	DeepSleep(DS_AON_GPIO, SLEEP_DURATION, CLOCK);
#endif

	dbg_printf("You won't see this log \r\n");
	while (1);
}
