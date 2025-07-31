#include "power_mode_api.h"
#include "gpio_api.h"
#include "gpio_ex_api.h"
#include "gpio_irq_api.h"
#include "gpio_irq_ex_api.h"
#include "serial_api.h"
#include "timer_api.h"
#include "pwmout_api.h"
#include "rtc_api.h"

//wake up by Stimer    : 0
//wake up by AON_GPIO  : 1
//wake up by UART      : 2	//Only support 4MHz clock source
//wake up by Gtimer    : 3	//Only support 4MHz clock source
//wake up by PON_GPIO  : 4
//wake up by RTC       : 5
#define WAKEUP_SOURCE 0
//Clock, 1: 4MHz, 0: 100kHz
#define CLOCK 0
//SLEEP_DURATION, 5s
#define SLEEP_DURATION (5 * 1000 * 1000)

#if (WAKEUP_SOURCE == 1)
#define WAKUPE_GPIO_PIN PA_2
static gpio_irq_t my_GPIO_IRQ;
void gpio_demo_irq_handler(uint32_t id, gpio_irq_event event)
{

	dbg_printf("%s==> \r\n", __FUNCTION__);

}
#endif

#if (WAKEUP_SOURCE == 2)
//UART0
#define UART_TX    PA_2
#define UART_RX    PA_3
static serial_t my_UART;
volatile char rc = 0;

static void uart_send_string(serial_t *my_UART, char *pstr)
{
	unsigned int i = 0;
	while (*(pstr + i) != 0) {
		serial_putc(my_UART, *(pstr + i));
		i++;
	}
}

static void uart_irq(uint32_t id, SerialIrq event)
{
	serial_t *my_UART = (void *)id;
	if (event == RxIrq) {
		rc = serial_getc(my_UART);
		serial_putc(my_UART, rc);
	}
	if ((event == TxIrq) && (rc != 0)) {
		uart_send_string(my_UART, (char *)"\r\n8735B$ \r\n");
		rc = 0;
	}
}
#endif

#if (WAKEUP_SOURCE == 3)
#define GTIMER_SLEEP_DURATION (5 * 1000 * 1000)
//TIMER0
#define WAKEUP_GTIMER TIMER0
static gtimer_t my_Gtimer;

static void Gtimer_timeout_handler(uint32_t id)
{
	dbg_printf("You will never see this message   \r\n");
}
#endif

#if (WAKEUP_SOURCE == 4)
#define WAKUPE_GPIO_PIN PF_1 //SLP_PON_GPIO
static gpio_irq_t my_GPIO_IRQ;

void gpio_demo_irq_handler(uint32_t id, gpio_irq_event event)
{
	dbg_printf("%s==> \r\n", __FUNCTION__);
}
#endif


#if (WAKEUP_SOURCE == 5)
static alarm_t alarm;
void rtc_handler(void)
{
	dbg_printf("%s==> \r\n", __FUNCTION__);
}
#endif



int main(void)
{
	hal_xtal_divider_enable(1);
	hal_32k_s1_sel(2);
	HAL_WRITE32(0x40009000, 0x18, 0x1 | HAL_READ32(0x40009000, 0x18)); //SWR 1.35V

	dbg_printf("\r\n   PM_Standby DEMO   \r\n");
	//dbg_printf("Wait 10s to enter Standby\r\n");
	//hal_delay_us(10 * 1000 * 1000);

#if (WAKEUP_SOURCE == 0)
//set gpio pull control
	gpio_t my_GPIO1;
	gpio_init(&my_GPIO1, PA_2);
	gpio_pull_ctrl(&my_GPIO1, PullDown);
	dbg_printf("Enter Standby, wake up by Stimer \r\n");
	for (int i = 5; i > 0; i--) {
		dbg_printf("Enter Standby by %d seconds \r\n", i);
		hal_delay_us(1 * 1000 * 1000);
	}
	Standby(SLP_AON_TIMER, SLEEP_DURATION, CLOCK, 0);

#elif (WAKEUP_SOURCE == 1)
	dbg_printf("Enter Standby, wake up by AON_GPIO");
//if there is no GPIO wakeup source please set a GPIO IRQ for wake up

//    gpio_irq_init(&my_GPIO_IRQ, WAKUPE_GPIO_PIN, NULL, (uint32_t)&my_GPIO_IRQ);
	gpio_irq_init(&my_GPIO_IRQ, WAKUPE_GPIO_PIN, gpio_demo_irq_handler, (uint32_t)&my_GPIO_IRQ);
	gpio_irq_pull_ctrl(&my_GPIO_IRQ, PullDown);
	gpio_irq_set(&my_GPIO_IRQ, IRQ_RISE, 1);
	dbg_printf("_A%d \r\n", WAKUPE_GPIO_PIN);

	for (int i = 5; i > 0; i--) {
		dbg_printf("Enter Standby by %d seconds \r\n", i);
		hal_delay_us(1 * 1000 * 1000);
	}

	Standby(SLP_AON_GPIO, SLEEP_DURATION, CLOCK, 0);

#elif (WAKEUP_SOURCE == 2)
	dbg_printf("Enter Standby, wake up by UART \r\n");
	hal_delay_ms(5);

	serial_init(&my_UART, UART_TX, UART_RX);
	serial_baud(&my_UART, 115200);
	serial_format(&my_UART, 8, ParityNone, 1);
	serial_irq_handler(&my_UART, uart_irq, (uint32_t)&my_UART);
	serial_irq_set(&my_UART, RxIrq, 1);
	serial_irq_set(&my_UART, TxIrq, 1);
	uart_send_string(&my_UART, (char *)"Enter Standby, wake up by UART \r\n");
	for (int i = 5; i > 0; i--) {
		dbg_printf("Enter Standby by UART %d seconds \r\n", i);
		hal_delay_us(1 * 1000 * 1000);
	}
	Standby(SLP_UART, SLEEP_DURATION, 1, 0);

#elif (WAKEUP_SOURCE == 3)
	dbg_printf("Enter Standby, wake up by Gtimer \r\n");
//set gpio pull control
	gpio_t my_GPIO1;
	gpio_init(&my_GPIO1, PA_2);
	gpio_pull_ctrl(&my_GPIO1, PullDown);

	hal_delay_ms(5);
	gtimer_init(&my_Gtimer, WAKEUP_GTIMER);

	for (int i = 5; i > 0; i--) {
		dbg_printf("Enter Standby by %d seconds \r\n", i);
		hal_delay_us(1 * 1000 * 1000);
	}
	gtimer_start_one_shout(&my_Gtimer, GTIMER_SLEEP_DURATION, (void *)Gtimer_timeout_handler, (uint32_t)NULL);
	Standby(SLP_GTIMER, SLEEP_DURATION, 1, 0);

#elif (WAKEUP_SOURCE == 4)
	dbg_printf("Enter Standby, wake up by PON_GPIO");

	hal_delay_ms(5);
//if there is no GPIO wakeup source please set a GPIO IRQ for wake up

//    gpio_irq_init(&my_GPIO_IRQ, WAKUPE_GPIO_PIN, NULL, (uint32_t)&my_GPIO_IRQ);
	gpio_irq_init(&my_GPIO_IRQ, WAKUPE_GPIO_PIN, gpio_demo_irq_handler, (uint32_t)&my_GPIO_IRQ);
	gpio_irq_pull_ctrl(&my_GPIO_IRQ, PullDown);
	gpio_irq_set(&my_GPIO_IRQ, IRQ_RISE, 1);
	dbg_printf("_A%d \r\n", WAKUPE_GPIO_PIN);

	for (int i = 5; i > 0; i--) {
		dbg_printf("Enter Standby by %d seconds \r\n", i);
		hal_delay_us(1 * 1000 * 1000);
	}
//set gpio pull control
	HAL_WRITE32(0x40009850, 0x0, 0x4f004f); //GPIOF_1/GPIOF_0
	HAL_WRITE32(0x40009854, 0x0, 0x8f004f); //GPIOF_3/GPIOF_2
	HAL_WRITE32(0x40009858, 0x0, 0x4f008f); //GPIOF_5/GPIOF_4
	HAL_WRITE32(0x4000985c, 0x0, 0x4f004f); //GPIOF_7/GPIOF_6
	HAL_WRITE32(0x40009860, 0x0, 0x4f004f); //GPIOF_9/GPIOF_8
	HAL_WRITE32(0x40009864, 0x0, 0x4f004f); //GPIOF_11/GPIOF_10
	HAL_WRITE32(0x40009868, 0x0, 0x4f004f); //GPIOF_13/GPIOF_12
	HAL_WRITE32(0x4000986C, 0x0, 0x4f004f); //GPIOF_15/GPIOF_14
	HAL_WRITE32(0x40009870, 0x0, 0x4f004f); //GPIOF_17/GPIOF_16
//	HAL_WRITE32(0x4000Ae04, 0x0, 0x20000); //GPIOF_17(VDD_DDR_EN) INPUT MODE

	gpio_t my_GPIO1;
	gpio_init(&my_GPIO1, PA_2);
	gpio_pull_ctrl(&my_GPIO1, PullDown);
	gpio_t my_GPIO2;
	gpio_init(&my_GPIO2, PA_3);
	gpio_pull_ctrl(&my_GPIO2, PullDown);
	Standby(SLP_PON_GPIO, SLEEP_DURATION, CLOCK, 0);

#elif (WAKEUP_SOURCE == 5)
	dbg_printf("Enter Standby, wake up by RTC \r\n");

	rtc_init();
	rtc_write(0);

	for (int i = 5; i > 0; i--) {
		dbg_printf("Enter Standby by %d seconds \r\n", i);
		hal_delay_us(1 * 1000 * 1000);
	}

	rtc_set_alarm_time(10, rtc_handler);

	Standby(SLP_RTC, SLEEP_DURATION, 1, 0);
#endif

	dbg_printf("You won't see this log \r\n");
	while (1);
}
