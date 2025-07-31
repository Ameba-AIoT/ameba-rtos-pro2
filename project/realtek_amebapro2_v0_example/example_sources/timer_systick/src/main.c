#include "FreeRTOS.h"
#include "osdep_service.h"
#include "task.h"
#include "diag.h"
#include "gpio_api.h"   // mbed
#include "main.h"


extern void console_init(void);

const char *test = "string in main.c";


#define GPIO_LED_PIN1       PE_0

static TimerHandle_t my_timer1;
gpio_t gpio_led1;

volatile uint32_t time2_expired = 0;

void timer1_timeout_handler(void *param)
{
	gpio_write(&gpio_led1, !gpio_read(&gpio_led1));
}

static void example_my_timer1(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	// Init LED control pin
	gpio_init(&gpio_led1, GPIO_LED_PIN1);
	gpio_dir(&gpio_led1, PIN_OUTPUT);    // Direction: Output
	gpio_mode(&gpio_led1, PullNone);     // No pull

	// Create a periodical timer

	my_timer1 = xTimerCreate("my timer1",    /* Text name to facilitate debugging. */
							 1000 / portTICK_RATE_MS, /* Tick every 1 sec */
							 pdTRUE,     /* The timer will auto-reload themselves when they expire. */
							 NULL,     /* In this case this is not used as the timer has its own callback. */
							 (TimerCallbackFunction_t)timer1_timeout_handler);  /* The callback to be called when the timer expires. */

	xTimerStart(my_timer1, (TickType_t) 0);
	while (1) {

	}
}
int main(void)
{

	rt_printf("hello IS - %s, %x\n\r", test, malloc(100));
	/* Initialize log uart and at command service */
	console_init();

	xTaskCreate(example_my_timer1, (const char *)"AAA", 512, NULL, tskIDLE_PRIORITY + 1, NULL);


	vTaskStartScheduler();

}