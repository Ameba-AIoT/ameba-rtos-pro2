#include "device.h"
#include "device_lock.h"
//#include "pwmout_api.h"   // mbed
//#include "main.h"

#include "analogin_api.h"
#include "wait_api.h"

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
#include "osdep_service.h"
#endif

#define MBED_ADC_EXAMPLE_PIN_0    PF_0
#define MBED_ADC_EXAMPLE_PIN_1    PF_1
#define MBED_ADC_EXAMPLE_PIN_2    PF_2
#define MBED_ADC_EXAMPLE_PIN_3    PF_3 //log uart



#if defined (__ICCARM__)
analogin_t   adc0;
analogin_t   adc1;
analogin_t   adc2;
analogin_t   adc3;
#else
analogin_t   adc0;
analogin_t   adc1;
analogin_t   adc2;
analogin_t   adc3;
#endif


void adc_delay(void)
{
	int i;
	for (i = 0; i < 8000000; i++) {
		asm(" nop");
	}
}

static void adc_test_task(void *param)
{

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	uint16_t adctmp     = 0;
	uint16_t adcdat0    = 0;
	uint16_t adcdat1    = 0;
	uint16_t adcdat2    = 0;
	uint16_t adcdat3    = 0;
	float    adcfloat   = 0;
	/**/
//	ConfigDebugErr |= (_DBG_ADC_ | _DBG_GDMA_); //| _DBG_MISC_
//	ConfigDebugInfo |= (_DBG_ADC_ | _DBG_GDMA_); //| _DBG_MISC_
//	ConfigDebugWarn|= (_DBG_ADC_ | _DBG_GDMA_); //| _DBG_MISC_

//	ConfigDebugErr |= (_DBG_MISC_ ); //| _DBG_MISC_
//	ConfigDebugInfo |= (_DBG_MISC_); //| _DBG_MISC_
//	ConfigDebugWarn|= (_DBG_MISC_ ); //| _DBG_MISC_

	device_mutex_lock(RT_DEV_LOCK_EFUSE);
	analogin_init(&adc1, MBED_ADC_EXAMPLE_PIN_0);
	analogin_init(&adc1, MBED_ADC_EXAMPLE_PIN_1);
	analogin_init(&adc2, MBED_ADC_EXAMPLE_PIN_2);
//    analogin_init(&adc3, MBED_ADC_EXAMPLE_PIN_3);
	device_mutex_unlock(RT_DEV_LOCK_EFUSE);

	for (;;) {
		adcdat0 = analogin_read_u16(&adc0);
		adcdat1 = analogin_read_u16(&adc1);
		adcdat2 = analogin_read_u16(&adc2);
//        adcdat3 = analogin_read_u16(&adc3);
		DBG_8735B("all channel\n");
		DBG_8735B("AD0:%08x, AD1:%08x, AD2:%08x\n", adcdat0, adcdat1, adcdat2);

		wait_ms(2000);

	}
	analogin_deinit(&adc0);
	analogin_deinit(&adc1);
	analogin_deinit(&adc2);
//	analogin_deinit(&adc3);

	vTaskDelete(NULL);
}

int main(void)
{
	if (xTaskCreate(adc_test_task, ((const char *)"adc_test_task"), 1024, NULL, (tskIDLE_PRIORITY + 1), NULL) != pdPASS) {
		dbg_printf("\n\r%s xTaskCreate(adc_test_task) failed", __FUNCTION__);
	}
	vTaskStartScheduler();
	while (1) {
		vTaskDelay((1000 / portTICK_RATE_MS));
	}
}
