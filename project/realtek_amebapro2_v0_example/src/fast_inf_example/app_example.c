/******************************************************************************
*
* Copyright(c) 2007 - 2024 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "fast_inf_example.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

int get_inf_result = 0;
void fast_inf_init_task(void *param)
{
	/* Execute application example */
	fast_inf_video_example();
	
	while(!get_inf_result) {
		vTaskDelay(1);
	}

	wlan_tcp_resume();

	vTaskDelete(NULL);
}

void app_example(void) {
	if (xTaskCreate(fast_inf_init_task, ((const char *)"fast_inf_init"), 2048, NULL, tskIDLE_PRIORITY + 6, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(init_task) failed\n", __FUNCTION__);
	}
}