/******************************************************************************
*
* Copyright(c) 2007 - 2023 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include "osdep_service.h"
#include "example_nn_file_tester.h"

void example_nn_file_tester_main(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif

	// object detection nn tester
	mmf2_example_vipnn_objectdet_test_init();

	// face detection nn tester
	//mmf2_example_vipnn_facedet_test_init();

	// face recognition nn tester
	//mmf2_example_vipnn_facerecog_test_init();

	vTaskDelete(NULL);
}

void example_nn_file_tester(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(example_nn_file_tester_main, ((const char *)"example_nn_file_tester_main"), 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n example_nn_file_tester_main: Create Task Error\n");
	}
}