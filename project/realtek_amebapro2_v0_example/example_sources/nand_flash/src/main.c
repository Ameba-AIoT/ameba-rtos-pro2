

#include "snand_api.h"
#include "device_lock.h"

// Decide starting flash address for storing application data
// User should pick address carefully to avoid corrupting image section

#define FLASH_TEST_BASE_BLOCK    0x100
static void nand_flash_test_task(void *param)
{


	dbg_printf("\r\n   NAND FLASH DEMO   \r\n");

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	snand_t flash;
	uint8_t data[2112] __attribute__((aligned(32)));
	uint8_t data_w[2112] __attribute__((aligned(32)));

	snand_init(&flash);

	dbg_printf("Bad block scan!\n\r");
	for (int i = 0; i < 1024; i++) {
		snand_page_read(&flash, i * 64, 2048 + 32, &data[0]);
		if (data[2048] != 0xff) {
			dbg_printf("Block %d is bad block!\n\r", i);
		}
	}
	dbg_printf("Bad block scan done!\n\r");
	snand_page_read(&flash, FLASH_TEST_BASE_BLOCK * 64, 2048 + 32, &data[0]);

	if (data[2048] == 0xff) {
		snand_global_unlock();
		snand_erase_block(&flash, FLASH_TEST_BASE_BLOCK * 64);
		snand_page_read(&flash, FLASH_TEST_BASE_BLOCK * 64, 2048 + 32, &data[0]);
		for (int i = 0; i < 2112; i++) {
			data_w[i] = i & 0xff ;
		}

		snand_page_write(&flash, FLASH_TEST_BASE_BLOCK * 64, 2048, &data_w[0]);

		//read & check data
		snand_page_read(&flash, FLASH_TEST_BASE_BLOCK * 64, 2048 + 32, &data[0]);
		for (int i = 0; i < 2048; i++) {
			if (data[i] != data_w[i]) {
				dbg_printf("data[%d]=%d is not data_w[%d]=%d !\n\r", i, data[i], i, data_w[i]);
			}
		}
		dbg_printf("NAND FLASH DEMO Done!\n\r");

	} else {
		dbg_printf("Block %d is Bad Block!\n\r", FLASH_TEST_BASE_BLOCK);

	}

	vTaskDelete(NULL);
}

int main(void)
{
	if (xTaskCreate(nand_flash_test_task, ((const char *)"nand_flash_test_task"), 4096, NULL, (tskIDLE_PRIORITY + 1), NULL) != pdPASS) {
		dbg_printf("\n\r%s xTaskCreate(nand_flash_test_task) failed", __FUNCTION__);
	}
	vTaskStartScheduler();
	while (1) {
		vTaskDelay((1000 / portTICK_RATE_MS));
	}
}
