/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "platform_opts.h"
#include "platform_stdlib.h"
#include "ff.h"
#include <fatfs_ext/inc/ff_driver.h>
#include "sdio_combine.h"
#include "sdio_host.h"
#include <disk_if/inc/sdcard.h>
#include "fatfs_sdcard_api.h"
#include "osdep_service.h"
static fatfs_sd_params_t fatfs_sd;

#define ENABLE_SD_POWER_RESET

static void *sd_sema = NULL;
static uint8_t sd_checking = 0;
extern phal_sdhost_adapter_t psdioh_adapter;

void sdh_card_insert_callback(void *pdata)
{
	printf("[In]\r\n");
	for (int i = 0; i < 50000; i++) {
		asm("nop");
	}
	if (!sd_checking) {
		if (sd_sema) {
			rtw_up_sema_from_isr(&sd_sema);
		}
	}

}
void sdh_card_remove_callback(void *pdata)
{
	printf("[Out]\r\n");
	for (int i = 0; i < 50000; i++) {
		asm("nop");
	}
	if (!sd_checking) {
		if (sd_sema) {
			rtw_up_sema_from_isr(&sd_sema);
		}
	}
}

int sd_do_mount(void *parm)
{
	int i = 0;
	int res = 0;
#ifdef ENABLE_SD_POWER_RESET
	sd_gpio_power_reset();
#endif
	res = f_mount(NULL, fatfs_sd.drv, 1);
	if (res) {
		printf("UMount failed %d\r\n", res);
	} else {
		printf("UMount Successful\r\n");
	}
	for (int i = 0; i < 50000; i++) {
		asm("nop");
	}
	res = f_mount(&fatfs_sd.fs, fatfs_sd.drv, 1);
	if (res) {
		printf("Mount failed %d\r\n", res);
	} else {
		printf("Mount Successful\r\n");
	}
	return res;
}

static void sd_hot_plug_thread(void *param)
{
	int res = 0;

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
	sd_gpio_init();
#ifdef ENABLE_SD_POWER_RESET
	sd_gpio_power_reset();
#endif
	sdio_driver_init();
	sdio_set_init_retry_time(2);

	fatfs_sd.drv_num = FATFS_RegisterDiskDriver(&SD_disk_Driver);

	if (fatfs_sd.drv_num < 0) {
		printf("Rigester disk driver to FATFS fail.\n\r");
	} else {
		fatfs_sd.drv[0] = fatfs_sd.drv_num + '0';
		fatfs_sd.drv[1] = ':';
		fatfs_sd.drv[2] = '/';
		fatfs_sd.drv[3] = 0;
	}

	res = f_mount(&fatfs_sd.fs, fatfs_sd.drv, 1);
	if (res) {
		printf("Mount failed %d\r\n", res);
	} else {
		printf("Mount Successful\r\n");
	}

	//printf("free space %d\r\n",fatfs_get_free_space());
	rtw_init_sema(&sd_sema, 0);
	while (1) {
		rtw_down_sema(&sd_sema);
		sd_checking = 1;
		vTaskDelay(200);	// delay to get correct sd voltage

		//SDIO_HOST_Type *psdioh = SDIO_HOST;
		SDHOST_Type *psdioh = psdioh_adapter->base_addr;//SDHOST_Type
		if (psdioh->card_exist_b.sd_exist) {
			printf("card inserted!\n");
			res = sd_do_mount(NULL);
			if (res) { //Try again for fast hot plug
				sd_do_mount(NULL);
			}
		} else {
			printf("card OUT!\r\n");
			if (psdioh->card_exist_b.sd_exist) {
				sd_do_mount(NULL);
			}
		}
		sd_checking = 0;
	}

	vTaskDelete(NULL);
}


void example_sd_hot_plug(void)
{
	if (xTaskCreate(sd_hot_plug_thread, ((const char *)"sd_hot_plug"), 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(sd_hot_plug) failed", __FUNCTION__);
	}
}