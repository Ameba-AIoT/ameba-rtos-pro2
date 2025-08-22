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

#if CONFIG_EXAMPLE_SD_HOT_PLUG

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
static FIL fil;
static FATFS fs;
void format_for_sdcard()
{
	//FATFS fs;      /* File system object (volume work area) */
	//FIL fil;       /* File object */
	FRESULT res;   /* API result code */
	UINT bw;       /* Bytes written */
	//vTaskDelay(5000);
	printf("Start format\r\n");
	sdio_driver_init();
	/* Register work area */

	FATFS_RegisterDiskDriver(&SD_disk_Driver);

	res = f_mount(&fs, "", 0);
	//if(res )
	//printf("f_mount fail\r\n");
	/* Create FAT volume with default cluster size */

	//res = f_mkfs("", 0, 8*1024);
	MKFS_PARM format_attr;
	format_attr.fmt = FM_EXFAT;
	format_attr.n_fat = 0;
	format_attr.align = 0;
	format_attr.n_root = 0;
	format_attr.au_size = 128;

	res =  f_mkfs("", &format_attr, NULL, 64 * 1024);
	if (res) {
		printf("f_mkfs fail %d\r\n", res);
	}
	//fatfs_test_code();
	//while(1)
	//vTaskDelay(100);
	/* Create a file as new */
	res = f_open(&fil, "hello.txt", FA_CREATE_NEW | FA_WRITE);
	if (res) {
		printf("f_open fail\r\n");
	}

	/* Write a message */
	f_write(&fil, "Hello, World!\r\n", 15, &bw);
	//if (bw != 15) ...

	/* Close the file */
	f_close(&fil);

	/* Unregister work area */
	f_mount(0, "", 0);
	printf("Finish\r\n");

	while (1) {
		vTaskDelay(100);
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

int sd_do_unmount(void *parm)
{
	int res = 0;
	res = f_mount(NULL, fatfs_sd.drv, 1);
	if (res) {
		printf("UMount failed %d\r\n", res);
	} else {
		printf("UMount Successful\r\n");
	}

	return res;
}

static void sd_hot_plug_thread(void *param)
{
	int res = 0;

	//format_for_sdcard();

#ifdef ENABLE_SD_POWER_RESET
	sd_gpio_power_reset();
#endif
	sd_gpio_init();
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
fail:
	vTaskDelete(NULL);
}


void example_sd_hot_plug(void)
{
	if (xTaskCreate(sd_hot_plug_thread, ((const char *)"sd_hot_plug"), 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(sd_hot_plug) failed", __FUNCTION__);
	}
}
#endif