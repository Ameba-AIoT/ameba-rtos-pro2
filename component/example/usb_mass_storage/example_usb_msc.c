#include "FreeRTOS.h"
#include "task.h"
//#include <platform/platform_stdlib.h>
#include "platform_opts.h"

#include "osdep_service.h"
#include "msc/inc/usbd_msc.h"
#include "fatfs_ramdisk_api.h"
#include "fatfs_sdcard_api.h"
#include "fatfs_flash_api.h"
#include <disk_if/inc/flash_fatfs.h>
#include "log_service.h"
#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8735B)
#include "sdio_combine.h"
#include "sys_api.h"
#endif

//#define USB_RAM
#define USB_SD
//#define USB_FLASH

static struct msc_opts *disk_operation = NULL;
static int msc_cmd_status = 0;
#define MSC_CMD_START  0x00
#define MSC_CMD_DEINIT 0X01
void atcmd_usb_msc_init(void);
void example_mass_storage_thread(void *param)
{
	int status = 0;
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
#ifdef USB_FLASH //It only support nor flash
	int boot_sel = 0;
	boot_sel = sys_get_boot_sel();
	if (boot_sel != 0x00) { //It is not nor flash
		goto exit;
	}
#ifndef SUPPORT_USB_FLASH_MASSSTORAGE
	printf("It need to enable the flag\r\n");
	goto exit;
#endif
#endif
	_usb_init();
#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8735B)
#ifdef USB_SD
	sd_gpio_init();
	sdio_driver_init();
#endif
#ifdef USB_RAM
	fatfs_ram_init();
#endif
#ifdef USB_FLASH
	fatfs_flash_init();
#endif
#endif
	status = wait_usb_ready();
	if (status != USBD_INIT_OK) {
		if (status == USBD_NOT_ATTACHED) {
			printf("\r\n NO USB device attached\n");
		} else {
			printf("\r\n USB init fail\n");
		}
		goto exit;
	}

	disk_operation = malloc(sizeof(struct msc_opts));
	if (disk_operation == NULL) {
		printf("\r\n disk_operation malloc fail\n");
		goto exit;
	}
#ifdef USB_SD
	disk_operation->disk_init = usb_sd_init;
	disk_operation->disk_deinit = usb_sd_deinit;
	disk_operation->disk_getcapacity = usb_sd_getcapacity;
	disk_operation->disk_read = usb_sd_readblocks;
	disk_operation->disk_write = usb_sd_writeblocks;
#endif
#ifdef USB_RAM
	disk_operation->disk_init = usb_ram_init;
	disk_operation->disk_deinit = usb_ram_deinit;
	disk_operation->disk_getcapacity = usb_ram_getcapacity;
	disk_operation->disk_read = usb_ram_readblocks;
	disk_operation->disk_write = usb_ram_writeblocks;
#endif
#ifdef USB_FLASH
	disk_operation->disk_init = usb_flash_init;
	disk_operation->disk_deinit = usb_flash_deinit;
	disk_operation->disk_getcapacity = usb_flash_getcapacity;
	disk_operation->disk_read = usb_flash_readblocks;
	disk_operation->disk_write = usb_flash_writeblocks;
#endif
	// load usb mass storage driver
	status = usbd_msc_init(MSC_NBR_BUFHD, MSC_BUFLEN, disk_operation);

	if (status) {
		printf("USB MSC driver load fail.\n");
	} else {
		printf("USB MSC driver load done, Available heap [0x%x]\n", xPortGetFreeHeapSize());
	}
	atcmd_usb_msc_init();
exit:
	vTaskDelete(NULL);
}


void example_mass_storage(void)
{
	if (xTaskCreate(example_mass_storage_thread, ((const char *)"example_fatfs_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_fatfs_thread) failed", __FUNCTION__);
	}
}

void example_mass_storage_deinit(void)
{
	usbd_msc_deinit();
	extern void _usb_deinit(void);
	_usb_deinit();
	if (disk_operation) {
		free(disk_operation);
		disk_operation = NULL;
	}
}
void AMSCD(void *arg)
{
	if (msc_cmd_status == MSC_CMD_START) {
		example_mass_storage_deinit();
		msc_cmd_status = MSC_CMD_DEINIT;
	} else {
		printf("It has already deinit\r\n");
	}
}

void AMSCE(void *arg)
{
	if (msc_cmd_status == MSC_CMD_DEINIT) {
		example_mass_storage();
		msc_cmd_status = MSC_CMD_START;
	} else {
		printf("The example is running\r\n");
	}
}

log_item_t usb_msc_items[] = {
	{"MSCD", AMSCD,},
	{"MSCE", AMSCE,},
};

void atcmd_usb_msc_init(void)
{
	log_service_add_table(usb_msc_items, sizeof(usb_msc_items) / sizeof(usb_msc_items[0]));
}
