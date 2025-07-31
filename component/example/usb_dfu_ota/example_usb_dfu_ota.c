#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include "platform_opts.h"

#define _USB_ERRNO_H //Prevent redefine
#include "osdep_service.h"
#include "dfu/inc/usbd_dfu.h"

#include "fwfs.h"
#include "sys_api.h"

#include "ota_8735b.h"

#if CONFIG_LWIP_LAYER
#include <lwip_netconf.h>
#include <dhcp/dhcps.h>
#endif
#include "log_service.h"

//#define BOOT_LOADER
#define USE_CHECKSUM
//#define USE_SERIAL_MAC //Setup the serial number with Mac address
static TaskHandle_t DfuHandle = NULL;
typedef struct {
	int read_bytes;
	int update_size;
	_file_checksum file_checksum;
	uint32_t cur_fw_idx;
	uint32_t target_fw_idx;
	FILE *fp;
	struct dfu_opts dfu_cb;
	_sema dfu_rest_sema;
} dfu_operate;

static dfu_operate usb_ota_dfu;
static int dfu_cmd_status = 0;
#define DFU_CMD_START  0x00
#define DFU_CMD_DEINIT 0X01

int ota_upgrade_from_usb(unsigned char *buf, unsigned int size, int index)
{
	dfu_operate *dfu = &usb_ota_dfu;
	int ret = 0;
	int wr_status = 0;
	dfu->read_bytes = size;
	dfu->update_size += dfu->read_bytes;
#ifdef USE_CHECKSUM
	dfu->file_checksum.c[0] = buf[dfu->read_bytes - 4];
	dfu->file_checksum.c[1] = buf[dfu->read_bytes - 3];
	dfu->file_checksum.c[2] = buf[dfu->read_bytes - 2];
	dfu->file_checksum.c[3] = buf[dfu->read_bytes - 1];
#endif
	wr_status = pfw_write(dfu->fp, buf, dfu->read_bytes);
	if (wr_status < 0) {
		printf("\n\r[%s] ota flash failed", __FUNCTION__);
		goto update_ota_exit;
	}
	return 0;
update_ota_exit:
	return -1;
}

int ota_checksum_from_usb(void *parm)
{
	unsigned char *buf = malloc(4096);
	dfu_operate *dfu = &usb_ota_dfu;
	pfw_close(dfu->fp);
	int chksum = 0;
	int chklen = dfu->update_size - 4; //ota_len - 4;		// skip 4byte ota length
	void *chkfp = NULL;
#ifdef BOOT_LOADER
	chkfp = pfw_open("BL_PRI", M_RAW | M_RDWR);
	printf("boot loader\r\n");
#else
	if (dfu->cur_fw_idx == 1) {
		chkfp = pfw_open("FW2", M_RAW | M_RDWR);
		printf("FW2\r\n");
	} else {
		chkfp = pfw_open("FW1", M_RAW | M_RDWR);
		printf("FW1\r\n");
	}
#endif

	if (!chkfp) {
		goto update_ota_exit;
	}
	printf("Checksum start\r\n");
	printf("chklen %d\r\n", chklen);
	while (chklen > 0) {
		int rdlen = chklen > 4096 ? 4096 : chklen;
		pfw_read(chkfp, buf, rdlen);
		for (int i = 0; i < rdlen; i++) {
			chksum += buf[i];
		}
		chklen -= rdlen;
	}

	printf("checksum Remote %x, Flash %x\n\r", dfu->file_checksum.u, chksum);
	if (dfu->file_checksum.u != chksum) {
		pfw_seek(chkfp, 0, SEEK_SET);
		memset(buf, 0, 4096);
		pfw_write(chkfp, buf, 4096);
		pfw_close(chkfp);
		printf("Check sum is fail\r\n");
		goto update_ota_exit;
	}
	pfw_close(chkfp);
	printf("\n\r[%s] Ready to reboot\r\n", __FUNCTION__);
	osDelay(100);
	if (buf) {
		free(buf);
	}
	return 0;
update_ota_exit:
	if (buf) {
		free(buf);
	}
	return -1;
}

int ota_reset_from_usb(void *parm)
{
	printf("ota_reset_from_usb\r\n");
	dfu_operate *dfu = &usb_ota_dfu;
	rtw_up_sema(&dfu->dfu_rest_sema);
	return 0;
}

void example_usb_dfu_ota_reset_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif
	dfu_operate *dfu = &usb_ota_dfu;
	while (1) {
		if (rtw_down_sema(&dfu->dfu_rest_sema)) {
			printf("dfu reset\r\n");
			ota_platform_reset();
			while (1) {
				vTaskDelay(1000);
			}
		}
	}
	vTaskDelete(NULL);
}
void atcmd_usb_dfu_init(void);
void example_usb_dfu_ota_thread(void *param)
{

	int status = 0;
#ifdef USE_SERIAL_MAC
	unsigned char *mac = LwIP_GetMAC(0);//Get the wifi MAC address
	char buf_serial[24] = {0};
	sprintf(buf_serial, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	printf("buf_serial %s\r\n", buf_serial);
	usbd_dfu_setup_serial_number(buf_serial);
#endif

	printf("%s %s %s\r\n", __TIME__, __DATE__, __FUNCTION__);

	dfu_operate *dfu = &usb_ota_dfu;

	memset(dfu, 0, sizeof(dfu_operate));

	dfu->dfu_cb.write = ota_upgrade_from_usb;
	dfu->dfu_cb.checksum = ota_checksum_from_usb;
	dfu->dfu_cb.reset = ota_reset_from_usb;

	dfu->cur_fw_idx = hal_sys_get_ld_fw_idx();

	rtw_init_sema(&dfu->dfu_rest_sema, 0);
#ifdef BOOT_LOADER
	dfu->fp = pfw_open("BL_PRI", M_RAW | M_CREATE);
	printf("Update boot loader\r\n");
#else
	printf("fw index %d\r\n", dfu->cur_fw_idx);
	if (1 == dfu->cur_fw_idx) {
		dfu->target_fw_idx = 2;
		dfu->fp = pfw_open("FW2", M_RAW | M_CREATE);
		printf("Update fw2\r\n");
	} else if (2 == dfu->cur_fw_idx) {
		dfu->target_fw_idx = 1;
		dfu->fp = pfw_open("FW1", M_RAW | M_CREATE);
		printf("Update fw1\r\n");
	}
#endif
	if (!dfu->fp) {
		printf("Can't open the file\r\n");
		goto exit;
	}

	_usb_init();

	status = wait_usb_ready();
	if (status != USBD_INIT_OK) {
		if (status == USBD_NOT_ATTACHED) {
			printf("\r\n NO USB device attached\n");
		} else {
			printf("\r\n USB init fail\n");
		}
		goto exit;
	}

	status = usbd_dfu_init(&dfu->dfu_cb);

	if (status) {
		printf("USB DFU driver load fail.\n");
	} else {
		printf("USB DFU driver load done, Available heap [0x%x]\n", xPortGetFreeHeapSize());
	}

	if (xTaskCreate(example_usb_dfu_ota_reset_thread, ((const char *)"example_usb_dfu_ota_reset_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, &DfuHandle) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_usb_dfu_ota_reset_thread) failed", __FUNCTION__);
	}
	atcmd_usb_dfu_init();
exit:
	vTaskDelete(NULL);
}


void example_usb_dfu_ota(void)
{
	if (xTaskCreate(example_usb_dfu_ota_thread, ((const char *)"example_usb_dfu_ota_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_usb_dfu_ota_thread) failed", __FUNCTION__);
	}
}

void example_usb_dfu_ota_deinit(void)
{
	dfu_operate *dfu = &usb_ota_dfu;
	usbd_dfu_deinit();
	rtw_free_sema(&dfu->dfu_rest_sema);
	if (dfu->fp) {
		pfw_close(dfu->fp);
		dfu->fp = NULL;
	}
	vTaskDelete(DfuHandle);
	extern void _usb_deinit(void);
	_usb_deinit();
}

void ADFUD(void *arg)
{
	if (dfu_cmd_status == DFU_CMD_START) {
		example_usb_dfu_ota_deinit();
		dfu_cmd_status = DFU_CMD_DEINIT;
	} else {
		printf("It has already deinit\r\n");
	}
}

void ADFUE(void *arg)
{
	if (dfu_cmd_status == DFU_CMD_DEINIT) {
		example_usb_dfu_ota();
		dfu_cmd_status = DFU_CMD_START;
	} else {
		printf("The example is running\r\n");
	}
}

log_item_t usb_dfu_items[] = {
	{"DFUD", ADFUD,},
	{"DFUE", ADFUE,},
};

void atcmd_usb_dfu_init(void)
{
	log_service_add_table(usb_dfu_items, sizeof(usb_dfu_items) / sizeof(usb_dfu_items[0]));
}