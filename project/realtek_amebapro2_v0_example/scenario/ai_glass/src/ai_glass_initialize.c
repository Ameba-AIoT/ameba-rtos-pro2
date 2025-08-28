/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 *
 ******************************************************************************/
#include <platform_opts.h>
#include "FreeRTOS.h"
#include "task.h"
#include <platform_stdlib.h>
#include "semphr.h"
#include "device.h"
#include "serial_api.h"
#include "uart_service.h"
#include "uart_cmd.h"
#include "wlan_scenario.h"
#include "wifi_structures.h"
#include "ai_glass_initialize.h"
#include "ai_glass_media.h"
#include "media_filesystem.h"
#include "vfs.h"
#include "fatfs_sdcard_api.h"
#include "log_service.h"
#include "sliding_windows.h"
#include "mmf2_mediatime_8735b.h"
#include "mmf2_dbg.h"
#include "ai_glass_dbg.h"
#include "lwip_netconf.h"
#include "ota_8735b.h"
#include "cJSON.h"
#include "sys_api.h"
#include <device_lock.h>
#include "snand_api.h"
#include "hal_crypto.h"
#include "uart_dbg.h"
#include "ai_glass_version.h"
#include "wifi_conf.h"

// Configure for ai glass
#define ENABLE_TEST_CMD             1   // For the tester to test some hardware
#define EXTDISK_PLATFORM            VFS_INF_EMMC //VFS_INF_SD
#define UART_TX                     PA_2
#define UART_RX                     PA_3
#define UART_BAUDRATE               2000000 //115200 //2000000 //3750000 //4000000
#define POWER_DOWN_TIMEOUT          700     // 700ms
#define UART_PROTOCAL_VERSION       1

// Definition for STA mode
#define MAX_SSID_LEN                33
#define MAX_PASSWORD_LEN            65
#define PSCAN_FAST_SURVEY           0x02

// Definition for UPDATE TYPE
#define UPDATE_DEFAULT_SNAPSHOT     1
#define UPDATE_DEFAULT_RECORD       2
#define UPDATE_RECORD_TIME          3

// Definition for buffer size
#define MAX_FILENAME_SIZE           128

// Parameters for ai glass
static const char *ai_glass_disk_name = "aiglass";
static uint8_t send_response_timer_setstop = 0;

static TimerHandle_t send_response_timer = NULL;
static SemaphoreHandle_t send_response_timermutex = NULL;
static SemaphoreHandle_t video_proc_sema = NULL;
static struct msc_opts *disk_operation = NULL;
static int usb_msc_initialed = 0;

static uint8_t temp_file_name[MAX_FILENAME_SIZE] = {0};
static uint8_t temp_rfile_name[MAX_FILENAME_SIZE] = {0};

// For OTA progress status
volatile uint8_t bt_progress;
volatile uint8_t cancel_bt_upgrade = 0;
volatile uint8_t cancel_wifi_upgrade = 0;

volatile int critical_process_started = 0;

// Funtion Prototype
static void ai_glass_init_external_disk(void);
static void ai_glass_deinit_external_disk(void);
static void ai_glass_init_ram_disk(void);
void ai_glass_log_init(void);

static char version_str[16] = {0};
static UpgradeInfo info;

static uint8_t g_current_wifi_mode = 0;
static int dual_snapshot = 0;
// These functions are for testing ai glass with mass storage
#include "usb.h"
#include "msc/inc/usbd_msc_config.h"
#include "msc/inc/usbd_msc.h"
#include "fatfs_ramdisk_api.h"
static int usb_msc_device_init(void)
{
	return 0;
}
static int usb_msc_device_deinit(void)
{
	return 0;
}

static void aiglass_mass_storage_init(void)
{
	if (usb_msc_initialed == 0) {
		ai_glass_init_external_disk();
		int status = 0;
		_usb_init();

		status = wait_usb_ready();
		if (status != USBD_INIT_OK) {
			if (status == USBD_NOT_ATTACHED) {
				AI_GLASS_WARN("NO USB device attached\r\n");
			} else {
				AI_GLASS_WARN("USB init fail\r\n");
			}
			goto exit;
		}

		if (disk_operation == NULL) {
			disk_operation = malloc(sizeof(struct msc_opts));
		}
		if (disk_operation == NULL) {
			AI_GLASS_ERR("disk_operation malloc fail\r\n");
			extern void _usb_deinit(void);
			_usb_deinit();
			goto exit;
		}

		disk_operation->disk_init = usb_msc_device_init;
		disk_operation->disk_deinit = usb_msc_device_deinit;
#if EXTDISK_PLATFORM == VFS_INF_RAM
		disk_operation->disk_getcapacity = usb_ram_getcapacity;
		disk_operation->disk_read = usb_ram_readblocks;
		disk_operation->disk_write = usb_ram_writeblocks;
#else
		disk_operation->disk_getcapacity = usb_sd_getcapacity;
		disk_operation->disk_read = usb_sd_readblocks;
		disk_operation->disk_write = usb_sd_writeblocks;
#endif

		// load usb mass storage driver
		status = usbd_msc_init(MSC_NBR_BUFHD, MSC_BUFLEN, disk_operation);

exit:
		if (status) {
			AI_GLASS_ERR("USB MSC driver load fail.\r\n");
			usb_msc_initialed = 0;
		} else {
			AI_GLASS_INFO("USB MSC driver load done, Available heap [0x%x]\r\n", xPortGetFreeHeapSize());
			usb_msc_initialed = 1;
		}
	}
}

static void aiglass_mass_storage_deinit(void)
{
	if (usb_msc_initialed == 1) {
		usbd_msc_deinit();
		extern void _usb_deinit(void);
		_usb_deinit();
		usb_msc_initialed = 0;
	}
}

static void ai_glass_init_external_disk(void)
{
	if (!extdisk_get_init_status()) {
		extdisk_filesystem_init(ai_glass_disk_name, VFS_FATFS, EXTDISK_PLATFORM);
	}
}

static void ai_glass_deinit_external_disk(void)
{
	if (extdisk_get_init_status()) {
		extdisk_filesystem_deinit(ai_glass_disk_name, VFS_FATFS, EXTDISK_PLATFORM);
	}
}

static void ai_glass_init_ram_disk(void)
{
	if (!ramdisk_get_init_status()) {
		ramdisk_filesystem_init("ai_ram");
	}
}

int ai_glass_disk_reformat(void)
{
	ai_glass_init_external_disk();
	AI_GLASS_MSG("Format disk to FAT32\r\n");
	int ret = vfs_user_format(ai_glass_disk_name, VFS_FATFS, EXTDISK_PLATFORM);
	if (ret == FR_OK) {
		AI_GLASS_MSG("format successfully\r\n");
		ai_glass_deinit_external_disk();
		return AI_GLASS_CMD_COMPLETE;
	} else {
		AI_GLASS_ERR("format failed %d\r\n", ret);
		return AI_GLASS_PROC_FAIL;
	}
}

typedef struct snapshot_pkt_s {
	uint8_t     status;
	uint8_t     version;
	uint8_t     q_vlaue;
	float       ROIX_TL;
	float       ROIY_TL;
	float       ROIX_BR;
	float       ROIY_BR;
	uint16_t    RESIZE_W;
	uint16_t    RESIZE_H;
} snapshot_pkt_t;

static void parser_snapshot_pkt2param(ai_glass_snapshot_param_t *snap_buf, uint8_t *raw_buf)
{
	snapshot_pkt_t aisnap_buf = {0};
	uint32_t temp_data = 0;
	if (snap_buf) {
		aisnap_buf.status = raw_buf[0];
		aisnap_buf.version = raw_buf[1];
		aisnap_buf.q_vlaue = raw_buf[2];
		temp_data = raw_buf[3] | (raw_buf[4] << 8) | (raw_buf[5] << 16) | (raw_buf[6] << 24);
		memcpy(&(aisnap_buf.ROIX_TL), &temp_data, sizeof(uint32_t));
		temp_data = raw_buf[7] | (raw_buf[8] << 8) | (raw_buf[9] << 16) | (raw_buf[10] << 24);
		memcpy(&(aisnap_buf.ROIY_TL), &temp_data, sizeof(uint32_t));
		temp_data = raw_buf[11] | (raw_buf[12] << 8) | (raw_buf[13] << 16) | (raw_buf[14] << 24);
		memcpy(&(aisnap_buf.ROIX_BR), &temp_data, sizeof(uint32_t));
		temp_data = raw_buf[15] | (raw_buf[16] << 8) | (raw_buf[17] << 16) | (raw_buf[18] << 24);
		memcpy(&(aisnap_buf.ROIY_BR), &temp_data, sizeof(uint32_t));
		aisnap_buf.RESIZE_W = raw_buf[19] | (raw_buf[20] << 8);
		aisnap_buf.RESIZE_H = raw_buf[21] | (raw_buf[22] << 8);

		AI_GLASS_MSG("AI_snapshot_parameter\r\n");

		//1 additional status parameter Main changes
		AI_GLASS_MSG("status = %u\r\n", aisnap_buf.status);
		AI_GLASS_MSG("version = %u\r\n", aisnap_buf.version);
		AI_GLASS_MSG("q vlaue = %u\r\n", aisnap_buf.q_vlaue);
		AI_GLASS_MSG("ROIX_TL = %f\r\n", aisnap_buf.ROIX_TL);
		AI_GLASS_MSG("ROIY_TL = %f\r\n", aisnap_buf.ROIY_TL);
		AI_GLASS_MSG("ROIX_BR = %f\r\n", aisnap_buf.ROIX_BR);
		AI_GLASS_MSG("ROIY_BR = %f\r\n", aisnap_buf.ROIY_BR);
		AI_GLASS_MSG("RESIZE_W = %u\r\n", aisnap_buf.RESIZE_W);
		AI_GLASS_MSG("RESIZE_H = %u\r\n", aisnap_buf.RESIZE_H);

		snap_buf->width = aisnap_buf.RESIZE_W;
		snap_buf->height = aisnap_buf.RESIZE_H;
		snap_buf->jpeg_qlevel = aisnap_buf.q_vlaue;
		snap_buf->roi.xmin = (uint32_t)(aisnap_buf.ROIX_TL * sensor_params[USE_SENSOR].sensor_width);
		snap_buf->roi.ymin = (uint32_t)(aisnap_buf.ROIY_TL * sensor_params[USE_SENSOR].sensor_height);
		snap_buf->roi.xmax = (uint32_t)(aisnap_buf.ROIX_BR * sensor_params[USE_SENSOR].sensor_width);
		snap_buf->roi.ymax = (uint32_t)(aisnap_buf.ROIY_BR * sensor_params[USE_SENSOR].sensor_height);
		snap_buf->status = aisnap_buf.status;
	}
}

//Check OTA files exists
static int ota_file_exists(char *version_str, char ota_versions[2][16])
{
#define OTA_FILE_WIFI_PREFIX "wifi_ota_v"
#define OTA_FILE_BT_PREFIX   "bt_ota_v"
#define OTA_FILE_EXTENSION   ".bin"

	ai_glass_init_external_disk();

	if (extdisk_get_init_status() != 1) {
		AI_GLASS_ERR("OTA check file: External disk is not initialized.\n");
		return 0;
	}

	uint16_t file_count = 0;
	const char *extensions[] = {OTA_FILE_EXTENSION};

	// Get file list in JSON format
	cJSON *file_list = extdisk_get_filelist("/", &file_count, extensions, 1, NULL);

	if (!file_list) {
		AI_GLASS_ERR("OTA check file: Unable to retrieve file list.\n");
		return 0;
	}

	printf("Raw JSON response: %s\n", cJSON_Print(file_list)); // Debugging

	int found_wifi = 0, found_bt = 0;

	// Extract "contents" array from JSON
	cJSON *contents = cJSON_GetObjectItem(file_list, "contents");
	if (!cJSON_IsArray(contents)) {
		AI_GLASS_ERR("OTA check file: 'contents' array missing or invalid.\n");
		cJSON_Delete(file_list);
		return 0;
	}

	// Iterate over the JSON "contents" array
	cJSON *file_item = NULL;
	cJSON_ArrayForEach(file_item, contents) {
		cJSON *name_obj = cJSON_GetObjectItem(file_item, "name");
		if (!cJSON_IsString(name_obj)) {
			continue;
		}

		char *filename = name_obj->valuestring;
		AI_GLASS_MSG("Found file: %s\r\n", filename); // Debugging

		if (!found_wifi && strncmp(filename, OTA_FILE_WIFI_PREFIX, strlen(OTA_FILE_WIFI_PREFIX)) == 0) {
			char *start = filename + strlen(OTA_FILE_WIFI_PREFIX);
			char *end = strstr(start, ".bin");
			if (end) {
				size_t len = end - start;
				if (len < 16) {
					strncpy(ota_versions[0], start, len);
					ota_versions[0][len] = '\0';
					if (strcmp(ota_versions[0], version_str) == 0) {
						found_wifi = 1;
					}
				}
			}
		}

		if (!found_bt && strncmp(filename, OTA_FILE_BT_PREFIX, strlen(OTA_FILE_BT_PREFIX)) == 0) {
			char *start = filename + strlen(OTA_FILE_BT_PREFIX);
			char *end = strstr(start, ".bin");
			if (end) {
				size_t len = end - start;
				if (len < 16) {
					strncpy(ota_versions[1], start, len);
					ota_versions[1][len] = '\0';
					if (strcmp(ota_versions[1], version_str) == 0) {
						found_bt = 1;
					}
				}
			}
		}

		if (found_wifi || found_bt) {
			break; // Only break when both found
		}
	}

	cJSON_Delete(file_list);

	if (!found_wifi && !found_bt) {
		AI_GLASS_ERR("OTA check file: Missing OTA file (WiFi: %d, BT: %d)\n", found_wifi, found_bt);
		return 0;

	} else if (strcmp(ota_versions[0], version_str) != 0 && strcmp(ota_versions[1], version_str) != 0) {
		// Both files found, but wrong version
		AI_GLASS_ERR("OTA check file: Wrong OTA version found (expected: %s, found: %s / %s)\n", version_str, ota_versions[0], ota_versions[1]);
		return 0;

	} else {
		// Both files found and versions match
		return 1;
	}
}

// Check OTA files exists and get OTA filename from EMMC
static int ota_filenames_exists(char ota_filenames[3][32], char *version_str, uint8_t mode)
{
#define OTA_FILE_WIFI_PREFIX "wifi_ota_v"
#define OTA_FILE_BT_PREFIX   "bt_ota_v"
#define OTA_FILE_BOOT_PREFIX "boot_ota_v"
#define OTA_FILE_EXTENSION   ".bin"

	ai_glass_init_external_disk();

	if (extdisk_get_init_status() != 1) {
		AI_GLASS_ERR("OTA get filename: External disk is not initialized.\n");
		return 0;
	}

	if (mode != 0x02 && mode != 0x03 && mode != 0x04) {
		AI_GLASS_ERR("OTA get filename: Unsupported mode 0x%02X\n", mode);
		return 0;
	}

	uint16_t file_count = 0;
	const char *extensions[] = {OTA_FILE_EXTENSION};

	// Get file list in JSON format
	cJSON *file_list = extdisk_get_filelist("/", &file_count, extensions, 1, NULL);

	if (!file_list) {
		AI_GLASS_ERR("OTA get filename: Unable to retrieve file list.\r\n");
		return 0;
	}

	AI_GLASS_MSG("Raw JSON response: %s\n", cJSON_Print(file_list)); // Debugging

	int found_wifi = 0, found_bt = 0, found_boot = 0;

	// Extract "contents" array from JSON
	cJSON *contents = cJSON_GetObjectItem(file_list, "contents");
	if (!cJSON_IsArray(contents)) {
		AI_GLASS_ERR("OTA get filename 'contents' array missing or invalid.\r\n");
		cJSON_Delete(file_list);
		return 0;
	}

	// Iterate over the JSON "contents" array
	cJSON *file_item = NULL;
	cJSON_ArrayForEach(file_item, contents) {
		cJSON *name_obj = cJSON_GetObjectItem(file_item, "name");
		if (!cJSON_IsString(name_obj)) {
			continue;
		}

		char *filename = name_obj->valuestring;
		AI_GLASS_MSG("Found file: %s\r\n", filename); // Debugging

		if ((mode == 0x02 || mode == 0x03) && !found_wifi && strncmp(filename, OTA_FILE_WIFI_PREFIX, strlen(OTA_FILE_WIFI_PREFIX)) == 0) {
			char *start = filename + strlen(OTA_FILE_WIFI_PREFIX);
			char *end = strstr(start, ".bin");
			if (end) {
				size_t len = end - start;
				if (len < 16) {
					char ver_buf[16] = {0};
					strncpy(ver_buf, start, len);
					ver_buf[len] = '\0';

					if (strcmp(ver_buf, version_str) == 0) {
						found_wifi = 1;
						strncpy(ota_filenames[0], filename, 32);
						AI_GLASS_MSG("Found WiFi OTA filename: %s\n", ota_filenames[0]);
					}
				}
			}
		}

		if ((mode == 0x04) && !found_bt && strncmp(filename, OTA_FILE_BT_PREFIX, strlen(OTA_FILE_BT_PREFIX)) == 0) {
			char *start = filename + strlen(OTA_FILE_BT_PREFIX);
			char *end = strstr(start, ".bin");
			if (end) {
				size_t len = end - start;
				if (len < 16) {
					char ver_buf[16] = {0};
					strncpy(ver_buf, start, len);
					ver_buf[len] = '\0';

					if (strcmp(ver_buf, version_str) == 0) {
						found_bt = 1;
						strncpy(ota_filenames[1], filename, 32);
						AI_GLASS_MSG("Found BT OTA filename: %s\n", ota_filenames[1]);
					}
				}
			}
		}

		if ((mode == 0x03) && !found_boot && strncmp(filename, OTA_FILE_BOOT_PREFIX, strlen(OTA_FILE_BOOT_PREFIX)) == 0) {
			char *start = filename + strlen(OTA_FILE_BOOT_PREFIX);
			char *end = strstr(start, ".bin");
			if (end) {
				size_t len = end - start;
				if (len < 16) {
					char ver_buf[16] = {0};
					strncpy(ver_buf, start, len);
					ver_buf[len] = '\0';

					if (strcmp(ver_buf, version_str) == 0) {
						found_boot = 1;
						strncpy(ota_filenames[3], filename, 32);
						AI_GLASS_MSG("Found Bootloader OTA filename: %s\n", ota_filenames[3]);
					}
				}
			}
		}

		if (mode == 0x02) {
			if (found_wifi) {
				break;  // Break when expected file is found based on mode
			}
		} else if (mode == 0x03) {
			if ((found_wifi && found_boot)) {
				break;  // Break when expected file is found based on mode
			}
		} else if (mode == 0x04) {
			if (found_bt) {
				break;  // Break when expected file is found based on mode
			}
		}
	}

	cJSON_Delete(file_list);

	if ((mode == 0x02 && found_wifi) || (mode == 0x04 && found_bt) || (mode == 0x03 && found_wifi && found_boot)) {
		return 1; // found
	} else {
		AI_GLASS_ERR("OTA get filename: Required OTA file missing (mode=0x%02X, WiFi: %d, BT: %d)\r\n", mode, found_wifi, found_bt);
		return 0; // not found
	}

}

static int clear_ota_signature(void)
{
	uint8_t cur_fw_idx = 0;
	uint8_t boot_sel = -1;

	cur_fw_idx = hal_sys_get_ld_fw_idx();
	if ((1 != cur_fw_idx) && (2 != cur_fw_idx)) {
		AI_GLASS_ERR("\n\rcurrent fw index is wrong %d \n\r", cur_fw_idx);
		return 0;
	}

	boot_sel = sys_get_boot_sel();
	if (0 == boot_sel) {
		// boot from NOR flash

		flash_t flash;
		uint8_t label_init_value[8] = {0x52, 0x54, 0x4c, 0x38, 0x37, 0x33, 0x35, 0x42};
		uint8_t next_fw_label[8] = {0};
		uint32_t cur_fw_addr = 0, next_fw_addr = 0;
		uint8_t *pbuf = NULL;
		uint32_t buf_size = 4096;

		device_mutex_lock(RT_DEV_LOCK_FLASH);
		if (1 == cur_fw_idx) {
			// fw1 record in partition table
			flash_read_word(&flash, 0x2060, &cur_fw_addr);
			// fw2 record in partition table
			flash_read_word(&flash, 0x2080, &next_fw_addr);
		} else if (2 == cur_fw_idx) {
			// fw2 record in partition table
			flash_read_word(&flash, 0x2080, &cur_fw_addr);
			// fw1 record in partition table
			flash_read_word(&flash, 0x2060, &next_fw_addr);
		}
		flash_stream_read(&flash, next_fw_addr, 8, next_fw_label);
		device_mutex_unlock(RT_DEV_LOCK_FLASH);

		if (0 != memcmp(next_fw_label, label_init_value, 8)) {
			AI_GLASS_ERR("\n\rOnly one valid fw, no fw to clear");
			return 0;
		}

		//erase current FW signature to make it boot from another FW image
		AI_GLASS_MSG("\n\rcurrent FW addr = 0x%08X", cur_fw_addr);

		pbuf = malloc(buf_size);
		if (!pbuf) {
			AI_GLASS_ERR("\n\rAllocate buf fail");
			return 0;
		}

		// need to enter critical section to prevent executing the XIP code at first sector after we erase it.
		device_mutex_lock(RT_DEV_LOCK_FLASH);
		flash_stream_read(&flash, cur_fw_addr, buf_size, pbuf);
		// NOT the first byte of ota signature to make it invalid
		pbuf[0] = ~(pbuf[0]);
		flash_erase_sector(&flash, cur_fw_addr);
		flash_burst_write(&flash, cur_fw_addr, buf_size, pbuf);
		device_mutex_unlock(RT_DEV_LOCK_FLASH);

		free(pbuf);
	} else if (1 == boot_sel) {
		// boot from NAND flash

		//uint8_t partition_data[2112] __attribute__((aligned(32)));
		//uint8_t data_r[2112] __attribute__((aligned(32)));
		uint8_t *partition_data;
		uint8_t *data_r;
		partition_data = malloc(2112);
		data_r = malloc(2112);
		uint32_t crc_out = 0;
		uint32_t crypto_ret;
		int update_partition_table = 0;
		int partition_start_block = 16 ; //B-cut:20

		if (IS_CUT_B(hal_sys_get_rom_ver())) {
			partition_start_block = 20 ; //B-cut:20
		}

		snand_t flash;
		snand_init(&flash);
		snand_global_unlock();


		//read partition_table block16-23
		for (int i = partition_start_block; i < 24; i++) {
			snand_page_read(&flash, i * 64, 2048 + 4, &partition_data[0]);
			if ((partition_data[2048] == 0xff) && (partition_data[2049] == 0xc4)) {
				break;
			}
		}

		if (1 == cur_fw_idx) {
			for (int i = 0; i < 16; i++) {
				if ((partition_data[i * 128] == 0x87) && (partition_data[i * 128 + 1] == 0xff) && (partition_data[i * 128 + 2] == 0x35) &&
					(partition_data[i * 128 + 3] == 0xff) && (partition_data[i * 128 + 4] == 0xc8) && (partition_data[i * 128 + 5] == 0xb9)) {
					AI_GLASS_ERR("partition_table FW2 type_id is valid \n\r");
					update_partition_table = 1;
				}
			}
			if (update_partition_table == 1) {
				for (int i = 0; i < 16; i++) {
					if ((partition_data[i * 128] == 0x87) && (partition_data[i * 128 + 1] == 0xff) && (partition_data[i * 128 + 2] == 0x35) &&
						(partition_data[i * 128 + 3] == 0xff) && (partition_data[i * 128 + 4] == 0xc7) && (partition_data[i * 128 + 5] == 0xc1)) {
						AI_GLASS_ERR("clear partition_table FW1 magic_num \n\r");
						partition_data[i * 128] = 0x0; //0x87 to 0x0
						partition_data[i * 128 + 2] = 0x0; //0x35 to 0x0
					}
				}
			}
		} else if (2 == cur_fw_idx) {
			for (int i = 0; i < 16; i++) {
				if ((partition_data[i * 128] == 0x87) && (partition_data[i * 128 + 1] == 0xff) && (partition_data[i * 128 + 2] == 0x35) &&
					(partition_data[i * 128 + 3] == 0xff) && (partition_data[i * 128 + 4] == 0xc7) && (partition_data[i * 128 + 5] == 0xc1)) {
					AI_GLASS_ERR("partition_table FW1 type_id is valid \n\r");
					update_partition_table = 1;
				}
			}
			if (update_partition_table == 1) {
				for (int i = 0; i < 16; i++) {
					if ((partition_data[i * 128] == 0x87) && (partition_data[i * 128 + 1] == 0xff) && (partition_data[i * 128 + 2] == 0x35) &&
						(partition_data[i * 128 + 3] == 0xff) && (partition_data[i * 128 + 4] == 0xc8) && (partition_data[i * 128 + 5] == 0xb9)) {
						AI_GLASS_ERR("clear partition_table FW2 magic_num \n\r");
						partition_data[i * 128] = 0x0; //0x87 to 0x0
						partition_data[i * 128 + 2] = 0x0; //0x35 to 0x0
					}
				}
			}
		}

		//update partition table CRC16
		if (update_partition_table == 1) {
			crypto_ret = hal_crypto_engine_init();
			if (crypto_ret != SUCCESS) {
				AI_GLASS_ERR("Crypto Init Failed!%d\r\n", crypto_ret);
				return 0;
			}
			crypto_ret =  hal_crypto_crc16_division(partition_data, 2048, &crc_out);
			if (crypto_ret != SUCCESS) {
				AI_GLASS_ERR("CRC failed\r\n");
				// ignore error and go-on
				return 0;
			}

			AI_GLASS_MSG("crc_out = 0x%x \n\r", crc_out);
			partition_data[2050] = (uint8_t)(crc_out & 0xff);
			partition_data[2051] = (uint8_t)(crc_out >> 8);
		}

		//update partition table block16-23
		if (update_partition_table == 1) {
			int success = 0;
			int fail = 0;
			for (int i = partition_start_block; i < 24; i++) {
				fail = 0;
				snand_erase_block(&flash, i * 64);
				snand_page_write(&flash, i * 64, 2048 + 4, &partition_data[0]);
				snand_page_read(&flash, i * 64, 2048 + 4, &data_r[0]);
				if (memcmp(partition_data, data_r, (2048 + 4)) != 0) {
					AI_GLASS_ERR("bolck %d write fail! \n\r", i);
					fail = 1;
					snand_erase_block(&flash, i * 64);
					data_r[2048] = 0;
					snand_page_write(&flash, i * 64, 2048 + 4, &data_r[0]);
				}
				if (fail == 0) {
					success = success + 1;
				}
				if (success == 2) {
					break;
				}
			}

		}
		free(partition_data);
		free(data_r);

	}

	AI_GLASS_MSG("\n\rClear OTA signature success.");
	return 1;
}


//8430
static void ai_glass_get_set_sys_upgrade(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_TX_OPC_CMD_TRANSFER_UPGRADE_DATA\r\n");

	deinitial_media();

	critical_process_started = 1;

	info = uart_parser_version_and_upgradetype(param);

	AI_GLASS_INFO("Upgrade type: %u, Version: %u.%u.%u.%u\n",
				  info.upgradetype,
				  info.version[0], info.version[1], info.version[2], info.version[3]);

	uint8_t status = AI_GLASS_CMD_COMPLETE;
	uart_resp_request_sys_upgrade(status);
	AI_GLASS_INFO("After 8430 CMD acknowledgement\r\n");

	if (info.upgradetype == 0x02) {
		AI_GLASS_INFO("Start WiFI OTA\r\n");

		// Convert received version to a string
		snprintf(version_str, sizeof(version_str), "%u.%u.%u.%u",
				 info.version[0], info.version[1],
				 info.version[2], info.version[3]);

		AI_GLASS_INFO("WIFI version to be upgrade to: %s\r\n", version_str);

		char ota_filenames[3][32] = {0};

		if (ota_filenames_exists(ota_filenames, version_str, info.upgradetype)) {

			char full_wifi_path[64];

			sprintf(full_wifi_path, "%s:/%s", ai_glass_disk_name, ota_filenames[0]);

			int ret = -1;
			ret = ext_storage_update_ota(full_wifi_path);
			if (!ret) {
				AI_GLASS_MSG("\n\r Ready to reboot\n");
				// uart_resp_request_sys_upgrade(status);
				ota_platform_reset();
			} else {
				AI_GLASS_ERR("\n\r OTA Process Failed\n");
				status = AI_GLASS_OTA_PROCESS_FAILED;
				uart_resp_request_sys_upgrade(status);
			}

		} else {
			AI_GLASS_ERR("OTA file name not found.\n");
			status = AI_GLASS_OTA_FILE_NOT_EXISTED;
			uart_resp_request_sys_upgrade(status);
		}
	}

	else if (info.upgradetype == 0x04) {
		AI_GLASS_INFO("Start BT OTA\r\n");

		// Convert received version to a string
		snprintf(version_str, sizeof(version_str), "%u.%u.%u.%u",
				 info.version[0], info.version[1],
				 info.version[2], info.version[3]);

		AI_GLASS_INFO("BT version to be upgrade to: %s\r\n", version_str);

		status = AI_GLASS_CMD_COMPLETE;
		uart_resp_start_bt_soc_fw_upgrade_ack(status);
		AI_GLASS_INFO("Send 631 CMD, waiting BT response of 631.\r\n");
	}

	else if (info.upgradetype == 0x03) {
		AI_GLASS_INFO("Start WIFI Bootloader and WIFI OTA\r\n");

		// Convert received version to a string
		snprintf(version_str, sizeof(version_str), "%u.%u.%u.%u",
				 info.version[0], info.version[1],
				 info.version[2], info.version[3]);

		AI_GLASS_INFO("Bootloader and WIFI version to be upgrade to: %s\r\n", version_str);

		char ota_filenames[3][32] = {0};

		if (ota_filenames_exists(ota_filenames, version_str, info.upgradetype)) {

			char boot_wifi_path[64];

			sprintf(boot_wifi_path, "%s:/%s", ai_glass_disk_name, ota_filenames[3]);

			int ret = -1;
			ret = ext_storage_update_boot_ota(boot_wifi_path);
			if (!ret) {
				AI_GLASS_MSG("\n\r Bootloader OTA done. Continue to upgrade wifi firmware...\r\n");
				char full_wifi_path[64];

				sprintf(full_wifi_path, "%s:/%s", ai_glass_disk_name, ota_filenames[0]);

				int ret = -1;
				ret = ext_storage_update_ota(full_wifi_path);
				if (!ret) {
					AI_GLASS_MSG("\n\r Ready to reboot\n");
					// uart_resp_request_sys_upgrade(status);
					ota_platform_reset();
				} else {
					AI_GLASS_ERR("\n\r OTA Wifi Firmware Process Failed\n");
					status = AI_GLASS_OTA_PROCESS_FAILED;
					uart_resp_request_sys_upgrade(status);
				}
			} else {
				AI_GLASS_ERR("\n\r OTA Bootloader Process Failed\n");
				status = AI_GLASS_OTA_PROCESS_FAILED;
				uart_resp_request_sys_upgrade(status);
			}
		} else {
			AI_GLASS_ERR("OTA file name not found.\n");
			status = AI_GLASS_OTA_FILE_NOT_EXISTED;
			uart_resp_request_sys_upgrade(status);
		}
	}

	AI_GLASS_INFO("end of UART_TX_OPC_CMD_TRANSFER_UPGRADE_DATA\r\n");

}

// 631
static void ai_glass_resp_bt_fw_upgrade(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_TX_OPC_CMD_START_BT_SOC_FW_UPGRADE\r\n");

	cancel_bt_upgrade = 0;  // Reset cancel flag on entry

	int packet_count = 0;

	uint32_t sendtime = mm_read_mediatime_ms();
	ai_glass_init_external_disk();
	if (extdisk_get_init_status() != 1) {
		AI_GLASS_ERR("Error: External disk is not initialized.\n");
		return;
	}

	wifi_off();
	vTaskDelay(20);

	AI_GLASS_INFO("Disabled WIFI\r\n");

	AI_GLASS_INFO("Sending bluetooth binary via UART...\r\n");

	char ota_filenames[3][32] = {0};
	if (ota_filenames_exists(ota_filenames, version_str, info.upgradetype)) {

		char full_bt_path[64];
		sprintf(full_bt_path, "/%s", ota_filenames[1]);

		// Currently using a hardcoded data_buffer size of 1541 bytes
		uint8_t data_buffer[1541] = {0};

		// Open the OTA file from eMMC
		FILE *ota_file = extdisk_fopen(full_bt_path, "rb");
		if (!ota_file) {
			AI_GLASS_ERR("Error: Failed to open bluetooth binary on eMMC!\n");
			return;
		}

		uint16_t tmp_uart_pic_size = uart_service_get_pic_size() - EMPTY_PACKET_LEN;
		uint16_t data_length = 0;
#if UPDATE_UPGRADE_PROGRESS_TO_8773
		int total_bytes_sent = 0;
		uart_resp_get_sys_upgrade((uint8_t) 2, (uint8_t) 0);
		extdisk_fseek(ota_file, 0, SEEK_END);
		int file_size = extdisk_ftell(ota_file);
		AI_GLASS_MSG("BT FW File_size: %d\r\n", file_size);
		extdisk_fseek(ota_file, 0, SEEK_SET);
#endif

		while (1) {
			if (cancel_bt_upgrade) {
				AI_GLASS_INFO("BT upgrade cancelled by command!\r\n");
				bt_progress = 0;
				packet_count = 0;
#if UPDATE_UPGRADE_PROGRESS_TO_8773
				AI_GLASS_INFO("FW rollback...\r\n");
				if (clear_ota_signature()) {
					uint8_t status = AI_GLASS_CMD_COMPLETE;
					uart_resp_set_wifi_fw_rollback(status);
					uart_resp_cancel_sys_upgrade(status);
					AI_GLASS_INFO("FW rollback done\r\n");
				} else {
					uint8_t status = AI_GLASS_SCEN_ERR;
					uart_resp_set_wifi_fw_rollback(status);
					uart_resp_cancel_sys_upgrade(status);
					AI_GLASS_ERR("FW rollback failed\r\n");
				}
#endif
				break;
			}

			memset(data_buffer, 0, tmp_uart_pic_size);
			int bytesRead = extdisk_fread(data_buffer, 1, tmp_uart_pic_size, ota_file);

			if (bytesRead <= 0) {
				if (ferror(ota_file)) {
					AI_GLASS_ERR("Error: File read failed!\n");
				} else {
					AI_GLASS_INFO("End of file reached.\n");
				}
				break;
			}
			data_length = bytesRead;

#if UPDATE_UPGRADE_PROGRESS_TO_8773
			total_bytes_sent += data_length;
			packet_count++;
			AI_GLASS_INFO("Total_bytes_sent: %u\r\n", total_bytes_sent);

			static uint8_t last_bt_progress = 0xFF;

			if (file_size > 0) {
				bt_progress = (uint8_t)(((total_bytes_sent * 100) / file_size));
				AI_GLASS_INFO("BT progress: %u\r\n", bt_progress);
				if (bt_progress > 99) {
					bt_progress = 99;
				}
			}
			// Send update only when progress changes
			if (bt_progress != last_bt_progress) {
				last_bt_progress = bt_progress;
				uart_resp_get_sys_upgrade((uint8_t) 2, bt_progress);
				AI_GLASS_INFO("BT progress update: %u%% after %d packets\r\n", bt_progress, packet_count);
			}
#endif

			AI_GLASS_MSG("[8735(2) Sending] Data Length: %d bytes, Data[0-2]: %02X %02X %02X\n", data_length, data_buffer[0], data_buffer[1], data_buffer[2]);

			uart_resp_transfer_upgrade_data(data_buffer, data_length);
			vTaskDelay(pdMS_TO_TICKS(10));
			// Prevent packet loss by adding delay between UART transmissions.
			if (extdisk_feof(ota_file)) {
				uart_resp_get_sys_upgrade((uint8_t) 2, (uint8_t) 100);
				AI_GLASS_MSG("Send BT progress status 100\r\n");
				break;
			}
		}
		extdisk_fclose(ota_file);
		// Add delay to ensure all UART data packets are fully sent before starting BT SoC OTA.
		// Without this delay, the last packet may be lost, causing an incomplete OTA binary and OTA failure.
		vTaskDelay(5000);
		AI_GLASS_INFO("Firmware transfer completed.\r\n");
		uint32_t endtime = mm_read_mediatime_ms();
		uint32_t transfertime = endtime - sendtime;
		uart_resp_finish_bt_soc_fw_upgrade();
		bt_progress = 0;
		AI_GLASS_MSG("End of START_BT_SOC_FW_UPGRADE_RESP = %lu\r\n", transfertime);

	}
	AI_GLASS_INFO("end of UART_TX_OPC_CMD_START_BT_SOC_FW_UPGRADE\r\n");
	critical_process_started = 0;
}

// UART_TX_OPC_CMD_FINISH_BT_SOC_FW_UPGRADE 633
static void ai_glass_resp_bt_fw_finish(uartcmdpacket_t *param)
{
	critical_process_started = 1;
	AI_GLASS_INFO("get UART_TX_OPC_CMD_FINISH_BT_SOC_FW_UPGRADE\r\n");
	if (extdisk_delete_bin_files()) {
		AI_GLASS_MSG("Delete OTA files\r\n");
	}
	AI_GLASS_INFO("end of UART_TX_OPC_CMD_FINISH_BT_SOC_FW_UPGRADE\r\n");
	critical_process_started = 0;

}

// For UART_RX_OPC_CMD_SET_WIFI_FW_ROLLBACK 8415
static void ai_glass_wifi_fw_rollback(uartcmdpacket_t *param)
{
	critical_process_started = 1;
	AI_GLASS_INFO("get UART_RX_OPC_CMD_SET_WIFI_FW_ROLLBACK\r\n");

	if (clear_ota_signature()) {
		uint8_t status = AI_GLASS_CMD_COMPLETE;

		uart_resp_set_wifi_fw_rollback(status);
	}
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_SET_WIFI_FW_ROLLBACK\r\n");
	//Reboot (or may let BT_SoC to control the power)
	critical_process_started = 0;
	ota_platform_reset();


}

static void ai_glass_get_query_info(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_QUERY_INFO\r\n");
	uart_resp_get_query_info(param);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_QUERY_INFO\r\n");
}

static void ai_glass_get_power_down(uartcmdpacket_t *param)
{
	uint8_t result = AI_GLASS_CMD_COMPLETE;
	AI_GLASS_INFO("get UART_RX_OPC_CMD_POWER_DOWN %lu\r\n", mm_read_mediatime_ms());
	// Wait until the video is down
	if (xSemaphoreTake(video_proc_sema, POWER_DOWN_TIMEOUT) != pdTRUE) {
		AI_GLASS_WARN("AI glass is snapshot or record, power down fail %lu\r\n", mm_read_mediatime_ms());
		result = AI_GLASS_BUSY;
		uart_resp_get_power_down(param, result);
		goto endofpowerdown;
	}

	if (critical_process_started == 1) {
		result = AI_GLASS_BUSY;
		uart_resp_get_power_down(param, result);
		goto endofpowerdown;
	}
	int ret = 0;
	wifi_disable_ap_mode();
	// Save filelist to EMMC
	ai_glass_init_external_disk();
	ret = extdisk_save_file_cntlist();
	AI_GLASS_MSG("Save FILE Cnt List status: %d, %lu\r\n", ret, mm_read_mediatime_ms());
	// Todo: get power down command
	uart_resp_get_power_down(param, result);
	xSemaphoreGive(video_proc_sema);
endofpowerdown:

	AI_GLASS_INFO("end of UART_RX_OPC_CMD_POWER_DOWN %lu\r\n", mm_read_mediatime_ms());
}

static void ai_glass_get_power_state(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_POWER_STATE\r\n");
	uint8_t power_result = 0;
	int wifi_stat = wifi_get_connect_status();
	switch (wifi_stat) {
	case WLAN_STAT_IDLE:
		power_result = UART_PWR_NORMAL;
		break;
	case WLAN_STAT_HTTP_IDLE:
		power_result = UART_PWR_APON;
		break;
	case WLAN_STAT_HTTP_CONNECTED:
		power_result = UART_PWR_HTTP_CONN;
		break;
	default:
		power_result = UART_PWR_APON;
		break;
	}
	uart_resp_get_power_state(param, power_result);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_POWER_STATE\r\n");
}

static void parser_record_param(ai_glass_record_param_t *rec_buf, uint8_t *raw_buf)
{
	if (rec_buf) {
		rec_buf->type = raw_buf[0];
		rec_buf->width = raw_buf[1] | (raw_buf[2] << 8);
		rec_buf->height = raw_buf[3] | (raw_buf[4] << 8);
		rec_buf->bps = raw_buf[5] | (raw_buf[6] << 8) | (raw_buf[7] << 16) | (raw_buf[8] << 24);
		rec_buf->fps = raw_buf[9] | (raw_buf[10] << 8);
		rec_buf->gop = raw_buf[11] | (raw_buf[12] << 8);
		rec_buf->roi.xmin = raw_buf[13] | (raw_buf[14] << 8) | (raw_buf[15] << 16) | (raw_buf[16] << 24);
		rec_buf->roi.ymin = raw_buf[17] | (raw_buf[18] << 8) | (raw_buf[19] << 16) | (raw_buf[20] << 24);
		rec_buf->roi.xmax = raw_buf[21] | (raw_buf[22] << 8) | (raw_buf[23] << 16) | (raw_buf[24] << 24);
		rec_buf->roi.ymax = raw_buf[25] | (raw_buf[26] << 8) | (raw_buf[27] << 16) | (raw_buf[28] << 24);

		rec_buf->minQp = raw_buf[29] | (raw_buf[30] << 8);
		rec_buf->maxQp = raw_buf[31] | (raw_buf[32] << 8);
		rec_buf->rotation = raw_buf[33];
		rec_buf->rc_mode = raw_buf[34];
		rec_buf->record_length = raw_buf[35] | (raw_buf[36] << 8);
	}
}

static void parser_life_snapshot_param(ai_glass_snapshot_param_t *snap_buf, uint8_t *raw_buf)
{
	if (snap_buf) {
		snap_buf->type = raw_buf[0];
		snap_buf->width = raw_buf[1] | (raw_buf[2] << 8) | (raw_buf[3] << 16) | (raw_buf[4] << 24);
		snap_buf->height = raw_buf[5] | (raw_buf[6] << 8) | (raw_buf[7] << 16) | (raw_buf[8] << 24);
		snap_buf->jpeg_qlevel = raw_buf[9] / 10;
		snap_buf->roi.xmin = raw_buf[10] | (raw_buf[11] << 8) | (raw_buf[12] << 16) | (raw_buf[13] << 24);
		snap_buf->roi.ymin = raw_buf[14] | (raw_buf[15] << 8) | (raw_buf[16] << 16) | (raw_buf[17] << 24);
		snap_buf->roi.xmax = raw_buf[18] | (raw_buf[19] << 8) | (raw_buf[20] << 16) | (raw_buf[21] << 24);
		snap_buf->roi.ymax = raw_buf[22] | (raw_buf[23] << 8) | (raw_buf[24] << 16) | (raw_buf[25] << 24);
		snap_buf->minQp = raw_buf[26] | (raw_buf[27] << 8);
		snap_buf->minQp = raw_buf[28] | (raw_buf[29] << 8);
		snap_buf->rotation = raw_buf[30];
	}
}

static void ai_glass_update_wifi_info(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_UPDATE_WIFI_INFO\r\n");
	uint8_t resp_stat = AI_GLASS_CMD_COMPLETE;
	ai_glass_snapshot_param_t temp_snapshot_param = {0};
	ai_glass_record_param_t temp_record_param = {0};
	uint8_t info_mode = 0;
	uint16_t info_size = 0;
	uint16_t record_time = 0;

	uint8_t *video_params = uart_parser_wifi_info_video_info(param, &info_mode, &info_size);
	AI_GLASS_MSG("info_mode = 0x%02x\r\n", info_mode);
	AI_GLASS_MSG("info_size = 0x%04x\r\n", info_size);
	switch (info_mode) {
	case UPDATE_DEFAULT_SNAPSHOT:
		AI_GLASS_MSG("Life snapshot param size = 0x%04x\r\n", sizeof(ai_glass_snapshot_param_t));
		media_get_life_snapshot_params(&temp_snapshot_param);
		AI_GLASS_INFO("Get LifeTime Snapshot Data\r\n");
		print_snapshot_data(&temp_snapshot_param);
		parser_life_snapshot_param(&temp_snapshot_param, video_params);
		if (media_update_life_snapshot_params(&temp_snapshot_param) != MEDIA_OK) {
			resp_stat = AI_GLASS_PARAMS_ERR;
		}
		media_get_life_snapshot_params(&temp_snapshot_param);
		AI_GLASS_INFO("Get LifeTime Snapshot Data Update Result\r\n");
		print_snapshot_data(&temp_snapshot_param);
		break;
	case UPDATE_DEFAULT_RECORD:
		AI_GLASS_MSG("Life record param size = 0x%04x\r\n", sizeof(ai_glass_record_param_t));
		parser_record_param(&temp_record_param, video_params);
		AI_GLASS_INFO("Get LifeTime Record Data\r\n");
		print_record_data(&temp_record_param);
		if (media_update_record_params(&temp_record_param) != MEDIA_OK) {
			resp_stat = AI_GLASS_PARAMS_ERR;
		}
		media_get_record_params(&temp_record_param);
		AI_GLASS_INFO("Get LifeTime Record Data Update Result\r\n");
		print_record_data(&temp_record_param);
		break;
	case UPDATE_RECORD_TIME:
		record_time = video_params[0] | video_params[1] << 8;
		AI_GLASS_MSG("Life record time = %d, info_size = %u\r\n", record_time, info_size);
		if (info_size > 0) {
			if (media_update_record_time(record_time) != MEDIA_OK) {
				resp_stat = AI_GLASS_PARAMS_ERR;
			}
		} else {
			resp_stat = AI_GLASS_PARAMS_ERR;
		}
		media_get_record_params(&temp_record_param);
		print_record_data(&temp_record_param);
		break;
	}

	uart_resp_update_wifi_info(param, resp_stat);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_UPDATE_WIFI_INFO\r\n");
}

static void ai_glass_set_gps(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_SET_GPS\r\n");
	uint32_t gps_week, gps_seconds = 0;
	float gps_latitude, gps_longitude, gps_altitude = 0;
	if (param->uart_pkt.length >= 34) {
		uart_parser_gps_data(param, &gps_week, &gps_seconds, &gps_latitude, &gps_longitude, &gps_altitude);

		AI_GLASS_INFO("gps_week = %d, %x\r\n", gps_week, gps_week);
		AI_GLASS_INFO("gps_seconds = %d, %x\r\n", gps_seconds, gps_seconds);
		media_filesystem_setup_gpstime(gps_week, gps_seconds);
		media_filesystem_setup_gpscoordinate(gps_latitude, gps_longitude, gps_altitude);
	} else {
		AI_GLASS_INFO("Invlaid GPS length = %d < 34\r\n", param->uart_pkt.length);
	}

	uint8_t status = AI_GLASS_CMD_COMPLETE;
	uart_resp_gps_data(param, status);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_SET_GPS\r\n");
}

static void ai_glass_snapshot(uartcmdpacket_t *param)
{
	uint8_t status = AI_GLASS_CMD_COMPLETE;
	AI_GLASS_MSG("get UART_RX_OPC_CMD_SNAPSHOT = %lu\r\n", mm_read_mediatime_ms());
	if (xSemaphoreTake(video_proc_sema, 0) != pdTRUE) {
		status = AI_GLASS_BUSY;
		AI_GLASS_WARN("AI glass is snapshot or record, current snapshot busy fail\r\n");
	} else {
		uint8_t mode = 0;
		uint8_t *snapshot_param = uart_parser_snapshot_video_info(param, &mode);
		AI_GLASS_MSG("%s get mode = %d\r\n", __func__, mode);
		if (mode == 1) {
			AI_GLASS_MSG("Process AI SNAPSHOT\r\n");
			ai_glass_snapshot_param_t ai_snap_params = {0};
			media_get_ai_snapshot_params(&ai_snap_params);
			parser_snapshot_pkt2param(&ai_snap_params, snapshot_param);
			if (media_update_ai_snapshot_params(&ai_snap_params) != MEDIA_OK) {
				AI_GLASS_WARN("Invlaid parmaeters set to default value\r\n");
			}
			AI_GLASS_MSG("snapshot initialed time = %lu\r\n", mm_read_mediatime_ms());
			int ret = ai_snapshot_initialize();
			if (ret == 0) {
				AI_GLASS_MSG("snapshot take time = %lu\r\n", mm_read_mediatime_ms());
				if (ai_snapshot_take("ai_snapshot.jpg") == 0) {
					status = AI_GLASS_CMD_COMPLETE;
				} else {
					status = AI_GLASS_PROC_FAIL;
				}
			} else if (ret == -2) {
				status = AI_GLASS_BUSY;
			} else {
				status = AI_GLASS_PROC_FAIL;
			}
			AI_GLASS_MSG("snapshot send pkt time = %lu\r\n", mm_read_mediatime_ms());
			if (ai_snap_params.status == 0) {
				uart_resp_snapshot(param, status);
			}
			if (ret == 0) {
				AI_GLASS_MSG("wait for ai snapshot deinit\r\n");
				while (ai_snapshot_deinitialize()) {
					vTaskDelay(1);
				}
				AI_GLASS_MSG("wait for ai snapshot deinit done = %lu\r\n", mm_read_mediatime_ms());
			}
			if (ai_snap_params.status == 1) {
				critical_process_started = 1;
				dual_snapshot = 1;
				AI_GLASS_MSG("AI+Lifetime Snapshot\r\n");
				goto lifetimesnapshot;
			}
		} else if (mode == 0) {
lifetimesnapshot:
			ai_glass_init_external_disk();
			AI_GLASS_MSG("Process LIFETIME SNAPSHOT\r\n");

			int ret = lifetime_snapshot_initialize();
			if (ret == 0) {
				status = AI_GLASS_DEVICE_WORKING_IN_PROG; // snapshot complete response requested to be sent earlier to BT instead of after lifetime_snapshot_take
				uart_resp_snapshot(param, status);
				uint8_t file_name_length = snapshot_param[0];
				char temp_record_filename_buffer[160] = {0};
				uint8_t lifetime_snap_name[160] = {0};
				if (file_name_length > 0 && file_name_length <= 125 && dual_snapshot != 1) {
					char uart_filename_str[160] = {0};
					memset(uart_filename_str, 0, file_name_length + 1);
					memcpy(uart_filename_str, snapshot_param + 1, file_name_length);
					AI_GLASS_MSG("Filename retrieved from 8773\r\n");
					extdisk_generate_unique_filename("", uart_filename_str, ".jpg", (char *)temp_record_filename_buffer, 160);
					snprintf((char *)lifetime_snap_name, sizeof(lifetime_snap_name), "%s%s", (const char *)temp_record_filename_buffer, ".jpg");
				} else {
					char *cur_time_str = (char *)media_filesystem_get_current_time_string();
					if (cur_time_str) {
						AI_GLASS_MSG("Filename generated from 8735B\r\n");
						extdisk_generate_unique_filename("PICTURE_0_0_", cur_time_str, ".jpg", (char *)temp_record_filename_buffer, 160);
						snprintf((char *)lifetime_snap_name, sizeof(lifetime_snap_name), "%s%s", (const char *)temp_record_filename_buffer, ".jpg");
						free(cur_time_str);
					} else {
						AI_GLASS_WARN("no memory for lifetime snapshot file name\r\n");
						extdisk_generate_unique_filename("PICTURE_0_0_", "19800101", ".jpg", (char *)temp_record_filename_buffer, 160);
					}
				}
				if (lifetime_snapshot_take((const char *)lifetime_snap_name) == 0) {
					if (lifetime_highres_save((const char *)lifetime_snap_name) != 0) {
						AI_GLASS_WARN("lifetime snapshot high res save failed\r\n");
						status = AI_GLASS_PROC_FAIL;
						uart_resp_snapshot(param, status);
					}
					extdisk_save_file_cntlist();
					AI_GLASS_MSG("Extdisk save file countlist done = %lu\r\n", mm_read_mediatime_ms());
					status = AI_GLASS_CMD_COMPLETE;
					critical_process_started = 0;
					uart_resp_snapshot(param, status);
				} else {
					status = AI_GLASS_PROC_FAIL;
					uart_resp_snapshot(param, status);
				}

				AI_GLASS_MSG("wait for lifetime snapshot deinit\r\n");
				while (lifetime_snapshot_deinitialize()) {
					vTaskDelay(1);
				}
				AI_GLASS_MSG("lifetime snapshot deinit done = %lu\r\n", mm_read_mediatime_ms());
			} else if (ret == -2) {
				status = AI_GLASS_BUSY;
				uart_resp_snapshot(param, status);
			} else {
				status = AI_GLASS_PROC_FAIL;
				uart_resp_snapshot(param, status);
			}
			// Save filelist to EMMC
			// extdisk_save_file_cntlist();
		} else {
			AI_GLASS_WARN("Not implement yet\r\n");
			status = AI_GLASS_PROC_FAIL;
			uart_resp_snapshot(param, status);
		}
		xSemaphoreGive(video_proc_sema);
	}
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_SNAPSHOT = %lu\r\n", mm_read_mediatime_ms());
}

static void ai_glass_get_file_name(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_FILE_NAME\r\n");
	uint8_t result = AI_GLASS_CMD_COMPLETE;
	uint32_t file_length = 0;

	memset(temp_file_name, 0x0, MAX_FILENAME_SIZE);
	snprintf((char *)temp_file_name, MAX_FILENAME_SIZE, "ai_snapshot.jpg");

	FILE *ai_snapshot_file = NULL;
	AI_GLASS_MSG("temp_file_name = %s\r\n", temp_file_name);
	ai_snapshot_file = ramdisk_fopen((const char *)temp_file_name, "rb");
	if (ai_snapshot_file != NULL) {
		ramdisk_fseek(ai_snapshot_file, 0, SEEK_END);
		file_length = ramdisk_ftell(ai_snapshot_file);
		ramdisk_fclose(ai_snapshot_file);
		result = AI_GLASS_CMD_COMPLETE;
		AI_GLASS_MSG("Get file name %s successfully\r\n", temp_file_name);
	} else {
		result = AI_GLASS_PROC_FAIL;
		AI_GLASS_MSG("Get file name %s fail\r\n", temp_file_name);
	}
	uart_resp_get_file_name(param, (const char *)temp_file_name, file_length, result);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_FILE_NAME\r\n");
}

static int aisnapshot_file_seek(FILE *ai_snapshot_rfile, uint32_t file_offset)
{
	return ramdisk_fseek(ai_snapshot_rfile, file_offset, SEEK_SET);
}

static int aisnapshot_file_read(uint8_t *buf, uint32_t read_size, FILE *ai_snapshot_rfile)
{
	return ramdisk_fread(buf, 1, read_size, ai_snapshot_rfile);
}

static int aisnapshot_file_eof(FILE *ai_snapshot_rfile)
{
	return ramdisk_feof(ai_snapshot_rfile);
}

static void ai_glass_get_pic_data(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_PICTURE_DATA\r\n");
	FILE *ai_snapshot_rfile = NULL;
	memset(temp_rfile_name, 0x0, MAX_FILENAME_SIZE);
	snprintf((char *)temp_rfile_name, MAX_FILENAME_SIZE, "ai_snapshot.jpg");
	AI_GLASS_MSG("temp_rfile_name = %s\r\n", temp_rfile_name);
	ai_snapshot_rfile = ramdisk_fopen((const char *)temp_rfile_name, "rb");
	if (ai_snapshot_rfile) {
		uart_resp_get_pic_data(param, ai_snapshot_rfile, aisnapshot_file_seek, aisnapshot_file_read, aisnapshot_file_eof);
		ramdisk_fclose(ai_snapshot_rfile);
	}
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_PICTURE_DATA\r\n");
}

static void ai_glass_get_trans_pic_stop(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_TRANS_PIC_STOP\r\n");
	uart_resp_get_trans_pic_stop(param);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_TRANS_PIC_STOP %lu\r\n", mm_read_mediatime_ms());
}

static void mp4_send_response_callback(struct tmrTimerControl *parm)
{
	uint8_t record_resp_status = AI_GLASS_CMD_COMPLETE;

	if (xSemaphoreTake(send_response_timermutex, portMAX_DELAY) == pdTRUE) {
		if (send_response_timer_setstop == 0) {
			if (current_state == STATE_END_RECORDING || current_state == STATE_ERROR) {
				if (current_state == STATE_ERROR) {
					record_resp_status = AI_GLASS_PROC_FAIL;
				} else {
					record_resp_status = AI_GLASS_CMD_COMPLETE;
				}
				lifetime_recording_deinitialize();
				send_response_timer_setstop = 1;
				xSemaphoreGive(send_response_timermutex);
				uart_resp_record_stop(record_resp_status);
				AI_GLASS_MSG("mp4_send_response_callback UART_TX_OPC_RESP_RECORD_STOP %lu\r\n", mm_read_mediatime_ms());
				xSemaphoreGive(video_proc_sema);
			} else {
				if (current_state == STATE_RECORDING || current_state == STATE_IDLE) {
					record_resp_status = AI_GLASS_CMD_COMPLETE;
				}
				uart_resp_record_cont(record_resp_status);
				AI_GLASS_MSG("mp4_send_response_callback %lu\r\n", mm_read_mediatime_ms());
				if (send_response_timer != NULL) {
					if (xTimerStart(send_response_timer, 0) != pdPASS) {
						AI_GLASS_ERR("Send timer failed\r\n");
					}
				}
				xSemaphoreGive(send_response_timermutex);
			}
		} else {
			xSemaphoreGive(send_response_timermutex);
		}
	} else {
		AI_GLASS_ERR("Send timer mutex failed\r\n");
	}
	return;
}
static void ai_glass_record_start(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_RECORD_START = %lu\r\n", mm_read_mediatime_ms());
	ai_glass_init_external_disk();
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	AI_GLASS_MSG("Opcode (hex): 0x%x\r\n", query_pkt->opcode);
	uint8_t record_start_status = AI_GLASS_CMD_COMPLETE;

	//UART PARSER_RECORDING_FILENAME_AND_LENGTH
	uint8_t record_filename_length = 0;
	uint8_t *record_filename = uart_parser_recording_video_info(param, &record_filename_length);
	const char *filename_str;
	char filename_buf[160] = {0}; // One extra for null terminator

	if (record_filename && record_filename_length < sizeof(filename_buf)) {
		memcpy(filename_buf, record_filename, record_filename_length);
		filename_buf[record_filename_length] = '\0'; // Null-terminate

		filename_str = filename_buf;
	}
	//This is to make sure that if there is no record filename, the length will not be passed into the function lifetime_recording initialize.
	else {
		record_filename_length = 0;
	}

	//Initialize function has a timer that constantly reads the status of MP4.
	if (xSemaphoreTake(video_proc_sema, 0) == pdTRUE) {
		AI_GLASS_MSG("Record start = %lu\r\n", mm_read_mediatime_ms());
		if (current_state == STATE_RECORDING || current_state == STATE_END_RECORDING) {
			AI_GLASS_MSG("Recording has started, not starting another recording\r\n");
			record_start_status = AI_GLASS_CMD_COMPLETE;
			uart_resp_record_start(record_start_status);
			xSemaphoreGive(video_proc_sema);
		} else if (current_state == STATE_IDLE) {
			int ret = lifetime_recording_initialize(record_filename_length, (const char *)filename_str);
			// Save filelist to EMMC
			if (send_response_timer != NULL && ret == 0) {
				extdisk_save_file_cntlist();
				if (xSemaphoreTake(send_response_timermutex, portMAX_DELAY) == pdTRUE) {
					if (xTimerStart(send_response_timer, 0) != pdPASS) {
						record_start_status = AI_GLASS_PROC_FAIL;
						uart_resp_record_start(record_start_status);
						AI_GLASS_ERR("Send UART_RX_OPC_CMD_RECORD_START timer failed\r\n");
						lifetime_recording_deinitialize();
						xSemaphoreGive(video_proc_sema);
					} else {
						record_start_status = AI_GLASS_CMD_COMPLETE;
						send_response_timer_setstop = 0;
						uart_resp_record_start(record_start_status);
					}
					xSemaphoreGive(send_response_timermutex);
				} else {
					record_start_status = AI_GLASS_PROC_FAIL;
					uart_resp_record_start(record_start_status);
					AI_GLASS_ERR("Send UART_RX_OPC_CMD_RECORD_START timer mutex failed\r\n");
					lifetime_recording_deinitialize();
					xSemaphoreGive(video_proc_sema);
				}
			} else {
				record_start_status = AI_GLASS_PROC_FAIL;
				uart_resp_record_start(record_start_status);
				AI_GLASS_ERR("Failed to create send_response_timer\r\n");
				xSemaphoreGive(video_proc_sema);
			}
		} else {
			record_start_status = AI_GLASS_PROC_FAIL;
			uart_resp_record_start(record_start_status);
			AI_GLASS_ERR("Failed because of the known record status\r\n");
			xSemaphoreGive(video_proc_sema);
		}
	} else {
		AI_GLASS_WARN("AI glass is snapshot or record, current record busy fail\r\n");
		record_start_status = AI_GLASS_BUSY;
		uart_resp_record_start(record_start_status);
	}

	AI_GLASS_INFO("end of UART_RX_OPC_CMD_RECORD_START\r\n");
}

static void ai_glass_record_sync_ts(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_RECORD_SYNC_TS\r\n");
	uart_resp_record_sync_ts(param);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_RECORD_SYNC_TS\r\n");
}

static void ai_glass_record_stop(uartcmdpacket_t *param)
{
	AI_GLASS_MSG("get UART_RX_OPC_CMD_RECORD_STOP %lu\r\n", mm_read_mediatime_ms());
	uint8_t record_stop_status = AI_GLASS_CMD_COMPLETE;
	if (current_state == STATE_RECORDING) {
		if (xSemaphoreTake(send_response_timermutex, portMAX_DELAY) == pdTRUE) {
			if (send_response_timer_setstop == 0) {
				if (send_response_timer != NULL) {
					if (xTimerIsTimerActive(send_response_timer) == pdTRUE) {
						xTimerStop(send_response_timer, 0);
					}
				}
				lifetime_recording_deinitialize();
				xSemaphoreGive(video_proc_sema);
				send_response_timer_setstop = 1;
				xSemaphoreGive(send_response_timermutex);
			} else {
				AI_GLASS_MSG("The recording timer has stop\r\n");
				xSemaphoreGive(send_response_timermutex);
			}
		}
	}
	uart_resp_record_stop(record_stop_status);
	AI_GLASS_MSG("end of UART_RX_OPC_CMD_RECORD_STOP %lu\r\n", mm_read_mediatime_ms());
}

static void ai_glass_get_file_cnt(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_FILE_CNT\r\n");
	ai_glass_init_external_disk();
	uint8_t result = AI_GLASS_CMD_COMPLETE;
	uint16_t film_num = extdisk_get_filecount(SYS_COUNT_FILM_LABEL);
	uint16_t snapshot_num = extdisk_get_filecount(SYS_COUNT_PIC_LABEL);

	AI_GLASS_MSG("mp4 file num = %u\r\n", film_num);
	AI_GLASS_MSG("jpg file num = %u\r\n", snapshot_num);
	uart_resp_get_file_cnt(param, film_num, snapshot_num, result);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_FILE_CNT\r\n");
}

static void ai_glass_delete_file(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_DELETE_FILE\r\n");
	ai_glass_init_external_disk();
	uart_resp_delete_file(param);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_DELETE_FILE\r\n");
}

static void ai_glass_delete_all_file(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_DELETE_ALL_FILES\r\n");
	if (ai_glass_disk_reformat() == AI_GLASS_CMD_COMPLETE) {
		uint8_t status = AI_GLASS_CMD_COMPLETE;
		uart_resp_delete_all_file(status);
	} else {
		uint8_t status = AI_GLASS_PROC_FAIL;
		uart_resp_delete_all_file(status);
	}
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_DELETE_ALL_FILES\r\n");
}

static void ai_glass_get_sd_info(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_SD_INFO %lu\r\n", mm_read_mediatime_ms());
	ai_glass_init_external_disk();
	uint64_t device_used_bytes = fatfs_get_used_space_byte();
	uint64_t device_total_bytes = device_used_bytes + fatfs_get_free_space_byte();
	uint32_t device_used_Kbytes = (uint32_t)(device_used_bytes / 1024);
	uint32_t device_total_Kbytes = (uint32_t)(device_total_bytes / 1024);

	uart_resp_get_sd_info(param, device_total_Kbytes, device_used_Kbytes);
	AI_GLASS_MSG("Get device memory: %lu/%luKB\r\n", device_used_Kbytes, device_total_Kbytes);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_SD_INFO %lu\r\n", mm_read_mediatime_ms());
}

static void ai_glass_set_ap_mode(uartcmdpacket_t *param)
{
	AI_GLASS_MSG("get UART_RX_OPC_CMD_SET_WIFI_MODE %lu\r\n", mm_read_mediatime_ms());
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint8_t mode = query_pkt->data_buf[0];
	uint8_t result = AI_GLASS_CMD_COMPLETE;
	rtw_softap_info_t wifi_cfg = {0};
	uint8_t password[MAX_AP_PASSWORD_LEN] = {0};
	wifi_cfg.password = password;

	if (mode == 1) {
		ai_glass_init_external_disk();
		if (wifi_enable_ap_mode(AI_GLASS_AP_SSID, AI_GLASS_AP_PASSWORD, AI_GLASS_AP_CHANNEL, 20) == WLAN_SET_OK) {
			wifi_get_ap_setting(&wifi_cfg);
			result = AI_GLASS_CMD_COMPLETE;
		} else {
			result = AI_GLASS_PROC_FAIL;
		}
	} else if (mode == 0) {
		if (wifi_disable_ap_mode() == WLAN_SET_OK) {
			result = AI_GLASS_CMD_COMPLETE;
		} else {
			result = AI_GLASS_PROC_FAIL;
		}
	} else {
		result = AI_GLASS_PARAMS_ERR;
	}
	AI_GLASS_MSG("UART_RX_OPC_CMD_SET_WIFI_MODE set mode %d done %lu\r\n", mode, mm_read_mediatime_ms());
	uart_resp_set_ap_mode(param, &wifi_cfg, MAX_AP_SSID_VALUE_LEN, MAX_AP_PASSWORD_LEN, result);

	if (mode == 1 && result == AI_GLASS_CMD_COMPLETE) {
		deinitial_media(); // To save power
	}
	AI_GLASS_MSG("end of UART_RX_OPC_CMD_SET_WIFI_MODE %lu\r\n", mm_read_mediatime_ms());
}

// For UART_RX_OPC_CMD_SET_STA_MODE
static void ai_glass_set_sta_mode(uartcmdpacket_t *param)
{
	AI_GLASS_MSG("get UART_RX_OPC_CMD_SET_STA_MODE %lu\r\n", mm_read_mediatime_ms());

	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);

	for (int i = 0; i < 128; i++) {
		AI_GLASS_INFO("%02X ", query_pkt->data_buf[i]);
		if ((i + 1) % 16 == 0) {
			AI_GLASS_INFO("\r\n");    // Pretty print in 16-byte rows
		}
	}
	AI_GLASS_INFO("\r\n");

	uint8_t new_mode = query_pkt->data_buf[0];
	uint8_t ssid_length = query_pkt->data_buf[1];
	uint8_t channel = query_pkt->data_buf[40];
	uint8_t password_length = query_pkt->data_buf[41];

	if (ssid_length > MAX_SSID_LEN) {
		ssid_length = MAX_SSID_LEN;
	}

	if (password_length > MAX_PASSWORD_LEN) {
		password_length = MAX_PASSWORD_LEN;
	}

	AI_GLASS_INFO("Mode: %d\r\n", new_mode);
	AI_GLASS_INFO("SSID Length: %d\r\n", ssid_length);
	AI_GLASS_INFO("Channel: %d\r\n", channel);
	AI_GLASS_INFO("Password Length: %d\r\n", password_length);

	// Create a buffer for the SSID (null-terminated)
	unsigned char ssid[MAX_SSID_LEN + 1]; // +1 for '\0'

	// Copy bytes 2 ~ (2 + ssid_length - 1)
	memcpy(ssid, &query_pkt->data_buf[2], ssid_length);

	// Null-terminate if you're treating it as a string
	ssid[ssid_length] = '\0';

	unsigned char password[MAX_PASSWORD_LEN + 1];
	memcpy(password, &query_pkt->data_buf[42], password_length);

	password[password_length] = '\0';

	uint32_t security_type_value;
	memcpy(&security_type_value, &query_pkt->data_buf[36], sizeof(security_type_value));

	AI_GLASS_INFO("From UART Password: %s\r\n", password);

	rtw_network_info_t connect_param = {0};
	memcpy(connect_param.ssid.val, ssid, ssid_length);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = password_length;
	connect_param.ssid.len = ssid_length;
	connect_param.security_type = (rtw_security_t)security_type_value;

	uint8_t result = AI_GLASS_CMD_COMPLETE;

	AI_GLASS_INFO("SSID: %s\r\n", connect_param.ssid.val);
	AI_GLASS_INFO("Password: %s\r\n", connect_param.password);
	AI_GLASS_INFO("Password length: %d\r\n", connect_param.password_len);
	AI_GLASS_INFO("SSID length: %d\r\n", connect_param.ssid.len);
	AI_GLASS_INFO("Security Type: %d\r\n", security_type_value);

	if (channel != 0) {
		connect_param.channel = (unsigned char)channel;
		connect_param.pscan_option = (unsigned char)PSCAN_FAST_SURVEY;
	}

	if (new_mode == 1 || new_mode == 2) {
		// Only proceed if current mode is 0 (AP -> STA transition)
		if (g_current_wifi_mode == 0) {
			// Init emmc and try to connect to STA
			ai_glass_init_external_disk();
			AI_GLASS_MSG("wifi_enable_sta_mode %lu\r\n", mm_read_mediatime_ms());

			if (wifi_enable_sta_mode(&connect_param, 100, 20) == WLAN_SET_OK) {
				result = AI_GLASS_CMD_COMPLETE;
				g_current_wifi_mode = new_mode;
				AI_GLASS_MSG("Current mode is %u\r\n", g_current_wifi_mode);

			} else {
				result = AI_GLASS_PROC_FAIL;
			}

			u32 ip;
			uint8_t ip0, ip1, ip2, ip3;
			while (1) {
				ip = *(u32 *)LwIP_GetIP(0);

				ip0 = (ip) & 0xFF;
				ip1 = (ip >> 8) & 0xFF;
				ip2 = (ip >> 16) & 0xFF;
				ip3 = (ip >> 24) & 0xFF;

				if (ip0 != 0 && ip1 != 0 && ip2 != 0 && ip3 != 0) {
					break;
				}
				AI_GLASS_INFO("Waiting for IP...\r\n");
				vTaskDelay(100);
			}

			AI_GLASS_INFO("ip_idx0: %d\r\n", ip0);
			AI_GLASS_INFO("ip_idx1: %d\r\n", ip1);
			AI_GLASS_INFO("ip_idx2: %d\r\n", ip2);
			AI_GLASS_INFO("ip_idx3: %d\r\n", ip3);

			uart_resp_set_sta_mode(param, result,
								   ip0,
								   ip1,
								   ip2,
								   ip3);
		} else {
			AI_GLASS_INFO("Already in STA mode. Skipping re-init.\r\n");
		}
	} else if (new_mode == 0) {
		if (wifi_disable_sta_mode() == WLAN_SET_OK) {
			AI_GLASS_INFO("STA mode disabled successfully.\r\n");
			g_current_wifi_mode = new_mode;
			AI_GLASS_MSG("Current mode is %u\r\n", g_current_wifi_mode);
			result = AI_GLASS_CMD_COMPLETE;
		} else {
			AI_GLASS_INFO("Fail to disable STA mode.\r\n");
			result = AI_GLASS_PROC_FAIL;
		}
		uart_resp_set_sta_mode(param, result, 0, 0, 0, 0);
	} else {
		result = AI_GLASS_PARAMS_ERR;
		AI_GLASS_INFO("Invalid mode value: %d\r\n", new_mode);
		uart_resp_set_sta_mode(param, result, 0, 0, 0, 0);
	}
}

static void ai_glass_get_pic_data_sliding_window(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_PICTURE_DATA SLIDING WINDOW\r\n");
	FILE *ai_snapshot_rfile = NULL;
	memset(temp_rfile_name, 0x0, MAX_FILENAME_SIZE);
	snprintf((char *)temp_rfile_name, MAX_FILENAME_SIZE, "ai_snapshot.jpg");
	AI_GLASS_MSG("temp_rfile_name = %s\r\n", temp_rfile_name);
	ai_snapshot_rfile = ramdisk_fopen((const char *)temp_rfile_name, "rb");
	if (ai_snapshot_rfile) {
		uart_resp_get_pic_data_sliding_window(param, ai_snapshot_rfile, aisnapshot_file_seek, aisnapshot_file_read, aisnapshot_file_eof);
		ramdisk_fclose(ai_snapshot_rfile);
	} else {
		AI_GLASS_ERR("AI snapshot jpeg open fail\r\n");
	}
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_PICTURE_DATA SLIDING WINDOW END\r\n");
}

static void ai_glass_get_pic_data_sliding_window_ack(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_PICTURE_DATA SLIDING WINDOW_ACK\r\n");
	uart_resp_get_pic_data_sliding_window_ack(param);
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_PICTURE_DATA SLIDING WINDOW_ACK END\r\n");
}

// {opcode, {is_critical, is_no_ack, callback}, {NULL, NULL})
static rxopc_item_t rx_opcode_basic_items[ ] = {
	{UART_RX_OPC_CMD_QUERY_INFO,        {true,  false, ai_glass_get_query_info},        {NULL, NULL}},
	{UART_RX_OPC_CMD_POWER_DOWN,        {true,  false, ai_glass_get_power_down},        {NULL, NULL}},
	{UART_RX_OPC_CMD_GET_POWER_STATE,   {true,  false, ai_glass_get_power_state},       {NULL, NULL}},
	{UART_RX_OPC_CMD_UPDATE_WIFI_INFO,  {true,  false, ai_glass_update_wifi_info},      {NULL, NULL}},
	{UART_RX_OPC_CMD_SET_GPS,           {true,  false, ai_glass_set_gps},               {NULL, NULL}},
	{UART_RX_OPC_CMD_SNAPSHOT,          {false, false, ai_glass_snapshot},              {NULL, NULL}},
	{UART_RX_OPC_CMD_GET_FILE_NAME,     {false, false, ai_glass_get_file_name},         {NULL, NULL}},
	{UART_RX_OPC_CMD_GET_PICTURE_DATA,  {false, false, ai_glass_get_pic_data},          {NULL, NULL}},
	{UART_RX_OPC_CMD_TRANS_PIC_STOP,    {true,  false, ai_glass_get_trans_pic_stop},    {NULL, NULL}},
	{UART_RX_OPC_CMD_RECORD_START,      {false, false, ai_glass_record_start},          {NULL, NULL}},
	{UART_RX_OPC_CMD_RECORD_SYNC_TS,    {false, false, ai_glass_record_sync_ts},        {NULL, NULL}},
	{UART_RX_OPC_CMD_RECORD_STOP,       {true,  false, ai_glass_record_stop},           {NULL, NULL}},
	{UART_RX_OPC_CMD_GET_FILE_CNT,      {false, false, ai_glass_get_file_cnt},          {NULL, NULL}},
	{UART_RX_OPC_CMD_DELETE_FILE,       {false, false, ai_glass_delete_file},           {NULL, NULL}},
	{UART_RX_OPC_CMD_DELETE_ALL_FILES,  {false, false, ai_glass_delete_all_file},       {NULL, NULL}},
	{UART_RX_OPC_CMD_GET_SD_INFO,       {false, false, ai_glass_get_sd_info},           {NULL, NULL}},
	{UART_RX_OPC_CMD_SET_WIFI_MODE,     {false, false, ai_glass_set_ap_mode},           {NULL, NULL}},
	{UART_RX_OPC_CMD_SET_STA_MODE,      {false, false, ai_glass_set_sta_mode},          {NULL, NULL}},

	{UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW,       {false, false, ai_glass_get_pic_data_sliding_window},       {NULL, NULL}},
	{UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW_ACK,   {false, true, ai_glass_get_pic_data_sliding_window_ack},    {NULL, NULL}},

	{UART_RX_OPC_CMD_SET_SYS_UPGRADE,                        {false, false, ai_glass_get_set_sys_upgrade},              {NULL, NULL}},
	{UART_TX_OPC_CMD_START_BT_SOC_FW_UPGRADE,                {false, false, ai_glass_resp_bt_fw_upgrade},               {NULL, NULL}},
	{UART_TX_OPC_CMD_FINISH_BT_SOC_FW_UPGRADE,               {false, false, ai_glass_resp_bt_fw_finish},                {NULL, NULL}},
	{UART_RX_OPC_CMD_SET_WIFI_FW_ROLLBACK,                   {false, false, ai_glass_wifi_fw_rollback},                 {NULL, NULL}},


};

void uart_fun_regist(void)
{
	uart_service_add_table(rx_opcode_basic_items, sizeof(rx_opcode_basic_items) / sizeof(rx_opcode_basic_items[0]));
}

void ai_glass_service_thread(void *param)
{
	AI_GLASS_MSG("ai_glass_service_thread start %lu\r\n", mm_read_mediatime_ms());

	initial_media_parameters();
	AI_GLASS_MSG("media system done %lu\r\n", mm_read_mediatime_ms());

	// cost about 60ms into this function
	media_filesystem_init();
	media_filesystem_setup_gpstime(0, 0); // Set up GPS start time to prevent failed for file system
	ai_glass_init_ram_disk();
	//ai_glass_init_external_disk(); // init EMMC here will cause 160 ms delay
	//extdisk_save_file_cntlist();
	AI_GLASS_MSG("vfs system done %lu\r\n", mm_read_mediatime_ms());
	ai_glass_log_init();

	uart_service_init(UART_TX, UART_RX, UART_BAUDRATE);

	send_response_timer = xTimerCreate("send_response_timer", 100 / portTICK_PERIOD_MS, pdFALSE, NULL, mp4_send_response_callback);
	if (send_response_timer == NULL) {
		AI_GLASS_ERR("send_response_timer create fail\r\n");
		goto exit;
	}
	send_response_timermutex = xSemaphoreCreateMutex();
	if (send_response_timermutex == NULL) {
		AI_GLASS_ERR("send_response_timermutex create fail\r\n");
		goto exit;
	}
	video_proc_sema = xSemaphoreCreateBinary();
	if (video_proc_sema == NULL) {
		AI_GLASS_ERR("video_proc_sema create fail\r\n");
		goto exit;
	}
	xSemaphoreGive(video_proc_sema);
	uart_fun_regist();
	uart_service_set_protocal_version(UART_PROTOCAL_VERSION);
	uart_service_start(1);
	AI_GLASS_MSG("uart service send data time %lu\r\n", mm_read_mediatime_ms());

exit:
	vTaskDelete(NULL);
}

void ai_glass_init(void)
{
	ai_glass_get_fw_version();
	if (xTaskCreate(ai_glass_service_thread, ((const char *)"example_uart_service_thread"), 4096, NULL, tskIDLE_PRIORITY + 5, NULL) != pdPASS) {
		AI_GLASS_ERR("\n\r%s xTaskCreate(example_uart_service_thread) failed", __FUNCTION__);
	}
}

// The below command is for testing
#if defined(ENABLE_TEST_CMD) && ENABLE_TEST_CMD
#include "gyrosensor_api.h"
static gyro_data_t gdata[100] = {0};
void gyro_read_gsensor_thread(void *param)
{
	AI_GLASS_MSG("Test Gyro Sensor Type: TDK ICM42670P/ICM42607P\r\n");
	gyroscope_fifo_init();
	while (1) {
		int read_cnt = gyroscope_fifo_read(gdata, 100);
		if (read_cnt > 0) {
			uint32_t cur_ts = mm_read_mediatime_ms();
			AI_GLASS_MSG("timestamp: %lu\r\n", cur_ts + gdata[read_cnt - 1].timestamp);
#if !IGN_ACC_DATA
			AI_GLASS_MSG("angular acceleration: X %f Y %f Z %f\r\n", gdata[read_cnt - 1].g[0], gdata[read_cnt - 1].g[1], gdata[read_cnt - 1].g[2]);
#endif
			AI_GLASS_MSG("angular velocity: X %f Y %f Z %f\r\n", gdata[read_cnt - 1].dps[0], gdata[read_cnt - 1].dps[1], gdata[read_cnt - 1].dps[2]);
		}
		vTaskDelay(30);
	}

	free(gdata);
	vTaskDelete(NULL);
}

void fTESTGSENSOR(void *arg)
{
	if (xTaskCreate(gyro_read_gsensor_thread, ((const char *)"gyro_task"), 32 * 1024, NULL, tskIDLE_PRIORITY + 7, NULL) != pdPASS) {
		AI_GLASS_ERR("\n\r%s xTaskCreate(gyro_task) failed", __FUNCTION__);
	}
}

void fDISKFORMAT(void *arg)
{
	ai_glass_init_external_disk();
	AI_GLASS_MSG("Format disk to FAT32\r\n");
	int ret = vfs_user_format(ai_glass_disk_name, VFS_FATFS, EXTDISK_PLATFORM);
	if (ret == FR_OK) {
		AI_GLASS_MSG("format successfully\r\n");
		ai_glass_deinit_external_disk();
	} else {
		AI_GLASS_ERR("format failed %d\r\n", ret);
	}
}

void fENABLEMSC(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};

	argc = parse_param(arg, argv);
	if (argc) {
		int msc_enable = atoi(argv[1]);
		if (msc_enable) {
			AI_GLASS_MSG("Enable mass storage device\r\n");
			aiglass_mass_storage_init();
		} else {
			AI_GLASS_MSG("Disable mass storage device\r\n");
			aiglass_mass_storage_deinit();
		}
	}
}

void fENABLEAPMODE(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};

	argc = parse_param(arg, argv);
	if (argc) {
		int apmode_enable = atoi(argv[1]);
		if (apmode_enable) {
			ai_glass_init_external_disk();
			AI_GLASS_MSG("Command enable AP mode start = %lu\r\n", mm_read_mediatime_ms());
			if (wifi_enable_ap_mode(AI_GLASS_AP_SSID, AI_GLASS_AP_PASSWORD, AI_GLASS_AP_CHANNEL, 20) == WLAN_SET_OK) {
				deinitial_media(); // For saving power
				AI_GLASS_MSG("Command enable AP mode OK = %lu\r\n", mm_read_mediatime_ms());
			} else {
				AI_GLASS_MSG("Command enable AP mode failed = %lu\r\n", mm_read_mediatime_ms());
			}
		} else {
			AI_GLASS_MSG("Command disable AP mode start = %lu\r\n", mm_read_mediatime_ms());
			if (wifi_disable_ap_mode() == WLAN_SET_OK) {
				AI_GLASS_MSG("Command disable AP mode OK = %lu\r\n", mm_read_mediatime_ms());
			} else {
				AI_GLASS_MSG("Command disable AP mode failed = %lu\r\n", mm_read_mediatime_ms());
			}
		}
	}
}

void fENABLESTAMODE(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};

	uint8_t mode  = 0;
	uint8_t ssid_length = MAX_SSID_LEN; //default max value
	char ssid[MAX_SSID_LEN + 1] = {0}; // +1 for null terminator
	uint32_t security_type = 0;
	uint8_t channel = 0;
	uint8_t resv_val = 0;
	uint8_t password_length = MAX_PASSWORD_LEN; //default max value
	char password[MAX_PASSWORD_LEN + 1] = {0}; // +1 for null terminator

	argc = parse_param(arg, argv);

	if (argc) {
		printf("argc = %d\r\n", argc);
		//mode
		if (atoi(argv[1]) == 0) {
			mode = 0;
			printf("Set wifi STA idle mode\r\n");
		} else {
			mode = 1;
			printf("Set wifi STA mode\r\n");
		}

		// ssid length
		if ((atoi(argv[2]) > 0) && (atoi(argv[2]) < MAX_SSID_LEN)) {
			ssid_length = atoi(argv[2]);
		} else {
			ssid_length = MAX_SSID_LEN;
			printf("Negative parameter set for SSID Length OR parameter is above than MAX_SSID_LEN, set to default max length for SSID\r\n");
		}

		// ssid string
		if (argc > 3 && strlen(argv[3]) <= ssid_length) {
			strncpy(ssid, argv[3], ssid_length);
			ssid[ssid_length] = '\0'; // Ensure null-termination
			printf("SSID: %s\r\n", ssid);
		} else {
			printf("SSID not provided or exceeds specified length\r\n");
		}

		// security type (right now security type do not check which values are valid)
		if (argc > 4) {
			security_type = (uint32_t) strtoul(argv[4], NULL, 0); // supports hex (e.g., 0x00400004)
			printf("Security Type: 0x%08X\r\n", security_type);
		} else {
			security_type = 0; // RTW_SECURITY_OPEN
			printf("No Security Type provided, defaulting to OPEN (0x%08X)\r\n", security_type);
		}

		// channel (right now no validation of channel yet)
		if (argc > 5) {
			channel = atoi(argv[5]);
			printf("Channel: %d\r\n", channel);
		}

		// password length
		if ((argc > 6) && (atoi(argv[6]) > 0) && (atoi(argv[6]) < MAX_PASSWORD_LEN)) {
			password_length = atoi(argv[6]);
			printf("Password Length: %d\r\n", password_length);
		} else {
			password_length = MAX_PASSWORD_LEN;
			printf("Negative parameter set for Password Length OR parameter is above than MAX_PASSWORD_LEN, set to default max length for Password\r\n");
		}

		// Password string
		if (argc > 7 && strlen(argv[7]) <= password_length) {
			strncpy(password, argv[7], password_length);
			password[password_length] = '\0'; // Ensure null-termination
			printf("Password: %s\r\n", password);
		} else {
			printf("Password not provided or exceeds specified length\r\n");
		}

		rtw_network_info_t connect_param = {0};
		memcpy(connect_param.ssid.val, ssid, ssid_length);
		connect_param.password = (unsigned char *)password;
		connect_param.password_len = password_length;
		connect_param.ssid.len = ssid_length;
		connect_param.security_type = (rtw_security_t)security_type;

		if (channel != 0) {
			connect_param.channel = (unsigned char)channel;
			connect_param.pscan_option = (unsigned char)PSCAN_FAST_SURVEY;
		}

		if (mode == 1) {
			AI_GLASS_MSG("Command enable STA mode start = %lu\r\n", mm_read_mediatime_ms());
			if (wifi_enable_sta_mode(&connect_param, 100, 20) == WLAN_SET_OK) {
				deinitial_media(); // For saving power
				AI_GLASS_MSG("Command enable STA mode OK = %lu\r\n", mm_read_mediatime_ms());
			} else {
				AI_GLASS_MSG("Command enable STA mode failed = %lu\r\n", mm_read_mediatime_ms());
			}
		} else {
			AI_GLASS_MSG("Command disable AP mode start = %lu\r\n", mm_read_mediatime_ms());
			if (wifi_disable_sta_mode() == WLAN_SET_OK) {
				AI_GLASS_MSG("Command disable STA mode OK = %lu\r\n", mm_read_mediatime_ms());
			} else {
				AI_GLASS_MSG("Command disable STA mode failed = %lu\r\n", mm_read_mediatime_ms());
			}
		}

	}
}

void fLFSNAPSHOT(void *arg)
{
	uint8_t status = AI_GLASS_CMD_COMPLETE;
	if (xSemaphoreTake(video_proc_sema, 0) != pdTRUE) {
		AI_GLASS_WARN("AI glass is snapshot or record, current snapshot busy fail\r\n");
		goto endofsnapshot;
	}
	AI_GLASS_MSG("snapshot aiglass_mass_storage_deinit time = %lu\r\n", mm_read_mediatime_ms());
	ai_glass_init_external_disk();
	AI_GLASS_MSG("Process LIFETIME SNAPSHOT\r\n");

	int ret = lifetime_snapshot_initialize();
	if (ret == 0) {
		char temp_record_filename_buffer[160] = {0};
		uint8_t lifetime_snap_name[160] = {0};

		char *cur_time_str = (char *)media_filesystem_get_current_time_string();
		if (cur_time_str) {
			extdisk_generate_unique_filename("PICTURE_0_0_", cur_time_str, ".jpg", (char *)temp_record_filename_buffer, 160);
			snprintf((char *)lifetime_snap_name, sizeof(lifetime_snap_name), "%s%s", (const char *)temp_record_filename_buffer, ".jpg");
			free(cur_time_str);
		} else {
			AI_GLASS_WARN("no memory for lifetime snapshot file name\r\n");
			extdisk_generate_unique_filename("PICTURE_0_0_", "19800101", ".jpg", (char *)temp_record_filename_buffer, 160);
		}
		if (lifetime_snapshot_take((const char *)lifetime_snap_name) == 0) {
			status = AI_GLASS_CMD_COMPLETE;
			if (lifetime_highres_save((const char *)lifetime_snap_name) != 0) {
				AI_GLASS_WARN("lifetime snapshot high res save failed\r\n");
				status = AI_GLASS_PROC_FAIL;
			}
		} else {
			status = AI_GLASS_PROC_FAIL;
		}

		AI_GLASS_MSG("wait for lifetime snapshot deinit\r\n");
		while (lifetime_snapshot_deinitialize()) {
			vTaskDelay(1);
		}
		AI_GLASS_MSG("lifetime snapshot deinit done = %lu\r\n", mm_read_mediatime_ms());
	} else if (ret == -2) {
		status = AI_GLASS_BUSY;
	} else {
		status = AI_GLASS_PROC_FAIL;
	}
	// Save filelist to EMMC
	extdisk_save_file_cntlist();
	xSemaphoreGive(video_proc_sema);
endofsnapshot:
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_SNAPSHOT = %lu\r\n", mm_read_mediatime_ms());
}

log_item_t at_ai_glass_items[ ] = {
	{"AT+AIGLASSFORMAT",    fDISKFORMAT,    {NULL, NULL}},
	{"AT+AIGLASSGSENSOR",   fTESTGSENSOR,   {NULL, NULL}},
	{"AT+AIGLASSMSC",       fENABLEMSC,     {NULL, NULL}},
	{"AT+AIGLASSSETAPMODE", fENABLEAPMODE,  {NULL, NULL}},
	{"AT+AIGLASSLFSNAP",    fLFSNAPSHOT,    {NULL, NULL}},
	{"AT+AIGLASSSETSTAMODE", fENABLESTAMODE, {NULL, NULL}},
};
#endif
void ai_glass_log_init(void)
{
#if defined(ENABLE_TEST_CMD) && ENABLE_TEST_CMD
	log_service_add_table(at_ai_glass_items, sizeof(at_ai_glass_items) / sizeof(at_ai_glass_items[0]));
#endif
}

log_module_init(ai_glass_log_init);
