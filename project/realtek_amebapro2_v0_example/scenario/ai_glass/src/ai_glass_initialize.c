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

// Funtion Prototype
static void ai_glass_init_external_disk(void);
static void ai_glass_init_ram_disk(void);
void ai_glass_log_init(void);

static const uint8_t version[4] = {0, 0, 0, 0};

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

static void ai_glass_init_ram_disk(void)
{
	if (!ramdisk_get_init_status()) {
		ramdisk_filesystem_init("ai_ram");
	}
}

typedef struct snapshot_pkt_s {
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
		aisnap_buf.version = raw_buf[0];
		aisnap_buf.q_vlaue = raw_buf[1];
		temp_data = raw_buf[2] | (raw_buf[3] << 8) | (raw_buf[4] << 16) | (raw_buf[5] << 24);
		memcpy(&(aisnap_buf.ROIX_TL), &temp_data, sizeof(uint32_t));
		temp_data = raw_buf[6] | (raw_buf[7] << 8) | (raw_buf[8] << 16) | (raw_buf[9] << 24);
		memcpy(&(aisnap_buf.ROIY_TL), &temp_data, sizeof(uint32_t));
		temp_data = raw_buf[10] | (raw_buf[11] << 8) | (raw_buf[12] << 16) | (raw_buf[13] << 24);
		memcpy(&(aisnap_buf.ROIX_BR), &temp_data, sizeof(uint32_t));
		temp_data = raw_buf[14] | (raw_buf[15] << 8) | (raw_buf[16] << 16) | (raw_buf[17] << 24);
		memcpy(&(aisnap_buf.ROIY_BR), &temp_data, sizeof(uint32_t));
		aisnap_buf.RESIZE_W = raw_buf[18] | (raw_buf[19] << 8);
		aisnap_buf.RESIZE_H = raw_buf[20] | (raw_buf[21] << 8);

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
	}
}

static void ai_glass_get_query_info(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_QUERY_INFO\r\n");
	uart_resp_get_query_info(param, version);
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
			uart_resp_snapshot(param, status);
			if (ret == 0) {
				AI_GLASS_MSG("wait for ai snapshot deinit\r\n");
				while (ai_snapshot_deinitialize()) {
					vTaskDelay(1);
				}
				AI_GLASS_MSG("wait for ai snapshot deinit done = %lu\r\n", mm_read_mediatime_ms());
			}
		} else if (mode == 0) {
			ai_glass_init_external_disk();
			AI_GLASS_MSG("Process LIFETIME SNAPSHOT\r\n");

			int ret = lifetime_snapshot_initialize();
			if (ret == 0) {
				status = AI_GLASS_CMD_COMPLETE; // snapshot complete response requested to be sent earlier to BT instead of after lifetime_snapshot_take
				uart_resp_snapshot(param, status);
				uint8_t file_name_length = snapshot_param[0];
				char temp_record_filename_buffer[160] = {0};
				uint8_t lifetime_snap_name[160] = {0};
				if (file_name_length > 0 && file_name_length <= 125) {
					char uart_filename_str[160] = {0};
					memset(uart_filename_str, 0, file_name_length + 1);
					memcpy(uart_filename_str, snapshot_param + 1, file_name_length);
					extdisk_generate_unique_filename("", uart_filename_str, ".jpg", (char *)temp_record_filename_buffer, 160);
					snprintf((char *)lifetime_snap_name, sizeof(lifetime_snap_name), "%s%s", (const char *)temp_record_filename_buffer, ".jpg");
				} else {
					char *cur_time_str = (char *)media_filesystem_get_current_time_string();
					if (cur_time_str) {
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
						status = AI_GLASS_HR_SAVE_FAILED;
						uart_resp_snapshot(param, status);
					}
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
			extdisk_save_file_cntlist();
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
	ai_glass_init_external_disk();
	uart_resp_delete_all_file(param);
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

	// Step 1: Disable AP mode
	if (wifi_disable_ap_mode() == WLAN_SET_OK) {
		AI_GLASS_INFO("AP mode disabled successfully.\r\n");
	} else {
		AI_GLASS_INFO("Fail to disable AP mode.\r\n");
	}

	// Step 2: Get params and set variables
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);

	for (int i = 0; i < 128; i++) {
		AI_GLASS_INFO("%02X ", query_pkt->data_buf[i]);
		if ((i + 1) % 16 == 0) {
			AI_GLASS_INFO("\r\n");    // Pretty print in 16-byte rows
		}
	}
	AI_GLASS_INFO("\r\n");

	uint8_t mode = query_pkt->data_buf[0];
	uint8_t ssid_length = query_pkt->data_buf[1];
	uint8_t channel = query_pkt->data_buf[40];
	uint8_t password_length = query_pkt->data_buf[41];

	if (ssid_length > MAX_SSID_LEN) {
		ssid_length = MAX_SSID_LEN;
	}

	if (password_length > MAX_PASSWORD_LEN) {
		password_length = MAX_PASSWORD_LEN;
	}

	AI_GLASS_INFO("Mode: %d\r\n", mode);
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

	if (mode == 1) {
		//Init emmc
		ai_glass_init_external_disk();
		AI_GLASS_MSG("wifi_enable_sta_mode %lu\r\n", mm_read_mediatime_ms());

		if (wifi_enable_sta_mode(&connect_param, 100, 2) == WLAN_SET_OK) {
			result = AI_GLASS_CMD_COMPLETE;
		} else {
			result = AI_GLASS_PROC_FAIL;
		}
		u32 ip = *(u32 *)LwIP_GetIP(0);
		// Send response
		uint8_t sta_status = result;
		// Get the IP address here:
		uint8_t ip0 = (ip) & 0xFF; // Highest byte (first)
		uint8_t ip1 = (ip >> 8) & 0xFF;
		uint8_t ip2 = (ip >> 16) & 0xFF;
		uint8_t ip3 = (ip >> 24) & 0xFF; // Lowest byte (last)

		AI_GLASS_INFO("ip_idx0: %d\r\n", ip0);
		AI_GLASS_INFO("ip_idx1: %d\r\n", ip1);
		AI_GLASS_INFO("ip_idx2: %d\r\n", ip2);
		AI_GLASS_INFO("ip_idx3: %d\r\n", ip3);
		uart_resp_set_sta_mode(param, sta_status, ip0, ip1, ip2, ip3);

	} else if (mode == 0) {
		// Disable AP mode
		if (wifi_disable_sta_mode() == WLAN_SET_OK) {
			AI_GLASS_INFO("STA mode disabled successfully.\r\n");
		} else {
			AI_GLASS_INFO("Fail to disable STA mode.\r\n");
		}
	} else {
		result = AI_GLASS_PARAMS_ERR;
		AI_GLASS_INFO("ERROR: Mode received for STA Mode is not 0 or 1.\r\n");
		uart_resp_set_sta_mode(param, result, 0, 0, 0, 0);
	}

	AI_GLASS_MSG("UART_RX_OPC_CMD_SET_STA_MODE set mode %d done %lu\r\n", mode, mm_read_mediatime_ms());
	if (mode == 1 && result == AI_GLASS_CMD_COMPLETE) {
		deinitial_media(); // To save power
	}
	AI_GLASS_MSG("end of UART_RX_OPC_CMD_SET_STA_MODE %lu\r\n", mm_read_mediatime_ms());
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
			if (wifi_enable_sta_mode(&connect_param, 100, 2) == WLAN_SET_OK) {
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
				status = AI_GLASS_HR_SAVE_FAILED;
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