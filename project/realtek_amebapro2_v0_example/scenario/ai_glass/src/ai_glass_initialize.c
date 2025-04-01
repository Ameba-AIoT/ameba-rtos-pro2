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

// Configure for ai glass
#define ENABLE_TEST_CMD             1   // For the tester to test some hardware
#define ENABLE_DISK_MASS_STORAGE    0   // For testing 8735 image and picture
#define EXTDISK_PLATFORM            VFS_INF_EMMC //VFS_INF_SD
#define UART_TX                     PA_2
#define UART_RX                     PA_3
#define UART_BAUDRATE               2000000 //115200 //2000000 //3750000 //4000000

// Definition for UPDATE TYPE
#define UPDATE_DEFAULT_SNAPSHOT     1
#define UPDATE_DEFAULT_RECORD       2
#define UPDATE_RECORD_TIME          3

// Definition for AI SNAPSHOT STATUS
#define AISNAPSHOT_IDLE             0
#define AISNAPSHOT_START            1
#define AISNAPSHOT_STOP             2

// Definition for buffer size
#define QUERY_INFO_TOTAL            32
#define MAX_FILENAME_SIZE           128
#define WIFI_RESERVED_SIZE          128

// Parameters for ai glass
static const char *ai_glass_disk_name = "aiglass";
static uint8_t send_response_timer_setstop = 0;
static int usb_msc_initialed = 0;
static int pic_trans_status = AISNAPSHOT_IDLE;

static TimerHandle_t send_response_timer = NULL;
static SlidingWindow *uart_sliding_window = NULL;
static SemaphoreHandle_t send_response_timermutex = NULL;
static SemaphoreHandle_t video_proc_sema = NULL;
static struct msc_opts *disk_operation = NULL;

static uint8_t reserve_query_info[QUERY_INFO_TOTAL] = {0};
static uint8_t temp_file_name[MAX_FILENAME_SIZE] = {0};
static uint8_t temp_rfile_name[MAX_FILENAME_SIZE] = {0};
static uint8_t wifi_reserve_buf[WIFI_RESERVED_SIZE] = {0};

// Funtion Prototype
static void ai_glass_init_external_disk(void);
static void ai_glass_init_ram_disk(void);
void ai_glass_log_init(void);

// These functions are for testing ai glass with mass storage
#if defined(ENABLE_DISK_MASS_STORAGE) && ENABLE_DISK_MASS_STORAGE
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
#endif

static void aiglass_mass_storage_init(void)
{
#if defined(ENABLE_DISK_MASS_STORAGE) && ENABLE_DISK_MASS_STORAGE
	if (usb_msc_initialed == 0) {
		ai_glass_init_external_disk();
		int status = 0;
		_usb_init();

		status = wait_usb_ready();
		if (status != USBD_INIT_OK) {
			if (status == USBD_NOT_ATTACHED) {
				AI_GLASS_WARN("\r\n NO USB device attached\n");
			} else {
				AI_GLASS_WARN("\r\n USB init fail\n");
			}
			goto exit;
		}

		if (disk_operation == NULL) {
			disk_operation = malloc(sizeof(struct msc_opts));
		}
		if (disk_operation == NULL) {
			AI_GLASS_ERR("\r\n disk_operation malloc fail\n");
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
			AI_GLASS_ERR("USB MSC driver load fail.\n");
			usb_msc_initialed = 0;
		} else {
			AI_GLASS_INFO("USB MSC driver load done, Available heap [0x%x]\n", xPortGetFreeHeapSize());
			usb_msc_initialed = 1;
		}
	}
#endif
}

static void aiglass_mass_storage_deinit(void)
{
#if defined(ENABLE_DISK_MASS_STORAGE) && ENABLE_DISK_MASS_STORAGE
	if (usb_msc_initialed == 1) {
		usbd_msc_deinit();
		extern void _usb_deinit(void);
		_usb_deinit();
		usb_msc_initialed = 0;
	}
#endif
}

// These functions are for ai glass needed
static void check_cmd_sample_fun(uartcmdpacket_t *param)
{
	AI_GLASS_MSG("cmd sync_word = 0x%02x\r\n", param->uart_pkt.sync_word);
	AI_GLASS_MSG("cmd seq_number = 0x%02x\r\n", param->uart_pkt.seq_number);
	AI_GLASS_MSG("cmd length = 0x%04x\r\n", param->uart_pkt.length);
	AI_GLASS_MSG("cmd opcode = 0x%04x\r\n", param->uart_pkt.opcode);
	for (int i = 0; i < param->uart_pkt.length - 2; i++) {
		AI_GLASS_MSG("cmd data_buf[%d] = 0x%02x\r\n", i, param->uart_pkt.data_buf[i]);
	}
	AI_GLASS_MSG("cmd checksum = 0x%02x\r\n", param->uart_pkt.checksum);
	AI_GLASS_MSG("cmd exp_seq_number = 0x%02x\r\n", param->exp_seq_number);
}

// For UART_RX_OPC_CMD_QUERY_INFO
static void uart_service_get_query_info(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_QUERY_INFO\r\n");
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint8_t temp_protocal_version = 0;
	uint8_t temp_wifi_ic_type = 0;
	uint16_t temp_buff_size = 0;
	uint16_t temp_pic_size = 0;

	// The pic size do not include empty packet size and need to add this size
	if (query_pkt->length >= 6) {
		temp_pic_size = query_pkt->data_buf[0] | (query_pkt->data_buf[1] << 8);
		temp_buff_size = query_pkt->data_buf[2] | (query_pkt->data_buf[3] << 8);
	} else {
		AI_GLASS_WARN("query cmd invalid\r\n");
	}
	AI_GLASS_MSG("get Packet size = %d\r\n", temp_pic_size);
	AI_GLASS_MSG("get Buffer check size = %d\r\n", temp_buff_size);
	temp_pic_size += EMPTY_PACKET_LEN;
	temp_buff_size += EMPTY_PACKET_LEN;

	if (!IS_VALID_UART_BUF_SIZE(temp_buff_size)) {
		temp_buff_size = UART_MAX_BUF_SIZE;
	}
	uart_buff_size = temp_buff_size;
	temp_buff_size -= EMPTY_PACKET_LEN;

	if (!IS_VALID_UART_BUF_SIZE(temp_pic_size)) {
		temp_pic_size = UART_MAX_PIC_SIZE;
	}
	uart_pic_size = temp_pic_size;
	temp_pic_size -= EMPTY_PACKET_LEN;

	temp_protocal_version = uart_protocal_version;
	temp_wifi_ic_type = uart_wifi_ic_type;

	uart_params_t reserve_param = {
		.data = reserve_query_info,
		.length = QUERY_INFO_TOTAL - 1 - 1 - 2 - 2,
		.next = NULL
	};
	uart_params_t wifiic_param = {
		.data = &temp_wifi_ic_type,
		.length = 1,
		.next = &reserve_param
	};
	uart_params_t protoc_param = {
		.data = &temp_protocal_version,
		.length = 1,
		.next = &wifiic_param
	};
	uart_params_t bufchk_param = {
		.data = (uint8_t *)(&temp_buff_size),
		.length = 2,
		.next = &protoc_param
	};
	uart_params_t pic_param = {
		.data = (uint8_t *)(&temp_pic_size),
		.length = 2,
		.next = &bufchk_param
	};
	uart_send_packet(UART_TX_OPC_RESP_QUERY_INFO, &pic_param, false, 2000);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_QUERY_INFO\r\n");
}

// For UART_RX_OPC_CMD_POWER_DOWN
static void uart_service_get_power_down(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_POWER_DOWN\r\n");
	// Todo: get power down command
	int ret = 0;
	// Save filelist to EMMC
	ai_glass_init_external_disk();
	ret = extdisk_save_file_cntlist();
	AI_GLASS_MSG("Save FILE Cnt List status: %d\r\n", ret);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_POWER_DOWN\r\n");
}

// For UART_RX_OPC_CMD_GET_POWER_STATE
static void uart_service_get_power_state(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_POWER_STATE\r\n");
	uint8_t power_result = 0;
	switch (wifi_get_connect_status()) {
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
	uart_params_t power_param = {
		.data = &power_result,
		.length = 1,
		.next = NULL
	};
	uart_send_packet(UART_TX_OPC_RESP_GET_POWER_STATE, &power_param, false, 200);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_POWER_STATE\r\n");
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


// For UART_RX_OPC_CMD_UPDATE_WIFI_INFO
static void uart_service_update_wifi_info(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_UPDATE_WIFI_INFO\r\n");
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint8_t info_mode = query_pkt->data_buf[0];
	uint16_t info_size = query_pkt->data_buf[1] | (query_pkt->data_buf[2] << 8);
	ai_glass_snapshot_param_t temp_snapshot_param = {0};
	ai_glass_record_param_t temp_record_param = {0};
	uint16_t record_time = 0;
	uint8_t resp_stat = AI_GLASS_CMD_COMPLETE;
	AI_GLASS_MSG("info_mode = 0x%02x\r\n", info_mode);
	AI_GLASS_MSG("info_size = 0x%04x\r\n", info_size);

	switch (info_mode) {
	case UPDATE_DEFAULT_SNAPSHOT:
		AI_GLASS_MSG("Life snapshot param size = 0x%04x\r\n", sizeof(ai_glass_snapshot_param_t));
		media_get_life_snapshot_params(&temp_snapshot_param);
		AI_GLASS_INFO("Get LifeTime Snapshot Data\r\n");
		print_snapshot_data(&temp_snapshot_param);
		parser_life_snapshot_param(&temp_snapshot_param, &(query_pkt->data_buf[3]));
		if (media_update_life_snapshot_params(&temp_snapshot_param) != MEDIA_OK) {
			resp_stat = AI_GLASS_PARAMS_ERR;
		}
		media_get_life_snapshot_params(&temp_snapshot_param);
		AI_GLASS_INFO("Get LifeTime Snapshot Data Update Result\r\n");
		print_snapshot_data(&temp_snapshot_param);
		break;
	case UPDATE_DEFAULT_RECORD:
		AI_GLASS_MSG("Life record param size = 0x%04x\r\n", sizeof(ai_glass_record_param_t));
		parser_record_param(&temp_record_param, &(query_pkt->data_buf[3]));
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
		record_time = query_pkt->data_buf[3] | query_pkt->data_buf[4] << 8;
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
	uart_params_t info_pkt = {
		.data = &resp_stat,
		.length = 1,
		.next = NULL
	};
	uart_send_packet(UART_TX_OPC_RESP_UPDATE_WIFI_INFO, &info_pkt, false, 2000);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_UPDATE_WIFI_INFO\r\n");
}

// For UART_RX_OPC_CMD_SET_GPS
static void uart_service_set_gps(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_SET_GPS\r\n");
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);

	AI_GLASS_MSG("get UART_RX_OPC_CMD_SET_GPS packet length = %d\r\n", query_pkt->length);
	uint32_t gps_week = 0, gps_seconds = 0;
	float gps_latitude = 0, gps_longitude = 0, gps_altitude = 0;
	uint8_t *raw_buf = (uint8_t *)query_pkt->data_buf;

	memcpy(&gps_latitude, raw_buf, sizeof(gps_latitude));
	raw_buf += sizeof(gps_latitude);
	memcpy(&gps_longitude, raw_buf, sizeof(gps_longitude));
	raw_buf += sizeof(gps_longitude);
	memcpy(&gps_altitude, raw_buf, sizeof(gps_altitude));
	raw_buf += sizeof(gps_altitude);

	memcpy(&gps_week, raw_buf, sizeof(gps_week));
	raw_buf += sizeof(gps_week);
	memcpy(&gps_seconds, raw_buf, sizeof(gps_seconds));
	raw_buf += sizeof(gps_seconds);

	media_filesystem_setup_gpstime(gps_week, gps_seconds);
	media_filesystem_setup_gpscoordinate(gps_latitude, gps_longitude, gps_altitude);
	uint8_t status = 0;
	uart_params_t set_gps_status_param = {
		.data = &status,
		.length = 1,
		.next = NULL
	};
	status = AI_GLASS_CMD_COMPLETE;
	uart_send_packet(UART_TX_OPC_RESP_SET_GPS, &set_gps_status_param, false, 2000);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_SET_GPS\r\n");
}

// For UART_RX_OPC_CMD_SNAPSHOT
static void uart_service_snapshot(uartcmdpacket_t *param)
{
	AI_GLASS_MSG("get UART_RX_OPC_CMD_SNAPSHOT = %lu\r\n", mm_read_mediatime_ms());
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);

	uint8_t status = 0;
	uint8_t mode = query_pkt->data_buf[0];
	AI_GLASS_MSG("%s get mode = %d\r\n", __func__, mode);
	uart_params_t snapshot_status_param = {
		.data = &status,
		.length = 1,
		.next = NULL
	};
	if (xSemaphoreTake(video_proc_sema, 0) != pdTRUE) {
		status = AI_GLASS_BUSY;
		AI_GLASS_WARN("AI glass is snapshot or record, current snapshot busy fail\r\n");
		goto endofsnapshot;
	}
	AI_GLASS_MSG("snapshot aiglass_mass_storage_deinit time = %lu\r\n", mm_read_mediatime_ms());
	if (mode == 1) {
		AI_GLASS_MSG("Process AI SNAPSHOT\r\n");

		ai_glass_snapshot_param_t ai_snap_params = {0};
		media_get_ai_snapshot_params(&ai_snap_params);
		parser_snapshot_pkt2param(&ai_snap_params, &(query_pkt->data_buf[1]));
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
		uart_send_packet(UART_TX_OPC_RESP_SNAPSHOT, &snapshot_status_param, false, 2000);
		if (ret == 0) {
			while (ai_snapshot_deinitialize()) {
				AI_GLASS_MSG("wait for ai snapshot deinit\r\n");
				vTaskDelay(1);
			}
		}
	} else if (mode == 0) {
		aiglass_mass_storage_deinit();
		ai_glass_init_external_disk();
		AI_GLASS_MSG("Process LIFETIME SNAPSHOT\r\n");

		int ret = lifetime_snapshot_initialize();
		if (ret == 0) {
			char *cur_time_str = (char *)media_filesystem_get_current_time_string();
			char temp_buffer[128] = {0};
			uint8_t lifetime_snap_name[128] = {0};
			extdisk_generate_unique_filename("lifesnap_", cur_time_str, ".jpg", (char *)temp_buffer, 128);
			snprintf((char *)lifetime_snap_name, sizeof(lifetime_snap_name), "%s%s", (const char *)temp_buffer, ".jpg");
			free(cur_time_str);
			if (lifetime_snapshot_take((const char *)lifetime_snap_name) == 0) {
				status = AI_GLASS_CMD_COMPLETE;
			} else {
				status = AI_GLASS_PROC_FAIL;
			}
			while (lifetime_snapshot_deinitialize()) {
				AI_GLASS_MSG("wait for ai snapshot deinit\r\n");
				vTaskDelay(1);
			}
		} else if (ret == -2) {
			status = AI_GLASS_BUSY;
		} else {
			status = AI_GLASS_PROC_FAIL;
		}
		uart_send_packet(UART_TX_OPC_RESP_SNAPSHOT, &snapshot_status_param, false, 2000);
		// Save filelist to EMMC
		extdisk_save_file_cntlist();
		aiglass_mass_storage_init();
	} else {
		AI_GLASS_WARN("Not implement yet\r\n");
		status = AI_GLASS_PROC_FAIL;
		uart_send_packet(UART_TX_OPC_RESP_SNAPSHOT, &snapshot_status_param, false, 2000);
	}
	xSemaphoreGive(video_proc_sema);
endofsnapshot:
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_SNAPSHOT\r\n");
}

// For UART_RX_OPC_CMD_GET_FILE_NAME
static void uart_get_file_name(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_FILE_NAME\r\n");
	uint8_t result = AI_GLASS_CMD_COMPLETE;
	uint16_t name_length = strlen("ai_snapshot.jpg");
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
		ai_snapshot_file = NULL;
		result = AI_GLASS_CMD_COMPLETE;
	} else {
		result = AI_GLASS_PROC_FAIL;
	}

	uart_params_t filename_param = {
		.data = temp_file_name,
		.length = strlen((const char *)temp_file_name),
		.next = NULL
	};
	uart_params_t filelen_param = {
		.data = (uint8_t *)(&file_length),
		.length = 4,
		.next = &filename_param
	};
	uart_params_t length_param = {
		.data = (uint8_t *)(&name_length),
		.length = 2,
		.next = &filelen_param
	};
	uart_params_t result_param = {
		.data = (uint8_t *)(&result),
		.length = 1,
		.next = &length_param
	};
	uart_send_packet(UART_TX_OPC_RESP_GET_FILE_NAME, &result_param, false, 2000);

	AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_FILE_NAME\r\n");
}

// For UART_RX_OPC_CMD_GET_PICTURE_DATA Todo
static void uart_get_pic_data(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_PICTURE_DATA\r\n");
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint32_t file_offset = query_pkt->data_buf[0] | (query_pkt->data_buf[1] << 8) | (query_pkt->data_buf[2] << 16) | (query_pkt->data_buf[3] << 24);
	uint8_t packet_num = query_pkt->data_buf[4];

	memset(temp_rfile_name, 0x0, MAX_FILENAME_SIZE);
	snprintf((char *)temp_rfile_name, MAX_FILENAME_SIZE, "ai_snapshot.jpg");

	uint16_t data_length = 0;
	AI_GLASS_MSG("temp_rfile_name = %s\r\n", temp_rfile_name);
	AI_GLASS_MSG("file_offset = %lu\r\n", file_offset);
	AI_GLASS_MSG("packet_num = %u\r\n", packet_num);
	uint16_t tmp_uart_pic_size = uart_pic_size - EMPTY_PACKET_LEN;
	uint8_t *data_buffer = malloc(tmp_uart_pic_size);
	uint8_t num_data = 0;
	uint8_t result = AI_GLASS_CMD_COMPLETE;

	if (data_buffer) {
		if (pic_trans_status == AISNAPSHOT_IDLE) {
			pic_trans_status = AISNAPSHOT_START;
			uart_params_t filedata_param = {
				.data = data_buffer,
				.length = tmp_uart_pic_size,
				.next = NULL
			};
			uart_params_t filelength_param = {
				.data = (uint8_t *)(&data_length),
				.length = 2,
				.next = &filedata_param
			};
			uart_params_t cnt_param = {
				.data = (uint8_t *)(&num_data),
				.length = 1,
				.next = &filelength_param
			};
			FILE *ai_snapshot_rfile = NULL;
			ai_snapshot_rfile = ramdisk_fopen((const char *)temp_rfile_name, "rb");
			if (ai_snapshot_rfile != NULL) {
				ramdisk_fseek(ai_snapshot_rfile, file_offset, SEEK_SET);
				for (uint8_t i = 0; i < packet_num; i++) {
					if (pic_trans_status == AISNAPSHOT_STOP) {
						AI_GLASS_MSG("Get stop\r\n");
						break;
					}
					data_length = 0;
					memset(data_buffer, 0x00, tmp_uart_pic_size);
					int bytesRead = fread(data_buffer, 1, tmp_uart_pic_size, ai_snapshot_rfile);
					if (bytesRead > 0) {
						data_length = bytesRead;
						filedata_param.length = data_length;
					} else {
						AI_GLASS_MSG("bytesRead = %d\r\n", bytesRead);
						break;
					}
					if (ramdisk_feof(ai_snapshot_rfile)) {
						num_data = 0xFF;
						uart_send_packet(UART_TX_OPC_RESP_GET_PICTURE_DATA, &cnt_param, false, 2000);
						break;
					} else {
						num_data = i;
					}
					uart_send_packet(UART_TX_OPC_RESP_GET_PICTURE_DATA, &cnt_param, false, 2000);
				}
				ramdisk_fclose(ai_snapshot_rfile);
				ai_snapshot_rfile = NULL;
				result = AI_GLASS_CMD_COMPLETE;
			} else {
				result = AI_GLASS_PROC_FAIL;
			}
			free(data_buffer);
			pic_trans_status = AISNAPSHOT_IDLE;
		} else {
			free(data_buffer);
			result = AI_GLASS_BUSY;
		}
	} else {
		result = AI_GLASS_PROC_FAIL;
	}

	AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_PICTURE_DATA\r\n");
}

// For UART_RX_OPC_CMD_TRANS_PIC_STOP Todo
static void uart_get_trans_pic_stop(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_TRANS_PIC_STOP\r\n");
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint8_t result = AI_GLASS_CMD_COMPLETE;
	uart_params_t result_param = {
		.data = &result,
		.length = 1,
		.next = NULL
	};
	if (pic_trans_status == AISNAPSHOT_START) {
		pic_trans_status = AISNAPSHOT_STOP;
		while (pic_trans_status == AISNAPSHOT_STOP) {
			vTaskDelay(1);
		}
		result = AI_GLASS_CMD_COMPLETE;
	} else if (pic_trans_status == AISNAPSHOT_IDLE) {
		result = AI_GLASS_CMD_COMPLETE;
	} else {
		result = AI_GLASS_BUSY;
	}
	uart_send_packet(UART_TX_OPC_RESP_TRANS_PIC_STOP, &result_param, false, 2000);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_TRANS_PIC_STOP\r\n");
}

// For UART_RX_OPC_CMD_RECORD_SYNC_TS
static void uart_service_record_sync_ts(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_RECORD_SYNC_TS\r\n");
	// Todo: will use in the future
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_RECORD_SYNC_TS\r\n");
}

// For UART_RX_OPC_CMD_RECORD_START Todo no need two timer
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
				uart_params_t record_resp_pkt = {
					.data = &record_resp_status,
					.length = 1,
					.next = NULL
				};
				lifetime_recording_deinitialize();
				aiglass_mass_storage_init();
				send_response_timer_setstop = 1;
				xSemaphoreGive(send_response_timermutex);
				uart_send_packet(UART_TX_OPC_RESP_RECORD_STOP, &record_resp_pkt, false, 2000);
				AI_GLASS_MSG("mp4_send_response_callback UART_TX_OPC_RESP_RECORD_STOP %lu\r\n", mm_read_mediatime_ms());
				xSemaphoreGive(video_proc_sema);
			} else {
				if (current_state == STATE_RECORDING || current_state == STATE_IDLE) {
					record_resp_status = AI_GLASS_CMD_COMPLETE;
				}
				uart_params_t record_resp_pkt = {
					.data = &record_resp_status,
					.length = 1,
					.next = NULL
				};
				uart_send_packet(UART_TX_OPC_RESP_RECORD_CONT, &record_resp_pkt, false, 2000);
				AI_GLASS_MSG("mp4_send_response_callback UART_TX_OPC_RESP_RECORD_CONT %lu\r\n", mm_read_mediatime_ms());
				if (send_response_timer != NULL) {
					if (xTimerStart(send_response_timer, 0) != pdPASS) {
						AI_GLASS_ERR("Send UART_TX_OPC_RESP_RECORD_CONT timer failed\r\n");
					}
				}
				xSemaphoreGive(send_response_timermutex);
			}
		} else {
			xSemaphoreGive(send_response_timermutex);
		}
	} else {
		AI_GLASS_ERR("Send UART_TX_OPC_RESP_RECORD_CONT timer mutex failed\r\n");
	}
	return;
}
static void uart_service_record_start(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_RECORD_START = %lu\r\n", mm_read_mediatime_ms());
	ai_glass_init_external_disk();
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	AI_GLASS_MSG("Opcode (hex): 0x%x\r\n", query_pkt->opcode);
	uint8_t record_start_status = AI_GLASS_CMD_COMPLETE;
	uart_params_t record_start_pkt = {
		.data = &record_start_status,
		.length = 1,
		.next = NULL
	};

	//Initialize function has a timer that constantly reads the status of MP4.
	if (xSemaphoreTake(video_proc_sema, 0) == pdTRUE) {
		aiglass_mass_storage_deinit();
		AI_GLASS_MSG("Record aiglass_mass_storage_deinit time = %lu\r\n", mm_read_mediatime_ms());
		if (current_state == STATE_RECORDING || current_state == STATE_END_RECORDING) {
			AI_GLASS_MSG("Recording has started, not starting another recording\r\n");
			record_start_status = AI_GLASS_CMD_COMPLETE;
			aiglass_mass_storage_init();
			uart_send_packet(UART_RX_OPC_CMD_RECORD_START, &record_start_pkt, false, 2000);
			xSemaphoreGive(video_proc_sema);
		} else if (current_state == STATE_IDLE) {
			lifetime_recording_initialize();
			// Save filelist to EMMC
			extdisk_save_file_cntlist();
			if (send_response_timer != NULL) {
				if (xSemaphoreTake(send_response_timermutex, portMAX_DELAY) == pdTRUE) {
					if (xTimerStart(send_response_timer, 0) != pdPASS) {
						record_start_status = AI_GLASS_PROC_FAIL;
						uart_send_packet(UART_RX_OPC_CMD_RECORD_START, &record_start_pkt, false, 2000);
						AI_GLASS_ERR("Send UART_RX_OPC_CMD_RECORD_START timer failed\r\n");
						lifetime_recording_deinitialize();
						aiglass_mass_storage_init();
						xSemaphoreGive(video_proc_sema);
					} else {
						record_start_status = AI_GLASS_CMD_COMPLETE;
						send_response_timer_setstop = 0;
						uart_send_packet(UART_RX_OPC_CMD_RECORD_START, &record_start_pkt, false, 2000);
					}
					xSemaphoreGive(send_response_timermutex);
				} else {
					record_start_status = AI_GLASS_PROC_FAIL;
					uart_send_packet(UART_RX_OPC_CMD_RECORD_START, &record_start_pkt, false, 2000);
					AI_GLASS_ERR("Send UART_RX_OPC_CMD_RECORD_START timer mutex failed\r\n");
					lifetime_recording_deinitialize();
					aiglass_mass_storage_init();
					xSemaphoreGive(video_proc_sema);
				}
			} else {
				record_start_status = AI_GLASS_PROC_FAIL;
				uart_send_packet(UART_RX_OPC_CMD_RECORD_START, &record_start_pkt, false, 2000);
				AI_GLASS_ERR("Failed to create send_response_timer\r\n");
				aiglass_mass_storage_init();
				xSemaphoreGive(video_proc_sema);
			}
		} else {
			record_start_status = AI_GLASS_PROC_FAIL;
			uart_send_packet(UART_RX_OPC_CMD_RECORD_START, &record_start_pkt, false, 2000);
			AI_GLASS_ERR("Failed because of the known record status\r\n");
			aiglass_mass_storage_init();
			xSemaphoreGive(video_proc_sema);
		}
	} else {
		AI_GLASS_WARN("AI glass is snapshot or record, current record busy fail\r\n");
		record_start_status = AI_GLASS_BUSY;
		uart_send_packet(UART_RX_OPC_CMD_RECORD_START, &record_start_pkt, false, 2000);
	}

	AI_GLASS_INFO("end of UART_RX_OPC_CMD_RECORD_START\r\n");
}

// For UART_RX_OPC_CMD_RECORD_STOP Todo
static void uart_service_record_stop(uartcmdpacket_t *param)
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
				aiglass_mass_storage_init();
				xSemaphoreGive(video_proc_sema);
				send_response_timer_setstop = 1;
				xSemaphoreGive(send_response_timermutex);
			} else {
				AI_GLASS_MSG("The recording timer has stop\r\n");
				xSemaphoreGive(send_response_timermutex);
			}
		}
	}
	uart_params_t record_stop_pkt = {
		.data = &record_stop_status,
		.length = 1,
		.next = NULL
	};
	uart_send_packet(UART_TX_OPC_RESP_RECORD_STOP, &record_stop_pkt, false, 2000);
	AI_GLASS_MSG("end of UART_RX_OPC_CMD_RECORD_STOP %lu\r\n", mm_read_mediatime_ms());
}

// For UART_RX_OPC_CMD_SET_WIFI_MODE
static void uart_set_ap_mode(uartcmdpacket_t *param)
{
	AI_GLASS_MSG("get UART_RX_OPC_CMD_SET_WIFI_MODE %lu\r\n", mm_read_mediatime_ms());
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint8_t mode = query_pkt->data_buf[0];
	uint8_t result = AI_GLASS_CMD_COMPLETE;
	rtw_softap_info_t wifi_cfg = {0};
	uint8_t ssid_length = 0;
	uint32_t security_type = 0;
	uint8_t password[MAX_AP_PASSWORD_LEN] = {0};
	wifi_cfg.password = password;
	uint8_t password_length = 0;
	uint8_t channel = 0;
	if (mode == 1) {
		ai_glass_init_external_disk();
		if (wifi_enable_ap_mode(AI_GLASS_AP_SSID, AI_GLASS_AP_PASSWORD, AI_GLASS_AP_CHANNEL, 20) == WLAN_SET_OK) {
			wifi_get_ap_setting(&wifi_cfg);
			ssid_length = wifi_cfg.ssid.len;
			security_type = wifi_cfg.security_type;
			password_length = wifi_cfg.password_len;
			channel = wifi_cfg.channel;
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
	uart_params_t resv1_param = {
		.data = (uint8_t *)wifi_reserve_buf,
		.length = 5,
		.next = NULL
	};
	uart_params_t password_param = {
		.data = (uint8_t *)password,
		.length = MAX_AP_PASSWORD_LEN,
		.next = &resv1_param
	};
	uart_params_t password_len_param = {
		.data = (uint8_t *)(&password_length),
		.length = 1,
		.next = &password_param
	};
	uart_params_t channel_param = {
		.data = (uint8_t *)(&channel),
		.length = 1,
		.next = &password_len_param
	};
	uart_params_t sectype_param = {
		.data = (uint8_t *)(&security_type),
		.length = 4,
		.next = &channel_param
	};
	uart_params_t resv0_param = {
		.data = (uint8_t *)wifi_reserve_buf,
		.length = 1,
		.next = &sectype_param
	};
	uart_params_t ssid_param = {
		.data = (uint8_t *)wifi_cfg.ssid.val,
		.length = MAX_AP_SSID_VALUE_LEN,
		.next = &resv0_param
	};
	uart_params_t ssidlen_param = {
		.data = (uint8_t *)(&ssid_length),
		.length = 1,
		.next = &ssid_param
	};
	uart_params_t result_param = {
		.data = (uint8_t *)(&result),
		.length = 1,
		.next = &ssidlen_param
	};
	uart_send_packet(UART_TX_OPC_RESP_SET_WIFI_MODE, &result_param, false, 2000);
	if (mode == 1 && result == AI_GLASS_CMD_COMPLETE) {
		deinitial_media(); // To save power
	}
	AI_GLASS_MSG("end of UART_RX_OPC_CMD_SET_WIFI_MODE %lu\r\n", mm_read_mediatime_ms());
}

// For UART_RX_OPC_CMD_GET_SD_INFO
static void uart_get_sd_info(uartcmdpacket_t *param)
{
	ai_glass_init_external_disk();
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_SD_INFO\r\n");
	uint64_t device_used_bytes = fatfs_get_used_space_byte();
	uint64_t device_total_bytes = device_used_bytes + fatfs_get_free_space_byte();
	uint32_t device_used_Kbytes = (uint32_t)(device_used_bytes / 1024);
	uint32_t device_total_Kbytes = (uint32_t)(device_total_bytes / 1024);

	AI_GLASS_MSG("Get device memory: %lu/%luKB\r\n", device_used_Kbytes, device_total_Kbytes);
	uart_params_t total_param = {
		.data = (uint8_t *)(&device_total_Kbytes),
		.length = 4,
		.next = NULL
	};
	uart_params_t used_param = {
		.data = (uint8_t *)(&device_used_Kbytes),
		.length = 4,
		.next = &total_param
	};
	uart_send_packet(UART_TX_OPC_RESP_GET_SD_INFO, &used_param, false, 2000);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_SD_INFO\r\n");
}

// For UART_RX_OPC_CMD_DELETE_FILE Todo
static void uart_delete_file(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_DELETE_FILE\r\n");
	ai_glass_init_external_disk();
	// Todo: will use in the future
	//check_cmd_sample_fun(param);
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);

	AI_GLASS_INFO("end of UART_RX_OPC_CMD_DELETE_FILE\r\n");
}

// For UART_RX_OPC_CMD_DELETE_ALL_FILES Todo
static void uart_delete_all_file(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_DELETE_ALL_FILES\r\n");
	ai_glass_init_external_disk();
	// Todo: will use in the future
	//check_cmd_sample_fun(param);
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);

	AI_GLASS_INFO("end of UART_RX_OPC_CMD_DELETE_ALL_FILES\r\n");
}

// For UART_RX_OPC_CMD_GET_FILE_CNT
static void uart_get_file_cnt(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_FILE_CNT\r\n");
	ai_glass_init_external_disk();
	uint8_t result = AI_GLASS_CMD_COMPLETE;
	uint16_t film_num = extdisk_get_filecount(SYS_COUNT_FILM_LABEL);
	uint16_t snapshot_num = extdisk_get_filecount(SYS_COUNT_PIC_LABEL);

	AI_GLASS_MSG("mp4 file num = %u\r\n", film_num);
	AI_GLASS_MSG("jpg file num = %u\r\n", snapshot_num);

	uart_params_t videofile_param = {
		.data = (uint8_t *)(&film_num),
		.length = 2,
		.next = NULL
	};
	uart_params_t snapshotfile_param = {
		.data = (uint8_t *)(&snapshot_num),
		.length = 2,
		.next = &videofile_param
	};
	uart_params_t result_param = {
		.data = (uint8_t *)(&result),
		.length = 1,
		.next = &snapshotfile_param
	};
	uart_send_packet(UART_TX_OPC_RESP_GET_FILE_CNT, &result_param, false, 2000);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_FILE_CNT\r\n");
}

static int send_packet(const sliding_pkt_t *spacket)
{
	// For sending ai-snapshot, the last image means this transmitision need to end
	if (spacket->spayload->label & PACKET_LABEL_EOF) {
		spacket->spayload->label |= PACKET_LABEL_FIN;
	}
	uart_params_t payload_param = {
		.data = (uint8_t *)(spacket->spayload->payload),
		.length = spacket->spayload->length,
		.next = NULL
	};
	uart_params_t packet_param = {
		.data = (uint8_t *)(spacket->spayload),
		.length = sizeof(uint8_t) + sizeof(uint8_t) * 3 + sizeof(uint16_t),
		.next = &payload_param
	};
	uart_params_t seq_param = {
		.data = (uint8_t *)(&(spacket->seq)),
		.length = 4,
		.next = &packet_param
	};
	AI_GLASS_MSG("send seq %u \r\n", spacket->seq);
	AI_GLASS_MSG("send label %02x \r\n", spacket->spayload->label);
	AI_GLASS_MSG("send pkt length %u \r\n", spacket->spayload->length);
	uart_send_packet(UART_TX_OPC_RESP_GET_PICTURE_DATA_SLIDING_WINDOW, &seq_param, true, 200);
	return 0;
}

static void uart_get_pic_data_sliding_window(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_PICTURE_DATA SLIDING WINDOW\r\n");
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint32_t file_offset = query_pkt->data_buf[0] | (query_pkt->data_buf[1] << 8) | (query_pkt->data_buf[2] << 16) | (query_pkt->data_buf[3] << 24);
	uint32_t file_length = query_pkt->data_buf[4] | (query_pkt->data_buf[5] << 8) | (query_pkt->data_buf[6] << 16) | (query_pkt->data_buf[7] << 24);
	uint32_t start_pic_packet_seq_num = query_pkt->data_buf[8] | (query_pkt->data_buf[9] << 8) | (query_pkt->data_buf[10] << 16) | (query_pkt->data_buf[11] << 24);
	uint16_t max_window_size = query_pkt->data_buf[12] | (query_pkt->data_buf[13] << 8);

	AI_GLASS_MSG("file_offset = %lu\r\n", file_offset);
	AI_GLASS_MSG("file_length = %lu\r\n", file_length);
	AI_GLASS_MSG("start_pic_packet_seq_num = %lu\r\n", start_pic_packet_seq_num);
	AI_GLASS_MSG("max_window_size = %u\r\n", max_window_size);
	uint16_t tmp_pic_size = uart_pic_size - EMPTY_PACKET_LEN;
	uint16_t pic_buf_size = tmp_pic_size * MAX_WINDOW_SIZE;

	if (!uart_sliding_window) {
		uart_sliding_window = create_sliding_window(send_packet, max_window_size, start_pic_packet_seq_num, UART_MAX_PIC_SIZE - EMPTY_PACKET_LEN);
		sliding_update_seg_data_size(uart_sliding_window, tmp_pic_size);
	}

	uint8_t *data_buffer = malloc(pic_buf_size);
	int remain_size = file_length;
	AI_GLASS_MSG("first remain_size = %u\r\n", remain_size);
	memset(temp_rfile_name, 0x0, MAX_FILENAME_SIZE);
	snprintf((char *)temp_rfile_name, MAX_FILENAME_SIZE, "ai_snapshot.jpg");
	if (data_buffer) {
		if (pic_trans_status == AISNAPSHOT_IDLE) {
			pic_trans_status = AISNAPSHOT_START;
			FILE *ai_snapshot_rfile = NULL;
			ai_snapshot_rfile = ramdisk_fopen((const char *)temp_rfile_name, "rb");
			if (ai_snapshot_rfile != NULL) {
				ramdisk_fseek(ai_snapshot_rfile, file_offset, SEEK_SET);
				while (1) {
					if (pic_trans_status == AISNAPSHOT_STOP) {
						AI_GLASS_MSG("Get stop\r\n");
						break;
					}
					memset(data_buffer, 0x00, pic_buf_size);
					int bytesRead = ramdisk_fread(data_buffer, 1, remain_size > pic_buf_size ? pic_buf_size : remain_size, ai_snapshot_rfile);
					if (bytesRead > 0) {
						remain_size -= bytesRead;
						AI_GLASS_MSG("reamin %d, send = %d\r\n", remain_size, bytesRead);
						if (ramdisk_feof(ai_snapshot_rfile) || remain_size == 0) {
							AI_GLASS_MSG("send last frame = %d\r\n", bytesRead);
							sliding_send_data(uart_sliding_window, data_buffer, bytesRead, false, 4000);
							break;
						} else {
							sliding_send_data(uart_sliding_window, data_buffer, bytesRead, true, 4000);
						}
					} else {
						AI_GLASS_ERR("read file error bytesRead = %d\r\n", bytesRead);
						break;
					}
				}
				ramdisk_fclose(ai_snapshot_rfile);
				ai_snapshot_rfile = NULL;
				AI_GLASS_MSG("all data has been push into sliding window buffer\r\n");
			} else {
				AI_GLASS_ERR("all data push into sliding window buffer fail\r\n");
			}
			free(data_buffer);
			pic_trans_status = AISNAPSHOT_IDLE;
		} else {
			free(data_buffer);
			AI_GLASS_WARN("sliding window system busy\r\n");
		}
	} else {
		AI_GLASS_ERR("data buffer allocate fail\r\n");
	}
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_PICTURE_DATA SLIDING WINDOW END\r\n");
}

static void uart_get_pic_data_sliding_window_ack(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_PICTURE_DATA SLIDING WINDOW_ACK\r\n");
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);

	ack_info_t ack_info = {0};
	uint8_t *extend_ack_info = NULL;

	ack_info.ack_seq = query_pkt->data_buf[0] | (query_pkt->data_buf[1] << 8) | (query_pkt->data_buf[2] << 16) | (query_pkt->data_buf[3] << 24);
	ack_info.window_size = query_pkt->data_buf[4] | (query_pkt->data_buf[5] << 8) ;
	ack_info.label = query_pkt->data_buf[6];
	ack_info.extend_length = query_pkt->data_buf[8] | (query_pkt->data_buf[9] << 8) | (query_pkt->data_buf[10] << 16) | (query_pkt->data_buf[11] << 24);

	// length include 2 bytes opcode
	if (query_pkt->length > 14) {
		extend_ack_info = &query_pkt->data_buf[12];
	}

	AI_GLASS_MSG("length = %u\r\n", query_pkt->length);
	AI_GLASS_MSG("ack_seq = %lu\r\n", ack_info.ack_seq);
	AI_GLASS_MSG("window_size = %u\r\n", ack_info.window_size);
	AI_GLASS_MSG("packet_label = %u\r\n", ack_info.label);
	AI_GLASS_MSG("extend_length = %lu\r\n", ack_info.extend_length);

	if (uart_sliding_window) {
		sliding_on_ack_received(uart_sliding_window, &ack_info, extend_ack_info);
	}
	if (ack_info.label & PACKET_LABEL_FIN) {
		uint8_t result = AI_GLASS_CMD_COMPLETE;
		uart_params_t result_param = {
			.data = &result,
			.length = 1,
			.next = NULL
		};
		if (pic_trans_status == AISNAPSHOT_START) {
			pic_trans_status = AISNAPSHOT_STOP;
			while (pic_trans_status == AISNAPSHOT_STOP) {
				vTaskDelay(1);
			}
			result = AI_GLASS_CMD_COMPLETE;
			delete_sliding_window(uart_sliding_window);
			uart_sliding_window = NULL;
		} else if (pic_trans_status == AISNAPSHOT_IDLE) {
			result = AI_GLASS_CMD_COMPLETE;
			delete_sliding_window(uart_sliding_window);
			uart_sliding_window = NULL;
		} else {
			result = AI_GLASS_BUSY;
		}
		uart_send_packet(UART_TX_OPC_RESP_TRANS_PIC_STOP, &result_param, false, 2000);
	}
}

void uart_fun_regist(void)
{
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_QUERY_INFO, uart_service_get_query_info);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_POWER_DOWN, uart_service_get_power_down);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_GET_POWER_STATE, uart_service_get_power_state);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_UPDATE_WIFI_INFO, uart_service_update_wifi_info);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_SET_GPS, uart_service_set_gps);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_SNAPSHOT, uart_service_snapshot);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_GET_FILE_NAME, uart_get_file_name);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_GET_PICTURE_DATA, uart_get_pic_data);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_TRANS_PIC_STOP, uart_get_trans_pic_stop);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_RECORD_START, uart_service_record_start);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_RECORD_SYNC_TS, uart_service_record_sync_ts);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_RECORD_STOP, uart_service_record_stop);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_GET_FILE_CNT, uart_get_file_cnt);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_DELETE_FILE, uart_delete_file);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_DELETE_ALL_FILES, uart_delete_all_file);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_GET_SD_INFO, uart_get_sd_info);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_SET_WIFI_MODE, uart_set_ap_mode);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW, uart_get_pic_data_sliding_window);
	uart_service_rx_cmd_reg(UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW_ACK, uart_get_pic_data_sliding_window_ack);
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
	AI_GLASS_MSG("Test Gyro Sensor Type: TDK ICM42670P/ICM42607P\n");
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
	AI_GLASS_MSG("Enable mass storage device\r\n");
	aiglass_mass_storage_init();
}

void fENABLEAPMODE(void *arg)
{
	ai_glass_init_external_disk();
	if (wifi_enable_ap_mode(AI_GLASS_AP_SSID, AI_GLASS_AP_PASSWORD, AI_GLASS_AP_CHANNEL, 20) == WLAN_SET_OK) {
		deinitial_media(); // For saving power
	}
}

void fDFILEENABLE(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};

	if (!arg) {
		AI_GLASS_ERR("\n\r[DFILEENABLE] Set up delete file after upload or not, DFILEENABLE = 0 or 1\n");
		return;
	}
	argc = parse_param(arg, argv);

	if (argc) {
		wifi_set_up_file_delete_flag(atoi(argv[1]));
	}
}

log_item_t at_ai_glass_items[ ] = {
	{"DISKFORMAT",      fDISKFORMAT,    {NULL, NULL}},
	{"TESTGSENSOR",     fTESTGSENSOR,   {NULL, NULL}},
	{"ENABLEMSC",       fENABLEMSC,     {NULL, NULL}},
	{"ENABLEAPMODE",    fENABLEAPMODE,  {NULL, NULL}},
	{"DFILEENABLE",     fDFILEENABLE,   {NULL, NULL}},
};
#endif
void ai_glass_log_init(void)
{
#if defined(ENABLE_TEST_CMD) && ENABLE_TEST_CMD
	log_service_add_table(at_ai_glass_items, sizeof(at_ai_glass_items) / sizeof(at_ai_glass_items[0]));
#endif
}

log_module_init(ai_glass_log_init);