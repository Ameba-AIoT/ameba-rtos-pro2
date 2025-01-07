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
#include "ai_glass_media.h"
#include "media_filesystem.h"
#include "vfs.h"
#include "fatfs_sdcard_api.h"
#include "log_service.h"

#define ENABLE_TEST_CMD             1   // For the tester to test some hardware
#define ENABLE_DISK_MASS_STORAGE    0   // For testing 8735 image and picture

#if defined(ENABLE_TEST_CMD) && ENABLE_TEST_CMD && defined(ENABLE_DISK_MASS_STORAGE) && ENABLE_DISK_MASS_STORAGE
#define ENABLE_VIDEO_SEND_LATER     0   // For testing 8735 without disable power immediately
#endif

#define EXTDISK_PLATFORM   VFS_INF_EMMC //VFS_INF_SD

#define UART_TX         PA_2
#define UART_RX         PA_3
#define UART_BAUDRATE   2000000 //115200 //2000000 //3750000 //4000000

static const char *ai_glass_disk_name = "aiglass";

static void ai_glass_init_external_disk(void);
static void ai_glass_init_ram_disk(void);

static TimerHandle_t send_response_timer = NULL;
static SemaphoreHandle_t send_response_timermutex = NULL;
static uint8_t send_response_timer_setstop = 0;
// these functions are for testing ai glass with mass storage
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

static int usb_msc_initialed = 0;
// these functions are for testing ai glass for disable ai glass later
#if defined(ENABLE_VIDEO_SEND_LATER) && ENABLE_VIDEO_SEND_LATER
static SemaphoreHandle_t vsend_sema = NULL;
#endif
#endif

static struct msc_opts *disk_operation = NULL;
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
				printf("\r\n NO USB device attached\n");
			} else {
				printf("\r\n USB init fail\n");
			}
			goto exit;
		}

		if (disk_operation == NULL) {
			disk_operation = malloc(sizeof(struct msc_opts));
		}
		if (disk_operation == NULL) {
			printf("\r\n disk_operation malloc fail\n");
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
			printf("USB MSC driver load fail.\n");
			usb_msc_initialed = 0;
		} else {
			printf("USB MSC driver load done, Available heap [0x%x]\n", xPortGetFreeHeapSize());
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

static void aiglass_init_video_send_end_semaphore(void)
{
#if defined(ENABLE_VIDEO_SEND_LATER) && ENABLE_VIDEO_SEND_LATER
	vsend_sema = xSemaphoreCreateBinary();
	if (vsend_sema == NULL) {
		printf("vsend_sema create fail\r\n");
	}
#endif
}

static int aiglass_get_video_send_end_semaphore(void)
{
#if defined(ENABLE_VIDEO_SEND_LATER) && ENABLE_VIDEO_SEND_LATER
	if (vsend_sema) {
		printf("Wait for get video end semaphore\r\n");
		return xSemaphoreTake(vsend_sema, portMAX_DELAY);
	}
#endif
	return pdTRUE;
}

static void aiglass_give_video_send_end_semaphore(void)
{
#if defined(ENABLE_VIDEO_SEND_LATER) && ENABLE_VIDEO_SEND_LATER
	if (vsend_sema) {
		printf("Give video end semaphore\r\n");
		xSemaphoreGive(vsend_sema);
	}
#endif
}

// these functions are for ai glass needed
static void check_cmd_sample_fun(uartcmdpacket_t *param)
{
	printf("cmd sync_word = 0x%02x\r\n", param->uart_pkt.sync_word);
	printf("cmd seq_number = 0x%02x\r\n", param->uart_pkt.seq_number);
	printf("cmd length = 0x%04x\r\n", param->uart_pkt.length);
	printf("cmd opcode = 0x%04x\r\n", param->uart_pkt.opcode);
	for (int i = 0; i < param->uart_pkt.length - 2; i++) {
		printf("cmd data_buf[%d] = 0x%02x\r\n", i, param->uart_pkt.data_buf[i]);
	}
	printf("cmd checksum = 0x%02x\r\n", param->uart_pkt.checksum);
	printf("cmd exp_seq_number = 0x%02x\r\n", param->exp_seq_number);
}

typedef struct temp_snapshot_pkt_s {
	uint8_t     version;
	uint8_t     q_vlaue;
	float       ROIX_TL;
	float       ROIY_TL;
	float       ROIX_BR;
	float       ROIY_BR;
	uint16_t    RESIZE_W;
	uint16_t    RESIZE_H;
} temp_snapshot_pkt_t;

// for UART_OPC_CMD_QUERY_INFO
#define QUERY_INFO_TOTAL 32
static uint8_t reserve_query_info[QUERY_INFO_TOTAL] = {0};
static void uart_service_get_query_info(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_QUERY_INFO\r\n");
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
		printf("[UART WARN] query cmd invalid\r\n");
	}
	printf("get Packet size = %d\r\n", temp_pic_size);
	printf("get Buffer check size = %d\r\n", temp_buff_size);
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
	uart_send_packet(UART_OPC_RESP_QUERY_INFO, &pic_param, 2000);
	printf("end of UART_OPC_CMD_QUERY_INFO\r\n");
}

// for UART_OPC_CMD_POWER_DOWN
static void uart_service_get_power_down(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_POWER_DOWN\r\n");
	// Todo: get power down command
	printf("end of UART_OPC_CMD_POWER_DOWN\r\n");
}

// for UART_OPC_CMD_GET_POWER_STATE
static void uart_service_get_power_state(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_GET_POWER_STATE\r\n");
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
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
	uart_send_packet(UART_OPC_RESP_GET_POWER_STATE, &power_param, 200);
	printf("end of UART_OPC_CMD_GET_POWER_STATE\r\n");
}

// for UART_OPC_CMD_UPDATE_WIFI_INFO
#define UPDATE_DEFAULT_SNAPSHOT 1
#define UPDATE_DEFAULT_RECORD   2
#define UPDATE_RECORD_TIME      3
static void uart_service_update_wifi_info(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_UPDATE_WIFI_INFO\r\n");
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint8_t info_mode = query_pkt->data_buf[0];
	uint16_t info_size = query_pkt->data_buf[1] | (query_pkt->data_buf[2] << 8);
	ai_glass_snapshot_param_t *temp_snapshot_param = NULL;
	ai_glass_record_param_t *temp_record_param = NULL;
	ai_glass_record_param_t temp_record;
	uint8_t resp_stat = AI_GLASS_CMD_COMPLETE;
	printf("info_mode = 0x%02x\r\n", info_mode);
	printf("info_size = 0x%04x\r\n", info_size);
	switch (info_mode) {
	case UPDATE_DEFAULT_SNAPSHOT:
		printf("Life snapshot param size = 0x%04x\r\n", sizeof(ai_glass_snapshot_param_t));
		if (info_size == sizeof(ai_glass_snapshot_param_t)) {
			temp_snapshot_param = (ai_glass_snapshot_param_t *)(&(query_pkt->data_buf[3]));
			if (media_update_life_snapshot_params(temp_snapshot_param) != MEDIA_OK) {
				resp_stat = AI_GLASS_PARAMS_ERR;
			}
			print_snapshot_data(temp_snapshot_param);
		} else {
			resp_stat = AI_GLASS_PARAMS_ERR;
		}
		break;
	case UPDATE_DEFAULT_RECORD:
		printf("Life record param size = 0x%04x\r\n", sizeof(ai_glass_record_param_t));
		if (info_size == sizeof(ai_glass_record_param_t)) {
			temp_record_param = (ai_glass_record_param_t *)(&(query_pkt->data_buf[3]));
			if (media_update_record_params(temp_record_param) != MEDIA_OK) {
				resp_stat = AI_GLASS_PARAMS_ERR;
			}
			print_record_data(temp_record_param);
		} else {
			resp_stat = AI_GLASS_PARAMS_ERR;
		}
		break;
	case UPDATE_RECORD_TIME:
		printf("Life record time = %d, info_size = %u\r\n", query_pkt->data_buf[3], info_size);
		if (info_size > 0) {
			if (media_update_record_time(query_pkt->data_buf[3]) != MEDIA_OK) {
				resp_stat = AI_GLASS_PARAMS_ERR;
			}
		} else {
			resp_stat = AI_GLASS_PARAMS_ERR;
		}
		media_get_record_params(&temp_record);
		print_record_data(&temp_record);
		break;
	}
	uart_params_t info_pkt = {
		.data = &resp_stat,
		.length = 1,
		.next = NULL
	};
	uart_send_packet(UART_OPC_RESP_UPDATE_WIFI_INFO, &info_pkt, 2000);
	printf("end of UART_OPC_CMD_UPDATE_WIFI_INFO\r\n");
}

// for UART_OPC_CMD_SET_GPS
static void uart_service_set_gps(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_SET_GPS\r\n");
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	//media_filesystem_setup_gpstime(uint32_t gps_week, uint32_t gps_seconds);
	uint8_t status = 0;
	uart_params_t set_gps_status_param = {
		.data = &status,
		.length = 1,
		.next = NULL
	};
	status = AI_GLASS_CMD_COMPLETE;
	uart_send_packet(UART_OPC_RESP_SET_GPS, &set_gps_status_param, 2000);
	printf("end of UART_OPC_CMD_SET_GPS\r\n");
}

// for UART_OPC_CMD_SNAPSHOT
#include "mmf2_mediatime_8735b.h"
static SemaphoreHandle_t video_proc_sema = NULL;
static void uart_service_snapshot(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_SNAPSHOT = %lu\r\n", mm_read_mediatime_ms());
	//check_cmd_sample_fun(param);
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);

	uint8_t status = 0;
	uint8_t mode = query_pkt->data_buf[0];
	temp_snapshot_pkt_t ai_snap_pkt_params;
	memcpy(&ai_snap_pkt_params, &(query_pkt->data_buf[1]), sizeof(temp_snapshot_pkt_t));
	printf("%s get mode = %d\r\n", __func__, mode);
	uart_params_t snapshot_status_param = {
		.data = &status,
		.length = 1,
		.next = NULL
	};
	if (xSemaphoreTake(video_proc_sema, 0) != pdTRUE) {
		status = AI_GLASS_BUSY;
		printf("AI glass is snapshot or record, current snapshot busy fail\r\n");
		goto endofsnapshot;
	}
	printf("snapshot aiglass_mass_storage_deinit time = %lu\r\n", mm_read_mediatime_ms());
	if (mode == 1) {
		//aiglass_mass_storage_deinit();
		printf("Process AI SNAPSHOT\r\n");
		printf("ai_snap_pkt_params\r\n");
		printf("version = %u\r\n", ai_snap_pkt_params.version);
		printf("q vlaue = %u\r\n", ai_snap_pkt_params.q_vlaue);
		printf("ROIX_TL = %f\r\n", ai_snap_pkt_params.ROIX_TL);
		printf("ROIY_TL = %f\r\n", ai_snap_pkt_params.ROIY_TL);
		printf("ROIX_BR = %f\r\n", ai_snap_pkt_params.ROIX_BR);
		printf("ROIY_BR = %f\r\n", ai_snap_pkt_params.ROIY_BR);
		printf("RESIZE_W = %u\r\n", ai_snap_pkt_params.RESIZE_W);
		printf("RESIZE_H = %u\r\n", ai_snap_pkt_params.RESIZE_H);

		ai_glass_snapshot_param_t ai_snap_params;
		media_get_ai_snapshot_params(&ai_snap_params);
		ai_snap_params.width = ai_snap_pkt_params.RESIZE_W;
		ai_snap_params.height = ai_snap_pkt_params.RESIZE_H;
		ai_snap_params.jpeg_qlevel = ai_snap_pkt_params.q_vlaue;
		if (media_update_ai_snapshot_params(&ai_snap_params) != MEDIA_OK) {
			printf("Invlaid parmaeters set to default value\r\n");
		}
		printf("snapshot initialed time = %lu\r\n", mm_read_mediatime_ms());
		int ret = ai_snapshot_initialize();
		if (ret == 0) {
			printf("snapshot take time = %lu\r\n", mm_read_mediatime_ms());
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
		printf("snapshot send pkt time = %lu\r\n", mm_read_mediatime_ms());
		uart_send_packet(UART_OPC_RESP_SNAPSHOT, &snapshot_status_param, 2000);
		//aiglass_mass_storage_init();
		if (ret == 0) {
			while (ai_snapshot_deinitialize()) {
				printf("wait for ai snapshot deinit\r\n");
				vTaskDelay(1);
			}
		}
	} else if (mode == 0) {
		aiglass_mass_storage_deinit();
		ai_glass_init_external_disk();
		printf("Process LIFETIME SNAPSHOT\r\n");
		printf("ai_snap_pkt_params\r\n");
		printf("version = %u\r\n", ai_snap_pkt_params.version);
		printf("q vlaue = %u\r\n", ai_snap_pkt_params.q_vlaue);
		printf("ROIX_TL = %f\r\n", ai_snap_pkt_params.ROIX_TL);
		printf("ROIY_TL = %f\r\n", ai_snap_pkt_params.ROIY_TL);
		printf("ROIX_BR = %f\r\n", ai_snap_pkt_params.ROIX_BR);
		printf("ROIY_BR = %f\r\n", ai_snap_pkt_params.ROIY_BR);
		printf("RESIZE_W = %u\r\n", ai_snap_pkt_params.RESIZE_W);
		printf("RESIZE_H = %u\r\n", ai_snap_pkt_params.RESIZE_H);

		int ret = lifetime_snapshot_initialize();
		if (ret == 0) {
			char *cur_time_str = (char *)media_filesystem_get_current_time_string();
			char temp_buffer[128] = {0};
			uint8_t lifetime_snap_name[128] = {0};
			//snprintf((char *)lifetime_snap_name, sizeof(lifetime_snap_name), "lifesnap_%s.jpg", cur_time_str);
			extdisk_generate_unique_filename("lifesnap_", cur_time_str, ".jpg", (char *)temp_buffer, 128);
			snprintf((char *)lifetime_snap_name, sizeof(lifetime_snap_name), "%s%s", (const char *)temp_buffer, ".jpg");
			free(cur_time_str);
			if (lifetime_snapshot_take((const char *)lifetime_snap_name) == 0) {
				status = AI_GLASS_CMD_COMPLETE;
			} else {
				status = AI_GLASS_PROC_FAIL;
			}
			while (lifetime_snapshot_deinitialize()) {
				printf("wait for ai snapshot deinit\r\n");
				vTaskDelay(1);
			}
		} else if (ret == -2) {
			status = AI_GLASS_BUSY;
		} else {
			status = AI_GLASS_PROC_FAIL;
		}
		uart_send_packet(UART_OPC_RESP_SNAPSHOT, &snapshot_status_param, 2000);
		aiglass_mass_storage_init();
		aiglass_get_video_send_end_semaphore();
	} else {
		printf("Not implement yet\r\n");
		status = AI_GLASS_PROC_FAIL;
		uart_send_packet(UART_OPC_RESP_SNAPSHOT, &snapshot_status_param, 2000);
	}
	xSemaphoreGive(video_proc_sema);
endofsnapshot:
	printf("end of UART_OPC_CMD_SNAPSHOT\r\n");
}

// for UART_OPC_CMD_GET_FILE_NAME
#define MAX_FILENAME_SIZE 128
static uint8_t temp_file_name[MAX_FILENAME_SIZE] = {0};
static void uart_get_file_name(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_GET_FILE_NAME\r\n");
	//check_cmd_sample_fun(param);
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint8_t result;
	uint16_t name_length = strlen("ai_snapshot.jpg");
	uint32_t file_length = 0;

	memset(temp_file_name, 0x0, MAX_FILENAME_SIZE);
	snprintf((char *)temp_file_name, MAX_FILENAME_SIZE, "ai_snapshot.jpg");

	FILE *ai_snapshot_file = NULL;
	printf("temp_file_name = %s\r\n", temp_file_name);
	ai_snapshot_file = ramdisk_fopen((const char *)temp_file_name, "rb");
	if (ai_snapshot_file != NULL) {
		fseek(ai_snapshot_file, 0, SEEK_END);
		file_length = ramdisk_ftell(ai_snapshot_file);
		fclose(ai_snapshot_file);
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
	uart_send_packet(UART_OPC_RESP_GET_FILE_NAME, &result_param, 2000);

	printf("end of UART_OPC_CMD_GET_FILE_NAME\r\n");
}

// for UART_OPC_CMD_GET_PICTURE_DATA Todo
#define AISNAPSHOT_IDLE         0
#define AISNAPSHOT_START        1
#define AISNAPSHOT_STOP         2
static int pic_trans_status = AISNAPSHOT_IDLE;
static uint8_t temp_rfile_name[MAX_FILENAME_SIZE] = {0};
static void uart_get_pic_data(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_GET_PICTURE_DATA\r\n");
	//check_cmd_sample_fun(param);
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint32_t file_offset = query_pkt->data_buf[0] | (query_pkt->data_buf[1] << 8) | (query_pkt->data_buf[2] << 16) | (query_pkt->data_buf[3] << 24);
	uint8_t packet_num = query_pkt->data_buf[4];

	memset(temp_rfile_name, 0x0, MAX_FILENAME_SIZE);
	snprintf((char *)temp_rfile_name, MAX_FILENAME_SIZE, "ai_snapshot.jpg");

	uint16_t data_length = 0;
	printf("temp_rfile_name = %s\r\n", temp_rfile_name);
	printf("file_offset = %lu\r\n", file_offset);
	printf("packet_num = %u\r\n", packet_num);
	uint16_t tmp_uart_pic_size = uart_pic_size - EMPTY_PACKET_LEN;
	uint8_t *data_buffer = malloc(tmp_uart_pic_size);
	uint8_t num_data = 0;
	uint8_t result = 0;

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
				fseek(ai_snapshot_rfile, file_offset, SEEK_SET);
				for (uint8_t i = 0; i < packet_num; i++) {
					if (pic_trans_status == AISNAPSHOT_STOP) {
						printf("Get stop\r\n");
						break;
					}
					data_length = 0;
					memset(data_buffer, 0x00, tmp_uart_pic_size);
					int bytesRead = fread(data_buffer, 1, tmp_uart_pic_size, ai_snapshot_rfile);
					if (bytesRead > 0) {
						data_length = bytesRead;
						filedata_param.length = data_length;
					} else {
						printf("bytesRead = %d\r\n", bytesRead);
						break;
					}
					if (ramdisk_feof(ai_snapshot_rfile)) {
						num_data = 0xFF;
						uart_send_packet(UART_OPC_RESP_GET_PICTURE_DATA, &cnt_param, 2000);
						break;
					} else {
						num_data = i;
					}
					uart_send_packet(UART_OPC_RESP_GET_PICTURE_DATA, &cnt_param, 2000);
				}
				fclose(ai_snapshot_rfile);
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

	printf("end of UART_OPC_CMD_GET_PICTURE_DATA\r\n");
}

// for UART_OPC_CMD_TRANS_PIC_STOP Todo
static void uart_get_trans_pic_stop(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_TRANS_PIC_STOP\r\n");
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint8_t result = 0;
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
	uart_send_packet(UART_OPC_RESP_TRANS_PIC_STOP, &result_param, 2000);
	printf("end of UART_OPC_CMD_TRANS_PIC_STOP\r\n");
}

// for UART_OPC_CMD_RECORD_SYNC_TS
static void uart_service_record_sync_ts(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_RECORD_SYNC_TS\r\n");
	// Todo: will use in the future
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	printf("end of UART_OPC_CMD_RECORD_SYNC_TS\r\n");
}

// for UART_OPC_CMD_RECORD_START Todo no need two timer
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
				aiglass_get_video_send_end_semaphore();
				uart_send_packet(UART_OPC_RESP_RECORD_STOP, &record_resp_pkt, 2000);
				printf("mp4_send_response_callback UART_OPC_RESP_RECORD_STOP %u\r\n", mm_read_mediatime_ms());
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
				uart_send_packet(UART_OPC_RESP_RECORD_CONT, &record_resp_pkt, 2000);
				printf("mp4_send_response_callback UART_OPC_RESP_RECORD_CONT %u\r\n", mm_read_mediatime_ms());
				if (send_response_timer != NULL) {
					if (xTimerStart(send_response_timer, 0) != pdPASS) {
						printf("Send UART_OPC_RESP_RECORD_CONT timer failed\r\n");
					}
				}
				xSemaphoreGive(send_response_timermutex);
			}
		} else {
			xSemaphoreGive(send_response_timermutex);
		}
	} else {
		printf("Send UART_OPC_RESP_RECORD_CONT timer mutex failed\r\n");
	}
	return;
}
static void uart_service_record_start(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_RECORD_START\r\n");
	ai_glass_init_external_disk();
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	printf("Opcode (hex): 0x%x\r\n", query_pkt->opcode);
	uint8_t record_start_status = AI_GLASS_CMD_COMPLETE;
	uart_params_t record_start_pkt = {
		.data = &record_start_status,
		.length = 1,
		.next = NULL
	};

	//Initialize function has a timer that constantly reads the status of MP4.
	if (xSemaphoreTake(video_proc_sema, 0) == pdTRUE) {
		aiglass_mass_storage_deinit();
		printf("snapshot aiglass_mass_storage_deinit time = %lu\r\n", mm_read_mediatime_ms());
		if (current_state == STATE_RECORDING || current_state == STATE_END_RECORDING) {
			printf("Recording has started, not starting another recording\r\n");
			record_start_status = AI_GLASS_CMD_COMPLETE;
			aiglass_mass_storage_init();
			uart_send_packet(UART_OPC_CMD_RECORD_START, &record_start_pkt, 2000);
			xSemaphoreGive(video_proc_sema);
		} else if (current_state == STATE_IDLE) {
			lifetime_recording_initialize();
			if (send_response_timer != NULL) {
				if (xSemaphoreTake(send_response_timermutex, portMAX_DELAY) == pdTRUE) {
					if (xTimerStart(send_response_timer, 0) != pdPASS) {
						record_start_status = AI_GLASS_PROC_FAIL;
						uart_send_packet(UART_OPC_CMD_RECORD_START, &record_start_pkt, 2000);
						printf("Send UART_OPC_CMD_RECORD_START timer failed\r\n");
						lifetime_recording_deinitialize();
						aiglass_mass_storage_init();
						xSemaphoreGive(video_proc_sema);
					} else {
						record_start_status = AI_GLASS_CMD_COMPLETE;
						send_response_timer_setstop = 0;
						uart_send_packet(UART_OPC_CMD_RECORD_START, &record_start_pkt, 2000);
					}
					xSemaphoreGive(send_response_timermutex);
				} else {
					record_start_status = AI_GLASS_PROC_FAIL;
					uart_send_packet(UART_OPC_CMD_RECORD_START, &record_start_pkt, 2000);
					printf("Send UART_OPC_CMD_RECORD_START timer mutex failed\r\n");
					lifetime_recording_deinitialize();
					aiglass_mass_storage_init();
					xSemaphoreGive(video_proc_sema);
				}
			} else {
				record_start_status = AI_GLASS_PROC_FAIL;
				uart_send_packet(UART_OPC_CMD_RECORD_START, &record_start_pkt, 2000);
				printf("Failed to create send_response_timer\r\n");
				aiglass_mass_storage_init();
				xSemaphoreGive(video_proc_sema);
			}
		} else {
			record_start_status = AI_GLASS_PROC_FAIL;
			uart_send_packet(UART_OPC_CMD_RECORD_START, &record_start_pkt, 2000);
			printf("Failed because of the known record status\r\n");
			aiglass_mass_storage_init();
			xSemaphoreGive(video_proc_sema);
		}
	} else {
		printf("AI glass is snapshot or record, current record busy fail\r\n");
		record_start_status = AI_GLASS_BUSY;
		uart_send_packet(UART_OPC_CMD_RECORD_START, &record_start_pkt, 2000);
	}

	printf("end of UART_OPC_CMD_RECORD_START\r\n");
}

// for UART_OPC_CMD_RECORD_STOP Todo
static void uart_service_record_stop(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_RECORD_STOP %u\r\n", mm_read_mediatime_ms());
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
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
				printf("The recording timer has stop\r\n");
				xSemaphoreGive(send_response_timermutex);
			}
		}
	}
	aiglass_get_video_send_end_semaphore();
	uart_params_t record_stop_pkt = {
		.data = &record_stop_status,
		.length = 1,
		.next = NULL
	};
	uart_send_packet(UART_OPC_RESP_RECORD_STOP, &record_stop_pkt, 2000);
	printf("end of UART_OPC_CMD_RECORD_STOP %u\r\n", mm_read_mediatime_ms());
}

// for UART_OPC_CMD_SET_WIFI_MODE
static uint8_t wifi_reserve_buf[111] = {0};
static void uart_set_ap_mode(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_SET_WIFI_MODE %u\r\n", mm_read_mediatime_ms());
	//check_cmd_sample_fun(param);
	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint8_t mode = query_pkt->data_buf[0];
	uint8_t result = UART_OK;
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
			result = UART_OK;
		} else {
			result = AI_GLASS_PROC_FAIL;
		}
	} else if (mode == 0) {
		if (wifi_disable_ap_mode() == WLAN_SET_OK) {
			result = UART_OK;
		} else {
			result = AI_GLASS_PROC_FAIL;
		}
	} else {
		result = AI_GLASS_PARAMS_ERR;
	}
	printf("UART_OPC_CMD_SET_WIFI_MODE set mode %d done %u\r\n", mode, mm_read_mediatime_ms());
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
	uart_send_packet(UART_OPC_RESP_SET_WIFI_MODE, &result_param, 2000);
	printf("end of UART_OPC_CMD_SET_WIFI_MODE %u\r\n", mm_read_mediatime_ms());
}

// for UART_OPC_CMD_GET_SD_INFO
static void uart_get_sd_info(uartcmdpacket_t *param)
{
	ai_glass_init_external_disk();
	//check_cmd_sample_fun(param);
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	printf("get UART_OPC_CMD_GET_SD_INFO\r\n");
	uint64_t device_used_bytes = fatfs_get_used_space_byte();
	uint64_t device_total_bytes = device_used_bytes + fatfs_get_free_space_byte();
	uint32_t device_used_Kbytes = (uint32_t)(device_used_bytes / 1024);
	uint32_t device_total_Kbytes = (uint32_t)(device_total_bytes / 1024);

	printf("Get device memory: %lu/%luKB\r\n", device_used_Kbytes, device_total_Kbytes);
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
	uart_send_packet(UART_OPC_RESP_GET_SD_INFO, &used_param, 2000);
	printf("end of UART_OPC_CMD_GET_SD_INFO\r\n");
}

// for UART_OPC_CMD_DELETE_FILE Todo
static void uart_delete_file(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_DELETE_FILE\r\n");
	ai_glass_init_external_disk();
	// Todo: will use in the future
	//check_cmd_sample_fun(param);
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);

	printf("end of UART_OPC_CMD_DELETE_FILE\r\n");
}

// for UART_OPC_CMD_DELETE_ALL_FILES Todo
static void uart_delete_all_file(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_DELETE_ALL_FILES\r\n");
	ai_glass_init_external_disk();
	// Todo: will use in the future
	//check_cmd_sample_fun(param);
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);

	printf("end of UART_OPC_CMD_DELETE_ALL_FILES\r\n");
}

// for UART_OPC_CMD_GET_FILE_CNT
static void uart_get_file_cnt(uartcmdpacket_t *param)
{
	printf("get UART_OPC_CMD_GET_FILE_CNT\r\n");
	ai_glass_init_external_disk();
	//check_cmd_sample_fun(param);
	//uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	const char *extensions[] = { ".mp4", ".jpeg", ".jpg" };
	uint16_t num_extensions = sizeof(extensions) / sizeof(extensions[0]);
	uint16_t counts[num_extensions];
	uint8_t result = AI_GLASS_CMD_COMPLETE;
	uint16_t video_num = 0;
	uint16_t snapshot_num = 0;

	extdisk_count_filenum("", extensions, counts, num_extensions, "ai_snapshot.jpg");
	printf("mp4 file num = %u\r\n", counts[0]);
	printf("jpeg file num = %u\r\n", counts[1]);
	printf("jpg file num = %u\r\n", counts[2]);

	video_num = counts[0];
	snapshot_num = counts[1] + counts[2];
	uart_params_t videofile_param = {
		.data = (uint8_t *)(&video_num),
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
	uart_send_packet(UART_OPC_RESP_GET_FILE_CNT, &result_param, 2000);
	printf("end of UART_OPC_CMD_GET_FILE_CNT\r\n");
}

void uart_fun_regist(void)
{
	uart_service_rx_cmd_reg(UART_OPC_CMD_QUERY_INFO, uart_service_get_query_info);
	uart_service_rx_cmd_reg(UART_OPC_CMD_POWER_DOWN, uart_service_get_power_down);
	uart_service_rx_cmd_reg(UART_OPC_CMD_GET_POWER_STATE, uart_service_get_power_state);
	uart_service_rx_cmd_reg(UART_OPC_CMD_UPDATE_WIFI_INFO, uart_service_update_wifi_info);
	uart_service_rx_cmd_reg(UART_OPC_CMD_SET_GPS, uart_service_set_gps);
	uart_service_rx_cmd_reg(UART_OPC_CMD_SNAPSHOT, uart_service_snapshot);
	uart_service_rx_cmd_reg(UART_OPC_CMD_GET_FILE_NAME, uart_get_file_name);
	uart_service_rx_cmd_reg(UART_OPC_CMD_GET_PICTURE_DATA, uart_get_pic_data);
	uart_service_rx_cmd_reg(UART_OPC_CMD_TRANS_PIC_STOP, uart_get_trans_pic_stop);
	uart_service_rx_cmd_reg(UART_OPC_CMD_RECORD_START, uart_service_record_start);
	uart_service_rx_cmd_reg(UART_OPC_CMD_RECORD_SYNC_TS, uart_service_record_sync_ts);
	uart_service_rx_cmd_reg(UART_OPC_CMD_RECORD_STOP, uart_service_record_stop);
	uart_service_rx_cmd_reg(UART_OPC_CMD_GET_FILE_CNT, uart_get_file_cnt);
	uart_service_rx_cmd_reg(UART_OPC_CMD_DELETE_FILE, uart_delete_file);
	uart_service_rx_cmd_reg(UART_OPC_CMD_DELETE_ALL_FILES, uart_delete_all_file);
	uart_service_rx_cmd_reg(UART_OPC_CMD_GET_SD_INFO, uart_get_sd_info);
	uart_service_rx_cmd_reg(UART_OPC_CMD_SET_WIFI_MODE, uart_set_ap_mode);
}

void ai_glass_log_init(void);
#include "mmf2_dbg.h"
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
	printf("ai_glass_service_thread start %lu\r\n", mm_read_mediatime_ms());

	initial_media_parameters();
	printf("media system done %lu\r\n", mm_read_mediatime_ms());

	// cost about 60ms into this function
	media_filesystem_init();
	media_filesystem_setup_gpstime(0, 0); // Set up GPS start time to prevent failed for file system
	ai_glass_init_ram_disk();
	//ai_glass_init_external_disk(); // init EMMC here will cause 160 ms delay
	printf("vfs system done %lu\r\n", mm_read_mediatime_ms());
	aiglass_init_video_send_end_semaphore();
	ai_glass_log_init();

	uart_service_init(UART_TX, UART_RX, UART_BAUDRATE);

	send_response_timer = xTimerCreate("send_response_timer", 100 / portTICK_PERIOD_MS, pdFALSE, NULL, mp4_send_response_callback);
	if (send_response_timer == NULL) {
		printf("send_response_timer create fail\r\n");
		goto exit;
	}
	send_response_timermutex = xSemaphoreCreateMutex();
	if (send_response_timermutex == NULL) {
		printf("send_response_timermutex create fail\r\n");
		goto exit;
	}
	video_proc_sema = xSemaphoreCreateBinary();
	if (video_proc_sema == NULL) {
		printf("video_proc_sema create fail\r\n");
		goto exit;
	}
	xSemaphoreGive(video_proc_sema);
	uart_fun_regist();
	uart_service_start(1);
	printf("uart service send data time %lu\r\n", mm_read_mediatime_ms());
exit:
	vTaskDelete(NULL);
}

void ai_glass_init(void)
{
	if (xTaskCreate(ai_glass_service_thread, ((const char *)"example_uart_service_thread"), 4096, NULL, tskIDLE_PRIORITY + 5, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_uart_service_thread) failed", __FUNCTION__);
	}
}

// The below command is for testing
#include "gyrosensor_api.h"
static gyro_data_t gdata[100] = {0};
void gyro_read_gsensor_thread(void *param)
{
	printf("Test Gyro Sensor Type: TDK ICM42670P/ICM42607P\n");
	gyroscope_fifo_init();
	while (1) {
		int read_cnt = gyroscope_fifo_read(gdata, 100);
		if (read_cnt > 0) {
			uint32_t cur_ts = mm_read_mediatime_ms();
			printf("timestamp: %lu\r\n", cur_ts + gdata[read_cnt - 1].timestamp);
#if !IGN_ACC_DATA
			printf("angular acceleration: X %f Y %f Z %f\r\n", gdata[read_cnt - 1].g[0], gdata[read_cnt - 1].g[1], gdata[read_cnt - 1].g[2]);
#endif
			printf("angular velocity: X %f Y %f Z %f\r\n", gdata[read_cnt - 1].dps[0], gdata[read_cnt - 1].dps[1], gdata[read_cnt - 1].dps[2]);
		}
		vTaskDelay(30);
	}

	free(gdata);
	vTaskDelete(NULL);
}

#if defined(ENABLE_TEST_CMD) && ENABLE_TEST_CMD
void fTESTGSENSOR(void *arg)
{
	if (xTaskCreate(gyro_read_gsensor_thread, ((const char *)"gyro_task"), 32 * 1024, NULL, tskIDLE_PRIORITY + 7, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(gyro_task) failed", __FUNCTION__);
	}
}

void fDISKFORMAT(void *arg)
{
	ai_glass_init_external_disk();
	printf("Format disk to FAT32\r\n");
	int ret = vfs_user_format(ai_glass_disk_name, VFS_FATFS, EXTDISK_PLATFORM);
	if (ret == FR_OK) {
		printf("format successfully\r\n");
	} else {
		printf("format failed %d\r\n", ret);
	}
}

void fSENDVIDEOEND(void *arg)
{
	printf("SEND VIDEO END to disable 8735 after lifetime video recording or liftime snapshot\r\n");
#if defined(ENABLE_VIDEO_SEND_LATER) && ENABLE_VIDEO_SEND_LATER
	aiglass_give_video_send_end_semaphore();
#endif
}

void fENABLEMSC(void *arg)
{
	printf("Enable mass storage device\r\n");
	aiglass_mass_storage_init();
}

void fENABLEAPMODE(void *arg)
{
	rtw_softap_info_t wifi_cfg = {0};
	ai_glass_init_external_disk();
	if (wifi_enable_ap_mode(AI_GLASS_AP_SSID, AI_GLASS_AP_PASSWORD, AI_GLASS_AP_CHANNEL, 20) == WLAN_SET_OK) {
		//wifi_get_ap_setting(&wifi_cfg);
	}
}

log_item_t at_ai_glass_items[ ] = {
	{"DISKFORMAT",      fDISKFORMAT,    {NULL, NULL}},
	{"TESTGSENSOR",     fTESTGSENSOR,   {NULL, NULL}},
	{"SENDVIDEOEND",    fSENDVIDEOEND,  {NULL, NULL}},
	{"ENABLEMSC",       fENABLEMSC,     {NULL, NULL}},
	{"ENABLEAPMODE",    fENABLEAPMODE,  {NULL, NULL}},
};
#endif
void ai_glass_log_init(void)
{
#if defined(ENABLE_TEST_CMD) && ENABLE_TEST_CMD
	log_service_add_table(at_ai_glass_items, sizeof(at_ai_glass_items) / sizeof(at_ai_glass_items[0]));
#endif
}

log_module_init(ai_glass_log_init);