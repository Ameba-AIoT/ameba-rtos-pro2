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
#include "gyrosensor_api.h"

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
#define UPDATE_WIFI_AP_CREDENTIALS  4

char ssid_buf[MAX_SSID_LEN + 1] = {0};
char password_buf[MAX_PASSWORD_LEN + 1] = {0};
// Definition for buffer size
#define MAX_FILENAME_SIZE           128

// Parameters for ai glass
static const char *ai_glass_disk_name = "aiglass";
static uint8_t send_response_timer_setstop = 0;

static TimerHandle_t send_response_timer = NULL;
static TimerHandle_t send_audio_response_timer = NULL;
static SemaphoreHandle_t send_response_timermutex = NULL;
static SemaphoreHandle_t send_audio_response_timermutex = NULL;
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
static void ai_glass_deinit_external_disk(void);
static void ai_glass_init_ram_disk(void);
void ai_glass_log_init(void);

static char version_str[16] = {0};
static UpgradeInfo info;

static uint8_t g_current_wifi_mode = 0; 
static int dual_snapshot = 0;
volatile int total_burst = 1;
// These functions are for testing ai glass with mass storage
#include "usb.h"
#include "msc/inc/usbd_msc_config.h"
#include "msc/inc/usbd_msc.h"
#include "fatfs_ramdisk_api.h"

//log ext disk
#include "stdio_port_func.h"

#if EXTDISK_LOG
static void (*wputc)(phal_uart_adapter_t puart_adapter, uint8_t tx_data) = hal_uart_wputc;
static FILE *g_emmc_log_fp = NULL;
static bool g_emmc_log_enabled = false;
extern hal_uart_adapter_t log_uart;
static uint8_t s_log_flush_buf[1024];  // 1KB is plenty; loop until ring empty
#endif

char burst_names[MAX_BURST][64]; 
volatile int burst_count = 0;

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

#if EXTDISK_LOG
// 16 KB ring buffer for logging
#define LOG_RING_SIZE (16 * 1024)
static char log_ring[LOG_RING_SIZE];
static volatile size_t log_head = 0; // write index
static volatile size_t log_tail = 0; // read index

// Non-blocking ring write: returns 1 on success, 0 if full
static inline int log_ring_write(char c)
{
    size_t next = (log_head + 1) % LOG_RING_SIZE;
    if (next == log_tail) {
        // Ring full -> drop to avoid blocking
        return 0;
    }
    log_ring[log_head] = c;
    log_head = next;
    return 1;
}

static void log_emmc_putc(void *arg, char c)
{
    // Always send to UART (snapshot/record control depends on this)
    wputc((phal_uart_adapter_t)arg, c);

    // Mirror to RAM ring; never touch disk here
    if (g_emmc_log_enabled) {
        (void)log_ring_write(c);
    }
}
#endif

#if EXTDISK_LOG
static SemaphoreHandle_t log_flush_mutex = NULL;
static TaskHandle_t log_task_handle = NULL;

// Dequeue up to 'max' bytes into 'out'; returns count
static size_t log_ring_read_bulk(char *out, size_t max)
{
    size_t count = 0;
    while (count < max && log_tail != log_head) {
        out[count++] = log_ring[log_tail];
        log_tail = (log_tail + 1) % LOG_RING_SIZE;
    }
    return count;
}

static void log_task(void *param)
{
    char buf[4096];
    for (;;) {
        if (!g_emmc_log_enabled || g_emmc_log_fp == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        size_t n = log_ring_read_bulk(buf, sizeof(buf));
        if (n == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Serialize disk writes
        if (xSemaphoreTake(log_flush_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Best effort; if FS is busy, drop instead of blocking
            int w = extdisk_fwrite(buf, 1, n, g_emmc_log_fp);
            if (w == (int)n) {
                // Flush only every N bytes to reduce overhead
                static size_t bytes_since_flush = 0;
                bytes_since_flush += n;
                if (bytes_since_flush >= 8 * 1024) {
                    extdisk_fflush(g_emmc_log_fp);
                    bytes_since_flush = 0;
                }
            }
            xSemaphoreGive(log_flush_mutex);
        }

        // Yield regardless, so we never hog CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif

static void print_camera_config(const CameraConfig *cfg) {
    printf("=== CameraConfig Debug Dump ===\n");
    printf("imu_rate_hz = %d\n", cfg->imu_rate_hz);
    printf("enable_stabilization = %d\n", cfg->enable_stabilization);
    printf("stabilization_alpha = %f\n", cfg->stabilization_alpha);
    printf("crop_ratio_min = %f\n", cfg->crop_ratio_min);
    printf("crop_ratio_max = %f\n", cfg->crop_ratio_max);
    printf("rs_readout_time_ms = %f\n", cfg->rs_readout_time_ms);

    const FisheyeParams *fp = &cfg->ldc_params.fisheye_params;
    printf("RMS_error = %lf\n", fp->rms_error);

    for (int r = 0; r < 3; r++) {
        printf("camera_matrix[%d] = [%lf, %lf, %lf]\n",
               r,
               fp->camera_matrix[r][0],
               fp->camera_matrix[r][1],
               fp->camera_matrix[r][2]);
    }

    printf("distortion_coeffs = [%lf, %lf, %lf, %lf]\n",
           fp->distortion_coeffs[0],
           fp->distortion_coeffs[1],
           fp->distortion_coeffs[2],
           fp->distortion_coeffs[3]);

    printf("radial_distortion_limit = %lf\n", fp->radial_distortion_limit);
}

static void uart_pack_uint32(uint8_t *buf, size_t *offset, size_t buf_size, uint32_t value) {
    if (*offset + 4 > buf_size) return;
    buf[(*offset)++] = (uint8_t)(value & 0xFF);
    buf[(*offset)++] = (uint8_t)((value >> 8) & 0xFF);
    buf[(*offset)++] = (uint8_t)((value >> 16) & 0xFF);
    buf[(*offset)++] = (uint8_t)((value >> 24) & 0xFF);
}

static void uart_pack_float(uint8_t *buf, size_t *offset, size_t buf_size, float value) {
    int32_t raw = (int32_t)(value * FLOAT_SCALE);
    uart_pack_uint32(buf, offset, buf_size, (uint32_t)raw);
}

static void uart_pack_double(uint8_t *buf, size_t *offset, size_t buf_size, double value) {
    int64_t raw = (int64_t)(value * FLOAT_SCALE);
    if (*offset + 8 > buf_size) return;
    for (int i = 0; i < 8; i++) {
        buf[(*offset)++] = (uint8_t)((raw >> (8*i)) & 0xFF);
    }
}

static void uart_pack_bool(uint8_t *buf, size_t *offset, size_t buf_size, bool value) {
    if (*offset + 1 > buf_size) return;
    buf[(*offset)++] = value ? 1 : 0;
}

static size_t uart_serialize_camera_config(uint8_t *buf, size_t buf_size, const CameraConfig *cfg) {
    size_t offset = 0;

    // Top-level fields
    uart_pack_uint32(buf, &offset, buf_size, cfg->imu_rate_hz);
    uart_pack_bool(buf, &offset, buf_size, cfg->enable_stabilization ? 1 : 0);
    uart_pack_float(buf, &offset, buf_size, cfg->stabilization_alpha);
    uart_pack_float(buf, &offset, buf_size, cfg->crop_ratio_min);
    uart_pack_float(buf, &offset, buf_size, cfg->crop_ratio_max);
    uart_pack_float(buf, &offset, buf_size, cfg->rs_readout_time_ms);

    // Fisheye params
    const FisheyeParams *fp = &cfg->ldc_params.fisheye_params;
    uart_pack_double(buf, &offset, buf_size, fp->rms_error);

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            uart_pack_double(buf, &offset, buf_size, fp->camera_matrix[r][c]);
        }
    }

    for (int i = 0; i < 4; i++) {
        uart_pack_double(buf, &offset, buf_size, fp->distortion_coeffs[i]);
    }

    uart_pack_double(buf, &offset, buf_size, fp->radial_distortion_limit);

    return offset; // total bytes written
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

void ai_glass_init_external_disk(void)
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

int ai_glass_disk_reformat(void) {
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
	uint8_t     lifetime_file_name_len;
	char        lifetime_file_name[49];
	uint32_t    isp_exposure_time;
	uint16_t    isp_exposure_gain;
	uint16_t    isp_red_gain;
	uint16_t    isp_blue_gain;
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
		aisnap_buf.lifetime_file_name_len = raw_buf[23];

		
		memcpy(aisnap_buf.lifetime_file_name, &raw_buf[24], aisnap_buf.lifetime_file_name_len);

		aisnap_buf.isp_exposure_time = raw_buf[72] | (raw_buf[73] << 8) | (raw_buf[74] << 16) | (raw_buf[75] << 24);
		aisnap_buf.isp_exposure_gain = raw_buf[77] | (raw_buf[78] << 8) | (raw_buf[79] << 16) | (raw_buf[80] << 24);
		aisnap_buf.isp_red_gain = raw_buf[81] | (raw_buf[82] << 8) | (raw_buf[83] << 16);
		aisnap_buf.isp_blue_gain = raw_buf[84] | (raw_buf[85] << 8) | (raw_buf[86] << 16);

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
		AI_GLASS_MSG("LF_FILENAME_LENGTH = %u\r\n", aisnap_buf.lifetime_file_name_len);
		AI_GLASS_MSG("LF_FILENAME = %s\r\n", aisnap_buf.lifetime_file_name);
		AI_GLASS_MSG("isp_exposure_time = %lu\r\n", aisnap_buf.isp_exposure_time);
		AI_GLASS_MSG("isp_exposure_gain = %u\r\n", aisnap_buf.isp_exposure_gain);
		AI_GLASS_MSG("isp_red_gain = %u\r\n", aisnap_buf.isp_red_gain);
		AI_GLASS_MSG("isp_blue_gain = %u\r\n", aisnap_buf.isp_blue_gain);

		snap_buf->width = aisnap_buf.RESIZE_W;
		snap_buf->height = aisnap_buf.RESIZE_H;
		snap_buf->jpeg_qlevel = aisnap_buf.q_vlaue;
		snap_buf->roi.xmin = (uint32_t)(aisnap_buf.ROIX_TL * sensor_params[current_sensor_id].sensor_width);
		snap_buf->roi.ymin = (uint32_t)(aisnap_buf.ROIY_TL * sensor_params[current_sensor_id].sensor_height);
		snap_buf->roi.xmax = (uint32_t)(aisnap_buf.ROIX_BR * sensor_params[current_sensor_id].sensor_width);
		snap_buf->roi.ymax = (uint32_t)(aisnap_buf.ROIY_BR * sensor_params[current_sensor_id].sensor_height);
		snap_buf->status = aisnap_buf.status;
		snap_buf->lifetime_file_name_len = aisnap_buf.lifetime_file_name_len;
		memcpy(snap_buf->lifetime_file_name,
       		aisnap_buf.lifetime_file_name,
       		sizeof(snap_buf->lifetime_file_name));
		snap_buf->isp_exposure_time = aisnap_buf.isp_exposure_time;
		snap_buf->isp_exposure_gain = aisnap_buf.isp_exposure_gain;
		snap_buf->isp_red_gain = aisnap_buf.isp_red_gain;
		snap_buf->isp_blue_gain = aisnap_buf.isp_blue_gain;
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

// Check if heap OTA filename matches the requested version
static int heap_ota_version_matches(uint8_t mode, const char *version_str)
{
    #define OTA_FILE_WIFI_PREFIX "wifi_ota_v"
    #define OTA_FILE_BT_PREFIX   "bt_ota_v"
    #define OTA_FILE_BOOT_PREFIX "boot_ota_v"
	#define OTA_FILE_NN_PREFIX "nn_ota_v"
	#define OTA_FILE_ISP_IQ_PREFIX "isp_iq_ota_v"
    #define OTA_FILE_EXTENSION   ".bin"

    int found_wifi = 0, found_bt = 0, found_boot = 0, found_nn = 0, found_isp_iq = 0;

    if (!g_heap_ota_data) {
        AI_GLASS_ERR("Heap OTA data not initialized.\n");
        return 0;
    }

    // Check WiFi OTA
    if ((mode == 0x02 || mode == 0x03 || mode == 0x0A || mode == 0x0B || mode == 0x12 || mode == 0x013 || mode == 0x1A || mode == 0x1B) && strncmp(g_heap_ota_data->wifi_filename, OTA_FILE_WIFI_PREFIX, strlen(OTA_FILE_WIFI_PREFIX)) == 0) {

        char *start = g_heap_ota_data->wifi_filename + strlen(OTA_FILE_WIFI_PREFIX);
        char *end = strstr(start, OTA_FILE_EXTENSION);
        if (end) {
            size_t len = end - start;
            if (len < 16) {
                char ver_buf[16] = {0};
                strncpy(ver_buf, start, len);
                ver_buf[len] = '\0';
                if (strcmp(ver_buf, version_str) == 0) {
					found_wifi = 1;
				}
            }
        }
    }

    // Check Bootloader OTA
    if ((mode == 0x03 || mode == 0x0B || mode == 0x13 || mode == 0x1B) && strncmp(g_heap_ota_data->boot_filename, OTA_FILE_BOOT_PREFIX, strlen(OTA_FILE_BOOT_PREFIX)) == 0) {

        char *start = g_heap_ota_data->boot_filename + strlen(OTA_FILE_BOOT_PREFIX);
        char *end = strstr(start, OTA_FILE_EXTENSION);
        if (end) {
            size_t len = end - start;
            if (len < 16) {
                char ver_buf[16] = {0};
                strncpy(ver_buf, start, len);
                ver_buf[len] = '\0';
                if (strcmp(ver_buf, version_str) == 0) {
					found_boot = 1;
				}
            }
        }
    }

    // Check BT OTA
    if (mode == 0x04) {
    	if (strcmp(g_heap_ota_data->bt_version, version_str) == 0) {
			found_bt = 1;
		}
        
    }

	 // Check NN OTA
    if ((mode == 0x0A || mode == 0x0B || mode == 0x1A || mode == 0x1B) && strncmp(g_heap_ota_data->nn_filename, OTA_FILE_NN_PREFIX, strlen(OTA_FILE_NN_PREFIX)) == 0) {
    	char *start = g_heap_ota_data->nn_filename + strlen(OTA_FILE_NN_PREFIX);
        char *end = strstr(start, OTA_FILE_EXTENSION);
        if (end) {
            size_t len = end - start;
            if (len < 16) {
                char ver_buf[16] = {0};
                strncpy(ver_buf, start, len);
                ver_buf[len] = '\0';
                if (strcmp(ver_buf, version_str) == 0) {
					found_nn = 1;
				}
            }
        }
    }

	 // Check ISP IQ OTA
    if ((mode == 0x12 || mode == 0x13 || mode == 0x1A || mode == 0x1B) && strncmp(g_heap_ota_data->isp_iq_filename, OTA_FILE_ISP_IQ_PREFIX, strlen(OTA_FILE_ISP_IQ_PREFIX)) == 0) {
    	char *start = g_heap_ota_data->isp_iq_filename + strlen(OTA_FILE_ISP_IQ_PREFIX);
        char *end = strstr(start, OTA_FILE_EXTENSION);
        if (end) {
            size_t len = end - start;
            if (len < 16) {
                char ver_buf[16] = {0};
                strncpy(ver_buf, start, len);
                ver_buf[len] = '\0';
                if (strcmp(ver_buf, version_str) == 0) {
					found_isp_iq = 1;
				}
            }
        }
    }

    // Final check based on mode
    if ((mode == 0x02 && found_wifi) || (mode == 0x03 && found_wifi && found_boot) || (mode == 0x04 && found_bt) || (mode == 0x0A && found_wifi && found_nn) || (mode == 0x0B && found_wifi && found_boot && found_nn)
	 || (mode == 0x12 && found_wifi && found_isp_iq) || (mode == 0x13 && found_wifi && found_boot && found_isp_iq) || (mode == 0x1A && found_wifi && found_nn && found_isp_iq) || (mode == 0x1B && found_wifi && found_boot && found_nn && found_isp_iq)) {
        return 1; // version matches
    }

    AI_GLASS_ERR("OTA file name mismatch (mode=0x%02X, WiFi: %d, Boot: %d, NN: %d, ISP IQ: %d, BT: %d)\n", mode, found_wifi, found_boot, found_nn, found_isp_iq, found_bt);
    return 0;
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

		//erase next FW signature to make it boot from another FW image
		AI_GLASS_MSG("\n\rnext FW addr = 0x%08X", next_fw_addr);

		pbuf = malloc(buf_size);
		if (!pbuf) {
			AI_GLASS_ERR("\n\rAllocate buf fail");
			return 0;
		}

		// need to enter critical section to prevent executing the XIP code at first sector after we erase it.
		device_mutex_lock(RT_DEV_LOCK_FLASH);
		flash_stream_read(&flash, next_fw_addr, buf_size, pbuf);
		// NOT the first byte of ota signature to make it invalid
		pbuf[0] = ~(pbuf[0]);
		flash_erase_sector(&flash, next_fw_addr); 
		flash_burst_write(&flash, next_fw_addr, buf_size, pbuf);
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

static void ai_glass_get_set_sys_upgrade(uartcmdpacket_t *param)
{
    AI_GLASS_INFO("get UART_TX_OPC_CMD_TRANSFER_UPGRADE_DATA\r\n");

    // Print heap OTA info if it exists
    if (g_heap_ota_data && (g_heap_ota_data->wifi_data || g_heap_ota_data->bt_data)) {
        AI_GLASS_INFO("Heap OTA WiFi file: %s, size: %u, Heap OTA BOOT file: %s, size: %u, Heap OTA BT version: %s, size: %u, Heap OTA NN file: %s, size: %u, Heap OTA ISP IQ file: %s, size: %u\n",
                      g_heap_ota_data->wifi_filename,
                      g_heap_ota_data->wifi_length,
					  g_heap_ota_data->boot_filename,
                      g_heap_ota_data->boot_length,
					  g_heap_ota_data->bt_version,
                      g_heap_ota_data->bt_length,
					  g_heap_ota_data->nn_filename,
                      g_heap_ota_data->nn_length,
					  g_heap_ota_data->isp_iq_filename,
                      g_heap_ota_data->isp_iq_length);
    } else {
        AI_GLASS_INFO("No heap OTA data available.\n");
    }

    // Parse UART packet to UpgradeInfo
    info = uart_parser_version_and_upgradetype(param);

    // Print parsed UART upgrade info
    AI_GLASS_INFO("Parsed Upgrade Info: type=%u, version=%u.%u.%u.%u\n",
                  info.upgradetype,
                  info.version[0], info.version[1],
                  info.version[2], info.version[3]);
	uint8_t status = AI_GLASS_CMD_COMPLETE;
	uint8_t power_result = 0;
	uart_resp_request_sys_upgrade(status);
	AI_GLASS_INFO("After 8430 CMD acknowledgement\r\n");

	if (info.upgradetype == 0x02) {
		AI_GLASS_INFO("Start WiFI OTA\r\n");

		// Convert received version to a string
		snprintf(version_str, sizeof(version_str), "%u.%u.%u.%u",
				 info.version[0], info.version[1],
				 info.version[2], info.version[3]);

		AI_GLASS_INFO("WIFI version to be upgrade to: %s\r\n", version_str);

		if (heap_ota_version_matches(info.upgradetype, version_str)) {
			
			int ret = -1;
				ret = heap_update_ota(g_heap_ota_data->wifi_data, g_heap_ota_data->wifi_length);
				if (!ret) {
					AI_GLASS_MSG("\n\r Ready to reboot\n");
					if (g_heap_ota_data && (g_heap_ota_data->bt_data != NULL)) {
						power_result = UART_PWR_WIFI_OTA_SUCCESS_BT_OTA_READY;
					} else {
						power_result = UART_PWR_WIFI_OTA_SUCCESS;
					}
					
					uart_resp_get_power_state(param, power_result);
				} else if (ret == -2){
					AI_GLASS_ERR("\n\r Received OTA process cancellation\n");
					power_result = AI_GLASS_WIFI_CANCEL_OTA_PROCESS;
					uart_resp_get_power_state(param, power_result);
				} else if (ret == -3){
					AI_GLASS_ERR("\n\r Wifi OTA malloc failed\n");
					power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
					uart_resp_get_power_state(param, power_result);
				} else if (ret == -4){
					AI_GLASS_ERR("\n\r Wifi OTA older timestamp version error\n");
					power_result = AI_GLASS_WIFI_OLDER_TIMESTAMP_VERSION;
					uart_resp_get_power_state(param, power_result);
				} else {
					AI_GLASS_ERR("\n\r OTA Process Failed\n");
					power_result = AI_GLASS_WIFI_OTA_FAILED;
					uart_resp_get_power_state(param, power_result);
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

		if (heap_ota_version_matches(info.upgradetype, version_str)) {

			int ret = -1;
			ret = heap_update_boot_ota(g_heap_ota_data->boot_data, g_heap_ota_data->boot_length);
			if (!ret) {
				AI_GLASS_MSG("\n\r Bootloader OTA done. Continue to upgrade wifi firmware...\r\n");

				int ret = -1;
				ret = heap_update_ota(g_heap_ota_data->wifi_data, g_heap_ota_data->wifi_length);
				if (!ret) {
					AI_GLASS_MSG("\n\r Ready to reboot\n");
					if (g_heap_ota_data && (g_heap_ota_data->bt_data != NULL)) {
						power_result = UART_PWR_WIFI_OTA_SUCCESS_BT_OTA_READY;
					} else {
						power_result = UART_PWR_WIFI_OTA_SUCCESS;
					}
					uart_resp_get_power_state(param, power_result);
				} else if (ret == -2){
					AI_GLASS_ERR("\n\r Received OTA process cancellation\n");
					power_result = AI_GLASS_WIFI_CANCEL_OTA_PROCESS;
					uart_resp_get_power_state(param, power_result);
				} else if (ret == -3){
					AI_GLASS_ERR("\n\r Wifi OTA malloc failed\n");
					power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
					uart_resp_get_power_state(param, power_result);
				} else if (ret == -4){
					AI_GLASS_ERR("\n\r Wifi OTA older timestamp version error\n");
					power_result = AI_GLASS_WIFI_OLDER_TIMESTAMP_VERSION;
					uart_resp_get_power_state(param, power_result);
				} else {
					AI_GLASS_ERR("\n\r OTA Process Failed\n");
					power_result = AI_GLASS_WIFI_OTA_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
			} else {
				if (ret == -3) {
					AI_GLASS_ERR("\n\r OTA Bootloader malloc Failed\n");
					power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
				else {
					AI_GLASS_ERR("\n\r OTA Bootloader Process Failed\n");
					power_result = AI_GLASS_WIFI_BL_OTA_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
			}
		} else {
			AI_GLASS_ERR("OTA file name not found.\n");
			status = AI_GLASS_OTA_FILE_NOT_EXISTED;
			uart_resp_request_sys_upgrade(status);
		}
	} else if (info.upgradetype == 0x0A) {
		AI_GLASS_INFO("Start WIFI NN and WIFI OTA\r\n");

		// Convert received version to a string
		snprintf(version_str, sizeof(version_str), "%u.%u.%u.%u",
				 info.version[0], info.version[1],
				 info.version[2], info.version[3]);

		AI_GLASS_INFO("NN and WIFI version to be upgrade to: %s\r\n", version_str);

		if (heap_ota_version_matches(info.upgradetype, version_str)) {

			int ret = -1;
			ret = heap_update_nn_ota(g_heap_ota_data->nn_data, g_heap_ota_data->nn_length);
			if (!ret) {
				AI_GLASS_MSG("\n\r NN OTA done. Continue to upgrade wifi firmware...\r\n");

				int ret = -1;
				ret = heap_update_ota(g_heap_ota_data->wifi_data, g_heap_ota_data->wifi_length);
				if (!ret) {
					AI_GLASS_MSG("\n\r Ready to reboot\n");
					if (g_heap_ota_data && (g_heap_ota_data->bt_data != NULL)) {
						power_result = UART_PWR_WIFI_OTA_SUCCESS_BT_OTA_READY;
					} else {
						power_result = UART_PWR_WIFI_OTA_SUCCESS;
					}
					uart_resp_get_power_state(param, power_result);
				} else if (ret == -2){
					AI_GLASS_ERR("\n\r Received OTA process cancellation\n");
					power_result = AI_GLASS_WIFI_CANCEL_OTA_PROCESS;
					uart_resp_get_power_state(param, power_result);
				} else if (ret == -3){
					AI_GLASS_ERR("\n\r Wifi OTA malloc failed\n");
					power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
					uart_resp_get_power_state(param, power_result);
				} else if (ret == -4){
					AI_GLASS_ERR("\n\r Wifi OTA older timestamp version error\n");
					power_result = AI_GLASS_WIFI_OLDER_TIMESTAMP_VERSION;
					uart_resp_get_power_state(param, power_result);
				} else {
					AI_GLASS_ERR("\n\r OTA Process Failed\n");
					power_result = AI_GLASS_WIFI_OTA_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
			} else {
				if (ret == -3) {
					AI_GLASS_ERR("\n\r OTA NN malloc Failed\n");
					power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
				else {
					AI_GLASS_ERR("\n\r OTA NN Process Failed\n");
					power_result = AI_GLASS_WIFI_NN_OTA_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
			}
		} else {
			AI_GLASS_ERR("OTA file name not found.\n");
			status = AI_GLASS_OTA_FILE_NOT_EXISTED;
			uart_resp_request_sys_upgrade(status);
		}
	} else if (info.upgradetype == 0x0B) {
		AI_GLASS_INFO("Start WIFI NN, WIFI Bootloader and WIFI OTA\r\n");

		// Convert received version to a string
		snprintf(version_str, sizeof(version_str), "%u.%u.%u.%u",
				 info.version[0], info.version[1],
				 info.version[2], info.version[3]);

		AI_GLASS_INFO("NN, BOOT and WIFI version to be upgrade to: %s\r\n", version_str);

		if (heap_ota_version_matches(info.upgradetype, version_str)) {
			int ret = -1;
			ret = heap_update_boot_ota(g_heap_ota_data->boot_data, g_heap_ota_data->boot_length);
			if (!ret) {
				AI_GLASS_MSG("\n\r Bootloader OTA done. Continue to upgrade NN wifi firmware...\r\n");
				int ret = -1;
				ret = heap_update_nn_ota(g_heap_ota_data->nn_data, g_heap_ota_data->nn_length);
				if (!ret) {
					AI_GLASS_MSG("\n\r NN OTA done. Continue to upgrade wifi firmware...\r\n");
					int ret = -1;
					ret = heap_update_ota(g_heap_ota_data->wifi_data, g_heap_ota_data->wifi_length);
					if (!ret) {
						AI_GLASS_MSG("\n\r Ready to reboot\n");
						if (g_heap_ota_data && (g_heap_ota_data->bt_data != NULL)) {
							power_result = UART_PWR_WIFI_OTA_SUCCESS_BT_OTA_READY;
						} else {
							power_result = UART_PWR_WIFI_OTA_SUCCESS;
						}
						uart_resp_get_power_state(param, power_result);
					} else if (ret == -2){
						AI_GLASS_ERR("\n\r Received OTA process cancellation\n");
						power_result = AI_GLASS_WIFI_CANCEL_OTA_PROCESS;
						uart_resp_get_power_state(param, power_result);
					} else if (ret == -3){
						AI_GLASS_ERR("\n\r Wifi OTA malloc failed\n");
						power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
						uart_resp_get_power_state(param, power_result);
					} else if (ret == -4){
						AI_GLASS_ERR("\n\r Wifi OTA older timestamp version error\n");
						power_result = AI_GLASS_WIFI_OLDER_TIMESTAMP_VERSION;
						uart_resp_get_power_state(param, power_result);
					} else {
						AI_GLASS_ERR("\n\r OTA Process Failed\n");
						power_result = AI_GLASS_WIFI_OTA_FAILED;
						uart_resp_get_power_state(param, power_result);
					}
				} else {
					if (ret == -3) {
						AI_GLASS_ERR("\n\r OTA NN malloc Failed\n");
						power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
						uart_resp_get_power_state(param, power_result);
					}
					else {
						AI_GLASS_ERR("\n\r OTA NN Process Failed\n");
						power_result = AI_GLASS_WIFI_NN_OTA_FAILED;
						uart_resp_get_power_state(param, power_result);
					}
				}
			} else {
				if (ret == -3) {
					AI_GLASS_ERR("\n\r OTA Bootloader malloc Failed\n");
					power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
				else {
					AI_GLASS_ERR("\n\r OTA Bootloader Process Failed\n");
					power_result = AI_GLASS_WIFI_BL_OTA_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
			}
		} else {
			AI_GLASS_ERR("OTA file name not found.\n");
			status = AI_GLASS_OTA_FILE_NOT_EXISTED;
			uart_resp_request_sys_upgrade(status);
		}
	} else if (info.upgradetype == 0x12) {
		AI_GLASS_INFO("Start WIFI ISP IQ and WIFI OTA\r\n");

		// Convert received version to a string
		snprintf(version_str, sizeof(version_str), "%u.%u.%u.%u",
				 info.version[0], info.version[1],
				 info.version[2], info.version[3]);

		AI_GLASS_INFO("ISP IQ and WIFI version to be upgrade to: %s\r\n", version_str);

		if (heap_ota_version_matches(info.upgradetype, version_str)) {
			int ret = -1;
			ret = heap_update_isp_iq_ota(g_heap_ota_data->isp_iq_data, g_heap_ota_data->isp_iq_length);
			if (!ret) {
				AI_GLASS_MSG("\n\r ISP_IQ OTA done. Continue to upgrade wifi firmware...\r\n");
				int ret = -1;
				ret = heap_update_ota(g_heap_ota_data->wifi_data, g_heap_ota_data->wifi_length);
				if (!ret) {
					AI_GLASS_MSG("\n\r Ready to reboot\n");
					if (g_heap_ota_data && (g_heap_ota_data->bt_data != NULL)) {
						power_result = UART_PWR_WIFI_OTA_SUCCESS_BT_OTA_READY;
					} else {
						power_result = UART_PWR_WIFI_OTA_SUCCESS;
					}
					uart_resp_get_power_state(param, power_result);
				} else if (ret == -2){
						AI_GLASS_ERR("\n\r Received OTA process cancellation\n");
						power_result = AI_GLASS_WIFI_CANCEL_OTA_PROCESS;
						uart_resp_get_power_state(param, power_result);
					} else if (ret == -3){
						AI_GLASS_ERR("\n\r Wifi OTA malloc failed\n");
						power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
						uart_resp_get_power_state(param, power_result);
					} else if (ret == -4){
						AI_GLASS_ERR("\n\r Wifi OTA older timestamp version error\n");
						power_result = AI_GLASS_WIFI_OLDER_TIMESTAMP_VERSION;
						uart_resp_get_power_state(param, power_result);
					} else {
						AI_GLASS_ERR("\n\r OTA Process Failed\n");
						power_result = AI_GLASS_WIFI_OTA_FAILED;
						uart_resp_get_power_state(param, power_result);
					}
			} else {
				if (ret == -3) {
					AI_GLASS_ERR("\n\r OTA ISP IQ malloc Failed\n");
					power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
				else {
					AI_GLASS_ERR("\n\r OTA ISP IQ Process Failed\n");
					power_result = AI_GLASS_WIFI_ISP_IQ_OTA_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
			}
		} else {
			AI_GLASS_ERR("OTA file name not found.\n");
			status = AI_GLASS_OTA_FILE_NOT_EXISTED;
			uart_resp_request_sys_upgrade(status);
		}
	} else if (info.upgradetype == 0x13) {
		AI_GLASS_INFO("Start WIFI ISP IQ, WIFI Bootloader and WIFI OTA\r\n");

		// Convert received version to a string
		snprintf(version_str, sizeof(version_str), "%u.%u.%u.%u",
				 info.version[0], info.version[1],
				 info.version[2], info.version[3]);

		AI_GLASS_INFO("ISP IQ, BOOT and WIFI version to be upgrade to: %s\r\n", version_str);

		if (heap_ota_version_matches(info.upgradetype, version_str)) {
			int ret = -1;
			ret = heap_update_boot_ota(g_heap_ota_data->boot_data, g_heap_ota_data->boot_length);
			if (!ret) {
				AI_GLASS_MSG("\n\r Bootloader OTA done. Continue to upgrade ISP IQ wifi firmware...\r\n");
				int ret = -1;
				ret = heap_update_isp_iq_ota(g_heap_ota_data->isp_iq_data, g_heap_ota_data->isp_iq_length);
				if (!ret) {
					AI_GLASS_MSG("\n\r ISP_IQ OTA done. Continue to upgrade wifi firmware...\r\n");
					int ret = -1;
					ret = heap_update_ota(g_heap_ota_data->wifi_data, g_heap_ota_data->wifi_length);
					if (!ret) {
						AI_GLASS_MSG("\n\r Ready to reboot\n");
						if (g_heap_ota_data && (g_heap_ota_data->bt_data != NULL)) {
							power_result = UART_PWR_WIFI_OTA_SUCCESS_BT_OTA_READY;
						} else {
							power_result = UART_PWR_WIFI_OTA_SUCCESS;
						}
						uart_resp_get_power_state(param, power_result);
					} else if (ret == -2){
						AI_GLASS_ERR("\n\r Received OTA process cancellation\n");
						power_result = AI_GLASS_WIFI_CANCEL_OTA_PROCESS;
						uart_resp_get_power_state(param, power_result);
					} else if (ret == -3){
						AI_GLASS_ERR("\n\r Wifi OTA malloc failed\n");
						power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
						uart_resp_get_power_state(param, power_result);
					} else if (ret == -4){
						AI_GLASS_ERR("\n\r Wifi OTA older timestamp version error\n");
						power_result = AI_GLASS_WIFI_OLDER_TIMESTAMP_VERSION;
						uart_resp_get_power_state(param, power_result);
					} else {
						AI_GLASS_ERR("\n\r OTA Process Failed\n");
						power_result = AI_GLASS_WIFI_OTA_FAILED;
						uart_resp_get_power_state(param, power_result);
					}
				} else {
					if (ret == -3) {
						AI_GLASS_ERR("\n\r OTA ISP IQ malloc Failed\n");
						power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
						uart_resp_get_power_state(param, power_result);
					}
					else {
						AI_GLASS_ERR("\n\r OTA ISP IQ Process Failed\n");
						power_result = AI_GLASS_WIFI_ISP_IQ_OTA_FAILED;
						uart_resp_get_power_state(param, power_result);
					}
				}
			} else {
				if (ret == -3) {
					AI_GLASS_ERR("\n\r OTA Bootloader malloc Failed\n");
					power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
				else {
					AI_GLASS_ERR("\n\r OTA Bootloader Process Failed\n");
					power_result = AI_GLASS_WIFI_BL_OTA_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
			}
		} else {
			AI_GLASS_ERR("OTA file name not found.\n");
			status = AI_GLASS_OTA_FILE_NOT_EXISTED;
			uart_resp_request_sys_upgrade(status);
		}
	} else if (info.upgradetype == 0x1A) {
		AI_GLASS_INFO("Start WIFI NN, WIFI ISP IQ and WIFI OTA\r\n");

		// Convert received version to a string
		snprintf(version_str, sizeof(version_str), "%u.%u.%u.%u",
				 info.version[0], info.version[1],
				 info.version[2], info.version[3]);

		AI_GLASS_INFO("NN, ISP IQ and WIFI version to be upgrade to: %s\r\n", version_str);

		if (heap_ota_version_matches(info.upgradetype, version_str)) {
			int ret = -1;
			ret = heap_update_nn_ota(g_heap_ota_data->nn_data, g_heap_ota_data->nn_length);
			if (!ret) {
				AI_GLASS_MSG("\n\r NN OTA done. Continue to upgrade ISP IQ wifi firmware...\r\n");
				int ret = -1;
				ret = heap_update_isp_iq_ota(g_heap_ota_data->isp_iq_data, g_heap_ota_data->isp_iq_length);
				if (!ret) {
					AI_GLASS_MSG("\n\r ISP_IQ OTA done. Continue to upgrade wifi firmware...\r\n");
					int ret = -1;
					ret = heap_update_ota(g_heap_ota_data->wifi_data, g_heap_ota_data->wifi_length);
					if (!ret) {
						AI_GLASS_MSG("\n\r Ready to reboot\n");
						if (g_heap_ota_data && (g_heap_ota_data->bt_data != NULL)) {
							power_result = UART_PWR_WIFI_OTA_SUCCESS_BT_OTA_READY;
						} else {
							power_result = UART_PWR_WIFI_OTA_SUCCESS;
						}
						uart_resp_get_power_state(param, power_result);
					} else if (ret == -2){
						AI_GLASS_ERR("\n\r Received OTA process cancellation\n");
						power_result = AI_GLASS_WIFI_CANCEL_OTA_PROCESS;
						uart_resp_get_power_state(param, power_result);
					} else if (ret == -3){
						AI_GLASS_ERR("\n\r Wifi OTA malloc failed\n");
						power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
						uart_resp_get_power_state(param, power_result);
					} else if (ret == -4){
						AI_GLASS_ERR("\n\r Wifi OTA older timestamp version error\n");
						power_result = AI_GLASS_WIFI_OLDER_TIMESTAMP_VERSION;
						uart_resp_get_power_state(param, power_result);
					} else {
						AI_GLASS_ERR("\n\r OTA Process Failed\n");
						power_result = AI_GLASS_WIFI_OTA_FAILED;
						uart_resp_get_power_state(param, power_result);
					}
				} else {
					if (ret == -3) {
						AI_GLASS_ERR("\n\r OTA ISP IQ malloc Failed\n");
						power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
						uart_resp_get_power_state(param, power_result);
					}
					else {
						AI_GLASS_ERR("\n\r OTA ISP IQ Process Failed\n");
						power_result = AI_GLASS_WIFI_ISP_IQ_OTA_FAILED;
						uart_resp_get_power_state(param, power_result);
					}
				}
			} else {
				if (ret == -3) {
					AI_GLASS_ERR("\n\r OTA NN malloc Failed\n");
					power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
				else {
					AI_GLASS_ERR("\n\r OTA NN Process Failed\n");
					power_result = AI_GLASS_WIFI_NN_OTA_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
			}
		} else {
			AI_GLASS_ERR("OTA file name not found.\n");
			status = AI_GLASS_OTA_FILE_NOT_EXISTED;
			uart_resp_request_sys_upgrade(status);
		}
	} else if (info.upgradetype == 0x1B) {
		AI_GLASS_INFO("Start WIFI NN, WIFI Bootloader, WIFI ISP IQ and WIFI OTA\r\n");

		// Convert received version to a string
		snprintf(version_str, sizeof(version_str), "%u.%u.%u.%u",
				 info.version[0], info.version[1],
				 info.version[2], info.version[3]);

		AI_GLASS_INFO("NN, BOOT, ISP IQ and WIFI version to be upgrade to: %s\r\n", version_str);

		if (heap_ota_version_matches(info.upgradetype, version_str)) {
			int ret = -1;
			ret = heap_update_boot_ota(g_heap_ota_data->boot_data, g_heap_ota_data->boot_length);
			if (!ret) {
				AI_GLASS_MSG("\n\r Bootloader OTA done. Continue to upgrade NN wifi firmware...\r\n");
				int ret = -1;
				ret = heap_update_nn_ota(g_heap_ota_data->nn_data, g_heap_ota_data->nn_length);
				if (!ret) {
					AI_GLASS_MSG("\n\r NN OTA done. Continue to upgrade ISP IQ wifi firmware...\r\n");
					int ret = -1;
					ret = heap_update_isp_iq_ota(g_heap_ota_data->isp_iq_data, g_heap_ota_data->isp_iq_length);
					if (!ret) {
						AI_GLASS_MSG("\n\r ISP_IQ OTA done. Continue to upgrade wifi firmware...\r\n");
						int ret = -1;
						ret = heap_update_ota(g_heap_ota_data->wifi_data, g_heap_ota_data->wifi_length);
						if (!ret) {
							AI_GLASS_MSG("\n\r Ready to reboot\n");
							if (g_heap_ota_data && (g_heap_ota_data->bt_data != NULL)) {
								power_result = UART_PWR_WIFI_OTA_SUCCESS_BT_OTA_READY;
							} else {
								power_result = UART_PWR_WIFI_OTA_SUCCESS;
							}
							uart_resp_get_power_state(param, power_result);
							//ota_platform_reset();
						} else if (ret == -2){
						AI_GLASS_ERR("\n\r Received OTA process cancellation\n");
						power_result = AI_GLASS_WIFI_CANCEL_OTA_PROCESS;
						uart_resp_get_power_state(param, power_result);
						} else if (ret == -3){
							AI_GLASS_ERR("\n\r Wifi OTA malloc failed\n");
							power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
							uart_resp_get_power_state(param, power_result);
						} else if (ret == -4){
							AI_GLASS_ERR("\n\r Wifi OTA older timestamp version error\n");
							power_result = AI_GLASS_WIFI_OLDER_TIMESTAMP_VERSION;
							uart_resp_get_power_state(param, power_result);
						} else {
							AI_GLASS_ERR("\n\r OTA Process Failed\n");
							power_result = AI_GLASS_WIFI_OTA_FAILED;
							uart_resp_get_power_state(param, power_result);
						}
					} else {
						if (ret == -3) {
						AI_GLASS_ERR("\n\r OTA ISP IQ malloc Failed\n");
						power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
						uart_resp_get_power_state(param, power_result);
						}
						else {
							AI_GLASS_ERR("\n\r OTA ISP IQ Process Failed\n");
							power_result = AI_GLASS_WIFI_ISP_IQ_OTA_FAILED;
							uart_resp_get_power_state(param, power_result);
						}
					}
				} else {
					if (ret == -3) {
					AI_GLASS_ERR("\n\r OTA NN malloc Failed\n");
					power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
					uart_resp_get_power_state(param, power_result);
					}
					else {
						AI_GLASS_ERR("\n\r OTA NN Process Failed\n");
						power_result = AI_GLASS_WIFI_NN_OTA_FAILED;
						uart_resp_get_power_state(param, power_result);
					}
				}
			} else {
				if (ret == -3) {
					AI_GLASS_ERR("\n\r OTA Bootloader malloc Failed\n");
					power_result = AI_GLASS_WIFI_OTA_MALLOC_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
				else {
					AI_GLASS_ERR("\n\r OTA Bootloader Process Failed\n");
					power_result = AI_GLASS_WIFI_BL_OTA_FAILED;
					uart_resp_get_power_state(param, power_result);
				}
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
    int packet_count   = 0;
    uint32_t sendtime  = mm_read_mediatime_ms();

    wifi_off();
    vTaskDelay(20);

    AI_GLASS_INFO("Disabled WIFI\r\n");
    AI_GLASS_INFO("Sending bluetooth binary via UART...\r\n");

    // Check version match before proceeding
    if (heap_ota_version_matches(info.upgradetype, version_str)) {

        uint16_t tmp_uart_pic_size = uart_service_get_pic_size() - EMPTY_PACKET_LEN;
        uint32_t file_size = g_heap_ota_data->bt_length;
        uint32_t offset = 0;
        uint16_t data_length = 0;
        uint8_t data_buffer[1541] = {0};

#if UPDATE_UPGRADE_PROGRESS_TO_8773
        int total_bytes_sent = 0;
        uart_resp_get_sys_upgrade((uint8_t)2, (uint8_t)0);
        AI_GLASS_MSG("BT FW File_size: %lu\r\n", file_size);
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

            // Determine how many bytes remain and copy into buffer
            uint32_t remain = file_size - offset;
            if (remain == 0) {
                AI_GLASS_INFO("End of BT FW buffer reached.\r\n");
                break;
            }

            data_length = (remain > tmp_uart_pic_size) ? tmp_uart_pic_size : remain;
            memset(data_buffer, 0, tmp_uart_pic_size);
            memcpy(data_buffer, g_heap_ota_data->bt_data + offset, data_length);
            offset += data_length;

#if UPDATE_UPGRADE_PROGRESS_TO_8773
            total_bytes_sent += data_length;
            packet_count++;
            AI_GLASS_INFO("Total_bytes_sent: %u\r\n", total_bytes_sent);

            static uint8_t last_bt_progress = 0xFF;
            if (file_size > 0) {
                bt_progress = (uint8_t)((total_bytes_sent * 100) / file_size);
                AI_GLASS_INFO("BT progress: %u\r\n", bt_progress);
                if (bt_progress > 99) {
                    bt_progress = 99;
                }
            }
            if (bt_progress != last_bt_progress) {
                last_bt_progress = bt_progress;
                uart_resp_get_sys_upgrade((uint8_t)2, bt_progress);
                AI_GLASS_INFO("BT progress update: %u%% after %d packets\r\n",
                               bt_progress, packet_count);
            }
#endif

            AI_GLASS_MSG("[8735(2) Sending] Data Length: %d bytes, Data[0-2]: %02X %02X %02X\n",
                         data_length, data_buffer[0], data_buffer[1], data_buffer[2]);

            uart_resp_transfer_upgrade_data(data_buffer, data_length);
            vTaskDelay(pdMS_TO_TICKS(10));

            if (offset >= file_size) {
#if UPDATE_UPGRADE_PROGRESS_TO_8773
                uart_resp_get_sys_upgrade((uint8_t)2, (uint8_t)100);
                AI_GLASS_MSG("Send BT progress status 100\r\n");
#endif
                break;
            }
        }

        // Delay to ensure all packets are sent before starting BT SoC OTA
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

static void ai_glass_get_cancel_sys_upgrade(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_TX_OPC_RESP_CANCEL_SYS_UPGRADE\r\n");
	cancel_bt_upgrade = 1;
	cancel_wifi_upgrade = 1;
	AI_GLASS_INFO("end of UART_TX_OPC_RESP_CANCEL_SYS_UPGRADE\r\n");
}

static void ai_glass_resp_bt_fw_finish(uartcmdpacket_t *param)
{
	critical_process_started = 1;
	AI_GLASS_INFO("get UART_TX_OPC_CMD_FINISH_BT_SOC_FW_UPGRADE\r\n");

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
uint32_t start_pdtime = 0;
static int check = 0;
static void ai_glass_get_power_down(uartcmdpacket_t *param)
{
	if(check == 0) {
		start_pdtime = mm_read_mediatime_ms();
		check +=1;
	}
	uint8_t result = AI_GLASS_CMD_COMPLETE;
	AI_GLASS_INFO("get UART_RX_OPC_CMD_POWER_DOWN %lu\r\n", mm_read_mediatime_ms());
	if (critical_process_started == 1) {
		AI_GLASS_WARN("AI glass is busy performing OTA or AI+Lifetime snapshot or replying to GET_SD_INFO or deinitializing wifi, power down failed %lu\r\n", mm_read_mediatime_ms());
		result = AI_GLASS_BUSY;
		uart_resp_get_power_down(param, result);
		goto endofpowerdown;
	}
	else if (xSemaphoreTake(video_proc_sema, POWER_DOWN_TIMEOUT) != pdTRUE) {
		// Wait until the video is down
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
#if EXTDISK_LOG
    // Force push all pending logs before shutdown
    if (g_emmc_log_enabled && g_emmc_log_fp && log_flush_mutex &&
        xSemaphoreTake(log_flush_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        size_t n;
        while ((n = log_ring_read_bulk((char *)s_log_flush_buf, sizeof(s_log_flush_buf))) > 0) {
            extdisk_fwrite(s_log_flush_buf, 1, n, g_emmc_log_fp);
        }
        extdisk_fflush(g_emmc_log_fp);
        xSemaphoreGive(log_flush_mutex);
    }
#endif
	// Todo: get power down command
	uart_resp_get_power_down(param, result);
	xSemaphoreGive(video_proc_sema);
	uint32_t stop_pdtime = mm_read_mediatime_ms();
	uint32_t pdtime = stop_pdtime - start_pdtime;
	check = 0;
	printf("Final power down time: %lu\r\n",pdtime);
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

static void parser_stream_param(ai_glass_stream_param_t *rec_buf, uint8_t *raw_buf)
{
    if (rec_buf) {
		// Streaming type and resolution
		rec_buf->type   	= raw_buf[0];
		rec_buf->resolution = raw_buf[1];

		// Width and Height
		rec_buf->width  = raw_buf[2] | (raw_buf[3] << 8);
		rec_buf->height = raw_buf[4] | (raw_buf[5] << 8);

		// FPS and BPS
		rec_buf->fps 	= raw_buf[6] | (raw_buf[7] << 8) | (raw_buf[8] << 16) | (raw_buf[9] << 24);
		rec_buf->bps    = raw_buf[10] | (raw_buf[11] << 8) | (raw_buf[12] << 16) | (raw_buf[13] << 24);

		// QP
		rec_buf->minQp  = raw_buf[14] | (raw_buf[15] << 8);
		rec_buf->maxQp  = raw_buf[16] | (raw_buf[17] << 8);

		// Rotation and RC mode
		rec_buf->rotation = raw_buf[18];
		rec_buf->rc_mode  = raw_buf[19];

		// Set ROI, level, profile, cavlc to defaults (since packet does not carry them)
		rec_buf->roi.xmin = 0;
		rec_buf->roi.ymin = 0;
		rec_buf->roi.xmax = 1;
		rec_buf->roi.ymax = 1;
		rec_buf->gop      = raw_buf[6] | (raw_buf[7] << 8) | (raw_buf[8] << 16) | (raw_buf[9] << 24);

		// Audio type
		rec_buf->audio_type = raw_buf[20];

		rec_buf->h264_level    = DEFAULT_STREAM_H264_LEVEL;
		rec_buf->h264_profile  = DEFAULT_STREAM_H264_PROFILE;
		rec_buf->h265_level    = DEFAULT_STREAM_H265_LEVEL;
		rec_buf->h265_profile  = DEFAULT_STREAM_H265_PROFILE;
		rec_buf->cavlc    = DEFAULT_STREAM_CAVLC;
	} 
    
}

static void parser_rtsp_stream_param(ai_glass_stream_param_t *rec_buf, uint8_t *raw_buf)
{
    if (rec_buf) {
		// Streaming type and resolution
		rec_buf->type   	= raw_buf[4];
		rec_buf->resolution = raw_buf[5];

		// Width and Height
		rec_buf->width  = raw_buf[6] | (raw_buf[7] << 8);
		rec_buf->height = raw_buf[8] | (raw_buf[9] << 8);

		// FPS and BPS
		rec_buf->fps 	= raw_buf[10] | (raw_buf[11] << 8) | (raw_buf[12] << 16) | (raw_buf[13] << 24);
		rec_buf->bps    = raw_buf[14] | (raw_buf[15] << 8) | (raw_buf[16] << 16) | (raw_buf[17] << 24);

		// QP
		rec_buf->minQp  = raw_buf[18] | (raw_buf[19] << 8);
		rec_buf->maxQp  = raw_buf[20] | (raw_buf[21] << 8);

		// Rotation and RC mode
		rec_buf->rotation = raw_buf[22];
		rec_buf->rc_mode  = raw_buf[23];
		
		// Set ROI, level, profile, cavlc to defaults (since packet does not carry them)
		rec_buf->roi.xmin = 0;
		rec_buf->roi.ymin = 0;
		rec_buf->roi.xmax = 1;
		rec_buf->roi.ymax = 1;
		rec_buf->gop      = raw_buf[10] | (raw_buf[11] << 8) | (raw_buf[12] << 16) | (raw_buf[13] << 24);

		// Audio type
		rec_buf->audio_type = raw_buf[24];

		rec_buf->h264_level    = DEFAULT_STREAM_H264_LEVEL;
		rec_buf->h264_profile  = DEFAULT_STREAM_H264_PROFILE;
		rec_buf->h265_level    = DEFAULT_STREAM_H265_LEVEL;
		rec_buf->h265_profile  = DEFAULT_STREAM_H265_PROFILE;
		rec_buf->cavlc    = DEFAULT_STREAM_CAVLC;
	} 
    
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
	case UPDATE_WIFI_AP_CREDENTIALS:
		AI_GLASS_MSG("Update WiFi AP credentials\r\n");
		uint8_t ssid_length = video_params[0];
		if (ssid_length == 0 || ssid_length > MAX_SSID_LEN) {
        	resp_stat = AI_GLASS_PARAMS_ERR;
        	break;
    	}
		uint8_t password_length = video_params[34];
		if (password_length > MAX_PASSWORD_LEN) {
			resp_stat = AI_GLASS_PARAMS_ERR;
			break;
		}
		if (info_size < (2 + ssid_length + password_length)) {
			resp_stat = AI_GLASS_PARAMS_ERR;
			break;
		}
		ai_glass_wifi_param_t wifi_param = {0};
		media_get_wifi_params_from_flash(&wifi_param);
		memcpy(wifi_param.ssid_buf, &video_params[1], ssid_length);
		wifi_param.ssid_buf[ssid_length] = '\0';
		memcpy(wifi_param.password_buf, &video_params[35], password_length);
    	wifi_param.password_buf[password_length] = '\0';
		AI_GLASS_MSG("SSID: %s\r\n", wifi_param.ssid_buf);
   		AI_GLASS_MSG("Password: %s\r\n", wifi_param.password_buf);
		if (media_update_wifi_params(&wifi_param) != MEDIA_OK) {
			resp_stat = AI_GLASS_PARAMS_ERR;
		}
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

static void ai_glass_snapshot(uartcmdpacket_t *param)
{
	uint8_t status = AI_GLASS_CMD_COMPLETE;
	uint8_t mode = 0;
	ai_glass_snapshot_param_t ai_snap_params = {0};
	isp_info_sync_t isp_info = {0};
	AI_GLASS_MSG("get UART_RX_OPC_CMD_SNAPSHOT = %lu\r\n", mm_read_mediatime_ms());
	if (xSemaphoreTake(video_proc_sema, 0) != pdTRUE) {
		if (current_state == STATE_RECORDING || current_state == STATE_END_RECORDING) {
			status = AI_GLASS_BUSY;
			AI_GLASS_MSG("Recording has started, not starting another recording\r\n");
		} else {
			if (BURST_MODE_MAX_COUNT == 2) {
				AI_GLASS_WARN("AI glass snapshot burst, snapshot + 1\r\n");
				total_burst++;
				uint8_t *snapshot_param = uart_parser_snapshot_video_info(param, &mode);
				uint8_t file_name_length = snapshot_param[0];
				char temp_record_filename_buffer[160] = {0};
				if (file_name_length > 0 && file_name_length <= 125 && dual_snapshot != 1) {
					char uart_filename_str[160] = {0};
					memset(uart_filename_str, 0, file_name_length + 1);
					memcpy(uart_filename_str, snapshot_param + 1, file_name_length);
					AI_GLASS_MSG("Filename retrieved from 8773 when snapshot + 1\r\n"); 
					extdisk_generate_unique_filename("", uart_filename_str, ".jpg", (char *)temp_record_filename_buffer, 160);
					if (burst_count < MAX_BURST) { 
						strncpy(burst_names[burst_count], temp_record_filename_buffer, 64-1); 
						burst_names[burst_count][64-1] = '\0'; 
						AI_GLASS_MSG("Stored burst filename %s for raw_index %d\r\n", burst_names[burst_count], burst_count); 
						burst_count++;
					}
				}
			} else {
				// status = AI_GLASS_BUSY;
				// uart_resp_snapshot(param, status);
			}
		}
	} else {
		uint8_t *snapshot_param = uart_parser_snapshot_video_info(param, &mode);
		AI_GLASS_MSG("%s get mode = %d\r\n", __func__, mode);
		if (mode == 1) {
			AI_GLASS_MSG("Process AI SNAPSHOT\r\n");
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
			AI_GLASS_MSG("Process LIFETIME SNAPSHOT\r\n");

			if (dual_snapshot != 1) {
				AI_GLASS_MSG("Received isp info from LF snapshot param\r\n");
				isp_info.isp_exposure_time = snapshot_param[49] | (snapshot_param[50] << 8) | (snapshot_param[51] << 16) | (snapshot_param[52] << 24);
				isp_info.isp_exposure_gain = snapshot_param[54] | (snapshot_param[55] << 8) | (snapshot_param[56] << 16) | (snapshot_param[57] << 24);
				isp_info.isp_red_gain = snapshot_param[58] | (snapshot_param[59] << 8) | (snapshot_param[60] << 16);
				isp_info.isp_blue_gain = snapshot_param[61] | (snapshot_param[62] << 8) | (snapshot_param[63] << 16);
				AI_GLASS_INFO("isp_exposure_time = %u\r\n", isp_info.isp_exposure_time);
				AI_GLASS_INFO("isp_exposure_gain = %u\r\n", isp_info.isp_exposure_gain);
				AI_GLASS_INFO("isp_red_gain = %u\r\n", isp_info.isp_red_gain);
				AI_GLASS_INFO("isp_blue_gain = %u\r\n", isp_info.isp_blue_gain);
			} else {
				// AI+Lifetime snapshot, store the ISP settings from AI snapshot params for use in lifetime snapshot
				AI_GLASS_MSG("Received isp info from AI snapshot param\r\n");
				isp_info.isp_exposure_time = ai_snap_params.isp_exposure_time;
				isp_info.isp_exposure_gain = ai_snap_params.isp_exposure_gain;
				isp_info.isp_red_gain = ai_snap_params.isp_red_gain;
				isp_info.isp_blue_gain = ai_snap_params.isp_blue_gain;
				AI_GLASS_INFO("isp_exposure_time = %u\r\n", isp_info.isp_exposure_time);
				AI_GLASS_INFO("isp_exposure_gain = %u\r\n", isp_info.isp_exposure_gain);
				AI_GLASS_INFO("isp_red_gain = %u\r\n", isp_info.isp_red_gain);
				AI_GLASS_INFO("isp_blue_gain = %u\r\n", isp_info.isp_blue_gain);
			}
			int ret = lifetime_snapshot_initialize(&isp_info);
			if (ret == 0) {
				uint8_t file_name_length = snapshot_param[0];
				char temp_record_filename_buffer[160] = {0};
				uint8_t lifetime_snap_name[160] = {0};
				if (file_name_length > 0 && file_name_length <= 125 && dual_snapshot != 1) {
					char uart_filename_str[160] = {0};
					memset(uart_filename_str, 0, file_name_length + 1);
					memcpy(uart_filename_str, snapshot_param + 1, file_name_length);
					AI_GLASS_MSG("Filename retrieved from 8773\r\n"); 
					extdisk_generate_unique_filename("", uart_filename_str, ".jpg", (char *)temp_record_filename_buffer, 160);
					snprintf((char *)lifetime_snap_name, sizeof(lifetime_snap_name), "%s", (const char *)temp_record_filename_buffer);
					if (burst_count < MAX_BURST) { 
						strncpy(burst_names[burst_count], temp_record_filename_buffer, 64-1); 
						burst_names[burst_count][64-1] = '\0'; 
						AI_GLASS_MSG("Stored burst filename %s for raw_index %d\r\n", burst_names[burst_count], burst_count); 
						burst_count++;
					}
				} else if((ai_snap_params.lifetime_file_name_len > 0) && (dual_snapshot == 1)) {
					file_name_length = ai_snap_params.lifetime_file_name_len;
					char uart_filename_str[160] = {0};
					memset(uart_filename_str, 0, file_name_length + 1);
					memcpy(uart_filename_str, ai_snap_params.lifetime_file_name, file_name_length);
					AI_GLASS_MSG("Filename retrieved from 8773 (AI snapshot params)\r\n");
					if (ai_snap_params.lifetime_file_name_len >= sizeof(ai_snap_params.lifetime_file_name)) {
						ai_snap_params.lifetime_file_name_len = sizeof(ai_snap_params.lifetime_file_name) - 1;
					}
					ai_snap_params.lifetime_file_name[ai_snap_params.lifetime_file_name_len] = '\0';
					
					extdisk_generate_unique_filename("", (const char *)uart_filename_str, ".jpg", (char *)temp_record_filename_buffer, 160);
					snprintf((char *)lifetime_snap_name, sizeof(lifetime_snap_name), "%s", (const char *)temp_record_filename_buffer);
					AI_GLASS_MSG("lifetime_snap_name = %s\r\n", (char *)lifetime_snap_name);
					
				} else {
					char *cur_time_str = (char *)media_filesystem_get_current_time_string();
					if (cur_time_str) {
						AI_GLASS_MSG("Filename generated from 8735B\r\n");
						extdisk_generate_unique_filename("PICTURE_0_0_", cur_time_str, ".jpg", (char *)temp_record_filename_buffer, 160);
						snprintf((char *)lifetime_snap_name, sizeof(lifetime_snap_name), "%s", (const char *)temp_record_filename_buffer);
						AI_GLASS_MSG("lifetime_snap_name = %s\r\n", (char *)lifetime_snap_name);
						free(cur_time_str);
					} else {
						AI_GLASS_WARN("no memory for lifetime snapshot file name\r\n");
						extdisk_generate_unique_filename("PICTURE_0_0_", "19800101", ".jpg", (char *)temp_record_filename_buffer, 160);
					}
				}
lifetimesnapshottake:
				if (lifetime_snapshot_take((const char *)lifetime_snap_name, param) == 0) {
					status = AI_GLASS_DEVICE_WORKING_IN_PROG;
					if ((current_sensor_id == SENSOR_IMX681) || (current_sensor_id == SENSOR_IMX471) || (current_sensor_id == SENSOR_OV13B10)) {
						//do nothing
					}
					else {
						// for non HR sensor, return 0x22 status
						uart_resp_snapshot(param, status);
					}
					ai_glass_init_external_disk();
					if (lifetime_highres_save((const char *)lifetime_snap_name, param) != 0) {
						AI_GLASS_WARN("lifetime snapshot high res save failed\r\n");
						status = AI_GLASS_PROC_FAIL;
						uart_resp_snapshot(param, status);
					}
					while (1) {
						uartcmdinfo_t *new_cmd = NULL;
						int ret = uart_wait_for_next_cmd_or_idle(1000, &new_cmd);

						if (ret == 1 && BURST_MODE_MAX_COUNT == 2) {
							// Snapshot detected
							AI_GLASS_MSG("Snapshot detected, continuing snapshot recurring\r\n");

							// Use filename already prepared earlier
							const char *filename = burst_names[burst_count - 1];

							// Print out the filename for verification
							AI_GLASS_MSG("Taking snapshot with filename: %s\r\n", filename);

							isp_info_sync_t isp_info = {0};
							lifetime_hr_snapshot_initialize(&isp_info);

							goto lifetimesnapshottake;
						} else { // ret == 0
							break;
						}
					}
				} else {
					status = AI_GLASS_PROC_FAIL;
					uart_resp_snapshot(param, status);
				}
				while (1) {
					uartcmdinfo_t *new_cmd = NULL;
					int ret = uart_wait_for_next_cmd_or_idle(1000, &new_cmd);

					if (ret == 1 && BURST_MODE_MAX_COUNT == 2) {
						// Snapshot detected
						AI_GLASS_MSG("Snapshot detected, continuing snapshot recurring\r\n");

						// Use filename already prepared earlier
						const char *filename = burst_names[burst_count - 1];

						// Print out the filename for verification
						AI_GLASS_MSG("Taking snapshot with filename: %s\r\n", filename);

						isp_info_sync_t isp_info = {0};
						lifetime_hr_snapshot_initialize(&isp_info);

						goto lifetimesnapshottake;
					} else { // ret == 0
						break;
					}
				}
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
		if (mode == 0) {
			while (1) {
				uartcmdinfo_t *new_cmd = NULL;
				int ret = uart_wait_for_next_cmd_or_idle(1000, &new_cmd);

				if (ret == 1 && BURST_MODE_MAX_COUNT == 2) {
					// Snapshot detected
					AI_GLASS_MSG("Snapshot detected, continuing snapshot recurring\r\n");

					// Use filename already prepared earlier
					const char *filename = burst_names[burst_count - 1];

					// Print out the filename for verification
					AI_GLASS_MSG("Taking snapshot with filename: %s\r\n", filename);

					isp_info_sync_t isp_info = {0};
					lifetime_hr_snapshot_initialize(&isp_info);

					lifetime_snapshot_take(filename, param);
					goto lifetimesnapshottake;
				} else { // ret == 0
					// Idle → finalize
					AI_GLASS_MSG("wait for lifetime snapshot deinit\r\n");
					extdisk_save_file_cntlist();
					while (lifetime_snapshot_deinitialize()) {
						vTaskDelay(1);
					}
					critical_process_started = 0;
					uartcmdpacket_t dummy_param;
					ai_glass_get_file_cnt(&dummy_param);
					status = AI_GLASS_CMD_COMPLETE;
					uart_resp_snapshot(param, status);
					break;
				}
			}
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
	if (dual_snapshot == 1 && ((uart_service_get_bt_role() == AI_GLASS_BT_ROLE_LEFT) || (uart_service_get_bt_role() == AI_GLASS_BT_ROLE_RIGHT))) {
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
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
				uartcmdpacket_t dummy_param;
				ai_glass_get_file_cnt(&dummy_param);
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

static void mp4_send_audio_response_callback(struct tmrTimerControl *parm)
{
	uint8_t record_resp_status = AI_GLASS_CMD_COMPLETE;

	if (xSemaphoreTake(send_audio_response_timermutex, portMAX_DELAY) == pdTRUE) {
		if (send_response_timer_setstop == 0) {
			if (current_state == STATE_END_RECORDING || current_state == STATE_ERROR) {
				if (current_state == STATE_ERROR) {
					record_resp_status = AI_GLASS_PROC_FAIL;
				} else {
					record_resp_status = AI_GLASS_CMD_COMPLETE;
				}
				lifetime_audio_deinitialize();
				send_response_timer_setstop = 1;
				xSemaphoreGive(send_audio_response_timermutex);
				uart_resp_audio_record_stop(record_resp_status);
				AI_GLASS_MSG("mp4_send_audio_response_callback UART_TX_OPC_RESP_RECORD_STOP %lu\r\n", mm_read_mediatime_ms());
				uartcmdpacket_t dummy_param;
				ai_glass_get_file_cnt(&dummy_param);
				xSemaphoreGive(video_proc_sema);
			} else {
				if (current_state == STATE_RECORDING || current_state == STATE_IDLE) {
					record_resp_status = AI_GLASS_CMD_COMPLETE;
				}
				uart_resp_audio_record_cont(record_resp_status);
				AI_GLASS_MSG("mp4_send_audio_response_callback %lu\r\n", mm_read_mediatime_ms());
				if (send_audio_response_timer != NULL) {
					if (xTimerStart(send_audio_response_timer, 0) != pdPASS) {
						AI_GLASS_ERR("Send timer failed\r\n");
					}
				}
				xSemaphoreGive(send_audio_response_timermutex);
			}
		} else {
			xSemaphoreGive(send_audio_response_timermutex);
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

static void ai_glass_audio_start(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_TX_OPC_CMD_AUDIO_START = %lu\r\n", mm_read_mediatime_ms());

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
	printf("Audio record filename: %s\r\n", filename_str);
	//Initialize function has a timer that constantly reads the status of MP4.
	
	AI_GLASS_MSG("Record start = %lu\r\n", mm_read_mediatime_ms());
	if (current_state == STATE_RECORDING || current_state == STATE_END_RECORDING) {
		AI_GLASS_MSG("Video recording has started, not starting another audio recording\r\n");
		record_start_status = AI_GLASS_CMD_COMPLETE;
		uart_resp_audio_record_start(record_start_status);
	} else if (current_state == STATE_IDLE) {
		int ret = lifetime_audio_initialize(record_filename_length, (const char *)filename_str);
		// Save filelist to EMMC
		if (send_audio_response_timer != NULL && ret == 0) {
			extdisk_save_file_cntlist();
			if (xSemaphoreTake(send_audio_response_timermutex, portMAX_DELAY) == pdTRUE) {
				if (xTimerStart(send_audio_response_timer, 0) != pdPASS) {
					record_start_status = AI_GLASS_PROC_FAIL;
					uart_resp_audio_record_start(record_start_status);
					AI_GLASS_ERR("Send UART_RX_OPC_CMD_RECORD_START timer failed\r\n");
					lifetime_audio_deinitialize();
				} else {
					record_start_status = AI_GLASS_CMD_COMPLETE;
					send_response_timer_setstop = 0;
					uart_resp_audio_record_start(record_start_status);
				}
				xSemaphoreGive(send_audio_response_timermutex);
			} else {
				record_start_status = AI_GLASS_PROC_FAIL;
				uart_resp_audio_record_start(record_start_status);
				AI_GLASS_ERR("Send UART_RX_OPC_CMD_RECORD_START timer mutex failed\r\n");
				lifetime_audio_deinitialize();			
			}
		} else {
			record_start_status = AI_GLASS_PROC_FAIL;
			uart_resp_audio_record_start(record_start_status);
			AI_GLASS_ERR("Failed to create send_response_timer\r\n");
			}
	} else {
		record_start_status = AI_GLASS_PROC_FAIL;
		uart_resp_audio_record_start(record_start_status);
		AI_GLASS_ERR("Failed because of the known record status\r\n");
	}

	AI_GLASS_INFO("end of UART_TX_OPC_CMD_AUDIO_START\r\n");
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
	uartcmdpacket_t dummy_param;
	ai_glass_get_file_cnt(&dummy_param);
	uart_resp_record_stop(record_stop_status);
	AI_GLASS_MSG("end of UART_RX_OPC_CMD_RECORD_STOP %lu\r\n", mm_read_mediatime_ms());
}

static void ai_glass_audio_stop(uartcmdpacket_t *param)
{
	AI_GLASS_MSG("get UART_RX_OPC_CMD_AUDIO_STOP %lu\r\n", mm_read_mediatime_ms());
	uint8_t record_stop_status = AI_GLASS_CMD_COMPLETE;
	if (current_state == STATE_RECORDING) {
		if (xSemaphoreTake(send_audio_response_timermutex, portMAX_DELAY) == pdTRUE) {
			if (send_response_timer_setstop == 0) {
				if (send_audio_response_timer != NULL) {
					if (xTimerIsTimerActive(send_audio_response_timer) == pdTRUE) {
						xTimerStop(send_audio_response_timer, 0);
					}
				}
				lifetime_audio_deinitialize();
				xSemaphoreGive(video_proc_sema);
				send_response_timer_setstop = 1;
				xSemaphoreGive(send_audio_response_timermutex);
			} else {
				AI_GLASS_MSG("The recording timer has stop\r\n");
				xSemaphoreGive(send_audio_response_timermutex);
			}
		}
	}
	uartcmdpacket_t dummy_param;
	ai_glass_get_file_cnt(&dummy_param);
	uart_resp_audio_record_stop(record_stop_status);
	AI_GLASS_MSG("end of UART_RX_OPC_CMD_AUDIO_STOP %lu\r\n", mm_read_mediatime_ms());
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
	}
	else {
		uint8_t status = AI_GLASS_PROC_FAIL;
		uart_resp_delete_all_file(status);
	}
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_DELETE_ALL_FILES\r\n");
}

static void ai_glass_get_sd_info(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_SD_INFO %lu\r\n", mm_read_mediatime_ms());
	critical_process_started = 1;
	ai_glass_init_external_disk();
	uint64_t device_used_bytes = fatfs_get_used_space_byte();
	uint64_t device_total_bytes = device_used_bytes + fatfs_get_free_space_byte();
	uint32_t device_used_Kbytes = (uint32_t)(device_used_bytes / 1024);
	uint32_t device_total_Kbytes = (uint32_t)(device_total_bytes / 1024);

	uart_resp_get_sd_info(param, device_total_Kbytes, device_used_Kbytes);
	critical_process_started = 0;
	AI_GLASS_MSG("Get device memory: %lu/%luKB\r\n", device_used_Kbytes, device_total_Kbytes);
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_SD_INFO %lu\r\n", mm_read_mediatime_ms());
}

static void ai_glass_set_ap_mode(uartcmdpacket_t *param)
{
	AI_GLASS_MSG("get UART_RX_OPC_CMD_SET_WIFI_MODE %lu\r\n", mm_read_mediatime_ms());
	critical_process_started = 1;

	uartpacket_t *query_pkt = (uartpacket_t *) & (param->uart_pkt);
	uint8_t mode = query_pkt->data_buf[0];
	uint8_t result = AI_GLASS_CMD_COMPLETE;
	rtw_softap_info_t wifi_cfg = {0};
	uint8_t password[MAX_AP_PASSWORD_LEN] = {0};
	wifi_cfg.password = password;

	ai_glass_wifi_param_t wifi_param = {0};
    media_get_wifi_params_from_flash(&wifi_param);

	if (mode == 1 || mode == 3) {
		ai_glass_init_external_disk();
		if (wifi_param.ssid_buf[0] != '\0' && wifi_param.password_buf[0] != '\0') {
			if (wifi_enable_ap_mode(wifi_param.ssid_buf, wifi_param.password_buf, AI_GLASS_AP_CHANNEL, 20) == WLAN_SET_OK) {
				wifi_get_ap_setting(&wifi_cfg);
				result = AI_GLASS_CMD_COMPLETE;
			} else {
				result = AI_GLASS_PROC_FAIL;
			}
		}
		else {
			if (wifi_enable_ap_mode(AI_GLASS_AP_SSID, AI_GLASS_AP_PASSWORD, AI_GLASS_AP_CHANNEL, 20) == WLAN_SET_OK) {
				wifi_get_ap_setting(&wifi_cfg);
				result = AI_GLASS_CMD_COMPLETE;
			} else {
				result = AI_GLASS_PROC_FAIL;
			}
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
	critical_process_started = 0;

	if (mode == 1 && result == AI_GLASS_CMD_COMPLETE) {
		deinitial_media(); // To save power
	}
	AI_GLASS_MSG("end of UART_RX_OPC_CMD_SET_WIFI_MODE %lu\r\n", mm_read_mediatime_ms());
}

// For UART_RX_OPC_CMD_SET_STA_MODE
static void ai_glass_set_sta_mode(uartcmdpacket_t *param)
{
	AI_GLASS_MSG("get UART_RX_OPC_CMD_SET_STA_MODE %lu\r\n", mm_read_mediatime_ms());
	critical_process_started = 1;

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

	if (new_mode == 1 || new_mode == 2 || new_mode == 4) {
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
			uint8_t ip0,ip1,ip2,ip3;
			while (1) {
				ip = *(u32 *)LwIP_GetIP(0);

				ip0 = (ip) & 0xFF;
				ip1 = (ip >> 8) & 0xFF;
				ip2 = (ip >> 16) & 0xFF;
				ip3 = (ip >> 24) & 0xFF;

				if (ip0 != 0 && ip1 != 0 && ip2 !=0 && ip3 !=0) {
					break;
				}
				AI_GLASS_INFO("Waiting for IP...\r\n");
				vTaskDelay(100);
			}

			u8 connected_channel = 0;
			uint8_t bandwidth_check = 0;   // 1 for 2.4Ghz, 2 for 5Ghz.
			wifi_get_channel(&channel);
			if(channel > 0 && channel < 15){
				AI_GLASS_INFO("Connected to 2.4GHz Network\r\n");
				bandwidth_check = 1;
			}
			else if(channel >= 36  && channel <= 165){
				AI_GLASS_INFO("Connected to 5GHz Network\r\n");
				bandwidth_check = 2;
			}

			AI_GLASS_INFO("ip_idx0: %d\r\n", ip0);
			AI_GLASS_INFO("ip_idx1: %d\r\n", ip1);
			AI_GLASS_INFO("ip_idx2: %d\r\n", ip2);
			AI_GLASS_INFO("ip_idx3: %d\r\n", ip3);
			AI_GLASS_INFO("bandwidth_check: %d\r\n", bandwidth_check);

			taskprint();
			
			uart_resp_set_sta_mode(param, result,
				ip0,
				ip1,
				ip2,
				ip3,
				bandwidth_check);
		} else {
			AI_GLASS_INFO("Already in STA mode. Skipping re-init.\r\n");
		}
	}
	else if (new_mode == 0) {
		if (wifi_disable_sta_mode() == WLAN_SET_OK) {
			AI_GLASS_INFO("STA mode disabled successfully.\r\n");
			g_current_wifi_mode = new_mode;
			AI_GLASS_MSG("Current mode is %u\r\n", g_current_wifi_mode);
			result = AI_GLASS_CMD_COMPLETE;
		} else {
			AI_GLASS_INFO("Fail to disable STA mode.\r\n");
			result = AI_GLASS_PROC_FAIL;
		}
		uart_resp_set_sta_mode(param, result, 0, 0, 0, 0, 0);
	}
	else {
		result = AI_GLASS_PARAMS_ERR;
		AI_GLASS_INFO("Invalid mode value: %d\r\n", new_mode);
		uart_resp_set_sta_mode(param, result, 0, 0, 0, 0, 0);
	}
	critical_process_started = 0;
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

static void ai_glass_rtsp_live_start(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_RTSP_LIVE_START\r\n");

	uint8_t result = AI_GLASS_CMD_COMPLETE;
    //STEP 1: Deinitialize media
	deinitial_media();

	//STEP 2: Set stream parameters
	ai_glass_stream_param_t temp_stream_param = {0};
	uint8_t *video_params = uart_parser_stream_info(param);
	parser_rtsp_stream_param(&temp_stream_param, video_params);
	AI_GLASS_INFO("Get Stream Data\r\n");
	print_stream_data(&temp_stream_param);
	if (media_update_stream_params(&temp_stream_param) != MEDIA_OK) {
		result = AI_GLASS_PARAMS_ERR;
		AI_GLASS_ERR("UPDATE STREAM PARAMS ERROR\r\n");
	}

    //STEP 3: Start stream service
	wifi_streaming_initialize();
	result = AI_GLASS_CMD_COMPLETE;
	//STEP 4: Respond status
	uart_resp_rtsp_live_start(param, result);
	AI_GLASS_INFO("get UART_RX_OPC_CMD_RTSP_LIVE_START END\r\n");
}

void ai_glass_rtsp_live_stop(uartcmdpacket_t *param)
{
    AI_GLASS_INFO("get UART_RX_OPC_CMD_RTSP_LIVE_STOP\r\n");
   	wifi_streaming_deinitialize();
	uint8_t resp_stat = AI_GLASS_CMD_COMPLETE;
	uart_resp_rtsp_live_stop(param, resp_stat);
    AI_GLASS_INFO("UART_TX_OPC_RESP_STOP_RTSP_STREAMING END\r\n");
}

static void ai_glass_live_start(uartcmdpacket_t *param)
{
	AI_GLASS_INFO("get UART_RX_OPC_CMD_LIVE_START\r\n");
	uint8_t resp_stat = AI_GLASS_CMD_COMPLETE;
	if (xSemaphoreTake(video_proc_sema, 0) == pdTRUE) {
		// ai_glass_init_external_disk();
		ai_glass_stream_param_t temp_stream_param = {0};
		uint8_t *video_params = uart_parser_stream_info(param);
		parser_stream_param(&temp_stream_param, video_params);
		AI_GLASS_INFO("Get Stream Data\r\n");
		print_stream_data(&temp_stream_param);
		if (media_update_stream_params(&temp_stream_param) != MEDIA_OK) {
			resp_stat = AI_GLASS_PARAMS_ERR;
			xSemaphoreGive(video_proc_sema);
		}
		if (current_state == STATE_RECORDING || current_state == STATE_END_RECORDING || current_state == STATE_STREAMING) {
			AI_GLASS_MSG("Video/Audio recording has started, not starting another streaming\r\n");
			resp_stat = AI_GLASS_CMD_COMPLETE;
			uart_resp_live_start(param, resp_stat);
			xSemaphoreGive(video_proc_sema);
		} else if (current_state == STATE_IDLE) {
			uart_resp_live_start(param, resp_stat);
			wifi_off();
			if (lifetime_streaming_initialize() < 0) {
				xSemaphoreGive(video_proc_sema);
			}
		}
	} else {
		AI_GLASS_WARN("AI glass is snapshot, record or streaming, currently snapshot or record busy fail\r\n");
		resp_stat = AI_GLASS_BUSY;
		uart_resp_live_start(param, resp_stat);
		xSemaphoreGive(video_proc_sema);
	}
	AI_GLASS_INFO("get UART_RX_OPC_CMD_LIVE_START END\r\n");
}

void ai_glass_live_stop(uartcmdpacket_t *param)
{
    AI_GLASS_INFO("get UART_RX_OPC_CMD_LIVE_STOP\r\n");
	if (current_state == STATE_STREAMING) {
		lifetime_streaming_deinitialize();
		uint8_t resp_stat = AI_GLASS_CMD_COMPLETE;
		uart_resp_live_stop(param, resp_stat);
		xSemaphoreGive(video_proc_sema);
	}
    AI_GLASS_INFO("UART_TX_OPC_RESP_STOP_STREAMING END\r\n");
}

// UART_RX_OPC_CMD_GET_WIFI_PARAMETER
static void ai_glass_get_wifi_parameter(uartcmdpacket_t *param) {
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_WIFI_PARAMETER\r\n");
    uint8_t g_camera_cfg_buf[512];
    size_t length = uart_serialize_camera_config(g_camera_cfg_buf, sizeof(g_camera_cfg_buf), &g_camera_cfg);

    // Call your existing UART response function
    int status = uart_resp_get_wifi_parameter(param, g_camera_cfg_buf, length);

    // Debug print
    print_camera_config(&g_camera_cfg);
	printf("CameraConfig sent successfully (%lu bytes)\n", length);
    if (status == 0) {
        printf("CameraConfig sent successfully (%lu bytes)\n", length);
    } else {
        printf("UART send failed: %d\n", status);
    }
	AI_GLASS_INFO("UART_RX_OPC_CMD_GET_WIFI_PARAMETER END\r\n");
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

	{UART_RX_OPC_CMD_AUDIO_START,      {false, false, ai_glass_audio_start},          {NULL, NULL}},
	{UART_RX_OPC_CMD_AUDIO_STOP,       {true,  false, ai_glass_audio_stop},           {NULL, NULL}},

	{UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW,       {false, false, ai_glass_get_pic_data_sliding_window},       {NULL, NULL}},
	{UART_RX_OPC_CMD_GET_PICTURE_DATA_SLIDING_WINDOW_ACK,   {false, true, ai_glass_get_pic_data_sliding_window_ack},    {NULL, NULL}},

	{UART_RX_OPC_CMD_SET_SYS_UPGRADE,                        {false, false, ai_glass_get_set_sys_upgrade},              {NULL, NULL}},
	{UART_RX_OPC_CMD_CANCEL_SYS_UPGRADE,                   	 {false, false, ai_glass_get_cancel_sys_upgrade},                 {NULL, NULL}},
	{UART_TX_OPC_CMD_START_BT_SOC_FW_UPGRADE,                {false, false, ai_glass_resp_bt_fw_upgrade},               {NULL, NULL}},
	{UART_TX_OPC_CMD_FINISH_BT_SOC_FW_UPGRADE,               {false, false, ai_glass_resp_bt_fw_finish},                {NULL, NULL}},
	{UART_RX_OPC_CMD_SET_WIFI_FW_ROLLBACK,                   {false, false, ai_glass_wifi_fw_rollback},                 {NULL, NULL}},

	{UART_RX_OPC_CMD_RTSP_LIVE_START,                   	 {false, false, ai_glass_rtsp_live_start},                  {NULL, NULL}},
	{UART_RX_OPC_CMD_RTSP_LIVE_STOP,                         {true, false, ai_glass_rtsp_live_stop},                    {NULL, NULL}},
	     
	{UART_RX_OPC_CMD_LIVE_START,                   			 {false, false, ai_glass_live_start},                       {NULL, NULL}},
	{UART_RX_OPC_CMD_LIVE_STOP,                              {true, false, ai_glass_live_stop},                         {NULL, NULL}},

	{UART_RX_OPC_CMD_GET_WIFI_PARAMETER,                     {false, false, ai_glass_get_wifi_parameter},               {NULL, NULL}},
};

void uart_fun_regist(void)
{
	uart_service_add_table(rx_opcode_basic_items, sizeof(rx_opcode_basic_items) / sizeof(rx_opcode_basic_items[0]));
}

#if EXTDISK_LOG

void ai_glass_extdisk_log_start(void)
{
    if (g_emmc_log_enabled) {
        AI_GLASS_INFO("[AIGLASS] eMMC log already active, skipping start.\n");
        return;
    }

    ai_glass_init_external_disk();

    g_emmc_log_fp = extdisk_fopen("uart_log.txt", "a+");
    if (!g_emmc_log_fp) {
        AI_GLASS_ERR("[AIGLASS] Failed to open log file on eMMC!\n");
        return;
    }

    g_emmc_log_enabled = true;

    // Bind stdio to our non-blocking putc
    stdio_port_init_s((void *)&log_uart, (stdio_putc_t)log_emmc_putc, (stdio_getc_t)&hal_uart_rgetc);
    stdio_port_init_ns((void *)&log_uart, (stdio_putc_t)log_emmc_putc, (stdio_getc_t)&hal_uart_rgetc);

    // Create logger infra once
    if (!log_flush_mutex) {
        log_flush_mutex = xSemaphoreCreateMutex();
    }
    if (!log_task_handle) {
        xTaskCreate(log_task, "log_task", 2048, NULL, tskIDLE_PRIORITY + 1, &log_task_handle);
    }

    AI_GLASS_INFO("[AIGLASS] eMMC logging active: uart_log.txt\n");
}

void ai_glass_extdisk_log_stop(void)
{
    // DO NOT print in this path; formatted printing consumes stack
    if (!g_emmc_log_enabled) {
        // Optional: avoid print here too
        // AI_GLASS_INFO("[AIGLASS] eMMC log not active, skipping stop.\n");
        return;
    }

    // Freeze the producer
    if (log_task_handle) vTaskSuspend(log_task_handle);

    g_emmc_log_enabled = false;

    // Drain remaining ring
    if (g_emmc_log_fp && log_flush_mutex &&
        xSemaphoreTake(log_flush_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        size_t n;
        while ((n = log_ring_read_bulk((char *)s_log_flush_buf, sizeof(s_log_flush_buf))) > 0) {
            extdisk_fwrite(s_log_flush_buf, 1, n, g_emmc_log_fp);
        }
        extdisk_fflush(g_emmc_log_fp);
        xSemaphoreGive(log_flush_mutex);
    }

    if (g_emmc_log_fp) {
        extdisk_fclose(g_emmc_log_fp);
        g_emmc_log_fp = NULL;
    }

    // Rebind to UART-only (lightweight)
    stdio_port_init_s((void *)&log_uart, (stdio_putc_t)wputc, (stdio_getc_t)&hal_uart_rgetc);
    stdio_port_init_ns((void *)&log_uart, (stdio_putc_t)wputc, (stdio_getc_t)&hal_uart_rgetc);

    // Avoid prints here to keep stack low
    if (log_task_handle) vTaskResume(log_task_handle);
}


void ai_glass_log_flush(void)
{
    if (g_emmc_log_enabled && g_emmc_log_fp && log_flush_mutex &&
        xSemaphoreTake(log_flush_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        size_t n, total = 0;
        while ((n = log_ring_read_bulk((char *)s_log_flush_buf, sizeof(s_log_flush_buf))) > 0) {
            int w = extdisk_fwrite(s_log_flush_buf, 1, n, g_emmc_log_fp);
            if (w > 0) total += w;
        }
        extdisk_fflush(g_emmc_log_fp);
        xSemaphoreGive(log_flush_mutex);
        printf("[AIGLASS] Force flushed %zu bytes to disk\n", total);
    }
}

#endif

void ai_glass_service_thread(void *param)
{
#if EXTDISK_LOG
	media_filesystem_init();
	ai_glass_log_init();
	ai_glass_get_fw_version();
	printf("[FOR OTA BOOTLOADER CHECK DIFFERENCE] count = %x\r\n",(*((volatile uint32_t *) 0xe0001004)));
#endif 
	AI_GLASS_MSG("ai_glass_service_thread start %lu\r\n", mm_read_mediatime_ms());

	initial_media_parameters();
	AI_GLASS_MSG("media system done %lu\r\n", mm_read_mediatime_ms());
#if !EXTDISK_LOG 
	// cost about 60ms into this function
	media_filesystem_init();
#endif 
	media_filesystem_setup_gpstime(0, 0); // Set up GPS start time to prevent failed for file system
	ai_glass_init_ram_disk();
	//ai_glass_init_external_disk(); // init EMMC here will cause 160 ms delay
	//extdisk_save_file_cntlist();
	AI_GLASS_MSG("vfs system done %lu\r\n", mm_read_mediatime_ms());
#if !EXTDISK_LOG 
	ai_glass_log_init();
#endif 
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
	send_audio_response_timer = xTimerCreate("send_audio_response_timer", 100 / portTICK_PERIOD_MS, pdFALSE, NULL, mp4_send_audio_response_callback);
	if (send_audio_response_timer == NULL) {
		AI_GLASS_ERR("send_audio_response_timer create fail\r\n");
		goto exit;
	}
	send_audio_response_timermutex = xSemaphoreCreateMutex();
	if (send_audio_response_timermutex == NULL) {
		AI_GLASS_ERR("send_audio_response_timermutex create fail\r\n");
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

	// ai_glass_init_external_disk();
	// AI_GLASS_MSG("Format disk to FAT32\r\n");
	// int ret = vfs_user_format(ai_glass_disk_name, VFS_FATFS, EXTDISK_PLATFORM);
	// if (ret == FR_OK) {
	// 	AI_GLASS_MSG("format successfully\r\n");
	// } else {
	// 	AI_GLASS_ERR("format failed %d\r\n", ret);
	// }
exit:
	vTaskDelete(NULL);
}

void ai_glass_init(void)
{
#if !EXTDISK_LOG 
	ai_glass_get_fw_version();
#endif
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

#if EXTDISK_LOG
	
	ai_glass_extdisk_log_stop();

#endif

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
	int argc = 0;
	char *argv[MAX_ARGC] = {0};
	argc = parse_param(arg, argv);
	if (argc > 1) {
		printf("argc = %d\r\n", argc);
		// burst mode
		total_burst = atoi(argv[1]);
	} else {
		total_burst = 1;
	}

	uint8_t status = AI_GLASS_CMD_COMPLETE;
	isp_info_sync_t isp_info = {0};
	if (xSemaphoreTake(video_proc_sema, 0) != pdTRUE) {
		AI_GLASS_WARN("AI glass is snapshot or record, current snapshot busy fail\r\n");
		goto endofsnapshot;
	}
	AI_GLASS_MSG("snapshot aiglass_mass_storage_deinit time = %lu\r\n", mm_read_mediatime_ms());
	ai_glass_init_external_disk();
	AI_GLASS_MSG("Process LIFETIME SNAPSHOT\r\n");

	int ret = lifetime_snapshot_initialize(&isp_info);
	if (ret == 0) {
		char temp_record_filename_buffer[160] = {0};
		uint8_t lifetime_snap_name[160] = {0};

		char *cur_time_str = (char *)media_filesystem_get_current_time_string();
		if (cur_time_str) {
			extdisk_generate_unique_filename("PICTURE_0_0_", cur_time_str, ".jpg", (char *)temp_record_filename_buffer, 160);
			snprintf((char *)lifetime_snap_name, sizeof(lifetime_snap_name), "%s", (const char *)temp_record_filename_buffer);
			free(cur_time_str);
		} else {
			AI_GLASS_WARN("no memory for lifetime snapshot file name\r\n");
			extdisk_generate_unique_filename("PICTURE_0_0_", "19800101", ".jpg", (char *)temp_record_filename_buffer, 160);
		}
		uartcmdpacket_t *param = NULL;
		if (lifetime_snapshot_take((const char *)lifetime_snap_name, param) == 0) {
			status = AI_GLASS_DEVICE_WORKING_IN_PROG;
			if (lifetime_highres_save((const char *)lifetime_snap_name, param) != 0) {
				AI_GLASS_WARN("lifetime snapshot high res save failed\r\n");
				status = AI_GLASS_PROC_FAIL;
			}
		} else {
			status = AI_GLASS_PROC_FAIL;
		}
		// Save filelist to EMMC
		extdisk_save_file_cntlist();
		AI_GLASS_MSG("Extdisk save file countlist done = %lu\r\n", mm_read_mediatime_ms());
		status = AI_GLASS_CMD_COMPLETE;
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
	xSemaphoreGive(video_proc_sema);
endofsnapshot:
	AI_GLASS_INFO("end of UART_RX_OPC_CMD_SNAPSHOT = %lu\r\n", mm_read_mediatime_ms());
}

void fCLEARMEDIAFLASH(void *arg)
{
    int argc = 0;
    char *argv[MAX_ARGC] = {0};
   
    argc = parse_param(arg, argv);  // Typical AT command parsing
 
    if (argc < 2) {
        AI_GLASS_ERR("[ATCMD CLEAR MEDIA FLASH] Usage: AT+AIGLASSCLEARMEDIAFLASH=XX\r\n");
        return;
    }
 
    unsigned int clear_macro = 0;
    if (sscanf(argv[1], "%x", &clear_macro) != 1) {
        AI_GLASS_ERR("[ATCMD CLEAR MEDIA FLASH] Invalid CLEAR ID format: %s\r\n", argv[1]);
        return;
    }
 
    if (media_clear_flash(clear_macro) < 0) {
        AI_GLASS_ERR("[ATCMD CLEAR MEDIA FLASH] Clear macro 0x%02X not found\r\n", clear_macro);
        return;
    }
 
    AI_GLASS_MSG("[ATCMD CLEAR MEDIA FLASH] Clear macro 0x%02X\r\n", clear_macro);
}

void fCHANGESENSOR(void *arg)
{
    int argc = 0;
    char *argv[MAX_ARGC] = {0};
	AI_GLASS_MSG("be4 switch Current Sensor ID: %u\n", current_sensor_id);
    argc = parse_param(arg, argv);  // Typical AT command parsing

    if (argc < 2) {
        AI_GLASS_ERR("[ATCMD SENSOR] Usage: AT+AIGLASSCHANGESENSOR=XX\r\n");
        return;
    }
	int sensor_index = 0;
    unsigned int sensor_macro = 0;
    if (sscanf(argv[1], "%x", &sensor_macro) != 1) {
        AI_GLASS_ERR("[ATCMD SENSOR] Invalid sensor ID format: %s\r\n", argv[1]);
        return;
    }
	if ((sensor_macro == SENSOR_SC5356_2M) || (sensor_macro == SENSOR_SC5356)) {
		// Find the index for this macro
		sensor_index = get_sensor_index_by_id((unsigned char)sensor_macro);
		AI_GLASS_MSG("[ATCMD SENSOR] Sensor macro 0x%02X found at index %d\r\n", sensor_macro, sensor_index);

		// Call your reinit function with sensor index
		reinit_sensor(sensor_index);

		AI_GLASS_MSG("Current Sensor ID: %u\n", current_sensor_id);
		if (sensor_index < 0) {
			AI_GLASS_ERR("[ATCMD SENSOR] Sensor macro 0x%02X not found\r\n", sensor_macro);
			return;
    	}
	} else {
		sensor_index = -1;
	}
    
}

void fLFRECORD(void *arg)
{
    AI_GLASS_INFO("RECORD_START = %lu\r\n", mm_read_mediatime_ms());
    int argc = 0;
    char *argv[MAX_ARGC] = {0};
   
    argc = parse_param(arg, argv);  // Typical AT command parsing
   
    uint16_t record_length = 0;
 
    if (argc) {
       
        record_length =  atoi(argv[1]);
    }
	
    media_update_record_time(record_length);
 
    uint8_t record_filename_length = 0;
    uint8_t *record_filename = 0;
 
    char filename_buf[160] = {0};
    const char *filename_str = NULL;
 
    if (record_filename && record_filename_length < sizeof(filename_buf)) {
        memcpy(filename_buf, record_filename, record_filename_length);
        filename_buf[record_filename_length] = '\0';  // Ensure null-terminated
 
        filename_str = filename_buf;
    } else {
        record_filename_length = 0;
    }
 
    ai_glass_init_external_disk();
 
    //Initialize function has a timer that constantly reads the status of MP4.
    if (xSemaphoreTake(video_proc_sema, 0) == pdTRUE) {
        AI_GLASS_MSG("Record start = %lu\r\n", mm_read_mediatime_ms());
        if (current_state == STATE_RECORDING || current_state == STATE_END_RECORDING) {
            AI_GLASS_MSG("Recording has started, not starting another recording\r\n");
            xSemaphoreGive(video_proc_sema);
        } else if (current_state == STATE_IDLE) {
            int ret = lifetime_recording_initialize(record_filename_length, filename_str);
            // Save filelist to EMMC
            if (send_response_timer != NULL && ret == 0) {
                extdisk_save_file_cntlist();
                if (xSemaphoreTake(send_response_timermutex, portMAX_DELAY) == pdTRUE) {
                    if (xTimerStart(send_response_timer, 0) != pdPASS) {
                        AI_GLASS_ERR("Send UART_RX_OPC_CMD_RECORD_START timer failed\r\n");
                        lifetime_recording_deinitialize();
                        xSemaphoreGive(video_proc_sema);
                    } else {
                        send_response_timer_setstop = 0;
                    }
                    xSemaphoreGive(send_response_timermutex);
                } else {
                    AI_GLASS_ERR("Send UART_RX_OPC_CMD_RECORD_START timer mutex failed\r\n");
                    lifetime_recording_deinitialize();
                    xSemaphoreGive(video_proc_sema);
                }
            } else {
                AI_GLASS_ERR("Failed to create send_response_timer\r\n");
                xSemaphoreGive(video_proc_sema);
            }
        } else {
            AI_GLASS_ERR("Failed because of the known record status\r\n");
            xSemaphoreGive(video_proc_sema);
        }
    } else {
        AI_GLASS_WARN("AI glass is snapshot or record, current record busy fail\r\n");
    }
 
    AI_GLASS_INFO("end of UART_RX_OPC_CMD_RECORD_START\r\n");
}

static void fTESTGSENSORCFG(void *arg) {
	AI_GLASS_INFO("get UART_RX_OPC_CMD_GSENSOR_CFG\r\n");
    uint8_t buf[512];
    size_t length = uart_serialize_camera_config(buf, sizeof(buf), &g_camera_cfg);

    // Debug print
    print_camera_config(&g_camera_cfg);
	
	printf("=== Raw UART Payload (%lu bytes) ===\n", length);
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", buf[i]);   // hex format
        if ((i + 1) % 16 == 0) {
            printf("\n");          // newline every 16 bytes
        }
    }
    if (length % 16 != 0) {
        printf("\n");
    }
	AI_GLASS_INFO("UART_RX_OPC_CMD_GSENSOR_CFG END\r\n");
}

log_item_t at_ai_glass_items[ ] = {
	{"AT+AIGLASSFORMAT",    fDISKFORMAT,            {NULL, NULL}},
	{"AT+AIGLASSGSENSOR",   fTESTGSENSOR,           {NULL, NULL}},
	{"AT+AIGLASSMSC",       fENABLEMSC,             {NULL, NULL}},
	{"AT+AIGLASSSETAPMODE", fENABLEAPMODE,          {NULL, NULL}},
	{"AT+AIGLASSLFSNAP",    fLFSNAPSHOT,            {NULL, NULL}},
	{"AT+AIGLASSSETSTAMODE", fENABLESTAMODE,        {NULL, NULL}},
	{"AT+AIGLASSCLEARMEDIAFLASH", fCLEARMEDIAFLASH, {NULL, NULL}},
	{"AT+AIGLASSCHANGESENSOR", fCHANGESENSOR,       {NULL, NULL}},
	{"AT+AIGLASSLFRECORD", fLFRECORD,               {NULL, NULL}},
	{"AT+AIGLASSGSENSORCFG", fTESTGSENSORCFG,       {NULL, NULL}},
};
#endif
void ai_glass_log_init(void)
{
#if defined(ENABLE_TEST_CMD) && ENABLE_TEST_CMD
	log_service_add_table(at_ai_glass_items, sizeof(at_ai_glass_items) / sizeof(at_ai_glass_items[0]));
#endif
#if EXTDISK_LOG
	ai_glass_extdisk_log_start();
#endif
}

log_module_init(ai_glass_log_init);
