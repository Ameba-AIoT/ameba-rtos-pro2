// JSON file saved in SD Card

#include "FreeRTOS.h"
#include "task.h"
#include "platform_opts.h"
#include "osdep_service.h"
#include "section_config.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "time.h"
#include "log_service.h"
#include <cJSON.h>
#include "platform_stdlib.h"
#include "media_filesystem.h"
#include "time.h"
//#include "ff.h"
#include "vfs.h"
#include "mmf2_mediatime_8735b.h"

#define SHOW_LIST_FILE_TIME 0
#define ENABLE_FILE_TIME_FUNCTION   1 // If enable this flag, please set original get_fattime to weak function

#define MAX_TAG_LEN     32
#define MAX_FILE_LEN    128
static char ai_glass_extdisk_tag[MAX_TAG_LEN] = {0};
static int ai_glass_extdisk_done = 0;
static SemaphoreHandle_t extdisk_mutex = NULL;

static char ai_glass_ramdisk_tag[MAX_TAG_LEN] = {0};
static int ai_glass_ramdisk_done = 0;
static SemaphoreHandle_t ramdisk_mutex = NULL;

static time_t gps_timeinfo = 0;

time_t media_filesystem_gpstime_to_time_t(uint32_t gps_week, uint32_t gps_seconds);

#if ENABLE_FILE_TIME_FUNCTION
unsigned long get_fattime(void)
{
	time_t cur_time = gps_timeinfo + mm_read_mediatime_ms() / 1000;
	struct tm *timeinfo = gmtime(&cur_time);

	unsigned long time_abs = 0;
	time_abs |= ((timeinfo->tm_year - 80) & 0x7F) << 25;
	time_abs |= ((timeinfo->tm_mon + 1) & 0x0F) << 21;
	time_abs |= (timeinfo->tm_mday & 0x1F) << 16;
	time_abs |= (timeinfo->tm_hour & 0x1F) << 11;
	time_abs |= (timeinfo->tm_min & 0x3F) << 5;
	time_abs |= (timeinfo->tm_sec / 2) & 0x1F;

	return time_abs;
}
#endif

static int show_utc_format_time(time_t rawtime)
{
	struct tm *timeinfo;

	//time(&rawtime);
	timeinfo = gmtime(&rawtime);

	printf("UTC format time: %s\r\n", asctime(timeinfo));
	return 0;
}

void media_filesystem_setup_gpstime(uint32_t gps_week, uint32_t gps_seconds)
{
	gps_timeinfo = media_filesystem_gpstime_to_time_t(gps_week, gps_seconds);
	printf("Set up GPS time for system\r\n");
	show_utc_format_time(gps_timeinfo);
}

#define LEAP_SECONDS 0 // Todo: If user want to transfer actual UTC time, the could use this
time_t media_filesystem_gpstime_to_time_t(uint32_t gps_week, uint32_t gps_seconds)
{
	struct tm gps_start_time = {0};
	gps_start_time.tm_year = 1980 - 1900;
	gps_start_time.tm_mon = 0;
	gps_start_time.tm_mday = 6;
	gps_start_time.tm_hour = 0;
	gps_start_time.tm_min = 0;
	gps_start_time.tm_sec = 0;

	// convert GPS start time to time_t format
	time_t gps_start_time_t = mktime(&gps_start_time);

	uint64_t total_seconds = (gps_week * 604800) + gps_seconds;

	time_t utcform_time_t = gps_start_time_t + total_seconds + LEAP_SECONDS;

	return utcform_time_t;
}

static void time_transfer_to_string_utcform(time_t rawtime, char *buffer, uint32_t buffer_size)
{
	struct tm *timeinfo = gmtime(&rawtime);

	strftime(buffer, buffer_size, "%Y-%m-%dT%H:%M:%S%z", timeinfo);
	//strftime(buffer, buffer_size, "%Y-%m-%dT%H:%M:%SZ", timeinfo);
	return;
}

static void time_transfer_to_string(time_t rawtime, char *buffer, uint32_t buffer_size)
{
	struct tm *timeinfo = gmtime(&rawtime);

	strftime(buffer, buffer_size, "%Y%m%d_%H%M%S", timeinfo);
	printf("get time = %s\r\n", buffer);
	//strftime(buffer, buffer_size, "%Y-%m-%dT%H:%M:%SZ", timeinfo);
	return;
}

#define CUR_TIME_BUFFER_SIZE 30
const char *media_filesystem_get_current_time_string(void)
{
	// Todo: add gps information
	time_t cur_time = gps_timeinfo + mm_read_mediatime_ms() / 1000;
	char *time_buffer = malloc(CUR_TIME_BUFFER_SIZE);
	time_transfer_to_string(cur_time, time_buffer, CUR_TIME_BUFFER_SIZE);
	return time_buffer;
}

int media_filesystem_init(void)
{
	vfs_init(NULL);
	extdisk_mutex = xSemaphoreCreateMutex();
	if (extdisk_mutex == NULL) {
		printf("extdisk_mutex create fail\r\n");
		vfs_deinit(NULL);
		return -1;
	}
	ramdisk_mutex = xSemaphoreCreateMutex();
	if (ramdisk_mutex == NULL) {
		printf("ramdisk_mutex create fail\r\n");
		vfs_deinit(NULL);
		vSemaphoreDelete(extdisk_mutex);
		return -1;
	}
	return 0;
}

static int is_excluded_file(const char *filename, const char *exclude_filename)
{
	uint32_t len_filename = strlen(filename);
	uint32_t len_exclude = strlen(exclude_filename);
	if (len_filename < len_exclude) {
		return 0;
	}
	return strcmp(filename + len_filename - len_exclude, exclude_filename) == 0;
}

static int check_extension(const char *filename, const char *ext)
{
	uint32_t len_filename = strlen(filename);
	uint32_t len_ext = strlen(ext);
	if (len_filename < len_ext) {
		return 0;
	}
	return strcmp(filename + len_filename - len_ext, ext) == 0;
}

// return extensions' num
// if not exist, return -1
static int check_valid_file_and_remove(const char *list_path, const char *filename, const char *exclude_filename, const char **extensions,
									   uint16_t num_extensions)
{
	struct stat finfo = {0};
	int res;

	if (is_excluded_file(filename, exclude_filename)) {
		printf("file %s is a exclude file %s\n", filename, exclude_filename);
		return -1;
	} else {
		char *file_path = malloc(PATH_MAX + 1);
		if (file_path) {
			snprintf(file_path, PATH_MAX + 1, "%s/%s", list_path, filename);
			res = stat(file_path, &finfo);

			if (res == 0) {
				if (finfo.st_size == 0) {
					printf("size 0 is invlaid %s, remove file\r\n", file_path);
					remove(file_path);
					free(file_path);
					return -1;
				}
			} else {
				printf("Failed to get file %s info (error: %d), remove file\r\n", file_path, res);
				remove(file_path);
				free(file_path);
				return -1;
			}
			free(file_path);
		}
		for (int i = 0; i < num_extensions; i++) {
			if (check_extension(filename, extensions[i])) {
				return i;
			}
		}
		//printf("%s check_extension failed\r\n", filename);
	}

	return -1;
}

static int file_exists(const char *filename)
{
	FILE *file = fopen(filename, "r");
	if (file) {
		fclose(file);
		return 1;
	}
	return 0;
}

FILE *extdisk_fopen(const char *filename, const char *mode)
{
	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return NULL;
	}
	char ai_glass_path[MAX_FILE_LEN] = {0};
	snprintf(ai_glass_path, sizeof(ai_glass_path), "%s%s", ai_glass_extdisk_tag, filename);
	return fopen(ai_glass_path, mode);
}

int extdisk_fclose(FILE *stream)
{
	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return -1;
	}
	return fclose(stream);
}

size_t extdisk_fread(void *ptr, size_t size, size_t count, FILE *stream)
{
	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return -1;
	}
	return fread(ptr, size, count, stream);
}

size_t extdisk_fwrite(const void *ptr, size_t size, size_t count, FILE *stream)
{
	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return -1;
	}
	return fwrite(ptr, size, count, stream);
}

int extdisk_fseek(FILE *stream, long int offset, int origin)
{
	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return -1;
	}
	return fseek(stream, offset, origin);
}

int extdisk_ftell(FILE *stream)
{
	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return -1;
	}
	return ftell(stream);
}

int extdisk_feof(FILE *stream)
{
	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return -1;
	}
	return feof(stream);
}

int extdisk_remove(const char *filename)
{
	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return -1;
	}
	char ai_glass_path[MAX_FILE_LEN] = {0};
	snprintf(ai_glass_path, sizeof(ai_glass_path), "%s%s", ai_glass_extdisk_tag, filename);
	return remove(ai_glass_path);
}

void extdisk_generate_unique_filename(const char *base_name, const char *time_str, const char *extension_name, char *new_name, uint32_t size)
{
	if (snprintf(new_name, size, "%s%s", base_name, time_str) >= size) {
		printf("Error: Filename buffer size too small.\n");
		return;
	}

	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return;
	}
	int count = 1;
	char temp_path[MAX_FILE_LEN] = {0};
	snprintf(temp_path, sizeof(temp_path), "%s%s%s", ai_glass_extdisk_tag, new_name, extension_name);

	while (file_exists(temp_path)) {
		if (snprintf(new_name, size, "%s%s_%d", base_name, time_str, count) >= size) {
			printf("Error: Filename buffer size too small.\n");
			return;
		}

		if (snprintf(temp_path, sizeof(temp_path), "%s%s%s", ai_glass_extdisk_tag, new_name, extension_name) >= sizeof(temp_path)) {
			printf("Error: Path buffer size too small.\n");
			return;
		}
		count++;
	}
}

static void extdisk_get_filenum(const char *dir_path, const char **extensions, uint16_t *ext_counts, uint16_t num_extensions, const char *exclude_filename)
{
	DIR *file_dir = NULL;
	char *filename;
	dirent *entry;
	//printf("survey dir: %s\r\n", dir_path);
	uint8_t *sub_dir_path = malloc(PATH_MAX + 1);
	file_dir = opendir(dir_path);

	if (file_dir) {
		for (;;) {
			// read directory
			entry = readdir(file_dir);

			if (!entry) {
				break;
			}
			filename = entry->d_name;
			if (filename[0] == '.' || ((filename[0] == '.') && (filename[1] == '.'))) {
				continue;
			}

			if (entry->d_type == DT_DIR) {
				if (sub_dir_path) {
					if (snprintf((char *)sub_dir_path, PATH_MAX + 1, "%s/%s", dir_path, entry->d_name) < (PATH_MAX + 1)) {
						extdisk_get_filenum((const char *)sub_dir_path, extensions, ext_counts, num_extensions, exclude_filename);
					}
				}
			} else {
				int extension_num = check_valid_file_and_remove(dir_path, entry->d_name, exclude_filename, extensions, num_extensions);
				if (extension_num >= 0) {
					ext_counts[extension_num]++;
				}
			}
		}
		// close directory
		closedir(file_dir);
	}
	if (sub_dir_path) {
		free(sub_dir_path);
	}
	return;
}

void extdisk_count_filenum(const char *dir_path, const char **extensions, uint16_t *ext_counts, uint16_t num_extensions, const char *exclude_filename)
{
	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return;
	}
	char ai_glass_path[MAX_FILE_LEN] = {0};
	snprintf(ai_glass_path, sizeof(ai_glass_path), "%s%s", ai_glass_extdisk_tag, dir_path);
	for (uint16_t i = 0; i < num_extensions; i++) {
		ext_counts[i] = 0;
	}

	extdisk_get_filenum(ai_glass_path, extensions, ext_counts, num_extensions, exclude_filename);

	return;
}

static cJSON *create_json_folder_object(const char *list_path, const char *foldername)
{
	cJSON *folder_obj = cJSON_CreateObject();
	cJSON_AddStringToObject(folder_obj, "type", "folder");
	cJSON_AddStringToObject(folder_obj, "name", foldername);
	struct stat finfo = {0};
	int res;
	res = stat(list_path, &finfo);
#if SHOW_LIST_FILE_TIME
	if (res == 0) {
		show_utc_format_time(finfo.st_mtime);
		printf("Folder: %s\n", list_path);
	} else {
		printf("Failed to get folder %s info (error: %d)\n", list_path, res);
	}
#endif
	//cJSON_AddNumberToObject(folder_obj, "time", finfo.st_mtime);
	char utctime_buffer[30] = {0};
	time_transfer_to_string_utcform(finfo.st_mtime, utctime_buffer, sizeof(utctime_buffer));
	cJSON_AddStringToObject(folder_obj, "time", (const char *)utctime_buffer);
	return folder_obj;
}

static cJSON *create_json_file_object(const char *list_path, const char *filename)
{
	cJSON *folder_obj = cJSON_CreateObject();
	cJSON_AddStringToObject(folder_obj, "type", "file");
	cJSON_AddStringToObject(folder_obj, "name", filename);
	struct stat finfo = {0};
	int res;
	char *file_path = malloc(PATH_MAX + 1);
	if (file_path) {
		snprintf(file_path, PATH_MAX + 1, "%s/%s", list_path, filename);
		res = stat(file_path, &finfo);
#if SHOW_LIST_FILE_TIME
		if (res == 0) {
			show_utc_format_time(finfo.st_mtime);
			printf("File: %s\n", file_path);
		} else {
			printf("Failed to get file %s info (error: %d)\n", file_path, res);
		}
#endif
		free(file_path);
	}
	//cJSON_AddNumberToObject(folder_obj, "time", finfo.st_mtime);
	char utctime_buffer[30] = {0};
	time_transfer_to_string_utcform(finfo.st_mtime, utctime_buffer, sizeof(utctime_buffer));
	cJSON_AddStringToObject(folder_obj, "time", (const char *)utctime_buffer);
	return folder_obj;
}

static cJSON *get_filelist(const char *list_path, const char *folder_name, uint16_t *file_number, const char **extensions, uint16_t num_extensions,
						   const char *exclude_filename)
{
	//int res = 0;
	DIR *file_dir;
	char *filename;
	dirent *entry;
	uint8_t *sub_dir_path = malloc(PATH_MAX + 1);

	cJSON *filelist_obj = NULL;
	filelist_obj = create_json_folder_object(list_path, folder_name);
	if (filelist_obj == NULL) {
		goto endoffun;
	}

	file_dir = opendir(list_path);

	if (file_dir) {
		cJSON *contents = cJSON_CreateArray();
		// each folder
		for (;;) {
			// read directory
			entry = readdir(file_dir);

			if (!entry) {
				break;
			}
			filename = entry->d_name;

			// Skips hidden files and directories, as well as the special . and .. entries.
			// More folder name of hidden folders can be added to skip if needed.
			if (filename[0] == '.' || (strcmp(filename, "System Volume Information") == 0) || ((filename[0] == '.') && (filename[1] == '.'))) {
				continue;
			}

			if (entry->d_type == DT_DIR) {
				if (sub_dir_path) {
					snprintf((char *)sub_dir_path, PATH_MAX + 1, "%s/%s", list_path, entry->d_name);
					cJSON_AddItemToArray(contents, get_filelist((const char *)sub_dir_path, (const char *)entry->d_name, file_number, extensions, num_extensions,
										 exclude_filename));
				}
			} else {
				// Add file to contents
				if (check_valid_file_and_remove(list_path, entry->d_name, exclude_filename, extensions, num_extensions) >= 0) {
					cJSON *file_obj = create_json_file_object(list_path, entry->d_name);
					if (file_obj != NULL) {
						cJSON_AddItemToArray(contents, file_obj);
						(*file_number)++;
					} else {
						printf("get file %s failed\r\n", entry->d_name);
					}
				} else {
					printf("file %s is not valid\r\n", entry->d_name);
				}
			}
		}
		cJSON_AddItemToObject(filelist_obj, "contents", contents);
		// close directory
		closedir(file_dir);
	}
endoffun:
	if (sub_dir_path) {
		free(sub_dir_path);
	}

	return filelist_obj;
}

cJSON *extdisk_get_filelist(const char *list_path, uint16_t *file_number, const char **extensions, uint16_t num_extensions, const char *exclude_filename)
{
	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return NULL;
	}
	char ai_glass_path[MAX_FILE_LEN] = {0};
	snprintf(ai_glass_path, sizeof(ai_glass_path), "%s%s", ai_glass_extdisk_tag, list_path);

	*file_number = 0;

	return get_filelist(ai_glass_path, ai_glass_path, file_number, extensions, num_extensions, exclude_filename);
}

const char *extdisk_get_filesystem_tag_name(void)
{
	if (!ai_glass_extdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return NULL;
	}
	char *tag_name = malloc(MAX_TAG_LEN);
	if (tag_name) {
		snprintf(tag_name, MAX_TAG_LEN, "%s", ai_glass_extdisk_tag);
	}
	return tag_name;
}

void ramdisk_generate_unique_filename(const char *base_name, const char *time_str, const char *extension_name, char *new_name, uint32_t size)
{
	if (snprintf(new_name, size, "%s%s", base_name, time_str) >= size) {
		printf("Error: Filename buffer size too small.\n");
		return;
	}

	if (!ai_glass_ramdisk_done) {
		printf("External disk is not initialized yet\r\n");
		return;
	}
	int count = 1;
	char temp_path[MAX_FILE_LEN] = {0};
	snprintf(temp_path, sizeof(temp_path), "%s%s%s", ai_glass_ramdisk_tag, new_name, extension_name);

	while (file_exists(temp_path)) {
		if (snprintf(new_name, size, "%s%s_%d", base_name, time_str, count) >= size) {
			printf("Error: Filename buffer size too small.\n");
			return;
		}

		if (snprintf(temp_path, sizeof(temp_path), "%s%s%s", ai_glass_ramdisk_tag, new_name, extension_name) >= sizeof(temp_path)) {
			printf("Error: Path buffer size too small.\n");
			return;
		}
		count++;
	}
}

FILE *ramdisk_fopen(const char *filename, const char *mode)
{
	if (!ai_glass_ramdisk_done) {
		printf("Ram disk is not initialized yet\r\n");
		return NULL;
	}
	char ai_glass_path[MAX_FILE_LEN] = {0};
	snprintf(ai_glass_path, sizeof(ai_glass_path), "%s%s", ai_glass_ramdisk_tag, filename);
	return fopen(ai_glass_path, mode);
}

int ramdisk_fclose(FILE *stream)
{
	if (!ai_glass_ramdisk_done) {
		printf("Ram disk is not initialized yet\r\n");
		return -1;
	}
	return fclose(stream);
}

size_t ramdisk_fread(void *ptr, size_t size, size_t count, FILE *stream)
{
	if (!ai_glass_ramdisk_done) {
		printf("Ram disk is not initialized yet\r\n");
		return -1;
	}
	return fread(ptr, size, count, stream);
}

size_t ramdisk_fwrite(const void *ptr, size_t size, size_t count, FILE *stream)
{
	if (!ai_glass_ramdisk_done) {
		printf("Ram disk is not initialized yet\r\n");
		return -1;
	}
	return fwrite(ptr, size, count, stream);
}

int ramdisk_fseek(FILE *stream, long int offset, int origin)
{
	if (!ai_glass_ramdisk_done) {
		printf("Ram disk is not initialized yet\r\n");
		return -1;
	}
	return fseek(stream, offset, origin);
}

int ramdisk_ftell(FILE *stream)
{
	if (!ai_glass_ramdisk_done) {
		printf("Ram disk is not initialized yet\r\n");
		return -1;
	}
	return ftell(stream);
}

int ramdisk_feof(FILE *stream)
{
	if (!ai_glass_ramdisk_done) {
		printf("Ram disk is not initialized yet\r\n");
		return -1;
	}
	return feof(stream);
}

int ramdisk_remove(const char *filename)
{
	if (!ai_glass_ramdisk_done) {
		printf("Ram disk is not initialized yet\r\n");
		return -1;
	}
	char ai_glass_path[MAX_FILE_LEN] = {0};
	snprintf(ai_glass_path, sizeof(ai_glass_path), "%s%s", ai_glass_extdisk_tag, filename);
	return remove(ai_glass_path);
}

int extdisk_get_init_status(void)
{
	return ai_glass_extdisk_done;
}

int ramdisk_get_init_status(void)
{
	return ai_glass_ramdisk_done;
}

int extdisk_filesystem_init(const char *disk_tag, int vfs_type, int interface)
{
	if (xSemaphoreTake(extdisk_mutex, portMAX_DELAY) != pdTRUE) {
		printf("Wait for extdisk mutex timeout\r\n");
		return -1;
	}
	if (ai_glass_extdisk_done) {
		printf("External disk has been initialized\r\n");
		xSemaphoreGive(extdisk_mutex);
		return -1;
	}
	memset(ai_glass_extdisk_tag, 0, MAX_TAG_LEN);
	snprintf(ai_glass_extdisk_tag, MAX_TAG_LEN, "%s:/", disk_tag);
	printf("ai glass get extdisk name %s\r\n", ai_glass_extdisk_tag);
	printf("ai glass extdisk time %d\r\n", mm_read_mediatime_ms());
	vfs_user_register(disk_tag, vfs_type, interface); // cost about 100ms for SD card, 150ms for EMMC card
	ai_glass_extdisk_done = 1;
	printf("ai glass extdisk done time %d\r\n", mm_read_mediatime_ms());
	xSemaphoreGive(extdisk_mutex);
	return 0;
}

int ramdisk_filesystem_init(const char *disk_tag)
{
	if (xSemaphoreTake(ramdisk_mutex, portMAX_DELAY) != pdTRUE) {
		printf("Wait for ramdisk mutex timeout\r\n");
		return -1;
	}
	if (ai_glass_ramdisk_done) {
		printf("External disk has been initialized\r\n");
		xSemaphoreGive(ramdisk_mutex);
		return -1;
	}
	memset(ai_glass_ramdisk_tag, 0, MAX_TAG_LEN);
	snprintf(ai_glass_ramdisk_tag, MAX_TAG_LEN, "%s:/", disk_tag);
	printf("ai glass get ramdisk name %s\r\n", ai_glass_ramdisk_tag);
	printf("ai glass ramdisk time %d\r\n", mm_read_mediatime_ms());
	vfs_user_register(disk_tag, VFS_FATFS, VFS_INF_RAM); // cost about 10ms for RAM disk
	ai_glass_ramdisk_done = 1;
	printf("ai glass ramdisk done time %d\r\n", mm_read_mediatime_ms());
	xSemaphoreGive(ramdisk_mutex);
	return 0;
}