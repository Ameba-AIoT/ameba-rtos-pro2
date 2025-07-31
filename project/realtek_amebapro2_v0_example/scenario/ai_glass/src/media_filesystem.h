#ifndef _MEDIA_FILESYSTEM_H
#define _MEDIA_FILESYSTEM_H

#include <platform_stdlib.h>
#include "platform_opts.h"
#include <cJSON.h>
#include "vfs.h"
#include "time.h"

#define SYS_COUNT_PIC_LABEL         "picture"
#define SYS_COUNT_FILM_LABEL        "film"
#define SYS_COUNT_SYS_LABEL         "sysfile"

enum {
	DIR_OPEN_FAIL   = -1,
	MEDIA_FILE_OK   = 0,
};

FILE *ramdisk_fopen(const char *filename, const char *mode);
int ramdisk_fclose(FILE *stream);
size_t ramdisk_fread(void *ptr, size_t size, size_t count, FILE *stream);
size_t ramdisk_fwrite(const void *ptr, size_t size, size_t count, FILE *stream);
int ramdisk_fseek(FILE *stream, long int offset, int origin);
int ramdisk_ftell(FILE *stream);
int ramdisk_feof(FILE *stream);
int ramdisk_remove(const char *filename);
void ramdisk_generate_unique_filename(const char *base_name, const char *time_str, const char *extension_name, char *new_name, uint32_t size);

FILE *extdisk_fopen(const char *filename, const char *mode);
int extdisk_fclose(FILE *stream);
size_t extdisk_fread(void *ptr, size_t size, size_t count, FILE *stream);
size_t extdisk_fwrite(const void *ptr, size_t size, size_t count, FILE *stream);
int extdisk_fseek(FILE *stream, long int offset, int origin);
int extdisk_ftell(FILE *stream);
int extdisk_feof(FILE *stream);
int extdisk_remove(const char *filename);
void extdisk_generate_unique_filename(const char *base_name, const char *time_str, const char *extension_name, char *new_name, uint32_t size);
int extdisk_get_filecount(const char *count_label);
int extdisk_save_file_cntlist(void);
int extdisk_delete_bin_files(void);

void extdisk_count_filenum(const char *dir_path, const char **extensions, uint16_t *ext_counts, uint16_t num_extensions, const char *exclude_filename);
cJSON *extdisk_get_filelist(const char *list_path, uint16_t *file_number, const char **extensions, uint16_t num_extensions, const char *exclude_filename);
const char *extdisk_get_filesystem_tag_name(void);

int extdisk_filesystem_init(const char *disk_tag, int vfs_type, int interface);
int extdisk_filesystem_deinit(const char *disk_tag, int vfs_type, int interface);
int ramdisk_filesystem_init(const char *disk_tag);

int extdisk_get_init_status(void);
int ramdisk_get_init_status(void);

int media_filesystem_init(void);
const char *media_filesystem_get_current_time_string(void);
time_t media_filesystem_gpstime_to_time_t(uint32_t gps_week, uint32_t gps_seconds);
void media_filesystem_setup_gpstime(uint32_t gps_week, uint32_t gps_seconds);
int media_filesystem_setup_gpscoordinate(float latitude, float longitude, float altitude);
void media_filesystem_get_gpscoordinate(float *latitude, float *longitude, float *altitude);
#endif
