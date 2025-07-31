#include "FreeRTOS.h"
#include "task.h"
#include "platform_opts.h"
#include "osdep_service.h"
#include "section_config.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "vfs.h"
#include "time.h"

#define STACK_SIZE		4096

int list_files(const char *list_path);
int del_dir(const char *path, int del_self);

FILE     *m_file;

#define TEST_BUF_SIZE	(512)
#define MAXIMUM_FILE_SIZE 100

int filter_fn(const struct dirent *ent)
{
	if (ent->d_type == DT_DIR) {
		//printf("dir %x\r\n",ent->d_type);
		return 0;
	} else {
		//printf("file %x\r\n",ent->d_type);
		return 1;
	}
}
#define TEST_INTERFACE_SD   0x00 //Fatfs
#define TEST_INTERFACE_LFS  0x01 //Littlefs
#define TEST_INTERFACE_RAM  0x02 //Fatfs
void example_std_file_thread(void *param)
{
	char WRBuf[TEST_BUF_SIZE] = {0};
	char RDBuf[TEST_BUF_SIZE] = {0};
	char test_info[] = "### Ameba test standard file VFS ###\n\r";
	int res = 0;
	char path[64];
	/* test files */
	char sd_fn[20] = "sd_file";
	char sd_dir[20] = "sd_dir";
	char sd_file2[20] = "file2";
	char sd_dir2[20] = "dir2";

	int br, bw = 0;

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	vTaskDelay(2000);
	printf("=== STD File Example ===\n\r");

	int interface = 3;
	int i = 0;
	vfs_init(NULL);
	vfs_user_register("sd", VFS_FATFS, VFS_INF_SD);
	vfs_user_register("lfs", VFS_LITTLEFS, 0);
	vfs_user_register("ram", VFS_FATFS, VFS_INF_RAM);

	const char *tag = NULL;
	for (i = 0; i < interface; i++) {
		memset(path, 0x00, sizeof(path));
		if (i == TEST_INTERFACE_SD) {
			tag = "sd:/";
		} else if (i == TEST_INTERFACE_LFS) {
			tag = "lfs:/";
		} else if (i == TEST_INTERFACE_RAM) {
			tag = "ram:/";
		} else {
			goto exit;
		}
		strcpy(path, tag);
		sprintf(&path[strlen(path)], "%s", sd_fn);

		printf("\n\r=== Delete test file ===\n\r");
		del_dir(tag, 0);

		printf("\n\r=== %s card FS Read/Write test ===\n\r", tag);
		m_file = fopen(path, "a+");

		if (!m_file) {
			printf("open file (%s) fail.\n", sd_fn);
			goto exit;
		}

		printf("Test file name: %s\n\n", sd_fn);

		memset(&WRBuf[0], 0x00, TEST_BUF_SIZE);
		memset(&RDBuf[0], 0x00, TEST_BUF_SIZE);

		strcpy(&WRBuf[0], &test_info[0]);

		do {
			bw = fwrite(WRBuf, 1, strlen(WRBuf), m_file);
			if (bw < 0) {
				fseek(m_file, 0, SEEK_SET);
				printf("Write error.\n");
			}
			printf("Write %d bytes.\n", bw);
		} while (bw < strlen(WRBuf));

		printf("Write content:\n%s\n", WRBuf);

		/* move the file pointer to the file head*/
		res = fseek(m_file, 0, SEEK_SET);

		br = fread(RDBuf, 1, TEST_BUF_SIZE, m_file);
		if (br < 0) {
			fseek(m_file, 0, SEEK_SET);
			printf("Read error.\n");
		}
		printf("Read %d bytes.\n", br);

		printf("Read content:\n%s\n", RDBuf);
		res = fclose(m_file);
		if (res) {
			printf("close file (%s) fail.\n", sd_fn);
		}

		// create directory: tag:/sd_dir
		memset(path, 0, sizeof(path));
		snprintf(path, sizeof(path), "%s%s", tag, sd_dir);
		printf("\n\rmkdir: %s \n\r", path);
		res = mkdir(path, 0);
		printf("mkdir: %d\n\r", res);

		// create file: tag:/sd_dir/file2
		memset(path, 0, sizeof(path));
		snprintf(path, sizeof(path), "%s%s/%s", tag, sd_dir, sd_file2);
		printf("\n\rCreate file: %s \n\r", path);
		m_file = fopen(path, "a+");
		res = fclose(m_file);
		if (res) {
			printf("close file (%s) fail.\n", sd_fn);
		}

		///////////////////////////////////////////////
		// create directory: tag:/sd_dir/dir2
		memset(path, 0, sizeof(path));
		snprintf(path, sizeof(path), "%s%s/%s", tag, sd_dir, sd_dir2);
		printf("\n\rmkdir: %s \n\r", path);
		res = mkdir(path, 0);
		printf("mkdir: %d\n\r", res);

		// read directory: 0:/
		struct dirent *dp;
		memset(path, 0, sizeof(path));
		snprintf(path, sizeof(path), "%s%s", tag, sd_dir);
		list_files(tag);
		// scandir: tag:/
		struct dirent **namelist = NULL;
		int count = scandir(tag, &namelist, filter_fn, alphasort);
		printf("==> %d files in %s \n\r", count, tag);
		if(count >= 0){
			for (int i = 0; i < count; i++) {
				printf("[%d] %s\n\r", i, namelist[i]->d_name);
			}
			for (int i = 0; i < count; i++) {
				free(namelist[i]);
			}
			free(namelist);
		}else{
			printf("scandir failed\r\n");
		}

		// stat: tag:/sd_file
		printf("\n\rstat: %ssd_file\n\r", tag);
		struct stat fstat;
		memset(path, 0, sizeof(path));
		snprintf(path, sizeof(path), "%s%s", tag, "sd_file");
		stat(path, &fstat);
		printf("file size: %d bytes\n\r", fstat.st_size);
		printf("file time: %d (Epoch)\n\r", fstat.st_atime);

		// access: tag:/some_file
		printf("\n\raccess: %ssome_file (Does not exist)\n\r", tag);
		memset(path, 0, sizeof(path));
		snprintf(path, sizeof(path), "%s%s", tag, "some_file");
		res = access(path, W_OK);
		printf("acess: %d\n\r", res);

		printf("\n\raccess: %ssd_file\n\r", tag);
		memset(path, 0, sizeof(path));
		snprintf(path, sizeof(path), "%s%s", tag, "sd_file");
		res = access(path, W_OK);
		printf("acess: %d\n\r", res);
		vTaskDelay(2000);
	}

	vfs_user_unregister("sd", VFS_FATFS, VFS_INF_SD);
	vfs_user_unregister("lfs", VFS_LITTLEFS, 0);
	vfs_user_unregister("ram", VFS_FATFS, VFS_INF_RAM);
	vfs_deinit(NULL);

exit:
	vTaskDelete(NULL);
}

int del_dir(const char *list_path, int del_self)
{
	int res;
	DIR *m_dir;
	char *filename;
	char path[1024];
	char file[1024];
	struct dirent *entry;
	m_dir = opendir(list_path);
	sprintf(path, "%s", list_path);

	if (m_dir) {
		for (;;) {
			// read directory and store it in file info object
			entry = readdir(m_dir);

			if (!entry) {
				break;
			}
			filename = entry->d_name;

			if (filename[0] == '.' || ((filename[0] == '.') && (filename[1] == '.'))) {
				continue;
			}

			sprintf((char *)file, "%s/%s", path, filename);

			if (entry->d_type == DT_DIR) {
				printf("del dir: %s\n\r", file);
				del_dir(file, 1);
			} else {
				printf("del file: %s\n\r", file);
				remove(file);
			}
		}
	}

	// close directory
	res = closedir(m_dir);

	// delete self?
	if (res == 0) {
		if (del_self == 1) {
			res = remove(path);
		}
	}

	return res;
}

void print_file_info(struct dirent *entry, char *path)
{
	char info[512];

	snprintf(info, sizeof(info),
			 "%c  %9u  %30s  %30s",
			 (entry->d_type == DT_DIR) ? 'D' : 'F',
			 entry->d_reclen,
			 entry->d_name,
			 path);
	printf("%s\n\r", info);
}

int list_files(const char *list_path)
{
	int res = 0;
	DIR *m_dir;
	char *filename;
	char path[1024];
	char file[512];
	struct dirent *entry;

	m_dir = opendir(list_path);
	snprintf(path, sizeof(path), "%s", list_path);
	if (m_dir) {
		for (;;) {
			// read directory
			entry = readdir(m_dir);

			if (!entry) {
				break;
			}
			filename = entry->d_name;
			if (filename[0] == '.' || ((filename[0] == '.') && (filename[1] == '.'))) {
				continue;
			}

			if (entry->d_type == DT_DIR) {
				snprintf((char *)file, sizeof(file), "%s/%s", path, filename);
				print_file_info(entry, path);
				res = list_files(file);
				if (res != 0) {
					break;
				}
			} else {
				print_file_info(entry, path);
			}
		}
	}

	// close directory
	res = closedir(m_dir);

	return res;
}

void example_std_file(void)
{
	if (xTaskCreate(example_std_file_thread, ((const char *)"example_std_file_thread"), STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_std_file_thread) failed", __FUNCTION__);
	}
}
