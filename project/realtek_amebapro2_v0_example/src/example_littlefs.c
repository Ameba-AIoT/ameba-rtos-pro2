#include "FreeRTOS.h"
#include "task.h"
#include "basic_types.h"
#include "platform_opts.h"


#include <cmsis.h>
#include "FreeRTOS.h"
#include "task.h"
#include "basic_types.h"
#include "platform_opts.h"
#include "section_config.h"
#include "flash_api.h" // Flash interface
#include "lfs.h"
#include "lfs_util.h"
#include "lfs_nor_flash_api.h"

struct lfs_config *cfg = &nor_cfg; //Use the nor flash api, It can replace the differnet file operation/

int _traverse_df_cb(void *p, lfs_block_t block)
{
	uint32_t *nb = p;
	*nb += 1;
	return 0;
}

int littlefs_list(lfs_t *lfs, char *path)
{
	lfs_dir_t dir;
	struct lfs_info info;
	int ret = 0;
	char cur_path[512] = {0};
	ret = lfs_dir_open(lfs, &dir, path);
	if (ret == LFS_ERR_OK) {
		sprintf(cur_path, path);
		for (;;) {
			ret = lfs_dir_read(lfs, &dir, &info);
			if (ret <= 0) {
				break;
			}
			if (info.name[0] == '.') {
				continue;
			}
			if (info.type == LFS_TYPE_DIR) {
				sprintf(&cur_path[strlen(path)], "/%s", info.name);
				printf("List folder %s %d %d\r\n", info.name, info.type, info.size);
				littlefs_list(lfs, cur_path);
			} else {
				printf("List file %s %s %d %d\r\n", path, info.name, info.type, info.size);
			}
		}
	}
	//Close directory
	ret = lfs_dir_close(lfs, &dir);
	if (ret < 0) {
		printf("Close directory fail: %d\r\n", ret);
	}
	return ret;
}


int del_dir_littlefs(lfs_t *lfs, const char *path, int del_self)
{
	lfs_dir_t dir;
	struct lfs_info info;
	int ret = 0;
	ret = lfs_dir_open(lfs, &dir, path);
	char file[512] = {0};
	if (ret == LFS_ERR_OK) {
		for (;;) {
			// read directory and store it in file info object
			ret = lfs_dir_read(lfs, &dir, &info);
			if (ret <= 0) {
				break;
			}
			if (info.name[0] == '.') {
				continue;
			}

			//printf("%s %d %d\r\n",info.name,info.type,info.size);
			sprintf((char *)file, "%s/%s", path, info.name);
			if (info.type == LFS_TYPE_DIR) {
				del_dir_littlefs(lfs, file, del_self);
			} else {
				printf("Delete file %s type %d size %d\r\n", info.name, info.type, info.size);
				ret = lfs_remove(lfs, file);
			}
		}
	}

	// close directory
	ret = lfs_dir_close(lfs, &dir);

	// delete self?
	if (ret == LFS_ERR_OK) {
		if (del_self == 1) {
			ret = lfs_remove(lfs, path);
			if (ret >= 0) {
				printf("Delete folder %s\r\n", path);
			}
		}
	}
	return ret;
}

void example_littlefs_thread(void *param)
{
	lfs_t *lfs = malloc(sizeof(lfs_t));
	lfs_file_t *file = malloc(sizeof(lfs_file_t));
	lfs_dir_t *dir = malloc(sizeof(lfs_dir_t));
	struct lfs_info *info = malloc(sizeof(struct lfs_info));
	int ret = 0;
	char *str_content = "hello_world";
	char *file_name = "hello.txt";
	char *file_folder = "flash";
	char r_buf[64];
	memset(r_buf, 0, sizeof(r_buf));

	// mount the filesystem
	ret = lfs_mount(lfs, cfg);
	// reformat if we can't mount the filesystem
	// this should only happen on the first boot
	if (ret) {
		ret = lfs_format(lfs, cfg);
		if (ret) {
			printf("lfs_format fail %d\r\n", ret);
			goto EXIT;
		}
		ret = lfs_mount(lfs, cfg);
		if (ret < 0) {
			printf("lfs_mount fail %d\r\n", ret);
			goto EXIT;
		}
	}


	vTaskDelay(1000);
	del_dir_littlefs(lfs, "", 1);

	ret = lfs_file_open(lfs, file, file_name, LFS_O_WRONLY | LFS_O_CREAT);
	if (ret < 0) {
		printf("lfs_file_open fail %d\r\n", ret);
		goto EXIT;
	}

	ret = lfs_file_seek(lfs, file, 0, LFS_SEEK_SET);
	if (ret < 0) {
		printf("lfs_file_seek fail %d\r\n", ret);
		goto EXIT;
	}

	ret = lfs_file_write(lfs, file, str_content, strlen(str_content));
	if (ret < 0) {
		printf("lfs_file_write fail %d\r\n", ret);
		goto EXIT;
	}

	ret = lfs_file_close(lfs, file);
	if (ret < 0) {
		printf("lfs_file_close fail %d\r\n", ret);
		goto EXIT;
	}

	ret = lfs_file_open(lfs, file, "hello.txt", LFS_O_RDONLY);
	if (ret < 0) {
		printf("lfs_file_open fail %d\r\n", ret);
		goto EXIT;
	}

	ret = lfs_file_size(lfs, file);
	if (ret < 0) {
		printf("lfs_file_size fail %d\r\n", ret);
		goto EXIT;
	}
	printf("lfs_file_size %d\r\n", ret);

	ret = lfs_file_read(lfs, file, r_buf, strlen(str_content));
	if (ret < 0) {
		printf("lfs_file_size fail %d\r\n", ret);
		goto EXIT;
	}
	printf("File content %s\r\n", r_buf);

	ret = lfs_file_close(lfs, file);
	if (ret < 0) {
		printf("lfs_file_close fail %d\r\n", ret);
		goto EXIT;
	}

	ret = lfs_mkdir(lfs, file_folder);
	if (ret < 0) {
		printf("lfs_mkdir fail %d\r\n", ret);
		//goto EXIT;
	}
	/////////////////
	char path[64] = {0};
	sprintf(path, "%s/%s", file_folder, file_name);
	printf("open path %s\r\n", path);
	ret = lfs_file_open(lfs, file, path, LFS_O_WRONLY | LFS_O_CREAT);
	printf("lfs_file_open %d\r\n", ret);

	ret = lfs_file_write(lfs, file, str_content, strlen(str_content));
	if (ret < 0) {
		printf("lfs_file_write fail %d\r\n", ret);
		goto EXIT;
	}

	ret = lfs_file_close(lfs, file);
	if (ret < 0) {
		printf("lfs_file_close fail %d\r\n", ret);
		goto EXIT;
	}
	////////////////
	char buff[256] = {0};
	strcpy(buff, "");
	littlefs_list(lfs, buff);

	uint32_t _df_nballocatedblock = 0;
	ret = lfs_fs_traverse(lfs, _traverse_df_cb, &_df_nballocatedblock);
	if (ret < 0) {
		printf("lfs_fs_traverse fail %d\r\n", ret);
		goto EXIT;
	}
	uint32_t available = cfg->block_count * cfg->block_size - _df_nballocatedblock * cfg->block_size;
	printf("Avaliable space %d %d\r\n", available, _df_nballocatedblock);

	printf("lfs_fs_size %d\r\n", lfs_fs_size(lfs));
	//
	ret = lfs_unmount(lfs);
	if (ret < 0) {
		printf("lfs_fs_traverse fail %d\r\n", ret);
		goto EXIT;
	}
	printf("lfs_unmount %d\r\n", ret);
EXIT:
	vTaskDelete(NULL);
}

void example_littlefs(void)
{
	if (xTaskCreate(example_littlefs_thread, ((const char *)"example_littlefs_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_fatfs_thread) failed", __FUNCTION__);
	}
}
