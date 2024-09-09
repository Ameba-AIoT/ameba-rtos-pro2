#include <cmsis.h>
#include "FreeRTOS.h"
#include "task.h"
#include "platform_opts.h"
#include "section_config.h"
#include "flash_api.h" // Flash interface
#include "lfs.h"
#include "lfs_util.h"
#include "lfs_nor_flash_api.h"
#include "lfs_nand_flash_api.h"
#include "ftl_common_api.h"
#include "lfs_reent.h"


#define NOR_BLOCK_COUNT  100
#define NAND_FLASH_BLOCK_COUNT 100

int ftl_block_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int ftl_block_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int ftl_block_erase(const struct lfs_config *c, lfs_block_t block);
int ftl_block_sync(const struct lfs_config *c);

struct lfs_config ftl_cfg = {
	// block device operations
	.read  = ftl_block_read,
	.prog  = ftl_block_prog,
	.erase = ftl_block_erase,
	.sync  = ftl_block_sync,
	.lock  =  lfs_system_lock,
	.unlock = lfs_system_unlock,
	// block device configuration
	.read_size = 0,
	.prog_size = 0,
	.block_size = 0,
	.block_count = 0,
	.cache_size = 0,
	.block_cycles = 100,
	.lookahead_size = 0
	//.lookahead_buffer = 256,
};

int ftl_block_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
	int ret = 0;
	if (sys_get_boot_sel() == FTL_NAND_FLASH) {
		ret = ftl_common_read(((NAND_APP_BASE / ftl_cfg.block_size) + block) * c->block_size + off, buffer, size);
	} else {
		ftl_common_read(block * c->block_size + FLASH_APP_BASE + off, buffer, size);
	}
	if (ret == 0) {
		ret = LFS_ERR_OK;
	} else {
		ret = LFS_ERR_IO;
	}
	return ret;
}

int ftl_block_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	int ret = 0;
	if (sys_get_boot_sel() == FTL_NAND_FLASH) {
		ret = ftl_common_write(((NAND_APP_BASE / ftl_cfg.block_size) + block) * c->block_size + off, (unsigned char *)buffer, size);
	} else {
		ftl_common_write(block * c->block_size + FLASH_APP_BASE + off, (unsigned char *)buffer, size);
	}
	if (ret == 0) {
		ret = LFS_ERR_OK;
	} else {
		ret = LFS_ERR_IO;
	}
	return ret;
}
int ftl_block_erase(const struct lfs_config *c, lfs_block_t block)
{
	int ret = 0;
	if (sys_get_boot_sel() == FTL_NAND_FLASH) {
		ret = ftl_common_erase(((NAND_APP_BASE / ftl_cfg.block_size) + block) * c->block_size);
	} else {
		ftl_common_erase(block * c->block_size + FLASH_APP_BASE);
	}
	if (ret == 0) {
		ret = LFS_ERR_OK;
	} else {
		ret = LFS_ERR_IO;
	}
	return ret;
}
int ftl_block_sync(const struct lfs_config *c)
{
	return 0;
}

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
	const char *str_content = "hello_world";
	const char *file_name = "hello.txt";
	const char *file_folder = "flash";
	char r_buf[64];
	memset(r_buf, 0, sizeof(r_buf));

	int type, page_size, block_size, block_cnt = 0;
	ftl_common_info(&type, &page_size, &block_size, &block_cnt);
	printf("type %d page_size %d block_size %d block_cnt %d\r\n", type, page_size, block_size, block_cnt);

	if (type == 0) {
		printf("It is nor flash\r\n");
		ftl_cfg.read_size = page_size;
		ftl_cfg.prog_size = page_size;
		ftl_cfg.block_size = page_size;
		ftl_cfg.block_count = NOR_BLOCK_COUNT;//You need to setup the nor sector count
		ftl_cfg.cache_size = page_size;
		ftl_cfg.block_cycles = 100;
		ftl_cfg.lookahead_size = page_size;
	} else {
		printf("It is nand flash\r\n");
		//cfg = &nand_cfg;
		ftl_cfg.read_size = page_size;
		ftl_cfg.prog_size = page_size;
		ftl_cfg.block_size = block_size;
		ftl_cfg.block_count = NAND_FLASH_BLOCK_COUNT;
		ftl_cfg.cache_size = page_size;
		ftl_cfg.block_cycles = 100;
		ftl_cfg.lookahead_size = page_size;
	}

	// mount the filesystem
	ret = lfs_mount(lfs, &ftl_cfg);
	// reformat if we can't mount the filesystem
	// this should only happen on the first boot
	if (ret) {
		ret = lfs_format(lfs, &ftl_cfg);
		if (ret) {
			printf("lfs_format fail %d\r\n", ret);
			goto EXIT;
		}
		ret = lfs_mount(lfs, &ftl_cfg);
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
	uint32_t available = ftl_cfg.block_count * ftl_cfg.block_size - _df_nballocatedblock * ftl_cfg.block_size;
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
	if (lfs) {
		free(lfs);
	}
	if (file) {
		free(file);
	}
	if (dir) {
		free(dir);
	}
	if (info) {
		free(info);
	}
	vTaskDelete(NULL);
}

void example_littlefs(void)
{
	if (xTaskCreate(example_littlefs_thread, ((const char *)"example_littlefs_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_littlefs_thread) failed", __FUNCTION__);
	}
}
