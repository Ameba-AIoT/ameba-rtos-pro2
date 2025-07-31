#include <stdio.h>
#include <string.h>
#include <vfs.h>
#include "log_service.h"
#include "console_xmodem.h"

#define OP_GET  	0
#define OP_PUT  	1
#define OP_LIST 	2
#define OP_MOUNT 	3
#define OP_UNMOUNT 	4
#define OP_TPUT 	0xfe
#define OP_TGET 	0xff

// ATXM=get,<filesystem filename (include path)>
// ATXM=get,sd:/xxxx.ooo
#define VFS_SD 	0
#define VFS_LFS 1
#define VFS_RAM 2
static int mounted_dev[4] = {0};

static int check_disk_mounted(char *path)
{
	if (memcmp(path, "sd", 2) == 0 && mounted_dev[VFS_SD]) {
		return 1;
	}
	if (memcmp(path, "lfs", 3) == 0 && mounted_dev[VFS_LFS]) {
		return 1;
	}
	if (memcmp(path, "ram", 3) == 0 && mounted_dev[VFS_RAM]) {
		return 1;
	}

	return 0;
}

void fATXM(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};

	if (!arg) {
		return;
	}
	argc = parse_param(arg, argv);
	if (argc != 3) {
		return;
	}

	char *cmd = argv[1];
	char *target = argv[2];

	int mode = -1;	//0: get, 1: set (not implement)
	if (strcmp(cmd, "get") == 0) {
		mode = OP_GET;
	} else if (strcmp(cmd, "put") == 0) {
		mode = OP_PUT;
	} else if (strcmp(cmd, "list") == 0) {
		mode = OP_LIST;
	} else if (strcmp(cmd, "mount") == 0) {
		mode = OP_MOUNT;
	} else if (strcmp(cmd, "umount") == 0) {
		mode = OP_UNMOUNT;
	} else if (strcmp(cmd, "tput") == 0) {
		mode = OP_TPUT;
	} else if (strcmp(cmd, "tget") == 0) {
		mode = OP_TGET;
	}

	if (mode == -1) {
		return ;
	}

	if (mode == OP_GET) {
		printf("xmodem transfer will start after 5 seconds\r\n");
		vTaskDelay(5000);
		if (!check_disk_mounted(target) || console_xmodem_tx_file(target) < 0) {
			printf("xmodem tx file %s fail\r\n", target);
		}
	} else if (mode == OP_PUT) {
		printf("xmodem transfer will start after 5 seconds\r\n");
		vTaskDelay(5000);
		if (!check_disk_mounted(target) || console_xmodem_rx_file(target) < 0) {
			printf("xmodem rx file %s fail\r\n", target);
		}
	} else if (mode == OP_LIST) {
		struct dirent **namelist;
		int n = scandir(target, &namelist, NULL, NULL);

		printf("--- list content of %s --- \r\n", target);
		if (n > 0) {
			while (n--) {
				printf("%s\n", namelist[n]->d_name);
				free(namelist[n]);
			}
			free(namelist);
		}
		printf("--- end list content   --- \r\n");
	} else if (mode == OP_MOUNT) {
		vfs_init(NULL);

		if (mounted_dev[VFS_SD] == 0 && memcmp(target, "sd", 2) == 0) {
			vfs_user_register("sd", VFS_FATFS, VFS_INF_SD);
			mounted_dev[VFS_SD] = 1;
		}
		if (mounted_dev[VFS_LFS] == 0 && memcmp(target, "lfs", 3) == 0) {
			vfs_user_register("lfs", VFS_LITTLEFS, 0);
			mounted_dev[VFS_LFS] = 1;
		}
		if (mounted_dev[VFS_RAM] == 0 && memcmp(target, "ram", 3) == 0) {
			vfs_user_register("ram", VFS_FATFS, VFS_INF_RAM);
			mounted_dev[VFS_RAM] = 1;
		}
	} else if (mode == OP_UNMOUNT) {
		if (mounted_dev[VFS_SD] && memcmp(target, "sd", 2) == 0) {
			vfs_user_unregister("sd", VFS_FATFS, VFS_INF_SD);
		}
		if (mounted_dev[VFS_LFS] && memcmp(target, "lfs", 3) == 0) {
			vfs_user_unregister("lfs", VFS_LITTLEFS, 0);
		}
		if (mounted_dev[VFS_RAM] && memcmp(target, "ram", 3) == 0) {
			vfs_user_unregister("ram", VFS_FATFS, VFS_INF_RAM);
		}

		if (!mounted_dev[VFS_SD] && !mounted_dev[VFS_LFS] && !mounted_dev[VFS_RAM]) {
			vfs_deinit(NULL);
		}
	} else if (mode == OP_TPUT) {
		unsigned char *data_buf = NULL;
		int data_len = 8192;
		data_buf = malloc(data_len);
		if (!data_buf) {
			return;
		}
		console_xmodem_rx_buffer(data_buf, data_len);

		console_dump_memory(data_buf, data_len);
		free(data_buf);
	} else if (mode == OP_TGET) {
		unsigned char *data_buf = NULL;
		int data_len = 8192;
		data_buf = malloc(data_len);
		if (!data_buf) {
			return;
		}

		strcpy((char *)data_buf, "test_test");
		for (int i = 9; i < data_len; i++) {
			data_buf[i] = (unsigned char)(i & 0xff);
		}

		console_xmodem_tx_buffer(data_buf, data_len);
		free(data_buf);
	}
}

static log_item_t xmodem_items[] = {
	{"ATXM", fATXM,},
};

void xmodem_atcmd_init(void)
{
	log_service_add_table(xmodem_items, sizeof(xmodem_items) / sizeof(xmodem_items[0]));
}

//#include "system_init.h"
//system_delay_init(xmodem_atcmd_init);