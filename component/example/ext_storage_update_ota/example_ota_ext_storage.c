#include <ota_8735b.h>
#include "vfs.h"

#define EXT_STORAGE   VFS_INF_EMMC //VFS_INF_SD 

#ifdef EXT_STORAGE_OTA_UPDATE
void ext_storage_update_ota_task(void *param)
{
	int ret = -1;
	vfs_init(NULL);

#if EXT_STORAGE == VFS_INF_SD
#define FILENAME	"sd:/ota_is_realtek.bin"
	printf("\n\r\n\r\n\r\n\r<<<<<<OTA from SD card example start>>>>>>>\n\r\n\r\n\r\n\r");

	// Initialize VFS and register SD card
	if (vfs_user_register("sd", VFS_FATFS, EXT_STORAGE) < 0) {
		printf("\n\r[%s] Failed to register VFS\n\r", __FUNCTION__);
		goto EXIT;
	}

#elif EXT_STORAGE == VFS_INF_EMMC
#define FILENAME	"emmc:/ota_is_realtek.bin"
	printf("\n\r\n\r\n\r\n\r<<<<<<OTA from EMMC example start>>>>>>>\n\r\n\r\n\r\n\r");
	// Initialize VFS and register EMMC
	if (vfs_user_register("emmc", VFS_FATFS, EXT_STORAGE) < 0) {
		printf("\n\r[%s] Failed to register VFS\n\r", __FUNCTION__);
		goto EXIT;
	}

#else
	printf("Not supported\n\r");
#endif

	// Start OTA update
	ret = ext_storage_update_ota((char *)FILENAME);
#if EXT_STORAGE == VFS_INF_SD
	// Unregister and deinitialize VFS
	vfs_user_unregister("sd", VFS_FATFS, VFS_INF_SD);
	vfs_deinit(NULL);
#elif EXT_STORAGE == VFS_INF_EMMC
	// Unregister and deinitialize VFS
	vfs_user_unregister("emmc", VFS_FATFS, VFS_INF_SD);
	vfs_deinit(NULL);
#endif

	printf("\n\r[%s] Update task exit", __FUNCTION__);
	if (!ret) {
		printf("\n\r[%s] Ready to reboot", __FUNCTION__);
		ota_platform_reset();
	}
	vTaskDelete(NULL);
EXIT:
	vTaskDelete(NULL);
}

void example_ota_ext_storage(void)
{
	if (xTaskCreate(ext_storage_update_ota_task, (char const *)"ext_storage_update_ota_task", 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r[%s] Create update task failed", __FUNCTION__);
	}
}
#endif