#ifndef _EXAMPLE_FATFS_H
#define _EXAMPLE_FATFS_H

/**
 * fatfs version configurations
 */
//#define FATFS_R_10C
//#define FATFS_R_13C
#define FATFS_R_14B
/** @} */

/**
 * fatfs interface configurations
 */
#ifdef CONFIG_PLATFORM_8735B
#define CONFIG_FATFS_IF_SD				1
#define CONFIG_FATFS_IF_USB				0
#define CONFIG_FATFS_IF_FLASH			1
#else
#define CONFIG_FATFS_IF_SD				0
#define CONFIG_FATFS_IF_USB				0
#define CONFIG_FATFS_IF_FLASH			1
#endif
/** @} */

void example_fatfs(void);

#endif /* _EXAMPLE_FATFS_H */

