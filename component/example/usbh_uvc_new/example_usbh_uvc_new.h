#ifndef EXAMPLE_USBH_UVC_NEW_H
#define EXAMPLE_USBH_UVC_NEW_H

#include <platform_opts.h>

#define CONFIG_EXAMPLE_USBH_UVC         1
#define CONFIG_FATFS_EN                 1
#define FATFS_DISK_SD                   1

#if defined(CONFIG_EXAMPLE_USBH_UVC) && CONFIG_EXAMPLE_USBH_UVC

void example_usbh_uvc(void);

#endif

#endif // EXAMPLE_USBH_UVC_NEW_H
