#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "hal.h"
#include "log_service.h"
#include <platform_opts.h>
#include "freertos_service.h"
#include "osdep_service.h"
#include "usbh_cdc_ecm_hal.h"

extern int usbd_get_device_status(void);


#if(CONFIG_ETHERNET == 1 && ETHERNET_INTERFACE == USB_INTERFACE)
void AUECM(void *arg)
{
	/* extern void example_usbh_ecm(void);
	example_usbh_ecm(); */
	extern void ethernet_usb_init(void);
	ethernet_usb_init();
	printf("Enter USB Ethernet host mode\r\n");
}
#endif
int ecm_log = 0;
void AUTES(void *arg)
{
	ecm_log = 1;
	extern int hcd_measure_flag;
	hcd_measure_flag = 1;
	printf("Enter USB Ethernet log\r\n");
}

void AUVID(void *arg)
{
	printf("Enable the video streaming test from ethernet\r\n");
	extern void example_media_rtsp_ethernet(void);
	example_media_rtsp_ethernet();
}

void AUEYE(void *arg)
{
	printf("Test eye pattern\r\n");
	usbh_eye_pattern(USBH_PACKET_MODE);
}

log_item_t usb_items[] = {
#if(CONFIG_ETHERNET == 1 && ETHERNET_INTERFACE == USB_INTERFACE)
	{"UECM", AUECM,},
#endif
	{"UTES", AUTES,},
	{"UVID", AUVID,},
	{"UEYE", AUEYE,},
};

void atcmd_usb_init(void)
{
	log_service_add_table(usb_items, sizeof(usb_items) / sizeof(usb_items[0]));
}
