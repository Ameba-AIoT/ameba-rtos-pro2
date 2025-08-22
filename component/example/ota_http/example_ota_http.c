
#include <osdep_service.h>
#include <wifi_constants.h>
#include "wifi_conf.h"
#include "lwip_netconf.h"

#if defined(CONFIG_PLATFORM_8711B)
#include "rtl8710b_ota.h"
#include <FreeRTOS.h>
#include <task.h>
#elif defined(CONFIG_PLATFORM_8195BHP)
#include <ota_8195b.h>
#elif defined(CONFIG_PLATFORM_8710C)
#include <ota_8710c.h>
#elif defined(CONFIG_PLATFORM_8735B)
#include <ota_8735b.h>
#elif defined(CONFIG_PLATFORM_8721D)
#include <platform_stdlib.h>
#include "rtl8721d_ota.h"
#include <FreeRTOS.h>
#include <task.h>
#elif defined(CONFIG_PLATFORM_AMEBALITE)
#include "ameba_ota.h"
#endif

#define PORT	80
static const char *host = "192.168.0.101";  //"m-apps.oss-cn-shenzhen.aliyuncs.com"
static const char *resource = "/bin/OTA_All.bin";     //"051103061600.bin"

#ifdef HTTP_OTA_UPDATE
void http_update_ota_task(void *param)
{
	(void)param;

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	printf("\n\r\n\r\n\r\n\r<<<<<< OTA HTTP Example >>>>>>>\n\r\n\r\n\r\n\r");

	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		printf("Wait for WIFI connection ...\n");
		vTaskDelay(1000);
	}
	int ret = -1;

	ret = http_update_ota((char *)host, PORT, (char *)resource);

	printf("\n\r[%s] Update task exit", __FUNCTION__);
	if (!ret) {
		printf("\n\r[%s] Ready to reboot", __FUNCTION__);
		ota_platform_reset();
	}
	vTaskDelete(NULL);
}


void example_ota_http(void)
{
	if (xTaskCreate(http_update_ota_task, (char const *)"http_update_ota_task", 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r[%s] Create update task failed", __FUNCTION__);
	}
}
#endif

