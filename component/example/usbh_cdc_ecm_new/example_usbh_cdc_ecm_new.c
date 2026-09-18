/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Uses the selected ported stack's host CDC ECM API.
 *
 * Two classes of adaptation, both already worked out by
 * component/example/usbh_composite_cdc_acm_ecm_new (this example reuses the
 * same fixes rather than re-deriving them):
 *
 *  1. component/os/os_wrapper does not exist in this SDK generation -
 *     task/semaphore handling below calls FreeRTOS directly instead, the
 *     same way component/example/usbd_cdc_acm_new already does.
 *
 *  2. This project has no existing lwip<->USB-Ethernet netif bridge for
 *     this ported host driver.
 *     component/ethernet_mii/ethernet_usb.c is this project's existing
 *     lwip<->USB-Ethernet bridge, but it targets the *legacy*
 *     host_new/cdc_ecm driver's API (usbh_cdc_ecm_senddata()) and its build
 *     is excluded whenever usb_stack=new (see application.cmake) - the two
 *     drivers' cdc_ecm APIs cannot both be linked into one image. This
 *     example provides its own equivalent rltk_mii_send()/rltk_mii_recv()
 *     pair (same driver-callback contract ethernetif_mii_init() expects),
 *     calling usbh_cdc_ecm_send_data() instead - verbatim copy of the
 *     rltk_mii_send/rltk_mii_recv/usbh_comp_ecm_netif_up-equivalent logic
 *     already proven in example_usbh_composite_cdc_acm_ecm_new.c, just
 *     without that file's 4G-dongle AT-command dial state machine (this is
 *     a plain single-class ECM host, matching the upstream example's scope).
 *
 * CONFIG_ETHERNET must be 1 (set via usbh_cdc_ecm_new.cmake's
 * app_example_flags) so LwIP_Init() (already run once during this project's
 * normal WiFi/lwip startup) registers xnetif[ETHERNET_IDX] via
 * ethernetif_mii_init() - no separate "eth init" call is needed here.
 *
 * The usbh_config_t / FIFO depth values below are configured for the
 * amebapro2 (rtl8735b) USB OTG controller, using the values already validated
 * by example_usbh_composite_cdc_acm_ecm_new.c minus the extra ACM class slot.
 */

/* Includes ------------------------------------------------------------------ */

#include <string.h>
#include <lwip/netifapi.h>
#include "lwip_netconf.h"
#include "lwip_intf.h"      /* struct eth_drv_sg / ethernetif_mii_recv, see component/ethernet_mii/ethernet_usb.c */
#include "usbh_cdc_ecm.h"
#include "FreeRTOS.h"
#include "task.h"

extern struct netif xnetif[NET_IF_NUM];

/* Private defines -----------------------------------------------------------*/

/* Re-init the USB host stack on hot-plug (detach/attach), matching the
 * upstream example - avoids leaking the previous session's driver state. */
#define CONFIG_USBH_CDC_ECM_HOTPLUG                    1

#define ECM_INIT_THREAD_STACK_SIZE                     1280U
#define ECM_INIT_THREAD_PRIORITY                        (tskIDLE_PRIORITY + 5)
#define ECM_LINK_THREAD_STACK_SIZE                     1600U
#define ECM_LINK_THREAD_PRIORITY                        (tskIDLE_PRIORITY + 3)
#define ECM_HOTPLUG_THREAD_STACK_SIZE                  1024U
#define ECM_HOTPLUG_THREAD_PRIORITY                     (tskIDLE_PRIORITY + 6)

#define ETHERNET_IDX (NET_IF_NUM - 1)

/* Private types -------------------------------------------------------------*/

typedef enum {
	ETH_STATUS_IDLE = 0U,
	ETH_STATUS_INIT,
} eth_state_t;

/* Private function prototypes -----------------------------------------------*/

static int cdc_ecm_cb_init(void);
static int cdc_ecm_cb_deinit(void);
static int cdc_ecm_cb_attach(void);
static int cdc_ecm_cb_detach(void);
static int cdc_ecm_cb_setup(void);
static int cdc_ecm_cb_bulk_receive(u8 *buf, u32 len);
static int cdc_ecm_cb_process(usb_host_t *host, u8 msg);
static int cdc_ecm_cb_device_check(usb_host_t *host, u8 cfg_max);

/* Private variables ---------------------------------------------------------*/

static const char *const TAG = "ECM";

static u8 *usbh_ecm_rx_buf;
static u8 usbh_ecm_tx_buf[1536] __attribute__((aligned(CACHE_LINE_SIZE)));

#if CONFIG_USBH_CDC_ECM_HOTPLUG
static u8 cdc_ecm_detach_pending;
static SemaphoreHandle_t cdc_ecm_detach_sema;
#endif

static const usbh_config_t usbh_ecm_cfg = {
	.speed = USB_SPEED_HIGH,
	/* 4U matches component/example/usbh_composite_cdc_acm_ecm_new's validated value. */
	.isr_priority = 4U,
	.main_task_stack_size = 1280U,
	.main_task_priority = tskIDLE_PRIORITY + 5,
	.tick_source = USBH_SOF_TICK,
	.class_num = 1U,   /* CDC-ECM only */
	/* DFIFO total 2232 DWORD, resv 8 DWORD for DMA addr. */
	.rx_fifo_depth = 1712,
	.nptx_fifo_depth = 256,
	.ptx_fifo_depth = 256,
};

static const usbh_cdc_ecm_state_cb_t cdc_ecm_usb_cb = {
	.init = cdc_ecm_cb_init,
	.deinit = cdc_ecm_cb_deinit,
	.attach = cdc_ecm_cb_attach,
	.detach = cdc_ecm_cb_detach,
	.setup = cdc_ecm_cb_setup,
	.bulk_received = cdc_ecm_cb_bulk_receive,
};

static const usbh_user_cb_t usbh_ecm_usr_cb = {
	.process = cdc_ecm_cb_process,
	.validate = cdc_ecm_cb_device_check,
};

static const usbh_cdc_ecm_priv_data_t ecm_priv = {
	NULL,
	NULL,
	0,
};

/* Private functions ---------------------------------------------------------*/

static int cdc_ecm_cb_device_check(usb_host_t *host, u8 cfg_max)
{
	UNUSED(cfg_max);
	return usbh_cdc_ecm_check_config_desc(host);
}

static int cdc_ecm_cb_init(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "INIT\n");
	return HAL_OK;
}

static int cdc_ecm_cb_deinit(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "DEINIT\n");
	return HAL_OK;
}

static int cdc_ecm_cb_attach(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "ATTACH\n");
	return HAL_OK;
}

/**
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static int cdc_ecm_cb_detach(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "DETACH\n");
#if CONFIG_USBH_CDC_ECM_HOTPLUG
	BaseType_t woken = pdFALSE;

	cdc_ecm_detach_pending = 1;
	xSemaphoreGiveFromISR(cdc_ecm_detach_sema, &woken);
	portYIELD_FROM_ISR(woken);
#endif
	return HAL_OK;
}

static int cdc_ecm_cb_setup(void)
{
	RTK_LOGS(TAG, RTK_LOG_INFO, "SETUP\n");
	return HAL_OK;
}

/**
  * @brief  CDC ECM bulk receive callback - a complete Ethernet frame per
  *         callback (the ported cdc_ecm driver delivers whole frames, unlike
  *         the legacy driver's chunked RX_BUFFER reassembly).
  * @note   Called in ISR context - no malloc/blocking calls.
  */
static int cdc_ecm_cb_bulk_receive(u8 *buf, u32 len)
{
	if (len > 0) {
		usbh_ecm_rx_buf = buf;
		ethernetif_mii_recv(&xnetif[ETHERNET_IDX], (int)len);
	}
	return HAL_OK;
}

static int cdc_ecm_cb_process(usb_host_t *host, u8 msg)
{
	UNUSED(host);
	switch (msg) {
	case USBH_MSG_USER_SET_CONFIG:
		usbh_cdc_ecm_choose_config(host);
		break;
	default:
		break;
	}
	return HAL_OK;
}

/*
 * lwip<->USB-Ethernet bridge - the fixed-name driver-callback contract
 * ethernetif_mii_init() (component/lwip/.../port/realtek/freertos/
 * ethernetif.c, registered for xnetif[ETHERNET_IDX] by LwIP_Init() whenever
 * CONFIG_ETHERNET==1) expects: netif->linkoutput calls rltk_mii_send() with
 * a scatter-gather list built from the outgoing pbuf chain, and
 * ethernetif_mii_recv() (called from cdc_ecm_cb_bulk_receive() above) calls
 * rltk_mii_recv() to fill the inbound pbuf chain from usbh_ecm_rx_buf.
 */

s8 rltk_mii_send(struct eth_drv_sg *sg_list, int sg_len, int total_len)
{
	struct eth_drv_sg *last_sg;
	u8 *pdata = usbh_ecm_tx_buf;
	int ret;

	if ((total_len < 0) || ((u32)total_len > sizeof(usbh_ecm_tx_buf))) {
		return -1;
	}
	for (last_sg = &sg_list[sg_len]; sg_list < last_sg; ++sg_list) {
		memcpy(pdata, (void *)(sg_list->buf), sg_list->len);
		pdata += sg_list->len;
	}
	ret = usbh_cdc_ecm_send_data(usbh_ecm_tx_buf, (u32)total_len, 1);
	return (ret == HAL_OK) ? 0 : -1;
}

void rltk_mii_recv(struct eth_drv_sg *sg_list, int sg_len)
{
	struct eth_drv_sg *last_sg;
	u8 *pbuf = usbh_ecm_rx_buf;

	for (last_sg = &sg_list[sg_len]; sg_list < last_sg; ++sg_list) {
		if (sg_list->buf != 0) {
			memcpy((void *)(sg_list->buf), pbuf, sg_list->len);
			pbuf += sg_list->len;
		}
	}
}

int usb_ethernet_transmit(u8 *buf, u32 len, u8 block)
{
	return usbh_cdc_ecm_send_data(buf, len, block);
}

static int cdc_ecm_do_init(void)
{
	int ret;

	ret = usbh_init(&usbh_ecm_cfg, &usbh_ecm_usr_cb);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Init USBH fail\n");
		return 0;
	}

	ret = usbh_cdc_ecm_init(&cdc_ecm_usb_cb, &ecm_priv);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Init CDC ECM fail\n");
		usbh_deinit();
		return 0;
	}

	usbh_start();

	while (!usbh_cdc_ecm_usb_is_ready()) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	/* USB enumerated: release the SOF data-transfer gate so bulk/intr
	 * scheduling can begin (see usbh_cdc_ecm_sof()). Reused on both the
	 * initial bring-up and every hot-plug re-init (this function is also
	 * called from the hotplug thread), re-arming the gate that detach()
	 * cleared. */
	usbh_cdc_ecm_prepare_done();

	return 1;
}

static void example_usbh_ecm_link_change_thread(void *param)
{
	u8 *mac;
	u32 dhcp_status;
	u32 link_down_log_cnt = 0;
	u8 link_is_up;
	eth_state_t ethernet_state = ETH_STATUS_IDLE;

	UNUSED(param);
	RTK_LOGS(TAG, RTK_LOG_INFO, "Enter link status task!\n");

	if (cdc_ecm_do_init() == 0) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "USB init fail\n");
		vTaskDelete(NULL);
	}

	for (;;) {
#if CONFIG_USBH_CDC_ECM_HOTPLUG
		/* Consume detach events first: the detach callback only flags this
		 * bit, so even if the dongle has already been re-plugged by the time
		 * we poll (link_is_up reads back as 1), we still tear down the eth
		 * default route here. */
		if (cdc_ecm_detach_pending) {
			cdc_ecm_detach_pending = 0;
			if (ethernet_state >= ETH_STATUS_INIT) {
				ethernet_state = ETH_STATUS_IDLE;
				netif_set_default(&xnetif[0]);
				RTK_LOGS(TAG, RTK_LOG_INFO, "Switch to unlink (detach event)\n");
			}
		}
#endif

		link_is_up = usbh_cdc_ecm_get_link_status();

		if (1 == link_is_up && (ethernet_state < ETH_STATUS_INIT)) {  /* unlink -> link */
			mac = (u8 *)usbh_cdc_ecm_process_mac_str();
			if (mac == NULL) {
				vTaskDelay(pdMS_TO_TICKS(1000));
			} else {
				RTK_LOGS(TAG, RTK_LOG_INFO, "Do DHCP\n");
				ethernet_state = ETH_STATUS_INIT;
				memcpy(xnetif[ETHERNET_IDX].hwaddr, mac, 6);
				RTK_LOGS(TAG, RTK_LOG_INFO, "MAC[%02x %02x %02x %02x %02x %02x]\n",
						 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
				netif_set_link_up(&xnetif[ETHERNET_IDX]);

				dhcp_status = LwIP_DHCP(ETHERNET_IDX, DHCP_START);
				if (DHCP_ADDRESS_ASSIGNED == dhcp_status) {
					netifapi_netif_set_default(&xnetif[ETHERNET_IDX]);
					RTK_LOGS(TAG, RTK_LOG_INFO, "Switch to link\n");
				} else {
					RTK_LOGS(TAG, RTK_LOG_INFO, "DHCP Fail\n");
				}
			}
		} else if (0 == link_is_up && (ethernet_state >= ETH_STATUS_INIT)) {  /* link -> unlink */
			ethernet_state = ETH_STATUS_IDLE;
			netif_set_default(&xnetif[0]);
			RTK_LOGS(TAG, RTK_LOG_INFO, "Switch to unlink\n");
		} else {
			if ((link_is_up == 0) && (++link_down_log_cnt % 10 == 1)) {
				RTK_LOGS(TAG, RTK_LOG_INFO, "ECM link is down (%u)\n", link_down_log_cnt);
			}
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}
}

#if CONFIG_USBH_CDC_ECM_HOTPLUG
static void example_usbh_ecm_hotplug_thread(void *param)
{
	UNUSED(param);

	for (;;) {
		xSemaphoreTake(cdc_ecm_detach_sema, portMAX_DELAY);
		RTK_LOGS(TAG, RTK_LOG_INFO, "Hot plug\n");

		usbh_stop();
		usbh_cdc_ecm_deinit();
		usbh_deinit();

		vTaskDelay(pdMS_TO_TICKS(10));

		/* Re-run the full init sequence (usbh_init + ecm_init + wait-for-ready).
		 * Reuses cdc_ecm_do_init() so the re-attach path stays in lock-step
		 * with the initial bring-up, including the usb_is_ready() wait. */
		if (cdc_ecm_do_init() == 0) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "Re-init after hot plug fail\n");
			break;
		}
	}
}
#endif

/* Exported functions --------------------------------------------------------*/

void example_usbh_cdc_ecm_new(void)
{
	TaskHandle_t link_task;
#if CONFIG_USBH_CDC_ECM_HOTPLUG
	TaskHandle_t hotplug_task;
#endif

	RTK_LOGS(TAG, RTK_LOG_INFO, "USBH ECM demo start\n");

#if CONFIG_USBH_CDC_ECM_HOTPLUG
	cdc_ecm_detach_sema = xSemaphoreCreateBinary();
	if (xTaskCreate(example_usbh_ecm_hotplug_thread, "usbh_ecm_hotplug", ECM_HOTPLUG_THREAD_STACK_SIZE,
					NULL, ECM_HOTPLUG_THREAD_PRIORITY, &hotplug_task) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create hotplug thread fail\n");
	}
#endif

	if (xTaskCreate(example_usbh_ecm_link_change_thread, "usbh_ecm_link", ECM_LINK_THREAD_STACK_SIZE,
					NULL, ECM_LINK_THREAD_PRIORITY, &link_task) != pdPASS) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "Create link status thread fail\n");
	}
}
