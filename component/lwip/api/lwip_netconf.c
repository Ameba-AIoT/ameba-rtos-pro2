/* Includes ------------------------------------------------------------------*/
#include "lwip_netconf.h"
#include "main.h"
#if CONFIG_WLAN
#include "wifi_ind.h"
#endif

#include <platform_stdlib.h>
#include "osdep_service.h"

#if defined(CONFIG_FAST_DHCP) && CONFIG_FAST_DHCP
#include "wifi_fast_connect.h"
#endif

#include "lwip/priv/tcpip_priv.h"

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

#if LWIP_VERSION_MAJOR >= 2 && LWIP_VERSION_MINOR >= 1
#if LWIP_IPV6
#include "lwip/dhcp6.h"
#include "lwip/prot/dhcp6.h"
#endif
#endif

/*Initial IP ADDRESS*/
#ifndef IP_ADDR0
#define IP_ADDR0   0
#define IP_ADDR1   0
#define IP_ADDR2   0
#define IP_ADDR3   0
#endif

/*Static IP ADDRESS*/
#ifndef STATIC_IP_ADDR0
#define STATIC_IP_ADDR0   192
#define STATIC_IP_ADDR1   168
#define STATIC_IP_ADDR2   1
#define STATIC_IP_ADDR3   80
#endif

/*NETMASK*/
#ifndef NETMASK_ADDR0
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0
#endif

/*Gateway Address*/
#ifndef GW_ADDR0
#define GW_ADDR0   0
#define GW_ADDR1   0
#define GW_ADDR2   0
#define GW_ADDR3   0
#endif

/*Static IP ADDRESS*/
#ifndef AP_IP_ADDR0
#define AP_IP_ADDR0   192
#define AP_IP_ADDR1   168
#define AP_IP_ADDR2   43
#define AP_IP_ADDR3   1
#endif

/*NETMASK*/
#ifndef AP_NETMASK_ADDR0
#define AP_NETMASK_ADDR0   255
#define AP_NETMASK_ADDR1   255
#define AP_NETMASK_ADDR2   255
#define AP_NETMASK_ADDR3   0
#endif

/*Gateway Address*/
#ifndef AP_GW_ADDR0
#define AP_GW_ADDR0   192
#define AP_GW_ADDR1   168
#define AP_GW_ADDR2   43
#define AP_GW_ADDR3   1
#endif

/*Static IP ADDRESS FOR ETHERNET*/
#ifndef ETH_IP_ADDR0
#define ETH_IP_ADDR0 192
#define ETH_IP_ADDR1 168
#define ETH_IP_ADDR2 0
#define ETH_IP_ADDR3 80
#endif

/*NETMASK FOR ETHERNET*/
#ifndef ETH_NETMASK_ADDR0
#define ETH_NETMASK_ADDR0 255
#define ETH_NETMASK_ADDR1 255
#define ETH_NETMASK_ADDR2 255
#define ETH_NETMASK_ADDR3 0
#endif

/*Gateway address for ethernet*/
#ifndef ETH_GW_ADDR0
#define ETH_GW_ADDR0 192
#define ETH_GW_ADDR1 168
#define ETH_GW_ADDR2 0
#define ETH_GW_ADDR3 1
#endif

/* Private define ------------------------------------------------------------*/
#define MAX_DHCP_TRIES 5
#if LWIP_VERSION_MAJOR >= 2 && LWIP_VERSION_MINOR >= 1
#if LWIP_IPV6
#define MAX_DHCP6_TRIES 5
#endif
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef CONFIG_AS_INIC_AP
#include "inic_ipc.h"
extern rtw_mode_t wifi_mode;
#endif

struct netif xnetif[NET_IF_NUM]; /* network interface structure */
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */

int lwip_init_done = 0;

/*
The lwip_compatibilty_enabled is used to configure the lwip settings, each bit controls one aspect.
bit 0: (0(default): default enable LWIP set_ip function, 1: disable LWIP set_ip function)
*/
int lwip_compatibilty_enabled;

#define lwip_compatibility_is_enable(a,b) ((a & b) ? 1 : 0)

void LwIP_set_compatibilty_enable(int compatibilty_enabled)
{
	lwip_compatibilty_enabled = compatibilty_enabled;
}

void LwIP_Init(void)
{
	struct ip_addr ipaddr;
	struct ip_addr netmask;
	struct ip_addr gw;
	int8_t idx = 0;

	/* Create tcp_ip stack thread */
	tcpip_init(NULL, NULL);

#ifdef LWIP_HOOK_TCP_ISN
	tcp_isn_init();
#endif

	/* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
	        struct ip_addr *netmask, struct ip_addr *gw,
	        void *state, err_t (* init)(struct netif *netif),
	        err_t (* input)(struct pbuf *p, struct netif *netif))

	Adds your network interface to the netif_list. Allocate a struct
	netif and pass a pointer to this structure as the first argument.
	Give pointers to cleared ip_addr structures when using DHCP,
	or fill them with sane numbers otherwise. The state pointer may be NULL.

	The init function pointer must point to a initialization function for
	your ethernet netif interface. The following code illustrates it's use.*/
	//printf("NET_IF_NUM:%d\n\r",NET_IF_NUM);
	for (idx = 0; idx < NET_IF_NUM; idx++) {
#if LWIP_VERSION_MAJOR >= 2
		if (idx == 0) {
			IP4_ADDR(ip_2_ip4(&ipaddr), IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
			IP4_ADDR(ip_2_ip4(&netmask), NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
			IP4_ADDR(ip_2_ip4(&gw), GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
		} else {
			IP4_ADDR(ip_2_ip4(&ipaddr), AP_IP_ADDR0, AP_IP_ADDR1, AP_IP_ADDR2, AP_IP_ADDR3);
			IP4_ADDR(ip_2_ip4(&netmask), AP_NETMASK_ADDR0, AP_NETMASK_ADDR1, AP_NETMASK_ADDR2, AP_NETMASK_ADDR3);
			IP4_ADDR(ip_2_ip4(&gw), AP_GW_ADDR0, AP_GW_ADDR1, AP_GW_ADDR2, AP_GW_ADDR3);
		}
#else
		if (idx == 0) {
			IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
			IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
			IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
		} else {
			IP4_ADDR(&ipaddr, AP_IP_ADDR0, AP_IP_ADDR1, AP_IP_ADDR2, AP_IP_ADDR3);
			IP4_ADDR(&netmask, AP_NETMASK_ADDR0, AP_NETMASK_ADDR1, AP_NETMASK_ADDR2, AP_NETMASK_ADDR3);
			IP4_ADDR(&gw, AP_GW_ADDR0, AP_GW_ADDR1, AP_GW_ADDR2, AP_GW_ADDR3);
		}
#endif
#if CONFIG_ETHERNET
		if (idx == NET_IF_NUM - 1) {
#if LWIP_VERSION_MAJOR >= 2
			IP4_ADDR(ip_2_ip4(&ipaddr), ETH_IP_ADDR0, ETH_IP_ADDR1, ETH_IP_ADDR2, ETH_IP_ADDR3);
			IP4_ADDR(ip_2_ip4(&netmask), ETH_NETMASK_ADDR0, ETH_NETMASK_ADDR1, ETH_NETMASK_ADDR2, ETH_NETMASK_ADDR3);
			IP4_ADDR(ip_2_ip4(&gw), ETH_GW_ADDR0, ETH_GW_ADDR1, ETH_GW_ADDR2, ETH_GW_ADDR3);
#else
			IP4_ADDR(&ipaddr, ETH_IP_ADDR0, ETH_IP_ADDR1, ETH_IP_ADDR2, ETH_IP_ADDR3);
			IP4_ADDR(&netmask, ETH_NETMASK_ADDR0, ETH_NETMASK_ADDR1, ETH_NETMASK_ADDR2, ETH_NETMASK_ADDR3);
			IP4_ADDR(&gw, ETH_GW_ADDR0, ETH_GW_ADDR1, ETH_GW_ADDR2, ETH_GW_ADDR3);
#endif
		}
#endif
		xnetif[idx].name[0] = 'r';
		xnetif[idx].name[1] = '0' + idx;
#if LWIP_VERSION_MAJOR >= 2
#if CONFIG_ETHERNET
		if (idx == NET_IF_NUM - 1) {
			netif_add(&xnetif[idx], ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw), NULL, &ethernetif_mii_init, &tcpip_input);
		} else {
			netif_add(&xnetif[idx], ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw), NULL, &ethernetif_init, &tcpip_input);
		}
#else
		netif_add(&xnetif[idx], ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw), NULL, &ethernetif_init, &tcpip_input);
#endif
#else

#if CONFIG_ETHERNET
		if (idx == NET_IF_NUM - 1) {
			netif_add(&xnetif[idx], &ipaddr, &netmask, &gw, NULL, &ethernetif_mii_init, &tcpip_input);
		} else {
			netif_add(&xnetif[idx], &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
		}
#else
		netif_add(&xnetif[idx], &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
#endif
#endif
		printf("interface %d is initialized\n", idx);

	}

	/*  Registers the default network interface. */
	netif_set_default(&xnetif[0]);

	/*move these operations to wifi_on/wifi_off*/
#if 0
	/*  When the netif is fully configured this function must be called.*/
	for (idx = 0; idx < NET_IF_NUM; idx++) {
		netif_set_up(&xnetif[idx]);
	}
#endif

	lwip_init_done = 1;
}

#if LWIP_VERSION_MAJOR >= 2 && LWIP_VERSION_MINOR >= 1
#if LWIP_IPV6 && (LWIP_IPV6_DHCP6_STATEFUL||LWIP_IPV6_DHCP6_STATELESS)
extern err_t dhcp6_enable(struct netif *netif);
#endif
#endif

#if defined(CONFIG_FAST_DHCP) && CONFIG_FAST_DHCP
extern uint32_t offer_ip;
extern uint32_t server_ip;

#endif
/**
  * @brief  LwIP_DHCP_Process_Handle
  * @param  None
  * @retval None
  */
uint8_t LwIP_DHCP(uint8_t idx, uint8_t dhcp_state)
{
	struct ip_addr ipaddr;
	struct ip_addr netmask;
	struct ip_addr gw;
	uint32_t IPaddress;
	uint8_t iptab[4];
	uint8_t DHCP_state;
	struct netif *pnetif = NULL;
	struct dhcp *dhcp = NULL;

	DHCP_state = dhcp_state;

#if !CONFIG_ETHERNET
	if (idx > 1) {
		idx = 1;
	}
#endif

	extern struct static_ip_config user_static_ip;
	if (user_static_ip.use_static_ip) {
		LwIP_SetIP(0, user_static_ip.addr, user_static_ip.netmask, user_static_ip.gw);
		iptab[3] = (uint8_t)(user_static_ip.addr >> 24);
		iptab[2] = (uint8_t)(user_static_ip.addr >> 16);
		iptab[1] = (uint8_t)(user_static_ip.addr >> 8);
		iptab[0] = (uint8_t)(user_static_ip.addr);
		printf("\n\rSet Interface %d static IP : %d.%d.%d.%d\n", idx, iptab[3], iptab[2], iptab[1], iptab[0]);
		return 0;
	}

	pnetif = &xnetif[idx];
	if (DHCP_state == 0) {
#if LWIP_VERSION_MAJOR >= 2
		ip_addr_set_zero(&pnetif->ip_addr);
		ip_addr_set_zero(&pnetif->netmask);
		ip_addr_set_zero(&pnetif->gw);
#else
		pnetif->ip_addr.addr = 0;
		pnetif->netmask.addr = 0;
		pnetif->gw.addr = 0;
#endif
	}

#if LWIP_VERSION_MAJOR >= 2
	dhcp = ((struct dhcp *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP));
	if (!netif_is_up(pnetif)) { // netif should be set up before doing dhcp request (in lwip v2.0.0)
		netif_set_up(pnetif);
	}
#else
	dhcp = pnetif->dhcp;
#endif

	for (;;) {
		//printf("\n\r ========DHCP_state:%d============\n\r",DHCP_state);
		switch (DHCP_state) {
		case DHCP_START: {
			/*acqurie wakelock to guarantee dhcp*/
#ifndef CONFIG_AS_INIC_AP
			rtw_wakelock_timeout(4 * 1000);
#endif

#if defined(CONFIG_FAST_DHCP) && CONFIG_FAST_DHCP
			if (check_is_the_same_ap()) {
				if ((offer_ip != 0 && offer_ip != 0xFFFFFFFF) || (dhcp != NULL)) {
					if (dhcp == NULL) {
						dhcp = (struct dhcp *)mem_malloc(sizeof(struct dhcp));
						if (dhcp == NULL) {
							printf("dhcp_start(): could not allocate dhcp\n");
							return -1;
						}
					}
					memset(dhcp, 0, sizeof(struct dhcp));
					dhcp->offered_ip_addr.addr = (u32_t)offer_ip;

#if LWIP_VERSION_MAJOR >= 2
					ip_addr_set_ip4_u32(&dhcp->server_ip_addr, (u32_t)server_ip);
#else
					dhcp->server_ip_addr.addr = (u32_t)server_ip;
#endif

#if LWIP_VERSION_MAJOR >= 2
					netif_set_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP, dhcp);
#else
					pnetif->dhcp = dhcp;
#endif
				}
			} else {
				if (dhcp != NULL) {
					memset(dhcp, 0, sizeof(struct dhcp));
				}
			}

#endif
			dhcp_start(pnetif);
#if LWIP_VERSION_MAJOR >= 2
			dhcp = ((struct dhcp *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP));
#else
			dhcp = pnetif->dhcp;
#endif
			IPaddress = 0;
			DHCP_state = DHCP_WAIT_ADDRESS;
		}
		break;

		case DHCP_WAIT_ADDRESS: {
			/* If DHCP stopped by wifi_disconn_hdl*/
#if LWIP_VERSION_MAJOR >= 2
#include "lwip/prot/dhcp.h"
			if ((dhcp_state_enum_t)dhcp->state == DHCP_STATE_OFF) {
				IP4_ADDR(ip_2_ip4(&ipaddr), IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
				IP4_ADDR(ip_2_ip4(&netmask), NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
				IP4_ADDR(ip_2_ip4(&gw), GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
				netif_set_addr(pnetif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
				printf("\n\rLwIP_DHCP: dhcp stop.");
				return DHCP_STOP;
			}
#else
			if (dhcp->state == DHCP_OFF) {
				IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
				IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
				IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
				netif_set_addr(pnetif, &ipaddr, &netmask, &gw);
				printf("\n\rLwIP_DHCP: dhcp stop.");
				return DHCP_STOP;
			}
#endif

			/* Read the new IP address */
#if LWIP_VERSION_MAJOR >= 2
			IPaddress = ip_addr_get_ip4_u32(netif_ip_addr4(pnetif));
#else
			IPaddress = pnetif->ip_addr.addr;
#endif

			if (IPaddress != 0) {
#if LWIP_RANDOMIZE_INITIAL_LOCAL_PORTS
				tcp_randomize_local_port();
				udp_randomize_local_port();
#endif
				DHCP_state = DHCP_ADDRESS_ASSIGNED;

				/* Stop DHCP */
				// dhcp_stop(pnetif);  /* can not stop, need to renew, Robbie*/

				iptab[0] = (uint8_t)(IPaddress >> 24);
				iptab[1] = (uint8_t)(IPaddress >> 16);
				iptab[2] = (uint8_t)(IPaddress >> 8);
				iptab[3] = (uint8_t)(IPaddress);
				printf("\n\rInterface %d IP address : %d.%d.%d.%d\n", idx, iptab[3], iptab[2], iptab[1], iptab[0]);

				/* Get dhcp server mac address */
				extern void etharp_issue_dhcpserver_arp_task(void);
				etharp_issue_dhcpserver_arp_task();

#if defined(CONFIG_FAST_DHCP) && CONFIG_FAST_DHCP
#if LWIP_VERSION_MAJOR >= 2
				dhcp = ((struct dhcp *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP));
#else
				dhcp = pnetif->dhcp;
#endif

				if (p_store_fast_connect_info) {
#if LWIP_VERSION_MAJOR >= 2
					p_store_fast_connect_info((uint32_t)dhcp->offered_ip_addr.addr, (uint32_t)ip_addr_get_ip4_u32(&dhcp->server_ip_addr));
#else
					p_store_fast_connect_info((uint32_t)dhcp->offered_ip_addr.addr, (uint32_t)dhcp->server_ip_addr.addr);
#endif
				}
#endif

				RTW_API_INFO("[%s] dhcp offered_t0_lease: %d", __FUNCTION__, dhcp->offered_t0_lease);
				return DHCP_ADDRESS_ASSIGNED;
			} else {
				/* DHCP timeout */
				if (dhcp->tries > MAX_DHCP_TRIES) {
					DHCP_state = DHCP_TIMEOUT;
					/* Stop DHCP */
					dhcp_stop(pnetif);

					/* Static address used */

#if LWIP_VERSION_MAJOR >= 2
					IP4_ADDR(ip_2_ip4(&ipaddr), STATIC_IP_ADDR0, STATIC_IP_ADDR1, STATIC_IP_ADDR2, STATIC_IP_ADDR3);
					IP4_ADDR(ip_2_ip4(&netmask), NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
					IP4_ADDR(ip_2_ip4(&gw), GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
					netif_set_addr(pnetif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
#else
					IP4_ADDR(&ipaddr, STATIC_IP_ADDR0, STATIC_IP_ADDR1, STATIC_IP_ADDR2, STATIC_IP_ADDR3);
					IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
					IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
					netif_set_addr(pnetif, &ipaddr, &netmask, &gw);
#endif
					iptab[0] = STATIC_IP_ADDR3;
					iptab[1] = STATIC_IP_ADDR2;
					iptab[2] = STATIC_IP_ADDR1;
					iptab[3] = STATIC_IP_ADDR0;
					printf("\n\rInterface %d DHCP timeout", idx);
					printf("\n\rStatic IP address : %d.%d.%d.%d", iptab[3], iptab[2], iptab[1], iptab[0]);

#if defined(CONFIG_FAST_DHCP) && CONFIG_FAST_DHCP
					if (p_store_fast_connect_info) {
#if LWIP_VERSION_MAJOR >= 2
						p_store_fast_connect_info((uint32_t)dhcp->offered_ip_addr.addr, (uint32_t)ip_addr_get_ip4_u32(&dhcp->server_ip_addr));
#else
						p_store_fast_connect_info((uint32_t)dhcp->offered_ip_addr.addr, (uint32_t)dhcp->server_ip_addr.addr);
#endif
					}
#endif

#if CONFIG_ETHERNET
					if (idx == NET_IF_NUM - 1) { // This is the ethernet interface, set it up for static ip address
						netif_set_up(pnetif);
					}
#endif
					return DHCP_TIMEOUT;
				}
			}
		}
		break;
		case DHCP_RELEASE_IP:
			printf("\n\rLwIP_DHCP: Release ip");
#if LWIP_VERSION_MAJOR >= 2
			dhcp_release(pnetif);
#else
			if (dhcp && dhcp->state != DHCP_OFF) {
				dhcp_release_unicast(pnetif);
			}
#endif
			return DHCP_RELEASE_IP;
		case DHCP_STOP:
			printf("\n\rLwIP_DHCP: dhcp stop.");
			dhcp_stop(pnetif);
			return DHCP_STOP;
		default:
			break;
		}
		/* wait 250 ms */
		vTaskDelay(10);
	}
}

void LwIP_ReleaseIP(uint8_t idx)
{
	struct ip_addr ipaddr;
	struct ip_addr netmask;
	struct ip_addr gw;
	struct netif *pnetif = &xnetif[idx];
#if LWIP_VERSION_MAJOR >= 2
	IP4_ADDR(ip_2_ip4(&ipaddr), 0, 0, 0, 0);
	IP4_ADDR(ip_2_ip4(&netmask), 255, 255, 255, 0);
	IP4_ADDR(ip_2_ip4(&gw), 0, 0, 0, 0);
	netifapi_netif_set_addr(pnetif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
#else
	IP4_ADDR(&ipaddr, 0, 0, 0, 0);
	IP4_ADDR(&netmask, 255, 255, 255, 0);
	IP4_ADDR(&gw, 0, 0, 0, 0);
	netifapi_netif_set_addr(pnetif, &ipaddr, &netmask, &gw);
#endif

#if LWIP_VERSION_MAJOR >= 2 && LWIP_VERSION_MINOR >= 1
#if LWIP_IPV6
	IP6_ADDR(ip_2_ip6(&ipaddr), 0, 0, 0, 0);
	for (int idx = 1; idx < LWIP_IPV6_NUM_ADDRESSES; idx++) {
		netif_ip6_addr_set_state(pnetif, idx, IP6_ADDR_INVALID);
		netif_ip6_addr_set(pnetif, idx, ip_2_ip6(&ipaddr));
	}
#endif
#endif
}

struct netconf_api_call_data {
	struct tcpip_api_call_data call;
	struct netif        *netif;
};

static void netconf_do_dhcp_stop(struct netconf_api_call_data *p)
{
	struct netconf_api_call_data *call = (struct netconf_api_call_data *)p;

	dhcp_stop(call->netif);
}

static int netconf_api_dhcp_stop(struct netif *netif)
{
	struct netconf_api_call_data call;

	call.netif = netif;

	return tcpip_api_call(netconf_do_dhcp_stop, (struct tcpip_api_call_data *)&call);
}

void LwIP_DHCP_stop(uint8_t idx)
{
	struct ip_addr ipaddr;
	struct ip_addr netmask;
	struct ip_addr gw;
	struct netif *pnetif = &xnetif[idx];
	netconf_api_dhcp_stop(pnetif);

#if LWIP_VERSION_MAJOR >= 2
	IP4_ADDR(ip_2_ip4(&ipaddr), IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
	IP4_ADDR(ip_2_ip4(&netmask), NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
	IP4_ADDR(ip_2_ip4(&gw), GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
	netifapi_netif_set_addr(pnetif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
#else
	IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
	IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
	IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

	netifapi_netif_set_addr(pnetif, &ipaddr, &netmask, &gw);
#endif
}

s8_t LwIP_etharp_find_addr(uint8_t idx, const ip4_addr_t *ipaddr,
						   struct eth_addr **eth_ret, const ip4_addr_t **ip_ret)
{
	struct netif *pnetif = &xnetif[idx];
	return etharp_find_addr(pnetif, ipaddr, eth_ret, ip_ret);
}

void LwIP_etharp_request(uint8_t idx, const ip4_addr_t *ipaddr)
{
	struct netif *pnetif = &xnetif[idx];
	etharp_request(pnetif, ipaddr);
}

#if !defined(CONFIG_MBED_ENABLED)
int netif_get_idx(struct netif *pnetif)
{
#if (CONFIG_LWIP_LAYER == 1)
	int idx = pnetif - xnetif;

	switch (idx) {
	case 0:
		return 0;
	case 1:
		return 1;
	default:
		return -1;
	}
#else
	UNUSED(pnetif);
	return -1;
#endif
}
#endif

void LwIP_netif_set_up(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	netif_set_up(pnetif);
}

void LwIP_netif_set_down(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	netif_set_down(pnetif);
}

void LwIP_netif_set_link_up(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	netif_set_link_up(pnetif);
}

void LwIP_netif_set_link_down(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	netif_set_link_down(pnetif);
}

uint8_t *LwIP_GetMAC(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	return (uint8_t *)(pnetif->hwaddr);
}

uint8_t *LwIP_GetIP(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	return (uint8_t *) & (pnetif->ip_addr);
}

#if LWIP_VERSION_MAJOR >= 2 && LWIP_VERSION_MINOR >= 1
#if LWIP_IPV6
#if LWIP_IPV6_DHCP6
uint8_t LwIP_DHCP6(uint8_t idx, uint8_t dhcp6_state)
{
	uint8_t *ipv6_global;
	uint8_t DHCP6_state;
	struct netif *pnetif = NULL;
	struct dhcp6 *dhcp6 = NULL;
	err_t err;

	DHCP6_state = dhcp6_state;

#if !CONFIG_ETHERNET
	if (idx > 1) {
		idx = 1;
	}
#endif

	pnetif = &xnetif[idx];
	if (DHCP6_state == 0) {
		for (int free_idx = 1; free_idx < LWIP_IPV6_NUM_ADDRESSES; free_idx++) {
			ip_addr_set_zero(&pnetif->ip6_addr[free_idx]);
			netif_ip6_addr_set_state(pnetif, free_idx, IP6_ADDR_INVALID);
		}
	}

	dhcp6 = ((struct dhcp6 *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP6));
	if (!netif_is_up(pnetif)) { // netif should be set up before doing dhcp request (in lwip v2.0.0)
		netif_set_up(pnetif);
	}

	for (;;) {
		//printf("\n\r ========DHCP6_state:%d============\n\r",DHCP6_state);
		switch (DHCP6_state) {

		case DHCP6_START: {
			err = dhcp6_enable(pnetif);
			if (err != ERR_OK) {
				printf("error in dhcp6_enable\r\n");
			}

			dhcp6 = ((struct dhcp6 *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP6));

			ipv6_global = 0;
			DHCP6_state = DHCP6_WAIT_ADDRESS;
		}
		break;
		case DHCP6_WAIT_ADDRESS: {

			/* Read the new IPv6 address */
			ipv6_global = LwIP_GetIPv6_global(pnetif);

			if (*ipv6_global != 0) {

				DHCP6_state = DHCP6_ADDRESS_ASSIGNED;

				printf("\n\rInterface %d IPv6 address : %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x",
					   idx, ipv6_global[0], ipv6_global[1],  ipv6_global[2],  ipv6_global[3],  ipv6_global[4],  ipv6_global[5],
					   ipv6_global[6], ipv6_global[7], ipv6_global[8], ipv6_global[9], ipv6_global[10], ipv6_global[11],
					   ipv6_global[12], ipv6_global[13], ipv6_global[14], ipv6_global[15]);

				/*Todo: error_flag for DHCPv6*/

				return DHCP6_ADDRESS_ASSIGNED;
			}

			else {

				/* DHCP timeout */
				if (dhcp6->tries > MAX_DHCP6_TRIES || pnetif->rs_timeout) {

					DHCP6_state = DHCP6_TIMEOUT;
					/* Stop DHCP */
					dhcp6_stop(pnetif);

					/*Todo: error_flag for DHCPv6*/

#if CONFIG_ETHERNET
					if (idx == NET_IF_NUM - 1) { // This is the ethernet interface, set it up for static ip address
						netif_set_up(pnetif);
					}
#endif
					return DHCP6_TIMEOUT;
				}
			}
		}
		break;
		case DHCP6_RELEASE_IP:
			printf("\n\rLwIP_DHCP6: Release ipv6");
			dhcp6_release(pnetif);
			return DHCP6_RELEASE_IP;
		case DHCP6_STOP:
			printf("\n\rLwIP_DHCP6: dhcp6 stop.");
			dhcp6_stop(pnetif);
			return DHCP6_STOP;
		default:
			break;
		}
		vTaskDelay(30);
	}
}
#endif
uint8_t *LwIP_GetIPv6_linklocal(struct netif *pnetif)
{
	return (uint8_t *) netif_ip6_addr(pnetif, 0)->addr;
}

uint8_t *LwIP_GetIPv6_global(struct netif *pnetif)
{
	return (uint8_t *) netif_ip6_addr(pnetif, 1)->addr;
}
#endif
#endif

uint8_t *LwIP_GetGW(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	return (uint8_t *) & (pnetif->gw);
}

uint8_t *LwIP_GetMASK(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	return (uint8_t *) & (pnetif->netmask);
}

uint8_t *LwIP_GetDHCPSERVER(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	struct dhcp *dhcp = NULL;
#if LWIP_VERSION_MAJOR >= 2
	dhcp = ((struct dhcp *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP));
#else
	dhcp = pnetif->dhcp;
#endif

	return (uint8_t *) & (dhcp->server_ip_addr);
}

void LwIP_wlan_set_netif_info(int idx_wlan, void *dev, unsigned char *dev_addr)
{
	rtw_memcpy(xnetif[idx_wlan].hwaddr, dev_addr, 6);
	xnetif[idx_wlan].state = dev;
}

void LwIP_ethernetif_recv(uint8_t idx, int total_len)
{
	ethernetif_recv(&xnetif[idx], total_len);
}

int LwIP_netif_is_valid_IP(int idx, unsigned char *ip_dest)
{
#if CONFIG_LWIP_LAYER == 1
	struct netif *pnetif = &xnetif[idx];

	ip_addr_t addr = { 0 };
	u32_t ip_dest_addr = { 0 };

	memcpy(&ip_dest_addr, ip_dest, 4);

#if LWIP_VERSION_MAJOR >= 2
	ip_addr_set_ip4_u32(&addr, ip_dest_addr);
#else
	addr.addr = ip_dest_addr;
#endif

#if (LWIP_VERSION_MAJOR >= 2)
	if ((ip_addr_get_ip4_u32(netif_ip_addr4(pnetif))) == 0) {
		return 1;
	}
#else

	if (pnetif->ip_addr.addr == 0) {
		return 1;
	}
#endif

	if (ip_addr_ismulticast(&addr) || ip_addr_isbroadcast(&addr, pnetif)) {
		return 1;
	}

	//if(ip_addr_netcmp(&(pnetif->ip_addr), &addr, &(pnetif->netmask))) //addr&netmask
	//	return 1;

	if (ip_addr_cmp(&(pnetif->ip_addr), &addr)) {
		return 1;
	}

	//printf("invalid IP: %d.%d.%d.%d ",ip_dest[0],ip_dest[1],ip_dest[2],ip_dest[3]);
#endif
	UNUSED(idx);
	UNUSED(ip_dest);
	return 0;
}

uint8_t *LwIP_GetBC(uint8_t idx)
{
	/* To avoid gcc warnings */
	UNUSED(idx);

#if LWIP_VERSION_MAJOR >= 2
	//struct dhcp *dhcp = ((struct dhcp*)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP));
	return NULL;
#else
	struct netif *pnetif = &xnetif[idx];
	return (uint8_t *) & (pnetif->dhcp->offered_bc_addr);
#endif
}

#if LWIP_DNS
void LwIP_GetDNS(struct ip_addr *dns)
{
#if LWIP_VERSION_MAJOR >= 2
	struct ip_addr *tmp = (struct ip_addr *)dns_getserver(0);
	*dns = *tmp;
#else
	*dns = dns_getserver(0);
#endif
}

void LwIP_SetDNS(struct ip_addr *dns)
{
	dns_setserver(0, dns);
}
#endif

void LwIP_SetIP(uint8_t idx, u32_t addr, u32_t netmask_addr, u32_t gw_addr)
{

	if (lwip_compatibility_is_enable(lwip_compatibilty_enabled, BIT(0))) {
		printf("[LwIP_SetIP] lwip_compatibility_is_enable return;\n\r");
		return;
	}

	struct netif *pnetif = &xnetif[idx];
	struct ip_addr ipaddr;
	struct ip_addr netmask;
	struct ip_addr gw;

#if CONFIG_WLAN
#if LWIP_VERSION_MAJOR >= 2
	ip_2_ip4(&ipaddr)->addr = PP_HTONL(addr);
	ip_2_ip4(&netmask)->addr = PP_HTONL(netmask_addr);
	ip_2_ip4(&gw)->addr = PP_HTONL(gw_addr);
	netif_set_addr(pnetif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
#else
	ipaddr->addr = PP_HTONL(addr);
	netmask->addr = PP_HTONL(netmask_addr);
	gw->addr = PP_HTONL(gw_addr);
	netif_set_addr(pnetif, &ipaddr, &netmask, &gw);
#endif
#endif
}

#if LWIP_AUTOIP
#include <lwip/autoip.h>
#if LWIP_VERSION_MAJOR >= 2
#include <lwip/prot/autoip.h>
#if LWIP_VERSION_MINOR >= 2
#include <lwip/prot/acd.h>
#endif
#endif

void LwIP_AUTOIP(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	uint8_t *ip = LwIP_GetIP(idx);
	struct autoip *autoip = NULL;

#if LWIP_VERSION_MAJOR >= 2
	autoip = ((struct autoip *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_AUTOIP));
#else
	autoip = pnetif->autoip;
#endif
	if (autoip && (autoip->tried_llipaddr >= MAX_CONFLICTS)) { // before autoip_start(), autoip may be NULL
		autoip->tried_llipaddr = 0;
	}

	autoip_start(pnetif);

#if LWIP_VERSION_MAJOR >= 2
	autoip = ((struct autoip *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_AUTOIP));
#else
	autoip = pnetif->autoip;
#endif

#if (LWIP_VERSION_MAJOR >= 2) && (LWIP_VERSION_MINOR >= 2)
	while (autoip->state == AUTOIP_STATE_CHECKING) {
#else
	while ((autoip->state == AUTOIP_STATE_PROBING) || (autoip->state == AUTOIP_STATE_ANNOUNCING)) {
#endif
		vTaskDelay(1000);
	}

	if (*((uint32_t *) ip) == 0) {
		struct ip_addr ipaddr;
		struct ip_addr netmask;
		struct ip_addr gw;

		printf("AUTOIP timeout\n");

		/* Static address used */
#if LWIP_VERSION_MAJOR >= 2
		IP4_ADDR(ip_2_ip4(&ipaddr), STATIC_IP_ADDR0, STATIC_IP_ADDR1, STATIC_IP_ADDR2, STATIC_IP_ADDR3);
		IP4_ADDR(ip_2_ip4(&netmask), NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
		IP4_ADDR(ip_2_ip4(&gw), GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
		netif_set_addr(pnetif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
#else
		IP4_ADDR(&ipaddr, STATIC_IP_ADDR0, STATIC_IP_ADDR1, STATIC_IP_ADDR2, STATIC_IP_ADDR3);
		IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
		IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
		netif_set_addr(pnetif, &ipaddr, &netmask, &gw);
#endif
		printf("Static IP address : %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
	} else {
		printf("\nLink-local address: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
	}
}

void LwIP_AUTOIP_STOP(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	autoip_stop(pnetif);
}
#endif
#if LWIP_IPV6
/* Get IPv6 address with lwip 1.5.0 */
void LwIP_AUTOIP_IPv6(struct netif *pnetif)
{
#if LWIP_VERSION_MAJOR >= 2
	uint8_t *ipv6 = (uint8_t *) netif_ip6_addr(pnetif, 0)->addr;
#else
	uint8_t *ipv6 = (uint8_t *) & (pnetif->ip6_addr[0].addr[0]);
#endif
	netif_create_ip6_linklocal_address(pnetif, 1);
	printf("\nIPv6 link-local address: %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x\n",
		   ipv6[0], ipv6[1],  ipv6[2],  ipv6[3],  ipv6[4],  ipv6[5],  ipv6[6], ipv6[7],
		   ipv6[8], ipv6[9], ipv6[10], ipv6[11], ipv6[12], ipv6[13], ipv6[14], ipv6[15]);
}
#endif

uint32_t LWIP_Get_Dynamic_Sleep_Interval()
{
#ifdef DYNAMIC_TICKLESS_SLEEP_INTERVAL
	return DYNAMIC_TICKLESS_SLEEP_INTERVAL;
#else
	return 0;
#endif
}

uint32_t LwIP_GetXID(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	struct dhcp *dhcp = NULL;
#if LWIP_VERSION_MAJOR >= 2
	dhcp = ((struct dhcp *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP));
#else
	dhcp = pnetif->dhcp;
#endif
	return dhcp->xid;
}

uint32_t LwIP_GetLEASETIME(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	struct dhcp *dhcp = NULL;
#if LWIP_VERSION_MAJOR >= 2
	dhcp = ((struct dhcp *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP));
#else
	dhcp = pnetif->dhcp;
#endif
	return dhcp->offered_t0_lease;
}

uint32_t LwIP_GetRENEWTIME(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	struct dhcp *dhcp = NULL;
#if LWIP_VERSION_MAJOR >= 2
	dhcp = ((struct dhcp *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP));
#else
	dhcp = pnetif->dhcp;
#endif

	return dhcp->t1_renew_time;
}

struct netif *LwIP_ip4_route_src_hook(const ip4_addr_t *src, const ip4_addr_t *dest)
{
	struct netif *netif = NULL;

	/* when src==NULL, the hook is called from ip4_route(dest) */
	if (src == NULL) {
		return NULL;
	}

	/* iterate through netifs */
	NETIF_FOREACH(netif) {
		/* is the netif up, does it have a link and a valid address? */
		if (netif_is_up(netif) && netif_is_link_up(netif) && !ip4_addr_isany_val(*netif_ip4_addr(netif))) {
			/* address matches? */
			if (ip4_addr_cmp(src, netif_ip4_addr(netif))) {
				return netif;
			}
		}
	}

	return NULL;
}
