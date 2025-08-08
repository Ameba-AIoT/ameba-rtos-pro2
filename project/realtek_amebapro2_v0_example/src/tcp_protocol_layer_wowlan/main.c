#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "main.h"
#include "log_service.h"
#include "osdep_service.h"
#include <platform_opts.h>
#include <platform_opts_bt.h>
#include "sys_api.h"

#include "wowlan_driver_api.h"
#include "wifi_conf.h"
#include <lwip_netconf.h>
#include <lwip/sockets.h>

#include "power_mode_api.h"
#include "time64.h"

#define LONG_RUN_TEST	0
#define STACKSIZE     2048
#define TCP_RESUME    1
#define TCP_RESUME_MAX_PACKETS 3
#define SSL_KEEPALIVE 0
#if LONG_RUN_TEST
#define WOWLAN_GPIO_WDT      0
#else
#define WOWLAN_GPIO_WDT      1
#endif
#define WOWLAN_CONNECTIVITY_CHECK     0
//Clock, 1: 4MHz, 0: 100kHz
#define CLOCK 0
//SLEEP_DURATION, 120s
#define SLEEP_DURATION (3 * 60 * 1000 * 1000)
#define TEST_DISCONNECT_PNO 0
#define TEST_DUAL_BAND_PNO	1


static uint8_t wowlan_wake_reason = 0;
static uint32_t pm_reason = 0;
static uint8_t suspend_wakeup = 0;
static uint8_t suspend_fail = 0;
static uint8_t wlan_resume = 0;
static uint8_t tcp_resume = 0;
static uint8_t ssl_resume = 0;
static uint8_t disconnect_pno = 0;
//dtim
static uint8_t lps_dtim = 10;

//stage1 setting
static uint8_t rx_bcn_window = 30;
static uint8_t bcn_limit = 2;

//stage2 setting
static uint8_t  stage2_start_window = 10;
static uint16_t  stage2_max_window = 310;
static uint8_t  stage2_increment_steps = 50;
static uint8_t  stage2_duration = 13;
static uint8_t  stage2_null_limit = 6;
static uint8_t  stage2_loop_limit = 6;

//pno scan setting
static uint8_t  pno_start_window = 5;
static uint8_t  pno_max_window = 240;
static uint8_t  pno_increment_steps = 10;
static uint32_t  pno_passive_scan_cnt = 3;
static uint32_t  pno_active_scan_cnt = 1;
static uint32_t  pno_duration = 180;
static uint8_t pno_intervaltime = 3;

static uint8_t set_dtimtimeout = 1;
static uint8_t set_rxbcnlimit = 8;
static uint8_t set_pstimeout = 16;
static uint8_t set_psretry = 7;
static uint8_t set_arpreqperiod = 50;
static uint8_t set_arpreqenable = 1;
static uint8_t set_tcpkaenable = 1;
#if LONG_RUN_TEST
static uint8_t set_pnoenable = 0;
#else
static uint8_t set_pnoenable = 1;
#endif

static uint8_t pre_dtim = 0;
static uint64_t sleep_duration = 0;
static uint8_t wowlan_dtim2 = 0;
static uint8_t sleeptime_threshold_l = 0;
static uint8_t sleeptime_threshold_h = 0;
static uint32_t bcn_cnt = 0;

#define SLEEPTIME_THRESHOLD_L 	(2 * 60 * 60)
#define SLEEPTIME_THRESHOLD_H 	(4 * 60 * 60)
#if LONG_RUN_TEST
#define RECV_BCN_THRESHOLD		5
#else
#define RECV_BCN_THRESHOLD		15
#endif

#if TEST_DISCONNECT_PNO
static const char *pno_scan_ssid = {"TEST_AP"};
static uint8_t pno_scan_ssid_len = 7;
#else
static char pno_scan_ssid[33] = {0};
static uint8_t pno_scan_ssid_len = 0;
static uint8_t pno_cur_ch = 0;
#endif

//tcp protocol keep alive power bit setting
static uint8_t tcp_ka_power_bit = 0;

static char server_ip[16] = "192.168.0.163";
static uint16_t server_port = 5566;
__attribute__((section(".retention.data"))) uint16_t retention_local_port __attribute__((aligned(32))) = 0;

#if LONG_RUN_TEST
static uint8_t goto_sleep = 1;
#else
static uint8_t goto_sleep = 0;
#endif
static int keepalive = 1, keepalive_idle = 240, keepalive_interval = 5, keepalive_count = 12;

#if CONFIG_WLAN
#include <wifi_fast_connect.h>
extern void wlan_network(void);
#endif

#include "gpio_api.h"
#include "gpio_irq_api.h"
#include "gpio_irq_ex_api.h"

#define WAKUPE_GPIO_PIN PA_2
static gpio_irq_t my_GPIO_IRQ;
void gpio_demo_irq_handler(uint32_t id, gpio_irq_event event)
{
	dbg_printf("%s==> \r\n", __FUNCTION__);
}

extern void console_init(void);

#include "mbedtls/version.h"
#include "mbedtls/config.h"
#include "mbedtls/ssl.h"
#include "mbedtls/platform.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/aes.h"
#include "mbedtls/md.h"
#if defined(MBEDTLS_VERSION_NUMBER) && (MBEDTLS_VERSION_NUMBER >= 0x03000000)
#include "mbedtls/../../library/ssl_misc.h"
#include "mbedtls/../../library/md_wrap.h"
#else
#include "mbedtls/ssl_internal.h"
#include "mbedtls/md_internal.h"
#endif

static void *_calloc_func(size_t nmemb, size_t size)
{
	size_t mem_size;
	void *ptr = NULL;

	mem_size = nmemb * size;
	ptr = pvPortMalloc(mem_size);

	if (ptr) {
		memset(ptr, 0, mem_size);
	}

	return ptr;
}

static int _random_func(void *p_rng, unsigned char *output, size_t output_len)
{
	/* To avoid gcc warnings */
	(void) p_rng;

	rtw_get_random_bytes(output, output_len);
	return 0;
}

uint8_t get_dtim_weight(uint64_t sleep_time)
{
	uint8_t weight = 0;

	if (sleep_time < (6 * 60 * 60)) {
		weight = 1;
	} else if ((sleep_time > (6 * 60 * 60)) && (sleep_time < (12 * 60 * 60))) {
		weight = 2;
	} else if (sleep_time > (12 * 60 * 60)) {
		weight = 3;
	}

	return weight;
}

void tcp_app_task(void *param)
{
	uint8_t standbyto = 0;

#if LONG_RUN_TEST
	vTaskDelay(6000);
#endif

	while (!goto_sleep) {
		if (tcp_resume) {
			break;
		}

		if (disconnect_pno) {
			goto pno_suspend;
		}

		vTaskDelay(2000);
		u32 value32;
		value32 = HAL_READ32(0x40080000, 0x54);
		value32 = value32 & 0xFF;
		printf("beacon recv per 20 = %d\r\n", value32);
	}

	int ret = 0;
	int sock_fd = -1;
#if SSL_KEEPALIVE
	mbedtls_ssl_context ssl;
	mbedtls_ssl_config conf;
#endif

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	// wait for IP address
	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(10);
	}

#if WOWLAN_CONNECTIVITY_CHECK
	//Set connectivity(connectivitycheck.gstatic.com) ip address
	wifi_set_connectivity_ip();
	//The upper layer determines whether to access the external network to decide whether to activate this function.
	if (set_tcpkaenable == 0) {
		wifi_set_connectivity_offload(60, 3600);
	}
#endif

	if (set_tcpkaenable) {
		// socket
		sock_fd = socket(AF_INET, SOCK_STREAM, 0);
		printf("\n\r socket(%d) \n\r", sock_fd);

		if (setsockopt(sock_fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) != 0) {
			printf("ERROR: SO_KEEPALIVE\n");
		}
		if (setsockopt(sock_fd, IPPROTO_TCP, TCP_KEEPIDLE, &keepalive_idle, sizeof(keepalive_idle)) != 0) {
			printf("ERROR: TCP_KEEPIDLE\n");
		}
		if (setsockopt(sock_fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepalive_interval, sizeof(keepalive_interval)) != 0) {
			printf("ERROR: TCP_KEEPINTVL\n");
		}
		if (setsockopt(sock_fd, IPPROTO_TCP, TCP_KEEPCNT, &keepalive_count, sizeof(keepalive_count)) != 0) {
			printf("ERROR: TCP_KEEPCNT\n");
		}

		if (tcp_resume) {
			// resume on the same local port
			if (retention_local_port != 0) {
				struct sockaddr_in local_addr;
				local_addr.sin_family = AF_INET;
				local_addr.sin_addr.s_addr = INADDR_ANY;
				local_addr.sin_port = htons(retention_local_port);
				printf("bind local port:%d %s \n\r", retention_local_port, bind(sock_fd, (struct sockaddr *) &local_addr, sizeof(local_addr)) == 0 ? "OK" : "FAIL");
			}

#if TCP_RESUME
			// resume tcp pcb
			extern int lwip_resume_tcp(int s);
			printf("resume TCP pcb & seqno & ackno %s \n\r", lwip_resume_tcp(sock_fd) == 0 ? "OK" : "FAIL");
#endif
		} else {
			// connect
			struct sockaddr_in server_addr;
			server_addr.sin_family = AF_INET;
			server_addr.sin_addr.s_addr = inet_addr(server_ip);
			server_addr.sin_port = htons(server_port);

			if (connect(sock_fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) == 0) {
				// retain local port
				struct sockaddr_in sin;
				socklen_t len = sizeof(sin);
				if (getsockname(sock_fd, (struct sockaddr *)&sin, &len) == -1) {
					printf("ERROR: getsockname \n\r");
				} else {
					retention_local_port = ntohs(sin.sin_port);
					dcache_clean_invalidate_by_addr((uint32_t *) &retention_local_port, sizeof(retention_local_port));
					printf("retain local port: %d \n\r", retention_local_port);
				}

				printf("connect to %s:%d OK \n\r", server_ip, server_port);
			} else {
				printf("connect to %s:%d FAIL \n\r", server_ip, server_port);
				close(sock_fd);
				goto exit1;
			}
		}
	}
#if SSL_KEEPALIVE
	mbedtls_platform_set_calloc_free(_calloc_func, vPortFree);

	mbedtls_ssl_init(&ssl);
	mbedtls_ssl_config_init(&conf);
	mbedtls_ssl_set_bio(&ssl, &sock_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

	if ((ret = mbedtls_ssl_config_defaults(&conf,
										   MBEDTLS_SSL_IS_CLIENT,
										   MBEDTLS_SSL_TRANSPORT_STREAM,
										   MBEDTLS_SSL_PRESET_DEFAULT)) != 0) {

		printf("\nERROR: mbedtls_ssl_config_defaults %d\n", ret);
		goto exit;
	}

	static int ciphersuites[] = {MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA384, 0};
	mbedtls_ssl_conf_ciphersuites(&conf, ciphersuites);
	mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_NONE);
	mbedtls_ssl_conf_rng(&conf, _random_func, NULL);
	mbedtls_ssl_conf_max_version(&conf, MBEDTLS_SSL_MAJOR_VERSION_3, MBEDTLS_SSL_MINOR_VERSION_3); // TLS 1.2

	if ((ret = mbedtls_ssl_setup(&ssl, &conf)) != 0) {
		printf("\nERROR: mbedtls_ssl_setup %d\n", ret);
		goto exit;
	}

	if (ssl_resume) {
		extern int mbedtls_ssl_resume(mbedtls_ssl_context * ssl);
		printf("resume SSL %s \n\r", mbedtls_ssl_resume(&ssl) == 0 ? "OK" : "FAIL");
	} else {
		// handshake
		if ((ret = mbedtls_ssl_handshake(&ssl)) != 0) {
			printf("\nERROR: mbedtls_ssl_handshake %d\n", ret);
			goto exit;
		} else {
			printf("\nUse ciphersuite %s\n", mbedtls_ssl_get_ciphersuite(&ssl));
		}
	}
#endif
	while (!goto_sleep) {
#if 0
#if SSL_KEEPALIVE
		ret = mbedtls_ssl_write(&ssl, (uint8_t *) "_APP", strlen("_APP"));
#else
		ret = write(sock_fd, "_APP", strlen("_APP"));
#endif
		printf("write application data %d \n\r", ret);
#endif
		if (ret < 0) {
#if SSL_KEEPALIVE
			mbedtls_ssl_close_notify(&ssl);
#endif
			goto exit;
		}

		vTaskDelay(5000);
	}

#if SSL_KEEPALIVE
	// retain ssl
	extern int mbedtls_ssl_retain(mbedtls_ssl_context * ssl);
	printf("retain SSL %s \n\r", mbedtls_ssl_retain(&ssl) == 0 ? "OK" : "FAIL");
#endif
#if TCP_RESUME
	// retain tcp pcb
	if (set_tcpkaenable) {
		extern int lwip_retain_tcp(int s);
		printf("retain TCP pcb %s \n\r", lwip_retain_tcp(sock_fd) == 0 ? "OK" : "FAIL");
	}
#endif

	// set ntp offload
	extern int wifi_set_ntp_offload(char *server_names[], uint8_t num_servers, uint32_t update_delay_ms);
	char *server_name[4] = {(char *)"pool.ntp.org"};
	wifi_set_ntp_offload(server_name, 1, 60 * 60 * 1000);

	if (set_tcpkaenable) {
		// set keepalive
		extern int wifi_set_tcp_protocol_keepalive_offload(int socket_fd, uint8_t power_bit);
		wifi_set_tcp_protocol_keepalive_offload(sock_fd, tcp_ka_power_bit);
	}

	wifi_set_dhcp_offload(); // after wifi_set_tcp_keep_alive_offload

	if (set_arpreqenable) {
		wifi_wowlan_set_arpreq_keepalive(0, 0); // arp0, null1
	}

	//dynamic dtim
	uint8_t level = rtl8735b_get_dynamic_dtim_level();
	uint8_t dtim_max =  10 - (10 % level);
	uint8_t weight = get_dtim_weight(sleep_duration);

	printf("\r\nsleep duration %llu pre_dtim %d  bcn_cnt %d\r\n", sleep_duration, pre_dtim, bcn_cnt);
	printf("\r\nlevel %d weight %d dtim_max %d\r\n", level, weight, dtim_max);

	wowlan_dtim2 = pre_dtim;
	if (pm_reason & BIT(3)) {
		if (wowlan_wake_reason) { //wlan wake
			if (wowlan_wake_reason == TX_TCP_SEND_LIMIT || wowlan_wake_reason == FW_BCN_TO_WAKEUP || wowlan_wake_reason == RX_TCP_FIN_PKT ||
				wowlan_wake_reason == RX_TCP_RST_PKT || wowlan_wake_reason == RX_DEAUTH ||
				wowlan_wake_reason == RX_DISASSOC) {
				if (sleep_duration < SLEEPTIME_THRESHOLD_L) {
					uint32_t keep_alive_fail_time = keepalive_idle * 3;
					if ((wowlan_wake_reason == RX_TCP_FIN_PKT || wowlan_wake_reason == RX_TCP_RST_PKT || wowlan_wake_reason == TX_TCP_SEND_LIMIT) &&
						(sleep_duration < keep_alive_fail_time)) {
						printf("\r\ntcp layer issue, do not adjust dtim\r\n");
					} else {
						if (wowlan_dtim2 == 1) {
							wowlan_dtim2 = 1;
						} else {
							wowlan_dtim2 = pre_dtim - level;
							if (wowlan_dtim2 == 0) {
								wowlan_dtim2 = 1;
							}
						}
					}
				} else if (sleep_duration > SLEEPTIME_THRESHOLD_H) {
					if (wowlan_dtim2 == 1) {
						wowlan_dtim2 = level + (level * (weight - 1));
					} else {
						wowlan_dtim2 = pre_dtim + (level * weight);
						if (wowlan_dtim2 > dtim_max) {
							wowlan_dtim2 = dtim_max;
						}
					}
				}
			} else if (wowlan_wake_reason == RX_TCP_WITH_PAYLOAD || wowlan_wake_reason == RX_UNICAST_PKT) {
				if (bcn_cnt > RECV_BCN_THRESHOLD) {
					if (wowlan_dtim2 == 1) {
						wowlan_dtim2 = level + (level * (weight - 1));
					} else {
						wowlan_dtim2 = pre_dtim + (level * weight);
						if (wowlan_dtim2 > dtim_max) {
							wowlan_dtim2 = dtim_max;
						}
					}
				}
			}
			rtl8735b_set_lps_dtim(wowlan_dtim2);
			printf("\r\nadjust dtim to %d\r\n", wowlan_dtim2);
		}
	} else if (pm_reason & (BIT(9) | BIT(10) | BIT(11) | BIT(12))) { //GPIO wake
		if (wowlan_dtim2 != 0) {
			if (bcn_cnt > RECV_BCN_THRESHOLD) {
				if (wowlan_dtim2 == 1) {
					wowlan_dtim2 = level + (level * (weight - 1));
				} else {
					wowlan_dtim2 = pre_dtim + (level * weight);
					if (wowlan_dtim2 > dtim_max) {
						wowlan_dtim2 = dtim_max;
					}
				}
			}
		} else {
			//first enter wowlan
			wowlan_dtim2 = dtim_max;
		}
		rtl8735b_set_lps_dtim(wowlan_dtim2);
		printf("\r\nadjust dtim to %d\r\n", wowlan_dtim2);
	} else if (pm_reason & BIT(6)) { //Timer wake
		if (suspend_fail && (pre_dtim != 0)) {
			printf("\r\nsuspend fail %d, suspend wakeup reason %d\r\n", suspend_fail, suspend_wakeup);
			if (suspend_wakeup == 0x74) {
				wowlan_dtim2 = 1;
			} else {
				wowlan_dtim2 = pre_dtim;
			}
		} else {
			if (wowlan_dtim2 != 0) {
				if (bcn_cnt > RECV_BCN_THRESHOLD) {
					if (wowlan_dtim2 == 1) {
						wowlan_dtim2 = level + (level * (weight - 1));
					} else {
						wowlan_dtim2 = pre_dtim + (level * weight);
						if (wowlan_dtim2 > dtim_max) {
							wowlan_dtim2 = dtim_max;
						}
					}
				}
			} else {
				//first enter wowlan
				wowlan_dtim2 = dtim_max;
			}
		}
		rtl8735b_set_lps_dtim(wowlan_dtim2);
		printf("\r\nadjust dtim to %d\r\n", wowlan_dtim2);
	} else {
		rtl8735b_set_lps_dtim(lps_dtim);
	}


	wifi_wowlan_set_dtimtimeout(set_dtimtimeout);

	//set bcn track stage2 parmeters
	//static uint8_t rx_bcn_window = 30;
	//static uint8_t bcn_limit = 5;
	wifi_wowlan_bcntrack_stage1(rx_bcn_window, bcn_limit);

	//set bcn track stage2 parmeters
	// static uint8_t  stage2_start_window = 10;
	// static uint16_t  stage2_max_window = 310;
	// static uint8_t  stage2_increment_steps = 50;
	// static uint8_t  stage2_duration = 13;
	// static uint8_t  sstage2_null_limit = 6;
	// static uint8_t  sstage2_null_limit = 6;
	wifi_wowlan_set_bcn_track(stage2_start_window, stage2_max_window, stage2_increment_steps, stage2_duration, stage2_null_limit, stage2_loop_limit);

pno_suspend:
	if (set_pnoenable) {
		if (!(wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS)) {
#if 1
			extern void wifi_get_pno_disconnect_params(char *ssid, uint8_t *ssid_len, uint8_t *channel);
			wifi_get_pno_disconnect_params(pno_scan_ssid, &pno_scan_ssid_len, &pno_cur_ch);
			printf("\r\nget pno scan ssid = %s ssid length = %d current channel = %d\r\n", pno_scan_ssid, pno_scan_ssid_len, pno_cur_ch);
			//set pno scan ssid and ssid length
			uint8_t pno_ch_list_len = 0;
			uint8_t *pno_ch_list = NULL;
			wifi_wowlan_set_pno_scan_ssid((uint8_t *)pno_scan_ssid, pno_scan_ssid_len, pno_cur_ch, pno_ch_list, pno_ch_list_len);

#if TEST_DUAL_BAND_PNO
			//set pno scan another band(trigger dual band pno)
			char *pno_scan_another_band_ssid = {(char *)"TEST_AP_5G"};
			uint8_t pno_scan_another_band_ssid_len = 10;
			uint8_t pno_another_band_ch_list_len = 8;
			uint8_t *pno_another_band_ch_list = malloc(pno_another_band_ch_list_len * sizeof(uint8_t));
			uint8_t ch36 = 36;
			for (int x = 0; x < pno_another_band_ch_list_len; x++) {
				pno_another_band_ch_list[x] = ch36 + (x * 4);
			}
			wifi_wowlan_set_pno_another_band_scan_ssid((uint8_t *)pno_scan_another_band_ssid, pno_scan_another_band_ssid_len, pno_another_band_ch_list,
					pno_another_band_ch_list_len);
#endif
			//set pno scan parmeters
			// static uint8_t  pno_start_window = 5;
			// static uint8_t  pno_max_window = 240;
			// static uint8_t  pno_increment_steps = 10;
			// static uint8_t  pno_passive_scan_cnt = 3;
			// static uint8_t  pno_active_scan_cnt = 1;
			// static uint32_t pno_duration = 180;
			// static uint8_t  pno_intervaltime = 3;
			wifi_wowlan_set_pno_scan(pno_start_window, pno_max_window, pno_increment_steps, pno_passive_scan_cnt, pno_active_scan_cnt, pno_duration, pno_intervaltime);
#else
			//test dual band 5G-2.4G
			extern void wifi_get_pno_disconnect_params(char *ssid, uint8_t *ssid_len, uint8_t *channel);
			wifi_get_pno_disconnect_params(pno_scan_ssid, &pno_scan_ssid_len, &pno_cur_ch);
			printf("\r\nget pno scan ssid = %s ssid length = %d current channel = %d\r\n", pno_scan_ssid, pno_scan_ssid_len, pno_cur_ch);
			//set pno scan ssid and ssid length
			uint8_t pno_ch_list_len = 8;
			uint8_t *pno_ch_list = malloc(pno_ch_list_len * sizeof(uint8_t));
			uint8_t ch36 = 36;
			for (int x = 0; x < pno_ch_list_len; x++) {
				pno_ch_list[x] = ch36 + (x * 4);
			}
			wifi_wowlan_set_pno_scan_ssid((uint8_t *)pno_scan_ssid, pno_scan_ssid_len, pno_cur_ch, pno_ch_list, pno_ch_list_len);

			//set pno scan another band(trigger dual band pno)
			char *pno_scan_another_band_ssid = {"TEST_AP_2G"};
			uint8_t pno_scan_another_band_ssid_len = 10;
			uint8_t pno_another_band_ch_list_len = 0;
			uint8_t *pno_another_band_ch_list = NULL;
			wifi_wowlan_set_pno_another_band_scan_ssid((uint8_t *)pno_scan_another_band_ssid, pno_scan_another_band_ssid_len, pno_another_band_ch_list,
					pno_another_band_ch_list_len);

			//set pno scan parmeters
			// static uint8_t  pno_start_window = 5;
			// static uint8_t  pno_max_window = 240;
			// static uint8_t  pno_increment_steps = 10;
			// static uint8_t  pno_passive_scan_cnt = 3;
			// static uint8_t  pno_active_scan_cnt = 1;
			// static uint32_t pno_duration = 180;
			// static uint8_t  pno_intervaltime = 3;
			wifi_wowlan_set_pno_scan(pno_start_window, pno_max_window, pno_increment_steps, pno_passive_scan_cnt, pno_active_scan_cnt, pno_duration, pno_intervaltime);
#endif
			//select fw
			rtl8735b_select_keepalive(WOWLAN_PNO);
		}  else {
			//select fw
			rtl8735b_select_keepalive(WOWLAN_TCPPTL_BCNV2);
		}

	} else {
		//select fw
		rtl8735b_select_keepalive(WOWLAN_TCPPTL_BCNV2);
	}

#if WOWLAN_GPIO_WDT
	wifi_wowlan_set_wdt(2, 5, 1, 1); //gpiof_2, io trigger interval 1 min, io pull high and trigger pull low, pulse duration 1ms
#endif

	if (!disconnect_pno) {
		extern int dhcp_retain(void);
		printf("retain DHCP %s \n\r", dhcp_retain() == 0 ? "OK" : "FAIL");
		// for wlan resume
		rtw_hal_wlan_resume_backup();
	}

	// sleep
	rtl8735b_set_lps_pg();
	rtw_enter_critical(NULL, NULL);
	ret = rtl8735b_suspend(0);
	if (ret == 0) { // should stop wifi application before doing rtl8735b_suspend(
		printf("rtl8735b_suspend\r\n");

		// wakeup GPIO
		gpio_irq_init(&my_GPIO_IRQ, WAKUPE_GPIO_PIN, gpio_demo_irq_handler, (uint32_t)&my_GPIO_IRQ);
		gpio_irq_pull_ctrl(&my_GPIO_IRQ, PullDown);
		gpio_irq_set(&my_GPIO_IRQ, IRQ_RISE, 1);
#if WOWLAN_GPIO_WDT

		//set gpio pull control
		HAL_WRITE32(0x40009850, 0x0, 0x4f004f); //GPIOF_1/GPIOF_0
		// HAL_WRITE32(0x40009854, 0x0, 0x8f004f); //GPIOF_3/GPIOF_2
		HAL_WRITE32(0x40009854, 0x0, 0x8f0000 | (0xffff & HAL_READ32(0x40009854, 0x0))); //GPIOF_3/GPIOF_2 keep wowlan wdt setting
		HAL_WRITE32(0x40009858, 0x0, 0x4f008f); //GPIOF_5/GPIOF_4
		HAL_WRITE32(0x4000985c, 0x0, 0x4f004f); //GPIOF_7/GPIOF_6
		HAL_WRITE32(0x40009860, 0x0, 0x4f004f); //GPIOF_9/GPIOF_8
		HAL_WRITE32(0x40009864, 0x0, 0x4f004f); //GPIOF_11/GPIOF_10
		HAL_WRITE32(0x40009868, 0x0, 0x4f004f); //GPIOF_13/GPIOF_12
		HAL_WRITE32(0x4000986C, 0x0, 0x4f004f); //GPIOF_15/GPIOF_14
		HAL_WRITE32(0x40009870, 0x0, 0x4f004f); //GPIOF_17/GPIOF_16
		rtw_exit_critical(NULL, NULL);
		while (1) {
			standbyto++;
			ret = Standby(DSTBY_AON_GPIO | DSTBY_PON_GPIO | DSTBY_WLAN | SLP_GTIMER, 0, 0, 1);
			if (standbyto >= 100) {
				printf("\r\nStandby fail 0x%x, need to sys_reset\r\n", ret);
				break;
			}
			vTaskDelay(1);
		}
#else
		// standby with retention: add SLP_GTIMER and set SramOption to retention mode(1)
		rtw_exit_critical(NULL, NULL);
		while (1) {
			standbyto++;
#if LONG_RUN_TEST
			ret = Standby(DSTBY_AON_GPIO | DSTBY_WLAN | SLP_GTIMER | DSTBY_AON_TIMER, SLEEP_DURATION, CLOCK, 1);
#else
			ret = Standby(DSTBY_AON_GPIO | DSTBY_WLAN | SLP_GTIMER, 0, 0, 1);
#endif
			if (standbyto >= 100) {
				printf("\r\nStandby fail 0x%x, need to sys_reset\r\n", ret);
				break;
			}
			vTaskDelay(1);
		}
#endif
		printf("wifi disconnect\r\n");
	} else {
		rtw_exit_critical(NULL, NULL);
		if (ret != -4) {
			wifi_off();
			HAL_WRITE32(0x40009000, 0x18, 0x1 | HAL_READ32(0x40009000, 0x18)); //SWR 1.35V
			Standby(SLP_AON_TIMER | SLP_GTIMER, 1000000 /* 1s */, 0/* CLOCK */, 1 /* SRAM retention */);
		}
		printf("rtl8735b_suspend fail\r\n");
	}

	while (1) {
		vTaskDelay(2000);
		printf("sleep fail!!!\r\n");
		sys_reset();
	}

exit:
	printf("\n\r close(%d) \n\r", sock_fd);
	close(sock_fd);
#if SSL_KEEPALIVE
	mbedtls_ssl_free(&ssl);
	mbedtls_ssl_config_free(&conf);
#endif

exit1:
	vTaskDelete(NULL);
}

int wlan_do_resume(void)
{
	rtw_hal_wlan_resume_restore();

	wifi_fast_connect_enable(1);
	wifi_fast_connect_load_fast_dhcp();

	extern uint8_t lwip_check_dhcp_resume(void);
	if (lwip_check_dhcp_resume() == 1) {
		extern int dhcp_resume(void);
		printf("\n\rresume DHCP %s\n\r", dhcp_resume() == 0 ? "OK" : "FAIL");
	} else {
		LwIP_DHCP(0, DHCP_START);
	}

	return 0;
}

void fPS(void *arg)
{
	int argc;
	char *argv[MAX_ARGC] = {0};

	argc = parse_param(arg, argv);

	if (strcmp(argv[1], "wowlan") == 0) {
		goto_sleep = 1;
	} else if (strcmp(argv[1], "tcp_keep_alive") == 0) {
		if (argc >= 4) {
			sprintf(server_ip, "%s", argv[2]);
			server_port = strtoul(argv[3], NULL, 10);
		}
		printf("setup tcp keep alive to %s:%d\r\n", server_ip, server_port);
	} else if (strcmp(argv[1], "dtim") == 0) {
		if (argc == 3 && atoi(argv[2]) >= 1 && atoi(argv[2]) <= 15) {
			printf("dtim=%02d", atoi(argv[2]));
			lps_dtim = atoi(argv[2]);
		}
	} else if (strcmp(argv[1], "stage1") == 0) {
		if (argc >= 4) {

			rx_bcn_window = atoi(argv[2]);
			bcn_limit = atoi(argv[3]);
		}
		printf("setup stage1 to %d:%d\r\n", atoi(argv[2]), atoi(argv[3]));
	} else if (strcmp(argv[1], "stage2") == 0) {
		if (argc >= 8) {
			stage2_start_window = atoi(argv[2]);
			stage2_max_window = atoi(argv[3]);
			stage2_increment_steps = atoi(argv[4]);
			stage2_duration = atoi(argv[5]);
			stage2_null_limit = atoi(argv[6]);
			stage2_loop_limit = atoi(argv[7]);
		}
		printf("setup stage2 to %d:%d:%d:%d\r\n", atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
	} else if (strcmp(argv[1], "pno_suspend") == 0) {
		disconnect_pno = atoi(argv[2]);
	} else if (strcmp(argv[1], "standby") == 0) {
		printf("into standby mode %d second\r\n", (SLEEP_DURATION / 1000000));
		rtl8735b_set_lps_pg();
		rtl8735b_suspend(0);
		Standby(DSTBY_AON_TIMER, SLEEP_DURATION, CLOCK, 0);
	} else if (strcmp(argv[1], "rx_bcn_limit") == 0) {
		if (argc == 3) {
			printf("rx_bcn_limit=%02d", atoi(argv[2]));
			set_rxbcnlimit = atoi(argv[2]);
			wifi_wowlan_set_rxbcnlimit(set_rxbcnlimit);
		}
	} else if (strcmp(argv[1], "ps_timeout") == 0) {
		if (argc == 3) {
			printf("ps_timeout=%02d", atoi(argv[2]));
			set_pstimeout = atoi(argv[2]);
			wifi_wowlan_set_pstimeout(set_pstimeout);
		}
	} else if (strcmp(argv[1], "ps_retry") == 0) {
		if (argc == 3) {
			printf("ps_retry=%02d", atoi(argv[2]));
			set_psretry = atoi(argv[2]);
			wifi_wowlan_set_psretry(set_psretry);
		}
	} else if (strcmp(argv[1], "dtimtimeout") == 0) {
		if (argc == 3 && atoi(argv[2]) >= 1) {
			printf("dtimtimeout=%02d", atoi(argv[2]));
			set_dtimtimeout = atoi(argv[2]);
		}
	} else if (strcmp(argv[1], "arp_timer") == 0) {
		if (argc == 3) {
			printf("set_arpreqperiod=%02d", atoi(argv[2]));
			set_arpreqperiod = atoi(argv[2]);
			rtw_hal_set_arpreq_period(set_arpreqperiod);
		}
	} else if (strcmp(argv[1], "arp_req") == 0) {
		if (argc == 3) {
			printf("set_arpreqeable=%02d", atoi(argv[2]));
			set_arpreqenable = atoi(argv[2]);
		}
	} else if (strcmp(argv[1], "tcp_ka_time_interval") == 0) {
		if (argc == 3) {
			printf("keepalive_idle=%02d", atoi(argv[2]));
			keepalive_idle = atoi(argv[2]);
		}
	} else if (strcmp(argv[1], "tcp_ka_retry_count") == 0) {
		if (argc == 3) {
			printf("keepalive_count=%02d", atoi(argv[2]));
			keepalive_count = atoi(argv[2]);
		}
	} else if (strcmp(argv[1], "tcp_ka_retry_interval") == 0) {
		if (argc == 3) {
			printf("keepalive_interval=%02d", atoi(argv[2]));
			keepalive_interval = atoi(argv[2]);
		}
	} else if (strcmp(argv[1], "tcp_ka_set") == 0) {
		if (argc == 3) {
			printf("set_tcpkaenable=%02d", atoi(argv[2]));
			set_tcpkaenable = atoi(argv[2]);
		}
	} else if (strcmp(argv[1], "pno_scan") == 0) {
		if (argc >= 8) {
			pno_start_window = atoi(argv[2]);
			pno_max_window = atoi(argv[3]);
			pno_increment_steps = atoi(argv[4]);
			pno_passive_scan_cnt = atoi(argv[5]);
			pno_active_scan_cnt = atoi(argv[6]);
			pno_duration = atoi(argv[7]);
			pno_intervaltime = atoi(argv[8]);
		}
		printf("setup pno scan to %d:%d:%d:%d:%d:%d\r\n", atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6]), atoi(argv[7]));
	} else if (strcmp(argv[1], "pno_enable") == 0) {
		if (argc == 3) {
			printf("set_pnoenable=%02d", atoi(argv[2]));
			set_pnoenable = atoi(argv[2]);
		}
	}
}

log_item_t at_power_save_items[ ] = {
	{"PS", fPS,},
};

void main(void)
{
	uint8_t channel = 0;
	pm_reason = Get_wake_reason();
	printf("\n\rpm_reason=0x%x\n\r", pm_reason);

	hal_xtal_divider_enable(1);
	hal_32k_s1_sel(2);
	HAL_WRITE32(0x40009000, 0x18, 0x1 | HAL_READ32(0x40009000, 0x18)); //SWR 1.35V

	if (pm_reason) {
		uint32_t tcp_resume_seqno = 0, tcp_resume_ackno = 0;
		uint8_t by_wlan = 0, wlan_mcu_ok = 0;

		if (pm_reason & BIT(3)) {
			// WLAN wake up
			by_wlan = 1;

			/* *************************************
				RX_DISASSOC = 0x04,
				RX_DEAUTH = 0x08,
				FW_DECISION_DISCONNECT = 0x10,
				RX_PATTERN_PKT = 0x23,
				TX_TCP_SEND_LIMIT = 0x69,
				RX_DHCP_NAK = 0x6A,
				DHCP_RETRY_LIMIT = 0x6B,
				RX_MQTT_PATTERN_MATCH = 0x6C,
				RX_MQTT_PUBLISH_WAKE = 0x6D,
				RX_MQTT_MTU_LIMIT_PACKET = 0x6E,
				RX_TCP_FROM_SERVER_TO  = 0x6F,
				FW_BCN_TO_WAKEUP = 0x74,
				RX_TCP_RST_FIN_PKT = 0x75,
				RX_TCP_ALL_PKT = 0x77,
			*************************************** */

			wowlan_wake_reason = rtl8735b_wowlan_wake_reason();
			if (wowlan_wake_reason != 0) {
				printf("\r\nwake fom wlan: 0x%02X\r\n", wowlan_wake_reason);

				extern uint8_t *read_rf_conuter_report(uint8_t log_en);
				read_rf_conuter_report(0);
				extern uint32_t rtl8735b_get_wakeup_bcn_cnt(void);
				bcn_cnt = rtl8735b_get_wakeup_bcn_cnt();
				extern uint64_t rtl8735b_wowlan_get_sleep_duration(void);
				sleep_duration = rtl8735b_wowlan_get_sleep_duration();
				extern uint8_t rtl8735b_wowlan_get_pre_dtim(void);
				pre_dtim = rtl8735b_wowlan_get_pre_dtim();
				if (wowlan_wake_reason == RX_TCP_WITH_PAYLOAD) {
					wlan_mcu_ok = 1;

					uint32_t packet_len = 0;
					uint8_t *wakeup_packet = rtl8735b_read_wakeup_packet(&packet_len, wowlan_wake_reason);

					// parse wakeup packet
					uint8_t *ip_header = NULL;
					uint8_t *tcp_header = NULL;
					uint8_t tcp_header_first4[4];
					tcp_header_first4[0] = (server_port & 0xff00) >> 8;
					tcp_header_first4[1] = (server_port & 0x00ff);
					tcp_header_first4[2] = (retention_local_port & 0xff00) >> 8;
					tcp_header_first4[3] = (retention_local_port & 0x00ff);

					for (int i = 0; i < packet_len - 4; i ++) {
						if ((memcmp(wakeup_packet + i, tcp_header_first4, 4) == 0) && (*(wakeup_packet + i - 20) == 0x45)) {
							ip_header = wakeup_packet + i - 20;
							tcp_header = wakeup_packet + i;
							break;
						}
					}

					if (ip_header && tcp_header) {
						if ((tcp_header[13] == 0x18) || (tcp_header[13] == 0x10)) {
							printf("PUSH + ACK or ACK with payload\n\r");
#if TCP_RESUME
							tcp_resume = 1;

							uint16_t ip_len = (((uint16_t) ip_header[2]) << 8) | ((uint16_t) ip_header[3]);

							uint32_t wakeup_eth_packet_len = 6 + 6 + (ip_len + 2);
							uint8_t *wakeup_eth_packet = (uint8_t *) malloc(wakeup_eth_packet_len);
							if (wakeup_eth_packet) {
								memcpy(wakeup_eth_packet, wakeup_packet + 4, 6);
								memcpy(wakeup_eth_packet + 6, wakeup_packet + 16, 6);
								memcpy(wakeup_eth_packet + 12, ip_header - 2, ip_len + 2);

								extern int lwip_set_resume_packet(uint8_t *packet, uint32_t packet_len);
								lwip_set_resume_packet(wakeup_eth_packet, wakeup_eth_packet_len);

								free(wakeup_eth_packet);
							}
#endif
						}
					}

					free(wakeup_packet);

#if defined(TCP_RESUME_MAX_PACKETS) && (TCP_RESUME_MAX_PACKETS > 1)
					for (int j = 1; j < TCP_RESUME_MAX_PACKETS; j ++) {
						wakeup_packet = rtl8735b_read_packet_with_index(&packet_len, j);
						if (wakeup_packet == NULL) {
							break;
						}

						ip_header = NULL;
						tcp_header = NULL;
						for (int i = 0; i < packet_len - 4; i ++) {
							if ((memcmp(wakeup_packet + i, tcp_header_first4, 4) == 0) && (*(wakeup_packet + i - 20) == 0x45)) {
								ip_header = wakeup_packet + i - 20;
								tcp_header = wakeup_packet + i;
								break;
							}
						}

						if (ip_header && tcp_header) {
							uint16_t ip_len = (((uint16_t) ip_header[2]) << 8) | ((uint16_t) ip_header[3]);

							uint32_t wakeup_eth_packet_len = 6 + 6 + (ip_len + 2);
							uint8_t *wakeup_eth_packet = (uint8_t *) malloc(wakeup_eth_packet_len);
							if (wakeup_eth_packet) {
								memcpy(wakeup_eth_packet, wakeup_packet + 4, 6);
								memcpy(wakeup_eth_packet + 6, wakeup_packet + 16, 6);
								memcpy(wakeup_eth_packet + 12, ip_header - 2, ip_len + 2);

								extern int lwip_set_resume_packet_with_index(uint8_t *packet, uint32_t packet_len, uint8_t index);
								lwip_set_resume_packet_with_index(wakeup_eth_packet, wakeup_eth_packet_len, (uint8_t) j);

								free(wakeup_eth_packet);
							}
						}

						free(wakeup_packet);
					}
#endif
					//} else if (wowlan_wake_reason == 0x70) {
					//	extern uint8_t rtw_hal_read_ch_pno_scan_from_txfifo(uint8_t reason);
					//	channel = rtw_hal_read_ch_pno_scan_from_txfifo(wowlan_wake_reason);
					//	printf("\r\nwake up from pno and camp on ch %d\r\n", channel);
				} else if (wowlan_wake_reason == FW_PNO_TIMEOUT) {
					printf("\r\nwake up from pno and no channel can't be scan\r\n");
				} else if (wowlan_wake_reason == FW_PNO_RECV_BCN_WAKEUP) {
					extern uint8_t rtw_hal_read_ch_pno_scan_from_txfifo(uint8_t reason);
					channel = rtw_hal_read_ch_pno_scan_from_txfifo(wowlan_wake_reason);
					printf("\r\nwake up from pno and camp on ch %d\r\n", channel);
				}  else if (wowlan_wake_reason == FW_PNO_RECV_PROBE_RESP_WAKEUP) {
					extern uint8_t rtw_hal_read_ch_pno_scan_from_txfifo(uint8_t reason);
					channel = rtw_hal_read_ch_pno_scan_from_txfifo(wowlan_wake_reason);
					printf("\r\nwake up from pno by receiving probe response and camp on ch %d\r\n", channel);
				} else if ((wowlan_wake_reason == RX_ICMP_REPLY) || (wowlan_wake_reason == ICMP_MAX_SEND)) {
					wlan_mcu_ok = 1;
				}

				//Enable send deauth before wifi connection
				if (wowlan_wake_reason != 0x77) {
					wifi_set_ap_compatibilty_enabled(0x0f);
				}
			}
		} else if (pm_reason & (BIT(9) | BIT(10) | BIT(11) | BIT(12))) {
			// AON GPIO wake up

			extern int rtw_hal_wowlan_check_wlan_mcu_wakeup(void);
			if (rtw_hal_wowlan_check_wlan_mcu_wakeup() == 1) {
				read_rf_conuter_report(0);
				bcn_cnt = rtl8735b_get_wakeup_bcn_cnt();
				sleep_duration = rtl8735b_wowlan_get_sleep_duration();
				pre_dtim = rtl8735b_wowlan_get_pre_dtim();
				wlan_mcu_ok = 1;
#if TCP_RESUME
				extern uint8_t lwip_check_tcp_resume(void);
				if (lwip_check_tcp_resume() == 1) {
					tcp_resume = 1;
				}
#endif
			} else {
				wlan_mcu_ok = 0;
				printf("\n\rERROR: rtw_hal_wowlan_check_wlan_mcu_wakeup \n\r");
			}
		} else if (pm_reason & BIT(6)) {
			//wakeup by suspend fail
			suspend_fail = rtw_hal_wowlan_get_suspend_fail();

//#if LONG_RUN_TEST
//			while(!rtw_hal_wowlan_check_wlan_mcu_wakeup());
//#endif

			if (rtw_hal_wowlan_check_wlan_mcu_wakeup() == 1) {
				read_rf_conuter_report(0);
				if (!suspend_fail) {
					bcn_cnt = rtl8735b_get_wakeup_bcn_cnt();
					sleep_duration = rtl8735b_wowlan_get_sleep_duration();
				}
				pre_dtim = rtl8735b_wowlan_get_pre_dtim();
			} else {
				printf("\n\rERROR: rtw_hal_wowlan_check_wlan_mcu_wakeup \n\r");
			}

			if (suspend_fail) {
				suspend_wakeup = rtw_hal_wowlan_get_suspend_wakeup_reason();
				printf("\r\nsuspend wakeup reason  0x%x\r\n", suspend_wakeup);
				if (suspend_wakeup == 0x77 || suspend_wakeup == 0) {
					//suspend fail -3/-6 and suspend fail -7 with wakeup reason 0x77
					wlan_mcu_ok = 1;

					if (suspend_wakeup == 0x77) {
						uint32_t packet_len = 0;
						uint8_t *wakeup_packet = rtw_hal_wowlan_get_suspend_wakeup_pattern(&packet_len);

						// parse wakeup packet
						uint8_t *ip_header = NULL;
						uint8_t *tcp_header = NULL;
						uint8_t tcp_header_first4[4];
						tcp_header_first4[0] = (server_port & 0xff00) >> 8;
						tcp_header_first4[1] = (server_port & 0x00ff);
						tcp_header_first4[2] = (retention_local_port & 0xff00) >> 8;
						tcp_header_first4[3] = (retention_local_port & 0x00ff);

						for (int i = 0; i < packet_len - 4; i ++) {
							if ((memcmp(wakeup_packet + i, tcp_header_first4, 4) == 0) && (*(wakeup_packet + i - 20) == 0x45)) {
								ip_header = wakeup_packet + i - 20;
								tcp_header = wakeup_packet + i;
								break;
							}
						}

						if (ip_header && tcp_header) {
							if ((tcp_header[13] == 0x18) || (tcp_header[13] == 0x10)) {
								printf("PUSH + ACK or ACK with payload\n\r");
#if TCP_RESUME
								tcp_resume = 1;

								uint16_t ip_len = (((uint16_t) ip_header[2]) << 8) | ((uint16_t) ip_header[3]);

								uint32_t wakeup_eth_packet_len = 6 + 6 + (ip_len + 2);
								uint8_t *wakeup_eth_packet = (uint8_t *) malloc(wakeup_eth_packet_len);
								if (wakeup_eth_packet) {
									memcpy(wakeup_eth_packet, wakeup_packet + 4, 6);
									memcpy(wakeup_eth_packet + 6, wakeup_packet + 16, 6);
									memcpy(wakeup_eth_packet + 12, ip_header - 2, ip_len + 2);

									extern int lwip_set_resume_packet(uint8_t *packet, uint32_t packet_len);
									lwip_set_resume_packet(wakeup_eth_packet, wakeup_eth_packet_len);

									free(wakeup_eth_packet);
								}
#endif
							}
						}

						free(wakeup_packet);

#if defined(TCP_RESUME_MAX_PACKETS) && (TCP_RESUME_MAX_PACKETS > 1)
						for (int j = 1; j < TCP_RESUME_MAX_PACKETS; j ++) {
							uint8_t *wakeup_packet = rtw_hal_wowlan_get_suspend_wakeup_pattern_with_index(&packet_len, j);
							if (wakeup_packet == NULL) {
								break;
							}

							ip_header = NULL;
							tcp_header = NULL;
							for (int i = 0; i < packet_len - 4; i ++) {
								if ((memcmp(wakeup_packet + i, tcp_header_first4, 4) == 0) && (*(wakeup_packet + i - 20) == 0x45)) {
									ip_header = wakeup_packet + i - 20;
									tcp_header = wakeup_packet + i;
									break;
								}
							}

							if (ip_header && tcp_header) {
								uint16_t ip_len = (((uint16_t) ip_header[2]) << 8) | ((uint16_t) ip_header[3]);

								uint32_t wakeup_eth_packet_len = 6 + 6 + (ip_len + 2);
								uint8_t *wakeup_eth_packet = (uint8_t *) malloc(wakeup_eth_packet_len);
								if (wakeup_eth_packet) {
									memcpy(wakeup_eth_packet, wakeup_packet + 4, 6);
									memcpy(wakeup_eth_packet + 6, wakeup_packet + 16, 6);
									memcpy(wakeup_eth_packet + 12, ip_header - 2, ip_len + 2);

									extern int lwip_set_resume_packet_with_index(uint8_t *packet, uint32_t packet_len, uint8_t index);
									lwip_set_resume_packet_with_index(wakeup_eth_packet, wakeup_eth_packet_len, (uint8_t) j);

									free(wakeup_eth_packet);
								}
							}

							free(wakeup_packet);
						}
					}
#endif
#if TCP_RESUME
					extern uint8_t lwip_check_tcp_resume(void);
					if (lwip_check_tcp_resume() == 1) {
						tcp_resume = 1;
					}
#endif
				}
			}
		}

		if (wlan_mcu_ok && (suspend_fail == 0)) {
			long long current_sec = 0;
			extern void wifi_read_ntp_current_sec(long long * current_sec);
			wifi_read_ntp_current_sec(&current_sec);
			if (current_sec == 0) {
				printf("get NTP Time fail!\r\n");
			} else {
				struct tm *current_tm = localtime(&current_sec);
				current_tm->tm_year += 1900;
				current_tm->tm_mon += 1;
				printf("%d-%d-%d %d:%d:%d UTC\n\r", current_tm->tm_year, current_tm->tm_mon, current_tm->tm_mday, current_tm->tm_hour, current_tm->tm_min, current_tm->tm_sec);
			}
		}

		if (tcp_resume) {
#if TCP_RESUME
			// set tcp resume port to drop packet before tcp resume done
			// must set before lwip init
			extern void tcp_set_resume_port(uint16_t port);
			tcp_set_resume_port(retention_local_port);

			extern void lwip_recover_resume_keepalive(void);
			lwip_recover_resume_keepalive();
#if SSL_KEEPALIVE
			ssl_resume = 1;
#endif
#endif
		}

		if (wlan_mcu_ok && (rtw_hal_wlan_resume_check() == 1)) {
			wlan_resume = 1;

			rtw_hal_read_aoac_rpt_from_txfifo(NULL, 0, 0);

			uint16_t dhcp_t1_time = rtw_hal_read_wowlan_t1_time();
			extern uint8_t lwip_set_dhcp_resume_t1(uint16_t t1_time);
			lwip_set_dhcp_resume_t1(dhcp_t1_time);
		}
	}

	console_init();
	log_service_add_table(at_power_save_items, sizeof(at_power_save_items) / sizeof(at_power_save_items[0]));

	if (wlan_resume) {
		p_wifi_do_fast_connect = wlan_do_resume;
		p_store_fast_connect_info = NULL;
	} else {
#if TEST_DISCONNECT_PNO
		wifi_fast_connect_enable(0);
#else
		if (wowlan_wake_reason == 0x71) {
			//for pno wakeup without channel
			wifi_fast_connect_enable(0);
			disconnect_pno = 1;
		} else if ((wowlan_wake_reason == 0x72) || (wowlan_wake_reason == 0x73)) {
			//for pno wakeup with channel
			//if wakeup from another band scan, please reconnect to the AP, do not use fast connect
			extern void wifi_set_pno_reconnect_channel(uint8_t channel);
			wifi_set_pno_reconnect_channel(channel);
			wifi_fast_connect_enable(1);
		} else {
			wifi_fast_connect_enable(1);
		}
#endif
	}

	wlan_network();

	if (xTaskCreate(tcp_app_task, ((const char *)"tcp_app_task"), STACKSIZE, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(tcp_app_task) failed\n", __FUNCTION__);
	}

	vTaskStartScheduler();
	while (1);
}
