#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "main.h"
#include "log_service.h"
#include "osdep_service.h"
#include <platform_opts.h>
#include <platform_opts_bt.h>
#include "sys_api.h"

#include "wifi_conf.h"
#include <lwip_netconf.h>
#include <lwip/sockets.h>

#include "power_mode_api.h"

#define STACKSIZE     2048
#define TCP_RESUME    1
#define SSL_KEEPALIVE 1

extern uint8_t rtl8735b_wowlan_wake_reason(void);
extern uint8_t rtl8735b_wowlan_wake_pattern(void);
extern uint8_t *rtl8735b_read_wakeup_packet(uint32_t *size, uint8_t wowlan_reason);
extern uint8_t *rtl8735b_read_ssl_conuter_report(void);
extern int rtl8735b_suspend(int mode);
extern void rtl8735b_set_lps_pg(void);
extern void wifi_set_tcpssl_keepalive(void);
extern int wifi_set_ssl_offload(uint8_t *ctr, uint8_t *iv, uint8_t *enc_key, uint8_t *dec_key, uint8_t *hmac_key, uint8_t *content, size_t len, uint8_t is_etm);
extern void wifi_set_ssl_counter_report(void);

static uint8_t wowlan_wake_reason = 0;
static uint8_t wlan_resume = 0;
static uint8_t tcp_resume = 0;
static uint8_t ssl_resume = 0;

static char server_ip[16] = "192.168.31.115";
static uint16_t server_port = 5566;
__attribute__((section(".retention.data"))) uint16_t retention_local_port __attribute__((aligned(32))) = 0;
static uint32_t interval_ms = 30000;
static uint32_t resend_ms = 10000;

static uint8_t goto_sleep = 0;

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

void set_tcp_connected_pattern(wowlan_pattern_t *pattern)
{
	// This pattern make STA can be wake from a connected TCP socket
	memset(pattern, 0, sizeof(wowlan_pattern_t));

	char buf[32];
	char mac[6];
	char ip_protocol[2] = {0x08, 0x00}; // IP {08,00} ARP {08,06}
	char ip_ver[1] = {0x45};
	char tcp_protocol[1] = {0x06}; // 0x06 for tcp
	char tcp_port[2] = {(server_port >> 8) & 0xFF, server_port & 0xFF};
	char flag2[1] = {0x18}; // PSH + ACK
	uint8_t *ip = LwIP_GetIP(0);
	const uint8_t data_mask[6] = {0x3f, 0x70, 0x80, 0xc0, 0x0F, 0x80};
	const uint8_t *mac_temp = LwIP_GetMAC(0);

	//wifi_get_mac_address(buf);
	//sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	memcpy(mac, mac_temp, 6);
	printf("mac = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X \r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	memcpy(pattern->eth_da, mac, 6);
	memcpy(pattern->eth_proto_type, ip_protocol, 2);
	memcpy(pattern->header_len, ip_ver, 1);
	memcpy(pattern->ip_proto, tcp_protocol, 1);
	memcpy(pattern->ip_da, ip, 4);
	memcpy(pattern->src_port, tcp_port, 2);
	memcpy(pattern->flag2, flag2, 1);
	memcpy(pattern->mask, data_mask, 6);

	//payload
	// uint8_t data[10] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
	// uint8_t payload_mask[9] = {0xc0, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	// memcpy(pattern->payload, data, 10);
	// memcpy(pattern->payload_mask, payload_mask, 9);

}

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
#define SSL_OFFLOAD_KEY_LEN 32 // aes256
#define SSL_OFFLOAD_MAC_LEN 48 // sha384
static uint8_t ssl_offload_ctr[8];
static uint8_t ssl_offload_enc_key[SSL_OFFLOAD_KEY_LEN];
static uint8_t ssl_offload_dec_key[SSL_OFFLOAD_KEY_LEN];
static uint8_t ssl_offload_hmac_key[SSL_OFFLOAD_MAC_LEN];
static uint8_t ssl_offload_iv[16];
static uint8_t ssl_offload_is_etm = 0;
uint8_t keepalive_content[] = {'_', 'A', 'l', 'i', 'v', 'e'};
size_t keepalive_len = sizeof(keepalive_content);

int set_ssl_offload(mbedtls_ssl_context *ssl, uint8_t *iv, uint8_t *content, size_t len)
{
	if (ssl->transform_out->cipher_ctx_enc.cipher_info->type != MBEDTLS_CIPHER_AES_256_CBC) {
		printf("ERROR: not AES256CBC\n\r");
		return -1;
	}

	if (ssl->transform_out->md_ctx_enc.md_info->type != MBEDTLS_MD_SHA384) {
		printf("ERROR: not SHA384\n\r");
		return -1;
	}

	// counter
#if (MBEDTLS_VERSION_NUMBER >= 0x03000000) || (MBEDTLS_VERSION_NUMBER == 0x02100600) || (MBEDTLS_VERSION_NUMBER == 0x021C0100)
	memcpy(ssl_offload_ctr, ssl->cur_out_ctr, 8);
#else
	memcpy(ssl_offload_ctr, ssl->out_ctr, 8);
#endif

	// aes enc key
	mbedtls_aes_context *enc_ctx = (mbedtls_aes_context *) ssl->transform_out->cipher_ctx_enc.cipher_ctx;

#if ((MBEDTLS_VERSION_NUMBER >= 0x03000000) || (MBEDTLS_VERSION_NUMBER == 0x02100600) || (MBEDTLS_VERSION_NUMBER == 0x021C0100)) && defined(MBEDTLS_AES_ALT)
	memcpy(ssl_offload_enc_key, enc_ctx->rk, SSL_OFFLOAD_KEY_LEN);
#elif (MBEDTLS_VERSION_NUMBER == 0x02040000)
	memcpy(ssl_offload_enc_key, enc_ctx->enc_key, SSL_OFFLOAD_KEY_LEN);
#else
#error "invalid mbedtls_aes_context for ssl offload"
#endif

	// aes dec key
	mbedtls_aes_context *dec_ctx = (mbedtls_aes_context *) ssl->transform_out->cipher_ctx_dec.cipher_ctx;
#if ((MBEDTLS_VERSION_NUMBER >= 0x03000000) || (MBEDTLS_VERSION_NUMBER == 0x02100600) || (MBEDTLS_VERSION_NUMBER == 0x021C0100)) && defined(MBEDTLS_AES_ALT)
	memcpy(ssl_offload_dec_key, dec_ctx->rk, SSL_OFFLOAD_KEY_LEN);
#elif (MBEDTLS_VERSION_NUMBER == 0x02040000)
	memcpy(ssl_offload_dec_key, dec_ctx->dec_key, SSL_OFFLOAD_KEY_LEN);
#else
#error "invalid mbedtls_aes_context for ssl offload"
#endif

	// hmac key
	uint8_t *hmac_ctx = (uint8_t *) ssl->transform_out->md_ctx_enc.hmac_ctx;
	for (int i = 0; i < SSL_OFFLOAD_MAC_LEN; i ++) {
		ssl_offload_hmac_key[i] = hmac_ctx[i] ^ 0x36;
	}

	// aes iv
	memcpy(ssl_offload_iv, iv, 16);

	// encrypt-then-mac
	if (ssl->session->encrypt_then_mac == MBEDTLS_SSL_ETM_ENABLED) {
		ssl_offload_is_etm = 1;
	}

	printf("session ssl_offload_is_etm = %d\r\n", ssl_offload_is_etm);

	wifi_set_ssl_offload(ssl_offload_ctr, ssl_offload_iv, ssl_offload_enc_key, ssl_offload_dec_key, ssl_offload_hmac_key, content, len, ssl_offload_is_etm);
	return 0;
}

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

_WEAK void ssl_connect_start_time(void)
{

}
_WEAK void ssl_connect_end_time(void)
{

}

void tcp_app_task(void *param)
{
	int ret;
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
	ssl_connect_start_time();

	// socket
	sock_fd = socket(AF_INET, SOCK_STREAM, 0);
	printf("\n\r socket(%d) \n\r", sock_fd);

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
			ssl_connect_end_time();
		} else {
			printf("connect to %s:%d FAIL \n\r", server_ip, server_port);
			close(sock_fd);
			goto exit1;
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
		ssl_connect_end_time();
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
#if SSL_KEEPALIVE
		ret = mbedtls_ssl_write(&ssl, (uint8_t *) "\n_APP", strlen("\n_APP"));
#else
		ret = write(sock_fd, "_APP", strlen("_APP"));
#endif
		printf("write application data %d \n\r", ret);

		if (ret < 0) {
#if SSL_KEEPALIVE
			mbedtls_ssl_close_notify(&ssl);
#endif
			goto exit;
		}

		vTaskDelay(5000);
	}

	// arp retain before wifi_set_tcp_keep_alive_offload() since arp table is clean after netif_set_link_down()
	extern int arp_retain(void);
	printf("retain ARP %s \n\r", arp_retain() == 0 ? "OK" : "FAIL");

	// set keepalive
#if SSL_KEEPALIVE
	uint8_t iv[16];
	memset(iv, 0xab, sizeof(iv));
	set_ssl_offload(&ssl, iv, keepalive_content, keepalive_len);

	uint8_t *ssl_record = NULL;
	uint8_t ssl_record_header[] = {/*type*/ 0x17 /*type*/, /*version*/ 0x03, 0x03 /*version*/, /*length*/ 0x00, 0x00 /*length*/};

	if (ssl_offload_is_etm) {
		// application data
		size_t ssl_data_len = (keepalive_len + 15) / 16 * 16;
		uint8_t *ssl_data = (uint8_t *) malloc(ssl_data_len);
		memcpy(ssl_data, keepalive_content, keepalive_len);
		size_t padlen = 16 - (keepalive_len + 1) % 16;
		if (padlen == 16) {
			padlen = 0;
		}
		for (int i = 0; i <= padlen; i++) {
			ssl_data[keepalive_len + i] = (uint8_t) padlen;
		}
		// length
		size_t out_length = 16 /*iv*/ + ssl_data_len + SSL_OFFLOAD_MAC_LEN;
		ssl_record_header[3] = (uint8_t)((out_length >> 8) & 0xff);
		ssl_record_header[4] = (uint8_t)(out_length & 0xff);
		// enc
		mbedtls_aes_context enc_ctx;
		mbedtls_aes_init(&enc_ctx);
		mbedtls_aes_setkey_enc(&enc_ctx, ssl_offload_enc_key, SSL_OFFLOAD_KEY_LEN * 8);
		uint8_t iv[16];
		memcpy(iv, ssl_offload_iv, 16); // must copy iv because mbedtls_aes_crypt_cbc() will modify iv
		mbedtls_aes_crypt_cbc(&enc_ctx, MBEDTLS_AES_ENCRYPT, ssl_data_len, iv, ssl_data, ssl_data);
		mbedtls_aes_free(&enc_ctx);
		// mac
		uint8_t pseudo_header[13];
		memcpy(pseudo_header, ssl_offload_ctr, 8);  // counter
		memcpy(pseudo_header + 8, ssl_record_header, 3); // type+version
		pseudo_header[11] = (uint8_t)(((16 /*iv*/ + ssl_data_len) >> 8) & 0xff);
		pseudo_header[12] = (uint8_t)((16 /*iv*/ + ssl_data_len) & 0xff);
		mbedtls_md_context_t md_ctx;
		mbedtls_md_init(&md_ctx);
		mbedtls_md_setup(&md_ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA384), 1);
		mbedtls_md_hmac_starts(&md_ctx, ssl_offload_hmac_key, SSL_OFFLOAD_MAC_LEN);
		mbedtls_md_hmac_update(&md_ctx, pseudo_header, 13);
		mbedtls_md_hmac_update(&md_ctx, ssl_offload_iv, 16);
		mbedtls_md_hmac_update(&md_ctx, ssl_data, ssl_data_len);
		uint8_t hmac[SSL_OFFLOAD_MAC_LEN];
		mbedtls_md_hmac_finish(&md_ctx, hmac);
		mbedtls_md_free(&md_ctx);
		// ssl record
		size_t ssl_record_len = sizeof(ssl_record_header) + 16 /* iv */ + ssl_data_len + SSL_OFFLOAD_MAC_LEN;
		ssl_record = (uint8_t *) malloc(ssl_record_len);
		memset(ssl_record, 0, ssl_record_len);
		memcpy(ssl_record, ssl_record_header, sizeof(ssl_record_header));
		memcpy(ssl_record + sizeof(ssl_record_header), ssl_offload_iv, 16);
		memcpy(ssl_record + sizeof(ssl_record_header) + 16, ssl_data, ssl_data_len);
		memcpy(ssl_record + sizeof(ssl_record_header) + 16 + ssl_data_len, hmac, SSL_OFFLOAD_MAC_LEN);
		free(ssl_data);
		// replace content
		//len = ssl_record_len;
		//printf("ssl_record_len = %d\r\n", ssl_record_len);
		wifi_set_tcp_keep_alive_offload(sock_fd, ssl_record, ssl_record_len, interval_ms, resend_ms, 1);
		// free ssl_record after content is not used anymore
		if (ssl_record) {
			free(ssl_record);
		}
	} else {
		// mac
		uint8_t mac_out_len[2];
		mac_out_len[0] = (uint8_t)((keepalive_len >> 8) & 0xff);
		mac_out_len[1] = (uint8_t)(keepalive_len & 0xff);
		mbedtls_md_context_t md_ctx;
		mbedtls_md_init(&md_ctx);
		mbedtls_md_setup(&md_ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA384), 1);
		mbedtls_md_hmac_starts(&md_ctx, ssl_offload_hmac_key, SSL_OFFLOAD_MAC_LEN);
		mbedtls_md_hmac_update(&md_ctx, ssl_offload_ctr, 8);
		mbedtls_md_hmac_update(&md_ctx, ssl_record_header, 3);
		mbedtls_md_hmac_update(&md_ctx, mac_out_len, 2);
		mbedtls_md_hmac_update(&md_ctx, keepalive_content, keepalive_len);
		uint8_t hmac[SSL_OFFLOAD_MAC_LEN];
		mbedtls_md_hmac_finish(&md_ctx, hmac);
		mbedtls_md_free(&md_ctx);
		// application data with mac
		size_t ssl_data_len = (keepalive_len + SSL_OFFLOAD_MAC_LEN + 15) / 16 * 16;
		uint8_t *ssl_data = (uint8_t *) malloc(ssl_data_len);
		memcpy(ssl_data, keepalive_content, keepalive_len);
		memcpy(ssl_data + keepalive_len, hmac, SSL_OFFLOAD_MAC_LEN);
		size_t padlen = 16 - (keepalive_len + SSL_OFFLOAD_MAC_LEN + 1) % 16;
		if (padlen == 16) {
			padlen = 0;
		}
		for (int i = 0; i <= padlen; i++) {
			ssl_data[keepalive_len + SSL_OFFLOAD_MAC_LEN + i] = (uint8_t) padlen;
		}
		// enc
		mbedtls_aes_context enc_ctx;
		mbedtls_aes_init(&enc_ctx);
		mbedtls_aes_setkey_enc(&enc_ctx, ssl_offload_enc_key, SSL_OFFLOAD_KEY_LEN * 8);
		uint8_t iv[16];
		memcpy(iv, ssl_offload_iv, 16); // must copy iv because mbedtls_aes_crypt_cbc() will modify iv
		mbedtls_aes_crypt_cbc(&enc_ctx, MBEDTLS_AES_ENCRYPT, ssl_data_len, iv, ssl_data, ssl_data);
		mbedtls_aes_free(&enc_ctx);
		// length
		size_t out_length = 16 /*iv*/ + ssl_data_len;
		ssl_record_header[3] = (uint8_t)((out_length >> 8) & 0xff);
		ssl_record_header[4] = (uint8_t)(out_length & 0xff);
		// ssl record
		size_t ssl_record_len = sizeof(ssl_record_header) + 16 /* iv */ + ssl_data_len;
		ssl_record = (uint8_t *) malloc(ssl_record_len);
		memset(ssl_record, 0, ssl_record_len);
		memcpy(ssl_record, ssl_record_header, sizeof(ssl_record_header));
		memcpy(ssl_record + sizeof(ssl_record_header), ssl_offload_iv, 16);
		memcpy(ssl_record + sizeof(ssl_record_header) + 16, ssl_data, ssl_data_len);
		free(ssl_data);
		// replace content
		//len = ssl_record_len;
		//printf("ssl_record_len = %d\r\n", ssl_record_len);
		wifi_set_tcp_keep_alive_offload(sock_fd, ssl_record, ssl_record_len, interval_ms, resend_ms, 1);

		// free ssl_record after content is not used anymore
		if (ssl_record) {
			free(ssl_record);
		}
	}

	wifi_set_tcpssl_keepalive();
	wifi_set_ssl_counter_report();
#else
	wifi_set_tcp_keep_alive_offload(sock_fd, keepalive_content, keepalive_len, interval_ms, resend_ms, 1);
#endif

#if SSL_KEEPALIVE
	// retain ssl
	extern int mbedtls_ssl_retain(mbedtls_ssl_context * ssl);
	printf("retain SSL %s \n\r", mbedtls_ssl_retain(&ssl) == 0 ? "OK" : "FAIL");
#endif
#if TCP_RESUME
	// retain tcp pcb
	extern int lwip_retain_tcp(int s);
	printf("retain TCP pcb %s \n\r", lwip_retain_tcp(sock_fd) == 0 ? "OK" : "FAIL");
#endif

	wifi_set_dhcp_offload(); // after wifi_set_tcp_keep_alive_offload
	wifi_wowlan_set_arpreq_keepalive(1, 0);

	// set wakeup pattern
	wowlan_pattern_t data_pattern;
	set_tcp_connected_pattern(&data_pattern);
	wifi_wowlan_set_pattern(data_pattern);

	extern int dhcp_retain(void);
	printf("retain DHCP %s \n\r", dhcp_retain() == 0 ? "OK" : "FAIL");

	// for wlan resume
	extern int rtw_hal_wlan_resume_backup(void);
	rtw_hal_wlan_resume_backup();

	// sleep
	rtl8735b_set_lps_pg();
	rtw_enter_critical(NULL, NULL);
	if (rtl8735b_suspend(0) == 0) { // should stop wifi application before doing rtl8735b_suspend(
		printf("rtl8735b_suspend\r\n");

		// wakeup GPIO
		gpio_irq_init(&my_GPIO_IRQ, WAKUPE_GPIO_PIN, gpio_demo_irq_handler, (uint32_t)&my_GPIO_IRQ);
		gpio_irq_pull_ctrl(&my_GPIO_IRQ, PullDown);
		gpio_irq_set(&my_GPIO_IRQ, IRQ_RISE, 1);

		// standby with retention: add SLP_GTIMER and set SramOption to retention mode(1)
		Standby(DSTBY_AON_GPIO | DSTBY_WLAN | SLP_GTIMER, 0, 0, 1);
	} else {
		printf("rtl8735b_suspend fail\r\n");
		sys_reset();
	}
	rtw_exit_critical(NULL, NULL);

	while (1) {
		vTaskDelay(2000);
		printf("sleep fail!!!\r\n");
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
	extern int rtw_hal_wlan_resume_restore(void);
	rtw_hal_wlan_resume_restore();

	wifi_fast_connect_enable(1);
	wifi_fast_connect_load_fast_dhcp();

	extern uint8_t lwip_check_arp_resume(void);
	if (lwip_check_arp_resume() == 1) {
		extern int arp_resume(void);
		printf("\n\rresume ARP %s\n\r", arp_resume() == 0 ? "OK" : "FAIL");
	}

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
	}
}

log_item_t at_power_save_items[ ] = {
	{"PS", fPS,},
};


void wlan_tcp_resume(void) 
{
	hal_xtal_divider_enable(1);
	hal_32k_s1_sel(2);
	HAL_WRITE32(0x40009000, 0x18, 0x1 | HAL_READ32(0x40009000, 0x18)); //SWR 1.35V

	uint32_t pm_reason = Get_wake_reason();
	printf("\n\rpm_reason=0x%x\n\r", pm_reason);

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
				RX_TCP_RST_FIN_PKT = 0x75,
			*************************************** */

			wowlan_wake_reason = rtl8735b_wowlan_wake_reason();
			if (wowlan_wake_reason != 0) {
				printf("\r\nwake fom wlan: 0x%02X\r\n", wowlan_wake_reason);
				if (wowlan_wake_reason == 0x6C || wowlan_wake_reason == 0x6D) {
					uint8_t wowlan_wakeup_pattern = rtl8735b_wowlan_wake_pattern();
					printf("\r\nwake fom wlan pattern index: 0x%02X\r\n", wowlan_wakeup_pattern);
				}

				if (wowlan_wake_reason == 0x23 || wowlan_wake_reason == 0x6C || wowlan_wake_reason == 0x6D) {
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
						if (tcp_header[13] == 0x18) {
							printf("PUSH + ACK\n\r");
#if TCP_RESUME
							tcp_resume = 1;

							uint16_t ip_len = (((uint16_t) ip_header[2]) << 8) | ((uint16_t) ip_header[3]);
							uint16_t tcp_payload_len = ip_len - 20 - 20;
							uint32_t wakeup_seqno = (((uint32_t) tcp_header[4]) << 24) | (((uint32_t) tcp_header[5]) << 16) |
													(((uint32_t) tcp_header[6]) << 8) | ((uint32_t) tcp_header[7]);
							uint32_t wakeup_ackno = (((uint32_t) tcp_header[8]) << 24) | (((uint32_t) tcp_header[9]) << 16) |
													(((uint32_t) tcp_header[10]) << 8) | ((uint32_t) tcp_header[11]);
							printf("tcp_payload_len=%d, wakeup_seqno=%u, wakeup_ackno=%u \n\r", tcp_payload_len, wakeup_seqno, wakeup_ackno);

							tcp_resume_seqno = wakeup_ackno;
							tcp_resume_ackno = wakeup_seqno + tcp_payload_len;
							printf("tcp_resume_seqno=%u, tcp_resume_ackno=%u \n\r", tcp_resume_seqno, tcp_resume_ackno);
#endif
						} else if (tcp_header[13] == 0x11) {
							printf("FIN + ACK\n\r");

							// not resume because TCP connection is closed
						}
					}

					free(wakeup_packet);
				}
			}
		} else if (pm_reason & (BIT(9) | BIT(10) | BIT(11) | BIT(12))) {
			// AON GPIO wake up

			extern int rtw_hal_wowlan_check_wlan_mcu_wakeup(void);
			if (rtw_hal_wowlan_check_wlan_mcu_wakeup() == 1) {
				wlan_mcu_ok = 1;
#if TCP_RESUME
				extern uint8_t lwip_check_tcp_resume(void);
				if (lwip_check_tcp_resume() == 1) {
					tcp_resume = 1;

					tcp_resume_seqno = *((uint32_t *)(rtl8735b_read_ssl_conuter_report() + 16));
					tcp_resume_ackno = *((uint32_t *)(rtl8735b_read_ssl_conuter_report() + 20));
					printf("\ntcp_resume_seqno=%u, tcp_resume_ackno=%u \n\r", tcp_resume_seqno, tcp_resume_ackno);
				}
#endif
			} else {
				wlan_mcu_ok = 0;
				printf("\n\rERROR: rtw_hal_wowlan_check_wlan_mcu_wakeup \n\r");
			}
		}

		if (tcp_resume) {
#if TCP_RESUME
			extern int lwip_set_tcp_resume(uint32_t seqno, uint32_t ackno);
			lwip_set_tcp_resume(tcp_resume_seqno, tcp_resume_ackno);

			// set tcp resume port to drop packet before tcp resume done
			// must set before lwip init
			extern void tcp_set_resume_port(uint16_t port);
			tcp_set_resume_port(retention_local_port);
#if SSL_KEEPALIVE
			ssl_resume = 1;

			uint8_t *ssl_counter = rtl8735b_read_ssl_conuter_report();
			uint8_t ssl_resume_out_ctr[8];
			uint8_t ssl_resume_in_ctr[8];
			memcpy(ssl_resume_out_ctr, ssl_counter, 8);
			memcpy(ssl_resume_in_ctr, ssl_counter + 8, 8);

			printf("ssl_resume_out_ctr = ");
			for (int i = 0; i < 8; i ++) {
				printf("%02X", ssl_resume_out_ctr[i]);
			}
			printf("\r\nssl_resume_in_ctr = ");
			for (int i = 0; i < 8; i ++) {
				printf("%02X", ssl_resume_in_ctr[i]);
			}

			extern int mbedtls_set_ssl_resume(uint8_t in_ctr[8], uint8_t out_ctr[8], uint8_t by_wlan);
			mbedtls_set_ssl_resume(ssl_resume_in_ctr, ssl_resume_out_ctr, by_wlan);
			//extern void mbedtls_set_ssl_resume_fix_ctr(uint8_t fix_ctr, uint8_t max_try);
			//mbedtls_set_ssl_resume_fix_ctr(1, 2);
#endif
#endif
		}

		extern int rtw_hal_wlan_resume_check(void);
		if (wlan_mcu_ok && (rtw_hal_wlan_resume_check() == 1)) {
			wlan_resume = 1;

			extern int rtw_hal_read_aoac_rpt_from_txfifo(u8 * buf, u16 addr, u16 len);
			rtw_hal_read_aoac_rpt_from_txfifo(NULL, 0, 0);
		}
	}

	//console_init();
	log_service_add_table(at_power_save_items, sizeof(at_power_save_items) / sizeof(at_power_save_items[0]));

	if (wlan_resume) {
		p_wifi_do_fast_connect = wlan_do_resume;
		p_store_fast_connect_info = NULL;
	} else {
		wifi_fast_connect_enable(1);
	}

	wlan_network();

	if (xTaskCreate(tcp_app_task, ((const char *)"tcp_app_task"), STACKSIZE, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(tcp_app_task) failed\n", __FUNCTION__);
	}
}
