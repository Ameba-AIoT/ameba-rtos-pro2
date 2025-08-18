#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "main.h"
#include "log_service.h"
#include "osdep_service.h"
#include <platform_opts.h>
#include <platform_opts_bt.h>
#include "sys_api.h"

#if CONFIG_WLAN
#include <wifi_fast_connect.h>
extern void wlan_network(void);
#endif

#include "wowlan_driver_api.h"
#include "wifi_conf.h"
#include <lwip_netconf.h>
#include <lwip/sockets.h>
#include <lwip/netif.h>

#include "hal_power_mode.h"
#include "power_mode_api.h"

#define SSL_KEEPALIVE	1
#define TCP_RESUME		1
// ignore mqtt header check
#define TCPSSL_MODE       0
#define TCP_SERVER_KEEP_ALIVE 0
#define AWS_IOT_MQTT      0
#include "MQTTClient.h"

#include "gpio_api.h"
#include "gpio_irq_api.h"
#include "gpio_irq_ex_api.h"

#define WAKUPE_GPIO_PIN PA_2
#define WOWLAN_MQTT_BCNV2	2

static uint8_t wowlan_wake_reason = 0;
static uint8_t wlan_resume = 0;
static uint8_t tcp_resume = 0;
static uint8_t ssl_resume = 0;
static uint32_t packet_len = 0;
static Network network = {0};

__attribute__((section(".retention.data"))) uint16_t retention_local_port __attribute__((aligned(32))) = 0;
static gpio_irq_t my_GPIO_IRQ;
void gpio_demo_irq_handler(uint32_t id, gpio_irq_event event)
{

	dbg_printf("%s==> \r\n", __FUNCTION__);

}

extern void console_init(void);

extern struct netif xnetif[NET_IF_NUM];
/**
 * Lunch a thread to send AT command automatically for a long run test
 */
#define LONG_RUN_TEST   (1)
//Clock, 1: 4MHz, 0: 100kHz
#define CLOCK 0
//SLEEP_DURATION, 120s
#define SLEEP_DURATION (120 * 1000 * 1000)

static char your_ssid[33]   = "your_ssid";
static char your_pw[33]     = "your_pw";
static rtw_security_t sec   = RTW_SECURITY_WPA2_AES_PSK;

static TaskHandle_t wowlan_thread_handle = NULL;

static char server_ip[16] = "192.168.50.197";
static uint16_t server_port = 8883;
static int enable_tcp_keep_alive = 0;
static uint32_t interval_ms = 180000;
static uint32_t resend_ms = 10000;

static int enable_wowlan_pattern = 0;

/*add for arlo*/
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

static uint8_t set_dtimtimeout = 1;
static uint8_t set_rxbcnlimit = 8;
static uint8_t set_pstimeout = 16;
static uint8_t set_psretry = 7;
static uint8_t set_arpreqperiod = 50;
static uint8_t set_arpreqenable = 1;
static uint8_t set_tcpkaenable = 1;
static uint8_t set_pnoenable = 1;
/*add for arlo*/

#define WOWLAN_GPIO_WDT      0

void print_PS_help(void)
{
	printf("PS=[wowlan|tcp_keep_alive],[options]\r\n");

	printf("\r\n");
	printf("PS=wowlan\r\n");
	printf("\tenter wake on wlan mode\r\n");

	printf("\r\n");
	printf("PS=tcp_keep_alive\r\n");
	printf("\tEnable TCP KEEP ALIVE with hardcode server IP & PORT. This setting only works in wake on wlan mode\r\n");
	printf("PS=tcp_keep_alive,192.168.1.100,5566\r\n");
	printf("\tEnable TCP KEEP ALIVE with server IP 192.168.1.100 and port 5566. This setting only works in wake on wlan mode\r\n");

}

static unsigned char sendbuf[2048], readbuf[2048];
static int ciphersuites[] = {MBEDTLS_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA384, 0};

static void messageArrived(MessageData *data)
{
	mqtt_printf(MQTT_INFO, "Message arrived on topic %.*s: %.*s\n", data->topicName->lenstring.len, data->topicName->lenstring.data,
				data->message->payloadlen, (char *)data->message->payload);
}

#include "mbedtls/version.h"
#include "mbedtls/config.h"
#include "mbedtls/ssl.h"
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
static uint8_t keepalive_content[] = {0xc0, 0x00};
static size_t keepalive_len = 2;

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

	// printf("session ssl_offload_is_etm = %d\r\n", ssl_offload_is_etm);

	wifi_set_ssl_offload(ssl_offload_ctr, ssl_offload_iv, ssl_offload_enc_key, ssl_offload_dec_key, ssl_offload_hmac_key, content, len, ssl_offload_is_etm);
	return 0;
}


static const char *test_ca_cert =
	"-----BEGIN CERTIFICATE-----\r\n"
	"MIIDWjCCAkKgAwIBAgIVAIDLSSoG+EARSbBprT4Im1uu8j2vMA0GCSqGSIb3DQEB\r\n"
	"CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t\r\n"
	"IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yMDA1MjkwNTM0\r\n"
	"MjVaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh\r\n"
	"dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDCin3pY25wZ5sBEWMP\r\n"
	"w74IuypwE2f8AfKERyjMtkLyqgQJQ6dLH9JouS8POFhwcC8MQ42Q4F3Z6KhiJLUL\r\n"
	"5+5J0GGPkuslMjFHf0CcO+XGXGoKDi0HcxbOiWMvdGwJGYSTE9A2knXi2bMY42w3\r\n"
	"8qpyplm2kpnMT4ijpMac0J1q2uxEsAgNrJdqLeRRfEkeavoqfyhlCJI1TBnFmkWk\r\n"
	"wa0cK+4N2+CqTXe+d8t2ET0zxiDna2Qv6JV1R17DL/4RCaCVdclNcIIzav4QMpT3\r\n"
	"AS1+npk4YPpJCZ8gpPqPPLavjD8DIhx0MMX4Wur1MkWyYaxRpTR1vV2tm9kAv7MS\r\n"
	"lLqfAgMBAAGjYDBeMB8GA1UdIwQYMBaAFJYunYJinS8UT8aIkYVz5d6ew53vMB0G\r\n"
	"A1UdDgQWBBQzwsPy8o8pSYrk2Maiy/cjKoe0GDAMBgNVHRMBAf8EAjAAMA4GA1Ud\r\n"
	"DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAHXtltkye2nDNXjhMb1eAcUs/\r\n"
	"LCzNtQTsXiVTLBB1/h7uZ3cbyqdfr6eAgmBcAofnxUfV4lRxpp0DRUeZ4pwLcKAP\r\n"
	"Kt0HcXXv4vKZlk58OgrC1rAgLL88MPq1tAWFA+2MYEKtAcr1WcoHEt5+3FrUwfLU\r\n"
	"sn0cw9aYPICxFm+YECPOpdpKIoFkQrSKYfrwY/tdcW2PNw83a/XLxIu0yc7Rp5us\r\n"
	"xEcC3+Ge05A9Y7/o07q86rMqxAAgUtxzeI589tFolabHViNYHSpRIPeLwqhXdRa0\r\n"
	"k5+NsBroU/YdvOUmzKn6XfI4nX4hLQJ2TbhAT8aq1ounGk6ZGqCbxt4mg5bB0w==\r\n"
	"-----END CERTIFICATE-----\r\n"	;

static const char *test_private_key =
	"-----BEGIN RSA PRIVATE KEY-----\r\n"
	"MIIEpAIBAAKCAQEAwop96WNucGebARFjD8O+CLsqcBNn/AHyhEcozLZC8qoECUOn\r\n"
	"Sx/SaLkvDzhYcHAvDEONkOBd2eioYiS1C+fuSdBhj5LrJTIxR39AnDvlxlxqCg4t\r\n"
	"B3MWzoljL3RsCRmEkxPQNpJ14tmzGONsN/KqcqZZtpKZzE+Io6TGnNCdatrsRLAI\r\n"
	"DayXai3kUXxJHmr6Kn8oZQiSNUwZxZpFpMGtHCvuDdvgqk13vnfLdhE9M8Yg52tk\r\n"
	"L+iVdUdewy/+EQmglXXJTXCCM2r+EDKU9wEtfp6ZOGD6SQmfIKT6jzy2r4w/AyIc\r\n"
	"dDDF+Frq9TJFsmGsUaU0db1drZvZAL+zEpS6nwIDAQABAoIBADWWusqAplp2X79y\r\n"
	"j6w3CnETRcRrxBgqXSjNBVMm3dhEtynqJfpOwMIySOFTbyFB9ePV8/g1pgSxzziB\r\n"
	"zhGCiSRyL33CRd4QLnz4c87VvRzgNiGg+Ax2SpEITXc0BdKX4eo16gQuYiTkPS6c\r\n"
	"7yGWShec9VeSmKUsP4J8kG2AFezp+HUygssFhD2HSB70oFIYVH894x1G25ZQJ6qB\r\n"
	"4OVaq+23RIrBtv8tM93l22AjmwYmAR+9d4lQdvGx635wV1oEwrXzFlRV6U9xpANm\r\n"
	"eEDUXGIzbA7J8zG8VeKRXsSzUf7VqMoolGERUv5gssbar/Zp/MDIFJ/Onr1VSGt6\r\n"
	"A01ZetECgYEA41hE4xUoIh0FVJjdjGUgGd2GUf/pHkmcfBZdEzOxjVyyyP3FhhXd\r\n"
	"QjmYVgAaB1gTFlxTl/KszCO9f1fy1lDJ4SqHhmHwLNM4dzdGyVHd9fBly3YunlgI\r\n"
	"WaKPJBAKCeS2vVJYP8Tt7xvyDC68Y+3QhHx/jkl2nWS8iZpHza7Bc1kCgYEA2w++\r\n"
	"FgudNq4IMT9d0Kpdix/qTCW1TLzSvcP6M+nCU3HGkQS2AEQWxbDATOs9CF7pLahl\r\n"
	"RqXp6zxlvPvPpHouhbURriWJ3ifRB5AY9xb73PoYcLQPBYtWcdmzoiQ+QCOt9m1B\r\n"
	"glwq8cBZ0pJ7g59NLAwnzH18BPYZwFniVnlatrcCgYEAjw9EigEOOCk66XkoMOiR\r\n"
	"wrT7iS4Ya862gf8wopys5d+nQYMvgKjRipLjoXp+5pAtsqx2je9PfUYgQLn/PY2o\r\n"
	"+9/fWFjY4dwodBx8lsLFgbW82MONomaTGpSIrpDJQGCD/a6LidVKRGS5c6EVO2yz\r\n"
	"Aiu1uLvRwElbutsyw+NsKEECgYBMbleyNWcq9tmg5S0EawysR/xliRqSpacZ0tDv\r\n"
	"X/YPSzkuy+f8e8U+QIc3zzPCQes1pPWBCs5s5uvQXkN6ba7hs+VxT2OlAVtrOhmb\r\n"
	"zIcf+JqiaBB9rLoCiySjw+V8V3aQ7lnW8/V/187/K2Cw8dnpLmyMapPk30Do3fOc\r\n"
	"nEbMTQKBgQCrTTho+9LHjWTkLIb+VYfPQ0IQpJbKcqE3K1QF2WxPyrOoH63fFBCB\r\n"
	"pOWEuLUuz2FAv1noAbN/6OQ8H/PT0AFJT/ghA04GnIUF0kjSzY60ehS2mVp6neP+\r\n"
	"AZjzZ6QJYlb5/PFz9oES448kpyaAoS2ke86+R4r4YOMBK+I5RVbfSQ==\r\n"
	"-----END RSA PRIVATE KEY-----\r\n";


// int keepalive_test(MQTTClient* c)
// {
// int rc = FAILURE;

// Timer timer;
// TimerInit(&timer);
// TimerCountdownMS(&timer, 1000);
// int len = MQTTSerialize_pingreq(c->buf, c->buf_size);

// if (len > 0 && (rc = sendPacket(c, len, &timer)) == SUCCESS){ // send the ping packet
// printf("\n%s:%d: Send ping req\n",__func__,__LINE__);//Evan
// }

// return rc;
// }

int keepalive_cnt = 0;
int keepalive_offload_test(void)
{
//step 1: connect to ssl server
//component\example\ssl_client\ssl_client.c
	int ret = 0;
	MQTTClient client;
	int rc = 0, count = 0;
	MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;
#if AWS_IOT_MQTT
	char *address = "a2zweh2b7yb784-ats.iot.ap-southeast-1.amazonaws.com";
#else
	const char *address = "192.168.1.68";
#endif

	const char *sub_topic[3] = {"wakeup", "wakeup1", "wakeup2"};
	const char *pub_topic = "wakeup";

	NetworkInit(&network);
	network.use_ssl = 1;
	network.rootCA = NULL;
#if AWS_IOT_MQTT
	network.clientCA = test_ca_cert;
	network.private_key = test_private_key;
#else
	network.clientCA = NULL;
	network.private_key = NULL;
#endif

	network.ciphersuites = ciphersuites;

	MQTTClientInit(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

//#if AWS_IOT_MQTT
	client.qos_limit = QOS1;
//#endif

	connectData.MQTTVersion = 3;
	//connectData.clientID.cstring = "ameba-iot";
	connectData.clientID.lenstring.data = (char *)"ameba-iot";
	connectData.clientID.lenstring.len = 9;
	connectData.keepAliveInterval = 15 * 60;
	connectData.cleansession = 0;

	int mqtt_pub_count = 0;

	while (1) {
		fd_set read_fds;
		fd_set except_fds;
		struct timeval timeout;

		FD_ZERO(&read_fds);
		FD_ZERO(&except_fds);
		if (client.mqttstatus == MQTT_RUNNING) {
			timeout.tv_sec = 0;
			timeout.tv_usec = 100000;
		} else {
			timeout.tv_sec = 1;
			timeout.tv_usec = 0;
		}

		if (network.my_socket >= 0) {
			FD_SET(network.my_socket, &read_fds);
			FD_SET(network.my_socket, &except_fds);
			rc = FreeRTOS_Select(network.my_socket + 1, &read_fds, NULL, &except_fds, &timeout);

			if (FD_ISSET(network.my_socket, &except_fds)) {
				mqtt_printf(MQTT_INFO, "except_fds is set");
				MQTTSetStatus(&client, MQTT_START); //my_socket will be close and reopen in MQTTDataHandle if STATUS set to MQTT_START
			} else if (rc == 0) { //select timeout
				// if(mqtt_pub_count >= 2)
				// {
				// keepalive_test(&client);
				// mqtt_pub_count = 0;
				// keepalive_cnt++;
				// }

			}
		}
#if AWS_IOT_MQTT
		MQTTDataHandle(&client, &read_fds, &connectData, messageArrived, address, (char **)sub_topic, 3);
#else
		MQTTDataHandle(&client, &read_fds, &connectData, messageArrived, server_ip, (char **)sub_topic, 3);
#endif

		int timercount = 0;
		if (client.mqttstatus == MQTT_RUNNING) {
			mqtt_pub_count++;
			// if(keepalive_cnt>=5)
			//MQTTStringFormat_connect(sendbuf, sizeof(sendbuf), &connectData);
			//printf("MQTT format: %s\r\n", sendbuf);
			if (mqtt_pub_count > 2) {
				break;
			}
		}
	}

	// retain local port
	struct sockaddr_in sin;
	socklen_t len = sizeof(sin);
	if (getsockname(network.my_socket, (struct sockaddr *)&sin, &len) == -1) {
		printf("ERROR: getsockname \n\r");
	} else {
		retention_local_port = ntohs(sin.sin_port);
		dcache_clean_invalidate_by_addr((uint32_t *) &retention_local_port, sizeof(retention_local_port));
		printf("retain local port: %d \n\r", retention_local_port);
	}

//step 2: set ssl offload
//use mbedtls_ssl_context
	if (network.use_ssl == 1) {
		//ssl
		uint8_t iv[16];
		memset(iv, 0xab, sizeof(iv));

		set_ssl_offload(network.ssl, iv, keepalive_content, keepalive_len);
	}

//step 3: set keep alive packet offload
	// ssl offload: must after mbedtls_platform_set_calloc_free() and wifi_set_ssl_offload()
	uint8_t *ssl_record = NULL;
	if (network.use_ssl == 1) {
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
			wifi_set_tcp_keep_alive_offload(network.my_socket, ssl_record, ssl_record_len, interval_ms, resend_ms, 0);

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
			wifi_set_tcp_keep_alive_offload(network.my_socket, ssl_record, ssl_record_len, interval_ms, resend_ms, 0);

			// free ssl_record after content is not used anymore
			if (ssl_record) {
				free(ssl_record);
			}
		}
	} else {
		wifi_set_tcp_keep_alive_offload(network.my_socket, keepalive_content, sizeof(keepalive_content), interval_ms, resend_ms, 0);
	}

#if SSL_KEEPALIVE
	// retain ssl
	extern int mbedtls_ssl_retain(mbedtls_ssl_context * ssl);
	printf("retain SSL %s \n\r", mbedtls_ssl_retain(network.ssl) == 0 ? "OK" : "FAIL");
#endif
#if TCP_RESUME
	// retain tcp pcb
	extern int lwip_retain_tcp(int s);
	printf("retain TCP pcb %s \n\r", lwip_retain_tcp(network.my_socket) == 0 ? "OK" : "FAIL");
#endif

	return ret;
}

void set_icmp_ping_pattern(wowlan_pattern_t *pattern)
{
	memset(pattern, 0, sizeof(wowlan_pattern_t));

	char buf[32], mac[6];
	const char ip_protocol[2] = {0x08, 0x00}; // IP {08,00} ARP {08,06}
	const char ip_ver[1] = {0x45};
	const uint8_t icmp_protocol[1] = {0x01};
	const uint8_t *ip = LwIP_GetIP(0);
	const uint8_t unicast_mask[6] = {0x3f, 0x70, 0x80, 0xc0, 0x03, 0x00};
	const uint8_t *mac_temp = LwIP_GetMAC(0);

	//wifi_get_mac_address(buf);
	//sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	memcpy(mac, mac_temp, 6);
	printf("mac = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X \r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	memcpy(pattern->eth_da, mac, 6);
	memcpy(pattern->eth_proto_type, ip_protocol, 2);
	memcpy(pattern->header_len, ip_ver, 1);
	memcpy(pattern->ip_proto, icmp_protocol, 1);
	memcpy(pattern->ip_da, ip, 4);
	memcpy(pattern->mask, unicast_mask, 6);
}

void set_tcp_not_connected_pattern(wowlan_pattern_t *pattern)
{
	// This pattern make STA can be wake from TCP SYN packet
	memset(pattern, 0, sizeof(wowlan_pattern_t));

	char buf[32];
	char mac[6];
	char ip_protocol[2] = {0x08, 0x00}; // IP {08,00} ARP {08,06}
	char ip_ver[1] = {0x45};
	char tcp_protocol[1] = {0x06}; // 0x06 for tcp
	char tcp_port[2] = {0x00, 0x50}; // port 80
	uint8_t *ip = LwIP_GetIP(0);
	const uint8_t uc_mask[6] = {0x3f, 0x70, 0x80, 0xc0, 0x33, 0x00};
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
	memcpy(pattern->dest_port, tcp_port, 2);
	memcpy(pattern->mask, uc_mask, 6);
}

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

extern u8 wowlan_qos_null;
extern void wifi_wowlan_set_tim_extend(uint8_t tim_more_pspacket,
									   uint8_t tim_dtim1to);

void wowlan_thread(void *param)
{
	int ret;
	int cnt = 0;
	int sock_fd = -1;
#if SSL_KEEPALIVE
	mbedtls_ssl_context ssl;
	mbedtls_ssl_config conf;
#endif

	vTaskDelay(1000);
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	while (1) {
		if (enable_tcp_keep_alive) {
#if WOWLAN_GPIO_WDT
			wifi_wowlan_set_wdt(2, 5, 1, 1); //gpiof_2, io trigger interval 1 min, io pull high and trigger pull low, pulse duration 1ms
#endif

			if (tcp_resume) {
				sock_fd = socket(AF_INET, SOCK_STREAM, 0);
				printf("\n\r socket(%d) \n\r", sock_fd);
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
//tcp resume read
#if 0
				uint32_t pattern_len = 0;
				uint8_t *wakeup_pattern_buf = malloc(packet_len * sizeof(uint8_t));
				pattern_len = read(network.my_socket, wakeup_pattern_buf, packet_len);
				uint32_t x = 0;
				while (1) {
					printf("%02x	", *(wakeup_pattern_buf + x));
					x++;
					if (x >= pattern_len) {
						break;
					}
					if ((x % 43) == 0) {
						printf("\n");
					}
				}
				free(wakeup_pattern_buf);
#endif
#if SSL_KEEPALIVE
				mbedtls_platform_set_calloc_free(_calloc_func, vPortFree);

				mbedtls_ssl_init(&ssl);
				mbedtls_ssl_config_init(&conf);
				mbedtls_ssl_set_bio(&ssl, &sock_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

				if ((ret = mbedtls_ssl_config_defaults(&conf, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT)) != 0) {
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
					printf("\r\nresume SSL %s \n\r", mbedtls_ssl_resume(&ssl) == 0 ? "OK" : "FAIL");
				}

#if 1 //mbedtls ssl read
				uint8_t *wakeup_pattern_buf = malloc(packet_len * sizeof(uint8_t));
				uint32_t pattern_len = mbedtls_ssl_read(&ssl, wakeup_pattern_buf, packet_len);
				printf("\r\nmbedtls ssl read length = %d\r\n", pattern_len);
				uint32_t x = 0;
				while (1) {
					printf("%02x	", *(wakeup_pattern_buf + x));
					x++;
					if (x >= pattern_len) {
						break;
					}
					if ((x % 43) == 0) {
						printf("\n");
					}
				}
				free(wakeup_pattern_buf);
#endif

#endif

			}
			while (keepalive_offload_test() != 0) {
				vTaskDelay(4000);
			}

			wifi_set_dhcp_offload();

			if (set_arpreqenable) {
				wifi_wowlan_set_arpreq_keepalive(0, 0);
			}

#if 0
//step 4: set wakeup pattern
#if 0
			//"test123456789"
			char *data = "123456789";
			wifi_wowlan_set_ssl_pattern(data, strlen(data), 7);
			//"wakeupabcd"
			char *data2 = "abcd";
			wifi_wowlan_set_ssl_pattern(data2, strlen(data2), 6);

			//"12did"
			char *data5 = "did";
			wifi_wowlan_set_ssl_pattern(data5, strlen(data5), 2);

			//"1234did"
			char *data6 = "did";
			wifi_wowlan_set_ssl_pattern(data6, strlen(data6), 4);

			extern void wifi_wowlan_set_patternoffset(uint8_t offset);
			wifi_wowlan_set_patternoffset(4);

#if TCP_SERVER_KEEP_ALIVE
			char *server_keepalive_content = "keepalive";
			wifi_wowlan_set_serverkeepalive(100, server_keepalive_content, strlen(server_keepalive_content), 0);
#endif

#else
			wifi_set_publish_wakeup();
#endif
#endif

			wifi_set_ssl_counter_report();

			//set tbtt interval
			//static uint8_t lps_dtim = 10;
			rtl8735b_set_lps_dtim(lps_dtim);

			wifi_wowlan_set_dtimtimeout(set_dtimtimeout);

			//set bcn track stage2 parmeters
			//static uint8_t rx_bcn_window = 30;
			//static uint8_t bcn_limit = 5;
			wifi_wowlan_bcntrack_stage1(rx_bcn_window, bcn_limit);

			//set bcn track stage2 parmeters
			// static uint8_t  stage2_start_window = 10;
			// static uint8_t  stage2_max_window = 210;
			// static uint8_t  stage2_increment_steps = 40;
			// static uint8_t  stage2_duration = 10;
			// static uint8_t  stage2_null_limit = 6;
			// static uint8_t  stage2_loop_limit = 6;
			wifi_wowlan_set_bcn_track(stage2_start_window, stage2_max_window, stage2_increment_steps, stage2_duration, stage2_null_limit, stage2_loop_limit);

			//select fw
			rtl8735b_select_keepalive(WOWLAN_MQTT_BCNV2);
		}

		if (enable_wowlan_pattern) {
			wowlan_pattern_t test_pattern;
			set_tcp_not_connected_pattern(&test_pattern);
			wifi_wowlan_set_pattern(test_pattern);
		}

#if 0//LONG_RUN_TEST
		wowlan_pattern_t ping_pattern;
		set_icmp_ping_pattern(&ping_pattern);
		wifi_wowlan_set_pattern(ping_pattern);
#endif

		if (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
			printf("wifi disconnect into standby mode\r\n");
			gpio_irq_init(&my_GPIO_IRQ, WAKUPE_GPIO_PIN, gpio_demo_irq_handler, (uint32_t)&my_GPIO_IRQ);
			gpio_irq_pull_ctrl(&my_GPIO_IRQ, PullDown);
			gpio_irq_set(&my_GPIO_IRQ, IRQ_RISE, 1);

			wifi_off();
			Standby(DSTBY_AON_GPIO | DSTBY_AON_TIMER, SLEEP_DURATION, CLOCK, 0);
		}

#if WOWLAN_GPIO_WDT
		//set gpio pull control
		HAL_WRITE32(0x40009850, 0x0, 0x4f004f); //GPIOF_1/GPIOF_0
		//HAL_WRITE32(0x40009854, 0x0, 0x8f004f); //GPIOF_3/GPIOF_2
		HAL_WRITE32(0x40009854, 0x0, 0x8f0000 | (0xffff & HAL_READ32(0x40009854, 0x0))); //GPIOF_3/GPIOF_2 keep wowlan wdt setting
		HAL_WRITE32(0x40009858, 0x0, 0x4f008f); //GPIOF_5/GPIOF_4
		HAL_WRITE32(0x4000985c, 0x0, 0x4f004f); //GPIOF_7/GPIOF_6
		HAL_WRITE32(0x40009860, 0x0, 0x4f004f); //GPIOF_9/GPIOF_8
		HAL_WRITE32(0x40009864, 0x0, 0x4f004f); //GPIOF_11/GPIOF_10
		HAL_WRITE32(0x40009868, 0x0, 0x4f004f); //GPIOF_13/GPIOF_12
		HAL_WRITE32(0x4000986C, 0x0, 0x4f004f); //GPIOF_15/GPIOF_14
		HAL_WRITE32(0x40009870, 0x0, 0x4f004f); //GPIOF_17/GPIOF_16
#endif
		extern int dhcp_retain(void);
		printf("retain DHCP %s \n\r", dhcp_retain() == 0 ? "OK" : "FAIL");
		// for wlan resume
		rtw_hal_wlan_resume_backup();

		printf("start suspend\r\n");
		rtl8735b_set_lps_pg();
		printf("rtl8735b_set_lps_pg\r\n");
		rtw_enter_critical(NULL, NULL);
		if (rtl8735b_suspend(0) == 0) { // should stop wifi application before doing rtl8735b_suspend(
			if (((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {

				dbg_printf("rtl8735b_suspend\r\n");
#if 0//LONG_RUN_TEST
				gpio_irq_init(&my_GPIO_IRQ, WAKUPE_GPIO_PIN, gpio_demo_irq_handler, (uint32_t)&my_GPIO_IRQ);
				gpio_irq_pull_ctrl(&my_GPIO_IRQ, PullDown);
				gpio_irq_set(&my_GPIO_IRQ, IRQ_RISE, 1);
				rtw_exit_critical(NULL, NULL);
				Standby(DSTBY_AON_GPIO | DSTBY_WLAN | DSTBY_AON_TIMER | SLP_GTIMER, SLEEP_DURATION, CLOCK, 1);
#else
#if WOWLAN_GPIO_WDT
				gpio_t my_GPIO1;
				gpio_init(&my_GPIO1, PA_2);
				gpio_irq_pull_ctrl((gpio_irq_t *)&my_GPIO1, PullDown);
				rtw_exit_critical(NULL, NULL);
				Standby(DSTBY_PON_GPIO | DSTBY_WLAN | SLP_GTIMER, 0, CLOCK, 1);
#else


				gpio_t my_GPIO1;
				gpio_init(&my_GPIO1, PA_2);
				gpio_irq_pull_ctrl((gpio_irq_t *)&my_GPIO1, PullDown);
				rtw_exit_critical(NULL, NULL);
				Standby(DSTBY_WLAN | SLP_GTIMER, 0, 0, 1);
#endif
#endif
			} else {
				rtw_exit_critical(NULL, NULL);
				dbg_printf("wifi disconnect into standby mode\r\n");
				//sys_reset();
			}
		} else {
			rtw_exit_critical(NULL, NULL);
			dbg_printf("rtl8735b_suspend fail\r\n");
			//sys_reset();
		}
		//rtw_exit_critical(NULL, NULL);

		printf("end suspend\r\n");

		while (1) {
			vTaskDelay(2000);
			sys_reset();
			printf("!!!\r\n");
		}
	}
exit:
	printf("\n\r close(%d) \n\r", sock_fd);
	close(sock_fd);
#if SSL_KEEPALIVE
	mbedtls_ssl_free(&ssl);
	mbedtls_ssl_config_free(&conf);
#endif
	vTaskDelete(NULL);
}

void fPS(void *arg)
{
	int argc;
	char *argv[MAX_ARGC] = {0};

	argc = parse_param(arg, argv);

	do {
		if (argc == 1) {
			print_PS_help();
			break;
		}

		if (strcmp(argv[1], "wowlan") == 0) {
			if (wowlan_thread_handle == NULL &&
				xTaskCreate(wowlan_thread, ((const char *)"wowlan_thread"), 4096, NULL, tskIDLE_PRIORITY + 1, &wowlan_thread_handle) != pdPASS) {
				printf("\r\n wowlan_thread: Create Task Error\n");
			}
		} else if (strcmp(argv[1], "tcp_keep_alive") == 0) {
			if (argc >= 4) {
				sprintf(server_ip, "%s", argv[2]);
				server_port = strtoul(argv[3], NULL, 10);
			}
			enable_tcp_keep_alive = 1;
			printf("setup tcp keep alive to %s:%d\r\n", server_ip, server_port);
		} else if (strcmp(argv[1], "arp") == 0) {
			if (argc >= 3) {
				char cp[16] = "192.168.1.100";
				sprintf(cp, "%s", argv[2]);
				ip4_addr_t dst_ip;
				if (ip4addr_aton(cp, &dst_ip) == 1) {
					printf("\n\r Success dst_ip: %s", ip4addr_ntoa(&dst_ip));
				}
				//for(int i = 0;i<1;i++){
				//arq request
				etharp_request(&xnetif[0], &dst_ip);
				//}
			}
		} else if (strcmp(argv[1], "mqtt_keep_alive") == 0) {
			if (argc >= 4) {
				sprintf(server_ip, "%s", argv[2]);
				server_port = strtoul(argv[3], NULL, 10);
			}
			enable_tcp_keep_alive = 1;
			printf("setup mqtt keep alive\r\n");
		} else if (strcmp(argv[1], "interval_ms") == 0) {
			if (argc == 3) {
				printf("interval_ms=%02d", atoi(argv[2]));
				interval_ms = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "resend_ms") == 0) {
			if (argc == 3) {
				printf("resend_ms=%02d", atoi(argv[2]));
				resend_ms = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "standby") == 0) {
			printf("into standby\r\n");
			gpio_t my_GPIO1;
			gpio_init(&my_GPIO1, PA_2);
			gpio_irq_pull_ctrl((gpio_irq_t *)&my_GPIO1, PullDown);
			Standby(SLP_AON_TIMER, SLEEP_DURATION, CLOCK, 0);
		} else if (strcmp(argv[1], "deepsleep") == 0) {
			printf("into deepsleep\r\n");
			gpio_t my_GPIO1;
			gpio_t my_GPIO2;
			gpio_init(&my_GPIO1, PA_2);
			gpio_irq_pull_ctrl((gpio_irq_t *)&my_GPIO1, PullDown);
			gpio_init(&my_GPIO2, PA_3);
			gpio_irq_pull_ctrl((gpio_irq_t *)&my_GPIO2, PullDown);

			for (int i = 5; i > 0; i--) {
				dbg_printf("Enter DeepSleep by %d seconds \r\n", i);
				hal_delay_us(1 * 1000 * 1000);
			}

			DeepSleep(DS_AON_TIMER, SLEEP_DURATION, CLOCK);
		} else if (strcmp(argv[1], "dhcp_renew") == 0) {
			printf("dhcp renew\r\n");
			wifi_set_dhcp_offload();
		} else if (strcmp(argv[1], "ant") == 0) {
			printf("rltk_set_antenna_diversity\r\n");
			extern int rltk_set_antenna_diversity(u8 enable);
			rltk_set_antenna_diversity(0);
		}

		else {
			print_PS_help();
		}
	} while (0);
}

#if LONG_RUN_TEST
extern char log_buf[LOG_SERVICE_BUFLEN];
extern xSemaphoreHandle log_rx_interrupt_sema;
static void long_run_test_thread(void *param)
{

	// vTaskDelay(3000);
	// sprintf(log_buf, "PS=mqtt_keep_alive,192.168.50.19,100");
	// xSemaphoreGive(log_rx_interrupt_sema);

	// vTaskDelay(1000);
	// sprintf(log_buf, "PS=wowlan");
	// xSemaphoreGive(log_rx_interrupt_sema);

	while (1) {
		vTaskDelay(100 * 20);
		u32 value32;
		value32 = HAL_READ32(0x40080000, 0x54);
		value32 = value32 & 0xFF;
		printf("beacon recv per 20 = %d\r\n", value32);

		// value32 = HAL_READ32(0x40080000, 0x58);
		// printf("0x58 = 0x%X\r\n",value32);
	}
}
#endif


log_item_t at_power_save_items[ ] = {
	{"PS", fPS,},
};

//char wakeup_packet[1024];
#if defined(VIDEO_EXAMPLE_ON)
/* entry for the example*/
__weak void app_example(void) {}
void run_app_example(void)
{
#if !defined(CONFIG_UNITEST) || (CONFIG_UNITEST==0)
	/* Execute application example */
	app_example();
#endif
}
#endif



int wlan_do_resume(void)
{
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

void main(void)
{

	/* Must have, please do not remove */
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
		RX_MQTT_PING_RSP_TO = 0x76,

	*************************************** */
	//get pm reason
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

				if (wowlan_wake_reason == RX_HW_PATTERN_PKT || wowlan_wake_reason == RX_MQTT_PATTERN_MATCH || wowlan_wake_reason == RX_MQTT_PUBLISH_WAKE ||
					wowlan_wake_reason == MQTT_FW_RX_TCP_PKT_WAKEUP) {
					wlan_mcu_ok = 1;

					uint8_t *wakeup_packet = rtl8735b_read_wakeup_packet(&packet_len, wowlan_wake_reason);

//read wifi buffer
#if 1
					uint32_t x = 0;
					while (1) {
						printf("%02x	", *(wakeup_packet + x));
						x++;
						if (x >= packet_len) {
							break;
						}
						if ((x % 43) == 0) {
							printf("\n");
						}
					}
#endif

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
							tcp_resume_seqno = *((uint32_t *)(rtl8735b_read_ssl_conuter_report() + 16));
							tcp_resume_ackno = *((uint32_t *)(rtl8735b_read_ssl_conuter_report() + 20)) - tcp_payload_len;
							printf("\ntcp_resume_seqno=%u, tcp_resume_ackno=%u \n\r", tcp_resume_seqno, tcp_resume_ackno);
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
						} else if (tcp_header[13] == 0x11) {
							printf("FIN + ACK\n\r");

							// not resume because TCP connection is closed
						}
					}

					free(wakeup_packet);

#if defined(TCP_RESUME_MAX_PACKETS) && (TCP_RESUME_MAX_PACKETS > 1)
					for (int j = 1; j < TCP_RESUME_MAX_PACKETS; j ++) {
						extern uint8_t *rtl8735b_read_packet_with_index(uint32_t *size, uint8_t index);
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
				}
			}
		} else if (pm_reason & (BIT(9) | BIT(10) | BIT(11) | BIT(12))) {
			// AON GPIO wake up

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

			///////??
			extern void lwip_recover_resume_keepalive(void);
			lwip_recover_resume_keepalive();

#if SSL_KEEPALIVE
			ssl_resume = 1;

			uint8_t *ssl_counter = rtl8735b_read_ssl_conuter_report();
			uint8_t ssl_resume_out_ctr[8];
			uint8_t ssl_resume_in_ctr[8];
			memcpy(ssl_resume_out_ctr, ssl_counter, 8);
			memcpy(ssl_resume_in_ctr, ssl_counter + 8, 8);

			uint8_t zero_ctr[8];
			memset(zero_ctr, 0, 8);
			if ((memcmp(zero_ctr, ssl_resume_in_ctr, 8) != 0) && by_wlan) {
				for (int i = 8; i > 0; i --) {
					if (ssl_resume_in_ctr[i - 1] != 0) {
						ssl_resume_in_ctr[i - 1] --;
						break;
					} else {
						ssl_resume_in_ctr[i - 1] = 0xff;
					}
				}
			}

			printf("ssl_resume_out_ctr = ");
			for (int i = 0; i < 8; i ++) {
				printf("%02X", ssl_resume_out_ctr[i]);
			}
			printf("\r\nssl_resume_in_ctr = ");
			for (int i = 0; i < 8; i ++) {
				printf("%02X", ssl_resume_in_ctr[i]);
			}
			extern int mbedtls_set_ssl_resume(uint8_t in_ctr[8], uint8_t out_ctr[8], uint8_t by_wlan);
			mbedtls_set_ssl_resume(ssl_resume_in_ctr, ssl_resume_out_ctr, 0);
			//extern void mbedtls_set_ssl_resume_fix_ctr(uint8_t fix_ctr, uint8_t max_try);
			//mbedtls_set_ssl_resume_fix_ctr(1, 2);
#endif
#endif
		}

		if (wlan_mcu_ok && (rtw_hal_wlan_resume_check() == 1)) {
			wlan_resume = 1;

			rtw_hal_read_aoac_rpt_from_txfifo(NULL, 0, 0);
		}
	}
	/* Initialize log uart and at command service */

	console_init();

	log_service_add_table(at_power_save_items, sizeof(at_power_save_items) / sizeof(at_power_save_items[0]));

	if (wlan_resume) {
		p_wifi_do_fast_connect = wlan_do_resume;
		p_store_fast_connect_info = NULL;
	} else {
		wifi_fast_connect_enable(1);
	}
	/* wlan intialization */
	wlan_network();

	sys_backtrace_enable();

#if LONG_RUN_TEST
	if (xTaskCreate(long_run_test_thread, ((const char *)"long_run_test_thread"), 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n long_run_test_thread: Create Task Error\n");
	}
#endif

#if defined(VIDEO_EXAMPLE_ON)
	voe_t2ff_prealloc();

	/* Execute application example */
	run_app_example();
#endif


	/*Enable Schedule, Start Kernel*/

	vTaskStartScheduler();

}
