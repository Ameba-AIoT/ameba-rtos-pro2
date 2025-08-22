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

#define MQTTSSL_KEEPALIVE 1
// ignore mqtt header check
#define TCPSSL_MODE       0
#define TCP_SERVER_KEEP_ALIVE 0
#define AWS_IOT_MQTT      0
#define MEDIA_QUICK_POWEROFF    0

#if MQTTSSL_KEEPALIVE
#include "MQTTClient.h"
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

static char server_ip[16] = "192.168.0.100";
static uint16_t server_port = 5566;
static int enable_tcp_keep_alive = 0;
static uint32_t interval_ms = 180000;
static uint32_t resend_ms = 10000;

static int enable_wowlan_pattern = 0;

#define KEEP_ALIVE_FINE_TUNE 1
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


#if MQTTSSL_KEEPALIVE
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

#endif

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
#if MQTTSSL_KEEPALIVE
//step 1: connect to ssl server
//component\example\ssl_client\ssl_client.c
	int ret = 0;
	MQTTClient client;
	Network network;
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

	return ret;
#else
	int socket_fd;
	int ret = 0;

	if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("ERROR: socket\n");
	} else {
		struct sockaddr_in server_addr;
		server_addr.sin_family = AF_INET;
		server_addr.sin_addr.s_addr = inet_addr(server_ip);	// IP of a TCP server
		server_addr.sin_port = htons(server_port);

		if (connect(socket_fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) == 0) {
			printf("\r\nconnected to %s:%d\r\n", server_ip, server_port);

			uint8_t data[] = {0xc0, 0x00};

//TCP Keep alive test
#if 0
			for (int i = 0; i < 5; i++) {
				send(socket_fd, data, 2, 0);
				vTaskDelay(2000);
			}
#endif

			wifi_set_tcp_keep_alive_offload(socket_fd, data, sizeof(data), interval_ms, resend_ms, 1);
			//should not send any IP packet after set_keep_send()
		} else {
			printf("ERROR: connect %s:%d\r\n", server_ip, server_port);
			ret = -1;
		}
	}
	return ret;
#endif
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

uint8_t dtimtimeout2 = 5;
uint8_t rx_bcn_limit2 = 8;
uint8_t l2_keepalive_period2 = 50;
uint8_t ps_timeout2 = 16;
uint8_t ps_retry2 = 7;
uint8_t sd_period = 20;
uint8_t sd_threshold = 6;
extern u8 new_track;
extern u8 tracklimit;
uint8_t wowlan_dtim2 = 10;
uint8_t bcn_to_limit2 = 20;
uint8_t arp_keep_alive = 1;
extern u8 wowlan_qos_null;
extern void wifi_wowlan_set_tim_extend(uint8_t tim_more_pspacket,
									   uint8_t tim_dtim1to);

void wowlan_thread(void *param)
{
	int ret;
	int cnt = 0;

	vTaskDelay(1000);
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	while (1) {
#if defined(VIDEO_EXAMPLE_ON) && MEDIA_QUICK_POWEROFF
		while (1) {
			vTaskDelay(100);
			extern int video_example_get_init_status(void);
			if (video_example_get_init_status()) {
				printf("mmf init done\r\n");
				break;
			}
		}
#endif
		if (enable_tcp_keep_alive) {
			wifi_set_powersave_mode(0xff, 0); //  Leave LPS mode

#if KEEP_ALIVE_FINE_TUNE
			switch (wowlan_dtim2) {
			case 1:
				bcn_to_limit2 = 6;
				break;
			case 4:
				bcn_to_limit2 = 12;
				break;
			case 6:
				bcn_to_limit2 = 18;
				break;
			case 8:
				bcn_to_limit2 = 16;
				break;
			case 10:
				bcn_to_limit2 = 20;
				break;
			}
			wifi_wowlan_set_pstune_param(ps_timeout2, ps_retry2, rx_bcn_limit2, dtimtimeout2, bcn_to_limit2);
			wifi_wowlan_set_fwdecision_param(80, 1, 0, 0, l2_keepalive_period2); // workaround check period < 100
#endif

#if WOWLAN_GPIO_WDT
			wifi_wowlan_set_wdt(2, 5, 1, 1); //gpiof_2, io trigger interval 1 min, io pull high and trigger pull low, pulse duration 1ms
#endif

			while (keepalive_offload_test() != 0) {
				vTaskDelay(4000);
			}

			wifi_set_dhcp_offload();

			if (arp_keep_alive) {
				wifi_wowlan_set_arpreq_keepalive(1, 0);
			}

//step 4: set wakeup pattern
#if MQTTSSL_KEEPALIVE
#if TCPSSL_MODE
			wifi_set_tcpssl_keepalive();

			//"mqtt topic wakeup"
			//char *data = "wakeup";
			//wifi_wowlan_set_ssl_pattern(data, strlen(data), 4); //offset mqtt header 4bytes

			//"mqtt topic wakeup1"
			char *data1 = "wakeup1";
			wifi_wowlan_set_ssl_pattern(data1, strlen(data1), 4);

			//"mqtt topic wakeup2"
			char *data2 = "wakeup2";
			wifi_wowlan_set_ssl_pattern(data2, strlen(data2), 4);

#if TCP_SERVER_KEEP_ALIVE
			char *server_keepalive_content = "wakeup";
			wifi_wowlan_set_serverkeepalive(100, server_keepalive_content, strlen(server_keepalive_content), 4);
#endif

#else
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
#else
			wowlan_pattern_t data_pattern;
			set_tcp_connected_pattern(&data_pattern);
			wifi_wowlan_set_pattern(data_pattern);
#endif

			wifi_set_ssl_counter_report();

			// while(1)
			// {
			// vTaskDelay(1000);
			// }

			wifi_set_powersave_mode(0xff, 1); //into LPS mode

#if KEEP_ALIVE_FINE_TUNE
			//tracklimit =10;
			wifi_wowlan_set_dtimto(1, 1, 40, wowlan_dtim2);
			//wifi_wowlan_set_smartdtim(sd_period, sd_threshold, 1, 6);
#endif

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

		//wifi_wowlan_set_tim_extend(1, 0);
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
		printf("start suspend\r\n");
#if defined(VIDEO_EXAMPLE_ON) && MEDIA_QUICK_POWEROFF
		//block voe command lock before poweroff
		printf("get voe lock\r\n");
		device_voe_lock();
		rtw_enter_critical(NULL, NULL);
		//power off video, audio, sd
		dbg_printf("audio power off\r\n");
		audio_poweroff();
		dbg_printf("sd power off\r\n");
		sd_gpio_power_off_withoutdelay();
		dbg_printf("video power off\r\n");
		video_poweroff();
		rtw_exit_critical(NULL, NULL);
		dbg_printf("media power off end\r\n");
#endif
		//select fw
		rtl8735b_select_keepalive(WOWLAN_NORMAL_BCNV1);

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
				Standby(DSTBY_AON_GPIO | DSTBY_WLAN | DSTBY_AON_TIMER, SLEEP_DURATION, CLOCK, 0);
#else
#if WOWLAN_GPIO_WDT
				gpio_t my_GPIO1;
				gpio_init(&my_GPIO1, PA_2);
				gpio_irq_pull_ctrl((gpio_irq_t *)&my_GPIO1, PullDown);
				rtw_exit_critical(NULL, NULL);
				Standby(DSTBY_PON_GPIO | DSTBY_WLAN, 0, CLOCK, 0);
#else


				gpio_t my_GPIO1;
				gpio_init(&my_GPIO1, PA_2);
				gpio_irq_pull_ctrl((gpio_irq_t *)&my_GPIO1, PullDown);
				rtw_exit_critical(NULL, NULL);
				Standby(DSTBY_WLAN, 0, 0, 0);
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

		} else if (strcmp(argv[1], "dtim10") == 0) {
			printf("dtim=10\r\n");
			wifi_wowlan_set_dtimto(0, 0, 0, 10);
		} else if (strcmp(argv[1], "dtim") == 0) {
			if (argc == 3 && atoi(argv[2]) >= 1 && atoi(argv[2]) <= 15) {
				printf("dtim=%02d", atoi(argv[2]));
				wifi_wowlan_set_dtimto(0, 0, 0, atoi(argv[2]));
				wowlan_dtim2 = atoi(argv[2]);
			} else {
				printf("dtim=10\r\n");
				wifi_wowlan_set_dtimto(0, 0, 0, 10);
			}
		} else if (strcmp(argv[1], "dtimto") == 0) {
			printf("dtim = 10 keep alive fine tune\r\n");
			wifi_wowlan_set_dtimto(1, 1, 50, 10);
		} else if (strcmp(argv[1], "dhcp_renew") == 0) {
			printf("dhcp renew\r\n");
			wifi_set_dhcp_offload();
		} else if (strcmp(argv[1], "arp_keep_alive") == 0) {
			printf("arp keep alive\r\n");
			wifi_wowlan_set_arp_rsp_keep_alive(1);
		} else if (strcmp(argv[1], "dto") == 0) {
			if (argc == 3) {
				printf("dtimtimeout=%02d", atoi(argv[2]));
				dtimtimeout2 = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "rx_bcn") == 0) {
			if (argc == 3) {
				printf("rx_bcn_limit=%02d", atoi(argv[2]));
				rx_bcn_limit2 = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "l2") == 0) {
			if (argc == 3) {
				printf("l2_keepalive_period=%02d", atoi(argv[2]));
				l2_keepalive_period2 = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "ps_timeout") == 0) {
			if (argc == 3) {
				printf("ps_timeout=%02d", atoi(argv[2]));
				ps_timeout2 = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "ps_retry") == 0) {
			if (argc == 3) {
				printf("ps_retry=%02d", atoi(argv[2]));
				ps_retry2 = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "sd_p") == 0) {
			if (argc == 3) {
				printf("sd_period=%02d", atoi(argv[2]));
				sd_period = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "sd_th") == 0) {
			if (argc == 3) {
				printf("sd_threshold=%02d", atoi(argv[2]));
				sd_threshold = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "new_track") == 0) {
			if (argc == 3) {
				printf("new_track=%02d", atoi(argv[2]));
				new_track = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "tracklimit") == 0) {
			if (argc == 3) {
				printf("tracklimit=%02d", atoi(argv[2]));
				tracklimit = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "ant") == 0) {
			printf("rltk_set_antenna_diversity\r\n");
			extern int rltk_set_antenna_diversity(u8 enable);
			rltk_set_antenna_diversity(0);
		} else if (strcmp(argv[1], "arp_keep") == 0) {
			if (argc == 3) {
				printf("arp_keep=%02d", atoi(argv[2]));
				arp_keep_alive = atoi(argv[2]);
			}
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

	uint8_t wowlan_wake_reason = rtl8735b_wowlan_wake_reason();
	u32 beacon_cnt = 20;
	uint8_t pre_dtim = 10;
	uint16_t tcp_cnt = 0;
	uint16_t tcp_retry_cnt = 0;
	uint8_t tcp_retry_avg_cnt = 0;
	u32 keepalive_time = 0;
	uint32_t tcp_resume_seqno = 0;
	uint32_t tcp_resume_ackno = 0;
	uint8_t ssl_resume_out_ctr[8];
	uint8_t ssl_resume_in_ctr[8];

	hal_xtal_divider_enable(1);
	hal_32k_s1_sel(2);
	HAL_WRITE32(0x40009000, 0x18, 0x1 | HAL_READ32(0x40009000, 0x18)); //SWR 1.35V

	if (wowlan_wake_reason != 0) {
		printf("\r\nwake fom wlan: 0x%02X\r\n", wowlan_wake_reason);
		u32 value32;
		value32 = HAL_READ32(0x40080000, 0x54);
		value32 = value32 & 0xFF;
		printf("beacon recv per 20 = %d\r\n", value32);
		beacon_cnt = value32;


		if (wowlan_wake_reason == RX_MQTT_PATTERN_MATCH || wowlan_wake_reason == RX_MQTT_PUBLISH_WAKE) {
			uint8_t wowlan_wakeup_pattern = rtl8735b_wowlan_wake_pattern();
			printf("\r\nwake fom wlan pattern index: 0x%02X\r\n", wowlan_wakeup_pattern);
		}

		uint8_t *ssl_counter = rtl8735b_read_ssl_conuter_report();

		int i = 0;
		printf("ssl_counter = \r\n");
		for (i = 0; i < 30; i++) {
			printf("%02X", ssl_counter[i]);
		}
		printf("\r\n");


		memcpy(ssl_resume_out_ctr, ssl_counter, 8);
		memcpy(ssl_resume_in_ctr, ssl_counter + 8, 8);

		memcpy(&tcp_resume_seqno, &ssl_counter[16], 4);
		memcpy(&tcp_resume_ackno, &ssl_counter[20], 4);

		memcpy(&pre_dtim, &ssl_counter[28], 1);
		memcpy(&tcp_cnt, &ssl_counter[24], 2);
		memcpy(&tcp_retry_cnt, &ssl_counter[26], 2);
		if (tcp_cnt && (tcp_cnt > tcp_retry_cnt)) {
			tcp_retry_avg_cnt = tcp_retry_cnt / (tcp_cnt - tcp_retry_cnt);
			keepalive_time = (tcp_cnt - tcp_retry_cnt) * interval_ms;
		}

		//smart dtim control
		//1. wakeup reason : wowlan_wake_reason
		//2. current dtim : pre_dtim
		//3. beacon recive rate : beacon_cnt
		//4. tcp retry avg cnt : tcp_retry_avg_cnt
		//5. keep alive time: tcp_cnt*interval_ms
		//dtim adjuest
		wowlan_dtim2 = pre_dtim;
		if (wowlan_wake_reason == RX_DEAUTH || wowlan_wake_reason == RX_DISASSOC ||
			wowlan_wake_reason == TX_TCP_SEND_LIMIT || wowlan_wake_reason == RX_TCP_FROM_SERVER_TO ||
			wowlan_wake_reason == RX_TCP_RST_FIN_PKT) {
			if (keepalive_time < (2 * 60 * 60 * 1000)) {
				if (wowlan_dtim2 >= 2) {
					wowlan_dtim2 = pre_dtim - 2;
				}
				if (wowlan_dtim2 < 1) {
					wowlan_dtim2 = 1;
				}
			} else if (keepalive_time > (4 * 60 * 60 * 1000)) {
				wowlan_dtim2 = pre_dtim + 2;
				if (wowlan_dtim2 > 8) {
					wowlan_dtim2 = 8;
				}
			}
		} else if (wowlan_wake_reason == RX_MQTT_PATTERN_MATCH || wowlan_wake_reason == RX_MQTT_PUBLISH_WAKE) {
			if (beacon_cnt > 15) {
				wowlan_dtim2 = pre_dtim + 2;
				if (wowlan_dtim2 > 8) {
					wowlan_dtim2 = 8;
				}
			}
		}

		printf("--------dtim adjust--------\r\n");
		printf("wakeup reason = 0x%X\r\n", wowlan_wake_reason);
		printf("wakeup beacon_cnt = %d\r\n", beacon_cnt);
		printf("pre_dtim = %d\r\n", pre_dtim);
		printf("tcp_cnt = %d\r\n", tcp_cnt);
		printf("tcp_retry_cnt = %d\r\n", tcp_retry_cnt);
		printf("tcp_retry_avg_cnt = %d\r\n", tcp_retry_avg_cnt);
		printf("\ntcp_resume_seqno=%u, tcp_resume_ackno=%u \n\r", tcp_resume_seqno, tcp_resume_ackno);
		printf("ssl_resume_out_ctr = ");
		for (int i = 0; i < 8; i ++) {
			printf("%02X", ssl_resume_out_ctr[i]);
		}
		printf("\r\nssl_resume_in_ctr = ");
		for (int i = 0; i < 8; i ++) {
			printf("%02X", ssl_resume_in_ctr[i]);
		}


		if (wowlan_wake_reason == RX_MQTT_PATTERN_MATCH || wowlan_wake_reason == RX_MQTT_PUBLISH_WAKE) {
			uint32_t packet_len = 0;
			uint8_t *wakeup_packet = rtl8735b_read_wakeup_packet(&packet_len, wowlan_wake_reason);
			//uint8_t *wakeup_packet = rtl8735b_read_wakeup_payload(&packet_len, 1);




#if 0
			int i = 0;
			do {
				printf("packet content[%d] = 0x%02X%02X%02X%02X\r\n", i, wakeup_packet[i + 3], wakeup_packet[i + 2], wakeup_packet[i + 1], wakeup_packet[i]);
				if (i >= packet_len) {
					break;
				}
				i += 4;
			} while (1);
#endif

			//do packet_parser
			free(wakeup_packet);
		}

		if (wowlan_wake_reason == RX_DEAUTH || wowlan_wake_reason == RX_DISASSOC) {
			uint32_t packet_len = 0;
			uint8_t *wakeup_packet = rtl8735b_read_wakeup_packet(&packet_len, wowlan_wake_reason);

#if 1
			int i = 0;
			do {
				printf("packet content[%d] = 0x%02X%02X%02X%02X\r\n", i, wakeup_packet[i + 3], wakeup_packet[i + 2], wakeup_packet[i + 1], wakeup_packet[i]);
				if (i >= packet_len) {
					break;
				}
				i += 4;
			} while (1);
#endif

			//do packet_parser
			if ((*wakeup_packet == 0xC0) || (*wakeup_packet == 0xA0)) {
				uint16_t reason_code = 0x0;
				memcpy(&reason_code, &wakeup_packet[24], 2);
				printf("reason_code = 0x%X\r\n", reason_code);
			}

			free(wakeup_packet);
		}
	}

	/* Initialize log uart and at command service */

	console_init();

	log_service_add_table(at_power_save_items, sizeof(at_power_save_items) / sizeof(at_power_save_items[0]));

	wifi_fast_connect_enable(1);
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
