#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "main.h"
#include <example_entry.h>
#include "platform_autoconf.h"
#include <osdep_service.h>
#include <wifi_conf.h>
#include <lwip_netconf.h>
#include "gpio_api.h"
#include "gpio_irq_api.h"
#include <lwipconf.h>
#include <flash_api.h>
#include <device_lock.h>
#include "doorbell_demo.h"
#include "app_setting.h"
#include "fatfs_sdcard_api.h"
#include <time.h>
#include "hal_power_mode.h"
#include "power_mode_api.h"
#include "sys_api_ext.h"
#include "serial_api.h"

gpio_t gpio_amp;
#ifndef DOORBELL_CHIME_MODE_ENABLE
gpio_irq_t gpio_btn;
gpio_t gpio_led_red;
gpio_t gpio_led_blue;
#else
gpio_t gpio_led2;
#endif

int playring = 0;
_sema doorbell_handle_sema;
extern int sd_ready;
static flash_t flash;
static char sd_filename[32];
static fatfs_sd_params_t fatfs_sd;
extern int rtsTimezone;
extern doorbell_ctr_t doorbell_handle;
extern int remote_poweroff;
extern struct netif xnetif[NET_IF_NUM];
extern int Wake_On_Wlan;

#if USE_ICC_CMD
extern void icc_task_func(void);
#else
extern void uart_cmd_task_func(void);
#endif
extern void mp4_record_stop(void);
extern void send_icc_cmd_poweroff(void);
extern struct tm sntp_gen_system_time(int timezone);
extern void start_doorbell_ring(void);
extern int isp_suspend_func(void *parm);
extern void example_qr_code_scanner_modified(void);
extern int check_doorbell_mmf_status(void);

void amp_gpio_enable(int enable)
{
	gpio_write(&gpio_amp, enable);
}

#ifdef DOORBELL_CHIME_MODE_ENABLE
void led_2_enable(int enable)
{
	gpio_write(&gpio_led2, enable);
}
#else
void led_blue_enable(int enable)
{
	gpio_write(&gpio_led_blue, enable);
}

void led_red_enable(int enable)
{
	gpio_write(&gpio_led_red, enable);
}
#endif

void thread_doorbell_status_monitor(void *parm)
{
	static int wifi_wait_count = 0;
	static int wifi_flag = 0;
	int i = 0;
	int led_value = 0;
	unsigned int start_tick = xTaskGetTickCount();
	unsigned int curr_tick;


#if ISP_BOOT_MODE_ENABLE
	while (check_doorbell_mmf_status() == 0) {
		vTaskDelay(1);
	}
	pre_example_entry();

#if defined(CONFIG_WIFI_NORMAL) && defined(CONFIG_NETWORK)
	wlan_network();
#endif
#endif
	//wait ring
	vTaskDelay(1000);
	if (playring) {
		vTaskDelay(5000);
	}

//#if DOORBELL_CHIME_MODE_ENABLE
#if 0
	while (wifi_is_ready_to_transceive(RTW_STA_INTERFACE) != RTW_SUCCESS) {
		curr_tick =  xTaskGetTickCount();
		if (curr_tick - start_tick >= WIFI_CONNECT_TIMEOUT) {
			printf("wifi connection timeout\r\n");
			amp_gpio_enable(0);

			wifi_disconnect();
			vTaskDelay(1000);
			printf("doorbellpoweroff\r\n");
			extern void uart_send_str(serial_t *sobj, char *pstr);
			extern serial_t    sobj;
			uart_send_str(&sobj, "doorbellpoweroff");
		}
		vTaskDelay(100);
	}
#endif

	/* Execute p2p*/
	skynet_device_run();


	//LED control
	while (1) {
		//QR code Scan
		if (qrcode_scanner_running()) {
			while (qrcode_scanner_running()) {
#if DOORBELL_CHIME_MODE_ENABLE
				led_2_enable(0);
				vTaskDelay(200);
				led_2_enable(1);
				vTaskDelay(200);
#else
				led_red_enable(0);
				vTaskDelay(200);
				led_red_enable(1);
				vTaskDelay(200);
#endif
			}
		}

		//wifi not connected
		if (wifi_is_ready_to_transceive(RTW_STA_INTERFACE) != RTW_SUCCESS) {
			doorbell_handle.doorbell_state &= ~ STATE_WIFI_CONNECTED;
#if DOORBELL_CHIME_MODE_ENABLE
			led_2_enable(led_value);
#else
			led_red_enable(led_value);
			led_blue_enable(~led_value);
#endif
			led_value = ~led_value;
			wifi_flag = 0;
		} else {
			if (wifi_flag == 0) {
				doorbell_handle.doorbell_state |= STATE_WIFI_CONNECTED;
#if DOORBELL_CHIME_MODE_ENABLE
				led_2_enable(0);
#else
				led_red_enable(0);
				led_blue_enable(0);
#endif
				wifi_flag = 1;
				printf("wifi connected\r\n");
			}
		}

#if DOORBELL_CHIME_MODE_ENABLE
		if (remote_poweroff) {
			amp_gpio_enable(0);

			wifi_disconnect();
			vTaskDelay(1000);
			printf("remote_poweroff doorbellpoweroff\r\n");
			extern void uart_send_str(serial_t *sobj, char *pstr);
			extern serial_t    sobj;
			uart_send_str(&sobj, "IPCdoorbellpoweroff");
			vTaskDelay(100);
			uart_send_str(&sobj, "doorbellpoweroff");
			vTaskDelay(100);
		}
#endif

		vTaskDelay(100);
	}
}

void doorbell_status_monitor()
{
	if (xTaskCreate(thread_doorbell_status_monitor, ((const char *)"thread_wifi_monitor"), 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(thread_wifi_monitor) failed", __FUNCTION__);
	}
}

void check_doorbell_status()
{
	extern int check_doorbell_mmf_status();
	while (1) {
		if (check_doorbell_mmf_status()) {
			break;
		}
		vTaskDelay(10);
	}
}

void set_icmp_ping_pattern(wowlan_pattern_t *pattern)
{
	memset(pattern, 0, sizeof(wowlan_pattern_t));

	char buf[32], mac[6];
	const char ip_protocol[2] = {0x08, 0x00}; // IP {08,00} ARP {08,06}
	const char ip_ver[1] = {0x45};
	const uint8_t icmp_protocol[1] = {0x01};
	const uint8_t *ip = LwIP_GetIP(&xnetif[0]);
	const uint8_t unicast_mask[6] = {0x3f, 0x70, 0x80, 0xc0, 0x03, 0x00};

	wifi_get_mac_address(buf);
	sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	memcpy(pattern->eth_da, mac, 6);
	memcpy(pattern->eth_proto_type, ip_protocol, 2);
	memcpy(pattern->header_len, ip_ver, 1);
	memcpy(pattern->ip_proto, icmp_protocol, 1);
	memcpy(pattern->ip_da, ip, 4);
	memcpy(pattern->mask, unicast_mask, 6);
}

void media_contol_thread(void *param)
{
	int timeoutstatus = 0;
	int qrtime = 0;
	int playbacktime = 0;
	int streamtime = 0;
	int speaktime = 0;
	doorbell_handle.speak_on = 0;
	doorbell_handle.stream_on = 0;;


	mmf2_h264_2way_audio_pcmu_doorbell_init();

	while (1) {
		if (rtw_down_timeout_sema(&doorbell_handle_sema, SUSPEND_TIME) == 0) {
			timeoutstatus = 1;
		}
		printf("Doorbell_state = 0x%x\r\n", doorbell_handle.doorbell_state);
		switch (doorbell_handle.new_state) {
		case STATE_ARAM:
			printf("STATE_ARAM\r\n");
			if (doorbell_handle.doorbell_state & STATE_RECORD || qrcode_scanner_running() || doorbell_handle.doorbell_state & STATE_NONESD) {
				//do something
				doorbell_handle.new_state = STATE_NORMAL;
			} else {
				doorbell_handle.new_state = STATE_NORMAL;

#ifndef DOORBELL_CHIME_MODE_ENABLE
				if (sd_ready) {
					DIR m_dir;
					fatfs_sd_get_param(&fatfs_sd);
					sprintf(sd_filename, "%s%s", fatfs_sd.drv, MP4_DIR);
					if (f_opendir(&m_dir, sd_filename) == 0) {
						f_closedir(&m_dir);
					} else {
						f_mkdir(sd_filename);
					}


					struct tm tm_now = sntp_gen_system_time(rtsTimezone);
					sprintf(sd_filename, "%s/%04d%02d%02d_%02d%02d%02d", MP4_DIR, tm_now.tm_year, tm_now.tm_mon, tm_now.tm_mday, tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
					//sprintf(sd_filename, "%s/%s", MP4_DIR, "mp4_record");
					mp4_record_start(sd_filename);

					doorbell_handle.doorbell_state |= STATE_RECORD;
				} else {
					doorbell_handle.doorbell_state |= STATE_NONESD;
				}
#endif
			}
			break;
		case STATE_QRCODE:
			printf("STATE_QRCODE\r\n");
			doorbell_handle.new_state = STATE_NORMAL;
			example_qr_code_scanner_modified();
			break;

		case STATE_RING:
#if USE_EVB
			amp_gpio_enable(0);

			wifi_disconnect();
			vTaskDelay(1000);
			printf("doorbellpoweroff\r\n");
			extern void uart_send_str(serial_t *sobj, char *pstr);
			extern serial_t    sobj;
			uart_send_str(&sobj, "IPCdoorbellpoweroff");
			vTaskDelay(100);
			uart_send_str(&sobj, "doorbellpoweroff");
			vTaskDelay(100);

			if (rltk_wlan_running(0)) {
				wifi_off();
			}

			isp_suspend_func(NULL);
			vTaskDelay(1000);
			send_icc_cmd_poweroff();

#else
			printf("STATE_RING\r\n");
			doorbell_handle.new_state = STATE_NORMAL;
			playring = 1;

			check_doorbell_status();

			printf("play doorbell\r\n");
			start_doorbell_ring();
			playring = 0;
#endif
			break;

		case STATE_PLAYBACK:
			printf("STATE_PLAYBACK\r\n");
			doorbell_handle.new_state = STATE_NORMAL;
			doorbell_handle.doorbell_state |= STATE_PLAYBACK;
			playbacktime = 0;
			break;

		case STATE_STREAM:
			printf("STATE_STREAM\r\n");
			printf("doorbell_handle.stream_on = %d\r\n", doorbell_handle.stream_on);
			if (doorbell_handle.stream_on <= 0) {
				//Stream on = 0 no one use stream
				doorbell_handle.new_state = STATE_NORMAL;
				doorbell_handle.doorbell_state &= ~STATE_STREAM;
				doorbell_handle.stream_on = 0;
			} else {
				//New stream on reset time
				doorbell_handle.new_state = STATE_NORMAL;
				doorbell_handle.doorbell_state |= STATE_STREAM;
				streamtime = MAX_SPEAK_TIMES;
			}
			break;

		case STATE_SPEAK:
			printf("STATE_SPEAK\r\n");
			if (doorbell_handle.speak_on <= 0) {
				//Stream on = 0 no one use speak
				doorbell_handle.new_state = STATE_NORMAL;
				doorbell_handle.doorbell_state &= ~STATE_SPEAK;
				doorbell_handle.speak_on = 0;
				speaktime = 0;
			} else {
				//speak on
				doorbell_handle.new_state = STATE_NORMAL;
				doorbell_handle.doorbell_state |= STATE_SPEAK;
				speaktime = MAX_STREAM_TIMES;
			}
			break;

		case STATE_NORMAL:
#ifndef DOORBELL_CHIME_MODE_ENABLE
			printf("STATE_NORMAL\r\n");
			if (qrcode_scanner_running() && timeoutstatus) {
				qrtime++;
				timeoutstatus = 0;
				if (qrtime >= 3) {
					qrcode_scanner_stop();
					qrtime = 0;
				}
			} else if ((doorbell_handle.doorbell_state & STATE_PLAYBACK) && timeoutstatus) {
				playbacktime++;
				timeoutstatus = 0;
				if (playbacktime >= 3) {
					doorbell_handle.doorbell_state &= ~STATE_PLAYBACK;
					playbacktime = 0;
				}
			} else if (timeoutstatus) {
				if (doorbell_handle.doorbell_state & STATE_RECORD) {
					mp4_record_stop();
					vTaskDelay(1000);
					doorbell_handle.doorbell_state &= ~STATE_RECORD;
				}
#if DOORBELL_AMP_ENABLE
				amp_gpio_enable(0);
#endif

				led_blue_enable(0);
				led_red_enable(0);
				printf("Timeout!!\r\n");

				Wake_On_Wlan = 1;

				while (1) {
					vTaskDelay(1000);
				}

			}
#else
			/*
			if(!(doorbell_handle.doorbell_state & STATE_SPEAK)){
			    if((doorbell_handle.doorbell_state & STATE_STREAM) && timeoutstatus){
			        //printf("STREAMING:%d\r\n",doorbell_handle.doorbell_state & STATE_STREAM);
			        streamtime --;
			        printf("STEAM TIMEs: %d\r\n", streamtime);
			        if(streamtime == 0){
			            doorbell_handle.doorbell_state &= ~STATE_STREAM;
			        }
			    }
			    else{
			        amp_gpio_enable(0);
			        wifi_disconnect();
			        vTaskDelay(1000);
			        printf("Timeout doorbellpoweroff\r\n");
			        extern void uart_send_str(serial_t *sobj, char *pstr);
			        extern serial_t    sobj;
			        uart_send_str(&sobj,"IPCdoorbellpoweroff");
			        vTaskDelay(100);
			        uart_send_str(&sobj,"doorbellpoweroff");
			        vTaskDelay(100);
			    }
			}
			*/
			if (!(doorbell_handle.doorbell_state & STATE_SPEAK) && !(doorbell_handle.doorbell_state & STATE_STREAM)) {
				amp_gpio_enable(0);
				wifi_disconnect();
				vTaskDelay(1000);
				printf("Timeout doorbellpoweroff\r\n");
				extern void uart_send_str(serial_t *sobj, char *pstr);
				extern serial_t    sobj;
				uart_send_str(&sobj, "IPCdoorbellpoweroff");
				vTaskDelay(100);
				uart_send_str(&sobj, "doorbellpoweroff");
				vTaskDelay(100);
			}

#if USE_EVB
			if (rltk_wlan_running(0)) {
				wifi_off();
			}

			isp_suspend_func(NULL);
			vTaskDelay(1000);
			send_icc_cmd_poweroff();
#endif
#endif
			break;
		}
	}

	//stop_all_streaming();
	//close_all_context();
	//vTaskDelete(NULL);
}

void media_control_init()
{
	if (xTaskCreate(media_contol_thread, ((const char *)"mmf_ctr"), 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n media_contol_thread: Create Task Error\n");
	}
}

void doorbell_init_function(void *parm)
{
	/* start mmf& p2p application */
	rtw_init_sema(&doorbell_handle_sema, 0);
	media_control_init();

	/* start video rate control */
	//media_rate_control();

	/* IO initial      */
	//AMP init
	gpio_init(&gpio_amp, AMP_PIN);
	gpio_dir(&gpio_amp, PIN_OUTPUT);
	gpio_mode(&gpio_amp, PullNone);
	gpio_write(&gpio_amp, 0);

#ifndef DOORBELL_CHIME_MODE_ENABLE
	//LED inital
	gpio_init(&gpio_led_red, BTN_RED); //BUTTON RED
	gpio_dir(&gpio_led_red, PIN_OUTPUT);
	gpio_mode(&gpio_led_red, PullNone);
	gpio_write(&gpio_led_red, 0);

	gpio_init(&gpio_led_blue, BTN_BLUE);//BUTTON BLUE
	gpio_dir(&gpio_led_blue, PIN_OUTPUT);
	gpio_mode(&gpio_led_blue, PullNone);
	gpio_write(&gpio_led_blue, 1);
#else
	//LED inital
	gpio_init(&gpio_led2, LED2); //LED2
	gpio_dir(&gpio_led2, PIN_OUTPUT);
	gpio_mode(&gpio_led2, PullNone);
	gpio_write(&gpio_led2, 0);
#endif

#if defined(DOORBELL_CHIME_BROAD)
#if USE_EVB
	amp_gpio_enable(0);
#else
	amp_gpio_enable(1);
#endif
#endif

	/* icc handler initial */
#if USE_EVB
	icc_task_func();
	uart_cmd_task_func();
#else
#if USE_ICC_CMD
	icc_task_func();
#else
	uart_cmd_task_func();
	init_AP1511B_sensor_service();
#endif
#endif

	/* doorbell moniter thread*/
	doorbell_status_monitor();
}