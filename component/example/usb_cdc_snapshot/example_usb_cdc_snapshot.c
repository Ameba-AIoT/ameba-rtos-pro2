#include "FreeRTOS.h"
#include "task.h"
#include "basic_types.h"
#include "platform_opts.h"

#include "usb.h"
#include "cdc/inc/usbd_cdc.h"

#include <stdio.h>
#include "vfs.h"

#define VIDEO_ENABLE

//#define JPEG_MODE

static int acm_receive(void *buf, u16 length);
usbd_cdc_acm_usr_cb_t cdc_acm_usr_cb = {
	.init = NULL,
	.deinit = NULL,
	.receive = acm_receive,
#if (CONFIG_USDB_CDC_ACM_APP == ACM_APP_ECHO_ASYNC)
	.transmit_complete = NULL,//acm_transmit_complete,
#endif
};
unsigned int snap_checksum = 0;

////////////////////////YUV SNAPSHOT//////////////////////////////////////////
#include "module_video.h"
//#include "mmf2_pro2_video_config.h"
#include "sensor.h"

#define V1_CHANNEL 0
#define V1_FPS 5
#define V1_GOP 5
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR

#ifdef JPEG_MODE
#define VIDEO_TYPE 0x02//VIDEO_JPEG
#else
#define VIDEO_TYPE VIDEO_NV12
#endif

struct yuv_snapshot_context {
	uint8_t snap_flag;
	_sema snapshot_sema;
	uint32_t isp_len;
	uint32_t isp_addr;
	hal_video_adapter_t  *v_adp;
	int type;
};

struct yuv_snapshot_context *yuv_ctx = NULL;
static void yuv_output_cb(void *param1, void  *param2, uint32_t arg)
{
	enc2out_t *enc2out = (enc2out_t *)param1;
	struct yuv_snapshot_context *yuv_ctx = (struct yuv_snapshot_context *)arg;
	if ((enc2out->codec & (CODEC_H264 | CODEC_HEVC | CODEC_JPEG)) != 0) {
		if (yuv_ctx->snap_flag) {
			yuv_ctx->isp_addr = (uint32_t)enc2out->jpg_addr;
			rtw_up_sema(&yuv_ctx->snapshot_sema);
			yuv_ctx->snap_flag = 0;
			yuv_ctx->isp_len = enc2out->jpg_len;//enc2out->enc_len;
		} else {
			video_encbuf_release(enc2out->ch, CODEC_JPEG, enc2out->jpg_len);
		}
	} else if ((enc2out->codec & (CODEC_NV12 | CODEC_RGB | CODEC_NV16)) != 0) {
		if (yuv_ctx->snap_flag) {
			yuv_ctx->isp_addr = (uint32_t)enc2out->isp_addr;
			rtw_up_sema(&yuv_ctx->snapshot_sema);
			yuv_ctx->snap_flag = 0;
			yuv_ctx->isp_len = enc2out->width * enc2out->height * 3 / 2;;//enc2out->enc_len;
		} else {
			video_ispbuf_release(enc2out->ch, (uint32_t)enc2out->isp_addr);
		}
	}

}

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.bps = V1_BPS,
	.fps = V1_FPS,
	.gop = V1_GOP,
	.jpeg_qlevel = 9,
	.rc_mode = V1_RCMODE,
	.direct_output = 1
};

static void yuv_snapshot_init(void *ctx)
{
	struct yuv_snapshot_context *yuv_ctx = (struct yuv_snapshot_context *)ctx;
	int iq_addr, sensor_addr;
	isp_info_t info;

	int voe_heap_size = video_buf_calc(1, video_v1_params.width, video_v1_params.height, V1_BPS, 1,
									   0, 0, 0, 0, 0,
									   0, 0, 0, 0, 0,
									   0, 0, 0);

	int sensor_id_value = 0;
	for (int i = 0; i < SENSOR_MAX; i++) {
		if (sen_id[i] == USE_SENSOR) {
			sensor_id_value = i;
			break;
		}
	}
	voe_get_sensor_info(sensor_id_value, &iq_addr, &sensor_addr);
	//voe_get_sensor_info(USE_SENSOR, &iq_addr, &sensor_addr);

	yuv_ctx->v_adp = video_init(iq_addr, sensor_addr);

	rtw_init_sema(&yuv_ctx->snapshot_sema, 0);
}

static void yuv_snapshot_start(void *ctx)
{
	int ret = 0;
	struct yuv_snapshot_context *yuv_ctx = (struct yuv_snapshot_context *)ctx;
	ret = video_open(&video_v1_params, yuv_output_cb, (void *)ctx);
	if (ret < 0) {
		printf("Please check sensor fisrt, the ID is %d\r\n", USE_SENSOR);
		while (1) {
			vTaskDelay(100);
		}
	}
	video_ctrl(video_v1_params.stream_id, VIDEO_NV12_OUTPUT, 0);
}

static unsigned char *yuv_snapshot_get(void *ctx)
{
	struct yuv_snapshot_context *yuv_ctx = (struct yuv_snapshot_context *)ctx;
	yuv_ctx->snap_flag = 1;
	video_ctrl(video_v1_params.stream_id, VIDEO_NV12_OUTPUT, 1);
	if (rtw_down_timeout_sema(&yuv_ctx->snapshot_sema, 1000)) {
		video_ctrl(video_v1_params.stream_id, VIDEO_NV12_OUTPUT, 0);
		return (unsigned char *)yuv_ctx->isp_addr;
	} else {
		printf("Can't get the buffer\r\n");
		return 0;
	}
}

static void yuv_snapshot_release(void *ctx)
{
	struct yuv_snapshot_context *yuv_ctx = (struct yuv_snapshot_context *)ctx;
	video_ispbuf_release(video_v1_params.stream_id, (uint32_t)yuv_ctx->isp_addr);
	video_ctrl(video_v1_params.stream_id, VIDEO_NV12_OUTPUT, 0);
}

static void yuv_snapshot_close(void *ctx)
{
	struct yuv_snapshot_context *yuv_ctx = (struct yuv_snapshot_context *)ctx;
	video_ctrl(video_v1_params.stream_id, VIDEO_NV12_OUTPUT, 0);
	video_close(video_v1_params.stream_id);
}


static void jpeg_snapshot_start(void *ctx)
{
	int ret = 0;
	struct yuv_snapshot_context *yuv_ctx = (struct yuv_snapshot_context *)ctx;
	ret = video_open(&video_v1_params, yuv_output_cb, (void *)ctx);
	if (ret < 0) {
		printf("Please check sensor fisrt, the ID is %d\r\n", USE_SENSOR);
		while (1) {
			vTaskDelay(100);
		}
	}
	video_ctrl(video_v1_params.stream_id, VIDEO_JPEG_OUTPUT, 0);
}

static unsigned char *jpeg_snapshot_get(void *ctx)
{
	struct yuv_snapshot_context *yuv_ctx = (struct yuv_snapshot_context *)ctx;
	yuv_ctx->snap_flag = 1;
	video_ctrl(video_v1_params.stream_id, VIDEO_JPEG_OUTPUT, 1);
	if (rtw_down_timeout_sema(&yuv_ctx->snapshot_sema, 1000)) {
		return (unsigned char *)yuv_ctx->isp_addr;
	} else {
		printf("Can't get the buffer\r\n");
		return 0;
	}
}

static void jpeg_snapshot_release(void *ctx)
{
	struct yuv_snapshot_context *yuv_ctx = (struct yuv_snapshot_context *)ctx;
	video_encbuf_release(video_v1_params.stream_id, CODEC_JPEG, yuv_ctx->isp_len);
}

static void jpeg_snapshot_close(void *ctx)
{
	struct yuv_snapshot_context *yuv_ctx = (struct yuv_snapshot_context *)ctx;
	video_ctrl(video_v1_params.stream_id, VIDEO_JPEG_OUTPUT, 0);
	video_close(video_v1_params.stream_id);
}

////////////////////////YUV SNAPSHOT//////////////////////////////////////////


//Loop back mode
#define CDC_LOG_SERVICE_BUFLEN 128
//static unsigned char log_buf[128] = {0};
static int log_current = 0;
static short buf_count = 0;

#define KEY_CTRL_D      0x4
#define KEY_NL			0xa // '\n'
#define KEY_ENTER		0xd // '\r'
#define KEY_BS    		0x8
#define KEY_ESC    		0x1B
#define KEY_LBRKT  		0x5B
#define STR_END_OF_MP_FORMAT	"\r\n\r\r#"

static char log_buf[CDC_LOG_SERVICE_BUFLEN];
static unsigned int cmd_history_count = 0;


#define CMD_HISTORY_LEN	4	// max number of executed command saved
static char cmd_history[CMD_HISTORY_LEN][CDC_LOG_SERVICE_BUFLEN];


typedef void (*shellFunction)(void);

typedef struct {
	uint8_t *name;
	shellFunction function;
	uint8_t *desc;
} shell_cmd;

#include <stdarg.h>
#define APP_TX_DATA_SIZE 1024
unsigned char UserTxBufferFS[APP_TX_DATA_SIZE];

static void usb_cdc_printf(const char *format, ...)
{
	va_list args;
	uint32_t length;
	va_start(args, format);
	length = vsnprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, (char *)format, args);
	va_end(args);
	usbd_cdc_acm_sync_transmit_data(UserTxBufferFS, length); //(UserTxBufferFS, length);
}

void shelltest1(void)
{
	unsigned char buf[32];
	int rc = 0;
	printf("%s\r\n", __FUNCTION__);
	usb_cdc_printf("%s\r\n", __FUNCTION__);
}

void shelltest2(void)
{
	printf("%s\r\n", __FUNCTION__);
	usb_cdc_printf("%s\r\n", __FUNCTION__);
}

void shelltest3(void)
{
	printf("%s\r\n", __FUNCTION__);
	usb_cdc_printf("%s\r\n", __FUNCTION__);
}

void shelltest4(void)
{
	printf("%s\r\n", __FUNCTION__);
	usb_cdc_printf("%s\r\n", __FUNCTION__);
}

void shelltest5(void)
{
	printf("%s\r\n", __FUNCTION__);
	usb_cdc_printf("%s\r\n", __FUNCTION__);
}

void shelltest6(void)
{
	printf("%s\r\n", __FUNCTION__);
	usb_cdc_printf("%s\r\n", __FUNCTION__);
}

void shelltest7(void)
{
	int i = 0;
	int count = 0;
	int rc = 0;
	int sum = 0;
	unsigned char *raw_data = NULL;
#ifdef JPEG_MODE
	raw_data = (unsigned char *)jpeg_snapshot_get(yuv_ctx);
#else
	raw_data = (unsigned char *)yuv_snapshot_get(yuv_ctx);
	yuv_snapshot_close(yuv_ctx);
#endif

	if (raw_data != NULL) {
		printf("Get the frame\r\n");
		snap_checksum = 0;
		for (i = 0; i < yuv_ctx->isp_len; i++) {
			snap_checksum += raw_data[i];
		}
#ifdef JPEG_MODE
		usb_cdc_printf("snapshot=%u,%u,%s\r\n", yuv_ctx->isp_len, snap_checksum, "jpg");
		printf("snapshot=%u,%u,%s\r\n", yuv_ctx->isp_len, snap_checksum, "jpg");
#else
		usb_cdc_printf("snapshot=%u,%u,%s\r\n", yuv_ctx->isp_len, snap_checksum, "nv12");
		printf("snapshot=%u,%u,%s\r\n", yuv_ctx->isp_len, snap_checksum, "nv12");
#endif
		count = yuv_ctx->isp_len / 1024;
		for (i = 0; i < count; i++) {
			rc = usbd_cdc_acm_sync_transmit_data((void *)(raw_data + i * 1024), 1024);
			vTaskDelay(1);
			if (rc < 0) {
				printf("Not enough buffer\r\n");
			}
			sum += 1024;
		}
		if (yuv_ctx->isp_len % 1024) {
			printf("remain %d\r\n", yuv_ctx->isp_len % 1024);
			usbd_cdc_acm_sync_transmit_data((void *)(raw_data + i * 1024), yuv_ctx->isp_len % 1024);
			sum += (yuv_ctx->isp_len % 1024);
		}
		usb_cdc_printf("\r\n");
		printf("finish %d\r\n", sum);
#ifdef JPEG_MODE
		jpeg_snapshot_release(yuv_ctx);
#else
		yuv_snapshot_release(yuv_ctx);
		yuv_snapshot_start(yuv_ctx);
#endif
	} else {
		printf("Can't get the frame\r\n");
	}
}

shell_cmd shellCommandList[] = {
	/*command               function                description*/
	{(uint8_t *)"test1",   shelltest1, (uint8_t *)"test1 shell"},
	{(uint8_t *)"test2",   shelltest2, (uint8_t *)"test2 shell"},
	{(uint8_t *)"test3",   shelltest3, (uint8_t *)"test3 shell"},
	{(uint8_t *)"test4",   shelltest4, (uint8_t *)"test4 shell"},
	{(uint8_t *)"test5",   shelltest5, (uint8_t *)"test5 shell"},
	{(uint8_t *)"test6",   shelltest6, (uint8_t *)"test6 shell"},
	{(uint8_t *)"snapshot", shelltest7, (uint8_t *)"snapshot shell"},
};


_sema usd_cdc_sema;

static void example_shell_cmd_thread(void *param)
{
	int flag = 0;
	rtw_init_sema(&usd_cdc_sema, 0);
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
	while (1) {
		flag = 0;
		rtw_down_sema(&usd_cdc_sema);
		for (int i = sizeof(shellCommandList) / sizeof(shell_cmd) - 1; i >=  0; i--) {
			if (strcmp((const char *)log_buf, (const char *)shellCommandList[i].name) == 0) {
				flag = 1;
				shellCommandList[i].function();
				break;
			}
		}
		if (!flag) {
			usb_cdc_printf("The command is not support\r\n");
		}
	}
	vTaskDelete(NULL);
}

static int acm_receive(void *buf, u16 length)
{
	int ret = 0;
	u16 len = length;
	int i = 0;

	unsigned char rc = 0;
	unsigned char *str = (unsigned char *)buf;
	static unsigned char temp_buf[CDC_LOG_SERVICE_BUFLEN] = "\0";
	static unsigned char combo_key = 0;
	static short buf_count = 0;
	static unsigned char key_enter = 0;
	static char cmd_history_index = 0;
	for (i = 0; i < len; i++) {
		rc = str[i];
		if (key_enter && rc == KEY_NL) {
			//serial_putc(sobj, rc);
			//return;
			goto EXIT;
		}

		if (rc == KEY_ESC) {
			combo_key = 1;
		} else if (rc == KEY_CTRL_D) {

		} else if (combo_key == 1) {
			if (rc == KEY_LBRKT) {
				combo_key = 2;
			} else {
				combo_key = 0;
			}
		} else if (combo_key == 2) {
			if (rc == 'A' || rc == 'B') { // UP or Down
				if (rc == 'A') {
					cmd_history_index--;
					if (cmd_history_index < 0) {
						cmd_history_index = (cmd_history_count > CMD_HISTORY_LEN) ? CMD_HISTORY_LEN - 1 : (cmd_history_count - 1) % CMD_HISTORY_LEN;
					}
				} else {
					cmd_history_index++;
					if (cmd_history_index > (cmd_history_count > CMD_HISTORY_LEN ? CMD_HISTORY_LEN - 1 : (cmd_history_count - 1) % CMD_HISTORY_LEN)) {
						cmd_history_index = 0;
					}
				}

				if (cmd_history_count > 0) {
					buf_count = strlen((char const *)temp_buf);
					memset(temp_buf, '\0', buf_count);
					while (--buf_count >= 0) {
						unsigned char temp[3] = {0};
						temp[0] = KEY_BS;
						temp[1] = 0x20;
						temp[2] = KEY_BS;
						usbd_cdc_acm_sync_transmit_data((void *)temp, sizeof(temp));
					}
					usb_cdc_printf("%s", cmd_history[cmd_history_index % CMD_HISTORY_LEN]);
					strcpy((char *)temp_buf, cmd_history[cmd_history_index % CMD_HISTORY_LEN]);
					buf_count = strlen((char const *)temp_buf);
				}
			}

			// exit combo
			combo_key = 0;
		} else if (rc == KEY_ENTER) {
			key_enter = 1;
			if (buf_count > 0) {
				rc = KEY_NL;
				usbd_cdc_acm_sync_transmit_data((void *)&rc, 1);
				rc = KEY_ENTER;
				usbd_cdc_acm_sync_transmit_data((void *)&rc, 1);
				memset(log_buf, '\0', CDC_LOG_SERVICE_BUFLEN);
				strncpy(log_buf, (char *)&temp_buf[0], buf_count);
				rtw_up_sema(&usd_cdc_sema);
				memset(temp_buf, '\0', buf_count);

				memset(cmd_history[((cmd_history_count) % CMD_HISTORY_LEN)], '\0', buf_count + 1);
				strcpy(cmd_history[((cmd_history_count++) % CMD_HISTORY_LEN)], log_buf);
				cmd_history_index = cmd_history_count % CMD_HISTORY_LEN;
				buf_count = 0;
			} else {
				usb_cdc_printf("%s", STR_END_OF_MP_FORMAT);
			}
		} else if (rc == KEY_BS) {
			if (buf_count > 0) {
				buf_count--;
				temp_buf[buf_count] = '\0';
				unsigned char temp[3] = {0};
				temp[0] = KEY_BS;
				temp[1] = 0x20;
				temp[2] = KEY_BS;
				usbd_cdc_acm_sync_transmit_data((void *)temp, sizeof(temp));
			} else {
				usb_cdc_printf("%s", STR_END_OF_MP_FORMAT);
			}
		} else {
			/* cache input characters */
			if (buf_count < (CDC_LOG_SERVICE_BUFLEN - 1)) {
				temp_buf[buf_count] = rc;
				buf_count++;
				usbd_cdc_acm_sync_transmit_data((void *)&rc, 1);
				key_enter = 0;
			} else if (buf_count == (CDC_LOG_SERVICE_BUFLEN - 1)) {
				temp_buf[buf_count] = '\0';
				usbd_cdc_acm_sync_transmit_data((void *)"ERROR\r\n", 7);
			}
		}
	}
	if (ret != 0) {
		printf("\nFail to transmit data: %d\n", ret);
	}
EXIT:
	return ret;
}

static void get_cdc_status(void) //Please
{
	printf("usb_insert %d\r\n", usb_insert_status()); //Check usb connetc
	printf("cdc_port_status %d\r\n", cdc_port_status()); //Check com port connect
}

void example_cdc_snapshot_thread(void *param)
{
	int status = 0;
	_usb_init();
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
	/////////////////snapshot///////////
	yuv_ctx = malloc(sizeof(struct yuv_snapshot_context));
	memset(yuv_ctx, 0, sizeof(struct yuv_snapshot_context));
	video_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	video_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
	video_v1_params.fps = sensor_params[USE_SENSOR].sensor_fps;
	video_v1_params.gop = sensor_params[USE_SENSOR].sensor_fps;
	yuv_snapshot_init(yuv_ctx);
#ifdef JPEG_MODE
	jpeg_snapshot_start(yuv_ctx);
#else
	yuv_snapshot_start(yuv_ctx);
#endif

	for (int i = 0; i < CMD_HISTORY_LEN; i++) {
		memset(cmd_history[i], '\0', CDC_LOG_SERVICE_BUFLEN);
	}

	status = wait_usb_ready();
	if (status != USBD_INIT_OK) {
		if (status == USBD_NOT_ATTACHED) {
			printf("\r\n NO USB device attached\n");
		} else {
			printf("\r\n USB init fail\n");
		}
		goto exit;
	}

	status = usbd_cdc_acm_init(0, 0, &cdc_acm_usr_cb);
	if (status) {
		printf("USB CDC driver load fail.\n");
	} else {
		printf("USB CDC driver load done, Available heap [0x%x]\n", xPortGetFreeHeapSize());
	}
	vTaskDelay(2000);
	get_cdc_status();

exit:
	vTaskDelete(NULL);
}


void example_usb_cdc_snapshot(void)
{
	if (xTaskCreate(example_cdc_snapshot_thread, ((const char *)"example_cdc_snapshot_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_cdc_thread) failed", __FUNCTION__);
	}
	if (xTaskCreate(example_shell_cmd_thread, ((const char *)"example_shell_cmd_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_shell_cmd_thread) failed", __FUNCTION__);
	}
}