
# Realtek SDK Scenario #

This scenario is intended for a templete for ai glass scenario

## Scenario information ##

- ai glass scenario templete
- This templete shows samples for user to do uart communication, video snapshot, video recording, wifi and file management

## Update in SDK ##

1. \component\file_system\fatfs\fatfs_sdcard_api.c
- comment fatfs_sd_close in function fatfs_sd_init

2. \component\media\mmfv2\module_mp4.c
- update the define
    undefine FATFS_SD_CARD
    undefine FATFS_RAM
    define VFS_ENABLE

- in function mp4_destroy
    //vfs_user_unregister("sd", VFS_FATFS, VFS_INF_SD);

- in function mp4_create
    //vfs_init(NULL);
    memcpy(ctx->mp4_muxer->_drv, "aiglass:/", strlen("aiglass:/")); //Set tag
    ctx->mp4_muxer->vfs_format_enable = 1;//Enable the vfs format
    //if (vfs_user_register("sd", VFS_FATFS, VFS_INF_SD) < 0) {
        //goto mp4_create_fail;
    //}

3. \component\soc\8735b\misc\platform\user_boot.c
- set bl_log_cust_ctrl to DISABLE

4. \component\video\driver\RTL8735B\video_user_boot.c
- open flag ISP_CONTROL_TEST for the isp pre-setting

- modify the following setting in the video_boot_stream (other keeping the same)
	//video channel 0
	.video_enable[STREAM_V1] = 1,
	.video_snapshot[STREAM_V1] = 0,
	.video_drop_frame[STREAM_V1] = 0,
	.video_params[STREAM_V1] = {
		.stream_id = STREAM_ID_V1,
		.type = CODEC_H264,
		.resolution = 0,
		.width  = 176,
		.height = 144,
		.bps = 1024 * 1024,
		.fps = 15,
		.gop = 15,
		.rc_mode = 2,
		.minQp = 25,
		.maxQp = 48,
		.jpeg_qlevel = 0,
		.rotation = 0,
		.out_buf_size = V1_ENC_BUF_SIZE,
		.out_rsvd_size = 0,
		.direct_output = 0,
		.use_static_addr = 0,
		.fcs = 1 //Enable the fcs for channel 0
	},

	//video channel 1
	.video_enable[STREAM_V2] = 1,
	.video_snapshot[STREAM_V2] = 1,
	.video_drop_frame[STREAM_V2] = 0,
	.video_params[STREAM_V2] = {
		.stream_id = STREAM_ID_V2,
		.type = CODEC_H264,
		.resolution = 0,
		.width = sensor_params[USE_SENSOR].sensor_width,
		.height = sensor_params[USE_SENSOR].sensor_height,
		.bps = 20 * 1024 * 1024,
		.fps = sensor_params[USE_SENSOR].sensor_fps,
		.gop = sensor_params[USE_SENSOR].sensor_fps,
		.rc_mode = 2,
		.minQp = 25,
		.maxQp = 48,
		.jpeg_qlevel = 0,
		.rotation = 0,
		.out_buf_size = V2_ENC_BUF_SIZE,
		.out_rsvd_size = 0,
		.direct_output = 0,
		.use_static_addr = 0,
		.fcs = 0,
	},

	.video_enable[STREAM_V4] = 0,

- modify the following setting in the user_boot_config_init (other keeping the same)
	video_boot_stream.init_isp_items.enable = 1;
	video_boot_stream.init_isp_items.init_brightness = 0;    //Default:0
	video_boot_stream.init_isp_items.init_contrast = 50;     //Default:50
	video_boot_stream.init_isp_items.init_flicker = 1;        //Default:1
	video_boot_stream.init_isp_items.init_hdr_mode = 0;       //Default:0
	video_boot_stream.init_isp_items.init_mirrorflip = 0xf0;  //Mirror and flip
	video_boot_stream.init_isp_items.init_saturation = 50;    //Default:50
	video_boot_stream.init_isp_items.init_wdr_level = 50;     //Default:50
	video_boot_stream.init_isp_items.init_wdr_mode = 2;       //Default:0
	video_boot_stream.init_isp_items.init_mipi_mode = 0;	  //Default:0

5. \project\realtek_amebapro2_v0_example\inc\sensor.h
- in sensor_params, modify [SENSOR_SC5356]       = {2592, 1944, 24},
- in sen_id, replace SENSOR_GC2053 by SENSOR_SC5356
- set USE_SENSOR to SENSOR_SC5356
- in manual_iq, replace iq_gc2053 by iq_sc5356
- set ENABLE_FCS to 1

6. \component\file_system\fatfs\fatfs_ramdisk_api.c
- set a proper size for RAM_DISK_SIZE which is the size of the ram disk
- Here we set the ram disk size to 1024*1024*2 since we only need 2M to store a 720P jpeg (720 * 1280 * 3 / 2 ~ 1.3MB)
1024*1024*2

7. \component\soc\8735b\fwlib\rtl8735b\lib\source\ram\video\voe_bin
- user could replace the iq in this folder to get a better video vision
- under project\realtek_amebapro2_v0_example\scenario\ai_glass\src\iq, there is some pre-setting iq firmware for some sensor

## Config for in this scenario ##
1. main.c
- UART_LOG_BAUDRATE: uart baudrate for the debug log, default 3000000
Note: this baudrate will have strong influence to the process time

2. ai_glass_media.h
- FLASH_FW_SELECT_ADDR: the flash address to store which FW to boot up for "nor flash"
- FLASH_FW_SELECT_SIZE: the flash size remain for store switch FW to boot up for "nor flash"
- FLASH_AI_SNAP_BLOCK_BASE: the flash address to store ai snapshot parameters for "nor flash"
- FLASH_AI_SNAP_BLOCK_SIZE: the flash size remain for ai snapshot parameters for "nor flash"
- FLASH_REC_BLOCK_BASE: the flash address to store lifetime record parameters for "nor flash"
- FLASH_REC_BLOCK_SIZE: the flash size remain for lifetime record parameters for "nor flash"
- FLASH_LIFE_SNAP_BLOCK_BASE: the flash address to store lifetime snapshot parameters for "nor flash"
- FLASH_LIFE_SNAP_BLOCK_SIZE: the flash size remain for lifetime snapshot parameters for "nor flash"
Note: this flash arrangement is only for "nor flash"

3. ai_glass_initialize.c
- ENABLE_TEST_CMD: The test command for the scenario will be opened, default 1
- ENABLE_DISK_MASS_STORAGE: For the tester to get EMMC disk through usb mass storage device, default 0
- ENABLE_VIDEO_SEND_LATER: For the tester to send video end command later and need to enable both ENABLE_TEST_CMD and ENABLE_DISK_MASS_STORAGE, default 0
When enabling ENABLE_VIDEO_SEND_LATER, the process will be block after recording or lifetime snapshot. User need to enter the command "SENDVIDEOEND"
- DISK_PLATFORM: the external platform to store lifetime snapshot or recording file, default VFS_INF_EMMC
- UART_TX: the uart tx pin to communicate with other soc, default PA_2
- UART_RX: the uart rx pin to communicate with other soc, default PA_3
- UART_BAUDRATE: the uart baudrate to communicate with other soc, default 2000000
Note: this baudrate will have strong influence to the process time but need to sync with another mcu

4. lifetime_recording_initialize.c
- ENABLE_GET_GSENSOR_INFO: During the lifetime recording, the gyro sensor will start to capture the data, default 1
- AUDIO_SAMPLE_RATE: audio samplerate, default 16000
- AUDIO_SRC: Audio interface during recording, default I2S_INTERFACE
- AUDIO_I2S_ROLE: I2S role when using the i2s interface, default I2S_SLAVE

5. wlan_scenario.h
- AI_GLASS_AP_IP_ADDRx: the IP address when enabling AP mode for 8735, default 192.168.43.1
- AI_GLASS_AP_NETMASK_ADDRx: the gateway mask when enabling AP mode for 8735, default 255.255.255.0
- AI_GLASS_AP_GW_ADDRx: the gateway address when enabling AP mode for 8735, default 192.168.43.1
- AI_GLASS_AP_SSID: the ssid when enabling AP mode for 8735, default AI_GLASS_AP
- AI_GLASS_AP_PASSWORD: the password when enabling AP mode for 8735, default rtkaiglass
- AI_GLASS_AP_CHANNEL: the channel when enabling AP mode for 8735, default 40
- HTTPD_CONNECT_TIMEOUT: the http connect timeout in seconds, default 15

6. uart_service.c
- UART_CMD_PRIORITY: the priority for the normal uart command sending from the other soc, default 5
- UART_CRITICAL_PRIORITY: the priority for the critical uart command sending from the other soc, default 7
- UART_ACK_PRIORITY: the priority for the thread to process the ack responding, default 8
- UART_RECV_PRIORITY: the priority for the thread to parser the uart packet, default 8
- UART_THREAD_NUM: the total thread num to process the normal uart command, default 3

7. uart_service.h
- UART_PROTOCAL_VER: The version to sync with the other mcu
- UART_WIFI_IC_TYPE: The wifi IC type to sync with the other mcu
- SEND_DATA_SHOW: Show the data when the data send to the other soc through uart, default 0
- RECEIVE_ACK_SHOW: Show the data for the received ack from the other soc through uart, default 0
- SEND_ACK_SHOW: Show the data for the sending ack to the other soc through uart, default 0
- MAX_UART_QUEUE_SIZE: the queue length for normal uart command, which means the max uart command num can be waiting to do process, default 10
- MAX_CRITICAL_QUEUE_SIZE: the queue length for critical uart command, which means the max uart command num can be waiting to do process, default 10
- MAX_UARTACK_QUEUE_SIZE: the queue length for ack sending to or receving from the other soc, default 20

8. gyrosensor\gyrosensor_api.h
- GYROSENSOR_I2C_MTR_SDA: the data pin for gyro sensor i2c intreface,default PF_2
- GYROSENSOR_I2C_MTR_SCL: the clock pin for gyro sensor i2c intreface,default PF_1

9. media_filesystem.c
- ENABLE_FILE_TIME_FUNCTION: the filetime function (get_fattime) for the files will be updated by the function in media_filesystem.c

10. sliding_windows.h
- MAX_PAYLOAD_SIZE: the max payload size for a packet, default 1024, but create_sliding_window will update this value when setting up
- ACK_QUEUE_LENGTH: the queue length for the ack queue, default 10
- PAYLOAD_QUEUE_LENGTH: the queue length for the payload queue, default 10
- MAX_WINDOW_SIZE: the max sliding window size, default 8
- MIN_WINDOW_SIZE: the min sliding window size, default 1
- TIMEOUT_PERIOD: the period for one packet to wait for timeout, default 1000 (ms)
- MAX_RETRIES: the max retries to re-send one packet, default 30
- MAX_DUPLICATE_ACKS: the num of continuous duplicate ack will trigger the re-send function, default 3
- SLIDING_SEND_PRIORITY: the priority of the sliding windows send thread, default 7
- SLIDING_ACK_PRIORITY: the priority of the sliding windows ack received thread, default 6
- SLIDING_SEND_STACK: the stack size of the sliding windows send thread, default 4096
- SLIDING_ACK_STACK: the stack size of the sliding windows ack received thread, default 4096
- MAX_EXTEND_BOX_NUM: the max extend box num for a ack packets, default 4

11. wlan_scenario.c
- USE_HTTPS: Open the https or not, default 0
- HTTPS_SRC_CRT: the source certificate for https connection
- HTTPS_SRC_KEY: the source key for https connection
- HTTPS_CA_CRT: the certificate authority for https connection
- HTTP_PORT: the port when open http as server, default 8080
- HTTPS_PORT: the port when open https as server, default 8080
- HTTP_OTA_TEST: enable simple OTA flow for 8735

## 12M Sensor Init Configuration ##

1. Please modify sensor driver setting and enable FCS bootup.
	project\realtek_amebapro2_v0_example\inc\sensor.h
	static const unsigned char sen_id[SENSOR_MAX] = {
		SENSOR_DUMMY,
		SENSOR_IMX681,
		SENSOR_IMX681_12M,
		SENSOR_IMX681_12M_SEQ,
	};
	#define USE_SENSOR      	SENSOR_IMX681
	static const      char manual_iq[SENSOR_MAX][64] = {
		"iq",
		"iq_imx681",
		"iq_imx681_12m",
		"iq_imx681_12m_seq",
	};
	#define ENABLE_FCS      	1

2. Please enable video high resolution settings.
	component\video\driver\RTL8735B\video_api.h
	#define USE_VIDEO_HR_FLOW 1

3. Disable OSD function
	component\media\mmfv2\module_video.c
	#define OSD_ENABLE 0

## Enabling OTA progress report to 8773 ##

1. Please enable the flag to update OTA progress to 8773.
   component\soc\8735b\misc\platform\ota_8735b.h	
   #define UPDATE_UPGRADE_PROGRESS_TO_8773   1	