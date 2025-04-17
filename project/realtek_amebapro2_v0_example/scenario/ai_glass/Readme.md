
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
	.video_params[STREAM_V1].width  = 176,
	.video_params[STREAM_V1].height = 144,
	.video_params[STREAM_V1].bps = 1024 * 1024,
	.video_params[STREAM_V1].fps = 15,
	.video_params[STREAM_V1].gop = 15,

	.video_params[STREAM_V2].width = sensor_params[USE_SENSOR].sensor_width,
	.video_params[STREAM_V2].height = sensor_params[USE_SENSOR].sensor_height,
	.video_params[STREAM_V2].bps = 2 * 1024 * 1024,
	.video_params[STREAM_V2].fps = sensor_params[USE_SENSOR].sensor_fps,
	.video_params[STREAM_V2].gop = sensor_params[USE_SENSOR].sensor_fps,
	.video_params[STREAM_V2].rc_mode = 2,
	.video_snapshot[STREAM_V2] = 1,

	.video_params[STREAM_V4].fcs = 0,

	.video_enable[STREAM_V1] = 1,
	.video_enable[STREAM_V2] = 1,
	.video_enable[STREAM_V3] = 0,
	.video_enable[STREAM_V4] = 0,
- modify the following setting in the user_boot_config_init (other keeping the same)
    video_boot_stream.init_isp_items.init_brightness = 0;    //Default:0
    video_boot_stream.init_isp_items.init_contrast = 50;     //Default:50
    video_boot_stream.init_isp_items.init_flicker = 1;        //Default:1
    video_boot_stream.init_isp_items.init_mirrorflip = 0xf3;  //Mirror and flip
    video_boot_stream.init_isp_items.init_saturation = 50;    //Default:50
    video_boot_stream.init_isp_items.init_wdr_level = 50;     //Default:50
    video_boot_stream.init_isp_items.init_wdr_mode = 0;       //Default:0

5. \project\realtek_amebapro2_v0_example\inc\sensor.h
- in sen_id, replace SENSOR_GC2053 by SENSOR_SC5356
- set USE_SENSOR to SENSOR_SC5356
- in manual_iq, replace iq_gc2053 by iq_sc5356
- set ENABLE_FCS to 1

6. \component\file_system\fatfs\fatfs_ramdisk_api.c
- set a proper size for RAM_DISK_SIZE which is the size of the ram disk
- Here we set the ram disk size to 1024*1024*2 since we only need 2M to store a 720P jpeg (720 * 1280 * 3 / 2 ~ 1.3MB)
1024*1024*2

7. \component\file_system\fatfs\r0.14\diskio.c
- DWORD get_fattime(void) modified to weak function __attribute__((weak)) DWORD get_fattime(void)

## Config for in this scenario ##
1. main.c
- UART_LOG_BAUDRATE: uart baudrate for the debug log, default 3000000
Note: this baudrate will have strong influence to the process time

2. ai_glass_media.h
- FLASH_AI_SNAPSHOT_DATA: the flash location to store ai snapshot parameters
- FLASH_RECORD_DATA: the flash location to store lifetime record parameters
- FLASH_SNAPSHOT_DATA: the flash location to store lifetime snapshot parameters

3. ai_glass_initialize.c
- ENABLE_TEST_CMD: The test command for the scenario will be opened, default 1
- ENABLE_DISK_MASS_STORAGE: For the tester to get EMMC disk through usb mass storage device, default 0
- ENABLE_VIDEO_SEND_LATER: For the tester to send video end command later and need to enable both ENABLE_TEST_CMD and ENABLE_DISK_MASS_STORAGE, default 0
When enabling ENABLE_VIDEO_SEND_LATER, the process will be block after recording or lifetime snapshot. User need to enter the command "SENDVIDEOEND"
- DISK_PLATFORM: the external platform to store lifetime snapshot or recording file, default VFS_INF_EMMC
- UART_TX: the uart tx pin to communicate with other soc, default PA_2
- UART_RX: the uart rx pin to communicate with other soc, default PA_3
- UART_BAUDRATE: the uart baudrate to communicate with other soc, default 2000000
Note: this baudrate will have strong influence to the process time

4. lifetime_recording_initialize.c
- ENABLE_GET_GSENSOR_INFO: During the lifetime recording, the gyro sensor will start to capture the data, default 1
- AUDIO_SAMPLE_RATE: audio samplerate, default 16000
- AUDIO_SRC: Audio interface during recording, default I2S_INTERFACE
- AUDIO_I2S_ROLE: I2S role when using the i2s interface, default I2S_MASTER

5. wlan_scenario.h
- AI_GLASS_AP_IP_ADDRx: the IP address when enabling AP mode for 8735, default 192.168.43.1
- AI_GLASS_AP_NETMASK_ADDRx: the gateway mask when enabling AP mode for 8735, default 255.255.255.0
- AI_GLASS_AP_GW_ADDR: the gateway address when enabling AP mode for 8735, default 192.168.43.1
- AI_GLASS_AP_SSID: the ssid when enabling AP mode for 8735, default AI_GLASS_AP
- AI_GLASS_AP_PASSWORD: the password when enabling AP mode for 8735, default 12345678
- AI_GLASS_AP_CHANNEL: the channel when enabling AP mode for 8735, default 6

6. uart_service.c
- UART_CMD_PRIORITY: the priority for the normal uart command sending from the other soc, default 5
- UART_CRITICAL_PRIORITY: the priority for the critical uart command sending from the other soc, default 7
- UART_ACK_PRIORITY: the priority for the thread to process the ack responding, default 8
- UART_THREAD_NUM: the total thread num to process the normal uart command, default 3

7. uart_service.h
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