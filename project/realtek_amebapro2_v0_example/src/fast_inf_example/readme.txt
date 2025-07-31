[Description]

	This example demonstrates the process to get first MD & NN results faster.
	To get the first inference result faster, we recommend the media init flow as following.
		CH0 (recording) -> CH2 (motion detection) -> CH4 (object detction) -> Waiting MD Result ->
		Wlan Init or Resume -> CH1 (rtsp) -> Audio -> MP4 -> RTSP -> CH3 (snapshot) -> Snapshot

	Except running this video example, there are some extra step as following.
	1. 	Enable FCS Mode in "project\realtek_amebapro2_v0_example\inc\sensor.h"
		#define ENABLE_FCS      	1
	
	2. 	Enable ch0 FCS mode and modify resolution settings in "\component\video\driver\RTL8735B\video_user_boot.c"
		If VOE version is newer than 1.5.0.0, user can also enable ch2, ch4 FCS mode.
		video_boot_stream_t video_boot_stream = {
			.video_params[STREAM_V1].width = 1920,
			.video_params[STREAM_V1].height = 1080,
			.video_params[STREAM_V1].fps = 20,
			.video_drop_frame[STREAM_V1] = 3,
			.video_params[STREAM_V1].fcs = 1,
			.video_params[STREAM_V3].width = 128,
			.video_params[STREAM_V3].height = 128,
			.video_params[STREAM_V3].fps = 20,
			.video_params[STREAM_V3].fcs = 1,
			.video_params[STREAM_V4].width = 320,
			.video_params[STREAM_V4].height = 320,
			.video_params[STREAM_V4].fps = 20,
			.video_drop_frame[STREAM_V4] = 3, //support after VOE1.5.3.0
			.video_params[STREAM_V4].fcs = 1, //support after VOE1.5.0.0
			.fcs_channel = 3,//FCS_TOTAL_NUMBER
			.extra_video_enable = 1
		}
	
	3. 	Modify video slot settings of ch2(MD) and ch4(NN) in "\component\video\driver\RTL8735B\video_boot.c". 
		This step enable ch2 and ch4 to buffer 3 slots in FCS mode.
		static unsigned char video_boot_slot_num[5] = {2, 2, 3, 2, 3};

	4. 	Since this example contains wlan tcp resume, please disable wifi connection in "project\realtek_amebapro2_v0_example\src\main.c"
		void setup(void)
		{
		#if 0
		#if ENABLE_FAST_CONNECT
			wifi_fast_connect_enable(1);
		#else
			wifi_fast_connect_enable(0);
		#endif
			wlan_network();
		#endif
			...
		}

	5. 	If the MD resolution is small, the user can enable copying the NV12 image to the MD queue, which can make the NV12 image update smoother.
		However, it may cause extra memory and copy time.
		static video_params_t video_v3_params = {
			.use_static_addr = 0,
		};

	6. 	If the system is busy, user can enable waiting MD result function to get MD result faster.
		#define WAIT_MD_RESULT 1

	7. 	To get NN result faster, we apply lightweight NN models. This saves model loading time. However, this also results in some loss of recognition accuracy.
		{
			"msg_level":3,
			"PROFILE":["FWFS"],
			"FWFS":{
				"files":[
					"yolo_fastest_320x320"
				]
			},
			"yolo_fastest_320x320":{
				"name" : "yolo_fastest.nb",
				"source":"binary",
				"file":"yolo_fastest_1.1_320x320_u8.nb"
			}
		}

	8. 	To prevent time waste of printing log, user should modify baudrate into 3000000.
		Modify baudrate in "project\realtek_amebapro2_v0_example\src\main.c"
		void log_uart_port_init(int log_uart_tx, int log_uart_rx, uint32_t baud_rate)
		{
			baud_rate = 3000000;  //115200, 1500000, 3000000
		}

	9. 	Adjusting flash speed to 125MHz reduces startup time and loading nn model time. (only for nor flash)
		Modify define in "component/soc/8735b/fwlib/rtl8735b/include/hal_spic.h"
		#define HIGH_SPEED_FLASH FLASH_SPEED_125MHz

	10. User can use command 'UC=TIME' to check the time when the inference results were first obtained.
		Motion can be detected as early as frame 4. If motion cannot detect at frame 4, please modify drop frame setting in "video_user_boot.c" to make sure that video timestamp is continuous.


[Execution]

	1. Modify server_ip in "project\realtek_amebapro2_v0_example\src\fast_inf_example\wlan_tcp_resume.c"
    2. Config command: cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DFAST_INF_EXAMPLE=ON
    3. Build acommand: make flash 
    4. Download firmware and reboot AmebaPro2
	5. Create a SSL server (ex. openssl s_server -port xxxx -cert ./xxx.crt -key ./xxx.key -CAfile ./xxx.crt)
	6. Example will connect to SSL server and write application data
	7. Input "PS=wowlan" to enter sleep.
	8. FW will send keep alive data.
	9. If SSL server sends data to device or GPIO_A2 pull high, device will wakeup, and example will do wlan resume, tcp resume, ssl resume, and write application data.
	10. Use command 'UC=TIME' to check the time when the inference results were first obtained.