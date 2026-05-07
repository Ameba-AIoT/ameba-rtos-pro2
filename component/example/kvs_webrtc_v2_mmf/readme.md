# Amazon KVS WebRTC V2 demo on AmebaPro2 #

## Download the necessary source code from Script
- Go to `component/example/kvs_webrtc_v2_mmf`
    ```
    ./fetch_aws_lib.sh
    ```
- This script will download the neccessary libraries for aws webrtc to `project/realtek_amebapro2_v0_example/src/amazon_kvs/lib_amazon_v2/libraries`
	- Reference: https://github.com/awslabs/freertos-webrtc-reference-on-amebapro-for-amazon-kinesis-video-streams/tree/main/libraries

## Set mbedtls version
- In KVS webrtc project, we have to use some function in mbedtls, we support mbedtls-2.16.6 or mbedtls-3.0.0
- Set mbedtls version to 2.16.6 or 3.0.0 in `project/realtek_amebapro2_v0_example/GCC-RELEASE/config.cmake`
    ```
    set(mbedtls "mbedtls-2.16.6") or set(mbedtls "mbedtls-3.0.0")
    ```

## Modify lwipopts.h
- Modify lwipopts.h in `component/lwip/api/`
    ```
    #define LWIP_IPV6       1
    ```
- Modify lwipopts.h in `component/lwip/api/` (define ENABLE_AMAZON_COMMON)
    ```
    #ifndef ENABLE_AMAZON_COMMON
    #define ENABLE_AMAZON_COMMON
    #endif
    ```

## Modify mbedtls-3.0.0/include/mbedtls/mbedtls_config.h
For mbedtls-3.0.0, modify the following file
- Modify mbedtls_config.h in `mbedtls-3.0.0/include/mbedtls/`
    ```
    #define MBEDTLS_FS_IO
    ```

## Congiure the example
- configure AWS key channel name in `component/example/kvs_webrtc_v2_mmf/webrtc_v2_src/demo_config/demo_config.h`
    ```
    /* Enter your AWS KVS key here */
    #define AWS_ACCESS_KEY_ID   "xxxxxxxxxx"
    #define AWS_SECRET_ACCESS_KEY   "xxxxxxxxxx"

    /* Setting your signaling channel name */
    #define AWS_KVS_CHANNEL_NAME "xxxxxxxxxx"
    ```
- configure video parameter in `component/example/kvs_webrtc_v2_mmf/example_kvs_webrtc_v2_mmf.c`
    ```
    ...
    #define V1_RESOLUTION VIDEO_HD
    #define V1_FPS 30
    #define V1_GOP 30
    #define V1_BPS 1024*1024
    ```

## Select camera sensor

- Check your camera sensor model, and define it in <AmebaPro2_SDK>/project/realtek_amebapro2_v0_example/inc/sensor.h
    ```
    #define USE_SENSOR SENSOR_GC2053
    ```
    
## Using AWS-IoT credential (optional)

- User can refer the following links to set up webrtc with AWS-IoT credential
  - With AWS IoT Thing credentials, it can be managed more securely.(https://iotlabtpe.github.io/Amazon-KVS-WebRTC-WorkShop/lab/lab-4.html)
  - Script for generate iot credential: https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c/blob/master/scripts/generate-iot-credential.sh
  - Configure the AWS Credential Certificate in `component/example/kvs_webrtc_v2_mmf/webrtc_v2_src/demo_config/demo_config.h`
  ```
    #define AWS_CREDENTIALS_ENDPOINT "xxxxxxxxxx"
    #define AWS_IOT_THING_NAME "xxxxxxxxxx"
    #define AWS_IOT_THING_ROLE_ALIAS "xxxxxxxxxx"
    #define AWS_IOT_THING_CERT "xxxxxxxxxx"
    #define AWS_IOT_THING_PRIVATE_KEY "xxxxxxxxxx"
  ```

## Build the project
- run following commands to build the image with option `-DEXAMPLE=kvs_webrtc_v2_mmf`
    ```
    cd project/realtek_amebapro2_v0_example/GCC-RELEASE
    mkdir build
    cd build
    cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=kvs_webrtc_v2_mmf
    cmake --build . --target flash -j4
    ```

- use image tool to download the image to AmebaPro2 and reboot

- configure WiFi Connection  
    While runnung the example, you may need to configure WiFi connection by using these commands in uart terminal.  
    ```
    ATW0=<WiFi_SSID> : Set the WiFi AP to be connected
    ATW1=<WiFi_Password> : Set the WiFi AP password
    ATWC : Initiate the connection
    ```

- if everything works fine, you should see the following log
    ```
    ...
    [INFO][kvs_webrtc_: kvs_webrtc_v2_main_thread: 140] [KVS WebRTC v2 module]: === KVS WebRTC V2 Start ===
    [INFO][MasterTask: Master_Task: 171] Start Master_Task.
    [INFO][kvs_webrtc_: kvs_webrtc_v2_main_thread: 142] [KVS WebRTC v2 module]: kvs_webrtc_master_entry called, master task created
    [video_init] uvcd iq is null, use default.
    [video_init] uvcd SNR is null, use default.
    IQ:FW size (76042)
    sensor:date 2024/9/12 version:RTL8735B_VOE_1.5.7.0
    sensor:FW size (3996)
    sensor timestamp: 2024/09/12
    iq timestamp: 0000/00/00 00:00:00
    voe_heap malloc 0x708936e0, size 11124480
    ISP:1 ENC:1 H265:1 NN:1
    hal_voe_ready 0x0 0xbf1208
    voe   :RTL8735B_VOE_1.6.9.0
    sensor:RTL8735B_VOE_1.5.7.0
    hal   :RTL8735B_VOE_1.6.9.0
    load time sensor:52us iq:966us itcm:0us dtcm:0us ddr:0us ddr2:0us
    rc_version RC_v1
    [video_pre_init_procedure] START
    hal_voe_send2voe too long 50816 cmd 0x00000206 p1 0x00000000 p2 0x00000000
    set ASP print off
    [INFO][MasterTask: platform_init: 172] waiting get epoch timer
    [INFO][MasterTask: AppMediaSourcePort_Init: 34] [WebRTC Port] AppMediaSourcePort_Init called. Hardware is ready.
    [INFO][SigControll: DescribeSignalingChannel: 642] Describing Signaling Channel.
    [INFO][SigControll: tlsHandshake: 593] (Network connection 0x704b9778) TLS handshake successful.
    [INFO][SigControll: GetSignalingChannelEndpoints: 745] Getting Signaling Channel Endpoints.
    [Driver]: TSFValue = 75274143411, tsf = 0, shift_set= 0x8000, bcn int = 100

    [INFO][SigControll: tlsHandshake: 593] (Network connection 0x704b9778) TLS handshake successful.
    [INFO][SigControll: SignalingController_QueryIceServerConfigs: 1504] Quering Ice Server Configurations.
    [INFO][SigControll: SignalingController_QueryIceServerConfigs: 1517] Ice server configs expired, Starting Refresing Configs.
    [INFO][SigControll: tlsHandshake: 593] (Network connection 0x704b9778) TLS handshake successful.
    [INFO][SigControll: ConnectToWssEndpoint: 937] Connecting to Websocket Endpoint.
    [INFO][SigControll: GenerateWebSocketClientKey: 974] Base64 encode output length 24, original length 25
    [INFO][SigControll: tlsHandshake: 593] (Network connection 0x704bd8d0) TLS handshake successful.
    [INFO][SigControll: GenerateAcceptKey: 1015] Base64 encode output length 28, original length 29
    [INFO][SigControll: Websocket_Connect: 1641] Successfully connect with WSS endpoint wss://m-96978bec.kinesisvideo.ap-east-1.amazonaws.com?X-Amz-ChannelARN=arn:aws:kinesisvideo:ap-east-1:553661462376:channel/webrtc_iot_thing_aws_demo/1772602328547.
    [INFO][SigControll: OnSignalingConnectionStateChange: 1457] Unblock signaling connection barrier.
    [INFO][SigControll: LogSignalingInfo: 367] ======================================== Channel Info ========================================
    [INFO][SigControll: LogSignalingInfo: 368] Signaling Channel ARN: arn:aws:kinesisvideo:ap-east-1:553661462376:channel/webrtc_iot_thing_aws_demo/1772602328547
    [INFO][SigControll: LogSignalingInfo: 370] ======================================== Endpoints Info ========================================
    [INFO][SigControll: LogSignalingInfo: 371] HTTPS Endpoint: https://r-5897be86.kinesisvideo.ap-east-1.amazonaws.com
    [INFO][SigControll: LogSignalingInfo: 372] WSS Endpoint: wss://m-96978bec.kinesisvideo.ap-east-1.amazonaws.com
    [INFO][SigControll: LogSignalingInfo: 373] WebRTC Endpoint: N/A
    [INFO][SigControll: LogSignalingInfo: 376] ======================================== Ice Server List ========================================
    [INFO][SigControll: LogSignalingInfo: 377] Ice Server Count: 2
    [INFO][SigControll: LogSignalingInfo: 380] ======================================== Ice Server[0] ========================================
    [INFO][SigControll: LogSignalingInfo: 381]     TTL (seconds): 300
    [INFO][SigControll: LogSignalingInfo: 382]     User Name: 1773899174:djE6YXJuOmF3czpraW5lc2lzdmlkZW86YXAtZWFzdC0xOjU1MzY2MTQ2MjM3NjpjaGFubmVsL3dlYnJ0Y19pb3RfdGhpbmdfYXdzX2RlbW8vMTc3MjYwMjMyODU0Nw==
    [INFO][SigControll: LogSignalingInfo: 383]     Password: 62uCDkJi5XtxzgS4d2ZJHAVjQEVAvgsyoXkLvj7NstI=
    [INFO][SigControll: LogSignalingInfo: 384]     URI Count: 3
    [INFO][SigControll: LogSignalingInfo: 388]         URI: turn:16-163-154-237.t-492ce24c.kinesisvideo.ap-east-1.amazonaws.com:443?transport=udp
    [INFO][SigControll: LogSignalingInfo: 388]         URI: turns:16-163-154-237.t-492ce24c.kinesisvideo.ap-east-1.amazonaws.com:443?transport=udp
    [INFO][SigControll: LogSignalingInfo: 388]         URI: turns:16-163-154-237.t-492ce24c.kinesisvideo.ap-east-1.amazonaws.com:443?transport=tcp
    [INFO][SigControll: LogSignalingInfo: 380] ======================================== Ice Server[1] ========================================
    [INFO][SigControll: LogSignalingInfo: 381]     TTL (seconds): 300
    [INFO][SigControll: LogSignalingInfo: 382]     User Name: 1773899174:djE6YXJuOmF3czpraW5lc2lzdmlkZW86YXAtZWFzdC0xOjU1MzY2MTQ2MjM3NjpjaGFubmVsL3dlYnJ0Y19pb3RfdGhpbmdfYXdzX2RlbW8vMTc3MjYwMjMyODU0Nw==
    [INFO][SigControll: LogSignalingInfo: 383]     Password: tD2Af0OsvyjoPxI5JMdh8111vANMixuZv+onXLD6BQc=
    [INFO][SigControll: LogSignalingInfo: 384]     URI Count: 3
    [INFO][SigControll: LogSignalingInfo: 388]         URI: turn:16-163-149-36.t-492ce24c.kinesisvideo.ap-east-1.amazonaws.com:443?transport=udp
    [INFO][SigControll: LogSignalingInfo: 388]         URI: turns:16-163-149-36.t-492ce24c.kinesisvideo.ap-east-1.amazonaws.com:443?transport=udp
    [INFO][SigControll: LogSignalingInfo: 388]         URI: turns:16-163-149-36.t-492ce24c.kinesisvideo.ap-east-1.amazonaws.com:443?transport=tcp
    [INFO][SigControll: SendWebsocketPing: 1267] wss ping ==>
    [INFO][SigControll: HandleWslayControlMessage: 176] <== wss pong
    ...
    ```

## Validate result
- we can use KVS WebRTC Test Page to test the result.  
  https://awslabs.github.io/amazon-kinesis-video-streams-webrtc-sdk-js/examples/index.html
- Please refer `test_page_setup.jpg` to set up the test page.

## Reference
- For more information about the feature options of aws webrtc, please refer to

  https://github.com/awslabs/freertos-webrtc-reference-on-amebapro-for-amazon-kinesis-video-streams?tab=readme-ov-file#feature-options
