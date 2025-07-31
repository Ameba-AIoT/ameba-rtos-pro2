# Amazon KVS WebRTC demo on AmebaPro2 #

## Download the necessary source code from Github
- Go to `project/realtek_amebapro2_v0_example/src/amazon_kvs/lib_amazon`
    ```
    cd project/realtek_amebapro2_v0_example/src/amazon_kvs/lib_amazon
    ```
- Clone the following repository for KVS webrtc
	- amazon-kinesis-video-streams-webrtc-sdk-c
    ```
    git clone -b webrtc-on-freertos-wss-1220-R https://github.com/ambiot-mini/amazon-kinesis-video-streams-webrtc-sdk-c.git
    ```
    - cisco/libsrtp
    ```
    git clone -b webrtc-on-freertos https://github.com/ambiot-mini/libsrtp.git
    ```
    - tatsuhiro-t/wslay
    ```
    git clone https://github.com/ambiot-mini/wslay.git
    ```
    - sctplab/usrsctp
    ```
    git clone -b webrtc-on-freertos https://github.com/ambiot-mini/usrsctp.git
    ```
    - nodejs/llhttp
    ```
    git clone -b release/v6.0.6 https://github.com/nodejs/llhttp.git
    ```

## Set mbedtls version
- In KVS webrtc project, we have to use some function in mbedtls-2.16.6  
- Set mbedtls version to 2.16.6 in `project/realtek_amebapro2_v0_example/GCC-RELEASE/config.cmake`
    ```
    set(mbedtls "mbedtls-2.16.6")
    ```

## Modify lwipopts.h
- Modify lwipopts.h in `component/lwip/api/`
    ```
    #define LWIP_IPV6       1
    ```

## Congiure the example
- configure AWS key channel name in `component/example/kvs_webrtc_mmf/sample_config_webrtc.h`
    ```
    /* Enter your AWS KVS key here */
    #define KVS_WEBRTC_ACCESS_KEY   "xxxxxxxxxx"
    #define KVS_WEBRTC_SECRET_KEY   "xxxxxxxxxx"

    /* Setting your signaling channel name */
    #define KVS_WEBRTC_CHANNEL_NAME "xxxxxxxxxx"
    ```
- configure video parameter in `component/example/kvs_webrtc_mmf/example_kvs_webrtc_mmf.c`
    ```
    ...
    #define V1_RESOLUTION VIDEO_HD
    #define V1_FPS 30
    #define V1_GOP 30
    #define V1_BPS 1024*1024
    ```

## Prepare cert
- Put the cert to SD card (`component/example/kvs_webrtc_mmf/certs/cert.pem`), and set its path in `sample_config_webrtc.h`
    ```
    /* Cert path */
    #define KVS_WEBRTC_ROOT_CA_PATH "sd:/cert.pem"
    ```

## Select camera sensor

- Check your camera sensor model, and define it in <AmebaPro2_SDK>/project/realtek_amebapro2_v0_example/inc/sensor.h
    ```
    #define USE_SENSOR SENSOR_GC2053
    ```
    
## Using AWS-IoT credential (optional)

- Testing Amazon KVS WebRTC with IAM user key(AK/SK) is easy but it is not recommended, user can refer the following links to set up webrtc with AWS-IoT credential
  - With AWS IoT Thing credentials, it can be managed more securely.(https://iotlabtpe.github.io/Amazon-KVS-WebRTC-WorkShop/lab/lab-4.html)
  - Script for generate iot credential: https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c/blob/master/scripts/generate-iot-credential.sh

## Build the project
- run following commands to build the image with option `-DEXAMPLE=kvs_webrtc_mmf`
    ```
    cd project/realtek_amebapro2_v0_example/GCC-RELEASE
    mkdir build
    cd build
    cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=kvs_webrtc_mmf
    cmake --build . --target flash
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
    wifi connected
    wifi connected
    === KVS Example Start ===
    [KVS WebRTC module]: webrtc branch name = webrtc-on-freertos-wss-0323-R
    [KVS WebRTC module]: webrtc commit hash = c5f1ee6763ede84a1b33e75665dbfdc9b1e69237
    [KVS WebRTC module]: waiting get epoch timer
    vfs_mutex init
    fatfs register
    sd mount
    Register disk driver to Fatfs.
    FATFS Register: disk driver 0
    SD_Init 0
    part_count = 0
    [WebRTC] Starting
    initializing the app with channel(webrtc_iot_thing)
    Difference between current time and iot expiration is 3599
    Iot credential expiration time 1649404892
    The number of threads: (1/1)
    AUDIO_OPUS
    The intialization of the media source is completed successfully
    The initialization of WebRTC  is completed successfully
    Creating Signaling Client Sync
    The number of threads: (2/2)
    The thread of handling msg is up
    Signaling client state changed to 1 - 'New'
    Signaling client state changed to 2 - 'Get Security Credentials'
    Difference between current time and iot expiration is 3600
    Iot credential expiration time 1649404894
    Signaling client state changed to 3 - 'Describe Channel'
    Skip the call of describing the channel
    Signaling client state changed to 5 - 'Get Channel Endpoint'
    Skip the call of getting the endpoint
    Signaling client state changed to 6 - 'Get ICE Server Configuration'
    The expiration of ice config: 16494015659528950, ttl: 300
    Signaling client state changed to 7 - 'Ready'
    Signaling client created successfully
    Signaling Client Connect Sync
    Signaling client state changed to 8 - 'Connecting'
    The number of threads: (3/3)
    Wss client is up
    Signaling client state changed to 9 - 'Connected'
    The bootup time of webrtc is 5367 ms
    wss ping ==>
    <== wss pong
    ...
    ```

## Validate result
- we can use KVS WebRTC Test Page to test the result.  
  https://awslabs.github.io/amazon-kinesis-video-streams-webrtc-sdk-js/examples/index.html
- Please refer `test_page_setup.jpg` to set up the test page.
