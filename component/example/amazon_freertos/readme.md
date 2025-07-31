# Amazon AWS IoT demo on AmebaPro2 #

This is a simple guide to amazon freertos demo, user can refer `AmebaPro2_Amazon_FreeRTOS_Getting_Started_Guide_v1.x.pdf` to get detailed instructions.

## Download the necessary source code from Github
- Go to `project/realtek_amebapro2_v0_example/src`
    ```
    cd project/realtek_amebapro2_v0_example/src
    ```
- Clone the following repository for AWS IoT
	- aws/amazon-freertos
    ```
    git clone --recurse-submodules -b amebaPro2-9.x-202107.00-LTS https://github.com/ambiot/amazon-freertos.git aws_iot_freertos_lts
    ```

## Modify FreeRTOSConfig.h

- Copy & paste below configurations to the end of FreeRTOSConfig.h in `project\realtek_amebapro2_v0_example\inc`:  
    ```
    /* Sets the length of the buffers into which logging messages are written - so
     * also defines the maximum length of each log message. */
    #define configLOGGING_MAX_MESSAGE_LENGTH            512

    /* Set to 1 to prepend each log message with a message number, the task name,
     * and a time stamp. */
    #define configLOGGING_INCLUDE_TIME_AND_TASK_NAME    1

    /* Map the FreeRTOS printf() to the logging task printf. */
    /* The function that implements FreeRTOS printf style output, and the macro
     * that maps the configPRINTF() macros to that function. */
    #define configPRINTF( X )    vLoggingPrintf X

    /* Non-format version thread-safe print. */
    #define configPRINT( X )     vLoggingPrint( X )

    /* Map the logging task's printf to the board specific output function. */
    #define configPRINT_STRING( X )         printf( X )

    #define iotconfigUSE_PORT_SPECIFIC_HOOKS
    ```

## Configure mbedtls setting

- In this project, we use mbedtls-2.16.6, same as KVS webrtc. Set mbedtls version to 2.16.6 in `project/realtek_amebapro2_v0_example/GCC-RELEASE/config.cmake`
  ```
  set(mbedtls "mbedtls-2.16.6")
  ```
- You have to modify some mbedtls config before running aws-iot demo, go to `component\ssl\mbedtls-2.16.6\include\mbedtls\config_rsa.h` check the following setting:
  ```
  #define MBEDTLS_THREADING_ALT
  //#define MBEDTLS_DEBUG_C
  #define MBEDTLS_THREADING_C
  ```

- The default mbedtls version of AmebaPro2 is 3.0.0. However, for the aws iot demo, we use mbedtls version 2.16.6 in default. It might be easier for user to use it with AWS KVS service now.
- If user want to use the aws-iot with mbedtls-3.0.0 or mbedtls-2.4.0, user can compare the config file between mbedtls-2.16.6 and mbedtls-3.0.0, mbedtls-2.4.0  

## Multiple definition issue
- multiple definition of `vApplicationGetIdleTaskMemory' and 'vApplicationGetTimerTaskMemory'
  - since aws demo runner have the same funtion that have been defined in SDK, so we should comment one of them, go to `component\os\freertos\freertos_cb.c` and comment these two funtions

    ```
    //void vApplicationGetIdleTaskMemory(...)
    //{
    //	...
    //}

    //void vApplicationGetTimerTaskMemory(...)
    //{
    //	...
    //}
    ```

## Configure NVM interface for PKCS11
- select a non-volatile memory(NVM) such as SD card and flash for the PKCS11 library
  - SD card: used by default, so please insert a SD card to the device
  - Flash: user can select the flash for pkcs11 in "project\realtek_amebapro2_v0_example\src\aws_iot_freertos_lts\vendors\realtek\boards\amebaPro2\ports\pkcs11"
    ```
    /* configure the NVM interface for pkcs11*/
    #define PKCS11_NVM_INTERFACE	PKCS11_AMEBA_FLASH
    ```
    in addition, please arrange a proper flash address(AWSIOT_PKCS11_DATA) in platform_opt.h to store pkcs11 data  

## Configure aws_clientcredential.h and aws_clientcredential_keys.h
- Refer to https://docs.aws.amazon.com/freertos/latest/userguide/freertos-prereqs.html, which will have the instructions for `Setting up your AWS account and permissions` and `Registering your MCU board with AWS IoT`. 

  - In aws_clientcredential.h(project\realtek_amebapro2_v0_example\src\aws_iot_freertos_lts\demos\include), set BROKER_ENDPOINT, THING_NAME, WIFI_SSID, PASSWORD info  
  ```
  #define clientcredentialMQTT_BROKER_ENDPOINT      "xxxxxxxx-ats.iot.xx-xxxxx-x.amazonaws.com"
  ...
  #define clientcredentialIOT_THING_NAME            "xxxxx"
  ...
  #define clientcredentialWIFI_SSID                 "SSID"
  #define clientcredentialWIFI_PASSWORD             "PASSWORD"
  ```

  - In aws_clientcredential_keys.h(project\realtek_amebapro2_v0_example\src\aws_iot_freertos_lts\demos\include), set Demo required credentials
  ```
  #define keyCLIENT_CERTIFICATE_PEM \
  "-----BEGIN CERTIFICATE-----\n" \
  "......\n" \
  "-----END CERTIFICATE-----\n"
  ...
  #define keyJITR_DEVICE_CERTIFICATE_AUTHORITY_PEM    NULL
  ...
  #define keyCLIENT_PRIVATE_KEY_PEM \
  "-----BEGIN RSA PRIVATE KEY-----\n"\
  "......\n" \
  "-----END RSA PRIVATE KEY-----\n"
  ```

## Congiure the example

- define the aws iot demo you want to run in aws_demo_config.h (project\realtek_amebapro2_v0_example\src\aws_iot_freertos_lts\vendors\realtek\boards\amebaPro2\aws_demos\config_files)
    ```
    //#define CONFIG_CORE_HTTP_MUTUAL_AUTH_DEMO_ENABLED
    #define CONFIG_CORE_MQTT_MUTUAL_AUTH_DEMO_ENABLED
    //#define CONFIG_DEVICE_SHADOW_DEMO_ENABLED
    //#define CONFIG_JOBS_DEMO_ENABLED
    ```

## Build the project
- run following commands to build the image with option `-DEXAMPLE=amazon_freertos`
    ```
    cd project/realtek_amebapro2_v0_example/GCC-RELEASE
    mkdir build
    cd build
    cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=amazon_freertos
    cmake --build . --target flash
    ```

- use image tool to download the image to AmebaPro2 and reboot


- if everything works fine, you should see the following log (MQTT mutual auth demo)
    ```
    ...
    1 429 [iot_thread] [INFO ][DEMO][429] ---------STARTING DEMO---------
    2 434 [iot_thread] [INFO ][INIT][434] SDK successfully initialized.
    ...
    Interface 0 IP address : 192.168.89.150
    3 7006 [iot_thread] [INFO ][DEMO][7006] Successfully initialized the demo. Network type for the demo: 1
    4 7015 [iot_thread] [INFO] Creating a TLS connection to xxxxxxxxxxxxxx-ats.iot.ap-southeast-1.amazonaws.com:8883.
    5 8332 [iot_thread] [INFO] Creating an MQTT connection to xxxxxxxxxxxxxx-ats.iot.ap-southeast-1.amazonaws.com.
    6 8490 [iot_thread] [INFO] Packet received. ReceivedBytes=2.
    7 8494 [iot_thread] [INFO] CONNACK session present bit not set.
    8 8500 [iot_thread] [INFO] Connection accepted.
    9 8504 [iot_thread] [INFO] Received MQTT CONNACK successfully from broker.
    10 8510 [iot_thread] [INFO] MQTT connection established with the broker.
    11 8517 [iot_thread] [INFO] An MQTT connection is established with xxxxxxxxxxxxxx-ats.iot.ap-southeast-1.amazonaws.com.
    12 8527 [iot_thread] [INFO] Attempt to subscribe to the MQTT topic ameba-ota/example/topic.
    13 8536 [iot_thread] [INFO] SUBSCRIBE sent for topic ameba-ota/example/topic to broker.
    14 8680 [iot_thread] [INFO] Packet received. ReceivedBytes=3.
    15 8684 [iot_thread] [INFO] Subscribed to the topic ameba-ota/example/topic with maximum QoS 1.
    16 9692 [iot_thread] [INFO] Publish to the MQTT topic ameba-ota/example/topic.
    17 9697 [iot_thread] [INFO] Attempt to receive publish message from broker.
    18 9919 [iot_thread] [INFO] Packet received. ReceivedBytes=2.
    19 9923 [iot_thread] [INFO] Ack packet deserialized with result: MQTTSuccess.
    20 9930 [iot_thread] [INFO] State record updated. New state=MQTTPublishDone.
    21 9937 [iot_thread] [INFO] PUBACK received for packet Id 2.
    22 9945 [iot_thread] [INFO] Packet received. ReceivedBytes=39.
    23 9949 [iot_thread] [INFO] De-serialized incoming PUBLISH packet: DeserializerResult=MQTTSuccess.
    24 9958 [iot_thread] [INFO] State record updated. New state=MQTTPubAckSend.
    25 9965 [iot_thread] [INFO] Incoming QoS : 1
    26 9969 [iot_thread] [INFO] Incoming Publish Topic Name: ameba-ota/example/topic matches subscribed topic.Incoming Publish Message : Hello World!Version=V10.4.3&MQTTLib=core-mqtt@v1.1.0
    ...
    ...
    248 122674 [iot_thread] [INFO] Demo run is successful with 3 successful loops out of total 3 loops.
    249 123681 [iot_thread] [INFO ][DEMO][123681] Demo completed successfully.

    Deinitializing WIFI ...
    WIFI deinitialized250 123809 [iot_thread] [INFO ][INIT][123809] SDK cleanup done.

    251 123813 [iot_thread] [INFO ][DEMO][123813] -------DEMO FINISHED-------
    ```

## Validate result
- user can refer the aws freertos website to validate the corresponding demo  
  https://docs.aws.amazon.com/freertos/latest/userguide/freertos-next-steps.html
