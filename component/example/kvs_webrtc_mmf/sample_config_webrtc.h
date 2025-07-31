#ifndef _SAMPLE_CONFIG_WEBRTC_H_
#define _SAMPLE_CONFIG_WEBRTC_H_

void example_kvs_webrtc(void);

/* Enter your AWS KVS key here */
#define KVS_WEBRTC_ACCESS_KEY   NULL  //"xxx"
#define KVS_WEBRTC_SECRET_KEY   NULL  //"xxx"

/* Setting your signaling channel name */
#define KVS_WEBRTC_CHANNEL_NAME "xxxxxxxxxxxxxxxxxxxx"

/* Setting your AWS region */
#define KVS_WEBRTC_REGION       "us-west-2"

/* Cert path */
#define KVS_WEBRTC_ROOT_CA_PATH "sd:/cert.pem"  //path to CA cert

/* File cache path */
#define KVS_WEBRTC_SIGNALING_CACHE_FILE_PATH    "sd:/SignalingCache_v0"

/* log level */
/* LOG_LEVEL_VERBOSE
 * LOG_LEVEL_DEBUG
 * LOG_LEVEL_INFO
 * LOG_LEVEL_WARN
 * LOG_LEVEL_ERROR
 * LOG_LEVEL_FATAL
 * LOG_LEVEL_SILEN */
#define KVS_WEBRTC_LOG_LEVEL    LOG_LEVEL_INFO

/* Enable two-way audio communication */
#define ENABLE_AUDIO_SENDRECV

/* Audio format setting */
#define AUDIO_G711_MULAW        1
#define AUDIO_G711_ALAW         0
#define AUDIO_OPUS              0
#if (AUDIO_G711_MULAW+AUDIO_G711_ALAW+AUDIO_OPUS) != 1
#error only one of audio format should be set
#endif

/*
 * Testing Amazon KVS WebRTC with IAM user key is easy but it is not recommended.
 * With AWS IoT Thing credentials, it can be managed more securely.(https://iotlabtpe.github.io/Amazon-KVS-WebRTC-WorkShop/lab/lab-4.html)
 * Script for generate iot credential: https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c/blob/master/scripts/generate-iot-credential.sh
 */
/* IoT credential configuration */
#define KVS_WEBRTC_IOT_CREDENTIAL_ENDPOINT      "xxxxxxxxxxxxxx.credentials.iot.xxxxxxxxx.amazonaws.com"  // IoT credentials endpointiot
#define KVS_WEBRTC_ROLE_ALIAS                   "xxxxxxxxxxxxxxxxxxxx"  // IoT role alias
#define KVS_WEBRTC_THING_NAME                   KVS_WEBRTC_CHANNEL_NAME  // iot thing name, recommended to be same as your channel name
#define KVS_WEBRTC_CERTIFICATE_PATH             "sd:/webrtc_iot_certifcate.pem"
#define KVS_WEBRTC_PRIVATE_KEY_PATH             "sd:/webrtc_iot_private.key"

#endif /* _SAMPLE_CONFIG_WEBRTC_H_ */

