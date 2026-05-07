/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SIGNALING_CONTROLLER_H
#define SIGNALING_CONTROLLER_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes. */
#include <stdint.h>
#include <stddef.h>
#include <time.h>


#include "networking.h"
#include "signaling_api.h"
#include "networking.h"
#include "message_queue.h"

/* Refer to https://docs.aws.amazon.com/IAM/latest/APIReference/API_AccessKey.html,
   length of access key ID should be limited to 128. There is no other definition of
   length of secret access key, set it same as access key ID for now. */

#define SIGNALING_CONTROLLER_FETCH_CREDS_GRACE_PERIOD_SEC           ( 30 )
#define SIGNALING_CONTROLLER_HTTP_URL_BUFFER_LENGTH                 ( 1024 )
#define SIGNALING_CONTROLLER_HTTP_BODY_BUFFER_LENGTH                ( 10 * 1024 )
#define SIGNALING_CONTROLLER_HTTP_RESPONSE_BUFFER_LENGTH            ( 10 * 1024 )
#define SIGNALING_CONTROLLER_MESSAGE_BUFFER_LENGTH                  ( 10 * 1024 )
#define SIGNALING_CONTROLLER_HTTP_NUM_RETRIES                       ( 5U )
#define SIGNALING_CONTROLLER_ARN_BUFFER_LENGTH                      ( 128 )
#define SIGNALING_CONTROLLER_ENDPOINT_BUFFER_LENGTH                 ( 128 )
#define SIGNALING_CONTROLLER_ICE_SERVER_MAX_URIS_COUNT              ( 3 )
#define SIGNALING_CONTROLLER_ICE_SERVER_URI_BUFFER_LENGTH           ( 256 )
#define SIGNALING_CONTROLLER_ICE_SERVER_USER_NAME_BUFFER_LENGTH     ( 256 )
#define SIGNALING_CONTROLLER_ICE_SERVER_PASSWORD_BUFFER_LENGTH      ( 256 )
#define SIGNALING_CONTROLLER_ICE_SERVER_MAX_CONFIG_COUNT            ( 5 )
#define SIGNALING_CONTROLLER_REMOTE_ID_MAX_LENGTH                   ( 256 )

/**
 * Grace period for refreshing the ICE configuration
 */
#define ICE_CONFIGURATION_REFRESH_GRACE_PERIOD_SEC ( 30 )

#define SIGNALING_CONTROLLER_MASTER_CLIENT_ID                       "ProducerMaster"
#define SIGNALING_CONTROLLER_MASTER_CLIENT_ID_LENGTH                ( 14 )

/* Viewer client ID is used to connect with WSS server. Remember to follow the rule in
 * https://docs.aws.amazon.com/kinesisvideostreams-webrtc-dg/latest/devguide/ConnectAsViewer.html
 * to NOT have prefix "AWS_" */
#define SIGNALING_CONTROLLER_VIEWER_CLIENT_ID_PREFIX                "ConsumerViewer_"
#define SIGNALING_CONTROLLER_VIEWER_CLIENT_ID_PREFIX_LENGTH         ( 15 )

typedef enum SignalingControllerEventStatus
{
    SIGNALING_CONTROLLER_EVENT_STATUS_NONE = 0,
    SIGNALING_CONTROLLER_EVENT_STATUS_SENT_DONE,
    SIGNALING_CONTROLLER_EVENT_STATUS_SENT_FAIL,
} SignalingControllerEventStatus_t;

typedef enum SignalingControllerConnectionState
{
    SIGNALING_CONTROLLER_STATE_NONE = 0,
    SIGNALING_CONTROLLER_STATE_INITED,
    SIGNALING_CONTROLLER_STATE_DISCONNECTED,
    SIGNALING_CONTROLLER_STATE_CONNECTED,
} SignalingControllerConnectionState_t;

typedef void ( * SignalingControllerConnectionStateCallback_t )( SignalingControllerConnectionState_t state,
                                                                 void * pUserData );

typedef struct SignalingControllerEventContentSend
{
    char remoteClientId[ SIGNALING_CONTROLLER_REMOTE_ID_MAX_LENGTH ];
    size_t remoteClientIdLength;
    SignalingTypeMessage_t messageType;
    char * pDecodeMessage;
    size_t decodeMessageLength;
    char correlationId[ SECRET_ACCESS_KEY_MAX_LEN ];
    size_t correlationIdLength;
} SignalingControllerEventContentSend_t;

typedef struct SignalingMessage
{
    const char * pRemoteClientId;
    size_t remoteClientIdLength;
    SignalingTypeMessage_t messageType;
    const char * pMessage;
    size_t messageLength;
    const char * pCorrelationId;
    size_t correlationIdLength;
} SignalingMessage_t;

typedef int ( * SignalingMessageReceivedCallback_t )( SignalingMessage_t * pSignalingMessage,
                                                      void * pUserData );
typedef int32_t (* SignalingControllerCompleteSendCallback)( SignalingControllerEventStatus_t status,
                                                             void * pUserContext );

typedef enum SignalingControllerResult
{
    SIGNALING_CONTROLLER_RESULT_OK = 0,
    SIGNALING_CONTROLLER_RESULT_BAD_PARAM,
    SIGNALING_CONTROLLER_RESULT_FAIL
} SignalingControllerResult_t;

typedef struct IceServerUri
{
    char uri[ SIGNALING_CONTROLLER_ICE_SERVER_URI_BUFFER_LENGTH + 1 ];
    size_t uriLength;
} IceServerUri_t;

typedef struct IceServerConfig
{
    uint32_t ttlSeconds;
    IceServerUri_t iceServerUris[ SIGNALING_CONTROLLER_ICE_SERVER_MAX_URIS_COUNT ];
    size_t iceServerUriCount;
    char userName[ SIGNALING_CONTROLLER_ICE_SERVER_USER_NAME_BUFFER_LENGTH + 1];
    size_t userNameLength;
    char password[ SIGNALING_CONTROLLER_ICE_SERVER_PASSWORD_BUFFER_LENGTH + 1];
    size_t passwordLength;
} IceServerConfig_t;

typedef enum SignalingControllerEvent
{
    SIGNALING_CONTROLLER_EVENT_NONE = 0,
    SIGNALING_CONTROLLER_EVENT_SEND_WSS_MESSAGE,
    SIGNALING_CONTROLLER_EVENT_RECONNECT_WSS,
    SIGNALING_CONTROLLER_EVENT_FORCE_REFRESH_ICE_SERVER_CONFIGS,
} SignalingControllerEvent_t;

typedef struct SignalingControllerEventMessage
{
    SignalingControllerEvent_t event;
    SignalingControllerEventContentSend_t eventContent;
    SignalingControllerCompleteSendCallback onCompleteCallback;
    void * pOnCompleteCallbackContext;
} SignalingControllerEventMessage_t;

typedef struct SignalingControllerContext
{
    SignalingControllerConnectionState_t connectionState;

    /* AKSK */
    char accessKeyId[ ACCESS_KEY_MAX_LEN + 1 ];
    size_t accessKeyIdLength;
    char secretAccessKey[ SECRET_ACCESS_KEY_MAX_LEN + 1 ];
    size_t secretAccessKeyLength;
    /* Session Token */
    char sessionToken[ SESSION_TOKEN_MAX_LEN + 1 ];
    size_t sessionTokenLength;
    /* Expiration */
    uint64_t expirationSeconds;

    /* Describe signaling channel */
    char signalingChannelArn[ SIGNALING_CONTROLLER_ARN_BUFFER_LENGTH + 1 ];
    size_t signalingChannelArnLength;

    /* Get signaling channel endpoints */
    char wssEndpoint[SIGNALING_CONTROLLER_ENDPOINT_BUFFER_LENGTH + 1];
    size_t wssEndpointLength;
    char httpsEndpoint[ SIGNALING_CONTROLLER_ENDPOINT_BUFFER_LENGTH + 1 ];
    size_t httpsEndpointLength;
    char webrtcEndpoint[ SIGNALING_CONTROLLER_ENDPOINT_BUFFER_LENGTH + 1 ];
    size_t webrtcEndpointLength;

    /* CA Cert */
    const uint8_t * pCaCertPem;
    size_t caCertPemSize;

    /* Credential Endpoint */
    const char * pIotCredentialsEndpoint;
    size_t iotCredentialsEndpointLength;

    /* AWS IoT Thing name */
    const char * pIotThingName;
    size_t iotThingNameLength;

    /* AWS IoT Thing Role Alias name */
    const char * pRoleAlias;
    size_t roleAliasLength;

    /* AWS IoT Thing certificate */
    const uint8_t * pIotThingCert;
    size_t iotThingCertSize;

    /* Channel Name */
    const char * pChannelName;
    size_t channelNameLength;

    /* AWS IoT Thing private key */
    const uint8_t * pIotThingPrivateKey;
    size_t iotThingPrivateKeySize;

    uint64_t iceServerConfigExpirationSec;
    uint8_t iceServerConfigsCount;
    IceServerConfig_t iceServerConfigs[ SIGNALING_CONTROLLER_ICE_SERVER_MAX_CONFIG_COUNT ];

    /* User Agent Name */
    const char * pUserAgentName;
    size_t userAgentNameLength;

    const char * pClientId;
    size_t clientIdLength;
    SignalingRole_t role;

    AwsConfig_t awsConfig;

    char httpUrlBuffer[ SIGNALING_CONTROLLER_HTTP_URL_BUFFER_LENGTH ];
    char httpBodyBuffer[ SIGNALING_CONTROLLER_HTTP_BODY_BUFFER_LENGTH ];
    char httpResponserBuffer[ SIGNALING_CONTROLLER_HTTP_RESPONSE_BUFFER_LENGTH ];
    char signalingRxMessageBuffer[ SIGNALING_CONTROLLER_MESSAGE_BUFFER_LENGTH ];
    size_t signalingRxMessageLength;
    char signalingTxMessageBuffer[ SIGNALING_CONTROLLER_MESSAGE_BUFFER_LENGTH ];
    size_t signalingTxMessageLength;
    char signalingIntermediateMessageBuffer[ SIGNALING_CONTROLLER_MESSAGE_BUFFER_LENGTH ];
    size_t signalingIntermediateMessageLength;

    MessageQueueHandler_t sendMessageQueue;

    SignalingMessageReceivedCallback_t messageReceivedCallback;
    void * pMessageReceivedCallbackData;
    SignalingControllerConnectionStateCallback_t connectionStateCallback;
    void * pConnectionStateCallbacCustomContext;

    NetworkingCorehttpContext_t httpContext;
    NetworkingWslayContext_t websocketContext;

    /* Configurations. */
    uint8_t enableStorageSession;
} SignalingControllerContext_t;


typedef struct AwsIotCredentials
{
    char * pIotThingName;
    size_t iotThingNameLength;
    char * pRoleAlias;
    size_t roleAliasLength;
    char * pIotCredentialsEndpoint;
    size_t iotCredentialsEndpointLength;

    /* AWS IoT Thing certificate */
    char * pIotThingCert;
    size_t iotThingCertSize;

    /* AWS IoT Thing private key */
    char * pIotThingPrivateKey;
    size_t iotThingPrivateKeySize;

} AwsIotCredentials_t;

typedef struct SignalingControllerConnectInfo
{
    AwsCredentials_t awsCreds;
    AwsConfig_t awsConfig;
    AwsIotCredentials_t awsIotCreds;
    SignalingChannelName_t channelName;
    const char * pUserAgentName;
    size_t userAgentNameLength;
    SignalingMessageReceivedCallback_t messageReceivedCallback;
    void * pMessageReceivedCallbackData;
    SignalingRole_t role;

    /* Configurations. */
    uint8_t enableStorageSession;
    char *pClientId;
    size_t clientIdLength;
} SignalingControllerConnectInfo_t;

SignalingControllerResult_t SignalingController_Init( SignalingControllerContext_t * pCtx,
                                                      const SSLCredentials_t * pSslCreds );
SignalingControllerResult_t SignalingController_RefreshIceServerConfigs( SignalingControllerContext_t * pCtx );
SignalingControllerResult_t SignalingController_StartListening( SignalingControllerContext_t * pCtx,
                                                                const SignalingControllerConnectInfo_t * pConnectInfo );
SignalingControllerResult_t SignalingController_SendMessage( SignalingControllerContext_t * pCtx,
                                                             SignalingControllerEventMessage_t * pEventMsg );
SignalingControllerResult_t SignalingController_QueryIceServerConfigs( SignalingControllerContext_t * pCtx,
                                                                       IceServerConfig_t ** ppIceServerConfigs,
                                                                       size_t * pIceServerConfigsCount );
SignalingControllerResult_t SignalingController_ExtractSdpMessageFromSignalingMessage( const char * pEventMessage,
                                                                                       size_t eventMessageLength,
                                                                                       uint8_t isSdpOffer,
                                                                                       const char ** ppSdpMessage,
                                                                                       size_t * pSdpMessageLength );
SignalingControllerResult_t SignalingController_DeserializeSdpContentNewline( const char * pSdpMessage,
                                                                              size_t sdpMessageLength,
                                                                              char * pFormalSdpMessage,
                                                                              size_t * pFormalSdpMessageLength );
SignalingControllerResult_t SignalingController_SerializeSdpContentNewline( const char * pSdpMessage,
                                                                            size_t sdpMessageLength,
                                                                            char * pEventSdpMessage,
                                                                            size_t * pEventSdpMessageLength );
SignalingControllerResult_t SignalingController_SetConnectionStateCallback( SignalingControllerContext_t * pCtx,
                                                                            SignalingControllerConnectionStateCallback_t callback,
                                                                            void * pCustomContext );

#ifdef __cplusplus
}
#endif

#endif /* SIGNALING_CONTROLLER_H */
