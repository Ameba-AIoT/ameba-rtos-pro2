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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

#include "sys_api.h"      /* sys_backtrace_enable() */
#include "sntp/sntp.h"    /* SNTP series APIs */
#include "wifi_conf.h"    /* WiFi series APIs */
#include "lwip_netconf.h" /* LwIP_GetIP() */
#include "srtp.h"

#include "demo_config.h"
#include "logging.h"
#include "app_common.h"
#include "networking_utils.h"
#include "string_utils.h"
#if METRIC_PRINT_ENABLED
#include "metric.h"
#endif
#include "app_media_source.h"

#if ENABLE_SCTP_DATA_CHANNEL
#include "peer_connection_sctp.h"
#endif /* ENABLE_SCTP_DATA_CHANNEL */

#define AWS_DEFAULT_STUN_SERVER_URL_POSTFIX       "amazonaws.com"
#define AWS_DEFAULT_STUN_SERVER_URL_POSTFIX_CN    "amazonaws.com.cn"
#define AWS_DEFAULT_STUN_SERVER_URL               "stun.kinesisvideo.%s.%s"

#define DEMO_WAIT_WIFI_TIME_MS                    ( 5000 ) /*Here we wait 5 second to wiat the fast connect */

#define DEMO_JSON_CANDIDATE_MAX_LENGTH            ( 512 )

#define DEMO_CANDIDATE_TYPE_HOST_STRING           "host"
#define DEMO_CANDIDATE_TYPE_SRFLX_STRING          "srflx"
#define DEMO_CANDIDATE_TYPE_PRFLX_STRING          "prflx"
#define DEMO_CANDIDATE_TYPE_RELAY_STRING          "relay"
#define DEMO_CANDIDATE_TYPE_UNKNOWN_STRING        "unknown"

#define DEMO_ICE_CANDIDATE_JSON_TEMPLATE          "{\"candidate\":\"%.*s\",\"sdpMid\":\"0\",\"sdpMLineIndex\":0}"
#define DEMO_ICE_CANDIDATE_JSON_MAX_LENGTH        ( 1024 )
#define DEMO_ICE_CANDIDATE_JSON_IPV4_TEMPLATE     "candidate:%u 1 udp %lu %d.%d.%d.%d %d typ %s raddr 0.0.0.0 rport 0 generation 0 network-cost 999"
#define DEMO_ICE_CANDIDATE_JSON_IPV6_TEMPLATE     "candidate:%u 1 udp %lu %02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X " \
                                                  "%d typ %s raddr ::/0 rport 0 generation 0 network-cost 999"

#define ICE_SERVER_TYPE_STUN                      "stun:"
#define ICE_SERVER_TYPE_STUN_LENGTH               ( 5 )
#define ICE_SERVER_TYPE_TURN                      "turn:"
#define ICE_SERVER_TYPE_TURN_LENGTH               ( 5 )
#define ICE_SERVER_TYPE_TURNS                     "turns:"
#define ICE_SERVER_TYPE_TURNS_LENGTH              ( 6 )

/**
 * EMA (Exponential Moving Average) alpha value and 1-alpha value - over appx 20 samples
 */
#define EMA_ALPHA_VALUE           ( ( double ) 0.05 )
#define ONE_MINUS_EMA_ALPHA_VALUE ( ( double ) ( 1 - EMA_ALPHA_VALUE ) )

/**
 * Calculates the EMA (Exponential Moving Average) accumulator value
 *
 * a - Accumulator value
 * v - Next sample point
 *
 * @return the new Accumulator value
 */
#define EMA_ACCUMULATOR_GET_NEXT( a, v ) ( double )( EMA_ALPHA_VALUE * ( v ) + ONE_MINUS_EMA_ALPHA_VALUE * ( a ) )

#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

#ifndef MAX
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#endif

#define APP_COMMON_SIGNALING_CONTROLER_START_UP_BARRIER_BIT ( 1 << 0 )

extern int crypto_init( void );
extern int platform_set_malloc_free( void * ( *malloc_func )( size_t ),
                                     void ( * free_func )( void * ) );
static void platform_init( void );
static void wifi_common_init( void );

static int32_t StartPeerConnectionSession( AppContext_t * pAppContext,
                                           AppSession_t * pAppSession,
                                           const char * pRemoteClientId,
                                           size_t remoteClientIdLength );
static int32_t ParseIceServerUri( IceControllerIceServer_t * pIceServer,
                                  char * pUri,
                                  size_t uriLength );
static int32_t GetIceServerList( AppContext_t * pAppContext,
                                 IceControllerIceServer_t * pOutputIceServers,
                                 size_t * pOutputIceServersCount );
#if ENABLE_TWCC_SUPPORT
    /* Sample callback for TWCC. The average packet loss is tracked using an exponential moving average (EMA).
       - If packet loss stays at or below 5%, the bitrate increases by 5%.
       - If packet loss exceeds 5%, the bitrate decreases by the same percentage as the loss.
       The bitrate is adjusted once per second, ensuring it stays within predefined limits. */
       static void SampleSenderBandwidthEstimationHandler( void * pCustomContext,
                                                           TwccBandwidthInfo_t * pTwccBandwidthInfo );
#endif /* ENABLE_TWCC_SUPPORT */
static int32_t InitializeAppSession( AppContext_t * pAppContext,
                                     AppSession_t * pAppSession );
static PeerConnectionResult_t HandleRxVideoFrame( void * pCustomContext,
                                                  PeerConnectionFrame_t * pFrame );
static PeerConnectionResult_t HandleRxAudioFrame( void * pCustomContext,
                                                  PeerConnectionFrame_t * pFrame );
static void HandleSdpOffer( AppContext_t * pAppContext,
                            const SignalingMessage_t * pSignalingMessage );
static void HandleSdpAnswer( AppContext_t * pAppContext,
                             const SignalingMessage_t * pSignalingMessage );
static void HandleRemoteCandidate( AppContext_t * pAppContext,
                                   const SignalingMessage_t * pSignalingMessage );
static const char * GetCandidateTypeString( IceCandidateType_t candidateType );
static void HandleLocalCandidateReady( void * pCustomContext,
                                       PeerConnectionIceLocalCandidate_t * pIceLocalCandidate );
static int OnSignalingMessageReceived( SignalingMessage_t * pSignalingMessage,
                                       void * pUserData );
static int32_t OnSendIceCandidateComplete( SignalingControllerEventStatus_t status,
                                           void * pUserContext );

#if ENABLE_SCTP_DATA_CHANNEL
#if ( DATACHANNEL_CUSTOM_CALLBACK_HOOK != 0 )
static void OnDataChannelMessage( PeerConnectionDataChannel_t * pDataChannel,
                                  uint8_t isBinary,
                                  uint8_t * pMessage,
                                  uint32_t pMessageLen );
#endif     /* ( DATACHANNEL_CUSTOM_CALLBACK_HOOK != 0 ) */
#endif /* ENABLE_SCTP_DATA_CHANNEL */

static void platform_init( void )
{
    long long sec;

    /* mbedtls init */
    crypto_init();
    platform_set_malloc_free( ( void ( * ) )calloc,
                              ( void ( * )( void * ) )free );

    /* Show backtrace if exception. */
    sys_backtrace_enable();

    /* Block until network ready. */
    wifi_common_init();

    /* Block until get time via SNTP. */
    sntp_init();

    while( ( sec = NetworkingUtils_GetCurrentTimeSec( NULL ) ) < 1000000000ULL )
    {
        vTaskDelay( pdMS_TO_TICKS( 200 ) );
        LogInfo( ( "waiting get epoch timer" ) );
    }
}

static void wifi_common_init( void )
{
    uint32_t wifi_wait_count = 0;

    while( !( ( wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS ) && ( *( u32 * ) LwIP_GetIP( 0 ) != IP_ADDR_INVALID ) ) )
    {
        vTaskDelay( 10 );
        wifi_wait_count += 10;

        if( wifi_wait_count >= DEMO_WAIT_WIFI_TIME_MS )
        {
            LogInfo( ( "\r\nuse ATW0, ATW1, ATWC to make wifi connection\r\n" ) );
            LogInfo( ( "wait for wifi connection...\r\n" ) );
            wifi_wait_count = 0;
        }
    }
}

static void SignalingController_Task( void * pParameter )
{
    int32_t ret = 0;
    AppContext_t * pAppContext = ( AppContext_t * ) pParameter;
    SignalingControllerResult_t signalingControllerReturn;
    SignalingControllerConnectInfo_t connectInfo;

    if( pAppContext == NULL )
    {
        LogError( ( "Invalid parameter, pAppContext: %p", pAppContext ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        memset( &connectInfo, 0, sizeof( SignalingControllerConnectInfo_t ) );

        #if ( JOIN_STORAGE_SESSION != 0 )
        connectInfo.enableStorageSession = 1;
        #endif

        connectInfo.awsConfig.pRegion = AWS_REGION;
        connectInfo.awsConfig.regionLen = strlen( AWS_REGION );
        connectInfo.awsConfig.pService = "kinesisvideo";
        connectInfo.awsConfig.serviceLen = strlen( "kinesisvideo" );

        connectInfo.channelName.pChannelName = AWS_KVS_CHANNEL_NAME;
        connectInfo.channelName.channelNameLength = strlen( AWS_KVS_CHANNEL_NAME );

        connectInfo.pUserAgentName = AWS_KVS_AGENT_NAME;
        connectInfo.userAgentNameLength = strlen( AWS_KVS_AGENT_NAME );

        connectInfo.messageReceivedCallback = OnSignalingMessageReceived;
        connectInfo.pMessageReceivedCallbackData = pAppContext;

        connectInfo.role = pAppContext->signalingControllerRole;
        connectInfo.pClientId = pAppContext->signalingControllerClientId;
        connectInfo.clientIdLength = pAppContext->signalingControllerClientIdLength;

        #if defined( AWS_ACCESS_KEY_ID )
            connectInfo.awsCreds.pAccessKeyId = AWS_ACCESS_KEY_ID;
            connectInfo.awsCreds.accessKeyIdLength = strlen( AWS_ACCESS_KEY_ID );
            connectInfo.awsCreds.pSecretAccessKey = AWS_SECRET_ACCESS_KEY;
            connectInfo.awsCreds.secretAccessKeyLength = strlen( AWS_SECRET_ACCESS_KEY );
        #if defined( AWS_SESSION_TOKEN )
            connectInfo.awsCreds.pSessionToken = AWS_SESSION_TOKEN;
            connectInfo.awsCreds.sessionTokenLength = strlen( AWS_SESSION_TOKEN );
        #endif     /* #if defined( AWS_SESSION_TOKEN ) */
        #endif /* #if defined( AWS_ACCESS_KEY_ID ) */

        #if defined( AWS_IOT_THING_ROLE_ALIAS )
            connectInfo.awsIotCreds.pIotCredentialsEndpoint = AWS_CREDENTIALS_ENDPOINT;
            connectInfo.awsIotCreds.iotCredentialsEndpointLength = strlen( AWS_CREDENTIALS_ENDPOINT );
            connectInfo.awsIotCreds.pIotThingName = AWS_IOT_THING_NAME;
            connectInfo.awsIotCreds.iotThingNameLength = strlen( AWS_IOT_THING_NAME );
            connectInfo.awsIotCreds.pRoleAlias = AWS_IOT_THING_ROLE_ALIAS;
            connectInfo.awsIotCreds.roleAliasLength = strlen( AWS_IOT_THING_ROLE_ALIAS );
        #endif /* #if defined( AWS_IOT_THING_ROLE_ALIAS ) */

        if( ret == 0 )
        {
            /* This should never return unless exception happens. */
            signalingControllerReturn = SignalingController_StartListening( &pAppContext->signalingControllerContext,
                                                                            &connectInfo );

            if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
            {
                LogError( ( "Fail to keep processing signaling controller." ) );
                ret = -1;
            }
        }
    }

    #if ENABLE_SCTP_DATA_CHANNEL
        /* TODO_SCTP: Move to a common shutdown function? */
        Sctp_DeInit();
    #endif /* ENABLE_SCTP_DATA_CHANNEL */

    for( ;; )
    {
        vTaskDelay( pdMS_TO_TICKS( 200 ) );
    }
}

static int32_t StartPeerConnectionSession( AppContext_t * pAppContext,
                                           AppSession_t * pAppSession,
                                           const char * pRemoteClientId,
                                           size_t remoteClientIdLength )
{
    int32_t ret = 0;
    PeerConnectionResult_t peerConnectionResult;
    PeerConnectionSessionConfiguration_t pcConfig;
    Transceiver_t * pTransceiver = NULL;

    if( remoteClientIdLength > REMOTE_ID_MAX_LENGTH )
    {
        LogWarn( ( "The remote client ID length(%u) is too long to store.", remoteClientIdLength ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        memset( &pcConfig,
                0,
                sizeof( PeerConnectionSessionConfiguration_t ) );
        pcConfig.iceServersCount = ICE_CONTROLLER_MAX_ICE_SERVER_COUNT;
        #if defined( AWS_CA_CERT_PATH )
        pcConfig.pRootCaPath = AWS_CA_CERT_PATH;
        pcConfig.rootCaPathLength = strlen( AWS_CA_CERT_PATH );
        #endif /* #if defined( AWS_CA_CERT_PATH ) */

        #if defined( AWS_CA_CERT_PEM )
        pcConfig.pRootCaPem = AWS_CA_CERT_PEM;
        pcConfig.rootCaPemLength = sizeof( AWS_CA_CERT_PEM );
        #endif /* #if defined( AWS_CA_CERT_PEM ) */

        pcConfig.canTrickleIce = 1U;
        pcConfig.natTraversalConfigBitmap = pAppContext->natTraversalConfig;

        ret = GetIceServerList( pAppContext,
                                pcConfig.iceServers,
                                &pcConfig.iceServersCount );
    }

    if( ret == 0 )
    {
        peerConnectionResult = PeerConnection_AddIceServerConfig( &pAppSession->peerConnectionSession,
                                                                  &pcConfig );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogWarn( ( "PeerConnection_AddIceServerConfig fail, result: %d", peerConnectionResult ) );
            ret = -1;
        }
    }

    if( ret == 0 )
    {
        peerConnectionResult = PeerConnection_SetOnLocalCandidateReady( &pAppSession->peerConnectionSession,
                                                                        HandleLocalCandidateReady,
                                                                        pAppSession );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogWarn( ( "PeerConnection_SetOnLocalCandidateReady fail, result: %d", peerConnectionResult ) );
            ret = -1;
        }
    }

    /* Add video transceiver */
    if( ret == 0 )
    {
        pTransceiver = &pAppSession->transceivers[ DEMO_TRANSCEIVER_MEDIA_INDEX_VIDEO ];
        ret = pAppContext->initTransceiverFunc( pAppContext->pAppMediaSourcesContext,
                                                TRANSCEIVER_TRACK_KIND_VIDEO,
                                                pTransceiver );
        if( ret != 0 )
        {
            LogError( ( "Fail to get video transceiver." ) );
        }
        else
        {
            peerConnectionResult = PeerConnection_AddTransceiver( &pAppSession->peerConnectionSession,
                                                                  pTransceiver );
            if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
            {
                LogError( ( "Fail to add video transceiver, result = %d.", peerConnectionResult ) );
                ret = -1;
            }
        }
    }

    /* Add audio transceiver */
    if( ret == 0 )
    {
        pTransceiver = &pAppSession->transceivers[ DEMO_TRANSCEIVER_MEDIA_INDEX_AUDIO ];
        ret = pAppContext->initTransceiverFunc( pAppContext->pAppMediaSourcesContext,
                                                TRANSCEIVER_TRACK_KIND_AUDIO,
                                                pTransceiver );
        if( ret != 0 )
        {
            LogError( ( "Fail to get audio transceiver." ) );
        }
        else
        {
            peerConnectionResult = PeerConnection_AddTransceiver( &pAppSession->peerConnectionSession,
                                                                  pTransceiver );

            if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
            {
                LogError( ( "Fail to add audio transceiver, result = %d.", peerConnectionResult ) );
                ret = -1;
            }
        }
    }

    if( ret == 0 )
    {
        pAppSession->remoteClientIdLength = remoteClientIdLength;
        memcpy( pAppSession->remoteClientId,
                pRemoteClientId,
                remoteClientIdLength );
        peerConnectionResult = PeerConnection_Start( &pAppSession->peerConnectionSession );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogError( ( "Fail to start peer connection, result = %d.", peerConnectionResult ) );
            ret = -1;
        }
    }

    return ret;
}

static int32_t ParseIceServerUri( IceControllerIceServer_t * pIceServer,
                                  char * pUri,
                                  size_t uriLength )
{
    int32_t ret = 0;
    StringUtilsResult_t retString;
    const char * pCurr, * pTail, * pNext;
    uint32_t port, portStringLength;

    /* Example Ice server URI:
     *  1. turn:35-94-7-249.t-490d1050.kinesisvideo.us-west-2.amazonaws.com:443?transport=udp
     *  2. stun:stun.kinesisvideo.us-west-2.amazonaws.com:443 */
    if( ( uriLength > ICE_SERVER_TYPE_STUN_LENGTH ) && ( strncmp( ICE_SERVER_TYPE_STUN,
                                                                  pUri,
                                                                  ICE_SERVER_TYPE_STUN_LENGTH ) == 0 ) )
    {
        pIceServer->serverType = ICE_CONTROLLER_ICE_SERVER_TYPE_STUN;
        pTail = pUri + uriLength;
        pCurr = pUri + ICE_SERVER_TYPE_STUN_LENGTH;
    }
    else if( ( ( uriLength > ICE_SERVER_TYPE_TURNS_LENGTH ) && ( strncmp( ICE_SERVER_TYPE_TURNS,
                                                                          pUri,
                                                                          ICE_SERVER_TYPE_TURNS_LENGTH ) == 0 ) ) )
    {
        pIceServer->serverType = ICE_CONTROLLER_ICE_SERVER_TYPE_TURNS;
        pTail = pUri + uriLength;
        pCurr = pUri + ICE_SERVER_TYPE_TURNS_LENGTH;
    }
    else if( ( uriLength > ICE_SERVER_TYPE_TURN_LENGTH ) && ( strncmp( ICE_SERVER_TYPE_TURN,
                                                                       pUri,
                                                                       ICE_SERVER_TYPE_TURN_LENGTH ) == 0 ) )
    {
        pIceServer->serverType = ICE_CONTROLLER_ICE_SERVER_TYPE_TURN;
        pTail = pUri + uriLength;
        pCurr = pUri + ICE_SERVER_TYPE_TURN_LENGTH;
    }
    else
    {
        /* Invalid server URI, drop it. */
        LogWarn( ( "Unable to parse Ice URI, drop it, URI: %.*s", ( int ) uriLength, pUri ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        pNext = memchr( pCurr,
                        ':',
                        pTail - pCurr );

        if( pNext == NULL )
        {
            LogWarn( ( "Unable to find second ':', drop it, URI: %.*s", ( int ) uriLength, pUri ) );
            ret = -1;
        }
        else
        {
            if( pNext - pCurr >= ICE_CONTROLLER_ICE_SERVER_URL_MAX_LENGTH )
            {
                LogWarn( ( "URL buffer is not enough to store Ice URL, length: %d, URI: %.*s",
                           pNext - pCurr,
                           ( int ) uriLength, pUri ) );
                ret = -1;
            }
            else
            {
                memcpy( pIceServer->url,
                        pCurr,
                        pNext - pCurr );
                pIceServer->urlLength = pNext - pCurr;
                /* Note that URL must be NULL terminated for DNS lookup. */
                pIceServer->url[ pIceServer->urlLength ] = '\0';
                pCurr = pNext + 1;
            }
        }
    }

    if( ( ret == 0 ) && ( pCurr <= pTail ) )
    {
        pNext = memchr( pCurr,
                        '?',
                        pTail - pCurr );

        if( pNext == NULL )
        {
            portStringLength = pTail - pCurr;
        }
        else
        {
            portStringLength = pNext - pCurr;
        }

        retString = StringUtils_ConvertStringToUl( pCurr,
                                                   portStringLength,
                                                   &port );

        if( ( retString != STRING_UTILS_RESULT_OK ) || ( port > UINT16_MAX ) )
        {
            LogWarn( ( "No valid port number, parsed string: %.*s", ( int ) portStringLength, pCurr ) );
            ret = -1;
        }
        else
        {
            pIceServer->iceEndpoint.transportAddress.port = ( uint16_t ) port;
            pCurr += portStringLength;
        }
    }

    if( ret == 0 )
    {
        if( pCurr >= pTail )
        {
            LogWarn( ( "No valid transport string found" ) );
            ret = -1;
        }
        else if( ( pIceServer->serverType == ICE_CONTROLLER_ICE_SERVER_TYPE_TURN ) ||
                 ( pIceServer->serverType == ICE_CONTROLLER_ICE_SERVER_TYPE_TURNS ) )
        {
            if( strncmp( pCurr,
                         "?transport=udp",
                         pTail - pCurr ) == 0 )
            {
                pIceServer->protocol = ICE_SOCKET_PROTOCOL_UDP;
            }
            else if( strncmp( pCurr,
                              "?transport=tcp",
                              pTail - pCurr ) == 0 )
            {
                pIceServer->protocol = ICE_SOCKET_PROTOCOL_TCP;
            }
            else
            {
                LogWarn( ( "Unknown transport string found, protocol: %.*s", ( int ) ( pTail - pCurr ), pCurr ) );
                ret = -1;
            }
        }
        else
        {
            /* Do nothing, coverity happy. */
        }
    }

    return ret;
}

static int32_t GetIceServerList( AppContext_t * pAppContext,
                                 IceControllerIceServer_t * pOutputIceServers,
                                 size_t * pOutputIceServersCount )
{
    int32_t skipProcess = 0;
    int32_t parseResult = 0;
    SignalingControllerResult_t signalingControllerReturn;
    IceServerConfig_t * pIceServerConfigs;
    size_t iceServerConfigsCount;
    char * pStunUrlPostfix;
    int written;
    uint32_t i, j;
    size_t currentIceServerIndex = 0U;

    if( ( pAppContext == NULL ) ||
        ( pOutputIceServers == NULL ) ||
        ( pOutputIceServersCount == NULL ) )
    {
        LogError( ( "Invalid input, pAppContext: %p, pOutputIceServers: %p, pOutputIceServersCount: %p",
                    pAppContext,
                    pOutputIceServers,
                    pOutputIceServersCount ) );
        skipProcess = 1;
    }
    else if( *pOutputIceServersCount < 1 )
    {
        /* At least one space for default STUN server. */
        LogError( ( "Invalid input, buffer size(%u) is insufficient",
                    *pOutputIceServersCount ) );
        skipProcess = -1;
    }
    else
    {
        /* Empty else marker. */
    }

    if( skipProcess == 0 )
    {
        signalingControllerReturn = SignalingController_QueryIceServerConfigs( &pAppContext->signalingControllerContext,
                                                                               &pIceServerConfigs,
                                                                               &iceServerConfigsCount );

        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to get Ice server configs, result: %d", signalingControllerReturn ) );
            skipProcess = -1;
        }
    }

    if( skipProcess == 0 )
    {
        /* Put the default STUN server into index 0. */
        if( strstr( AWS_REGION,
                    "cn-" ) )
        {
            pStunUrlPostfix = AWS_DEFAULT_STUN_SERVER_URL_POSTFIX_CN;
        }
        else
        {
            pStunUrlPostfix = AWS_DEFAULT_STUN_SERVER_URL_POSTFIX;
        }

        /* Get the default STUN server. */
        written = snprintf( pOutputIceServers[ currentIceServerIndex ].url,
                            ICE_CONTROLLER_ICE_SERVER_URL_MAX_LENGTH,
                            AWS_DEFAULT_STUN_SERVER_URL,
                            AWS_REGION,
                            pStunUrlPostfix );

        if( written < 0 )
        {
            LogError( ( "snprintf fail, errno: %s", strerror( errno ) ) );
            skipProcess = -1;
        }
        else if( written == ICE_CONTROLLER_ICE_SERVER_URL_MAX_LENGTH )
        {
            LogError( ( "buffer has no space for default STUN server" ) );
            skipProcess = -1;
        }
        else
        {
            /* STUN server is written correctly. Set UDP as protocol since we always use UDP to query server reflexive address. */
            pOutputIceServers[ currentIceServerIndex ].protocol = ICE_SOCKET_PROTOCOL_UDP;
            pOutputIceServers[ currentIceServerIndex ].serverType = ICE_CONTROLLER_ICE_SERVER_TYPE_STUN;
            pOutputIceServers[ currentIceServerIndex ].userNameLength = 0U;
            pOutputIceServers[ currentIceServerIndex ].passwordLength = 0U;
            pOutputIceServers[ currentIceServerIndex ].iceEndpoint.isPointToPoint = 0U;
            pOutputIceServers[ currentIceServerIndex ].iceEndpoint.transportAddress.port = 443;
            pOutputIceServers[ currentIceServerIndex ].url[ written ] = '\0'; /* It must be NULL terminated for DNS query. */
            pOutputIceServers[ currentIceServerIndex ].urlLength = written;
            currentIceServerIndex++;
        }
    }

    if( skipProcess == 0 )
    {
        /* Parse Ice server confgis into IceControllerIceServer_t structure. */
        for( i = 0; i < iceServerConfigsCount; i++ )
        {
            if( pIceServerConfigs[ i ].userNameLength > ICE_CONTROLLER_ICE_SERVER_USERNAME_MAX_LENGTH )
            {
                LogError( ( "The length of Ice server's username is too long to store, length: %u", pIceServerConfigs[ i ].userNameLength ) );
                continue;
            }
            else if( pIceServerConfigs[ i ].passwordLength > ICE_CONTROLLER_ICE_SERVER_PASSWORD_MAX_LENGTH )
            {
                LogError( ( "The length of Ice server's password is too long to store, length: %u", pIceServerConfigs[ i ].passwordLength ) );
                continue;
            }
            else if( currentIceServerIndex >= *pOutputIceServersCount )
            {
                LogWarn( ( "The size of Ice server buffer has no space for more server info, current index: %u, buffer size: %u, skipped server config idx: %lu",
                           currentIceServerIndex,
                           *pOutputIceServersCount,
                           i ) );
                break;
            }
            else
            {
                /* Do nothing, coverity happy. */
            }

            for( j = 0; j < pIceServerConfigs[ i ].iceServerUriCount; j++ )
            {
                if( currentIceServerIndex >= *pOutputIceServersCount )
                {
                    LogWarn( ( "The size of Ice server buffer has no space for more server info, current index: %u, buffer size: %u, skipped server URL: %.*s",
                               currentIceServerIndex,
                               *pOutputIceServersCount,
                               ( int ) pIceServerConfigs[ i ].iceServerUris[ j ].uriLength,
                               pIceServerConfigs[ i ].iceServerUris[ j ].uri ) );
                    break;
                }

                /* Parse each URI */
                parseResult = ParseIceServerUri( &pOutputIceServers[ currentIceServerIndex ],
                                                 pIceServerConfigs[ i ].iceServerUris[ j ].uri,
                                                 pIceServerConfigs[ i ].iceServerUris[ j ].uriLength );

                if( parseResult != 0 )
                {
                    continue;
                }

                memcpy( pOutputIceServers[ currentIceServerIndex ].userName,
                        pIceServerConfigs[ i ].userName,
                        pIceServerConfigs[ i ].userNameLength );
                pOutputIceServers[ currentIceServerIndex ].userNameLength = pIceServerConfigs[ i ].userNameLength;
                memcpy( pOutputIceServers[ currentIceServerIndex ].password,
                        pIceServerConfigs[ i ].password,
                        pIceServerConfigs[ i ].passwordLength );
                pOutputIceServers[ currentIceServerIndex ].passwordLength = pIceServerConfigs[ i ].passwordLength;
                currentIceServerIndex++;
            }
        }
    }

    if( skipProcess == 0 )
    {
        *pOutputIceServersCount = currentIceServerIndex;
    }

    return skipProcess;
}

#if ENABLE_TWCC_SUPPORT
/* Sample callback for TWCC. The average packet loss is tracked using an exponential moving average (EMA).
   - If packet loss stays at or below 5%, the bitrate increases by 5%.
   - If packet loss exceeds 5%, the bitrate decreases by the same percentage as the loss.
   The bitrate is adjusted once per second, ensuring it stays within predefined limits. */
static void SampleSenderBandwidthEstimationHandler( void * pCustomContext,
                                                    TwccBandwidthInfo_t * pTwccBandwidthInfo )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    PeerConnectionTwccMetaData_t * pTwccMetaData = NULL;
    uint64_t videoBitrate = 0;
    uint64_t audioBitrate = 0;
    uint64_t currentTimeUs = 0;
    uint64_t timeDifference = 0;
    uint32_t lostPacketCount = 0;
    uint8_t isLocked = 0;
    double percentLost = 0.0;

    if( ( pCustomContext == NULL ) ||
        ( pTwccBandwidthInfo == NULL ) )
    {
        LogError( ( "Invalid input, pCustomContext: %p, pTwccBandwidthInfo: %p",
                    pCustomContext, pTwccBandwidthInfo ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    // Calculate packet loss
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pTwccMetaData = ( PeerConnectionTwccMetaData_t * ) pCustomContext;

        currentTimeUs = NetworkingUtils_GetCurrentTimeUs( NULL );
        lostPacketCount = pTwccBandwidthInfo->sentPackets - pTwccBandwidthInfo->receivedPackets;
        percentLost = ( double ) ( ( pTwccBandwidthInfo->sentPackets > 0 ) ? ( ( double ) ( lostPacketCount * 100 ) / ( double )pTwccBandwidthInfo->sentPackets ) : 0.0 );
        timeDifference = currentTimeUs - pTwccMetaData->lastAdjustmentTimeUs;

        pTwccMetaData->averagePacketLoss = EMA_ACCUMULATOR_GET_NEXT( pTwccMetaData->averagePacketLoss,
                                                                     ( ( double ) percentLost ) );

        if( timeDifference < PEER_CONNECTION_TWCC_BITRATE_ADJUSTMENT_INTERVAL_US )
        {
            // Too soon for another adjustment
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_TWCC_INIT;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( xSemaphoreTake( pTwccMetaData->twccBitrateMutex,
                            portMAX_DELAY ) == pdTRUE )
        {
            isLocked = 1;
        }
        else
        {
            LogError( ( "Failed to lock Twcc mutex." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TAKE_TWCC_MUTEX;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        videoBitrate = pTwccMetaData->currentVideoBitrate;
        audioBitrate = pTwccMetaData->currentAudioBitrate;

        if( pTwccMetaData->averagePacketLoss <= 5 )
        {
            // Increase encoder bitrates by 5 percent with cap at MAX_BITRATE
            videoBitrate = ( uint64_t ) MIN( videoBitrate * 1.05,
                                             PEER_CONNECTION_MAX_VIDEO_BITRATE_KBPS );
            audioBitrate = ( uint64_t ) MIN( audioBitrate * 1.05,
                                             PEER_CONNECTION_MAX_AUDIO_BITRATE_BPS );
        }
        else
        {
            // Decrease encoder bitrate by average packet loss percent, with a cap at MIN_BITRATE
            videoBitrate = ( uint64_t ) MAX( videoBitrate * ( 1.0 - ( pTwccMetaData->averagePacketLoss / 100.0 ) ),
                                             PEER_CONNECTION_MIN_VIDEO_BITRATE_KBPS );
            audioBitrate = ( uint64_t ) MAX( audioBitrate * ( 1.0 - ( pTwccMetaData->averagePacketLoss / 100.0 ) ),
                                             PEER_CONNECTION_MIN_AUDIO_BITRATE_BPS );
        }

        pTwccMetaData->updatedVideoBitrate = videoBitrate;
        pTwccMetaData->updatedAudioBitrate = audioBitrate;
    }

    if( isLocked != 0 )
    {
        xSemaphoreGive( pTwccMetaData->twccBitrateMutex );

    }
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pTwccMetaData->lastAdjustmentTimeUs = currentTimeUs;

        LogInfo( ( "Adjusted made : average packet loss = %.2f%%, timeDifference = %llu us", pTwccMetaData->averagePacketLoss, timeDifference  ) );
        LogInfo( ( "Suggested video bitrate: %llu kbps, suggested audio bitrate: %llu bps, sent: %llu bytes, %llu packets,   received: %llu bytes, %llu packets, in %llu msec ",
                   videoBitrate, audioBitrate, pTwccBandwidthInfo->sentBytes, pTwccBandwidthInfo->sentPackets, pTwccBandwidthInfo->receivedBytes, pTwccBandwidthInfo->receivedPackets, pTwccBandwidthInfo->duration / 10000ULL ) );
    }

}
#endif

static int32_t InitializeAppSession( AppContext_t * pAppContext,
                                     AppSession_t * pAppSession )
{
    int32_t ret = 0;
    PeerConnectionResult_t peerConnectionResult;
    PeerConnectionSessionConfiguration_t pcConfig;

    memset( &pcConfig,
            0,
            sizeof( PeerConnectionSessionConfiguration_t ) );
    pcConfig.canTrickleIce = 1U;
    pcConfig.natTraversalConfigBitmap = pAppContext->natTraversalConfig;

    peerConnectionResult = PeerConnection_Init( &pAppSession->peerConnectionSession,
                                                &pcConfig );
    if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
    {
        LogWarn( ( "PeerConnection_Init fail, result: %d", peerConnectionResult ) );
        ret = -1;
    }

#if ENABLE_TWCC_SUPPORT
    if( peerConnectionResult == PEER_CONNECTION_RESULT_OK )
    {
        /* In case you want to set a different callback based on your business logic, you could replace SampleSenderBandwidthEstimationHandler() with your Handler. */
        peerConnectionResult = PeerConnection_SetSenderBandwidthEstimationCallback( &pAppSession->peerConnectionSession,
                                                                                    SampleSenderBandwidthEstimationHandler,
                                                                                    &pAppSession->peerConnectionSession.twccMetaData );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogError( ( "Fail to set Sender Bandwidth Estimation Callback, result: %d", peerConnectionResult ) );
            ret = -1;
        }
    }
#endif /* ENABLE_TWCC_SUPPORT */

    if( ret == 0 )
    {
        pAppSession->pSignalingControllerContext = &( pAppContext->signalingControllerContext );
    }

    return ret;
}

static PeerConnectionResult_t HandleRxVideoFrame( void * pCustomContext,
                                                  PeerConnectionFrame_t * pFrame )
{
    int32_t resultMedia = 0;
    AppContext_t * pAppContext = ( AppContext_t * ) pCustomContext;
    MediaFrame_t frame;

    if( pFrame != NULL )
    {
        LogDebug( ( "Received video frame with length: %u", pFrame->dataLength ) );

        memset( &frame, 0, sizeof( MediaFrame_t ) );
        frame.trackKind = TRANSCEIVER_TRACK_KIND_VIDEO;
        frame.pData = pFrame->pData;
        frame.size = pFrame->dataLength;
        frame.freeData = 0U;
        frame.timestampUs = pFrame->presentationUs;
        resultMedia = AppMediaSource_RecvFrame( pAppContext->pAppMediaSourcesContext,
                                                &frame );
        if( resultMedia != 0U )
        {
            LogDebug( ( "Dropping Rx video data with result: %ld", resultMedia ) );
        }
    }

    return PEER_CONNECTION_RESULT_OK;
}

static PeerConnectionResult_t HandleRxAudioFrame( void * pCustomContext,
                                                  PeerConnectionFrame_t * pFrame )
{
    int32_t resultMedia = 0;
    AppContext_t * pAppContext = ( AppContext_t * ) pCustomContext;
    MediaFrame_t frame;

    if( pFrame != NULL )
    {
        LogDebug( ( "Received audio frame with length: %u", pFrame->dataLength ) );

        memset( &frame, 0, sizeof( MediaFrame_t ) );
        frame.trackKind = TRANSCEIVER_TRACK_KIND_AUDIO;
        frame.pData = pFrame->pData;
        frame.size = pFrame->dataLength;
        frame.freeData = 0U;
        frame.timestampUs = pFrame->presentationUs;
        resultMedia = AppMediaSource_RecvFrame( pAppContext->pAppMediaSourcesContext,
                                                &frame );
        if( resultMedia != 0U )
        {
            LogDebug( ( "Dropping Rx audio data with result: %ld", resultMedia ) );
        }
    }

    return PEER_CONNECTION_RESULT_OK;
}

static void HandleSdpOffer( AppContext_t * pAppContext,
                            const SignalingMessage_t * pSignalingMessage )
{
    uint8_t skipProcess = 0;
    SignalingControllerResult_t signalingControllerReturn;
    const char * pSdpOfferMessage = NULL;
    size_t sdpOfferMessageLength = 0;
    PeerConnectionResult_t peerConnectionResult;
    PeerConnectionBufferSessionDescription_t bufferSessionDescription;
    size_t formalSdpMessageLength = 0;
    size_t sdpAnswerMessageLength = 0;
    SignalingControllerEventMessage_t eventMessage = {
        .event = SIGNALING_CONTROLLER_EVENT_SEND_WSS_MESSAGE,
        .onCompleteCallback = NULL,
        .pOnCompleteCallbackContext = NULL,
    };
    AppSession_t * pAppSession = NULL;

    if( ( pAppContext == NULL ) ||
        ( pSignalingMessage == NULL ) )
    {
        LogError( ( "Invalid input, pAppContext: %p, pSignalingMessage: %p", pAppContext, pSignalingMessage ) );
        skipProcess = 1;
    }

    if( skipProcess == 0 )
    {
        /* Get the SDP content in pSdpOfferMessage. */
        signalingControllerReturn = SignalingController_ExtractSdpMessageFromSignalingMessage( pSignalingMessage->pMessage,
                                                                                               pSignalingMessage->messageLength,
                                                                                               1U,
                                                                                               &pSdpOfferMessage,
                                                                                               &sdpOfferMessageLength );

        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to parse SDP offer content, result: %d, event message(%u): %.*s.",
                        signalingControllerReturn,
                        pSignalingMessage->messageLength,
                        ( int ) pSignalingMessage->messageLength,
                        pSignalingMessage->pMessage ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        /* Translate the newline into SDP formal format. The end pattern from signaling event message is "\\n" or "\\r\\n",
         * so we replace that with "\n" by calling this function. Note that this doesn't support inplace replacement. */
        formalSdpMessageLength = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;
        signalingControllerReturn = SignalingController_DeserializeSdpContentNewline( pSdpOfferMessage,
                                                                                      sdpOfferMessageLength,
                                                                                      pAppContext->sdpBuffer,
                                                                                      &formalSdpMessageLength );

        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to deserialize SDP offer newline, result: %d, event message(%u): %.*s.",
                        signalingControllerReturn,
                        pSignalingMessage->messageLength,
                        ( int ) pSignalingMessage->messageLength,
                        pSignalingMessage->pMessage ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        pAppSession = AppCommon_GetPeerConnectionSession( pAppContext,
                                                          pSignalingMessage->pRemoteClientId,
                                                          pSignalingMessage->remoteClientIdLength );

        if( pAppSession == NULL )
        {
            LogWarn( ( "No available peer connection session for remote client ID(%u): %.*s",
                       pSignalingMessage->remoteClientIdLength,
                       ( int ) pSignalingMessage->remoteClientIdLength,
                       pSignalingMessage->pRemoteClientId ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        bufferSessionDescription.pSdpBuffer = pAppContext->sdpBuffer;          /*  Memory to fill the actual sdp Buffer */
        bufferSessionDescription.sdpBufferLength = formalSdpMessageLength;
        bufferSessionDescription.type = SDP_CONTROLLER_MESSAGE_TYPE_OFFER;
        peerConnectionResult = PeerConnection_SetRemoteDescription( &pAppSession->peerConnectionSession,
                                                                    &bufferSessionDescription );

        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogWarn( ( "PeerConnection_SetRemoteDescription fail, result: %d, dropping ICE candidate.", peerConnectionResult ) );
        }
    }

    if( skipProcess == 0 )
    {
        peerConnectionResult = PeerConnection_SetVideoOnFrame( &pAppSession->peerConnectionSession,
                                                               HandleRxVideoFrame,
                                                               pAppContext );

        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogWarn( ( "PeerConnection_SetVideoOnFrame fail, result: %d.", peerConnectionResult ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        peerConnectionResult = PeerConnection_SetAudioOnFrame( &pAppSession->peerConnectionSession,
                                                               HandleRxAudioFrame,
                                                               pAppContext );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogWarn( ( "PeerConnection_SetAudioOnFrame fail, result: %d.", peerConnectionResult ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        memset( &bufferSessionDescription,
                0,
                sizeof( PeerConnectionBufferSessionDescription_t ) );
        bufferSessionDescription.pSdpBuffer = pAppContext->sdpBuffer;
        bufferSessionDescription.sdpBufferLength = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;
        peerConnectionResult = PeerConnection_SetLocalDescription( &pAppSession->peerConnectionSession,
                                                                   &bufferSessionDescription );

        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogWarn( ( "PeerConnection_SetLocalDescription fail, result: %d.", peerConnectionResult ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        pAppContext->sdpConstructedBufferLength = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;
        peerConnectionResult = PeerConnection_CreateAnswer( &pAppSession->peerConnectionSession,
                                                            &bufferSessionDescription,
                                                            pAppContext->sdpConstructedBuffer,
                                                            &pAppContext->sdpConstructedBufferLength );

        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogWarn( ( "PeerConnection_CreateAnswer fail, result: %d.", peerConnectionResult ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        /* Translate from SDP formal format into signaling event message by replacing newline with "\\n" or "\\r\\n". */
        sdpAnswerMessageLength = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;
        signalingControllerReturn = SignalingController_SerializeSdpContentNewline( pAppContext->sdpConstructedBuffer,
                                                                                    pAppContext->sdpConstructedBufferLength,
                                                                                    pAppContext->sdpBuffer,
                                                                                    &sdpAnswerMessageLength );

        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to serialize SDP answer newline, result: %d, constructed buffer(%u): %.*s",
                        signalingControllerReturn,
                        pAppContext->sdpConstructedBufferLength,
                        ( int ) pAppContext->sdpConstructedBufferLength,
                        pAppContext->sdpConstructedBuffer ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        eventMessage.eventContent.correlationIdLength = 0U;
        memset( eventMessage.eventContent.correlationId,
                0,
                SECRET_ACCESS_KEY_MAX_LEN );
        eventMessage.eventContent.messageType = SIGNALING_TYPE_MESSAGE_SDP_ANSWER;
        eventMessage.eventContent.pDecodeMessage = pAppContext->sdpBuffer;
        eventMessage.eventContent.decodeMessageLength = sdpAnswerMessageLength;
        memcpy( eventMessage.eventContent.remoteClientId,
                pSignalingMessage->pRemoteClientId,
                pSignalingMessage->remoteClientIdLength );
        eventMessage.eventContent.remoteClientIdLength = pSignalingMessage->remoteClientIdLength;

        signalingControllerReturn = SignalingController_SendMessage( &pAppContext->signalingControllerContext,
                                                                     &eventMessage );

        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            skipProcess = 1;
            LogError( ( "Send signaling message fail, result: %d", signalingControllerReturn ) );
        }
    }
}

static void HandleSdpAnswer( AppContext_t * pAppContext,
                             const SignalingMessage_t * pSignalingMessage )
{
    uint8_t skipProcess = 0;
    SignalingControllerResult_t signalingControllerReturn;
    const char * pSdpAnswerMessage = NULL;
    size_t sdpAnswerMessageLength = 0;
    PeerConnectionResult_t peerConnectionResult;
    PeerConnectionBufferSessionDescription_t bufferSessionDescription;
    size_t formalSdpMessageLength = 0;
    AppSession_t * pAppSession = NULL;

    if( ( pAppContext == NULL ) ||
        ( pSignalingMessage == NULL ) )
    {
        LogError( ( "Invalid input, pAppContext: %p, pEvent: %p", pAppContext, pSignalingMessage ) );
        skipProcess = 1;
    }

    if( skipProcess == 0 )
    {
        /* Get the SDP content in SDP answer. */
        signalingControllerReturn = SignalingController_ExtractSdpMessageFromSignalingMessage( pSignalingMessage->pMessage,
                                                                                               pSignalingMessage->messageLength,
                                                                                               0U,
                                                                                               &pSdpAnswerMessage,
                                                                                               &sdpAnswerMessageLength );
        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to parse SDP offer content, result: %d, event message(%u): %.*s.",
                        signalingControllerReturn,
                        pSignalingMessage->messageLength,
                        ( int ) pSignalingMessage->messageLength,
                        pSignalingMessage->pMessage ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        /* Translate the newline into SDP formal format. The end pattern from signaling event message is "\\n" or "\\r\\n",
         * so we replace that with "\n" by calling this function. Note that this doesn't support inplace replacement. */
        formalSdpMessageLength = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;
        signalingControllerReturn = SignalingController_DeserializeSdpContentNewline( pSdpAnswerMessage,
                                                                                      sdpAnswerMessageLength,
                                                                                      pAppContext->sdpBuffer,
                                                                                      &formalSdpMessageLength );
        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to deserialize SDP offer newline, result: %d, event message(%u): %.*s.",
                        signalingControllerReturn,
                        pSignalingMessage->messageLength,
                        ( int ) pSignalingMessage->messageLength,
                        pSignalingMessage->pMessage ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        pAppSession = AppCommon_GetPeerConnectionSession( pAppContext,
                                                          pSignalingMessage->pRemoteClientId,
                                                          pSignalingMessage->remoteClientIdLength );
        if( pAppSession == NULL )
        {
            LogWarn( ( "No available peer connection session for remote client ID(%u): %.*s",
                       pSignalingMessage->remoteClientIdLength,
                       ( int ) pSignalingMessage->remoteClientIdLength,
                       pSignalingMessage->pRemoteClientId ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        bufferSessionDescription.pSdpBuffer = pAppContext->sdpBuffer;
        bufferSessionDescription.sdpBufferLength = formalSdpMessageLength;
        bufferSessionDescription.type = SDP_CONTROLLER_MESSAGE_TYPE_ANSWER;
        peerConnectionResult = PeerConnection_SetRemoteDescription( &pAppSession->peerConnectionSession,
                                                                    &bufferSessionDescription );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogWarn( ( "PeerConnection_SetRemoteDescription fail, result: %d, dropping ICE candidate.", peerConnectionResult ) );
        }
    }

    if( skipProcess == 0 )
    {
        peerConnectionResult = PeerConnection_SetVideoOnFrame( &pAppSession->peerConnectionSession,
                                                               HandleRxVideoFrame,
                                                               pAppContext );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogWarn( ( "PeerConnection_SetVideoOnFrame fail, result: %d.", peerConnectionResult ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        peerConnectionResult = PeerConnection_SetAudioOnFrame( &pAppSession->peerConnectionSession,
                                                               HandleRxAudioFrame,
                                                               pAppContext );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogWarn( ( "PeerConnection_SetAudioOnFrame fail, result: %d.", peerConnectionResult ) );
            skipProcess = 1;
        }
    }
}

static void HandleRemoteCandidate( AppContext_t * pAppContext,
                                   const SignalingMessage_t * pSignalingMessage )
{
    uint8_t skipProcess = 0;
    PeerConnectionResult_t peerConnectionResult;
    AppSession_t * pAppSession = NULL;

    pAppSession = AppCommon_GetPeerConnectionSession( pAppContext,
                                                      pSignalingMessage->pRemoteClientId,
                                                      pSignalingMessage->remoteClientIdLength );

    if( pAppSession == NULL )
    {
        LogWarn( ( "No available peer connection session for remote client ID(%u): %.*s",
                   pSignalingMessage->remoteClientIdLength,
                   ( int ) pSignalingMessage->remoteClientIdLength,
                   pSignalingMessage->pRemoteClientId ) );
        skipProcess = 1;
    }

    if( skipProcess == 0 )
    {
        peerConnectionResult = PeerConnection_AddRemoteCandidate( &pAppSession->peerConnectionSession,
                                                                  pSignalingMessage->pMessage,
                                                                  pSignalingMessage->messageLength );

        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogWarn( ( "PeerConnection_AddRemoteCandidate fail, result: %d, dropping ICE candidate.", peerConnectionResult ) );
        }
    }
}

static const char * GetCandidateTypeString( IceCandidateType_t candidateType )
{
    const char * ret;

    switch( candidateType )
    {
        case ICE_CANDIDATE_TYPE_HOST:
            ret = DEMO_CANDIDATE_TYPE_HOST_STRING;
            break;

        case ICE_CANDIDATE_TYPE_PEER_REFLEXIVE:
            ret = DEMO_CANDIDATE_TYPE_PRFLX_STRING;
            break;

        case ICE_CANDIDATE_TYPE_SERVER_REFLEXIVE:
            ret = DEMO_CANDIDATE_TYPE_SRFLX_STRING;
            break;

        case ICE_CANDIDATE_TYPE_RELAY:
            ret = DEMO_CANDIDATE_TYPE_RELAY_STRING;
            break;

        default:
            ret = DEMO_CANDIDATE_TYPE_UNKNOWN_STRING;
            break;
    }

    return ret;
}

static void HandleLocalCandidateReady( void * pCustomContext,
                                       PeerConnectionIceLocalCandidate_t * pIceLocalCandidate )
{
    uint8_t skipProcess = 0;
    AppSession_t * pAppSession = ( AppSession_t * ) pCustomContext;
    SignalingControllerResult_t signalingControllerReturn;
    SignalingControllerEventMessage_t eventMessage = {
        .event = SIGNALING_CONTROLLER_EVENT_SEND_WSS_MESSAGE,
        .onCompleteCallback = OnSendIceCandidateComplete,
        .pOnCompleteCallbackContext = NULL,
    };
    int written;
    char * pBuffer;
    char candidateStringBuffer[ DEMO_JSON_CANDIDATE_MAX_LENGTH ];

    if( ( pAppSession == NULL ) ||
        ( pIceLocalCandidate == NULL ) )
    {
        /* Log for debugging. */
        LogWarn( ( "Invalid local candidate ready event, pAppSession: %p, pIceLocalCandidate: %p",
                   pAppSession,
                   pIceLocalCandidate ) );

        skipProcess = 1;
    }

    if( skipProcess == 0 )
    {
        if( pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.family == STUN_ADDRESS_IPv4 )
        {
            written = snprintf( candidateStringBuffer,
                                DEMO_JSON_CANDIDATE_MAX_LENGTH,
                                DEMO_ICE_CANDIDATE_JSON_IPV4_TEMPLATE,
                                pIceLocalCandidate->localCandidateIndex,
                                pIceLocalCandidate->pLocalCandidate->priority,
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 0 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 1 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 2 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 3 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.port,
                                GetCandidateTypeString( pIceLocalCandidate->pLocalCandidate->candidateType ) );
        }
        else
        {
            written = snprintf( candidateStringBuffer,
                                DEMO_JSON_CANDIDATE_MAX_LENGTH,
                                DEMO_ICE_CANDIDATE_JSON_IPV6_TEMPLATE,
                                pIceLocalCandidate->localCandidateIndex,
                                pIceLocalCandidate->pLocalCandidate->priority,
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 0 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 1 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 2 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 3 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 4 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 5 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 6 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 7 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 8 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 9 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 10 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 11 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 12 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 13 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 14 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[ 15 ],
                                pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.port,
                                GetCandidateTypeString( pIceLocalCandidate->pLocalCandidate->candidateType ) );
        }

        if( written < 0 )
        {
            LogError( ( "snprintf returns fail, error: %d", written ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        /* Format this into candidate string. */
        pBuffer = ( char * ) malloc( DEMO_JSON_CANDIDATE_MAX_LENGTH );
        LogVerbose( ( "Allocating buffer at %p", pBuffer ) );
        memset( pBuffer,
                0,
                DEMO_JSON_CANDIDATE_MAX_LENGTH );

        written = snprintf( pBuffer,
                            DEMO_JSON_CANDIDATE_MAX_LENGTH,
                            DEMO_ICE_CANDIDATE_JSON_TEMPLATE,
                            written,
                            candidateStringBuffer );

        if( written < 0 )
        {
            LogError( ( "snprintf returns fail, error: %d", written ) );
            skipProcess = 1;
            free( pBuffer );
        }
    }

    if( skipProcess == 0 )
    {
        eventMessage.eventContent.correlationIdLength = 0U;
        eventMessage.eventContent.messageType = SIGNALING_TYPE_MESSAGE_ICE_CANDIDATE;
        eventMessage.eventContent.pDecodeMessage = pBuffer;
        eventMessage.eventContent.decodeMessageLength = written;
        memcpy( eventMessage.eventContent.remoteClientId,
                pAppSession->remoteClientId,
                pAppSession->remoteClientIdLength );
        eventMessage.eventContent.remoteClientIdLength = pAppSession->remoteClientIdLength;

        /* We dynamically allocate buffer for signaling controller to keep using it.
         * callback it as context to free memory. */
        eventMessage.pOnCompleteCallbackContext = pBuffer;

        signalingControllerReturn = SignalingController_SendMessage( pAppSession->pSignalingControllerContext,
                                                                     &eventMessage );

        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Send signaling message fail, result: %d", signalingControllerReturn ) );
            skipProcess = 1;
            free( pBuffer );
        }
        else
        {
            LogDebug( ( "Sent local candidate to remote peer, msg(%d): %.*s",
                        written,
                        written,
                        pBuffer ) );
        }
    }
}

static int32_t OnSendIceCandidateComplete( SignalingControllerEventStatus_t status,
                                           void * pUserContext )
{
    LogDebug( ( "Freeing buffer at %p", pUserContext ) );
    free( pUserContext );

    return 0;
}

static void OnSignalingConnectionStateChange( SignalingControllerConnectionState_t state,
                                              void * pUserData )
{
    AppContext_t * pAppContext = ( AppContext_t * ) pUserData;
    static uint8_t isFirst = 1U;
    uint8_t skipProcess = 0U;
    EventBits_t uxBits = 0U;

    if( pAppContext == NULL )
    {
        LogError( ( "Unexpected callback with NULL context." ) );
        skipProcess = 1U;
    }

    if( skipProcess == 0U )
    {
        if( ( isFirst != 0U ) &&
            ( state == SIGNALING_CONTROLLER_STATE_CONNECTED ) )
        {
            uxBits = xEventGroupSetBits( pAppContext->signalingConnectionBarrier,
                                         APP_COMMON_SIGNALING_CONTROLER_START_UP_BARRIER_BIT );
            /* Since we configure xClearBitOnExit while waiting, the return value might be cleared automatically.
             * Thus we don't check return value here. */
            ( void ) uxBits;

            isFirst = 0U;
            LogInfo( ( "Unblock signaling connection barrier." ) );
        }
    }
}

static int OnSignalingMessageReceived( SignalingMessage_t * pSignalingMessage,
                                       void * pUserData )
{
    AppContext_t * pAppContext = ( AppContext_t * ) pUserData;

    LogDebug( ( "Received Message from websocket server!" ) );
    LogDebug( ( "Message Type: %x", pSignalingMessage->messageType ) );
    LogDebug( ( "Sender ID: %.*s", ( int ) pSignalingMessage->remoteClientIdLength, pSignalingMessage->pRemoteClientId ) );
    LogDebug( ( "Correlation ID: %.*s", ( int ) pSignalingMessage->correlationIdLength, pSignalingMessage->pCorrelationId ) );
    LogDebug( ( "Message Length: %u, Message:", pSignalingMessage->messageLength ) );
    LogDebug( ( "%.*s", ( int ) pSignalingMessage->messageLength, pSignalingMessage->pMessage ) );

    switch( pSignalingMessage->messageType )
    {
        case SIGNALING_TYPE_MESSAGE_SDP_OFFER:
            #if METRIC_PRINT_ENABLED
            Metric_StartEvent( METRIC_EVENT_SENDING_FIRST_FRAME );
            #endif
            HandleSdpOffer( pAppContext,
                            pSignalingMessage );
            break;

        case SIGNALING_TYPE_MESSAGE_SDP_ANSWER:
            HandleSdpAnswer( pAppContext,
                             pSignalingMessage );
            break;

        case SIGNALING_TYPE_MESSAGE_ICE_CANDIDATE:
            HandleRemoteCandidate( pAppContext,
                                   pSignalingMessage );
            break;

        case SIGNALING_TYPE_MESSAGE_STATUS_RESPONSE:
            break;

        default:
            break;
    }

    return 0;
}

#if ENABLE_SCTP_DATA_CHANNEL
#if ( DATACHANNEL_CUSTOM_CALLBACK_HOOK != 0 )

static void OnDataChannelMessage( PeerConnectionDataChannel_t * pDataChannel,
                                  uint8_t isBinary,
                                  uint8_t * pMessage,
                                  uint32_t pMessageLen )
{
    char ucSendMessage[DEFAULT_DATA_CHANNEL_ON_MESSAGE_BUFFER_SIZE];
    PeerConnectionResult_t retStatus = PEER_CONNECTION_RESULT_OK;
    if( ( pMessage == NULL ) || ( pDataChannel == NULL ) )
    {
        LogError( ( "No message or pDataChannel received in OnDataChannelMessage" ) );
        return;
    }

    if( isBinary )
    {
        LogWarn( ( "[VIEWER] [Peer: %s Channel Name: %s] >>> DataChannel Binary Message",
                   pDataChannel->pPeerConnection->combinedName,
                   pDataChannel->ucDataChannelName ) );
    }
    else {
        LogWarn( ( "[VIEWER] [Peer: %s Channel Name: %s] >>> DataChannel String Message: %.*s\n",
                   pDataChannel->pPeerConnection->combinedName,
                   pDataChannel->ucDataChannelName,
                   ( int ) pMessageLen, pMessage ) );

        sprintf( ucSendMessage, "Received %ld bytes, ECHO: %.*s", ( long int ) pMessageLen, ( int ) ( pMessageLen > ( DEFAULT_DATA_CHANNEL_ON_MESSAGE_BUFFER_SIZE - 128 ) ? ( DEFAULT_DATA_CHANNEL_ON_MESSAGE_BUFFER_SIZE - 128 ) : pMessageLen ), pMessage );
        retStatus = PeerConnectionSCTP_DataChannelSend( pDataChannel, 0U, ( uint8_t * ) ucSendMessage, strlen( ucSendMessage ) );
    }

    if( retStatus != PEER_CONNECTION_RESULT_OK )
    {
        LogWarn( ( "[KVS Master] OnDataChannelMessage(): operation returned status code: 0x%08x \n", ( unsigned int ) retStatus ) );
    }

}

OnDataChannelMessageReceived_t PeerConnectionSCTP_SetChannelOnMessageCallbackHook( PeerConnectionSession_t * pPeerConnectionSession,
                                                                                   uint32_t ulChannelId,
                                                                                   const uint8_t * pucName,
                                                                                   uint32_t ulNameLen )
{
    ( void ) pPeerConnectionSession;
    ( void ) ulChannelId;
    ( void ) pucName;
    ( void ) ulNameLen;

    return OnDataChannelMessage;
}

#endif /* (DATACHANNEL_CUSTOM_CALLBACK_HOOK != 0) */
#endif /* ENABLE_SCTP_DATA_CHANNEL */

int AppCommon_Init( AppContext_t * pAppContext,
                    InitTransceiverFunc_t initTransceiverFunc,
                    void * pMediaContext )
{
    int ret = 0;
    SignalingControllerResult_t signalingControllerReturn;
    SSLCredentials_t sslCreds;
    int i;

    if( pAppContext == NULL )
    {
        LogError( ( "Invalid input, demo context is NULL" ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        platform_init();

        memset( pAppContext, 0, sizeof( AppContext_t ) );
        memset( &sslCreds, 0, sizeof( SSLCredentials_t ) );

        srtp_init();

        #if ENABLE_SCTP_DATA_CHANNEL
        Sctp_Init();
        #endif /* ENABLE_SCTP_DATA_CHANNEL */

        /* Configure random seed. In FreeRTOS environment, Sctp_Init() calls srand() with getpid() 
         * which always returns 0xFFFFFFFF, resulting in a fixed random sequence. Therefore, we must 
         * call srand() after Sctp_Init() to ensure proper randomization. */
        srand( NetworkingUtils_GetCurrentTimeSec( NULL ) );

        pAppContext->natTraversalConfig = ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_ALLOW_ALL;
        pAppContext->initTransceiverFunc = initTransceiverFunc;
        pAppContext->pAppMediaSourcesContext = pMediaContext;
    }

    if( ret == 0 )
    {
        pAppContext->signalingConnectionBarrier = xEventGroupCreate();
        if( pAppContext->signalingConnectionBarrier == NULL )
        {
            LogError( ( "Failed to create signaling connection barrier, errno: %d", errno ) );
            ret = -1;
        }
    }

    if( ret == 0 )
    {
        #if defined( AWS_CA_CERT_PEM )
        sslCreds.pCaCertPem = ( uint8_t * ) AWS_CA_CERT_PEM;
        #endif /* #if defined( AWS_CA_CERT_PEM ) */

        #if defined( AWS_IOT_THING_ROLE_ALIAS )
        sslCreds.pDeviceCertPem = ( uint8_t * ) AWS_IOT_THING_CERT;
        sslCreds.pDeviceKeyPem = ( uint8_t * ) AWS_IOT_THING_PRIVATE_KEY;
        #endif

        signalingControllerReturn = SignalingController_Init( &pAppContext->signalingControllerContext, &sslCreds );

        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to initialize signaling controller." ) );
            ret = -1;
        }
    }

    if( ret == 0 )
    {
        signalingControllerReturn = SignalingController_SetConnectionStateCallback( &pAppContext->signalingControllerContext,
                                                                                    OnSignalingConnectionStateChange,
                                                                                    pAppContext );
        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to set signaling connection state callback." ) );
            ret = -1;
        }
    }

    #if METRIC_PRINT_ENABLED
    if( ret == 0 )
    {
        /* Initialize Metric. */
        Metric_Init();
    }
    #endif

    if( ret == 0 )
    {
        for( i = 0; i < AWS_MAX_VIEWER_NUM; i++ )
        {
            ret = InitializeAppSession( pAppContext,
                                        &( pAppContext->appSessions[ i ] ) );
            if( ret != 0 )
            {
                LogError( ( "Fail to initialize peer connection sessions." ) );
                break;
            }
        }
    }

    return ret;
}

int AppCommon_StartSignalingController( AppContext_t * pAppContext )
{
    int32_t ret = 0;
    EventBits_t uxBits = 0U;

    if( pAppContext == NULL )
    {
        LogError( ( "Invalid parameter, pAppContext: %p", pAppContext ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        if( xTaskCreate( SignalingController_Task,
                         "SigController",
                         20480,
                         pAppContext,
                         tskIDLE_PRIORITY + 4,
                         pAppContext->pSignalingControllerTaskHandler ) != pdPASS )
        {
            LogError( ( "xTaskCreate(SigController) failed" ) );
            ret = -1;
        }
    }

    if( ret == 0 )
    {
        uxBits = xEventGroupWaitBits( pAppContext->signalingConnectionBarrier,
                                      APP_COMMON_SIGNALING_CONTROLER_START_UP_BARRIER_BIT,
                                      pdTRUE,
                                      pdTRUE,
                                      portMAX_DELAY );
        if( ( uxBits & APP_COMMON_SIGNALING_CONTROLER_START_UP_BARRIER_BIT ) != APP_COMMON_SIGNALING_CONTROLER_START_UP_BARRIER_BIT )
        {
            LogError( ( "Unexpected return value from signaling controller start up barrier, uxBits: 0x%lx", uxBits ) );
            ret = -1;
        }
    }

    return ret;
}

void AppCommon_WaitSignalingControllerStop( AppContext_t * pAppContext )
{
    uint8_t skipProcess = 0U;

    if( pAppContext == NULL )
    {
        LogError( ( "Invalid parameter, pAppContext: %p", pAppContext ) );
        skipProcess = 1;
    }

    if( skipProcess == 0 )
    {
        /*
         * In FreeRTOS, the signaling controller runs as an endless task.
         * When a user invokes this API, they expect it to wait until signaling
         * completes, but this would cause unexpected blocking behavior.
         * Therefore, we simply delete the current task to free resources.
         */
        vTaskDelete( NULL );
    }
}

AppSession_t * AppCommon_GetPeerConnectionSession( AppContext_t * pAppContext,
                                                   const char * pRemoteClientId,
                                                   size_t remoteClientIdLength )
{
    AppSession_t * pAppSession = NULL;
    int i;
    int32_t initResult;

    for( i = 0; i < AWS_MAX_VIEWER_NUM; i++ )
    {
        if( ( pAppContext->appSessions[ i ].remoteClientIdLength == remoteClientIdLength ) &&
            ( ( remoteClientIdLength == 0 ) ||
              ( strncmp( pAppContext->appSessions[ i ].remoteClientId,
                         pRemoteClientId,
                         remoteClientIdLength ) == 0 ) ) )
        {
            /* Found existing session. */
            pAppSession = &pAppContext->appSessions[ i ];
            break;
        }
        else if( ( pAppSession == NULL ) &&
                 ( pAppContext->appSessions[i].peerConnectionSession.state == PEER_CONNECTION_SESSION_STATE_INITED ) )
        {
            /* Found free session, keep looping to find existing one. */
            pAppSession = &pAppContext->appSessions[ i ];
        }
        else
        {
            /* Do nothing. */
        }
    }

    if( ( pAppSession != NULL ) && ( pAppSession->peerConnectionSession.state == PEER_CONNECTION_SESSION_STATE_INITED ) )
    {
        LogInfo( ( "Start peer connection on idx: %d for client ID(%u): %.*s",
                   i,
                   remoteClientIdLength,
                   ( int ) remoteClientIdLength,
                   pRemoteClientId ) );

        initResult = StartPeerConnectionSession( pAppContext,
                                                 pAppSession,
                                                 pRemoteClientId,
                                                 remoteClientIdLength );

        if( initResult != 0 )
        {
            pAppSession = NULL;
            LogError( ( "Fail to start peer connection session, result: %ld", initResult ) );
        }
    }

    return pAppSession;
}
