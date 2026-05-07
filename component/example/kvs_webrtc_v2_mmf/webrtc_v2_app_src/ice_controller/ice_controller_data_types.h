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

#ifndef ICE_CONTROLLER_DATA_TYPES_H
#define ICE_CONTROLLER_DATA_TYPES_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes. */
#include <stdint.h>
#include "demo_config.h"
#include "ice_data_types.h"
#include "timer_controller.h"
#include "lwip/sockets.h"
#include "transport_mbedtls.h"
#include "transport_dtls_mbedtls.h"

/* FreeRTOS includes. */
#include "semphr.h"

/**
 * Set default maximum Ice server count to 7.
 * Note that the first Ice server is the default STUN server.
 */
#define ICE_CONTROLLER_MAX_ICE_SERVER_COUNT ( 7 )

#define ICE_CONTROLLER_IP_ADDR_STRING_BUFFER_LENGTH ( 39 )
#define ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE ( 1024 )

/**
 * Maximum allowed ICE URI length
 */
#define ICE_CONTROLLER_ICE_SERVER_URL_MAX_LENGTH ( 256 )

/**
 * Maximum allowed ICE configuration user name length
 * https://docs.aws.amazon.com/kinesisvideostreams/latest/dg/API_AWSAcuitySignalingService_GetIceServerConfig.html#API_AWSAcuitySignalingService_GetIceServerConfig_RequestSyntax
 */
#define ICE_CONTROLLER_ICE_SERVER_USERNAME_MAX_LENGTH ( 256 )

/**
 * Maximum allowed ICE configuration password length
 * https://docs.aws.amazon.com/kinesisvideostreams/latest/dg/API_AWSAcuitySignalingService_IceServer.html#KinesisVideo-Type-AWSAcuitySignalingService_IceServer-Password
 */
#define ICE_CONTROLLER_ICE_SERVER_PASSWORD_MAX_LENGTH ( 256 )

#define ICE_CONTROLLER_MAX_CANDIDATE_PAIR_COUNT       ( 1024 )
#define ICE_CONTROLLER_MAX_LOCAL_CANDIDATE_COUNT      ( 100 )
#define ICE_CONTROLLER_MAX_REMOTE_CANDIDATE_COUNT     ( 100 )

#define ICE_CONTROLLER_PRINT_CONNECTIVITY_CHECK_PERIOD_MS ( 10000 )

#if LIBRARY_LOG_LEVEL >= LOG_DEBUG
#define ICE_CONTROLLER_CONNECTIVITY_TIMER_INTERVAL_MS ( 5000 )
#else
#define ICE_CONTROLLER_CONNECTIVITY_TIMER_INTERVAL_MS ( 200 )
#endif /* LIBRARY_LOG_LEVEL >= LOG_VERBOSE */
#define ICE_CONTROLLER_PERIODIC_TIMER_INTERVAL_MS ( 1000 )
#define ICE_CONTROLLER_CLOSING_INTERVAL_MS ( 100 )

/* Expiration timeout in mili-seconds. */
#define ICE_CONTROLLER_CONNECTIVITY_CHECK_TIMEOUT_MS ( 24000 )

#define ICE_CONTROLLER_MAX_PATH_LENGTH ( 2048 )
#define ICE_CONTROLLER_MAX_PEM_LENGTH ( 2048 )

#define ICE_CONTROLLER_MAX_MTU ( 1500 )

typedef enum IceControllerSocketType
{
    ICE_CONTROLLER_SOCKET_TYPE_NONE = 0,
    ICE_CONTROLLER_SOCKET_TYPE_TCP,
    ICE_CONTROLLER_SOCKET_TYPE_TLS,
    ICE_CONTROLLER_SOCKET_TYPE_UDP,
    ICE_CONTROLLER_SOCKET_TYPE_DTLS,
} IceControllerSocketType_t;

typedef enum IceControllerCallbackEvent
{
    ICE_CONTROLLER_CB_EVENT_NONE = 0,
    ICE_CONTROLLER_CB_EVENT_LOCAL_CANDIDATE_READY,
    ICE_CONTROLLER_CB_EVENT_PROCESS_ICE_CANDIDATES_AND_PAIRS,
    ICE_CONTROLLER_CB_EVENT_PEER_TO_PEER_CONNECTION_FOUND,
    ICE_CONTROLLER_CB_EVENT_PERIODIC_CONNECTION_CHECK,
    ICE_CONTROLLER_CB_EVENT_ICE_CLOSING,
    ICE_CONTROLLER_CB_EVENT_ICE_CLOSED,
    ICE_CONTROLLER_CB_EVENT_ICE_CLOSE_NOTIFY,
    ICE_CONTROLLER_CB_EVENT_MAX,
} IceControllerCallbackEvent_t;

typedef struct IceControllerLocalCandidateReadyMsg
{
    const IceCandidate_t * pLocalCandidate;
    size_t localCandidateIndex;
} IceControllerLocalCandidateReadyMsg_t;

typedef struct IceControllerCallbackContent
{
    union
    {
        IceControllerLocalCandidateReadyMsg_t localCandidateReadyMsg; /* ICE_CONTROLLER_CB_EVENT_LOCAL_CANDIDATE_READY */
        /* NULL for ICE_CONTROLLER_CB_EVENT_PROCESS_ICE_CANDIDATES_AND_PAIRS */
        /* NULL for ICE_CONTROLLER_CB_EVENT_PEER_TO_PEER_CONNECTION_FOUND */
        /* NULL for ICE_CONTROLLER_CB_EVENT_PERIODIC_CONNECTION_CHECK */
    } iceControllerCallbackContent;
} IceControllerCallbackContent_t;

typedef int32_t (* OnIceEventCallback_t)( void * pCustomContext,
                                          IceControllerCallbackEvent_t event,
                                          IceControllerCallbackContent_t * pEventMsg );

typedef int32_t (* OnRecvNonStunPacketCallback_t)( void * pCustomContext,
                                                   uint8_t * pBuffer,
                                                   size_t bufferLength );

typedef enum IceControllerResult
{
    /* Info codes. */
    ICE_CONTROLLER_RESULT_OK = 0,
    ICE_CONTROLLER_RESULT_FOUND_CONNECTION,
    ICE_CONTROLLER_RESULT_CONNECTION_IN_PROGRESS,
    ICE_CONTROLLER_RESULT_CONNECTION_CLOSED,
    ICE_CONTROLLER_RESULT_CONTEXT_ALREADY_CLOSED,
    ICE_CONTROLLER_RESULT_NOT_STUN_PACKET,
    ICE_CONTROLLER_RESULT_CONNECTIVITY_CHECK_TIMEOUT,

    /* Error codes. */
    ICE_CONTROLLER_RESULT_BAD_PARAMETER,
    ICE_CONTROLLER_RESULT_IPV6_NOT_SUPPORT,
    ICE_CONTROLLER_RESULT_IP_BUFFER_TOO_SMALL,
    ICE_CONTROLLER_RESULT_CANDIDATE_SEND_FAIL,
    ICE_CONTROLLER_RESULT_INVALID_IP_ADDR,
    ICE_CONTROLLER_RESULT_INVALID_JSON,
    ICE_CONTROLLER_RESULT_INVALID_PROTOCOL,
    ICE_CONTROLLER_RESULT_INVALID_PACKET,
    ICE_CONTROLLER_RESULT_FAIL_CREATE_ICE_AGENT,
    ICE_CONTROLLER_RESULT_FAIL_SOCKET_CREATE,
    ICE_CONTROLLER_RESULT_FAIL_SOCKET_BIND,
    ICE_CONTROLLER_RESULT_FAIL_SOCKET_CONNECT,
    ICE_CONTROLLER_RESULT_FAIL_SOCKET_NTOP,
    ICE_CONTROLLER_RESULT_FAIL_SOCKET_TYPE,
    ICE_CONTROLLER_RESULT_FAIL_SOCKET_GETSOCKNAME,
    ICE_CONTROLLER_RESULT_FAIL_SOCKET_SENDTO,
    ICE_CONTROLLER_RESULT_FAIL_ADD_HOST_CANDIDATE,
    ICE_CONTROLLER_RESULT_FAIL_ADD_RELAY_CANDIDATE,
    ICE_CONTROLLER_RESULT_FAIL_ADD_REMOTE_CANDIDATE,
    ICE_CONTROLLER_RESULT_FAIL_ADD_IPv6_REMOTE_CANDIDATE,
    ICE_CONTROLLER_RESULT_FAIL_ADD_NON_UDP_REMOTE_CANDIDATE,
    ICE_CONTROLLER_RESULT_FAIL_ADD_CANDIDATE_TYPE,
    ICE_CONTROLLER_RESULT_FAIL_TIMER_INIT,
    ICE_CONTROLLER_RESULT_FAIL_DNS_QUERY,
    ICE_CONTROLLER_RESULT_FAIL_SET_CONNECTIVITY_CHECK_TIMER,
    ICE_CONTROLLER_RESULT_FAIL_QUERY_CANDIDATE_PAIR_COUNT,
    ICE_CONTROLLER_RESULT_FAIL_QUERY_LOCAL_CANDIDATE_COUNT,
    ICE_CONTROLLER_RESULT_FAIL_MUTEX_CREATE,
    ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE,
    ICE_CONTROLLER_RESULT_FAIL_CONNECTION_NOT_READY,
    ICE_CONTROLLER_RESULT_FAIL_CREATE_TURN_CHANNEL_DATA,
    ICE_CONTROLLER_RESULT_FAIL_SEND_BIND_RESPONSE,
    ICE_CONTROLLER_RESULT_FAIL_SEND_NOMINATION_REQUEST,
    ICE_CONTROLLER_RESULT_FAIL_FIND_SOCKET_CONTEXT,
    ICE_CONTROLLER_RESULT_FAIL_FIND_NOMINATED_CONTEXT,
    ICE_CONTROLLER_RESULT_FAIL_SOCKET_CONTEXT_ALREADY_CLOSED,
    ICE_CONTROLLER_RESULT_FAIL_EXCEED_MTU,
    ICE_CONTROLLER_RESULT_FAIL_CREATE_NEXT_PAIR_REQUEST,
    ICE_CONTROLLER_RESULT_NO_SOCKET_CONTEXT_AVAILABLE,
    ICE_CONTROLLER_RESULT_JSON_CANDIDATE_NOT_FOUND,
    ICE_CONTROLLER_RESULT_JSON_CANDIDATE_INVALID_PRIORITY,
    ICE_CONTROLLER_RESULT_JSON_CANDIDATE_INVALID_PROTOCOL,
    ICE_CONTROLLER_RESULT_JSON_CANDIDATE_INVALID_PORT,
    ICE_CONTROLLER_RESULT_JSON_CANDIDATE_INVALID_TYPE_ID,
    ICE_CONTROLLER_RESULT_JSON_CANDIDATE_INVALID_TYPE,
    ICE_CONTROLLER_RESULT_JSON_CANDIDATE_LACK_OF_ELEMENT,
} IceControllerResult_t;

typedef enum IceControllerEvent
{
    ICE_CONTROLLER_EVENT_NONE = 0,
    ICE_CONTROLLER_EVENT_DTLS_HANDSHAKE_DONE,
} IceControllerEvent_t;

/* https://developer.mozilla.org/en-US/docs/Web/API/RTCIceCandidate/candidate
 * https://tools.ietf.org/html/rfc5245#section-15.1 */
typedef enum IceControllerCandidateDeserializerState
{
    ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_FOUNDATION = 0,
    ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_COMPONENT,
    ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_PROTOCOL,
    ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_PRIORITY,
    ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_IP,
    ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_PORT,
    ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_TYPE_ID,
    ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_TYPE_VAL,
    ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_MAX,
} IceControllerCandidateDeserializerState_t;

typedef enum IceControllerIceServerType
{
    ICE_CONTROLLER_ICE_SERVER_TYPE_NONE = 0,
    ICE_CONTROLLER_ICE_SERVER_TYPE_STUN,  /* STUN server (used for NAT traversal) */
    ICE_CONTROLLER_ICE_SERVER_TYPE_TURN,  /* TURN server */
    ICE_CONTROLLER_ICE_SERVER_TYPE_TURNS, /* Secure TURN server (e.g., over TLS or DTLS) */
} IceControllerIceServerType_t;

typedef enum IceControllerSocketContextState
{
    ICE_CONTROLLER_SOCKET_CONTEXT_STATE_NONE = 0,
    ICE_CONTROLLER_SOCKET_CONTEXT_STATE_CONNECTION_IN_PROGRESS,
    ICE_CONTROLLER_SOCKET_CONTEXT_STATE_CREATE,
    ICE_CONTROLLER_SOCKET_CONTEXT_STATE_READY,
    ICE_CONTROLLER_SOCKET_CONTEXT_STATE_SELECTED,
} IceControllerSocketContextState_t;

typedef struct IceControllerMetrics
{
    uint32_t pendingSrflxCandidateNum;
    uint32_t pendingRelayCandidateNum;
    uint32_t isFirstConnectivityRequest;

    uint64_t printCandidatePairsStatusMs;
} IceControllerMetrics_t;

typedef struct IceControllerCandidate
{
    IceSocketProtocol_t protocol;
    uint32_t priority;
    IceEndpoint_t iceEndpoint;
    IceCandidateType_t candidateType;
} IceControllerCandidate_t;

typedef struct IceControllerIceServer
{
    IceControllerIceServerType_t serverType; /* STUN or TURN */
    char url[ ICE_CONTROLLER_ICE_SERVER_URL_MAX_LENGTH ];
    size_t urlLength;
    IceEndpoint_t iceEndpoint; //IP address
    char userName[ ICE_CONTROLLER_ICE_SERVER_USERNAME_MAX_LENGTH ]; //user name
    size_t userNameLength;
    char password[ ICE_CONTROLLER_ICE_SERVER_PASSWORD_MAX_LENGTH ]; //password
    size_t passwordLength;
    IceSocketProtocol_t protocol; //tcp or udp
} IceControllerIceServer_t;

typedef struct IceControllerSocketContext
{
    IceControllerSocketContextState_t state;
    IceControllerSocketType_t socketType;
    TlsSession_t tlsSession;

    IceCandidate_t * pLocalCandidate;
    IceCandidate_t * pRemoteCandidate;
    IceControllerIceServer_t * pIceServer;
    IceCandidatePair_t * pCandidatePair;
    int socketFd;
} IceControllerSocketContext_t;

typedef struct IceControllerIceServerConfig
{
    IceControllerIceServer_t * pIceServers;
    size_t iceServersCount;
    char * pRootCaPath;
    size_t rootCaPathLength;
    char * pRootCaPem;
    size_t rootCaPemLength;
} IceControllerIceServerConfig_t;

typedef struct IceControllerStunMsgHeader
{
    uint16_t msgType; //StunMessageType_t
    uint8_t contentLength[2];
    uint8_t magicCookie[ STUN_HEADER_MAGIC_COOKIE_OFFSET ];
    uint8_t transactionId[ STUN_HEADER_TRANSACTION_ID_LENGTH ];
    uint8_t pStunAttributes[0];
} IceControllerStunMsgHeader_t;

typedef struct IceControllerSocketListenerContext
{
    volatile uint8_t executeSocketListener;
    OnRecvNonStunPacketCallback_t onRecvNonStunPacketFunc;
    void * pOnRecvNonStunPacketCallbackContext;
} IceControllerSocketListenerContext_t;

typedef enum IceControllerState
{
    ICE_CONTROLLER_STATE_NONE = 0,
    ICE_CONTROLLER_STATE_NEW,
    ICE_CONTROLLER_STATE_PROCESS_CANDIDATES_AND_PAIRS,
    ICE_CONTROLLER_STATE_NOMINATING,
    ICE_CONTROLLER_STATE_READY,
    ICE_CONTROLLER_STATE_CLOSING,
    ICE_CONTROLLER_STATE_CLOSED,
} IceControllerState_t;

typedef enum IceControllerNatTraversalConfig
{
    ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_SEND_HOST = ( 1 << 0 ),
    ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_ACCEPT_HOST = ( 1 << 1 ),
    ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_SEND_SRFLX = ( 1 << 2 ),
    ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_ACCEPT_SRFLX = ( 1 << 3 ),
    ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_SEND_RELAY = ( 1 << 4 ),
    ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_ACCEPT_RELAY = ( 1 << 5 ),
    ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_ALLOW_ALL = 0xFF,
} IceControllerNatTraversalConfig_t;

typedef struct IceControllerInitConfig
{
    IceControllerNatTraversalConfig_t natTraversalConfigBitmap;

    /* Callback functions. */
    OnIceEventCallback_t onIceEventCallbackFunc;
    void * pOnIceEventCallbackContext;
    OnRecvNonStunPacketCallback_t onRecvNonStunPacketFunc;
    void * pOnRecvNonStunPacketCallbackContext;
} IceControllerInitConfig_t;

typedef struct IceControllerStartConfig
{
    const char * pLocalUserName;
    size_t localUserNameLength;
    const char * pLocalPassword;
    size_t localPasswordLength;
    const char * pRemoteUserName;
    size_t remoteUserNameLength;
    const char * pRemotePassword;
    size_t remotePasswordLength;
    const char * pCombinedName;
    size_t combinedNameLength;

    uint8_t isControlling;
} IceControllerStartConfig_t;

typedef struct IceControllerContext
{
    IceControllerState_t state;
    IceContext_t iceContext;

    IceControllerNatTraversalConfig_t natTraversalConfigBitmap;
    IceControllerIceServer_t iceServers[ ICE_CONTROLLER_MAX_ICE_SERVER_COUNT ]; /* Reserve 1 space for default STUN server. */
    size_t iceServersCount;
    char rootCaPath[ ICE_CONTROLLER_MAX_PATH_LENGTH + 1 ];
    size_t rootCaPathLength;
    char rootCaPem[ ICE_CONTROLLER_MAX_PEM_LENGTH + 1 ];
    size_t rootCaPemLength;

    IceControllerMetrics_t metrics;

    TimerHandler_t timerHandler;
    uint32_t timerIntervalMs;

    IceControllerSocketListenerContext_t socketListenerContext;

    /* Original remote info. */
    IceControllerSocketContext_t socketsContexts[ ICE_CONTROLLER_MAX_LOCAL_CANDIDATE_COUNT ];
    size_t socketsContextsCount;
    IceControllerSocketContext_t * pNominatedSocketContext;

    /* For ICE component. */
    IceEndpoint_t localEndpoints[ ICE_CONTROLLER_MAX_LOCAL_CANDIDATE_COUNT ];
    size_t localIceEndpointsCount;
    size_t candidateFoundationCounter;
    IceCandidate_t localCandidatesBuffer[ ICE_CONTROLLER_MAX_LOCAL_CANDIDATE_COUNT ];
    IceCandidate_t remoteCandidatesBuffer[ ICE_CONTROLLER_MAX_REMOTE_CANDIDATE_COUNT ];
    IceCandidatePair_t candidatePairsBuffer[ ICE_CONTROLLER_MAX_CANDIDATE_PAIR_COUNT ];
    IceTurnServer_t turnServersBuffer[ ICE_CONTROLLER_MAX_ICE_SERVER_COUNT ];
    TransactionIdStore_t transactionIdStore;
    TransactionIdSlot_t transactionIdsBuffer[ ICE_CONTROLLER_MAX_CANDIDATE_PAIR_COUNT ];

    OnIceEventCallback_t onIceEventCallbackFunc;
    void * pOnIceEventCustomContext;

    /* Mutex to protect global variables shared between Ice controller and socket listener. */
    SemaphoreHandle_t socketMutex;
    /* Mutex to ice context while invoking APIs of ICE library. */
    SemaphoreHandle_t iceMutex;

    uint64_t connectivityCheckTimeoutMs;
    uint8_t addLocalCandidates;
} IceControllerContext_t;

#ifdef __cplusplus
}
#endif

#endif /* ICE_CONTROLLER_DATA_TYPES_H */
