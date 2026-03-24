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

#ifndef NETWORKING_H
#define NETWORKING_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>

#include "core_http_helper.h"
#include "wslay_helper.h"

#define SIGNALING_CONTROLLER_WEBSOCKET_NUM_RETRIES                  ( 5U )

typedef enum WebsocketResult
{
    WEBSOCKET_RESULT_OK = 0,
    WEBSOCKET_RESULT_FAIL,
    WEBSOCKET_RESULT_BAD_PARAMETER,
} WebsocketResult_t;

typedef WebsocketResult_t (* WebsocketMessageCallback_t)( char * pMessage,
                                                          size_t messageLength,
                                                          void * pUserContext );

typedef struct SSLCredentials
{
    const uint8_t * pCaCertPem;
    const uint8_t * pDeviceCertPem;
    const uint8_t * pDeviceKeyPem;
} SSLCredentials_t;

typedef struct AwsCredentials
{
    /* user-agent */
    const char * pUserAgent;
    size_t userAgentLength;

    /* Region */
    const char * pRegion;
    size_t regionLength;

    /* AKSK */
    char * pAccessKeyId;
    size_t accessKeyIdLength;
    char * pSecretAccessKey;
    size_t secretAccessKeyLength;

    /* CA Cert Path */
    char * pCaCertPath;
    size_t caCertPathLength;

    /* Or CA PEM */
    const uint8_t * pRootCa;
    size_t rootCaSize;

    /* IoT thing credentials for role alias. */
    const uint8_t * pIotThingCert;
    size_t iotThingCertSize;
    const uint8_t * pIotThingPrivateKey;
    size_t iotThingPrivateKeySize;
    const char * pIotThingName;
    size_t iotThingNameLength;
    char * pSessionToken;
    size_t sessionTokenLength;

    uint64_t expirationSeconds;
} AwsCredentials_t;

typedef struct AwsConfig
{
    const char * pRegion;
    size_t regionLen;

    const char * pService;
    size_t serviceLen;
} AwsConfig_t;

typedef struct NetworkingCorehttpContext
{
    /* The transport layer interface used by the HTTP Client library. */
    TransportInterface_t xTransportInterface;
    /* The network context for the transport layer interface. */
    TlsNetworkContext_t xTlsNetworkContext;
    TlsTransportParams_t xTlsTransportParams;

    char hostName[ NETWORKING_COREHTTP_HOST_NAME_MAX_LENGTH ];

    uint8_t requestBuffer[ NETWORKING_COREHTTP_BUFFER_LENGTH ];
    char sigv4AuthBuffer[ NETWORKING_COREHTTP_SIGV4_METADATA_BUFFER_LENGTH ];
    size_t sigv4AuthBufferLength;
} NetworkingCorehttpContext_t;

typedef struct NetworkingWslayContext
{
    /* The transport layer interface used by the HTTP Client library. */
    TransportInterface_t xTransportInterface;
    /* The network context for the transport layer interface. */
    TlsNetworkContext_t xTlsNetworkContext;
    TlsTransportParams_t xTlsTransportParams;

    /* Rx path: callback user to handle received message. */
    WebsocketMessageCallback_t websocketRxCallback;
    void * pWebsocketRxCallbackContext;
    char websocketTxBuffer[ NETWORKING_WEBSOCKET_BUFFER_LENGTH ];
    size_t websocketTxBufferLength;
    char websocketRxBuffer[ NETWORKING_WEBSOCKET_BUFFER_LENGTH ];
    size_t websocketRxBufferLength;
    char metaBuffer[ NETWORKING_META_BUFFER_LENGTH ];
    size_t metaBufferLength;
    char sigv4AuthBuffer[ NETWORKING_META_BUFFER_LENGTH ];
    size_t sigv4AuthBufferLength;

    wslay_event_context_ptr wslayContext;
    uint8_t connectionEstablished;
    uint8_t connectionCloseRequested;

    TickType_t lastPingTick;
    int socketWakeUp;
    struct sockaddr_in socketWakeUpAddr;
} NetworkingWslayContext_t;

typedef struct WebsocketServerInfo
{
    char * pUrl;
    size_t urlLength;
    uint16_t port;
} WebsocketServerInfo_t;

typedef enum HttpResult
{
    HTTP_RESULT_OK = 0,
    HTTP_RESULT_FAIL,
    HTTP_RESULT_BAD_PARAMETER,
} HttpResult_t;

typedef enum HttpVerb
{
    HTTP_GET,
    HTTP_POST,
} HttpVerb_t;

typedef struct HttpRequest
{
    HttpVerb_t verb;
    char * pUrl;
    size_t urlLength;
    char * pBody;
    size_t bodyLength;

    uint8_t isFetchingCredential;
} HttpRequest_t;

typedef struct HttpResponse
{
    char * pBuffer;
    size_t bufferLength;
} HttpResponse_t;


HttpResult_t Http_Init( NetworkingCorehttpContext_t * pHttpCtx );
HttpResult_t Http_Send( NetworkingCorehttpContext_t * pHttpCtx,
                        HttpRequest_t * pRequest,
                        const AwsCredentials_t * pAwsCredentials,
                        size_t timeoutMs,
                        HttpResponse_t * pResponse );
WebsocketResult_t Websocket_Init( NetworkingWslayContext_t * pWebsocketCtx,
                                  WebsocketMessageCallback_t rxCallback,
                                  void * pRxCallbackContext );
WebsocketResult_t Websocket_Connect( NetworkingWslayContext_t * pWebsocketCtx,
                                     const AwsCredentials_t * pAwsCredentials,
                                     WebsocketServerInfo_t * pServerInfo );
WebsocketResult_t Websocket_Disconnect( NetworkingWslayContext_t * pWebsocketCtx );
WebsocketResult_t Websocket_Send( NetworkingWslayContext_t * pWebsocketCtx,
                                  char * pMessage,
                                  size_t messageLength );
WebsocketResult_t Websocket_Recv( NetworkingWslayContext_t * pWebsocketCtx );
WebsocketResult_t Websocket_Signal( NetworkingWslayContext_t * pWebsocketCtx );

#ifdef __cplusplus
}
#endif

#endif /* NETWORKING_H */
