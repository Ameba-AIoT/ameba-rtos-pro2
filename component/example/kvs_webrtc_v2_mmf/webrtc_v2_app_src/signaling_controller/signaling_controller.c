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

#include <string.h>
#include "logging.h"
#include "signaling_controller.h"
#include "signaling_api.h"
#include "networking.h"
#include "base64.h"
#if METRIC_PRINT_ENABLED
#include "metric.h"
#endif
#include "core_json.h"

#include "core_http_helper.h"
#include "wslay_helper.h"

#ifndef MIN
#define MIN( a,b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

#define SIGNALING_CONTROLLER_MESSAGE_QUEUE_NAME "/WebrtcApplicationSignalingController"

#define MAX_QUEUE_MSG_NUM ( 10 )
#define WEBSOCKET_ENDPOINT_PORT ( 443U )

#define SIGNALING_CONTROLLER_SDP_EVENT_MESSAGE_TYPE_KEY "type"
#define SIGNALING_CONTROLLER_SDP_EVENT_MESSAGE_TYPE_VALUE_OFFER "offer"
#define SIGNALING_CONTROLLER_SDP_EVENT_MESSAGE_TYPE_VALUE_ANSWER "answer"
#define SIGNALING_CONTROLLER_SDP_EVENT_MESSAGE_CONTENT_KEY "sdp"
#define SIGNALING_CONTROLLER_SDP_EVENT_MESSAGE_NEWLINE_ENDING "\\n"

#define SIGNALING_CONTROLLER_REFRESH_ICE_SERVER_CONFIGS_TIMEOUT ( 15 )
#define SIGNALING_CONTROLLER_RETRY_CONNECT_TIMEOUT_MS ( 2000 )

static uint8_t AreCredentialsExpired( SignalingControllerContext_t * pCtx );

static SignalingControllerResult_t UpdateIceServerConfigs( SignalingControllerContext_t * pCtx,
                                                           SignalingIceServer_t * pIceServerList,
                                                           size_t iceServerListNum );

static SignalingControllerResult_t ConnectToSignalingService( SignalingControllerContext_t * pCtx );

static SignalingControllerResult_t JoinStorageSession( SignalingControllerContext_t *pCtx );

static WebsocketResult_t HandleWssMessage( char * pMessage,
                                           size_t messageLength,
                                           void * pUserContext )
{
    WebsocketResult_t ret = WEBSOCKET_RESULT_OK;
    SignalingResult_t retSignal;
    WssRecvMessage_t wssRecvMessage;
    SignalingControllerContext_t * pCtx = ( SignalingControllerContext_t * ) pUserContext;
    SignalingMessage_t signalingMessage;
    Base64Result_t retBase64;
    bool needCallback = true;
    MessageQueueResult_t retMessageQueue;
    SignalingControllerEventMessage_t eventMessage = {
        .event = SIGNALING_CONTROLLER_EVENT_NONE,
        .onCompleteCallback = NULL,
        .pOnCompleteCallbackContext = NULL,
    };

    if( ( pMessage == NULL ) || ( messageLength == 0 ) )
    {
        LogDebug( ( "Received empty signaling message" ) );
        ret = WEBSOCKET_RESULT_BAD_PARAMETER;
    }

    if( ret == WEBSOCKET_RESULT_OK )
    {
        // Parse the response
        retSignal = Signaling_ParseWssRecvMessage( pMessage, ( size_t ) messageLength, &wssRecvMessage );
        if( retSignal != SIGNALING_RESULT_OK )
        {
            ret = NETWORKING_WSLAY_RESULT_UNKNOWN_MESSAGE;
        }
    }

    /* Decode base64 payload. */
    if( ret == WEBSOCKET_RESULT_OK )
    {
        pCtx->signalingRxMessageLength = SIGNALING_CONTROLLER_MESSAGE_BUFFER_LENGTH;
        retBase64 = Base64_Decode( wssRecvMessage.pBase64EncodedPayload, wssRecvMessage.base64EncodedPayloadLength, pCtx->signalingRxMessageBuffer, &pCtx->signalingRxMessageLength );

        if( retBase64 != BASE64_RESULT_OK )
        {
            ret = NETWORKING_WSLAY_RESULT_FAIL_BASE64_DECODE;
        }
    }

    if( ret == WEBSOCKET_RESULT_OK )
    {
        switch( wssRecvMessage.messageType )
        {
            case SIGNALING_TYPE_MESSAGE_GO_AWAY:
                eventMessage.event = SIGNALING_CONTROLLER_EVENT_RECONNECT_WSS;
                retMessageQueue = MessageQueue_Send( &pCtx->sendMessageQueue, &eventMessage, sizeof( SignalingControllerEventMessage_t ) );

                needCallback = false;

                if( retMessageQueue != MESSAGE_QUEUE_RESULT_OK )
                {
                    ret = WEBSOCKET_RESULT_FAIL;
                }

                if( ret == WEBSOCKET_RESULT_OK )
                {
                    /* Wake the running thread up to handle event. */
                    ( void ) Websocket_Signal( &( pCtx->websocketContext ) );
                }
                break;

            case SIGNALING_TYPE_MESSAGE_STATUS_RESPONSE:

                if( strcmp( wssRecvMessage.statusResponse.pStatusCode,"200" ) != 0 )
                {
                    LogWarn( ( "Failed to deliver message. Correlation ID: %.*s, Error Type: %.*s, Error Code: %.*s, Description: %.*s!",
                               ( int ) wssRecvMessage.statusResponse.correlationIdLength,
                               wssRecvMessage.statusResponse.pCorrelationId,
                               ( int ) wssRecvMessage.statusResponse.errorTypeLength,
                               wssRecvMessage.statusResponse.pErrorType,
                               ( int ) wssRecvMessage.statusResponse.statusCodeLength,
                               wssRecvMessage.statusResponse.pStatusCode,
                               ( int ) wssRecvMessage.statusResponse.descriptionLength,
                               wssRecvMessage.statusResponse.pDescription ) );
                }
                break;

            case SIGNALING_TYPE_MESSAGE_RECONNECT_ICE_SERVER:
                eventMessage.event = SIGNALING_CONTROLLER_EVENT_FORCE_REFRESH_ICE_SERVER_CONFIGS;
                retMessageQueue = MessageQueue_Send( &pCtx->sendMessageQueue, &eventMessage, sizeof( SignalingControllerEventMessage_t ) );

                needCallback = false;

                if( retMessageQueue != MESSAGE_QUEUE_RESULT_OK )
                {
                    ret = WEBSOCKET_RESULT_FAIL;
                }

                if( ret == WEBSOCKET_RESULT_OK )
                {
                    /* Wake the running thread up to handle event. */
                    ( void ) Websocket_Signal( &( pCtx->websocketContext ) );
                }
                break;
            default:
                break;
        }
    }

    if( ret == WEBSOCKET_RESULT_OK )
    {
        memset( &signalingMessage, 0, sizeof( SignalingMessage_t ) );
        signalingMessage.pRemoteClientId = wssRecvMessage.pSenderClientId;
        signalingMessage.remoteClientIdLength = wssRecvMessage.senderClientIdLength;
        signalingMessage.pCorrelationId = wssRecvMessage.statusResponse.pCorrelationId;
        signalingMessage.correlationIdLength = wssRecvMessage.statusResponse.correlationIdLength;
        signalingMessage.messageType = wssRecvMessage.messageType;
        signalingMessage.pMessage = pCtx->signalingRxMessageBuffer;
        signalingMessage.messageLength = pCtx->signalingRxMessageLength;

        if( ( needCallback == true ) && ( pCtx->messageReceivedCallback != NULL ) )
        {
            pCtx->messageReceivedCallback( &signalingMessage, pCtx->pMessageReceivedCallbackData );
        }
    }

    return ret;
}

static SignalingControllerResult_t SignalingController_HttpInit( SignalingControllerContext_t * pCtx )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    HttpResult_t retHttp;

    retHttp = Http_Init( &( pCtx->httpContext ) );

    if( retHttp != HTTP_RESULT_OK )
    {
        LogError( ( "Http_Init fails with return 0x%x", retHttp ) );
        ret = SIGNALING_CONTROLLER_RESULT_FAIL;
    }

    return ret;
}

static SignalingControllerResult_t HttpSend( SignalingControllerContext_t * pCtx,
                                             HttpRequest_t * pRequest,
                                             size_t timeoutMs,
                                             HttpResponse_t * pResponse )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    HttpResult_t retHttp;
    int i;
    AwsCredentials_t awsCreds;

    awsCreds.pRootCa = pCtx->pCaCertPem;
    awsCreds.rootCaSize = pCtx->caCertPemSize;

    awsCreds.pIotThingCert = pCtx->pIotThingCert;
    awsCreds.iotThingCertSize = pCtx->iotThingCertSize;

    awsCreds.pIotThingPrivateKey = pCtx->pIotThingPrivateKey;
    awsCreds.iotThingPrivateKeySize = pCtx->iotThingPrivateKeySize;

    awsCreds.pUserAgent = pCtx->pUserAgentName;
    awsCreds.userAgentLength = pCtx->userAgentNameLength;

    awsCreds.pIotThingName = pCtx->pIotThingName;
    awsCreds.iotThingNameLength = pCtx->iotThingNameLength;

    awsCreds.pSessionToken = &( pCtx->sessionToken[ 0 ] );
    awsCreds.sessionTokenLength = pCtx->sessionTokenLength;

    awsCreds.pRegion = pCtx->awsConfig.pRegion;
    awsCreds.regionLength = pCtx->awsConfig.regionLen;

    /* sigv4Credential */
    awsCreds.pAccessKeyId = &( pCtx->accessKeyId[ 0 ] );
    awsCreds.accessKeyIdLength = pCtx->accessKeyIdLength;

    awsCreds.pSecretAccessKey = &( pCtx->secretAccessKey[ 0 ] );
    awsCreds.secretAccessKeyLength = pCtx->secretAccessKeyLength;

    awsCreds.expirationSeconds = pCtx->expirationSeconds;

    for( i = 0; i < SIGNALING_CONTROLLER_HTTP_NUM_RETRIES; i++ )
    {
        retHttp = Http_Send( &( pCtx->httpContext ), pRequest, &( awsCreds ), timeoutMs, pResponse );

        if( retHttp == HTTP_RESULT_OK )
        {
            break;
        }
    }

    if( retHttp != HTTP_RESULT_OK )
    {
        LogError( ( "Http_Send fails with return 0x%x", retHttp ) );
        ret = SIGNALING_CONTROLLER_RESULT_FAIL;
    }

    return ret;
}

static void OnConnectionStateChange( SignalingControllerContext_t * pCtx,
                                     SignalingControllerConnectionState_t newState )
{
    if( ( pCtx->connectionStateCallback != NULL ) && ( pCtx->connectionState != newState ) )
    {
        pCtx->connectionState = newState;
        pCtx->connectionStateCallback( newState, pCtx->pConnectionStateCallbacCustomContext );
    }
}

static SignalingControllerResult_t SignalingController_WebsocketInit( SignalingControllerContext_t * pCtx )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    WebsocketResult_t retWebsocket;

    retWebsocket = Websocket_Init( &( pCtx->websocketContext ), HandleWssMessage, pCtx );

    if( retWebsocket != WEBSOCKET_RESULT_OK )
    {
        LogError( ( "Fail to initialize websocket library, return=0x%x", retWebsocket ) );
        ret = SIGNALING_CONTROLLER_RESULT_FAIL;
    }

    return ret;
}

static SignalingControllerResult_t SignalingController_WebsocketConnect( SignalingControllerContext_t * pCtx,
                                                                         WebsocketServerInfo_t * pServerInfo )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    WebsocketResult_t retWebsocket;
    int i;
    AwsCredentials_t awsCreds;

    awsCreds.pRootCa = pCtx->pCaCertPem;
    awsCreds.rootCaSize = pCtx->caCertPemSize;

    awsCreds.pIotThingCert = pCtx->pIotThingCert;
    awsCreds.iotThingCertSize = pCtx->iotThingCertSize;

    awsCreds.pIotThingPrivateKey = pCtx->pIotThingPrivateKey;
    awsCreds.iotThingPrivateKeySize = pCtx->iotThingPrivateKeySize;

    awsCreds.pUserAgent = pCtx->pUserAgentName;
    awsCreds.userAgentLength = pCtx->userAgentNameLength;

    awsCreds.pIotThingName = pCtx->pIotThingName;
    awsCreds.iotThingNameLength = pCtx->iotThingNameLength;

    awsCreds.pSessionToken = &( pCtx->sessionToken[ 0 ] );
    awsCreds.sessionTokenLength = pCtx->sessionTokenLength;

    awsCreds.pRegion = pCtx->awsConfig.pRegion;
    awsCreds.regionLength = pCtx->awsConfig.regionLen;

    /* sigv4Credential */
    awsCreds.pAccessKeyId = &( pCtx->accessKeyId[ 0 ] );
    awsCreds.accessKeyIdLength = pCtx->accessKeyIdLength;

    awsCreds.pSecretAccessKey = &( pCtx->secretAccessKey[ 0 ] );
    awsCreds.secretAccessKeyLength = pCtx->secretAccessKeyLength;

    awsCreds.expirationSeconds = pCtx->expirationSeconds;

    for( i = 0; i < SIGNALING_CONTROLLER_WEBSOCKET_NUM_RETRIES; i++ )
    {
        retWebsocket = Websocket_Connect( &( pCtx->websocketContext ), &awsCreds, pServerInfo );

        if( retWebsocket == WEBSOCKET_RESULT_OK )
        {
            break;
        }
    }

    if( retWebsocket != WEBSOCKET_RESULT_OK )
    {
        LogError( ( "Fail to connect url: %.*s:%u, return=0x%x",
                    ( int ) pServerInfo->urlLength, pServerInfo->pUrl, pServerInfo->port, retWebsocket ) );
        ret = SIGNALING_CONTROLLER_RESULT_FAIL;
    }

    return ret;
}

static uint8_t AreCredentialsExpired( SignalingControllerContext_t * pCtx )
{
    uint8_t credentialsExpired = 0U;
    uint64_t currentTimeSeconds = NetworkingUtils_GetCurrentTimeSec( NULL );

    if( ( pCtx->iotThingNameLength > 0 ) &&
        ( pCtx->roleAliasLength > 0 ) &&
        ( pCtx->iotCredentialsEndpointLength ) )
    {
        if( ( pCtx->expirationSeconds == 0 ) ||
            ( currentTimeSeconds >= pCtx->expirationSeconds - SIGNALING_CONTROLLER_FETCH_CREDS_GRACE_PERIOD_SEC ) )
        {
            credentialsExpired = 1U;
        }
    }

    return credentialsExpired;
}

static void LogSignalingInfo( SignalingControllerContext_t * pCtx )
{
    size_t i, j;

    LogInfo( ( "======================================== Channel Info ========================================" ) );
    LogInfo( ( "Signaling Channel ARN: %s", &( pCtx->signalingChannelArn[ 0 ] ) ) );

    LogInfo( ( "======================================== Endpoints Info ========================================" ) );
    LogInfo( ( "HTTPS Endpoint: %s", &( pCtx->httpsEndpoint[ 0 ] ) ) );
    LogInfo( ( "WSS Endpoint: %s", &( pCtx->wssEndpoint[ 0 ] ) ) );
    LogInfo( ( "WebRTC Endpoint: %s", pCtx->webrtcEndpointLength == 0 ? "N/A" : &( pCtx->webrtcEndpoint[ 0 ] ) ) );

    /* Ice server list */
    LogInfo( ( "======================================== Ice Server List ========================================" ) );
    LogInfo( ( "Ice Server Count: %u", pCtx->iceServerConfigsCount ) );
    for( i = 0; i < pCtx->iceServerConfigsCount; i++ )
    {
        LogInfo( ( "======================================== Ice Server[%u] ========================================", i ) );
        LogInfo( ( "    TTL (seconds): %lu", pCtx->iceServerConfigs[ i ].ttlSeconds ) );
        LogInfo( ( "    User Name: %s", pCtx->iceServerConfigs[ i ].userName ) );
        LogInfo( ( "    Password: %s", pCtx->iceServerConfigs[ i ].password ) );
        LogInfo( ( "    URI Count: %u", pCtx->iceServerConfigs[ i ].iceServerUriCount ) );

        for( j = 0; j < pCtx->iceServerConfigs[ i ].iceServerUriCount; j++ )
        {
            LogInfo( ( "        URI: %s", &( pCtx->iceServerConfigs[ i ].iceServerUris[ j ].uri[ 0 ] ) ) );
        }
    }
}

static SignalingControllerResult_t UpdateIceServerConfigs( SignalingControllerContext_t * pCtx,
                                                           SignalingIceServer_t * pIceServerList,
                                                           size_t iceServerListNum )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    uint64_t iceServerConfigTimeSec;
    uint64_t minTTL = UINT64_MAX;
    uint8_t i;
    uint8_t j;

    if( ( pCtx == NULL ) || ( pIceServerList == NULL ) )
    {
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        for( i = 0; i < iceServerListNum; i++ )
        {
            if( i >= SIGNALING_CONTROLLER_ICE_SERVER_MAX_CONFIG_COUNT )
            {
                break;
            }
            else if( pIceServerList[i].userNameLength >= SIGNALING_CONTROLLER_ICE_SERVER_USER_NAME_BUFFER_LENGTH )
            {
                LogError( ( "The length of user name of ice server is too long to store, length=%d", pIceServerList[i].userNameLength ) );
                ret = SIGNALING_CONTROLLER_RESULT_FAIL;
                break;
            }
            else if( pIceServerList[i].passwordLength >= SIGNALING_CONTROLLER_ICE_SERVER_PASSWORD_BUFFER_LENGTH )
            {
                LogError( ( "The length of password of ice server is too long to store, length=%d", pIceServerList[i].passwordLength ) );
                ret = SIGNALING_CONTROLLER_RESULT_FAIL;
                break;
            }
            else
            {
                /* Do nothing, coverity happy. */
            }

            memcpy( pCtx->iceServerConfigs[i].userName, pIceServerList[i].pUserName, pIceServerList[i].userNameLength );
            pCtx->iceServerConfigs[i].userNameLength = pIceServerList[i].userNameLength;
            memcpy( pCtx->iceServerConfigs[i].password, pIceServerList[i].pPassword, pIceServerList[i].passwordLength );
            pCtx->iceServerConfigs[i].passwordLength = pIceServerList[i].passwordLength;
            pCtx->iceServerConfigs[i].ttlSeconds = pIceServerList[i].messageTtlSeconds;

            minTTL = MIN( minTTL, pCtx->iceServerConfigs[i].ttlSeconds );

            for( j = 0; j < pIceServerList[i].urisNum; j++ )
            {
                if( j >= SIGNALING_CONTROLLER_ICE_SERVER_MAX_URIS_COUNT )
                {
                    break;
                }
                else if( pIceServerList[i].urisLength[j] >= SIGNALING_CONTROLLER_ICE_SERVER_URI_BUFFER_LENGTH )
                {
                    LogError( ( "The length of URI of ice server is too long to store, length=%d", pIceServerList[i].urisLength[j] ) );
                    ret = SIGNALING_CONTROLLER_RESULT_FAIL;
                    break;
                }
                else
                {
                    /* Do nothing, coverity happy. */
                }

                memcpy( &pCtx->iceServerConfigs[i].iceServerUris[j].uri, pIceServerList[i].pUris[j], pIceServerList[i].urisLength[j] );
                pCtx->iceServerConfigs[i].iceServerUris[j].uriLength = pIceServerList[i].urisLength[j];
            }

            if( ret == SIGNALING_CONTROLLER_RESULT_OK )
            {
                pCtx->iceServerConfigs[i].iceServerUriCount = j;
            }
            else
            {
                break;
            }
        }
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        /* Update context with latest ICE server configuration, including server count and expiration. */
        pCtx->iceServerConfigsCount = i;

        iceServerConfigTimeSec = NetworkingUtils_GetCurrentTimeSec( NULL );

        if( minTTL < ICE_CONFIGURATION_REFRESH_GRACE_PERIOD_SEC )
        {
            LogWarn( ( "Minimum TTL is less than Refresh Grace Period." ) );
        }

        pCtx->iceServerConfigExpirationSec = iceServerConfigTimeSec + ( minTTL - ICE_CONFIGURATION_REFRESH_GRACE_PERIOD_SEC );
    }

    return ret;
}

static SignalingControllerResult_t FetchTemporaryCredentials( SignalingControllerContext_t * pCtx )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    SignalingResult_t retSignal;
    SignalingRequest_t signalRequest;
    HttpRequest_t request;
    HttpResponse_t response;
    SignalingCredential_t retCredentials;

    LogInfo( ( "Fetching Temporary Credentials." ) );

    if( pCtx == NULL )
    {
        LogError( ( "Invalid input, pCtx: 0x%p", pCtx ) );
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        // Prepare URL buffer
        signalRequest.pUrl = pCtx->httpUrlBuffer;
        signalRequest.urlLength = SIGNALING_CONTROLLER_HTTP_URL_BUFFER_LENGTH;

        retSignal = Signaling_ConstructFetchTempCredsRequestForAwsIot( pCtx->pIotCredentialsEndpoint,
                                                                       pCtx->iotCredentialsEndpointLength,
                                                                       pCtx->pRoleAlias,
                                                                       pCtx->roleAliasLength,
                                                                       &signalRequest );

        if( retSignal != SIGNALING_RESULT_OK )
        {
            LogError( ( "Fail to construct Fetch Temporary Credential request, return=0x%x", retSignal ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        memset( &request, 0, sizeof( HttpRequest_t ) );
        request.pUrl = signalRequest.pUrl;
        request.urlLength = signalRequest.urlLength;
        request.isFetchingCredential = 1U;

        memset( &response, 0, sizeof( HttpResponse_t ) );
        response.pBuffer = pCtx->httpResponserBuffer;
        response.bufferLength = SIGNALING_CONTROLLER_HTTP_RESPONSE_BUFFER_LENGTH;

        ret = HttpSend( pCtx, &request, 0, &response );
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {

        retSignal = Signaling_ParseFetchTempCredsResponseFromAwsIot( response.pBuffer,
                                                                     response.bufferLength,
                                                                     &retCredentials );

        if( retSignal != SIGNALING_RESULT_OK )
        {
            LogError( ( "Fail to parse fetching credentials response, return=0x%x, response(%zu): %.*s", retSignal, response.bufferLength,
                        ( int ) response.bufferLength, response.pBuffer ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
        else
        {
            LogDebug( ( "Access Key ID : %.*s \n \n Secret Access Key ID : %.*s \n \n Session Token : %.*s \n \n Expiration : %.*s",
                        ( int ) retCredentials.accessKeyIdLength, retCredentials.pAccessKeyId,
                        ( int ) retCredentials.secretAccessKeyLength, retCredentials.pSecretAccessKey,
                        ( int ) retCredentials.sessionTokenLength, retCredentials.pSessionToken,
                        ( int ) retCredentials.expirationLength, ( char * ) retCredentials.pExpiration ) );
        }
    }

    // Parse the response
    if( ( ret == SIGNALING_CONTROLLER_RESULT_OK ) && ( retCredentials.pAccessKeyId != NULL ) )
    {
        if( retCredentials.accessKeyIdLength > ACCESS_KEY_MAX_LEN )
        {
            /* Return Access Key is longer than expectation. Drop it. */
            LogError( ( "Length of Access Key ID(%zu) is out of maximum value.",
                        retCredentials.accessKeyIdLength ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
        else
        {
            memcpy( pCtx->accessKeyId, retCredentials.pAccessKeyId, retCredentials.accessKeyIdLength );
            pCtx->accessKeyIdLength = retCredentials.accessKeyIdLength;
            pCtx->accessKeyId[ pCtx->accessKeyIdLength ] = '\0';
        }
    }

    if( ( ret == SIGNALING_CONTROLLER_RESULT_OK ) && ( retCredentials.pSecretAccessKey != NULL ) )
    {
        if( retCredentials.secretAccessKeyLength > SECRET_ACCESS_KEY_MAX_LEN )
        {
            /* Return Secret Access Key is longer than expectation. Drop it. */
            LogError( ( "Secret Access Key Greater than MAX Length. " ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
        else
        {
            memcpy( pCtx->secretAccessKey, retCredentials.pSecretAccessKey, retCredentials.secretAccessKeyLength );
            pCtx->secretAccessKeyLength = retCredentials.secretAccessKeyLength;
            pCtx->secretAccessKey[ pCtx->secretAccessKeyLength ] = '\0';
        }
    }

    if( ( ret == SIGNALING_CONTROLLER_RESULT_OK ) && ( retCredentials.pSessionToken != NULL ) )
    {
        if( retCredentials.sessionTokenLength > SESSION_TOKEN_MAX_LEN )
        {
            /* Return Session Token is longer than expectation. Drop it. */
            LogError( ( "Session Token Greater than MAX Length. " ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
        else
        {
            memcpy( pCtx->sessionToken, retCredentials.pSessionToken, retCredentials.sessionTokenLength );
            pCtx->sessionTokenLength = retCredentials.sessionTokenLength;
            pCtx->sessionToken[ pCtx->sessionTokenLength ] = '\0';
        }
    }

    if( ( ret == SIGNALING_CONTROLLER_RESULT_OK ) && ( retCredentials.pExpiration != NULL ) )
    {
        if( retCredentials.expirationLength > EXPIRATION_MAX_LEN )
        {
            /* Return Expiration for Access Key's is longer than expectation. Drop it. */
            LogError( ( "Expiration for Access Key's Greater than MAX Length. " ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
        else
        {
            pCtx->expirationSeconds = NetworkingUtils_GetTimeFromIso8601( retCredentials.pExpiration, retCredentials.expirationLength );
        }
    }

    return ret;
}

static SignalingControllerResult_t DescribeSignalingChannel( SignalingControllerContext_t * pCtx )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    SignalingResult_t retSignal;
    SignalingAwsRegion_t awsRegion;
    SignalingChannelName_t channelName;
    SignalingRequest_t signalRequest;
    HttpRequest_t request;
    HttpResponse_t response;
    SignalingChannelInfo_t signalingChannelInfo;

    LogInfo( ( "Describing Signaling Channel." ) );

    // Prepare AWS region
    awsRegion.pAwsRegion = pCtx->awsConfig.pRegion;
    awsRegion.awsRegionLength = pCtx->awsConfig.regionLen;
    // Prepare channel name
    channelName.pChannelName = pCtx->pChannelName;
    channelName.channelNameLength = pCtx->channelNameLength;
    // Prepare URL buffer
    signalRequest.pUrl = pCtx->httpUrlBuffer;
    signalRequest.urlLength = SIGNALING_CONTROLLER_HTTP_URL_BUFFER_LENGTH;
    // Prepare body buffer
    signalRequest.pBody = pCtx->httpBodyBuffer;
    signalRequest.bodyLength = SIGNALING_CONTROLLER_HTTP_BODY_BUFFER_LENGTH;

    retSignal = Signaling_ConstructDescribeSignalingChannelRequest( &awsRegion,
                                                                    &channelName,
                                                                    &signalRequest );

    if( retSignal != SIGNALING_RESULT_OK )
    {
        LogError( ( "Fail to construct describe signaling channel request, return=0x%x", retSignal ) );
        ret = SIGNALING_CONTROLLER_RESULT_FAIL;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        memset( &request, 0, sizeof( HttpRequest_t ) );
        request.pUrl = signalRequest.pUrl;
        request.urlLength = signalRequest.urlLength;
        request.pBody = signalRequest.pBody;
        request.bodyLength = signalRequest.bodyLength;

        memset( &response, 0, sizeof( HttpResponse_t ) );
        response.pBuffer = pCtx->httpResponserBuffer;
        response.bufferLength = SIGNALING_CONTROLLER_HTTP_RESPONSE_BUFFER_LENGTH;

        ret = HttpSend( pCtx, &request, 0, &response );
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        retSignal = Signaling_ParseDescribeSignalingChannelResponse( response.pBuffer, response.bufferLength, &signalingChannelInfo );

        if( retSignal != SIGNALING_RESULT_OK )
        {
            LogError( ( "Fail to parse describe signaling channel response, return=0x%x, response(%u): %.*s", retSignal, response.bufferLength,
                        ( int ) response.bufferLength, response.pBuffer ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        if( ( signalingChannelInfo.pChannelStatus == NULL ) || ( strncmp( signalingChannelInfo.pChannelStatus, "ACTIVE", signalingChannelInfo.channelStatusLength ) != 0 ) )
        {
            LogError( ( "No active channel status found." ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
    }

    // Parse the response
    if( ( ret == SIGNALING_CONTROLLER_RESULT_OK ) && ( signalingChannelInfo.channelArn.pChannelArn != NULL ) )
    {
        if( signalingChannelInfo.channelArn.channelArnLength > SIGNALING_CONTROLLER_ARN_BUFFER_LENGTH )
        {
            /* Return ARN is longer than expectation. Drop it. */
            LogError( ( "No active channel status found." ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
        else
        {
            strncpy( &( pCtx->signalingChannelArn[ 0 ] ),
                     signalingChannelInfo.channelArn.pChannelArn,
                     signalingChannelInfo.channelArn.channelArnLength );
            pCtx->signalingChannelArn[ signalingChannelInfo.channelArn.channelArnLength ] = '\0';
            pCtx->signalingChannelArnLength = signalingChannelInfo.channelArn.channelArnLength;
        }
    }

    if( ( ret == SIGNALING_CONTROLLER_RESULT_OK ) && ( signalingChannelInfo.channelArn.pChannelArn != NULL ) )
    {
        strncpy( &( pCtx->signalingChannelArn[ 0 ] ),
                 signalingChannelInfo.channelArn.pChannelArn,
                 signalingChannelInfo.channelArn.channelArnLength );
        pCtx->signalingChannelArn[ signalingChannelInfo.channelArn.channelArnLength ] = '\0';
        pCtx->signalingChannelArnLength = signalingChannelInfo.channelArn.channelArnLength;
    }

    return ret;
}

static SignalingControllerResult_t GetSignalingChannelEndpoints( SignalingControllerContext_t * pCtx )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    SignalingResult_t retSignal;
    SignalingAwsRegion_t awsRegion;
    GetSignalingChannelEndpointRequestInfo_t endpointRequestInfo;
    SignalingRequest_t signalRequest;
    HttpRequest_t request;
    HttpResponse_t response;
    SignalingChannelEndpoints_t signalingEndpoints;

    LogInfo( ( "Getting Signaling Channel Endpoints." ) );

    // Prepare AWS region
    awsRegion.pAwsRegion = pCtx->awsConfig.pRegion;
    awsRegion.awsRegionLength = pCtx->awsConfig.regionLen;
    // Prepare URL buffer
    signalRequest.pUrl = pCtx->httpUrlBuffer;
    signalRequest.urlLength = SIGNALING_CONTROLLER_HTTP_URL_BUFFER_LENGTH;
    // Prepare body buffer
    signalRequest.pBody = pCtx->httpBodyBuffer;
    signalRequest.bodyLength = SIGNALING_CONTROLLER_HTTP_BODY_BUFFER_LENGTH;
    // Create the API url
    endpointRequestInfo.channelArn.pChannelArn = &( pCtx->signalingChannelArn[ 0 ] );
    endpointRequestInfo.channelArn.channelArnLength = pCtx->signalingChannelArnLength;
    endpointRequestInfo.protocols = SIGNALING_PROTOCOL_WEBSOCKET_SECURE |
                                    SIGNALING_PROTOCOL_HTTPS;
    if( pCtx->enableStorageSession != 0 )
    {
        endpointRequestInfo.protocols |= SIGNALING_PROTOCOL_WEBRTC;
    }
    endpointRequestInfo.role = pCtx->role;

    retSignal = Signaling_ConstructGetSignalingChannelEndpointRequest( &awsRegion, &endpointRequestInfo, &signalRequest );

    if( retSignal != SIGNALING_RESULT_OK )
    {
        LogError( ( "Fail to construct get signaling channel endpoint request, return=0x%x", retSignal ) );
        ret = SIGNALING_CONTROLLER_RESULT_FAIL;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        memset( &request, 0, sizeof( HttpRequest_t ) );
        request.pUrl = signalRequest.pUrl;
        request.urlLength = signalRequest.urlLength;
        request.pBody = signalRequest.pBody;
        request.bodyLength = signalRequest.bodyLength;
        request.isFetchingCredential = 0U;

        memset( &response, 0, sizeof( HttpResponse_t ) );
        response.pBuffer = pCtx->httpResponserBuffer;
        response.bufferLength = SIGNALING_CONTROLLER_HTTP_RESPONSE_BUFFER_LENGTH;

        ret = HttpSend( pCtx, &request, 0, &response );
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        retSignal = Signaling_ParseGetSignalingChannelEndpointResponse( response.pBuffer, response.bufferLength, &signalingEndpoints );

        if( retSignal != SIGNALING_RESULT_OK )
        {
            LogError( ( "Fail to parse get signaling channel endpoint response, return=0x%x", retSignal ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
    }

    // Parse the response
    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        if( ( signalingEndpoints.httpsEndpoint.pEndpoint == NULL ) || ( signalingEndpoints.httpsEndpoint.endpointLength > SIGNALING_CONTROLLER_ENDPOINT_BUFFER_LENGTH ) )
        {
            LogError( ( "No valid HTTPS endpoint found in response" ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
        else
        {
            strncpy( &( pCtx->httpsEndpoint[ 0 ] ),
                     signalingEndpoints.httpsEndpoint.pEndpoint,
                     signalingEndpoints.httpsEndpoint.endpointLength );
            pCtx->httpsEndpoint[ signalingEndpoints.httpsEndpoint.endpointLength ] = '\0';
            pCtx->httpsEndpointLength = signalingEndpoints.httpsEndpoint.endpointLength;
        }
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        if( ( signalingEndpoints.wssEndpoint.pEndpoint == NULL ) || ( signalingEndpoints.wssEndpoint.endpointLength > SIGNALING_CONTROLLER_ENDPOINT_BUFFER_LENGTH ) )
        {
            LogError( ( "No valid websocket endpoint found in response" ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
        else
        {
            strncpy( &( pCtx->wssEndpoint[ 0 ] ),
                     signalingEndpoints.wssEndpoint.pEndpoint,
                     signalingEndpoints.wssEndpoint.endpointLength );
            pCtx->wssEndpoint[ signalingEndpoints.wssEndpoint.endpointLength ] = '\0';
            pCtx->wssEndpointLength = signalingEndpoints.wssEndpoint.endpointLength;
        }
    }

    if( ( ret == SIGNALING_CONTROLLER_RESULT_OK ) && ( signalingEndpoints.webrtcEndpoint.pEndpoint != NULL ) )
    {
        if( signalingEndpoints.webrtcEndpoint.endpointLength > SIGNALING_CONTROLLER_ENDPOINT_BUFFER_LENGTH )
        {
            LogError( ( "Length of webRTC endpoint name is too long to store, length=%d", signalingEndpoints.webrtcEndpoint.endpointLength ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
        else
        {
            strncpy( &( pCtx->webrtcEndpoint[ 0 ] ),
                     signalingEndpoints.webrtcEndpoint.pEndpoint,
                     signalingEndpoints.webrtcEndpoint.endpointLength );
            pCtx->webrtcEndpoint[ signalingEndpoints.webrtcEndpoint.endpointLength ] = '\0';
            pCtx->webrtcEndpointLength = signalingEndpoints.webrtcEndpoint.endpointLength;
        }
    }

    return ret;
}

static SignalingControllerResult_t GetIceServerConfigs( SignalingControllerContext_t * pCtx )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    SignalingResult_t retSignal;
    SignalingChannelEndpoint_t signalingChannelHttpEndpoint;
    GetIceServerConfigRequestInfo_t getIceServerConfigRequestInfo;
    SignalingRequest_t signalRequest;
    HttpRequest_t request;
    HttpResponse_t response;
    SignalingIceServer_t iceServers[ SIGNALING_CONTROLLER_ICE_SERVER_MAX_CONFIG_COUNT ];
    size_t iceServersNum = SIGNALING_CONTROLLER_ICE_SERVER_MAX_CONFIG_COUNT;

    // Prepare HTTP endpoint
    signalingChannelHttpEndpoint.pEndpoint = &( pCtx->httpsEndpoint[ 0 ] );
    signalingChannelHttpEndpoint.endpointLength = pCtx->httpsEndpointLength;
    // Prepare URL buffer
    signalRequest.pUrl = pCtx->httpUrlBuffer;
    signalRequest.urlLength = SIGNALING_CONTROLLER_HTTP_URL_BUFFER_LENGTH;
    // Prepare body buffer
    signalRequest.pBody = pCtx->httpBodyBuffer;
    signalRequest.bodyLength = SIGNALING_CONTROLLER_HTTP_BODY_BUFFER_LENGTH;
    // Create the API url
    getIceServerConfigRequestInfo.channelArn.pChannelArn = &( pCtx->signalingChannelArn[ 0 ] );
    getIceServerConfigRequestInfo.channelArn.channelArnLength = pCtx->signalingChannelArnLength;
    getIceServerConfigRequestInfo.pClientId = pCtx->pClientId;
    getIceServerConfigRequestInfo.clientIdLength = pCtx->clientIdLength;

    retSignal = Signaling_ConstructGetIceServerConfigRequest( &signalingChannelHttpEndpoint, &getIceServerConfigRequestInfo, &signalRequest );

    if( retSignal != SIGNALING_RESULT_OK )
    {
        LogError( ( "Fail to construct get ICE server config request, return=0x%x", retSignal ) );
        ret = SIGNALING_CONTROLLER_RESULT_FAIL;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        memset( &request, 0, sizeof( HttpRequest_t ) );
        request.pUrl = signalRequest.pUrl;
        request.urlLength = signalRequest.urlLength;
        request.pBody = signalRequest.pBody;
        request.bodyLength = signalRequest.bodyLength;
        request.isFetchingCredential = 0U;

        memset( &response, 0, sizeof( HttpResponse_t ) );
        response.pBuffer = pCtx->httpResponserBuffer;
        response.bufferLength = SIGNALING_CONTROLLER_HTTP_RESPONSE_BUFFER_LENGTH;

        ret = HttpSend( pCtx, &request, 0, &response );
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        retSignal = Signaling_ParseGetIceServerConfigResponse( response.pBuffer, response.bufferLength, iceServers, &iceServersNum );

        if( retSignal != SIGNALING_RESULT_OK )
        {
            LogError( ( "Fail to parse get ICE server config response, return=0x%x", retSignal ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
    }

    // Parse the response
    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        ret = UpdateIceServerConfigs( pCtx, iceServers, iceServersNum );
    }

    return ret;
}

static SignalingControllerResult_t ConnectToWssEndpoint( SignalingControllerContext_t * pCtx )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    SignalingResult_t retSignal;
    SignalingChannelEndpoint_t wssEndpoint;
    ConnectWssEndpointRequestInfo_t wssEndpointRequestInfo;
    SignalingRequest_t signalRequest;
    WebsocketServerInfo_t serverInfo;

    LogInfo( ( "Connecting to Websocket Endpoint." ) );

    // Prepare WSS endpoint
    wssEndpoint.pEndpoint = &( pCtx->wssEndpoint[ 0 ] );
    wssEndpoint.endpointLength = pCtx->wssEndpointLength;
    // Prepare URL buffer
    signalRequest.pUrl = pCtx->httpUrlBuffer;
    signalRequest.urlLength = SIGNALING_CONTROLLER_HTTP_URL_BUFFER_LENGTH;
    // Prepare body buffer
    signalRequest.pBody = NULL;
    signalRequest.bodyLength = 0;
    // Create the API url
    memset( &wssEndpointRequestInfo, 0, sizeof( ConnectWssEndpointRequestInfo_t ) );
    wssEndpointRequestInfo.channelArn.pChannelArn = pCtx->signalingChannelArn;
    wssEndpointRequestInfo.channelArn.channelArnLength = pCtx->signalingChannelArnLength;
    wssEndpointRequestInfo.role = pCtx->role;
    wssEndpointRequestInfo.pClientId = pCtx->pClientId;
    wssEndpointRequestInfo.clientIdLength = pCtx->clientIdLength;

    retSignal = Signaling_ConstructConnectWssEndpointRequest( &wssEndpoint, &wssEndpointRequestInfo, &signalRequest );

    if( retSignal != SIGNALING_RESULT_OK )
    {
        LogError( ( "Fail to construct connect WSS endpoint request, return=0x%x", retSignal ) );
        ret = SIGNALING_CONTROLLER_RESULT_FAIL;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        serverInfo.pUrl = signalRequest.pUrl;
        serverInfo.urlLength = signalRequest.urlLength;
        serverInfo.port = WEBSOCKET_ENDPOINT_PORT;
        ret = SignalingController_WebsocketConnect( pCtx, &serverInfo );

        if( ret != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to connect with WSS endpoint" ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
    }

    return ret;
}

static void HandleForceRefreshIceServerConfigs( SignalingControllerContext_t * pCtx )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    uint64_t initTimeSec = NetworkingUtils_GetCurrentTimeSec( NULL );
    uint64_t currTimeSec = initTimeSec;

    while( currTimeSec < initTimeSec + SIGNALING_CONTROLLER_REFRESH_ICE_SERVER_CONFIGS_TIMEOUT )
    {
        ret = SignalingController_RefreshIceServerConfigs( pCtx );

        if( ret == SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogInfo( ( "Ice-Server Reconnection Successful." ) );
            break;
        }
        else
        {
            LogError( ( "Unable to Reconnect Ice Server." ) );

            currTimeSec = NetworkingUtils_GetCurrentTimeSec( NULL );
        }
    }
}

static SignalingControllerResult_t HandleEvent( SignalingControllerContext_t * pCtx,
                                                SignalingControllerEventMessage_t * pEventMsg )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    WebsocketResult_t websocketRet;
    SignalingControllerEventStatus_t callbackEventStatus = SIGNALING_CONTROLLER_EVENT_STATUS_NONE;
    Base64Result_t retBase64;
    WssSendMessage_t wssSendMessage;
    SignalingControllerEventContentSend_t * pEventContentSend;
    SignalingResult_t retSignal;

    switch( pEventMsg->event )
    {
        case SIGNALING_CONTROLLER_EVENT_SEND_WSS_MESSAGE:
            /* Allocate the ring buffer to store constructed signaling messages. */
            pEventContentSend = &pEventMsg->eventContent;
            callbackEventStatus = SIGNALING_CONTROLLER_EVENT_STATUS_SENT_FAIL;

            LogDebug( ( "Sending WSS message(%u): %.*s",
                        pEventContentSend->decodeMessageLength,
                        ( int ) pEventContentSend->decodeMessageLength, pEventContentSend->pDecodeMessage ) );

            /* Then fill the event information, like correlation ID, recipient client ID and base64 encoded message.
             * Note that the message now is not based encoded yet. */
            pCtx->signalingIntermediateMessageLength = SIGNALING_CONTROLLER_MESSAGE_BUFFER_LENGTH;
            retBase64 = Base64_Encode( pEventContentSend->pDecodeMessage, pEventContentSend->decodeMessageLength, pCtx->signalingIntermediateMessageBuffer, &pCtx->signalingIntermediateMessageLength );
            if( retBase64 != BASE64_RESULT_OK )
            {
                ret = SIGNALING_CONTROLLER_RESULT_FAIL;
            }

            if( ret == SIGNALING_CONTROLLER_RESULT_OK )
            {
                /* Construct signaling message into ring buffer. */
                memset( &wssSendMessage, 0, sizeof( WssSendMessage_t ) );

                // Prepare the buffer to send
                wssSendMessage.messageType = pEventContentSend->messageType;
                wssSendMessage.pBase64EncodedMessage = pCtx->signalingIntermediateMessageBuffer;
                wssSendMessage.base64EncodedMessageLength = pCtx->signalingIntermediateMessageLength;
                wssSendMessage.pCorrelationId = pEventContentSend->correlationId;
                wssSendMessage.correlationIdLength = pEventContentSend->correlationIdLength;
                wssSendMessage.pRecipientClientId = pEventContentSend->remoteClientId;
                wssSendMessage.recipientClientIdLength = pEventContentSend->remoteClientIdLength;

                /* We must preserve LWS_PRE ahead of buffer for libwebsockets. */
                pCtx->signalingTxMessageLength = SIGNALING_CONTROLLER_MESSAGE_BUFFER_LENGTH;
                retSignal = Signaling_ConstructWssMessage( &wssSendMessage, pCtx->signalingTxMessageBuffer, &pCtx->signalingTxMessageLength );
                if( retSignal != SIGNALING_RESULT_OK )
                {
                    LogError( ( "Fail to construct Wss message, result: %d", retSignal ) );
                    ret = SIGNALING_CONTROLLER_RESULT_FAIL;
                }
            }

            if( ret == SIGNALING_CONTROLLER_RESULT_OK )
            {
                LogVerbose( ( "Constructed WSS message length: %u, message: \n%.*s", pCtx->signalingTxMessageLength,
                              ( int ) pCtx->signalingTxMessageLength, pCtx->signalingTxMessageBuffer ) );

                /* Finally, sent it to websocket layer. */
                websocketRet = Websocket_Send( &( pCtx->websocketContext ), pCtx->signalingTxMessageBuffer, pCtx->signalingTxMessageLength );
                if( websocketRet != WEBSOCKET_RESULT_OK )
                {
                    LogError( ( "Fail to construct Wss message, result: %d", retSignal ) );
                    ret = SIGNALING_CONTROLLER_RESULT_FAIL;
                    callbackEventStatus = SIGNALING_CONTROLLER_EVENT_STATUS_SENT_FAIL;
                }
                else
                {
                    callbackEventStatus = SIGNALING_CONTROLLER_EVENT_STATUS_SENT_DONE;
                }
            }
            break;

        case SIGNALING_CONTROLLER_EVENT_RECONNECT_WSS:

            /* Disconnect the Web-Socket Server. */
            websocketRet = Websocket_Disconnect( &( pCtx->websocketContext ) );

            if( websocketRet == WEBSOCKET_RESULT_OK )
            {
                LogInfo( ( "Disconnected Websocket Server. " ) );

                /* Change the State to Describe state and attempt Re-connection. */
                ret = ConnectToSignalingService( pCtx );
            }
            if( ret != SIGNALING_CONTROLLER_RESULT_OK )
            {
                LogInfo( ( "Reconnection Un-Succesfull. %d ",ret ) );
                websocketRet = NETWORKING_WSLAY_RESULT_FAIL_CONNECT;
            }
            if( ret == SIGNALING_CONTROLLER_RESULT_OK )
            {
                LogInfo( ( "Reconnection Succesfull. " ) );
            }
            break;
        case SIGNALING_CONTROLLER_EVENT_FORCE_REFRESH_ICE_SERVER_CONFIGS:
            HandleForceRefreshIceServerConfigs( pCtx );
            break;
        default:
            /* Ignore unknown event. */
            LogWarn( ( "Received unknown event %d", pEventMsg->event ) );
            break;
    }

    if( ( pEventMsg->onCompleteCallback != NULL ) && ( callbackEventStatus != SIGNALING_CONTROLLER_EVENT_STATUS_NONE ) )
    {
        pEventMsg->onCompleteCallback( callbackEventStatus, pEventMsg->pOnCompleteCallbackContext );
    }

    return ret;
}

SignalingControllerResult_t SignalingController_Init( SignalingControllerContext_t * pCtx,
                                                      const SSLCredentials_t * pSslCreds )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    MessageQueueResult_t retMessageQueue;

    if( ( pCtx == NULL ) || ( pSslCreds == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: 0x%p ", pCtx ) );
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        if( pSslCreds->pCaCertPem != NULL )
        {
            pCtx->pCaCertPem = pSslCreds->pCaCertPem;

            /* 1 is to account for the NULL terminator required by the SSL library */
            pCtx->caCertPemSize = strlen( ( char * )pSslCreds->pCaCertPem ) + 1;
        }

        if( pSslCreds->pDeviceCertPem != NULL )
        {
            pCtx->pIotThingCert = pSslCreds->pDeviceCertPem;

            /* 1 is to account for the NULL terminator required by the SSL library */
            pCtx->iotThingCertSize = strlen( ( char * )pSslCreds->pDeviceCertPem ) + 1;
        }

        if( pSslCreds->pDeviceKeyPem != NULL )
        {
            pCtx->pIotThingPrivateKey = pSslCreds->pDeviceKeyPem;

            /* 1 is to account for the NULL terminator required by the SSL library */
            pCtx->iotThingPrivateKeySize = strlen( ( char * )pSslCreds->pDeviceKeyPem ) + 1;
        }
    }

    /* Initialize HTTP. */
    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        ret = SignalingController_HttpInit( pCtx );
    }

    /* Initializa Websocket. */
    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        ret = SignalingController_WebsocketInit( pCtx );
    }

    /* Initializa Message Queue. */
    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        /* Delete message queue from previous round. */
        MessageQueue_Destroy( NULL, SIGNALING_CONTROLLER_MESSAGE_QUEUE_NAME );

        retMessageQueue = MessageQueue_Create( &pCtx->sendMessageQueue, SIGNALING_CONTROLLER_MESSAGE_QUEUE_NAME, sizeof( SignalingControllerEventMessage_t ), MAX_QUEUE_MSG_NUM );
        if( retMessageQueue != MESSAGE_QUEUE_RESULT_OK )
        {
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        OnConnectionStateChange( pCtx, SIGNALING_CONTROLLER_STATE_INITED );
    }

    return ret;
}

SignalingControllerResult_t SignalingController_RefreshIceServerConfigs( SignalingControllerContext_t * pCtx )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    WebsocketResult_t retWebsocket = WEBSOCKET_RESULT_OK;

    if( pCtx == NULL )
    {
        LogWarn( ( "Invalid input parameter, pCtx is NULL" ) );
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        /* Disconnect the Web-Socket Server. */
        retWebsocket = Websocket_Disconnect( &( pCtx->websocketContext ) );
        if( retWebsocket != WEBSOCKET_RESULT_OK )
        {
            LogWarn( ( "Web-Socket Disconnect Unsuccessfull with reason %d.", retWebsocket ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        LogDebug( ( "Disconnected Websocket Server." ) );

        /* Change the State to Describe state and attempt Re-connection. */
        ret = GetIceServerConfigs( pCtx );
    }
    else
    {
        LogWarn( ( " Fetching ICE Server List Unsuccessfull. " ) );
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        /* Reconnect to the websocket secure endpoint. */
        ret = ConnectToWssEndpoint( pCtx );
    }
    else
    {
        LogWarn( ( " Reconnection WSS Endpoint Unsuccessfull. " ) );
    }

    return ret;
}

static SignalingControllerResult_t ConnectToSignalingService( SignalingControllerContext_t * pCtx )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    IceServerConfig_t * pIceServerConfigs;
    size_t iceServerConfigsCount;

    /* Get security token. */
    if( AreCredentialsExpired( pCtx ) != 0U )
    {
        #if METRIC_PRINT_ENABLED
        Metric_StartEvent( METRIC_EVENT_SIGNALING_GET_CREDENTIALS );
        #endif
        ret = FetchTemporaryCredentials( pCtx );
        #if METRIC_PRINT_ENABLED
        Metric_EndEvent( METRIC_EVENT_SIGNALING_GET_CREDENTIALS );
        #endif
    }

    /* Execute describe channel if no channel ARN. */
    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        #if METRIC_PRINT_ENABLED
        Metric_StartEvent( METRIC_EVENT_SIGNALING_DESCRIBE_CHANNEL );
        #endif
        ret = DescribeSignalingChannel( pCtx );
        #if METRIC_PRINT_ENABLED
        Metric_EndEvent( METRIC_EVENT_SIGNALING_DESCRIBE_CHANNEL );
        #endif
    }

    /* Query signaling channel endpoints with channel ARN. */
    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        #if METRIC_PRINT_ENABLED
        Metric_StartEvent( METRIC_EVENT_SIGNALING_GET_ENDPOINTS );
        #endif
        ret = GetSignalingChannelEndpoints( pCtx );
        #if METRIC_PRINT_ENABLED
        Metric_EndEvent( METRIC_EVENT_SIGNALING_GET_ENDPOINTS );
        #endif
    }

    /* Query ICE server list with HTTPS endpoint. */
    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        ret = SignalingController_QueryIceServerConfigs( pCtx, &pIceServerConfigs, &iceServerConfigsCount );
    }

    /* Connect websocket secure endpoint. */
    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        #if METRIC_PRINT_ENABLED
        Metric_StartEvent( METRIC_EVENT_SIGNALING_CONNECT_WSS_SERVER );
        #endif
        ret = ConnectToWssEndpoint( pCtx );
        #if METRIC_PRINT_ENABLED
        Metric_EndEvent( METRIC_EVENT_SIGNALING_CONNECT_WSS_SERVER );
        #endif
    }

    /* Join the storage session, if enabled. */
    if ( ret == SIGNALING_CONTROLLER_RESULT_OK && 
         ( pCtx->enableStorageSession != 0 ) )
    {
        #if METRIC_PRINT_ENABLED
        Metric_StartEvent( METRIC_EVENT_SIGNALING_JOIN_STORAGE_SESSION );
        #endif
        ret = JoinStorageSession( pCtx );
        #if METRIC_PRINT_ENABLED
        Metric_EndEvent( METRIC_EVENT_SIGNALING_JOIN_STORAGE_SESSION );
        #endif
    }

    /* Print metric. */
    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        OnConnectionStateChange( pCtx, SIGNALING_CONTROLLER_STATE_CONNECTED );
        LogSignalingInfo( pCtx );
    }
    else
    {
        OnConnectionStateChange( pCtx, SIGNALING_CONTROLLER_STATE_DISCONNECTED );
    }

    return ret;
}

SignalingControllerResult_t SignalingController_ConnectServers( SignalingControllerContext_t * pCtx,
                                                                const SignalingControllerConnectInfo_t * pConnectInfo )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;

    pCtx->awsConfig = pConnectInfo->awsConfig;

    pCtx->pChannelName = pConnectInfo->channelName.pChannelName;
    pCtx->channelNameLength = pConnectInfo->channelName.channelNameLength;

    pCtx->pUserAgentName = pConnectInfo->pUserAgentName;
    pCtx->userAgentNameLength = pConnectInfo->userAgentNameLength;

    pCtx->pIotCredentialsEndpoint = pConnectInfo->awsIotCreds.pIotCredentialsEndpoint;
    pCtx->iotCredentialsEndpointLength = pConnectInfo->awsIotCreds.iotCredentialsEndpointLength;

    pCtx->pIotThingName = pConnectInfo->awsIotCreds.pIotThingName;
    pCtx->iotThingNameLength = pConnectInfo->awsIotCreds.iotThingNameLength;

    pCtx->pRoleAlias = pConnectInfo->awsIotCreds.pRoleAlias;
    pCtx->roleAliasLength = pConnectInfo->awsIotCreds.roleAliasLength;

    memcpy( pCtx->accessKeyId, pConnectInfo->awsCreds.pAccessKeyId, pConnectInfo->awsCreds.accessKeyIdLength );
    pCtx->accessKeyIdLength = pConnectInfo->awsCreds.accessKeyIdLength;
    pCtx->accessKeyId[ pCtx->accessKeyIdLength ] = '\0';

    memcpy( pCtx->secretAccessKey, pConnectInfo->awsCreds.pSecretAccessKey, pConnectInfo->awsCreds.secretAccessKeyLength );
    pCtx->secretAccessKeyLength = pConnectInfo->awsCreds.secretAccessKeyLength;
    pCtx->secretAccessKey[ pCtx->secretAccessKeyLength ] = '\0';

    memcpy( pCtx->sessionToken, pConnectInfo->awsCreds.pSessionToken, pConnectInfo->awsCreds.sessionTokenLength );
    pCtx->sessionTokenLength = pConnectInfo->awsCreds.sessionTokenLength;
    pCtx->sessionToken[ pCtx->sessionTokenLength ] = '\0';

    pCtx->messageReceivedCallback = pConnectInfo->messageReceivedCallback;
    pCtx->pMessageReceivedCallbackData = pConnectInfo->pMessageReceivedCallbackData;

    pCtx->expirationSeconds = pConnectInfo->awsCreds.expirationSeconds;

    pCtx->enableStorageSession = pConnectInfo->enableStorageSession;

    pCtx->pClientId = pConnectInfo->pClientId;
    pCtx->clientIdLength = pConnectInfo->clientIdLength;
    pCtx->role = pConnectInfo->role;

    ret = ConnectToSignalingService( pCtx );

    if( ret != SIGNALING_CONTROLLER_RESULT_OK )
    {
        LogError( ( "Failed to connect to signaling service. Result: %d!", ret ) );
    }

    return ret;
}

SignalingControllerResult_t SignalingController_StartListening( SignalingControllerContext_t * pCtx,
                                                                const SignalingControllerConnectInfo_t * pConnectInfo )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;;
    WebsocketResult_t websocketRet;
    MessageQueueResult_t messageQueueRet;
    SignalingControllerEventMessage_t eventMsg;
    size_t eventMsgLength;

    if( ( pCtx == NULL ) || ( pConnectInfo == NULL ) )
    {
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }
    else if( ( pConnectInfo->role == SIGNALING_ROLE_VIEWER ) &&
             ( pConnectInfo->enableStorageSession != 0U ) )
    {
        /* Viewer doesn't support Join Storage Session.
         * Refer to https://docs.aws.amazon.com/kinesisvideostreams-webrtc-dg/latest/devguide/getting-started-ingestion.html
         * for more detail. */
        LogError( ( "Viewer doesn't support Join Storage Session." ) );
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        for( ;; )
        {
            ret = SignalingController_ConnectServers( pCtx, pConnectInfo );
        
            if( ret != SIGNALING_CONTROLLER_RESULT_OK )
            {
                LogError( ( "Fail to connect with signaling controller." ) );
                vTaskDelay( pdMS_TO_TICKS( SIGNALING_CONTROLLER_RETRY_CONNECT_TIMEOUT_MS ) );
            }
            else
            {
                for( ;; )
                {
                    websocketRet = Websocket_Recv( &( pCtx->websocketContext ) );
            
                    if( websocketRet != WEBSOCKET_RESULT_OK )
                    {
                        LogError( ( "Websocket_Recv fail, return 0x%x", websocketRet ) );
                        break;
                    }
            
                    messageQueueRet = MessageQueue_IsEmpty( &pCtx->sendMessageQueue );
            
                    while( messageQueueRet == MESSAGE_QUEUE_RESULT_MQ_HAVE_MESSAGE )
                    {
                        /* Handle event. */
                        eventMsgLength = sizeof( SignalingControllerEventMessage_t );
                        messageQueueRet = MessageQueue_Recv( &pCtx->sendMessageQueue, &eventMsg, &eventMsgLength );
                        if( messageQueueRet == MESSAGE_QUEUE_RESULT_OK )
                        {
                            /* Received message, process it. */
                            LogDebug( ( "EventMsg: event: %d, pOnCompleteCallbackContext: %p", eventMsg.event, eventMsg.pOnCompleteCallbackContext ) );
                            ret = HandleEvent( pCtx, &eventMsg );
                            if( ret != SIGNALING_CONTROLLER_RESULT_OK )
                            {
                                LogError( ( "Fail to handle event, return %d", ret ) );
                            }
                        }
            
                        messageQueueRet = MessageQueue_IsEmpty( &pCtx->sendMessageQueue );
                    }

                    if( AreCredentialsExpired( pCtx ) != 0U )
                    {
                        ret = FetchTemporaryCredentials( pCtx );
                        if( ret != SIGNALING_CONTROLLER_RESULT_OK )
                        {
                            LogWarn( ( "Fail to fetch temporary credentials, reconnecting." ) );
                            break;
                        }
                    }
                }
            }
        }
    }

    return ret;
}

SignalingControllerResult_t SignalingController_SendMessage( SignalingControllerContext_t * pCtx,
                                                             SignalingControllerEventMessage_t * pEventMsg )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    MessageQueueResult_t retMessageQueue;

    if( ( pCtx == NULL ) || ( pEventMsg == NULL ) )
    {
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        retMessageQueue = MessageQueue_Send( &pCtx->sendMessageQueue, pEventMsg, sizeof( SignalingControllerEventMessage_t ) );
        if( retMessageQueue != MESSAGE_QUEUE_RESULT_OK )
        {
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        /* Wake the running thread up to handle event. */
        ( void ) Websocket_Signal( &( pCtx->websocketContext ) );
    }

    return ret;
}

SignalingControllerResult_t SignalingController_QueryIceServerConfigs( SignalingControllerContext_t * pCtx,
                                                                       IceServerConfig_t ** ppIceServerConfigs,
                                                                       size_t * pIceServerConfigsCount )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    uint64_t currentTimeSec;

    LogInfo( ( "Quering Ice Server Configurations." ) );

    if( ( pCtx == NULL ) || ( ppIceServerConfigs == NULL ) || ( pIceServerConfigsCount == NULL ) )
    {
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        currentTimeSec = NetworkingUtils_GetCurrentTimeSec( NULL );

        if( ( pCtx->iceServerConfigsCount == 0 ) || ( pCtx->iceServerConfigExpirationSec < currentTimeSec ) )
        {
            LogInfo( ( "Ice server configs expired, Starting Refresing Configs." ) );

            #if METRIC_PRINT_ENABLED
            Metric_StartEvent( METRIC_EVENT_SIGNALING_GET_ICE_SERVER_LIST );
            #endif
            ret = GetIceServerConfigs( pCtx );
            #if METRIC_PRINT_ENABLED
            Metric_EndEvent( METRIC_EVENT_SIGNALING_GET_ICE_SERVER_LIST );
            #endif
        }

        *ppIceServerConfigs = pCtx->iceServerConfigs;
        *pIceServerConfigsCount = pCtx->iceServerConfigsCount;
    }

    return ret;
}

SignalingControllerResult_t SignalingController_ExtractSdpMessageFromSignalingMessage( const char * pEventMessage,
                                                                                       size_t eventMessageLength,
                                                                                       uint8_t isSdpOffer,
                                                                                       const char ** ppSdpMessage,
                                                                                       size_t * pSdpMessageLength )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    JSONStatus_t jsonResult;
    size_t start = 0, next = 0;
    JSONPair_t pair = { 0 };
    uint8_t isContentFound = 0;
    const char * pTargetTypeValue = isSdpOffer == 1 ? SIGNALING_CONTROLLER_SDP_EVENT_MESSAGE_TYPE_VALUE_OFFER : SIGNALING_CONTROLLER_SDP_EVENT_MESSAGE_TYPE_VALUE_ANSWER;

    if( ( pEventMessage == NULL ) ||
        ( ppSdpMessage == NULL ) ||
        ( pSdpMessageLength == NULL ) )
    {
        LogError( ( "Invalid input, pEventMessage: %p, ppSdpMessage: %p, pSdpMessageLength: %p", pEventMessage, ppSdpMessage, pSdpMessageLength ) );
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        jsonResult = JSON_Validate( pEventMessage, eventMessageLength );

        if( jsonResult != JSONSuccess )
        {
            LogWarn( ( "Input message is not valid JSON message, result: %d, message(%d): %.*s",
                       jsonResult,
                       eventMessageLength,
                       ( int ) eventMessageLength,
                       pEventMessage ) );
            ret = SIGNALING_CONTROLLER_RESULT_FAIL;
        }
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        jsonResult = JSON_Iterate( pEventMessage, eventMessageLength, &start, &next, &pair );

        while( jsonResult == JSONSuccess )
        {
            if( ( strncmp( pair.key, SIGNALING_CONTROLLER_SDP_EVENT_MESSAGE_TYPE_KEY, pair.keyLength ) == 0 ) &&
                ( strncmp( pair.value, pTargetTypeValue, pair.valueLength ) != 0 ) )
            {
                /* It's not expected SDP offer message. */
                LogWarn( ( "Message type \"%.*s\" is not SDP target type \"%s\"",
                           ( int ) pair.valueLength, pair.value,
                           pTargetTypeValue ) );
                ret = SIGNALING_CONTROLLER_RESULT_FAIL;
                break;
            }
            else if( strncmp( pair.key, SIGNALING_CONTROLLER_SDP_EVENT_MESSAGE_CONTENT_KEY, pair.keyLength ) == 0 )
            {
                *ppSdpMessage = pair.value;
                *pSdpMessageLength = pair.valueLength;
                isContentFound = 1;
                break;
            }
            else
            {
                /* Skip unknown attributes. */
            }

            jsonResult = JSON_Iterate( pEventMessage, eventMessageLength, &start, &next, &pair );
        }
    }

    if( ( ret == SIGNALING_CONTROLLER_RESULT_OK ) && !isContentFound )
    {
        LogWarn( ( "No target content found in event message, result: %d, SDP target type \"%s\", message(%d): %.*s",
                   jsonResult,
                   pTargetTypeValue,
                   eventMessageLength,
                   ( int ) eventMessageLength,
                   pEventMessage ) );
        ret = SIGNALING_CONTROLLER_RESULT_FAIL;
    }

    return ret;
}

SignalingControllerResult_t SignalingController_DeserializeSdpContentNewline( const char * pSdpMessage,
                                                                              size_t sdpMessageLength,
                                                                              char * pFormalSdpMessage,
                                                                              size_t * pFormalSdpMessageLength )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    const char * pCurSdp = pSdpMessage, * pNext = NULL;
    char * pCurOutput = NULL;
    size_t lineLength = 0, outputLength = 0;

    if( ( pSdpMessage == NULL ) ||
        ( pFormalSdpMessage == NULL ) ||
        ( pFormalSdpMessageLength == NULL ) )
    {
        LogError( ( "Invalid input, pSdpMessage: %p, pFormalSdpMessage: %p, pFormalSdpMessageLength: %p", pSdpMessage, pFormalSdpMessage, pFormalSdpMessageLength ) );
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        pCurOutput = pFormalSdpMessage;

        while( ( pNext = strstr( pCurSdp, SIGNALING_CONTROLLER_SDP_EVENT_MESSAGE_NEWLINE_ENDING ) ) != NULL )
        {
            lineLength = pNext - pCurSdp;

            if( ( lineLength >= 2 ) &&
                ( pCurSdp[ lineLength - 2 ] == '\\' ) && ( pCurSdp[ lineLength - 1 ] == 'r' ) )
            {
                lineLength -= 2;
            }

            if( *pFormalSdpMessageLength < outputLength + lineLength + 2 )
            {
                LogWarn( ( "Buffer space is not enough to store formal SDP message, buffer size: %u, SDP message(%u): %.*s",
                           *pFormalSdpMessageLength,
                           sdpMessageLength,
                           ( int ) sdpMessageLength,
                           pSdpMessage ) );
                ret = SIGNALING_CONTROLLER_RESULT_FAIL;
                break;
            }

            memcpy( pCurOutput, pCurSdp, lineLength );
            pCurOutput += lineLength;
            *pCurOutput++ = '\r';
            *pCurOutput++ = '\n';
            outputLength += lineLength + 2;

            pCurSdp = pNext + 2;
        }
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        *pFormalSdpMessageLength = outputLength;
    }

    return ret;
}

SignalingControllerResult_t SignalingController_SerializeSdpContentNewline( const char * pSdpMessage,
                                                                            size_t sdpMessageLength,
                                                                            char * pEventSdpMessage,
                                                                            size_t * pEventSdpMessageLength )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    const char * pCurSdp = pSdpMessage, * pNext, * pTail;
    char * pCurOutput = pEventSdpMessage;
    size_t lineLength, outputLength = 0;
    int writtenLength;

    if( ( pSdpMessage == NULL ) ||
        ( pEventSdpMessage == NULL ) ||
        ( pEventSdpMessageLength == NULL ) )
    {
        LogError( ( "Invalid input, pSdpMessage: %p, pEventSdpMessage: %p, pEventSdpMessageLength: %p", pSdpMessage, pEventSdpMessage, pEventSdpMessageLength ) );
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        pTail = pSdpMessage + sdpMessageLength;

        while( ( pNext = memchr( pCurSdp, '\n', pTail - pCurSdp ) ) != NULL )
        {
            lineLength = pNext - pCurSdp;

            if( ( lineLength > 0 ) &&
                ( pCurSdp[ lineLength - 1 ] == '\r' ) )
            {
                lineLength--;
            }
            else
            {
                /* do nothing, coverity happy. */
            }

            if( *pEventSdpMessageLength < outputLength + lineLength + 4 )
            {
                LogError( ( "The output buffer length(%u) is too small to store serialized %u bytes message.",
                            *pEventSdpMessageLength,
                            sdpMessageLength ) );
                ret = SIGNALING_CONTROLLER_RESULT_FAIL;
                break;
            }

            writtenLength = snprintf( pCurOutput, *pEventSdpMessageLength - outputLength, "%.*s\\r\\n",
                                      ( int ) lineLength,
                                      pCurSdp );
            if( writtenLength < 0 )
            {
                ret = SIGNALING_CONTROLLER_RESULT_FAIL;
                LogError( ( "snprintf returns fail %d", writtenLength ) );
                break;
            }
            else
            {
                outputLength += lineLength + 4;
                pCurOutput += lineLength + 4;
            }

            pCurSdp = pNext + 1;
        }
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        if( pTail > pCurSdp )
        {
            /* Copy the ending string. */
            lineLength = pTail - pCurSdp;
            memcpy( pCurOutput, pCurSdp, lineLength );

            outputLength += lineLength;
            pCurOutput += lineLength;
            pCurSdp += lineLength;
        }
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        *pEventSdpMessageLength = outputLength;
    }

    return ret;
}

SignalingControllerResult_t SignalingController_SetConnectionStateCallback( SignalingControllerContext_t * pCtx,
                                                                            SignalingControllerConnectionStateCallback_t callback,
                                                                            void * pCustomContext )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;

    if( pCtx == NULL )
    {
        LogError( ( "Invalid input: pCtx is NULL" ) );
        ret = SIGNALING_CONTROLLER_RESULT_BAD_PARAM;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        pCtx->connectionStateCallback = callback;
        pCtx->pConnectionStateCallbacCustomContext = pCustomContext;
    }

    return ret;
}

static SignalingControllerResult_t JoinStorageSession( SignalingControllerContext_t *pCtx )
{
    SignalingControllerResult_t ret = SIGNALING_CONTROLLER_RESULT_OK;
    SignalingResult_t signalingResult;
    SignalingChannelEndpoint_t webrtcEndpoint;
    JoinStorageSessionRequestInfo_t joinSessionRequestInfo;
    SignalingRequest_t signalingRequest;
    HttpRequest_t httpRequest;
    HttpResponse_t httpResponse;

    LogInfo( ( "Joining Storage Session." ) );

    signalingRequest.pUrl = &( pCtx->httpUrlBuffer[ 0 ] );
    signalingRequest.urlLength = SIGNALING_CONTROLLER_HTTP_URL_BUFFER_LENGTH;

    signalingRequest.pBody = &( pCtx->httpBodyBuffer[ 0 ] );
    signalingRequest.bodyLength = SIGNALING_CONTROLLER_HTTP_BODY_BUFFER_LENGTH;

    /* Create the API request. */
    memset( &( joinSessionRequestInfo ), 0, sizeof( JoinStorageSessionRequestInfo_t ) );
    joinSessionRequestInfo.channelArn.pChannelArn = &( pCtx->signalingChannelArn[ 0 ] );
    joinSessionRequestInfo.channelArn.channelArnLength = pCtx->signalingChannelArnLength;
    joinSessionRequestInfo.role = SIGNALING_ROLE_MASTER;

    webrtcEndpoint.pEndpoint = &( pCtx->webrtcEndpoint[ 0 ] );
    webrtcEndpoint.endpointLength = pCtx->webrtcEndpointLength;

    LogDebug( ( "Joining storage session for channel: %s with length: %d",
                pCtx->signalingChannelArn,
                pCtx->signalingChannelArnLength ) );
    signalingResult = Signaling_ConstructJoinStorageSessionRequest( &( webrtcEndpoint ),
                                                                    &( joinSessionRequestInfo ),
                                                                    &( signalingRequest ) );

    if( signalingResult != SIGNALING_RESULT_OK )
    {
        LogError( ( "Failed to construct join storage session request, return=0x%x", signalingResult ) );
        ret = SIGNALING_CONTROLLER_RESULT_FAIL;
    }

    if( ret == SIGNALING_CONTROLLER_RESULT_OK )
    {
        memset( &( httpRequest ), 0, sizeof( HttpRequest_t ) );
        httpRequest.pUrl = signalingRequest.pUrl;
        httpRequest.urlLength = signalingRequest.urlLength;
        httpRequest.pBody = signalingRequest.pBody;
        httpRequest.bodyLength = signalingRequest.bodyLength;
        httpRequest.verb = HTTP_POST;

        memset( &( httpResponse ), 0, sizeof( HttpResponse_t ) );
        httpResponse.pBuffer = pCtx->httpResponserBuffer;
        httpResponse.bufferLength = SIGNALING_CONTROLLER_HTTP_RESPONSE_BUFFER_LENGTH;
        ret = HttpSend( pCtx, &( httpRequest ), 0, &( httpResponse ) );
        if ( ret != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "HTTP request failed, error=0x%x", ret ) );
        }
    }

    return ret;
}
