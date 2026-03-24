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

#include <errno.h>
#include <time.h>
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip_netconf.h"
#include "logging.h"
#include "ice_controller.h"
#include "ice_controller_private.h"
#include "ice_api.h"
#include "signaling_controller.h"
#if METRIC_PRINT_ENABLED
#include "metric.h"
#endif
#include "networking_utils.h"

#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_UNKNOWN "UNKNOWN"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_BINDING_REQUEST "BINDING_REQUEST"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_BINDING_SUCCESS "BINDING_SUCCESS_RESPONSE"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_BINDING_FAILURE "BINDING_FAILURE_RESPONSE"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_BINDING_INDICATION "BINDING_INDICATION"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_ALLOCATE_REQUEST "ALLOCATE_REQUEST"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_ALLOCATE_SUCCESS "ALLOCATE_SUCCESS_RESPONSE"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_ALLOCATE_FAILURE "ALLOCATE_FAILURE_RESPONSE"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_REFRESH_REQUEST "REFRESH_REQUEST"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_REFRESH_SUCCESS "REFRESH_SUCCESS_RESPONSE"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_REFRESH_FAILURE "REFRESH_FAILURE_RESPONSE"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CREATE_PERMISSION_REQUEST "CREATE_PERMISSION_REQUEST"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CREATE_PERMISSION_SUCCESS "CREATE_PERMISSION_SUCCESS_RESPONSE"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CREATE_PERMISSION_FAILURE "CREATE_PERMISSION_FAILURE_RESPONSE"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CHANNEL_BIND_REQUEST "CHANNEL_BIND_REQUEST"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CHANNEL_BIND_SUCCESS "CHANNEL_BIND_SUCCESS_RESPONSE"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CHANNEL_BIND_FAILURE "CHANNEL_BIND_FAILURE_RESPONSE"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_SEND_INDICATION "SEND_INDICATION"
#define ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_DATA_INDICATION "DATA_INDICATION"

#define ICE_CONTROLLER_RESEND_DELAY_MS ( 50 )
#define ICE_CONTROLLER_RESEND_TIMEOUT_MS ( 1000 )

static void GetLocalIPAdresses( IceEndpoint_t * pLocalIceEndpoints,
                                size_t * pLocalIceEndpointsNum )
{
    size_t localEndpointsSize = *pLocalIceEndpointsNum;
    uint8_t * pIpv4Address;

    if( localEndpointsSize >= 1 )
    {
        pIpv4Address = LwIP_GetIP( 0 );
        memset( &pLocalIceEndpoints[ 0 ], 0, sizeof( IceEndpoint_t ) );
        pLocalIceEndpoints[ 0 ].transportAddress.family = STUN_ADDRESS_IPv4;
        pLocalIceEndpoints[ 0 ].transportAddress.port = 0;
        memcpy( pLocalIceEndpoints[ 0 ].transportAddress.address, pIpv4Address, STUN_IPV4_ADDRESS_SIZE );
        pLocalIceEndpoints[ 0 ].isPointToPoint = 0;

        *pLocalIceEndpointsNum = 1;
    }
}

void IceControllerNet_UpdateSocketContext( IceControllerContext_t * pCtx,
                                           IceControllerSocketContext_t * pSocketContext,
                                           IceControllerSocketContextState_t newState,
                                           IceCandidate_t * pLocalCandidate,
                                           IceCandidate_t * pRemoteCandidate,
                                           IceControllerIceServer_t * pIceServer )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;

    if( ( pCtx == NULL ) || ( pSocketContext == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pSocketContext: %p, pLocalCandidate: %p", pCtx, pSocketContext, pLocalCandidate ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( xSemaphoreTake( pCtx->socketMutex, portMAX_DELAY ) == pdTRUE )
        {
            pSocketContext->state = newState;
            pSocketContext->pLocalCandidate = pLocalCandidate;
            pSocketContext->pRemoteCandidate = pRemoteCandidate;
            pSocketContext->pIceServer = pIceServer;

            xSemaphoreGive( pCtx->socketMutex );
        }
        else
        {
            LogError( ( "Failed to lock socket mutex." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
        }
    }
}

static IceControllerResult_t CreateSocketContextUdp( IceControllerContext_t * pCtx,
                                                     uint16_t family,
                                                     IceEndpoint_t * pBindEndpoint,
                                                     IceEndpoint_t * pConnectEndpoint,
                                                     IceSocketProtocol_t protocol,
                                                     IceControllerSocketContext_t ** ppOutSocketContext )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceControllerSocketContext_t * pSocketContext = NULL;
    struct sockaddr_in ipv4Address;
    // struct sockaddr_in6 ipv6Addr;
    struct sockaddr * pSockAddress = NULL;
    socklen_t addressLength;
    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = 1000
    };
    uint32_t sendBufferSize = 0;
    uint8_t needBinding = pBindEndpoint != NULL ? 1 : 0;

    ( void ) pConnectEndpoint;

    /* Find a free socket context. */
    if( pCtx->socketsContextsCount < ICE_CONTROLLER_MAX_LOCAL_CANDIDATE_COUNT )
    {
        pSocketContext = &pCtx->socketsContexts[ pCtx->socketsContextsCount++ ];
    }
    else
    {
        LogWarn( ( "No socket context available for ice controller. Current number: %u", pCtx->socketsContextsCount ) );
        ret = ICE_CONTROLLER_RESULT_NO_SOCKET_CONTEXT_AVAILABLE;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        pSocketContext->socketFd = socket( family == STUN_ADDRESS_IPv4 ? AF_INET : AF_INET6,
                                           SOCK_DGRAM,
                                           0 );

        if( pSocketContext->socketFd == -1 )
        {
            LogError( ( "socket() failed to create socket with errno: %s", strerror( errno ) ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_SOCKET_CREATE;
        }
    }

    if( ( ret == ICE_CONTROLLER_RESULT_OK ) && needBinding )
    {
        if( pBindEndpoint->transportAddress.family == STUN_ADDRESS_IPv4 )
        {
            memset( &ipv4Address, 0, sizeof( ipv4Address ) );
            ipv4Address.sin_family = AF_INET;
            ipv4Address.sin_port = 0; // use next available port
            memcpy( &ipv4Address.sin_addr, pBindEndpoint->transportAddress.address, STUN_IPV4_ADDRESS_SIZE );
            pSockAddress = ( struct sockaddr * ) &ipv4Address;
            addressLength = sizeof( struct sockaddr_in );
        }
        else
        {
            /* TODO: skip IPv6 for now. */
            // memset( &ipv6Addr, 0x00, sizeof(ipv6Addr) );
            // ipv6Addr.sin6_family = AF_INET6;
            // ipv6Addr.sin6_port = 0; // use next available port
            // memcpy(&ipv6Addr.sin6_addr, pBindEndpoint->transportAddress.address, STUN_IPV4_ADDRESS_SIZE);
            // pSockAddress = (struct sockaddr*) &ipv6Addr;
            // addressLength = sizeof(struct sockaddr_in6);
            ret = ICE_CONTROLLER_RESULT_IPV6_NOT_SUPPORT;
            close( pSocketContext->socketFd );
            pSocketContext->socketFd = -1;
            pCtx->socketsContextsCount--;
        }
    }

    if( ( ret == ICE_CONTROLLER_RESULT_OK ) && needBinding )
    {
        if( bind( pSocketContext->socketFd, pSockAddress, addressLength ) < 0 )
        {
            LogError( ( "socket() failed to bind socket with errno: %s", strerror( errno ) ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_SOCKET_BIND;
            close( pSocketContext->socketFd );
            pSocketContext->socketFd = -1;
            pCtx->socketsContextsCount--;
        }
    }

    if( ( ret == ICE_CONTROLLER_RESULT_OK ) && needBinding )
    {
        if( getsockname( pSocketContext->socketFd, pSockAddress, &addressLength ) < 0 )
        {
            LogError( ( "getsockname() failed with errno: %s", strerror( errno ) ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_SOCKET_GETSOCKNAME;
            close( pSocketContext->socketFd );
            pSocketContext->socketFd = -1;
            pCtx->socketsContextsCount--;
        }
        else
        {
            pBindEndpoint->transportAddress.port = ( uint16_t ) pBindEndpoint->transportAddress.family == STUN_ADDRESS_IPv4 ? ntohs( ipv4Address.sin_port ) : 0U;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        setsockopt( pSocketContext->socketFd, SOL_SOCKET, SO_SNDBUF, &sendBufferSize, sizeof( sendBufferSize ) );
        setsockopt( pSocketContext->socketFd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof( struct timeval ) );
        setsockopt( pSocketContext->socketFd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof( struct timeval ) );
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Assign to output when success. */
        pSocketContext->socketType = ICE_CONTROLLER_SOCKET_TYPE_UDP;
        *ppOutSocketContext = pSocketContext;
    }

    return ret;
}

static IceControllerResult_t CreateSocketContextTcp( IceControllerContext_t * pCtx,
                                                     uint16_t family,
                                                     IceEndpoint_t * pBindEndpoint,
                                                     IceEndpoint_t * pConnectEndpoint,
                                                     IceSocketProtocol_t protocol,
                                                     IceControllerSocketContext_t ** ppOutSocketContext )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceControllerSocketContext_t * pSocketContext = NULL;
    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = 1000
    };
    uint32_t sendBufferSize = 0;
    TlsTransportStatus_t xNetworkStatus;
    NetworkCredentials_t credentials;
    const char * pRemoteIpPos;
    char remoteIpAddr[ INET_ADDRSTRLEN ];

    pRemoteIpPos = inet_ntop( AF_INET,
                              pConnectEndpoint->transportAddress.address,
                              remoteIpAddr,
                              INET_ADDRSTRLEN );
    LogInfo( ( "Start TLS handshaking with %s:%d", pRemoteIpPos ? pRemoteIpPos : "UNKNOWN", pConnectEndpoint->transportAddress.port ) );
    if( pRemoteIpPos == NULL )
    {
        LogError( ( "Unknown address, address: 0x%02x%02x%02x%02x",
                    pConnectEndpoint->transportAddress.address[0],
                    pConnectEndpoint->transportAddress.address[1],
                    pConnectEndpoint->transportAddress.address[2],
                    pConnectEndpoint->transportAddress.address[3] ) );
        ret = ICE_CONTROLLER_RESULT_FAIL_SOCKET_NTOP;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Find a free socket context. */
        if( pCtx->socketsContextsCount < ICE_CONTROLLER_MAX_LOCAL_CANDIDATE_COUNT )
        {
            pSocketContext = &pCtx->socketsContexts[ pCtx->socketsContextsCount++ ];
        }
        else
        {
            LogWarn( ( "No socket context available for ice controller. Current number: %u", pCtx->socketsContextsCount ) );
            ret = ICE_CONTROLLER_RESULT_NO_SOCKET_CONTEXT_AVAILABLE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        memset( &credentials, 0, sizeof( NetworkCredentials_t ) );
        if( pCtx->rootCaPathLength > 0 )
        {
            credentials.pRootCaPath = ( const uint8_t * ) pCtx->rootCaPath;
            credentials.rootCaPathLength = pCtx->rootCaPathLength;
        }

        if( pCtx->rootCaPemLength > 0 )
        {
            credentials.pRootCa = ( const uint8_t * ) pCtx->rootCaPem;
            credentials.rootCaSize = pCtx->rootCaPemLength;
        }

        credentials.disableSni = pdTRUE;
        pSocketContext->tlsSession.xTlsNetworkContext.pParams = &pSocketContext->tlsSession.xTlsTransportParams;

        LogInfo( ( "Establishing a TLS session with %s:%d.",
                   pRemoteIpPos,
                   pConnectEndpoint->transportAddress.port ) );

        /* Attempt to create a server-authenticated TLS connection. */
        xNetworkStatus = TLS_FreeRTOS_Connect( &pSocketContext->tlsSession.xTlsNetworkContext,
                                               pRemoteIpPos,
                                               pConnectEndpoint->transportAddress.port,
                                               &credentials,
                                               0U,
                                               0U,
                                               TLS_CONNECT_NON_BLOCKING_HANDSHAKE );

        if( xNetworkStatus == TLS_TRANSPORT_HANDSHAKE_IN_PROGRESS )
        {
            LogDebug( ( "Connection in-progress with TLS/TCP TURN server..." ) );
            ret = ICE_CONTROLLER_RESULT_CONNECTION_IN_PROGRESS;
        }
        else if( xNetworkStatus != TLS_TRANSPORT_SUCCESS )
        {
            LogError( ( "Connection with TLS/TCP TURN server failed with return %d", xNetworkStatus ) );
            pCtx->socketsContextsCount--;
            pSocketContext->socketFd = -1;
            ret = ICE_CONTROLLER_RESULT_FAIL_SOCKET_CONNECT;
        }
        else
        {
            LogInfo( ( "Connected to TLS/TCP TURN server successfully" ) );
        }
    }

    if( ( ret == ICE_CONTROLLER_RESULT_OK ) ||
        ( ret == ICE_CONTROLLER_RESULT_CONNECTION_IN_PROGRESS ) )
    {
        pSocketContext->socketFd = TLS_FreeRTOS_GetSocketFd( &pSocketContext->tlsSession.xTlsNetworkContext );

        setsockopt( pSocketContext->socketFd, SOL_SOCKET, SO_SNDBUF, &sendBufferSize, sizeof( sendBufferSize ) );
        setsockopt( pSocketContext->socketFd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof( struct timeval ) );
        setsockopt( pSocketContext->socketFd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof( struct timeval ) );

        pSocketContext->socketType = ICE_CONTROLLER_SOCKET_TYPE_TLS;
        *ppOutSocketContext = pSocketContext;
    }

    return ret;
}

static IceControllerResult_t CreateSocketContext( IceControllerContext_t * pCtx,
                                                  uint16_t family,
                                                  IceEndpoint_t * pBindEndpoint,
                                                  IceEndpoint_t * pConnectEndpoint,
                                                  IceSocketProtocol_t protocol,
                                                  IceControllerSocketContext_t ** ppOutSocketContext )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    uint8_t isLocked = 0;

    if( ( pCtx == NULL ) || ( ppOutSocketContext == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, ppOutSocketContext: %p", pCtx, ppOutSocketContext ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( xSemaphoreTake( pCtx->socketMutex, portMAX_DELAY ) == pdTRUE )
        {
            isLocked = 1;
        }
        else
        {
            LogError( ( "Failed to lock socket mutex." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( protocol == ICE_SOCKET_PROTOCOL_UDP )
        {
            ret = CreateSocketContextUdp( pCtx,
                                          family,
                                          pBindEndpoint,
                                          pConnectEndpoint,
                                          protocol,
                                          ppOutSocketContext );
        }
        else if( protocol == ICE_SOCKET_PROTOCOL_TCP )
        {
            ret = CreateSocketContextTcp( pCtx,
                                          family,
                                          pBindEndpoint,
                                          pConnectEndpoint,
                                          protocol,
                                          ppOutSocketContext );
        }
        else
        {
            LogError( ( "Unknown socket protocol: %d", protocol ) );
            ret = ICE_CONTROLLER_RESULT_INVALID_PROTOCOL;
        }
    }

    if( isLocked != 0 )
    {
        xSemaphoreGive( pCtx->socketMutex );
    }

    return ret;
}

static IceControllerResult_t SendSocketPacket( IceControllerSocketContext_t * pSocketContext,
                                               const uint8_t * pBuffer,
                                               size_t length,
                                               int flags,
                                               struct sockaddr * pDestinationAddress,
                                               socklen_t addressLength,
                                               IceEndpoint_t * pDestinationEndpoint )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    int sentBytes, sendTotalBytes = 0;
    uint32_t totalDelayMs = 0;
    #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
    char ipBuffer[ INET_ADDRSTRLEN ];
    #endif /* #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE  */

    while( sendTotalBytes < length )
    {
        if( pSocketContext->socketType == ICE_CONTROLLER_SOCKET_TYPE_UDP )
        {
            sentBytes = sendto( pSocketContext->socketFd,
                                pBuffer + sendTotalBytes,
                                length - sendTotalBytes,
                                flags,
                                pDestinationAddress,
                                addressLength );
        }
        else if( pSocketContext->socketType == ICE_CONTROLLER_SOCKET_TYPE_TLS )
        {
            sentBytes = TLS_FreeRTOS_send( &pSocketContext->tlsSession.xTlsNetworkContext,
                                           pBuffer + sendTotalBytes,
                                           length - sendTotalBytes );
        }
        else
        {
            /* The socket type is checked before invoking this function, so this condition should never happen. */
            LogError( ( "Fail to send because of unknown socket type %d", pSocketContext->socketType ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_SOCKET_SENDTO;
            break;
        }

        if( sentBytes < 0 )
        {
            if( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) )
            {
                /* Just retry for these kinds of errno. */
            }
            else if( ( errno == ENOMEM ) || ( errno == ENOSPC ) || ( errno == ENOBUFS ) )
            {
                vTaskDelay( pdMS_TO_TICKS( ICE_CONTROLLER_RESEND_DELAY_MS ) );
                totalDelayMs += ICE_CONTROLLER_RESEND_DELAY_MS;

                if( ICE_CONTROLLER_RESEND_TIMEOUT_MS <= totalDelayMs )
                {
                    LogWarn( ( "Fail to send before timeout: %dms", ICE_CONTROLLER_RESEND_TIMEOUT_MS ) );
                    ret = ICE_CONTROLLER_RESULT_FAIL_SOCKET_SENDTO;
                    break;
                }
            }
            else
            {
                LogWarn( ( "Failed to send to socket fd: %d error, errno(%d): %s", pSocketContext->socketFd, errno, strerror( errno ) ) );
                LogVerbose( ( "Source family: %d, IP:port: %s:%u",
                              pSocketContext->pLocalCandidate->endpoint.transportAddress.family,
                              IceControllerNet_LogIpAddressInfo( &pSocketContext->pLocalCandidate->endpoint, ipBuffer, sizeof( ipBuffer ) ),
                              pSocketContext->pLocalCandidate->endpoint.transportAddress.port ) );

                LogVerbose( ( "Dest family: %d, IP:port: %s:%u",
                              pDestinationEndpoint->transportAddress.family,
                              IceControllerNet_LogIpAddressInfo( pDestinationEndpoint, ipBuffer, sizeof( ipBuffer ) ),
                              pDestinationEndpoint->transportAddress.port ) );
                ret = ICE_CONTROLLER_RESULT_FAIL_SOCKET_SENDTO;
                break;
            }
        }
        else
        {
            sendTotalBytes += sentBytes;
        }
    }

    return ret;
}

void IceControllerNet_FreeSocketContext( IceControllerContext_t * pCtx,
                                         IceControllerSocketContext_t * pSocketContext )
{
    TlsTransportStatus_t retTlsTransport;

    if( pSocketContext && ( pSocketContext->socketFd != -1 ) )
    {
        if( xSemaphoreTake( pCtx->socketMutex, portMAX_DELAY ) == pdTRUE )
        {
            if( pSocketContext->socketType == ICE_CONTROLLER_SOCKET_TYPE_TLS )
            {
                retTlsTransport = TLS_FreeRTOS_Disconnect( &pSocketContext->tlsSession.xTlsNetworkContext );
                if( retTlsTransport != TLS_TRANSPORT_SUCCESS )
                {
                    LogWarn( ( "Fail to disconnect TLS session with return %d", retTlsTransport ) );
                }
            }

            close( pSocketContext->socketFd );
            pSocketContext->socketFd = -1;
            pSocketContext->state = ICE_CONTROLLER_SOCKET_CONTEXT_STATE_NONE;

            xSemaphoreGive( pCtx->socketMutex );
        }
        else
        {
            LogError( ( "Failed to lock socket mutex." ) );
        }
    }
}

static void AddHostCandidate( IceControllerContext_t * pCtx,
                              IceEndpoint_t * pLocalIceEndpoint )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    IceCandidate_t * pCandidate;
    IceControllerSocketContext_t * pSocketContext;
    IceControllerCallbackContent_t localCandidateReadyContent;
    int32_t retLocalCandidateReady;
    #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
    char ipBuffer[ INET_ADDRSTRLEN ];
    #endif /* #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE  */

    ret = CreateSocketContext( pCtx, pLocalIceEndpoint->transportAddress.family, pLocalIceEndpoint, NULL, ICE_SOCKET_PROTOCOL_UDP, &pSocketContext );

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
        {
            iceResult = Ice_AddHostCandidate( &pCtx->iceContext, pLocalIceEndpoint );
            xSemaphoreGive( pCtx->iceMutex );

            if( iceResult != ICE_RESULT_OK )
            {
                /* Free resource that already created. */
                LogError( ( "Ice_AddHostCandidate fail, result: %d", iceResult ) );
                IceControllerNet_FreeSocketContext( pCtx, pSocketContext );
                ret = ICE_CONTROLLER_RESULT_FAIL_ADD_HOST_CANDIDATE;
            }
        }
        else
        {
            LogError( ( "Failed to add host candidate: mutex lock acquisition." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        pCandidate = &( pCtx->iceContext.pLocalCandidates[ pCtx->iceContext.numLocalCandidates - 1 ] );
        if( pCtx->onIceEventCallbackFunc )
        {
            localCandidateReadyContent.iceControllerCallbackContent.localCandidateReadyMsg.pLocalCandidate = pCandidate;
            localCandidateReadyContent.iceControllerCallbackContent.localCandidateReadyMsg.localCandidateIndex = pCtx->candidateFoundationCounter;
            retLocalCandidateReady = pCtx->onIceEventCallbackFunc( pCtx->pOnIceEventCustomContext, ICE_CONTROLLER_CB_EVENT_LOCAL_CANDIDATE_READY, &localCandidateReadyContent );
            if( retLocalCandidateReady == 0 )
            {
                pCtx->candidateFoundationCounter++;
            }
            else
            {
                /* Free resource that already created. */
                IceControllerNet_FreeSocketContext( pCtx, pSocketContext );
                LogError( ( "Fail to send local candidate, ret: %ld.", retLocalCandidateReady ) );
                ret = ICE_CONTROLLER_RESULT_CANDIDATE_SEND_FAIL;
            }
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        IceControllerNet_UpdateSocketContext( pCtx, pSocketContext, ICE_CONTROLLER_SOCKET_CONTEXT_STATE_READY, pCandidate, NULL, NULL );

        LogInfo( ( "Created host candidate with fd %d, ID: 0x%04x",
                   pSocketContext->socketFd,
                   pCandidate->candidateId ) );
        LogVerbose( ( "host candidate's local IP/port: %s/%d",
                      IceControllerNet_LogIpAddressInfo( pLocalIceEndpoint, ipBuffer, sizeof( ipBuffer ) ),
                      pLocalIceEndpoint->transportAddress.port ) );
    }
}

static void AddSrflxCandidate( IceControllerContext_t * pCtx,
                               IceEndpoint_t * pLocalIceEndpoint )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    uint32_t i;
    IceControllerSocketContext_t * pSocketContext = NULL;
    #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
    char ipBuffer[ INET_ADDRSTRLEN ];
    #endif /* #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE  */
    IceControllerResult_t dnsResult;

    for( i = 0; i < pCtx->iceServersCount; i++ )
    {
        /* Reset ret for every round. */
        ret = ICE_CONTROLLER_RESULT_OK;

        if( pCtx->iceServers[ i ].serverType != ICE_CONTROLLER_ICE_SERVER_TYPE_STUN )
        {
            /* Not STUN server, no need to create srflx candidate for this server. */
            continue;
        }

        dnsResult = IceControllerNet_DnsLookUp( pCtx->iceServers[ i ].url,
                                                &pCtx->iceServers[ i ].iceEndpoint.transportAddress );
        if( dnsResult != ICE_CONTROLLER_RESULT_OK )
        {
            LogWarn( ( "Fail to get the DNS result of STUN server: %.*s",
                       ( int ) pCtx->iceServers[ i ].urlLength,
                       pCtx->iceServers[ i ].url ) );
            continue;
        }

        /* Only support IPv4 STUN for now. */
        if( ( pCtx->iceServers[ i ].iceEndpoint.transportAddress.family == STUN_ADDRESS_IPv4 ) &&
            ( pLocalIceEndpoint->transportAddress.family == pCtx->iceServers[ i ].iceEndpoint.transportAddress.family ) )
        {
            ret = CreateSocketContext( pCtx, pLocalIceEndpoint->transportAddress.family, pLocalIceEndpoint, NULL, ICE_SOCKET_PROTOCOL_UDP, &pSocketContext );
            if( ( ret != ICE_CONTROLLER_RESULT_OK ) ||
                ( pSocketContext == NULL ) )
            {
                LogError( ( "Fail to create socket context for srflx candidate." ) );
                continue;
            }
        }
        else
        {
            LogWarn( ( "STUN server's IP family is not supported: %.*s",
                       ( int ) pCtx->iceServers[ i ].urlLength,
                       pCtx->iceServers[ i ].url ) );
            continue;
        }

        if( ret == ICE_CONTROLLER_RESULT_OK )
        {
            if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
            {
                iceResult = Ice_AddServerReflexiveCandidate( &pCtx->iceContext,
                                                             pLocalIceEndpoint );
                xSemaphoreGive( pCtx->iceMutex );

                if( iceResult != ICE_RESULT_OK )
                {
                    /* Free resource that already created. */
                    LogError( ( "Ice_AddServerReflexiveCandidate fail, result: %d", iceResult ) );
                    IceControllerNet_FreeSocketContext( pCtx, pSocketContext );
                    ret = ICE_CONTROLLER_RESULT_FAIL_ADD_HOST_CANDIDATE;
                    break;
                }
            }
            else
            {
                LogError( ( "Failed to add server reflexive candidate: mutex lock acquisition." ) );
                ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
            }
        }

        if( ret == ICE_CONTROLLER_RESULT_OK )
        {
            IceControllerNet_UpdateSocketContext( pCtx, pSocketContext, ICE_CONTROLLER_SOCKET_CONTEXT_STATE_CREATE, &pCtx->iceContext.pLocalCandidates[ pCtx->iceContext.numLocalCandidates - 1 ], NULL, &pCtx->iceServers[ i ] );

            LogInfo( ( "Created srflx candidate with fd %d, ID: 0x%04x",
                       pSocketContext->socketFd,
                       pCtx->iceContext.pLocalCandidates[ pCtx->iceContext.numLocalCandidates - 1 ].candidateId ) );

            LogVerbose( ( "srflx candidate's local IP/port: %s/%d",
                          IceControllerNet_LogIpAddressInfo( pLocalIceEndpoint, ipBuffer, sizeof( ipBuffer ) ),
                          pLocalIceEndpoint->transportAddress.port ) );
            pCtx->metrics.pendingSrflxCandidateNum++;
        }
    }
}

static void AddRelayCandidates( IceControllerContext_t * pCtx )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    uint32_t i;
    IceControllerSocketContext_t * pSocketContext = NULL;
    #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
    char ipBuffer[ INET_ADDRSTRLEN ];
    #endif /* #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE  */
    IceControllerResult_t dnsResult;

    if( pCtx == NULL )
    {
        LogError( ( "Invalid input, pCtx: %p", pCtx ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Loop through all ICE server configs and start allocate TURN with UDP and TLS TURN servers. */
        for( i = 0; i < pCtx->iceServersCount; i++ )
        {
            /* Reset ret for every round. */
            ret = ICE_CONTROLLER_RESULT_OK;

            if( ( pCtx->iceServers[i].serverType != ICE_CONTROLLER_ICE_SERVER_TYPE_TURN ) &&
                ( pCtx->iceServers[i].serverType != ICE_CONTROLLER_ICE_SERVER_TYPE_TURNS ) )
            {
                /* Skip STUN servers. */
                continue;
            }
            else if( ( pCtx->iceServers[i].protocol != ICE_SOCKET_PROTOCOL_UDP ) &&
                     ( pCtx->iceServers[i].protocol != ICE_SOCKET_PROTOCOL_TCP ) )
            {
                LogInfo( ( "Unknown TURN Server, protocol: %d, Server URL: %.*s",
                           pCtx->iceServers[i].protocol,
                           ( int ) pCtx->iceServers[i].urlLength,
                           pCtx->iceServers[i].url ) );
                continue;
            }
            else if( ( pCtx->iceServers[i].protocol == ICE_SOCKET_PROTOCOL_UDP ) &&
                     ( pCtx->iceServers[i].serverType != ICE_CONTROLLER_ICE_SERVER_TYPE_TURN ) )
            {
                /* For now we do not support DTLS connection over TURN server. */
                LogInfo( ( "Only pure UDP TURN server is supported, serverType: %d, Server URL: %.*s",
                           pCtx->iceServers[i].serverType,
                           ( int ) pCtx->iceServers[i].urlLength,
                           pCtx->iceServers[i].url ) );
                continue;
            }
            else if( ( pCtx->iceServers[i].protocol == ICE_SOCKET_PROTOCOL_TCP ) &&
                     ( pCtx->iceServers[i].serverType != ICE_CONTROLLER_ICE_SERVER_TYPE_TURNS ) )
            {
                /* For now we only support TLS connection over TURN server. */
                LogInfo( ( "Only TLS/TCP TURN server is supported, serverType: %d, Server URL: %.*s",
                           pCtx->iceServers[i].serverType,
                           ( int ) pCtx->iceServers[i].urlLength,
                           pCtx->iceServers[i].url ) );
                continue;
            }
            else
            {
                LogInfo( ( "Creating connection with TURN server %.*s, protocol: %s.",
                           ( int ) pCtx->iceServers[i].urlLength,
                           pCtx->iceServers[i].url,
                           pCtx->iceServers[i].protocol == ICE_SOCKET_PROTOCOL_UDP ? "UDP" : "TLS" ) );
            }

            dnsResult = IceControllerNet_DnsLookUp( pCtx->iceServers[ i ].url,
                                                    &pCtx->iceServers[ i ].iceEndpoint.transportAddress );
            if( dnsResult != ICE_CONTROLLER_RESULT_OK )
            {
                LogWarn( ( "Fail to get the DNS result of STUN server: %.*s",
                           ( int ) pCtx->iceServers[ i ].urlLength,
                           pCtx->iceServers[ i ].url ) );
                continue;
            }

            ret = CreateSocketContext( pCtx, STUN_ADDRESS_IPv4, NULL, &pCtx->iceServers[i].iceEndpoint, pCtx->iceServers[i].protocol, &pSocketContext );

            if( ret == ICE_CONTROLLER_RESULT_OK )
            {
                if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
                {
                    iceResult = Ice_AddRelayCandidate( &pCtx->iceContext, &pCtx->iceServers[i].iceEndpoint, pCtx->iceServers[i].userName, pCtx->iceServers[i].userNameLength, pCtx->iceServers[i].password, pCtx->iceServers[i].passwordLength );
                    xSemaphoreGive( pCtx->iceMutex );

                    if( iceResult != ICE_RESULT_OK )
                    {
                        /* Free resource that already created. */
                        LogError( ( "Ice_AddRelayCandidate fail, result: %d", iceResult ) );
                        IceControllerNet_FreeSocketContext( pCtx, pSocketContext );
                        ret = ICE_CONTROLLER_RESULT_FAIL_ADD_RELAY_CANDIDATE;
                        break;
                    }
                }
                else
                {
                    LogError( ( "Failed to add relay candidate: mutex lock acquisition." ) );
                    ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
                }
            }

            if( ret == ICE_CONTROLLER_RESULT_OK )
            {
                IceControllerNet_UpdateSocketContext( pCtx,
                                                      pSocketContext,
                                                      ICE_CONTROLLER_SOCKET_CONTEXT_STATE_CREATE,
                                                      &( pCtx->iceContext.pLocalCandidates[ pCtx->iceContext.numLocalCandidates - 1 ] ),
                                                      NULL,
                                                      &( pCtx->iceServers[ i ] ) );

                LogInfo( ( "Created relay candidate with fd %d, ID: 0x%04x",
                           pSocketContext->socketFd,
                           pCtx->iceContext.pLocalCandidates[ pCtx->iceContext.numLocalCandidates - 1 ].candidateId ) );
                LogVerbose( ( "relay candidate's local IP/port: %s/%d",
                              IceControllerNet_LogIpAddressInfo( &pCtx->iceServers[ i ].iceEndpoint, ipBuffer, sizeof( ipBuffer ) ),
                              pCtx->iceServers[ i ].iceEndpoint.transportAddress.port ) );

                pCtx->metrics.pendingRelayCandidateNum++;
            }
            else if( ret == ICE_CONTROLLER_RESULT_CONNECTION_IN_PROGRESS )
            {
                IceControllerNet_UpdateSocketContext( pCtx,
                                                      pSocketContext,
                                                      ICE_CONTROLLER_SOCKET_CONTEXT_STATE_CONNECTION_IN_PROGRESS,
                                                      NULL,
                                                      NULL,
                                                      &( pCtx->iceServers[ i ] ) );

                LogVerbose( ( "Connection in-progress with TURN server for socket fd %d...", pSocketContext->socketFd ) );

                pCtx->metrics.pendingRelayCandidateNum++;
            }
        }
    }
}

static IceControllerResult_t SendBindingResponse( IceControllerContext_t * pCtx,
                                                  IceControllerSocketContext_t * pSocketContext,
                                                  IceCandidatePair_t * pCandidatePair,
                                                  uint8_t * pTransactionIdBuffer )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    uint8_t sentStunBuffer[ ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE ];
    size_t sentStunBufferLength = ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE;
    IceEndpoint_t * pDestEndpoint = NULL;

    if( ( pCtx == NULL ) ||
        ( pSocketContext == NULL ) ||
        ( pCandidatePair == NULL ) ||
        ( pTransactionIdBuffer == NULL ) )
    {
        LogWarn( ( "Invalid input to send binding response, pCtx: %p, pSocketContext: %p, pCandidatePair: %p, pTransactionIdBuffer: %p", pCtx, pSocketContext, pCandidatePair, pTransactionIdBuffer ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
        {
            iceResult = Ice_CreateResponseForRequest( &pCtx->iceContext,
                                                      pCandidatePair,
                                                      pTransactionIdBuffer,
                                                      sentStunBuffer,
                                                      &sentStunBufferLength );
            xSemaphoreGive( pCtx->iceMutex );

            if( iceResult != ICE_RESULT_OK )
            {
                LogWarn( ( "Unable to create STUN binding response, result: %d", iceResult ) );
                ret = ICE_CONTROLLER_RESULT_FAIL_SEND_BIND_RESPONSE;
            }
        }
        else
        {
            LogError( ( "Failed to create binding response: mutex lock acquisition." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        IceControllerNet_LogStunPacket( sentStunBuffer, sentStunBufferLength );

        if( pSocketContext->pLocalCandidate->candidateType == ICE_CANDIDATE_TYPE_RELAY )
        {
            pDestEndpoint = &( pSocketContext->pIceServer->iceEndpoint );
        }
        else
        {
            pDestEndpoint = &pCandidatePair->pRemoteCandidate->endpoint;
        }

        ret = IceControllerNet_SendPacket( pCtx, pSocketContext, pDestEndpoint, sentStunBuffer, sentStunBufferLength );
        if( ret != ICE_CONTROLLER_RESULT_OK )
        {
            LogWarn( ( "Unable to send STUN response, result: %d", ret ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_SEND_BIND_RESPONSE;
        }
        else
        {
            LogDebug( ( "Sending STUN bind response back to remote, local/remote candidate ID: 0x%04x / 0x%04x",
                        pCandidatePair->pLocalCandidate->candidateId,
                        pCandidatePair->pRemoteCandidate->candidateId ) );
        }
    }

    return ret;
}

static IceControllerResult_t SendNominationRequest( IceControllerContext_t * pCtx,
                                                    IceControllerSocketContext_t * pSocketContext,
                                                    IceCandidatePair_t * pCandidatePair,
                                                    uint8_t * pTransactionIdBuffer )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    uint8_t sentStunBuffer[ ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE ];
    size_t sentStunBufferLength = ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE;
    IceEndpoint_t * pDestEndpoint = NULL;
    uint64_t currentTimeSeconds = NetworkingUtils_GetCurrentTimeSec( NULL );

    if( ( pCtx == NULL ) ||
        ( pSocketContext == NULL ) ||
        ( pCandidatePair == NULL ) ||
        ( pTransactionIdBuffer == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pSocketContext: %p, pCandidatePair: %p, pTransactionIdBuffer: %p",
                    pCtx, pSocketContext, pCandidatePair, pTransactionIdBuffer ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
        {
            iceResult = Ice_CreateNextPairRequest( &pCtx->iceContext,
                                                   pCandidatePair,
                                                   currentTimeSeconds,
                                                   sentStunBuffer,
                                                   &sentStunBufferLength );
            xSemaphoreGive( pCtx->iceMutex );

            if( iceResult != ICE_RESULT_OK )
            {
                LogWarn( ( "Unable to create STUN binding response, result: %d", iceResult ) );
                ret = ICE_CONTROLLER_RESULT_FAIL_SEND_BIND_RESPONSE;
            }
        }
        else
        {
            LogError( ( "Failed to create binding response: mutex lock acquisition." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        IceControllerNet_LogStunPacket( sentStunBuffer, sentStunBufferLength );

        if( pSocketContext->pLocalCandidate->candidateType == ICE_CANDIDATE_TYPE_RELAY )
        {
            pDestEndpoint = &( pSocketContext->pIceServer->iceEndpoint );
        }
        else
        {
            pDestEndpoint = &pCandidatePair->pRemoteCandidate->endpoint;
        }

        ret = IceControllerNet_SendPacket( pCtx, pSocketContext, pDestEndpoint, sentStunBuffer, sentStunBufferLength );
        if( ret != ICE_CONTROLLER_RESULT_OK )
        {
            LogWarn( ( "Unable to send STUN nomination request, result: %d", ret ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_SEND_NOMINATION_REQUEST;
        }
        else
        {
            LogDebug( ( "Sent STUN nomination request back to remote, local/remote candidate ID: 0x%04x / 0x%04x",
                        pCandidatePair->pLocalCandidate->candidateId,
                        pCandidatePair->pRemoteCandidate->candidateId ) );
        }
    }

    return ret;
}

static IceControllerResult_t CheckNomination( IceControllerContext_t * pCtx,
                                              IceControllerSocketContext_t * pSocketContext,
                                              IceCandidatePair_t * pCandidatePair )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
        char ipBuffer[ INET_ADDRSTRLEN ];
        char ipBuffer2[ INET_ADDRSTRLEN ];
    #endif /* #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE */

    if( ( pCtx == NULL ) ||
        ( pSocketContext == NULL ) ||
        ( pCandidatePair == NULL ) )
    {
        LogWarn( ( "Invalid input, pCtx: %p, pSocketContext: %p, pCandidatePair: %p",
                   pCtx, pSocketContext, pCandidatePair ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( ( pCandidatePair->state == ICE_CANDIDATE_PAIR_STATE_SUCCEEDED ) &&
            ( pCtx->pNominatedSocketContext == NULL ) )
        {
            #if METRIC_PRINT_ENABLED
            Metric_EndEvent( METRIC_EVENT_ICE_FIND_P2P_CONNECTION );
            #endif
            LogInfo( ( "Found nomination pair, local/remote candidate ID: 0x%04x / 0x%04x",
                       pCandidatePair->pLocalCandidate->candidateId,
                       pCandidatePair->pRemoteCandidate->candidateId ) );

            LogVerbose( ( "Candidiate pair is nominated, local IP/port: %s/%u, remote IP/port: %s/%u",
                          IceControllerNet_LogIpAddressInfo( &pCandidatePair->pLocalCandidate->endpoint, ipBuffer, sizeof( ipBuffer ) ), pCandidatePair->pLocalCandidate->endpoint.transportAddress.port,
                          IceControllerNet_LogIpAddressInfo( &pCandidatePair->pRemoteCandidate->endpoint, ipBuffer2, sizeof( ipBuffer2 ) ), pCandidatePair->pRemoteCandidate->endpoint.transportAddress.port ) );

            ret = ICE_CONTROLLER_RESULT_FOUND_CONNECTION;
        }
    }

    return ret;
}

IceControllerResult_t IceControllerNet_ConvertIpString( const char * pIpAddr,
                                                        size_t ipAddrLength,
                                                        IceEndpoint_t * pDestinationIceEndpoint )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    char ipAddress[ ICE_CONTROLLER_IP_ADDR_STRING_BUFFER_LENGTH + 1 ];

    if( ipAddrLength > ICE_CONTROLLER_IP_ADDR_STRING_BUFFER_LENGTH )
    {
        LogWarn( ( "invalid IP address detected, IP: %.*s",
                   ( int ) ipAddrLength, pIpAddr ) );
        ret = ICE_CONTROLLER_RESULT_IP_BUFFER_TOO_SMALL;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        memcpy( ipAddress, pIpAddr, ipAddrLength );
        ipAddress[ ipAddrLength ] = '\0';

        if( inet_pton( AF_INET, ipAddress, pDestinationIceEndpoint->transportAddress.address ) == 1 )
        {
            pDestinationIceEndpoint->transportAddress.family = STUN_ADDRESS_IPv4;
        }
        else if( inet_pton( AF_INET6, ipAddress, pDestinationIceEndpoint->transportAddress.address ) == 1 )
        {
            pDestinationIceEndpoint->transportAddress.family = STUN_ADDRESS_IPv6;
        }
        else
        {
            ret = ICE_CONTROLLER_RESULT_INVALID_IP_ADDR;
        }
    }

    return ret;
}

IceControllerResult_t IceControllerNet_Htons( uint16_t port,
                                              uint16_t * pOutPort )
{
    *pOutPort = htons( port );

    return ICE_CONTROLLER_RESULT_OK;
}

IceControllerResult_t IceControllerNet_SendPacket( IceControllerContext_t * pCtx,
                                                   IceControllerSocketContext_t * pSocketContext,
                                                   IceEndpoint_t * pRemoteEndpoint,
                                                   const uint8_t * pBuffer,
                                                   size_t bufferLength )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    struct sockaddr * pDestinationAddress = NULL;
    struct sockaddr_in ipv4Address;
    struct sockaddr_in6 ipv6Address;
    socklen_t addressLength = 0;
    uint8_t isLocked = 0;

    if( ( pCtx == NULL ) || ( pSocketContext == NULL ) || ( pRemoteEndpoint == NULL ) || ( pBuffer == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pSocketContext: %p, pRemoteEndpoint: %p, pBuffer: %p",
                    pCtx, pSocketContext, pRemoteEndpoint, pBuffer ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( xSemaphoreTake( pCtx->socketMutex, portMAX_DELAY ) == pdTRUE )
        {
            isLocked = 1;
        }
        else
        {
            LogError( ( "Failed to lock socket mutex." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( pSocketContext->state == ICE_CONTROLLER_SOCKET_CONTEXT_STATE_NONE )
        {
            /* The socket context has been closed, skip sending process. */
            LogDebug( ( "The socket has been close, skip sending." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_SOCKET_CONTEXT_ALREADY_CLOSED;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Set socket destination address, including IP type (v4/v6), IP address and port. */
        if( pSocketContext->pLocalCandidate->endpoint.transportAddress.family != pRemoteEndpoint->transportAddress.family )
        {
            LogWarn( ( "The sending IP family: %d is different from receiving IP family: %d",
                       pSocketContext->pLocalCandidate->endpoint.transportAddress.family,
                       pRemoteEndpoint->transportAddress.family ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_SOCKET_SENDTO;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( pRemoteEndpoint->transportAddress.family == STUN_ADDRESS_IPv4 )
        {
            memset( &ipv4Address, 0, sizeof( ipv4Address ) );
            ipv4Address.sin_family = AF_INET;
            ipv4Address.sin_port = htons( pRemoteEndpoint->transportAddress.port );
            memcpy( &ipv4Address.sin_addr, pRemoteEndpoint->transportAddress.address, STUN_IPV4_ADDRESS_SIZE );

            pDestinationAddress = ( struct sockaddr * ) &ipv4Address;
            addressLength = sizeof( ipv4Address );
        }
        else
        {
            memset( &ipv6Address, 0, sizeof( ipv6Address ) );
            ipv6Address.sin6_family = AF_INET6;
            ipv6Address.sin6_port = htons( pRemoteEndpoint->transportAddress.port );
            memcpy( &ipv6Address.sin6_addr, pRemoteEndpoint->transportAddress.address, STUN_IPV6_ADDRESS_SIZE );

            pDestinationAddress = ( struct sockaddr * ) &ipv6Address;
            addressLength = sizeof( ipv6Address );
        }
    }

    /* Send data */
    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( ( pSocketContext->socketType == ICE_CONTROLLER_SOCKET_TYPE_UDP ) ||
            ( pSocketContext->socketType == ICE_CONTROLLER_SOCKET_TYPE_TLS ) )
        {
            ret = SendSocketPacket( pSocketContext, pBuffer, bufferLength, 0, pDestinationAddress, addressLength, pRemoteEndpoint );
        }
        else
        {
            LogError( ( "Internal error, invalid socket type %d", pSocketContext->socketType ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_SOCKET_TYPE;
        }
    }

    if( isLocked != 0 )
    {
        xSemaphoreGive( pCtx->socketMutex );
    }

    if( ret == ICE_CONTROLLER_RESULT_FAIL_SOCKET_SENDTO )
    {
        /*
         * Socket read error detected.
         * This typically indicates the remote peer closed the connection or WiFi disconnection.
         * Action required: Close the local socket to properly terminate the connection.
         */
        ( void ) Ice_CloseCandidate( &pCtx->iceContext, pSocketContext->pLocalCandidate );
        IceControllerNet_FreeSocketContext( pCtx, pSocketContext );

        if( pSocketContext == pCtx->pNominatedSocketContext )
        {
            /* Disconnecting nominated socket connection, closing. */
            LogWarn( ( "Unable to send packet through nominated socket, closing session: %.*s",
                       ( int ) pCtx->iceContext.creds.combinedUsernameLength,
                       pCtx->iceContext.creds.pCombinedUsername ) );

            /* Notify peer connection for closing the connection. */
            if( pCtx->onIceEventCallbackFunc )
            {
                pCtx->onIceEventCallbackFunc( pCtx->pOnIceEventCustomContext,
                                              ICE_CONTROLLER_CB_EVENT_ICE_CLOSE_NOTIFY,
                                              NULL );
                
                /* Re-set the timer. */
                IceController_UpdateTimerInterval( pCtx,
                                                   ICE_CONTROLLER_CLOSING_INTERVAL_MS );
            }
            else
            {
                LogError( ( "There is no ICE event callback function set." ) );
            }
        }
    }

    return ret;
}

void IceControllerNet_AddLocalCandidates( IceControllerContext_t * pCtx )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    uint32_t i;

    if( pCtx == NULL )
    {
        LogError( ( "Invalid input, pCtx: %p", pCtx ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Collect information from local network interfaces. */
        pCtx->localIceEndpointsCount = ICE_CONTROLLER_MAX_LOCAL_CANDIDATE_COUNT;
        GetLocalIPAdresses( pCtx->localEndpoints, &pCtx->localIceEndpointsCount );

        /* Start gathering local candidates. */
        for( i = 0; i < pCtx->localIceEndpointsCount; i++ )
        {
            if( ICE_CONTROLLER_IS_NAT_CONFIG_SET( pCtx, ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_SEND_HOST ) )
            {
                #if METRIC_PRINT_ENABLED
                Metric_StartEvent( METRIC_EVENT_ICE_GATHER_HOST_CANDIDATES );
                #endif
                AddHostCandidate( pCtx, &pCtx->localEndpoints[i] );
                #if METRIC_PRINT_ENABLED
                Metric_EndEvent( METRIC_EVENT_ICE_GATHER_HOST_CANDIDATES );
                #endif
            }

            if( ICE_CONTROLLER_IS_NAT_CONFIG_SET( pCtx, ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_SEND_SRFLX ) )
            {
                #if METRIC_PRINT_ENABLED
                Metric_StartEvent( METRIC_EVENT_ICE_GATHER_SRFLX_CANDIDATES );
                #endif
                AddSrflxCandidate( pCtx, &pCtx->localEndpoints[i] );
            }
        }

        if( ICE_CONTROLLER_IS_NAT_CONFIG_SET( pCtx, ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_SEND_RELAY ) )
        {
            #if METRIC_PRINT_ENABLED
                Metric_StartEvent( METRIC_EVENT_ICE_GATHER_RELAY_CANDIDATES );
            #endif
            AddRelayCandidates( pCtx );
        }
    }
}

IceControllerResult_t IceControllerNet_ExecuteTlsHandshake( IceControllerContext_t * pCtx,
                                                            IceControllerSocketContext_t * pSocketContext,
                                                            uint8_t isIceLockTakenBeforeCall )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    TlsTransportStatus_t transportResult = TLS_TRANSPORT_SUCCESS;
    IceResult_t iceResult = ICE_RESULT_OK;

    if( ( pCtx == NULL ) || ( pSocketContext == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pSocketContext: %p",
                    pCtx, pSocketContext ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }
    else if( pSocketContext->socketType != ICE_CONTROLLER_SOCKET_TYPE_TLS )
    {
        LogError( ( "Invalid socket type: %d", pSocketContext->socketType ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( xSemaphoreTake( pCtx->socketMutex, portMAX_DELAY ) == pdTRUE )
        {
            transportResult = TLS_FreeRTOS_ContinueHandshake( &( pSocketContext->tlsSession.xTlsNetworkContext ) );

            xSemaphoreGive( pCtx->socketMutex );
        }
        else
        {
            LogError( ( "Failed to lock socket mutex." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( transportResult == TLS_TRANSPORT_SUCCESS )
        {
            LogVerbose( ( "Connection with TURN server successful for socket fd %d", pSocketContext->socketFd ) );

            if( ( isIceLockTakenBeforeCall != 0U ) ||
                ( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE ) )
            {
                iceResult = Ice_AddRelayCandidate( &( pCtx->iceContext ),
                                                   &( pSocketContext->pIceServer->iceEndpoint ),
                                                   &( pSocketContext->pIceServer->userName[ 0 ] ),
                                                   pSocketContext->pIceServer->userNameLength,
                                                   &( pSocketContext->pIceServer->password[ 0 ] ),
                                                   pSocketContext->pIceServer->passwordLength );
                if( isIceLockTakenBeforeCall == 0U )
                {
                    xSemaphoreGive( pCtx->iceMutex );
                }
            }
            else
            {
                LogError( ( "Failed to lock ice mutex." ) );
                ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
            }

            if( ret == ICE_CONTROLLER_RESULT_OK )
            {
                if( iceResult != ICE_RESULT_OK )
                {
                    LogError( ( "Failed to created relay candidate for socket fd %d",
                                pSocketContext->socketFd ) );
                    IceControllerNet_FreeSocketContext( pCtx, pSocketContext );
                }
                else
                {
                    LogInfo( ( "Created relay candidate with fd %d, ID: 0x%04x",
                               pSocketContext->socketFd,
                               pCtx->iceContext.pLocalCandidates[ pCtx->iceContext.numLocalCandidates - 1 ].candidateId ) );

                    IceControllerNet_UpdateSocketContext( pCtx,
                                                          pSocketContext,
                                                          ICE_CONTROLLER_SOCKET_CONTEXT_STATE_CREATE,
                                                          &( pCtx->iceContext.pLocalCandidates[ pCtx->iceContext.numLocalCandidates - 1 ] ),
                                                          NULL,
                                                          pSocketContext->pIceServer );
                }
            }
        }
        else if( transportResult == TLS_TRANSPORT_HANDSHAKE_IN_PROGRESS )
        {
            LogVerbose( ( "Connection still in-progress with TURN server for socket fd %d...", pSocketContext->socketFd ) );
        }
        else
        {
            LogError( ( "Connection with TURN server failed for socket fd %d", pSocketContext->socketFd ) );
            IceControllerNet_FreeSocketContext( pCtx, pSocketContext );
        }
    }

    return ret;
}

IceControllerResult_t IceControllerNet_HandleStunPacket( IceControllerContext_t * pCtx,
                                                         IceControllerSocketContext_t * pSocketContext,
                                                         uint8_t * pReceiveBuffer,
                                                         size_t receiveBufferLength,
                                                         IceEndpoint_t * pRemoteIceEndpoint,
                                                         IceCandidatePair_t * pCandidatePair )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceHandleStunPacketResult_t iceHandleStunResult;
    uint8_t * pTransactionIdBuffer;
    int32_t retLocalCandidateReady;
    IceControllerCallbackContent_t localCandidateReadyContent;
    uint8_t sentStunBuffer[ ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE ];
    size_t sentStunBufferLength = ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE;
    IceResult_t iceResult;
    uint64_t currentTimeSeconds = NetworkingUtils_GetCurrentTimeSec( NULL );

    if( ( pCtx == NULL ) || ( pReceiveBuffer == NULL ) || ( pRemoteIceEndpoint == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pReceiveBuffer: %p, pRemoteIceEndpoint: %p",
                    pCtx, pReceiveBuffer, pRemoteIceEndpoint ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
        {
            iceHandleStunResult = Ice_HandleStunPacket( &pCtx->iceContext,
                                                        pReceiveBuffer,
                                                        ( size_t ) receiveBufferLength,
                                                        pSocketContext->pLocalCandidate,
                                                        pRemoteIceEndpoint,
                                                        currentTimeSeconds,
                                                        &pTransactionIdBuffer,
                                                        &pCandidatePair );
            xSemaphoreGive( pCtx->iceMutex );
        }
        else
        {
            LogError( ( "Failed to handle stun packet: mutex lock acquisition." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( iceHandleStunResult != ICE_HANDLE_STUN_PACKET_RESULT_NOT_STUN_PACKET )
        {
            IceControllerNet_LogStunPacket( pReceiveBuffer, receiveBufferLength );

            if( pCandidatePair != NULL )
            {
                LogDebug( ( "Receiving STUN packet, local/remote candidate ID: 0x%04x / 0x%04x",
                            pCandidatePair->pLocalCandidate->candidateId,
                            pCandidatePair->pRemoteCandidate->candidateId ) );
            }
        }

        LogVerbose( ( "Ice_HandleStunPacket return %d", iceHandleStunResult ) );

        switch( iceHandleStunResult )
        {
            case ICE_HANDLE_STUN_PACKET_RESULT_UPDATED_SERVER_REFLEXIVE_CANDIDATE_ADDRESS:
                if( pCtx->onIceEventCallbackFunc )
                {
                    /* Update socket context. */
                    IceControllerNet_UpdateSocketContext( pCtx, pSocketContext, ICE_CONTROLLER_SOCKET_CONTEXT_STATE_READY, pSocketContext->pLocalCandidate, pSocketContext->pRemoteCandidate, pSocketContext->pIceServer );

                    localCandidateReadyContent.iceControllerCallbackContent.localCandidateReadyMsg.pLocalCandidate = pSocketContext->pLocalCandidate;
                    localCandidateReadyContent.iceControllerCallbackContent.localCandidateReadyMsg.localCandidateIndex = pCtx->candidateFoundationCounter;
                    retLocalCandidateReady = pCtx->onIceEventCallbackFunc( pCtx->pOnIceEventCustomContext, ICE_CONTROLLER_CB_EVENT_LOCAL_CANDIDATE_READY, &localCandidateReadyContent );
                    if( retLocalCandidateReady == 0 )
                    {
                        pCtx->candidateFoundationCounter++;
                    }
                    else
                    {
                        /* Free resource that already created. */
                        LogWarn( ( "Fail to send server reflexive candidate to remote peer, ret: %ld.", retLocalCandidateReady ) );
                    }
                }
                else
                {
                    LogError( ( "Unable to send srflx candidate ready message." ) );
                }

                pCtx->metrics.pendingSrflxCandidateNum--;
                #if METRIC_PRINT_ENABLED
                if( pCtx->metrics.pendingSrflxCandidateNum == 0 )
                {
                    Metric_EndEvent( METRIC_EVENT_ICE_GATHER_SRFLX_CANDIDATES );
                }
                #endif
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_UPDATED_RELAY_CANDIDATE_ADDRESS:
                if( pCtx->onIceEventCallbackFunc )
                {
                    /* Update socket context. */
                    IceControllerNet_UpdateSocketContext( pCtx, pSocketContext, ICE_CONTROLLER_SOCKET_CONTEXT_STATE_READY, pSocketContext->pLocalCandidate, pSocketContext->pRemoteCandidate, pSocketContext->pIceServer );

                    localCandidateReadyContent.iceControllerCallbackContent.localCandidateReadyMsg.pLocalCandidate = pSocketContext->pLocalCandidate;
                    localCandidateReadyContent.iceControllerCallbackContent.localCandidateReadyMsg.localCandidateIndex = pCtx->candidateFoundationCounter;
                    retLocalCandidateReady = pCtx->onIceEventCallbackFunc( pCtx->pOnIceEventCustomContext, ICE_CONTROLLER_CB_EVENT_LOCAL_CANDIDATE_READY, &localCandidateReadyContent );
                    if( retLocalCandidateReady == 0 )
                    {
                        pCtx->candidateFoundationCounter++;
                    }
                    else
                    {
                        /* Free resource that already created. */
                        LogWarn( ( "Fail to send relay candidate to remote peer, ret: %ld.", retLocalCandidateReady ) );
                    }

                    pCtx->metrics.pendingRelayCandidateNum--;
                    #if METRIC_PRINT_ENABLED
                    if( pCtx->metrics.pendingRelayCandidateNum == 0 )
                    {
                        Metric_EndEvent( METRIC_EVENT_ICE_GATHER_RELAY_CANDIDATES );
                    }
                    #endif
                }
                else
                {
                    LogError( ( "Unable to send relay candidate ready message." ) );
                }
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_SEND_TRIGGERED_CHECK:
            case ICE_HANDLE_STUN_PACKET_RESULT_SEND_RESPONSE_FOR_REMOTE_REQUEST:
                ret = SendBindingResponse( pCtx, pSocketContext, pCandidatePair, pTransactionIdBuffer );

                if( ret == ICE_CONTROLLER_RESULT_OK )
                {
                    ret = CheckNomination( pCtx,
                                           pSocketContext,
                                           pCandidatePair );
                }
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_SEND_CHANNEL_BIND_REQUEST:
                if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
                {
                    iceResult = Ice_CreateNextPairRequest( &pCtx->iceContext,
                                                           pCandidatePair,
                                                           currentTimeSeconds,
                                                           sentStunBuffer,
                                                           &sentStunBufferLength );
                    xSemaphoreGive( pCtx->iceMutex );

                    if( iceResult != ICE_RESULT_OK )
                    {
                        LogWarn( ( "Unable to create channel binding message, result: %d", iceResult ) );
                    }
                    else
                    {
                        LogDebug( ( "Sending channel binding request, local/remote candidate ID: 0x%04x / 0x%04x",
                                    pCandidatePair->pLocalCandidate->candidateId,
                                    pCandidatePair->pRemoteCandidate->candidateId ) );
                        IceControllerNet_LogStunPacket( sentStunBuffer, sentStunBufferLength );

                        if( IceControllerNet_SendPacket( pCtx, pSocketContext, &( pSocketContext->pIceServer->iceEndpoint ), sentStunBuffer, sentStunBufferLength ) != ICE_CONTROLLER_RESULT_OK )
                        {
                            LogWarn( ( "Unable to send channel binding message" ) );
                        }
                    }
                }
                else
                {
                    LogError( ( "Failed to create channel binding request: mutex lock acquisition." ) );
                    ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
                }
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_SEND_CONNECTIVITY_CHECK_REQUEST:
                if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
                {
                    iceResult = Ice_CreateNextPairRequest( &pCtx->iceContext,
                                                           pCandidatePair,
                                                           currentTimeSeconds,
                                                           sentStunBuffer,
                                                           &sentStunBufferLength );
                    xSemaphoreGive( pCtx->iceMutex );

                    if( iceResult != ICE_RESULT_OK )
                    {
                        LogWarn( ( "Unable to STUN binding request  message, result: %d", iceResult ) );
                    }
                    else
                    {
                        LogDebug( ( "Sending STUN binding request, local/remote candidate ID: 0x%04x / 0x%04x",
                                    pCandidatePair->pLocalCandidate->candidateId,
                                    pCandidatePair->pRemoteCandidate->candidateId ) );
                        IceControllerNet_LogStunPacket( sentStunBuffer, sentStunBufferLength );

                        if( IceControllerNet_SendPacket( pCtx, pSocketContext, &( pSocketContext->pIceServer->iceEndpoint ), sentStunBuffer, sentStunBufferLength ) != ICE_CONTROLLER_RESULT_OK )
                        {
                            LogWarn( ( "Unable to send STUN binding request message" ) );
                        }
                    }
                }
                else
                {
                    LogError( ( "Failed to create connectivity check binding request: mutex lock acquisition." ) );
                    ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
                }
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_SEND_RESPONSE_AND_START_NOMINATION:
                /* In this case, we have to do 2 actions:
                 * 1. Send binding response back to remote peer.
                 * 2. Send nomination request for this pair. */
                ret = SendBindingResponse( pCtx, pSocketContext, pCandidatePair, pTransactionIdBuffer );
                if( ret != ICE_CONTROLLER_RESULT_OK )
                {
                    LogError( ( "Failed to send response for STUN binding request, result: %d", ret ) );
                    break;
                }

                /* 
                 * Note: After successfully sending the response, intentionally 
                 * falls through to the case below to send the nominating request.
                 */
            case ICE_HANDLE_STUN_PACKET_RESULT_START_NOMINATION:
                /* Add logic to start nomination flow here. NOTE: Take care of sending binding request part. */
                ret = SendNominationRequest( pCtx, pSocketContext, pCandidatePair, pTransactionIdBuffer );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_VALID_CANDIDATE_PAIR:
                LogInfo( ( "A valid candidate pair is found" ) );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_CANDIDATE_PAIR_READY:
                ret = CheckNomination( pCtx,
                                       pSocketContext,
                                       pCandidatePair );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_INTEGRITY_MISMATCH:
                LogWarn( ( "Message Integrity check of the received packet failed" ) );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_FINGERPRINT_MISMATCH:
                LogWarn( ( "FingerPrint check of the received packet failed" ) );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_INVALID_PACKET_TYPE:
                LogWarn( ( "Invalid Type of Packet received" ) );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_CANDIDATE_PAIR_NOT_FOUND:
                LogDebug( ( "Valid Candidate Pair is not found, it might be a duplicate response, local candidate ID: 0x%04x",
                            pSocketContext->pLocalCandidate->candidateId ) );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_CANDIDATE_NOT_FOUND:
                LogError( ( "Error : Valid Server Reflexive Candidate is not found, local candidate ID: 0x%04x",
                            pSocketContext->pLocalCandidate->candidateId ) );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_SEND_ALLOCATION_REQUEST:
                /* Received TURN allocation error response, get the nonce/realm from the message.
                 * Send the TURN allocation request again. */
                if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
                {
                    sentStunBufferLength = ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE;
                    iceResult = Ice_CreateNextCandidateRequest( &pCtx->iceContext,
                                                                pSocketContext->pLocalCandidate,
                                                                currentTimeSeconds,
                                                                sentStunBuffer,
                                                                &sentStunBufferLength );
                    xSemaphoreGive( pCtx->iceMutex );

                    if( iceResult == ICE_RESULT_OK )
                    {
                        LogDebug( ( "Sending TURN allocation request, local candidate ID: 0x%04x",
                                    pSocketContext->pLocalCandidate->candidateId ) );
                        IceControllerNet_LogStunPacket( sentStunBuffer, sentStunBufferLength );

                        if( IceControllerNet_SendPacket( pCtx, pSocketContext, &( pSocketContext->pIceServer->iceEndpoint ), sentStunBuffer, sentStunBufferLength ) != ICE_CONTROLLER_RESULT_OK )
                        {
                            LogWarn( ( "Unable to send STUN allocation request" ) );
                        }
                    }
                    else
                    {
                        LogWarn( ( "Not able to create candidate request with return: %d", iceResult ) );
                    }
                }
                else
                {
                    LogError( ( "Failed to create allocation request: mutex lock acquisition." ) );
                    ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
                }
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_FRESH_COMPLETE:
                LogInfo( ( "TURN session of local candidate ID: 0x%04x is refreshed.",
                           pSocketContext->pLocalCandidate->candidateId ) );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_TURN_SESSION_TERMINATED:
                LogInfo( ( "TURN session of local candidate ID: 0x%04x is terminated.",
                           pSocketContext->pLocalCandidate->candidateId ) );

                /* Close the socket as the TURN session is terminated. */
                IceControllerNet_FreeSocketContext( pCtx, pSocketContext );

                ret = ICE_CONTROLLER_RESULT_CONNECTION_CLOSED;
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_DROP_PACKET:
                LogInfo( ( "Drop the packet of local candidate ID: 0x%04x.",
                           pSocketContext->pLocalCandidate->candidateId ) );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_UNEXPECTED_RESPONSE:
                LogDebug( ( "Unexpected response. pair state is %d, local candidate ID: 0x%04x",
                            pCandidatePair->state,
                            pSocketContext->pLocalCandidate->candidateId ) );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_OK:
                LogVerbose( ( "ICE_HANDLE_STUN_PACKET_RESULT_OK" ) );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_NOT_STUN_PACKET:
                ret = ICE_CONTROLLER_RESULT_NOT_STUN_PACKET;
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_MATCHING_TRANSACTION_ID_NOT_FOUND:
                LogVerbose( ( "Transaction ID not matching, might be a duplicate response" ) );
                break;
            case ICE_HANDLE_STUN_PACKET_RESULT_FRESH_CHANNEL_BIND_COMPLETE:
                LogVerbose( ( "Channel binding success response, this might be a duplicate response." ) );
                break;
            default:
                LogWarn( ( "Unknown case: %d, packet length: %u, first two bytes: 0x%02x 0x%02x",
                           iceHandleStunResult,
                           receiveBufferLength,
                           pReceiveBuffer[ 0 ], pReceiveBuffer[ 1 ] ) );
                break;
        }
    }

    return ret;
}

IceControllerResult_t IceControllerNet_DnsLookUp( char * pUrl,
                                                  IceTransportAddress_t * pIceTransportAddress )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    int dnsResult;
    struct addrinfo * pResult = NULL;
    struct addrinfo * pIterator;
    struct sockaddr_in * ipv4Address;
    // struct sockaddr_in6 * ipv6Address;
    struct addrinfo hints = { 0 };

    if( ( pUrl == NULL ) || ( pIceTransportAddress == NULL ) )
    {
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Restrict getaddrinfo to query IPv4 only. */
        memset( &hints, 0, sizeof( struct addrinfo ) );
        hints.ai_family = AF_INET;
        dnsResult = getaddrinfo( pUrl, NULL, &hints, &pResult );
        if( dnsResult != 0 )
        {
            LogWarn( ( "DNS query failing, url: %s, result: %d", pUrl, dnsResult ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_DNS_QUERY;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        for( pIterator = pResult; pIterator; pIterator = pIterator->ai_next )
        {
            if( pIterator->ai_family == AF_INET )
            {
                ipv4Address = ( struct sockaddr_in * ) pIterator->ai_addr;
                pIceTransportAddress->family = STUN_ADDRESS_IPv4;
                memcpy( pIceTransportAddress->address, &ipv4Address->sin_addr, STUN_IPV4_ADDRESS_SIZE );
                break;
            }
            else if( pIterator->ai_family == AF_INET6 )
            {
                /* TODO: IPv6 */
                // ipv6Address = ( struct sockaddr_in6 * ) pIterator->ai_addr;
                // pIceTransportAddress->family = STUN_ADDRESS_IPv6;
                // memcpy( pIceTransportAddress->address, &ipv6Address->sin6_addr, STUN_IPV6_ADDRESS_SIZE );
                // break;
                continue;
            }
        }

        if( pIterator == NULL )
        {
            LogWarn( ( "No IPv4 address found for the given url: %s", pUrl ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_DNS_QUERY;
        }
    }

    if( pResult )
    {
        freeaddrinfo( pResult );
    }

    return ret;
}

#if LIBRARY_LOG_LEVEL >= LOG_INFO
const char * IceControllerNet_LogIpAddressInfo( const IceEndpoint_t * pIceEndpoint,
                                                char * pIpBuffer,
                                                size_t ipBufferLength )
{
    const char * ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_UNKNOWN;

    if( ( pIceEndpoint != NULL ) && ( pIpBuffer != NULL ) )
    {
        switch( pIceEndpoint->transportAddress.family )
        {
            case STUN_ADDRESS_IPv4:
                ret = inet_ntop( AF_INET, pIceEndpoint->transportAddress.address, pIpBuffer, ipBufferLength );
                if( ret == NULL )
                {
                    /*
                     * IP address to string conversion failed due to insufficient buffer size.
                     * Required: IPv4 (16 bytes), IPv6 (46 bytes)
                     */
                    ret = "Masked IPv4 Address";
                }
                break;
            case STUN_ADDRESS_IPv6:
                ret = inet_ntop( AF_INET6, pIceEndpoint->transportAddress.address, pIpBuffer, ipBufferLength );
                if( ret == NULL )
                {
                    /*
                     * IP address to string conversion failed due to insufficient buffer size.
                     * Required: IPv4 (16 bytes), IPv6 (46 bytes)
                     */
                    ret = "Masked IPv6 Address";
                }
                break;
            default:
                ret = "Unsupported IP Family";
                break;
        }
    }

    return ret;
}
#endif /* #if LIBRARY_LOG_LEVEL >= LOG_INFO */

#if LIBRARY_LOG_LEVEL >= LOG_VERBOSE

#define SWAP_BYTES_16( value )          \
    ( ( ( ( value ) >> 8 ) & 0xFF ) |   \
      ( ( ( value ) & 0xFF ) << 8 ) )

static uint16_t ReadUint16Swap( const uint8_t * pSrc )
{
    return SWAP_BYTES_16( *( ( uint16_t * )( pSrc ) ) );
}

static uint16_t ReadUint16NoSwap( const uint8_t * pSrc )
{
    return *( ( uint16_t * )( pSrc ) );
}

static const char * convertStunMsgTypeToString( uint16_t stunMsgType )
{
    const char * ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_UNKNOWN;
    static ReadUint16_t readUint16Fn;
    static uint8_t isFirst = 1;
    uint8_t isLittleEndian;
    uint16_t msgType;

    if( isFirst )
    {
        isFirst = 0;
        isLittleEndian = ( *( uint8_t * )( &( uint16_t ) { 1 } ) == 1 );

        if( isLittleEndian != 0 )
        {
            readUint16Fn = ReadUint16Swap;
        }
        else
        {
            readUint16Fn = ReadUint16NoSwap;
        }
    }

    msgType = readUint16Fn( ( uint8_t * ) &stunMsgType );
    switch( msgType )
    {
        case STUN_MESSAGE_TYPE_BINDING_REQUEST:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_BINDING_REQUEST;
            break;
        case STUN_MESSAGE_TYPE_BINDING_SUCCESS_RESPONSE:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_BINDING_SUCCESS;
            break;
        case STUN_MESSAGE_TYPE_BINDING_FAILURE_RESPONSE:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_BINDING_FAILURE;
            break;
        case STUN_MESSAGE_TYPE_BINDING_INDICATION:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_BINDING_INDICATION;
            break;
        case STUN_MESSAGE_TYPE_ALLOCATE_REQUEST:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_ALLOCATE_REQUEST;
            break;
        case STUN_MESSAGE_TYPE_ALLOCATE_SUCCESS_RESPONSE:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_ALLOCATE_SUCCESS;
            break;
        case STUN_MESSAGE_TYPE_ALLOCATE_ERROR_RESPONSE:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_ALLOCATE_FAILURE;
            break;
        case STUN_MESSAGE_TYPE_REFRESH_REQUEST:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_REFRESH_REQUEST;
            break;
        case STUN_MESSAGE_TYPE_REFRESH_SUCCESS_RESPONSE:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_REFRESH_SUCCESS;
            break;
        case STUN_MESSAGE_TYPE_REFRESH_ERROR_RESPONSE:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_REFRESH_FAILURE;
            break;
        case STUN_MESSAGE_TYPE_CREATE_PERMISSION_REQUEST:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CREATE_PERMISSION_REQUEST;
            break;
        case STUN_MESSAGE_TYPE_CREATE_PERMISSION_SUCCESS_RESPONSE:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CREATE_PERMISSION_SUCCESS;
            break;
        case STUN_MESSAGE_TYPE_CREATE_PERMISSION_ERROR_RESPONSE:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CREATE_PERMISSION_FAILURE;
            break;
        case STUN_MESSAGE_TYPE_CHANNEL_BIND_REQUEST:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CHANNEL_BIND_REQUEST;
            break;
        case STUN_MESSAGE_TYPE_CHANNEL_BIND_SUCCESS_RESPONSE:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CHANNEL_BIND_SUCCESS;
            break;
        case STUN_MESSAGE_TYPE_CHANNEL_BIND_ERROR_RESPONSE:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_CHANNEL_BIND_FAILURE;
            break;
        case STUN_MESSAGE_TYPE_SEND_INDICATION:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_SEND_INDICATION;
            break;
        case STUN_MESSAGE_TYPE_DATA_INDICATION:
            ret = ICE_CONTROLLER_STUN_MESSAGE_TYPE_STRING_DATA_INDICATION;
            break;
    }

    return ret;
}
#endif /* #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE */

void IceControllerNet_LogStunPacket( uint8_t * pStunPacket,
                                     size_t stunPacketSize )
{
    #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
    const uint8_t * pStunMsgContent = pStunPacket;
    IceControllerStunMsgHeader_t * pStunMsgHeader = ( IceControllerStunMsgHeader_t * ) pStunMsgContent;

    if( ( pStunPacket == NULL ) || ( stunPacketSize < sizeof( IceControllerStunMsgHeader_t ) ) )
    {
        // invalid STUN packet, ignore it
    }
    else
    {
        do
        {
            if( ( pStunPacket[0] & 0xF0 ) == 0x40 )
            {
                LogVerbose( ( "TURN channel number: 0x%02x%02x, TURN application data length: 0x%02x%02x",
                              pStunPacket[ 0 ], pStunPacket[ 1 ],
                              pStunPacket[ 2 ], pStunPacket[ 3 ] ) );
                pStunMsgContent = &pStunPacket[ ICE_TURN_CHANNEL_DATA_MESSAGE_HEADER_LENGTH ];
                pStunMsgHeader = ( IceControllerStunMsgHeader_t * ) pStunMsgContent;
                if( stunPacketSize < sizeof( IceControllerStunMsgHeader_t ) + ICE_TURN_CHANNEL_DATA_MESSAGE_HEADER_LENGTH )
                {
                    // invalid STUN packet, ignore it.
                    LogWarn( ( "Invalid TURN packet, packet size: %u", stunPacketSize ) );
                    break;
                }
            }

            /*
             * demux each packet off of its first byte
             * https://tools.ietf.org/html/rfc5764#section-5.1.2
             * +----------------+
             * | 127 < B < 192 -+--> forward to RTP/RTCP
             * |                |
             * |  19 < B < 64  -+--> forward to DTLS
             * |                |
             * |       B < 2   -+--> forward to STUN
             * +----------------+
             */
            if( pStunMsgContent[ 0 ] < 2 )
            {
                LogVerbose( ( "Dumping STUN packets: STUN type: %s, content length:: 0x%02x%02x, transaction ID: 0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
                              convertStunMsgTypeToString( pStunMsgHeader->msgType ),
                              pStunMsgHeader->contentLength[ 0 ], pStunMsgHeader->contentLength[ 1 ],
                              pStunMsgHeader->transactionId[ 0 ], pStunMsgHeader->transactionId[ 1 ], pStunMsgHeader->transactionId[ 2 ], pStunMsgHeader->transactionId[ 3 ],
                              pStunMsgHeader->transactionId[ 4 ], pStunMsgHeader->transactionId[ 5 ], pStunMsgHeader->transactionId[ 6 ], pStunMsgHeader->transactionId[ 7 ],
                              pStunMsgHeader->transactionId[ 8 ], pStunMsgHeader->transactionId[ 9 ], pStunMsgHeader->transactionId[ 10 ], pStunMsgHeader->transactionId[ 11 ] ) );
            }
        } while( 0U );
    }
    #endif /* #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE  */

    ( void ) pStunPacket;
    ( void ) stunPacketSize;
}
