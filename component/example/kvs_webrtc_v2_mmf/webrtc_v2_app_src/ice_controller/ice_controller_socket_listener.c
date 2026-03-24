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
#include "logging.h"
#include "ice_api.h"
#include "ice_controller.h"
#include "ice_controller_private.h"
#include "task.h"
#include "stun_deserializer.h"
#include "transport_mbedtls.h"

#if ENABLE_SCTP_DATA_CHANNEL
    #include "sctp_utils.h"
    #include "peer_connection_sctp.h"
#endif /* ENABLE_SCTP_DATA_CHANNEL */

#define ICE_CONTROLLER_SOCKET_LISTENER_SELECT_BLOCK_TIME_MS ( 50 )
#define RX_BUFFER_SIZE ( 4096 )

static int32_t RecvPacketUdp( IceControllerSocketContext_t * pSocketContext,
                              uint8_t * pBuffer,
                              size_t bufferSize,
                              int flags,
                              IceEndpoint_t * pRemoteEndpoint )
{
    int32_t ret;
    struct sockaddr_storage srcAddress;
    socklen_t srcAddressLength = sizeof( srcAddress );
    struct sockaddr_in * pIpv4Address;
    struct sockaddr_in6 * pIpv6Address;
    uint8_t keepProcess = 1U;

    ret = recvfrom( pSocketContext->socketFd,
                    pBuffer,
                    bufferSize,
                    flags,
                    ( struct sockaddr * ) &srcAddress,
                    &srcAddressLength );

    if( ret < 0 )
    {
        if( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) )
        {
            /* Timeout, no more data to receive. */
            ret = 0;
            keepProcess = 0U;
        }
    }
    else if( ret == 0 )
    {
        /* Nothing to do if receive 0 byte. */
        keepProcess = 0U;
    }
    else
    {
        /* Empty else marker. */
    }

    if( keepProcess != 0U )
    {
        /* Received data, handle this STUN message. */
        if( srcAddress.ss_family == AF_INET )
        {
            pIpv4Address = ( struct sockaddr_in * ) &srcAddress;

            pRemoteEndpoint->transportAddress.family = STUN_ADDRESS_IPv4;
            pRemoteEndpoint->transportAddress.port = ntohs( pIpv4Address->sin_port );
            memcpy( pRemoteEndpoint->transportAddress.address, &pIpv4Address->sin_addr, STUN_IPV4_ADDRESS_SIZE );
        }
        else if( srcAddress.ss_family == AF_INET6 )
        {
            pIpv6Address = ( struct sockaddr_in6 * ) &srcAddress;

            pRemoteEndpoint->transportAddress.family = STUN_ADDRESS_IPv6;
            pRemoteEndpoint->transportAddress.port = ntohs( pIpv6Address->sin6_port );
            memcpy( pRemoteEndpoint->transportAddress.address, &pIpv6Address->sin6_addr, STUN_IPV6_ADDRESS_SIZE );
        }
        else
        {
            /* Unknown IP type, drop packet. */
            LogWarn( ( "Unknown source type(%d) from UDP connection.", srcAddress.ss_family ) );
            ret = -1;
        }
    }

    return ret;
}

static int32_t RecvPacketTls( IceControllerSocketContext_t * pSocketContext,
                              uint8_t * pBuffer,
                              size_t bufferSize,
                              IceEndpoint_t * pRemoteEndpoint )
{
    int32_t ret;

    memcpy( pRemoteEndpoint, &( pSocketContext->pIceServer->iceEndpoint ), sizeof( IceEndpoint_t ) );
    ret = TLS_FreeRTOS_recv( &pSocketContext->tlsSession.xTlsNetworkContext,
                             pBuffer,
                             bufferSize );

    if( ret < 0 )
    {
        LogError( ( "Receiving %ld from TLS connection", ret ) );
    }

    return ret;
}

static IceCandidatePair_t * FindCandidatePairByRemoteIceEndpoint( IceControllerContext_t * pCtx,
                                                                  IceControllerSocketContext_t * pSocketContext,
                                                                  IceEndpoint_t * pRemoteIceEndpoint )
{
    IceControllerResult_t result = ICE_CONTROLLER_RESULT_OK;
    IceCandidatePair_t * pCandidatePair = NULL;
    IceResult_t iceResult;
    size_t count;
    size_t i;
    uint8_t isLocked = 0U;

    /* Take ice lock. */
    if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
    {
        isLocked = 1U;
    }
    else
    {
        LogError( ( "Failed to process candidate pairs: mutex lock acquisition." ) );
        result = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
    }

    if( result == ICE_CONTROLLER_RESULT_OK )
    {
        iceResult = Ice_GetCandidatePairCount( &pCtx->iceContext,
                                               &count );
        if( iceResult != ICE_RESULT_OK )
        {
            LogError( ( "Fail to query valid candidate pair count, result: %d", iceResult ) );
            result = ICE_CONTROLLER_RESULT_FAIL_QUERY_CANDIDATE_PAIR_COUNT;
        }
    }

    if( result == ICE_CONTROLLER_RESULT_OK )
    {
        for( i = 0; i < count; i++ )
        {
            if( ( memcmp( &pCtx->iceContext.pCandidatePairs[i].pLocalCandidate->endpoint.transportAddress,
                          &pSocketContext->pLocalCandidate->endpoint.transportAddress,
                          sizeof( IceTransportAddress_t ) ) == 0 ) &&
                ( memcmp( &pCtx->iceContext.pCandidatePairs[i].pRemoteCandidate->endpoint.transportAddress,
                          &pRemoteIceEndpoint->transportAddress,
                          sizeof( IceTransportAddress_t ) ) == 0 ) )
            {
                pCandidatePair = &pCtx->iceContext.pCandidatePairs[i];
                break;
            }
        }
    }

    if( isLocked != 0U )
    {
        xSemaphoreGive( pCtx->iceMutex );
    }

    return pCandidatePair;
}

static IceControllerResult_t UpdateNominatedSocketContext( IceControllerContext_t * pCtx,
                                                           IceControllerSocketContext_t * pSocketContext,
                                                           IceCandidatePair_t * pCandidatePair,
                                                           IceEndpoint_t * pRemoteIceEndpoint )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceCandidatePair_t * pOriginalCandidatePair = NULL;
    OnIceEventCallback_t onIceEventCallbackFunc = NULL;
    void * pOnIceEventCallbackCustomContext = NULL;
    int32_t retPeerToPeerConnectionFound = 0;
    #if LIBRARY_LOG_LEVEL >= LOG_INFO
    char ipBuffer[ INET_ADDRSTRLEN ];
    #endif

    /* Find valid candidate pair pointer for current packet.
     * There are two scenarios:
     *   1. Host/Server Reflexive Candidates: The candidate pair pointer is NULL because we haven't mapped the remote endpoint.
     *   2. Peer/Client Reflexive Candidates: The candidate pair must be valid because we should extract it from Ice_HandleTurnPacket.
     */
    if( ( pSocketContext->pLocalCandidate->candidateType == ICE_CANDIDATE_TYPE_HOST ) ||
        ( pSocketContext->pLocalCandidate->candidateType == ICE_CANDIDATE_TYPE_SERVER_REFLEXIVE ) )
    {
        if( pCandidatePair == NULL )
        {
            pCandidatePair = FindCandidatePairByRemoteIceEndpoint( pCtx, pSocketContext, pRemoteIceEndpoint );
            if( pCandidatePair == NULL )
            {
                LogWarn( ( "Invalid to find candidate pair for the remote endpoint." ) );

                LogInfo( ( "Target remote endpoint IP address: %s, port: %u",
                           IceControllerNet_LogIpAddressInfo( pRemoteIceEndpoint, ipBuffer, sizeof( ipBuffer ) ),
                           pRemoteIceEndpoint->transportAddress.port ) );
                ret = ICE_CONTROLLER_RESULT_INVALID_PACKET;
            }
        }
        else
        {
            LogWarn( ( "ICE Candidate Pair Error: Host/Srflx candidate contains valid pair pointer when it should be NULL." ) );
            ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
        }
    }
    else
    {
        /* Relay candidate. */
        if( pCandidatePair == NULL )
        {
            LogWarn( ( "ICE Candidate Pair Error: Relay candidate contains NULL pair pointer when it should be valid." ) );
            ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Update nominated socket context. */
        if( xSemaphoreTake( pCtx->socketMutex, portMAX_DELAY ) == pdTRUE )
        {
            /* While running as viewer, the other master side might start DTLS handshaking
             * earlier than ICE_CANDIDATE STUN binding request. */
            if( pCtx->pNominatedSocketContext != NULL )
            {
                pOriginalCandidatePair = pCtx->pNominatedSocketContext->pCandidatePair;
            }
            pCtx->pNominatedSocketContext = pSocketContext;
            pCtx->pNominatedSocketContext->pRemoteCandidate = pCandidatePair->pRemoteCandidate;
            pCtx->pNominatedSocketContext->pCandidatePair = pCandidatePair;
            pCtx->pNominatedSocketContext->state = ICE_CONTROLLER_SOCKET_CONTEXT_STATE_SELECTED;

            onIceEventCallbackFunc = pCtx->onIceEventCallbackFunc;
            pOnIceEventCallbackCustomContext = pCtx->pOnIceEventCustomContext;

            ( void ) pOriginalCandidatePair;

            /* We have finished accessing the shared resource.  Release the mutex. */
            xSemaphoreGive( pCtx->socketMutex );

            LogInfo( ( "Nominated pair is changed from local/remote candidate ID: 0x%04x / 0x%04x to local/remote candidate ID: 0x%04x / 0x%04x",
                       pOriginalCandidatePair == NULL? 0:pOriginalCandidatePair->pLocalCandidate->candidateId,
                       pOriginalCandidatePair == NULL? 0:pOriginalCandidatePair->pRemoteCandidate->candidateId,
                       pCandidatePair->pLocalCandidate->candidateId,
                       pCandidatePair->pRemoteCandidate->candidateId ) );

            if( pOriginalCandidatePair == NULL )
            {
                IceController_UpdateState( pCtx, ICE_CONTROLLER_STATE_READY );
                IceController_UpdateTimerInterval( pCtx, ICE_CONTROLLER_PERIODIC_TIMER_INTERVAL_MS );

                /* Found nominated pair, execute DTLS handshake and release all other resources. */
                if( onIceEventCallbackFunc )
                {
                    retPeerToPeerConnectionFound = onIceEventCallbackFunc( pOnIceEventCallbackCustomContext,
                                                                           ICE_CONTROLLER_CB_EVENT_PEER_TO_PEER_CONNECTION_FOUND,
                                                                           NULL );
                    if( retPeerToPeerConnectionFound != 0 )
                    {
                        LogError( ( "Fail to handle peer to peer connection found event, ret: %ld", retPeerToPeerConnectionFound ) );
                    }
                }
                else
                {
                    LogWarn( ( "No callback function to handle P2P connection found event." ) );
                }
            }
        }
        else
        {
            LogError( ( "Failed to update nominated socket context: mutex lock acquisition." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
        }
    }

    return ret;
}

static void HandleRxPacket( IceControllerContext_t * pCtx,
                            IceControllerSocketContext_t * pSocketContext,
                            OnRecvNonStunPacketCallback_t onRecvNonStunPacketFunc,
                            void * pOnRecvNonStunPacketCallbackContext )
{
    uint8_t skipProcess = 0;
    int32_t readBytes = 0;
    IceEndpoint_t remoteIceEndpoint;
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    IceCandidatePair_t * pCandidatePair = NULL;
    uint8_t * pTurnPayload = NULL;
    uint16_t turnPayloadBufferLength = 0;
    uint8_t receiveBuffer[ RX_BUFFER_SIZE ];
    uint8_t * pProcessingBuffer = receiveBuffer;
    size_t processingBufferLength = 0;

    if( ( pCtx == NULL ) || ( pSocketContext == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pSocketContext: %p", pCtx, pSocketContext ) );
        skipProcess = 1;
    }

    while( !skipProcess )
    {
        memset( &remoteIceEndpoint, 0, sizeof( IceEndpoint_t ) );
        if( pSocketContext->socketType == ICE_CONTROLLER_SOCKET_TYPE_UDP )
        {
            readBytes = RecvPacketUdp( pSocketContext, pProcessingBuffer, RX_BUFFER_SIZE, 0, &remoteIceEndpoint );
        }
        else if( pSocketContext->socketType == ICE_CONTROLLER_SOCKET_TYPE_TLS )
        {
            readBytes = RecvPacketTls( pSocketContext, pProcessingBuffer, RX_BUFFER_SIZE, &remoteIceEndpoint );
        }
        else
        {
            LogError( ( "Internal error, invalid socket type %d", pSocketContext->socketType ) );

            skipProcess = 1;
            break;
        }

        if( readBytes < 0 )
        {
            LogError( ( "Fail to receive packets from socket ID: %d, errno: %s", pSocketContext->socketFd, strerror( errno ) ) );
            skipProcess = 1;
            break;
        }
        else if( readBytes == 0 )
        {
            /* Nothing to do if receive 0 byte. */
            break;
        }
        else
        {
            /* Received valid data, keep addressing. */
            LogVerbose( ( "Receiving %ld btyes on local candidate ID: 0x%04x", readBytes, pSocketContext->pLocalCandidate->candidateId ) );
            processingBufferLength = ( size_t ) readBytes;
        }

        if( pSocketContext->pLocalCandidate->candidateType == ICE_CANDIDATE_TYPE_RELAY )
        {
            if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
            {
                iceResult = Ice_HandleTurnPacket( &pCtx->iceContext,
                                                  pProcessingBuffer,
                                                  processingBufferLength,
                                                  pSocketContext->pLocalCandidate,
                                                  ( const uint8_t ** ) &pTurnPayload,
                                                  &turnPayloadBufferLength,
                                                  &pCandidatePair );
                xSemaphoreGive( pCtx->iceMutex );

                if( iceResult == ICE_RESULT_OK )
                {
                    LogVerbose( ( "Removed TURN channel header for local/remote candidate ID 0x%04x / 0x%04x, number: 0x%02x%02x, length: 0x%02x%02x",
                                  pCandidatePair->pLocalCandidate->candidateId,
                                  pCandidatePair->pRemoteCandidate->candidateId,
                                  pProcessingBuffer[ 0 ], pProcessingBuffer[ 1 ],
                                  pProcessingBuffer[ 2 ], pProcessingBuffer[ 3 ] ) );

                    /* Received TURN buffer, replace buffer pointer for further processing. */
                    pProcessingBuffer = pTurnPayload;
                    processingBufferLength = turnPayloadBufferLength;
                }
                else
                {
                    /* TURN prefix not required, keep original buffer. */
                    LogDebug( ( "No need to remove TURN header, result: %d", iceResult ) );
                }
            }
            else
            {
                LogError( ( "Failed to handle TURN packet: mutex lock acquisition." ) );
                break;
            }
        }

        /*
         *  demux each packet off of its first byte
         *  https://tools.ietf.org/html/rfc5764#section-5.1.2
         *  +----------------+
         *  | 127 < B < 192 -+--> forward to RTP/RTCP
         *  |                |
         *  |  19 < B < 64  -+--> forward to DTLS
         *  |                |
         *  |       B < 2   -+--> forward to STUN
         *  +----------------+
         */
        if( processingBufferLength > 0 )
        {
            if( ( ( pProcessingBuffer[ 0 ] > 127 ) && ( pProcessingBuffer[ 0 ] < 192 ) ) ||
                ( ( pProcessingBuffer[ 0 ] > 19 ) && ( pProcessingBuffer[ 0 ] < 64 ) ) )
            {
                /* It's not STUN packet, deliever to peer connection to handle RTP or DTLS packet. */
                /* When ICE controlling agent sends all binding requests with USE-CANDIDATE flag in connectivity stage,
                 * it's possible to pick different agent between local and remote peer. Thus we update nominated pair pointer
                 * to handle current packet. */
                if( onRecvNonStunPacketFunc )
                {
                    if( pCtx->pNominatedSocketContext != pSocketContext )
                    {
                        ret = UpdateNominatedSocketContext( pCtx, pSocketContext, pCandidatePair, &remoteIceEndpoint );
                    }

                    if( ret == ICE_CONTROLLER_RESULT_OK )
                    {
                        ( void ) onRecvNonStunPacketFunc( pOnRecvNonStunPacketCallbackContext,
                                                          pProcessingBuffer,
                                                          processingBufferLength );
                    }
                    else
                    {
                        LogWarn( ( "DTLS packet rejected: Received from non-selected ICE candidate pair" ) );
                    }
                }
                else
                {
                    LogError( ( "No callback function to handle DTLS/RTP/RTCP packets." ) );
                }
            }
            else if( pProcessingBuffer[ 0 ] < 2 )
            {
                /* STUN packet. */
                ret = IceControllerNet_HandleStunPacket( pCtx,
                                                         pSocketContext,
                                                         pProcessingBuffer,
                                                         processingBufferLength,
                                                         &remoteIceEndpoint,
                                                         pCandidatePair );
                if( ( ret == ICE_CONTROLLER_RESULT_FOUND_CONNECTION ) &&
                    ( pCtx->pNominatedSocketContext == NULL ) )
                {
                    UpdateNominatedSocketContext( pCtx,
                                                  pSocketContext,
                                                  pCandidatePair,
                                                  &remoteIceEndpoint );
                }
                else if( ( ret == ICE_CONTROLLER_RESULT_FOUND_CONNECTION ) || ( ret == ICE_CONTROLLER_RESULT_OK ) )
                {
                    /* Handle STUN packet successfully, keep processing. */
                }
                else if( ret == ICE_CONTROLLER_RESULT_CONNECTION_CLOSED )
                {
                    /* Socket has been closed, skip the next recv loop. */
                    break;
                }
                else
                {
                    LogError( ( "Fail to handle this RX packet, ret: %d, readBytes: %u", ret, processingBufferLength ) );
                }
            }
            else
            {
                /* Unknown packet. */
                LogWarn( ( "drop unknown packet, length=%u, first byte=0x%02x",
                           processingBufferLength,
                           pProcessingBuffer[ 0 ] ) );
            }
        }
    }

    if( readBytes < 0 )
    {
        /*
         * Socket read error detected (readBytes < 0).
         * This typically indicates the remote peer closed the connection.
         * Action required: Close the local socket to properly terminate the connection.
         */
        ( void ) Ice_CloseCandidate( &pCtx->iceContext, pSocketContext->pLocalCandidate );
        IceControllerNet_FreeSocketContext( pCtx, pSocketContext );
    }
}

static void pollingSockets( IceControllerContext_t * pCtx )
{
    fd_set rfds;
    int i;
    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = ICE_CONTROLLER_SOCKET_LISTENER_SELECT_BLOCK_TIME_MS * 1000,
    };
    int maxFd = 0;
    int retSelect;
    uint8_t skipProcess = 0;
    int fds[ ICE_CONTROLLER_MAX_LOCAL_CANDIDATE_COUNT ];
    size_t fdsCount;
    OnRecvNonStunPacketCallback_t onRecvNonStunPacketFunc;
    void * pOnRecvNonStunPacketCallbackContext = NULL;
    IceControllerSocketContext_t * pSocketContext;

    FD_ZERO( &rfds );

    if( xSemaphoreTake( pCtx->socketMutex, portMAX_DELAY ) == pdTRUE )
    {
        for( i = 0; i < pCtx->socketsContextsCount; i++ )
        {
            fds[i] = pCtx->socketsContexts[i].socketFd;
        }
        fdsCount = pCtx->socketsContextsCount;
        onRecvNonStunPacketFunc = pCtx->socketListenerContext.onRecvNonStunPacketFunc;
        pOnRecvNonStunPacketCallbackContext = pCtx->socketListenerContext.pOnRecvNonStunPacketCallbackContext;

        /* We have finished accessing the shared resource.  Release the mutex. */
        xSemaphoreGive( pCtx->socketMutex );
    }
    else
    {
        LogError( ( "Unexpected behavior: fail to take mutex" ) );
        skipProcess = 1;
    }

    if( !skipProcess )
    {
        /* Set rfds for select function. */
        for( i = 0; i < fdsCount; i++ )
        {
            /* fds might be removed for any reason. Handle that by checking if it's -1. */
            if( fds[i] >= 0 )
            {
                FD_SET( fds[i], &rfds );
                if( fds[i] > maxFd )
                {
                    maxFd = fds[i];
                }
            }
        }

        /* Poll all socket handlers. */
        retSelect = select( maxFd + 1, &rfds, NULL, NULL, &tv );
        if( retSelect < 0 )
        {
            LogError( ( "select return error value %d", retSelect ) );
            skipProcess = 1;
        }
        else if( retSelect == 0 )
        {
            /* It's just timeout. */
            skipProcess = 1;
        }
        else
        {
            /* Empty else marker. */
        }
    }

    if( !skipProcess )
    {
        for( i = 0; i < fdsCount; i++ )
        {
            if( ( fds[i] >= 0 ) && FD_ISSET( fds[i], &rfds ) )
            {
                pSocketContext = &( pCtx->socketsContexts[ i ] );

                if( pSocketContext->state == ICE_CONTROLLER_SOCKET_CONTEXT_STATE_CONNECTION_IN_PROGRESS )
                {
                    ( void ) IceControllerNet_ExecuteTlsHandshake( pCtx, pSocketContext, 0U );
                }
                else
                {
                    HandleRxPacket( pCtx,
                                    pSocketContext,
                                    onRecvNonStunPacketFunc,
                                    pOnRecvNonStunPacketCallbackContext );
                }
            }
        }
    }
}

IceControllerResult_t IceControllerSocketListener_StartPolling( IceControllerContext_t * pCtx )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;

    if( xSemaphoreTake( pCtx->socketMutex, portMAX_DELAY ) == pdTRUE )
    {
        pCtx->socketListenerContext.executeSocketListener = 1;

        /* We have finished accessing the shared resource.  Release the mutex. */
        xSemaphoreGive( pCtx->socketMutex );

        LogDebug( ( "Socket Listener: start polling" ) );
    }
    else
    {
        LogError( ( "Unexpected behavior: fail to take mutex" ) );
        ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
    }

    return ret;
}

IceControllerResult_t IceControllerSocketListener_StopPolling( IceControllerContext_t * pCtx )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;

    if( xSemaphoreTake( pCtx->socketMutex, portMAX_DELAY ) == pdTRUE )
    {
        pCtx->socketListenerContext.executeSocketListener = 0;

        /* We have finished accessing the shared resource.  Release the mutex. */
        xSemaphoreGive( pCtx->socketMutex );

        LogDebug( ( "Socket Listener: stop polling" ) );
    }
    else
    {
        LogError( ( "Unexpected behavior: fail to take mutex" ) );
        ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
    }

    return ret;
}

IceControllerResult_t IceControllerSocketListener_Init( IceControllerContext_t * pCtx,
                                                        OnRecvNonStunPacketCallback_t onRecvNonStunPacketFunc,
                                                        void * pOnRecvNonStunPacketCallbackContext )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;

    if( pCtx == NULL )
    {
        LogError( ( "Invalid input: pCtx is NULL" ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        pCtx->socketListenerContext.executeSocketListener = 0;
        pCtx->socketListenerContext.onRecvNonStunPacketFunc = onRecvNonStunPacketFunc;
        pCtx->socketListenerContext.pOnRecvNonStunPacketCallbackContext = pOnRecvNonStunPacketCallbackContext;
    }

    return ret;
}

void IceControllerSocketListener_Task( void * pParameter )
{
    IceControllerContext_t * pCtx = ( IceControllerContext_t * ) pParameter;

    for( ;; )
    {
        while( pCtx->socketListenerContext.executeSocketListener == 0 )
        {
            vTaskDelay( pdMS_TO_TICKS( ICE_CONTROLLER_SOCKET_LISTENER_SELECT_BLOCK_TIME_MS ) );
        }

        if( pCtx->socketListenerContext.executeSocketListener == 1 )
        {
            pollingSockets( pCtx );
        }
    }
}
