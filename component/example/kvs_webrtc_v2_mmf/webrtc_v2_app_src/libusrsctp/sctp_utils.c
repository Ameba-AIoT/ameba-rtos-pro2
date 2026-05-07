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

#if ENABLE_SCTP_DATA_CHANNEL

/* Standard includes. */
#include <stdio.h>
#include <unistd.h>

/* Application includes. */
#include "peer_connection.h"
#include "networking_utils.h"
#include "peer_connection_sctp.h"


#define SCTP_MTU                        1188
#define SCTP_ASSOCIATION_DEFAULT_PORT   5000

#define SCTP_SESSION_SHUTDOWN_INITIATED 0
#define SCTP_SESSION_ACTIVE             1
#define SCTP_SESSION_SHUTDOWN_COMPLETED 2

#define SECONDS_TO_USEC( x )            ( ( x ) * 1000000 )

#define SCTP_PPID_DCEP                  50
#define SCTP_PPID_STRING                51
#define SCTP_PPID_BINARY                53
#define SCTP_PPID_STRING_EMPTY          56
#define SCTP_PPID_BINARY_EMPTY          57
/*-----------------------------------------------------------*/

static SctpUtilsResult_t ConfigureSctpSocket( struct socket * pSocket );

static int OnSctpOutboundPacket( void * pAddr,
                                 void * pData,
                                 size_t length,
                                 uint8_t tos,
                                 uint8_t setDf );

static SctpUtilsResult_t HandleDcepMessage( SctpSession_t * pSctpSession,
                                            uint16_t channelId,
                                            uint8_t * pData,
                                            size_t length );

static int OnSctpInboundPacket( struct socket * pSocket,
                                union sctp_sockstore addr,
                                void * pData,
                                size_t length,
                                struct sctp_rcvinfo rcv,
                                int flags,
                                void * pUlpInfo );

static SctpUtilsResult_t SendOpenDataChannelAck( SctpSession_t * pSctpSession,
                                                 uint16_t channelId );
/*-----------------------------------------------------------*/

/* Configure the SCTP socket with default settings. */
static SctpUtilsResult_t ConfigureSctpSocket( struct socket * pSocket )
{
    SctpUtilsResult_t retStatus = SCTP_UTILS_RESULT_OK;
    struct linger lingerOpts;
    struct sctp_event event;
    struct sctp_initmsg initmsg;
    uint32_t i;
    uint32_t valueOn = 1;
    uint16_t eventTypes[] = { SCTP_ASSOC_CHANGE,
                              SCTP_PEER_ADDR_CHANGE,
                              SCTP_REMOTE_ERROR,
                              SCTP_SHUTDOWN_EVENT,
                              SCTP_ADAPTATION_INDICATION,
                              SCTP_PARTIAL_DELIVERY_EVENT };

    if( usrsctp_set_non_blocking( pSocket, 1 ) != 0 )
    {
        LogError( ( "usrsctp_set_non_blocking failed!" ) );
        retStatus = SCTP_UTILS_RESULT_FAIL;
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        /* OnSctpOutboundPacket must not be called after close. */
        lingerOpts.l_onoff = 1;
        lingerOpts.l_linger = 0;
        if( usrsctp_setsockopt( pSocket,
                                SOL_SOCKET,
                                SO_LINGER,
                                &( lingerOpts ),
                                sizeof( lingerOpts ) ) != 0 )
        {
            LogError( ( "usrsctp_setsockopt failed: SOL_SOCKET, SO_LINGER!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        /* Packets are generally sent as soon as possible and no unnecessary
         * delays are introduced, at the cost of more packets in the network. */
        if( usrsctp_setsockopt( pSocket,
                                IPPROTO_SCTP,
                                SCTP_NODELAY,
                                &( valueOn ),
                                sizeof( valueOn ) ) != 0 )
        {
            LogError( ( "usrsctp_setsockopt failed: IPPROTO_SCTP, SCTP_NODELAY!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        memset( &( event ), 0, sizeof( event ) );
        event.se_assoc_id = SCTP_FUTURE_ASSOC;
        event.se_on = 1;
        for( i = 0; i < ( uint32_t ) ( sizeof( eventTypes ) / sizeof( eventTypes[ 0 ] ) ); i++ )
        {
            event.se_type = eventTypes[ i ];
            if( usrsctp_setsockopt( pSocket,
                                    IPPROTO_SCTP,
                                    SCTP_EVENT,
                                    &( event ),
                                    sizeof( event ) ) != 0 )
            {
                LogError( ( "usrsctp_setsockopt failed: IPPROTO_SCTP, SCTP_EVENT!" ) );
                retStatus = SCTP_UTILS_RESULT_FAIL;
                break;
            }
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        memset( &( initmsg ), 0, sizeof( struct sctp_initmsg ) );
        initmsg.sinit_num_ostreams = 300;
        initmsg.sinit_max_instreams = 300;

        if( usrsctp_setsockopt( pSocket,
                                IPPROTO_SCTP,
                                SCTP_INITMSG,
                                &( initmsg ),
                                sizeof( struct sctp_initmsg ) ) != 0 )
        {
            LogError( ( "usrsctp_setsockopt failed: IPPROTO_SCTP, SCTP_INITMSG!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

/* Callback from the SCTP stack when an outbound packet is
 * ready for processing. */
static int OnSctpOutboundPacket( void * pAddr,
                                 void * pData,
                                 size_t length,
                                 uint8_t tos,
                                 uint8_t setDf )
{
    ( void ) ( tos );
    ( void ) ( setDf );

    SctpSession_t * pSctpSession = ( SctpSession_t * ) pAddr;

    if( ( pSctpSession == NULL ) ||
        ( pSctpSession->shutdownStatus == SCTP_SESSION_SHUTDOWN_INITIATED ) ||
        ( pSctpSession->sctpSessionCallbacks.outboundPacketCallback == NULL ) )
    {
        if( pSctpSession != NULL )
        {
            pSctpSession->shutdownStatus = SCTP_SESSION_SHUTDOWN_COMPLETED;
        }
        return -1;
    }

    /* Call the session and channel specific callback configured by the peer
     * connection. */
    pSctpSession->sctpSessionCallbacks.outboundPacketCallback( pSctpSession->sctpSessionCallbacks.pUserData,
                                                               pData,
                                                               length );

    return 0;
}


/*-----------------------------------------------------------*/

/* Handle an incoming DCEP message. */
static SctpUtilsResult_t HandleDcepMessage( SctpSession_t * pSctpSession,
                                            uint16_t channelId,
                                            uint8_t * pData,
                                            size_t length )
{

    SctpUtilsResult_t retStatus = SCTP_UTILS_RESULT_OK;
    DcepContext_t dcepCtx;
    DcepResult_t dcepResult = DCEP_RESULT_OK;
    DcepMessageType_t dcepMessageType;
    DcepChannelOpenMessage_t channelOpenMessage;

    dcepResult = Dcep_Init( &( dcepCtx ) );

    if( dcepResult != DCEP_RESULT_OK )
    {
        retStatus = SCTP_UTILS_RESULT_FAIL;
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        dcepResult = Dcep_GetMessageType( &( dcepCtx ), pData, length, &( dcepMessageType ) );

        if( dcepResult != DCEP_RESULT_OK )
        {
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        switch( dcepMessageType )
        {
            case DCEP_MESSAGE_DATA_CHANNEL_ACK:
            {
                if( pSctpSession->sctpSessionCallbacks.dataChannelOpenAckCallback( pSctpSession->sctpSessionCallbacks.pUserData,
                                                                                   channelId ) == SCTP_UTILS_RESULT_OK )
                {
                    LogInfo( ( "Successfully opened data channel ID: %u", ( unsigned int ) channelId ) );
                }
                else
                {
                    LogWarn( ( " Failed to open data channel for which DCEP_MESSAGE_DATA_CHANNEL_ACK was received " ) );
                }
            }
            break;

            case DCEP_MESSAGE_DATA_CHANNEL_OPEN:
            {
                dcepResult = Dcep_DeserializeChannelOpenMessage( &( dcepCtx ),
                                                                 pData,
                                                                 length,
                                                                 &( channelOpenMessage ) );

                if( dcepResult == DCEP_RESULT_OK )
                {
                    pSctpSession->sctpSessionCallbacks.dataChannelOpenCallback( pSctpSession->sctpSessionCallbacks.pUserData,
                                                                                channelId,
                                                                                channelOpenMessage.pChannelName,
                                                                                channelOpenMessage.channelNameLength );

                    /* Send DATA_CHANNEL_ACK Message. */
                    if( SendOpenDataChannelAck( pSctpSession, channelId ) != SCTP_UTILS_RESULT_OK )
                    {
                        LogWarn( ( "Failed to sending DCEP_MESSAGE_DATA_CHANNEL_ACK!" ) );
                    }
                }
                else
                {
                    retStatus = SCTP_UTILS_RESULT_FAIL;
                }
            }
            break;

            default:
            {
                LogWarn( ( "Unknown SCTP DCEP message type: %d", ( int ) dcepMessageType ) );
                retStatus = SCTP_UTILS_RESULT_FAIL;
            }
            break;
        }
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

/* Process an incoming SCTP packet, this API is passed as a callback to the
 * SCTP stack to be called while there is a valid packet ready. */
static int OnSctpInboundPacket( struct socket * pSocket,
                                union sctp_sockstore addr,
                                void * pData,
                                size_t length,
                                struct sctp_rcvinfo rcv,
                                int flags,
                                void * pUlpInfo )
{
    int retStatus = 1;
    SctpSession_t * pSctpSession = ( SctpSession_t * ) pUlpInfo;
    uint8_t isBinary = 0U;

    ( void )( pSocket );
    ( void )( addr );
    ( void )( flags );

    rcv.rcv_ppid = ntohl( rcv.rcv_ppid );

    switch( rcv.rcv_ppid )
    {
        /* Process incoming DCEP messages. */
        case SCTP_PPID_DCEP:
        {
            if( HandleDcepMessage( pSctpSession,
                                   rcv.rcv_sid,
                                   pData,
                                   length ) != SCTP_UTILS_RESULT_OK )
            {
                retStatus = 1;
            }
        }
        break;

        /* Process incoming application data. */
        case SCTP_PPID_BINARY:
        case SCTP_PPID_BINARY_EMPTY:
            isBinary = true;
        /* Intentional fallthrough. */
        case SCTP_PPID_STRING:
        case SCTP_PPID_STRING_EMPTY:
        {

            pSctpSession->sctpSessionCallbacks.dataChannelMessageCallback( pSctpSession->sctpSessionCallbacks.pUserData,
                                                                           rcv.rcv_sid,
                                                                           isBinary,
                                                                           pData,
                                                                           length );
        }
        break;

        default:
        {
            LogWarn( ( "Unhandled PPID on incoming SCTP message %ld", ( unsigned long ) rcv.rcv_ppid ) );
        }
        break;
    }

    /*
     * IMPORTANT!!! The allocation is done in the sctp library using default
     * allocator, so we need to use the default free API.
     */
    if( pData != NULL )
    {
        free( pData );
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

static SctpUtilsResult_t SendOpenDataChannelAck( SctpSession_t * pSctpSession,
                                                 uint16_t channelId )
{
    DcepContext_t dcepCtx;
    DcepResult_t dcepResult = DCEP_RESULT_OK;
    SctpUtilsResult_t retStatus = SCTP_UTILS_RESULT_OK;

    dcepResult = Dcep_Init( &( dcepCtx ) );

    if( dcepResult != DCEP_RESULT_OK )
    {
        LogError( ( "Dcep_Init failed!" ) );
        retStatus = SCTP_UTILS_RESULT_FAIL;
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        pSctpSession->packetSize = sizeof( pSctpSession->packet );
        dcepResult = Dcep_SerializeChannelAckMessage( &( dcepCtx ),
                                                      &( pSctpSession->packet[ 0 ] ),
                                                      &( pSctpSession->packetSize ) );

        if( dcepResult != DCEP_RESULT_OK )
        {
            LogError( ( "Dcep_SerializeChannelAckMessage failed!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        memset( &( pSctpSession->spa ), 0x00, sizeof( struct sctp_sendv_spa ) );
        pSctpSession->spa.sendv_flags |= SCTP_SEND_SNDINFO_VALID;
        pSctpSession->spa.sendv_sndinfo.snd_sid = channelId;
        pSctpSession->spa.sendv_sndinfo.snd_ppid = htonl( SCTP_PPID_DCEP );

        if( usrsctp_sendv( pSctpSession->socket,
                           &( pSctpSession->packet[ 0 ] ),
                           pSctpSession->packetSize,
                           NULL,
                           0,
                           &( pSctpSession->spa ),
                           sizeof( pSctpSession->spa ),
                           SCTP_SENDV_SPA,
                           0 ) <= 0 )
        {
            LogError( ( "usrsctp_sendv failed!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

SctpUtilsResult_t Sctp_Init( void )
{
    SctpUtilsResult_t retStatus = SCTP_UTILS_RESULT_OK;

    usrsctp_init( 0, &OnSctpOutboundPacket, NULL );

    /* Disable Explicit Congestion Notification. */
    usrsctp_sysctl_set_sctp_ecn_enable( 0 );

    return retStatus;
}
/*-----------------------------------------------------------*/

/* De initialize the SCTP stack. */
void Sctp_DeInit( void )
{
    /* Need to block until usrsctp_finish or sctp thread could be calling free
     * objects and cause segfault. */
    while( usrsctp_finish() != 0 )
    {
        vTaskDelay( pdMS_TO_TICKS( SCTP_TEARDOWN_POLLING_INTERVAL_MSEC ) );
    }
}
/*-----------------------------------------------------------*/

/* Create the SCTP session by connecting to the remote socket. */
SctpUtilsResult_t Sctp_CreateSession( SctpSession_t * pSctpSession,
                                      uint8_t isServer )
{
    SctpUtilsResult_t retStatus = SCTP_UTILS_RESULT_OK;
    struct sockaddr_conn localConn, remoteConn;
    struct sctp_paddrparams params;
    int32_t connectStatus = 0;

    if( pSctpSession == NULL )
    {
        retStatus = SCTP_UTILS_RESULT_BAD_PARAM;
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        memset( &( params ), 0x00, sizeof( struct sctp_paddrparams ) );
        memset( &( localConn ), 0x00, sizeof( struct sockaddr_conn ) );
        memset( &( remoteConn ), 0x00, sizeof( struct sockaddr_conn ) );

        localConn.sconn_family = AF_CONN;
        localConn.sconn_port = ntohs( SCTP_ASSOCIATION_DEFAULT_PORT );
        localConn.sconn_addr = pSctpSession;

        remoteConn.sconn_family = AF_CONN;
        remoteConn.sconn_port = ntohs( SCTP_ASSOCIATION_DEFAULT_PORT );
        remoteConn.sconn_addr = pSctpSession;

        pSctpSession->socket = usrsctp_socket( AF_CONN,
                                               SOCK_STREAM,
                                               IPPROTO_SCTP,
                                               &OnSctpInboundPacket,
                                               NULL,
                                               0,
                                               pSctpSession );
        if( pSctpSession->socket == NULL )
        {
            LogError( ( "usrsctp_socket failed to create socket!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        usrsctp_register_address( pSctpSession );
        if( ConfigureSctpSocket( pSctpSession->socket ) != SCTP_UTILS_RESULT_OK )
        {
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        if( usrsctp_bind( pSctpSession->socket,
                          ( struct sockaddr * ) &( localConn ),
                          sizeof( localConn ) ) != 0 )
        {
            LogError( ( "usrsctp_bind failed!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        connectStatus = usrsctp_connect( pSctpSession->socket,
                                         ( struct sockaddr * ) &( remoteConn ),
                                         sizeof( remoteConn ) );
        if( !( ( connectStatus >= 0 ) || ( errno == EINPROGRESS ) ) )
        {
            LogError( ( "usrsctp_connect failed!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        memcpy( &( params.spp_address ), &( remoteConn ), sizeof( remoteConn ) );
        params.spp_flags = SPP_PMTUD_DISABLE;
        params.spp_pathmtu = SCTP_MTU;
        if( usrsctp_setsockopt( pSctpSession->socket,
                                IPPROTO_SCTP,
                                SCTP_PEER_ADDR_PARAMS,
                                &( params ),
                                sizeof( params ) ) != 0 )
        {
            LogError( ( "usrsctp_setsockopt failed: IPPROTO_SCTP, SCTP_PEER_ADDR_PARAMS!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        if( isServer == 0 )
        {
            pSctpSession->currentChannelId = 0;
        }
        else
        {
            pSctpSession->currentChannelId = 1;
        }

        pSctpSession->shutdownStatus = SCTP_SESSION_ACTIVE;
    }

    if( retStatus != SCTP_UTILS_RESULT_OK )
    {
        Sctp_FreeSession( pSctpSession );
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

SctpUtilsResult_t Sctp_FreeSession( SctpSession_t * pSctpSession )
{
    SctpUtilsResult_t retStatus = SCTP_UTILS_RESULT_OK;
    uint64_t shutdownTimeout;

    if( pSctpSession == NULL )
    {
        retStatus = SCTP_UTILS_RESULT_BAD_PARAM;
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        if( pSctpSession->shutdownStatus == SCTP_SESSION_ACTIVE )
        {
            usrsctp_deregister_address( pSctpSession );
    
            /* handle issue mentioned here: https://github.com/sctplab/usrsctp/issues/147
             * the change in shutdownStatus will trigger OnSctpOutboundPacket to
             * return -1. */
            pSctpSession->shutdownStatus = SCTP_SESSION_SHUTDOWN_INITIATED;
    
            if( pSctpSession->socket != NULL )
            {
                usrsctp_set_ulpinfo( pSctpSession->socket, NULL );
                usrsctp_shutdown( pSctpSession->socket, SHUT_RDWR );
                usrsctp_close( pSctpSession->socket );
            }
    
            shutdownTimeout = NetworkingUtils_GetCurrentTimeUs( NULL ) +
                              SECONDS_TO_USEC( SCTP_SHUTDOWN_TIMEOUT_SEC );
            while( ( pSctpSession->shutdownStatus != SCTP_SESSION_SHUTDOWN_COMPLETED ) &&
                   ( NetworkingUtils_GetCurrentTimeUs( NULL ) < shutdownTimeout ) )
            {
                vTaskDelay( pdMS_TO_TICKS( SCTP_TEARDOWN_POLLING_INTERVAL_MSEC ) );
            }
        }
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

/* Pass a decrypted DTLS packet to the SCTP stack for further processing
 * of SCTP specific stuff. */
SctpUtilsResult_t Sctp_ProcessMessage( SctpSession_t * pSctpSession,
                                       uint8_t * pBuf,
                                       uint32_t bufLen )
{
    SctpUtilsResult_t retStatus = SCTP_UTILS_RESULT_OK;

    if( ( pSctpSession == NULL ) ||
        ( pBuf == NULL ) )
    {
        retStatus = SCTP_UTILS_RESULT_BAD_PARAM;
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        usrsctp_conninput( pSctpSession, pBuf, bufLen, 0 );
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

SctpUtilsResult_t Sctp_OpenDataChannel( SctpSession_t * pSctpSession,
                                        const SctpDataChannelInitInfo_t * pDataChannelInitInfo,
                                        SctpDataChannel_t * pDataChannel )
{
    DcepContext_t dcepCtx;
    DcepResult_t dcepResult = DCEP_RESULT_OK;
    DcepChannelOpenMessage_t dcepChannelOpenMessage;
    SctpUtilsResult_t retStatus = SCTP_UTILS_RESULT_OK;
    uint16_t channelId = 0;

    if( ( pSctpSession == NULL ) ||
        ( pDataChannelInitInfo == NULL ) ||
        ( pDataChannel == NULL ) )
    {
        retStatus = SCTP_UTILS_RESULT_BAD_PARAM;
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        dcepResult = Dcep_Init( &( dcepCtx ) );

        if( dcepResult != DCEP_RESULT_OK )
        {
            LogError( ( "Dcep_Init failed!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        memset( &( dcepChannelOpenMessage ), 0, sizeof( DcepChannelOpenMessage_t ) );

        dcepChannelOpenMessage.channelType = pDataChannelInitInfo->channelType;
        dcepChannelOpenMessage.numRetransmissions = pDataChannelInitInfo->numRetransmissions;
        dcepChannelOpenMessage.maxLifetimeInMilliseconds = pDataChannelInitInfo->maxLifetimeInMilliseconds;

        dcepChannelOpenMessage.pChannelName = ( const uint8_t * ) pDataChannelInitInfo->pChannelName;
        dcepChannelOpenMessage.channelNameLength = pDataChannelInitInfo->channelNameLen;

        dcepChannelOpenMessage.pProtocol = NULL;
        dcepChannelOpenMessage.protocolLength = 0;

        memset( &( pSctpSession->packet[ 0 ] ), 0x00, sizeof( pSctpSession->packet ) );
        pSctpSession->packetSize = sizeof( pSctpSession->packet );

        dcepResult = Dcep_SerializeChannelOpenMessage( &( dcepCtx ),
                                                       &( dcepChannelOpenMessage ),
                                                       &( pSctpSession->packet[ 0 ] ),
                                                       &( pSctpSession->packetSize ) );

        if( dcepResult != DCEP_RESULT_OK )
        {
            LogError( ( "Dcep_SerializeChannelOpenMessage failed!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        channelId = pSctpSession->currentChannelId;
        pSctpSession->currentChannelId += 2;

        memset( &( pSctpSession->spa ), 0x00, sizeof( struct sctp_sendv_spa ) );
        pSctpSession->spa.sendv_flags |= SCTP_SEND_SNDINFO_VALID;
        pSctpSession->spa.sendv_sndinfo.snd_sid = channelId;
        pSctpSession->spa.sendv_sndinfo.snd_ppid = htonl( SCTP_PPID_DCEP );

        if( usrsctp_sendv( pSctpSession->socket,
                           &( pSctpSession->packet[ 0 ] ),
                           pSctpSession->packetSize,
                           NULL,
                           0,
                           &( pSctpSession->spa ),
                           sizeof( pSctpSession->spa ),
                           SCTP_SENDV_SPA,
                           0 ) <= 0 )
        {
            LogError( ( "usrsctp_sendv failed!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        pDataChannel->channelType = pDataChannelInitInfo->channelType;
        pDataChannel->numRetransmissions = pDataChannelInitInfo->numRetransmissions;
        pDataChannel->maxLifetimeInMilliseconds = pDataChannelInitInfo->maxLifetimeInMilliseconds;
        pDataChannel->channelId = channelId;
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

/* Write SCTP message to the given session and stream. */
SctpUtilsResult_t Sctp_SendMessage( SctpSession_t * pSctpSession,
                                    const SctpDataChannel_t * pDataChannel,
                                    uint8_t isBinary,
                                    uint8_t * pMessage,
                                    uint32_t messageLen )
{
    SctpUtilsResult_t retStatus = SCTP_UTILS_RESULT_OK;

    if( ( pSctpSession == NULL ) ||
        ( pDataChannel == NULL ) ||
        ( pMessage == NULL ) )
    {
        retStatus = SCTP_UTILS_RESULT_BAD_PARAM;
    }


    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        memset( &( pSctpSession->spa ), 0x00, sizeof( struct sctp_sendv_spa ) );

        pSctpSession->spa.sendv_flags |= SCTP_SEND_SNDINFO_VALID;
        pSctpSession->spa.sendv_sndinfo.snd_sid = pDataChannel->channelId;

        if( ( pDataChannel->channelType == DCEP_DATA_CHANNEL_RELIABLE_UNORDERED ) ||
            ( pDataChannel->channelType == DCEP_DATA_CHANNEL_PARTIAL_RELIABLE_REXMIT_UNORDERED ) ||
            ( pDataChannel->channelType == DCEP_DATA_CHANNEL_PARTIAL_RELIABLE_TIMED_UNORDERED ) )
        {
            pSctpSession->spa.sendv_sndinfo.snd_flags |= SCTP_UNORDERED;
        }

        if( ( pDataChannel->channelType == DCEP_DATA_CHANNEL_PARTIAL_RELIABLE_REXMIT ) ||
            ( pDataChannel->channelType == DCEP_DATA_CHANNEL_PARTIAL_RELIABLE_REXMIT_UNORDERED ) )
        {
            pSctpSession->spa.sendv_prinfo.pr_policy = SCTP_PR_SCTP_RTX;
            pSctpSession->spa.sendv_prinfo.pr_value = pDataChannel->numRetransmissions;
        }

        if( ( pDataChannel->channelType == DCEP_DATA_CHANNEL_PARTIAL_RELIABLE_TIMED ) ||
            ( pDataChannel->channelType == DCEP_DATA_CHANNEL_PARTIAL_RELIABLE_TIMED_UNORDERED ) )
        {
            pSctpSession->spa.sendv_prinfo.pr_policy = SCTP_PR_SCTP_TTL;
            pSctpSession->spa.sendv_prinfo.pr_value = pDataChannel->maxLifetimeInMilliseconds;
        }

        pSctpSession->spa.sendv_sndinfo.snd_ppid = isBinary ? htonl( SCTP_PPID_BINARY ) : htonl( SCTP_PPID_STRING );

        if( usrsctp_sendv( pSctpSession->socket,
                           pMessage,
                           messageLen,
                           NULL,
                           0,
                           &( pSctpSession->spa ),
                           sizeof( pSctpSession->spa ),
                           SCTP_SENDV_SPA,
                           0 ) <= 0 )
        {
            LogError( ( "usrsctp_sendv failed!" ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

SctpUtilsResult_t Sctp_CloseDataChannel( SctpSession_t * pSctpSession,
                                         const SctpDataChannel_t * pDataChannel )
{
    SctpUtilsResult_t retStatus = SCTP_UTILS_RESULT_OK;
    struct sctp_reset_streams * pSrs;
    uint8_t srsBuffer[ 64 ] = { 0 };
    size_t len;

    if( ( pSctpSession == NULL ) ||
        ( pDataChannel == NULL ) )
    {
        retStatus = SCTP_UTILS_RESULT_BAD_PARAM;
    }

    if( retStatus == SCTP_UTILS_RESULT_OK )
    {
        len = sizeof( sctp_assoc_t ) + ( ( 2 + 1 ) * sizeof( uint16_t ) );
        pSrs = ( struct sctp_reset_streams * ) &( srsBuffer[ 0 ] );

        pSrs->srs_flags = SCTP_STREAM_RESET_OUTGOING;
        pSrs->srs_number_streams = 1U;
        pSrs->srs_stream_list[ 0 ] = pDataChannel->channelId;

        if( usrsctp_setsockopt( pSctpSession->socket,
                                IPPROTO_SCTP,
                                SCTP_RESET_STREAMS,
                                pSrs,
                                ( socklen_t ) len ) < 0 )
        {
            LogDebug( ( "Failed to reset SCTP streams - this is expected if viewer was already closed." ) );
            retStatus = SCTP_UTILS_RESULT_FAIL;
        }
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

#endif /* ENABLE_SCTP_DATA_CHANNEL */
