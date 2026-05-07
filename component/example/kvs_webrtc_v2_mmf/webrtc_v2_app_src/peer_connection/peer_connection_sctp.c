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
#include <assert.h>

#if ENABLE_SCTP_DATA_CHANNEL

#include "ice_controller_data_types.h"
#include "peer_connection_sctp.h"
/*-----------------------------------------------------------*/

static PeerConnectionDataChannel_t globalDataChannels[MAX_SCTP_DATA_CHANNELS];
static int uKVSDataChannelCount = 0;
/*-----------------------------------------------------------*/

static void OnSCTPSessionOutboundPacket( void * customData,
                                         uint8_t * pPacket,
                                         uint32_t packetLen );
static void OnSCTPSessionDataChannelOpen( void * customData,
                                          uint16_t channelId,
                                          const uint8_t * pName,
                                          uint16_t nameLen );
static void OnSCTPSessionDataChannelMessage( void * customData,
                                             uint16_t channelId,
                                             uint8_t isBinary,
                                             uint8_t * pMessage,
                                             uint32_t pMessageLen );
static SctpUtilsResult_t OnSCTPSessionDataChannelAckOpen( void * customData,
                                                          uint16_t channelId );

/*-----------------------------------------------------------*/

/* Allocate a SCTP data channel from the global array of data channel */
PeerConnectionDataChannel_t * PeerConnectionSCTP_AllocateDataChannel( void )
{

    PeerConnectionDataChannel_t * pChannel = NULL;

    if( uKVSDataChannelCount < MAX_SCTP_DATA_CHANNELS )
    {
        uint32_t ulIter = 0;
        for(; ulIter < MAX_SCTP_DATA_CHANNELS; ulIter++ )
        {
            if( globalDataChannels[ulIter].ucChannelActive == 0 )
            {
                pChannel = &globalDataChannels[ulIter];
                memset( pChannel, 0, sizeof( PeerConnectionDataChannel_t ) );
                globalDataChannels[ulIter].ucChannelActive = 1;
                uKVSDataChannelCount++;
                break;
            }

        }
    }

    return pChannel;

}
/*-----------------------------------------------------------*/

/* Free the data channel */
PeerConnectionResult_t PeerConnectionSCTP_DeallocateDataChannel( PeerConnectionDataChannel_t * pChannel )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    if( pChannel == NULL )
    {
        LogError( ( "Invalid input, pChannel: %p", pChannel ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        pChannel->ucChannelActive = 0;
        if( uKVSDataChannelCount > 0 )
        {
            uKVSDataChannelCount--;
        }
    }

    return ret;

}
/*-----------------------------------------------------------*/

/* Create and configure a data channel that will be established once
 * SCTP session is active. */
PeerConnectionResult_t PeerConnectionSCTP_CreateDataChannel( PeerConnectionSession_t * pSession,
                                                             char * pcDataChannelName,
                                                             SctpDataChannelInitInfo_t * pDataChannelInitInfo,
                                                             PeerConnectionDataChannel_t ** ppChannel )
{

    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    PeerConnectionDataChannel_t * pChannel;
    PeerConnectionDataChannel_t * pxDataChannelIterator = NULL;

    if( ( pSession == NULL ) || ( ppChannel == NULL ) || ( pcDataChannelName == NULL ) )
    {
        LogError( ( "Invalid input." ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        pChannel = PeerConnectionSCTP_AllocateDataChannel();
        if( pChannel == NULL )
        {
            LogError( ( "No free data channel available input." ) );
            ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
        }
        else if( pSession->uKvsDataChannelCount < PEER_CONNECTION_MAX_SCTP_DATA_CHANNELS_PER_PEER )
        {
            /* Set channel name */
            strncpy( pChannel->ucDataChannelName, pcDataChannelName, MAX_DATA_CHANNEL_NAME_LEN );

            /* Set channel settings */
            if( pDataChannelInitInfo != NULL )
            {
                memcpy( &( pChannel->dataChannelInitInfo ), pDataChannelInitInfo, sizeof( SctpDataChannelInitInfo_t ) );
            }
            else
            {
                memset( &( pChannel->dataChannelInitInfo ), 0, sizeof( SctpDataChannelInitInfo_t ) );
                /* Use default. */
                pChannel->dataChannelInitInfo.channelType = DCEP_DATA_CHANNEL_RELIABLE;
                pChannel->dataChannelInitInfo.maxLifetimeInMilliseconds = 0;
                pChannel->dataChannelInitInfo.numRetransmissions = 0;
            }
            pChannel->pPeerConnection = pSession;

            if( pSession->pDataChannels == NULL )
            {
                /* No other data channels are defined yet - so this is the first in the
                 * list. */
                pChannel->pxNext = NULL;
                pSession->pDataChannels = pChannel;
            }
            else
            {
                pxDataChannelIterator = pSession->pDataChannels;
                for( ; ; )
                {
                    if( pxDataChannelIterator == pChannel )
                    {
                        /* This data channel has already been added to the list. */
                        break;
                    }

                    if( pxDataChannelIterator->pxNext == NULL )
                    {
                        pChannel->pxNext = NULL;
                        pxDataChannelIterator->pxNext = pChannel;
                        break;
                    }

                    pxDataChannelIterator = pxDataChannelIterator->pxNext;
                }
            }
            pSession->uKvsDataChannelCount++;

            *ppChannel = pChannel;
        }
        else
        {
            LogError( ( "Sessions has more than PEER_CONNECTION_MAX_SCTP_DATA_CHANNELS_PER_PEER data channels opened." ) );
            ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
        }
    }

    return ret;
}
/*-----------------------------------------------------------*/

/* Send string data to remote through a given data channel */
PeerConnectionResult_t PeerConnectionSCTP_DataChannelSend( PeerConnectionDataChannel_t * pChannel,
                                                           uint8_t isBinary,
                                                           uint8_t * pMessage,
                                                           uint32_t pMessageLen )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    SctpSession_t * pSctpSession;


    if( ( pMessage == NULL ) || ( pChannel == NULL ) )
    {
        LogError( ( "No message or pDataChannel received in PeerConnectionSCTP_DataChannelSend" ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {

        pSctpSession = &( pChannel->pPeerConnection->sctpSession );

        if( Sctp_SendMessage( pSctpSession, &( pChannel->dataChannel ), isBinary, pMessage, pMessageLen ) != SCTP_UTILS_RESULT_OK )
        {
            LogError( ( "SCTP_WriteMessageSCTPSession error" ) );
            ret = PEER_CONNECTION_RESULT_FAIL_SCTP_WRITE;
        }
    }

    return ret;
}
/*-----------------------------------------------------------*/

#if ( DATACHANNEL_CUSTOM_CALLBACK_HOOK == 0 )

/* Default on data channel message callback */
static void OnDataChannelMessage( PeerConnectionDataChannel_t * pDataChannel,
                                  uint8_t isBinary,
                                  uint8_t * pMessage,
                                  uint32_t pMessageLen )
{
    char ucSendMessage[DEFAULT_DATA_CHANNEL_ON_MESSAGE_BUFFER_SIZE];
    PeerConnectionResult_t retStatus = PEER_CONNECTION_RESULT_OK;
    if( ( pMessage == NULL ) || ( pDataChannel == NULL ) )
    {
        LogError( ( "No message or pDataChannel received in onDataChannelMessage" ) );
        return;
    }

    if( isBinary )
    {
        LogWarn( ( "=============>>>DataChannel Binary Message" ) );
    }
    else {
        LogWarn( ( "=============>>> DataChannel String Message: %.*s\n", ( int ) pMessageLen, pMessage ) );
        /* Send a response to the message sent by the viewer */
        sprintf( ucSendMessage, "Received %ld bytes, ECHO: %.*s", ( long int ) pMessageLen, ( int ) ( pMessageLen > ( DEFAULT_DATA_CHANNEL_ON_MESSAGE_BUFFER_SIZE - 128 ) ? ( DEFAULT_DATA_CHANNEL_ON_MESSAGE_BUFFER_SIZE - 128 ) : pMessageLen ), pMessage );
        retStatus = PeerConnectionSCTP_DataChannelSend( pDataChannel, 0U, ( uint8_t * ) ucSendMessage, strlen( ucSendMessage ) );
    }

    if( retStatus != PEER_CONNECTION_RESULT_OK )
    {
        LogInfo( ( "[KVS Master] dataChannelSend(): operation returned status code: 0x%08x \n", ( unsigned int ) retStatus ) );
    }

}
/*-----------------------------------------------------------*/

#endif /* (DATACHANNEL_CUSTOM_CALLBACK_HOOK == 0) */

/* Allocate SCTP and initiate session creation */
PeerConnectionResult_t PeerConnectionSCTP_AllocateSCTP( PeerConnectionSession_t * pSession )
{

    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;

    /* Create the SCTP Session */
    pSession->sctpSession.sctpSessionCallbacks.outboundPacketCallback = OnSCTPSessionOutboundPacket;
    pSession->sctpSession.sctpSessionCallbacks.dataChannelMessageCallback = OnSCTPSessionDataChannelMessage;
    pSession->sctpSession.sctpSessionCallbacks.dataChannelOpenCallback = OnSCTPSessionDataChannelOpen;
    pSession->sctpSession.sctpSessionCallbacks.dataChannelOpenAckCallback = OnSCTPSessionDataChannelAckOpen;
    pSession->sctpSession.sctpSessionCallbacks.pUserData = ( void * ) pSession;

    if( Sctp_CreateSession( &( pSession->sctpSession ), pSession->dtlsSession.isServer ) == SCTP_UTILS_RESULT_OK )
    {
        uint32_t ulChannelsCreateFailed = 0;
        PeerConnectionDataChannel_t * pxIterator = pSession->pDataChannels;

        /* Create the data channels initialized by the application, if any */
        while( pxIterator != NULL )
        {
            pxIterator->dataChannelInitInfo.pChannelName = &( pxIterator->ucDataChannelName[ 0 ] );
            pxIterator->dataChannelInitInfo.channelNameLen = strlen( pxIterator->ucDataChannelName );

            if( Sctp_OpenDataChannel( &( pSession->sctpSession ),
                                      &( pxIterator->dataChannelInitInfo ),
                                      &( pxIterator->dataChannel ) ) != SCTP_UTILS_RESULT_OK )
            {
                LogError( ( "Error creating data channel." ) );
                ulChannelsCreateFailed = 1;
            }
            else
            {
                #if DATACHANNEL_CUSTOM_CALLBACK_HOOK
                {
                    pxIterator->onDataChannelMessage = PeerConnectionSCTP_SetChannelOnMessageCallbackHook( \
                        pSession, pxIterator->dataChannel.channelId, ( uint8_t * )pxIterator->ucDataChannelName, \
                        ( uint32_t ) strlen( pxIterator->ucDataChannelName ) );
                }
                #else
                {
                    pxIterator->onDataChannelMessage = OnDataChannelMessage;
                }
                #endif /* DATACHANNEL_CUSTOM_CALLBACK_HOOK */
            }
            pxIterator = pxIterator->pxNext;
        }

        if( ulChannelsCreateFailed == 0 )
        {
            ret = PEER_CONNECTION_RESULT_OK;
        }

    }


    return ret;

}
/*-----------------------------------------------------------*/

/* Decrypt the incoming DTLS packet and feed it to the SCTP stack. */
void PeerConnectionSCTP_ProcessSCTPData( PeerConnectionSession_t * pSession,
                                         uint8_t * receiveBuffer,
                                         int readBytes )
{

    if( Sctp_ProcessMessage( &( pSession->sctpSession ), receiveBuffer, readBytes ) != SCTP_UTILS_RESULT_OK )
    {
        LogWarn( ( "Failed to process SCTP packet" ) );
    }

}
/*-----------------------------------------------------------*/

/* Get data channel that matches the given channel ID */
PeerConnectionDataChannel_t * pxGetDataChannelWithID( PeerConnectionSession_t * pSession,
                                                      uint32_t channelId )
{
    PeerConnectionDataChannel_t * pxIterator;

    pxIterator = pSession->pDataChannels;
    while( pxIterator != NULL )
    {
        if( pxIterator->dataChannel.channelId == channelId )
        {
            break;
        }
        pxIterator = pxIterator->pxNext;
    }

    return pxIterator;
}

/*-----------------------------------------------------------*/

/* Callback used to send the SCTP outbound packet coming from the SCTP stack
 * This API uses underlying crypto library to encrypt the packet before sending
 * it on the network. */
static void OnSCTPSessionOutboundPacket( void * customData,
                                         uint8_t * pPacket,
                                         uint32_t packetLen )
{

    PeerConnectionSession_t * pPeerConnectionSession = NULL;
    if( customData == NULL )
    {
        LogError( ( "No context found" ) );
        return;
    }

    pPeerConnectionSession = ( PeerConnectionSession_t * ) customData;

    if( DTLS_Send( &( pPeerConnectionSession->dtlsSession.xNetworkContext ), pPacket, packetLen ) < 0 )
    {
        LogError( ( "SCTP encrypt error" ) );
    }

}
/*-----------------------------------------------------------*/

/* A message has arrived on the given SCTP session find out the data channel
 * to which its destined to and call the target application provided callback
 * of the channel with the incoming data. */
static void OnSCTPSessionDataChannelMessage( void * customData,
                                             uint16_t channelId,
                                             uint8_t isBinary,
                                             uint8_t * pMessage,
                                             uint32_t pMessageLen )
{
    PeerConnectionSession_t * pPeerConnectionSession = ( PeerConnectionSession_t * ) customData;
    PeerConnectionDataChannel_t * pChannel = NULL;

    if( customData == NULL )
    {
        LogError( ( "No context found" ) );
        return;
    }

    pChannel = pxGetDataChannelWithID( pPeerConnectionSession, channelId );

    if( pChannel != NULL )
    {
        pChannel->onDataChannelMessage( pChannel, isBinary, pMessage, pMessageLen );
    }
    else
    {
        LogError( ( "No channel or message handler found" ) );
    }

}
/*-----------------------------------------------------------*/

/* Callback function sets data channel to open when there is a valid
 * incoming DCEP DATA_CHANNEL_ACK Message from the remote. */
static SctpUtilsResult_t OnSCTPSessionDataChannelAckOpen( void * customData,
                                                          uint16_t channelId )
{
    SctpUtilsResult_t ret = SCTP_UTILS_RESULT_OK;
    PeerConnectionSession_t * pPeerConnectionSession = ( PeerConnectionSession_t * ) customData;
    PeerConnectionDataChannel_t * pChannel = NULL;

    pChannel = pxGetDataChannelWithID( pPeerConnectionSession, channelId );

    if( pChannel != NULL )
    {
        pChannel->ucChannelOpen = 1U;
    }
    else
    {
        ret = SCTP_UTILS_RESULT_FAIL;
    }

    return ret;

}
/*-----------------------------------------------------------*/

/* Callback to allocate and initialise a data channel when there is a valid
 * incoming DCEP DATA_CHANNEL_OPEN Message from the remote. */
static void OnSCTPSessionDataChannelOpen( void * customData,
                                          uint16_t channelId,
                                          const uint8_t * pName,
                                          uint16_t nameLen )
{
    PeerConnectionSession_t * pPeerConnectionSession = ( PeerConnectionSession_t * ) customData;
    PeerConnectionDataChannel_t * pChannel = NULL;

    if( ( pPeerConnectionSession == NULL ) || ( ( uKVSDataChannelCount >= MAX_SCTP_DATA_CHANNELS ) ) )
    {
        LogError( ( "No context found or not enough data channel remaining" ) );
        return;
    }

    if( ( pPeerConnectionSession->uKvsDataChannelCount < PEER_CONNECTION_MAX_SCTP_DATA_CHANNELS_PER_PEER ) \
        && ( ( pChannel = PeerConnectionSCTP_AllocateDataChannel() ) != NULL ) )
    {
        PeerConnectionDataChannel_t * pxIterator = NULL;

        strncpy( ( pChannel->ucDataChannelName ), ( char * ) pName, nameLen );
        pChannel->pPeerConnection = pPeerConnectionSession;
        pChannel->dataChannel.channelId = channelId;

        #if DATACHANNEL_CUSTOM_CALLBACK_HOOK
        {
            pChannel->onDataChannelMessage = PeerConnectionSCTP_SetChannelOnMessageCallbackHook( pPeerConnectionSession, channelId, pName, nameLen );
        }
        #else
        {
            pChannel->onDataChannelMessage = OnDataChannelMessage;
        }
        #endif /* DATACHANNEL_CUSTOM_CALLBACK_HOOK */

        pChannel->ucChannelOpen = 1U;

        if( pPeerConnectionSession->pDataChannels == NULL )
        {
            /* No other data channels are defined yet - so this is the first in the
             * list. */
            pChannel->pxNext = NULL;
            pPeerConnectionSession->pDataChannels = pChannel;
        }
        else
        {
            pxIterator = pPeerConnectionSession->pDataChannels;
            for( ; ; )
            {
                if( pxIterator == pChannel )
                {
                    /* This data channel has already been added to the list. */
                    break;
                }

                if( pxIterator->pxNext == NULL )
                {
                    pChannel->pxNext = NULL;
                    pxIterator->pxNext = pChannel;
                    break;
                }

                pxIterator = pxIterator->pxNext;
            }
        }

        pPeerConnectionSession->uKvsDataChannelCount++;
    }
    else
    {
        LogError( ( "All %d data channel handles are open, no free handles available", MAX_SCTP_DATA_CHANNELS ) );
    }

}
/*-----------------------------------------------------------*/

PeerConnectionResult_t PeerConnectionSCTP_CloseDataChannel( PeerConnectionDataChannel_t * pChannel )
{
    PeerConnectionSession_t * pPeerConnectionSession = pChannel->pPeerConnection;
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( Sctp_CloseDataChannel( &( pPeerConnectionSession->sctpSession ), &( pChannel->dataChannel  ) ) != SCTP_UTILS_RESULT_OK )
    {
        ret = PEER_CONNECTION_RESULT_FAIL_SCTP_CLOSE;
    }
    else
    {

        PeerConnectionDataChannel_t * pxIterator = pPeerConnectionSession->pDataChannels;
        PeerConnectionDataChannel_t * pxPrev = NULL;

        pChannel->ucChannelOpen = 0U;
        while( pxIterator != NULL )
        {
            if( pxIterator == pChannel )
            {
                break;
            }
            pxPrev = pxIterator;
            pxIterator = pxIterator->pxNext;
        }

        if( pxIterator != NULL )
        {
            if( pxPrev == NULL )
            {
                pPeerConnectionSession->pDataChannels = pxIterator->pxNext;
            }
            else
            {
                pxPrev->pxNext = pxIterator->pxNext;
            }

            if( pPeerConnectionSession->uKvsDataChannelCount > 0 )
            {
                pPeerConnectionSession->uKvsDataChannelCount--;
            }
        }

        ret = PeerConnectionSCTP_DeallocateDataChannel( pChannel );
    }
    return ret;
}
/*-----------------------------------------------------------*/

/* Close all open data channels and free the SCTP session */
PeerConnectionResult_t PeerConnectionSCTP_DeallocateSCTP( PeerConnectionSession_t * pSession )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    PeerConnectionDataChannel_t * pxIterator = pSession->pDataChannels;

    while( pxIterator != NULL )
    {
        Sctp_CloseDataChannel( &( pSession->sctpSession ), &( pxIterator->dataChannel ) );
        PeerConnectionSCTP_DeallocateDataChannel( pxIterator );
        pxIterator = pxIterator->pxNext;
    }

    pSession->pDataChannels = NULL;

    if( Sctp_FreeSession( &( pSession->sctpSession ) ) != SCTP_UTILS_RESULT_OK )
    {
        ret = PEER_CONNECTION_RESULT_FAIL_SCTP_CLOSE;
    }

    return ret;

}
/*-----------------------------------------------------------*/

#endif /* ENABLE_SCTP_DATA_CHANNEL */
