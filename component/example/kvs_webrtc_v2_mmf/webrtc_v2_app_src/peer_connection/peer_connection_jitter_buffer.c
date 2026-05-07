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

#include <stdlib.h>
#include "logging.h"
#include "peer_connection.h"
#include "peer_connection_jitter_buffer.h"
#include "h264_depacketizer.h"
#include "g711_depacketizer.h"
#include "opus_depacketizer.h"
#include "peer_connection_g711_helper.h"
#include "peer_connection_h264_helper.h"
#include "peer_connection_h265_helper.h"
#include "peer_connection_opus_helper.h"
#include "FreeRTOS.h"

#define PEER_CONNECTION_JITTER_BUFFER_MAX_PACKETS_NUM_IN_A_FRAME ( 32 )
#define PEER_CONNECTION_JITTER_BUFFER_SEQ_WRAPPER_THRESHOLD ( 10 )
#define PEER_CONNECTION_JITTER_BUFFER_TIMESTAMP_WRAPPER_THRESHOLD_SEC ( 0.1 )
#define PEER_CONNECTION_JITTER_BUFFER_WRAP( x, max ) ( ( x ) % max )
#define PEER_CONNECTION_JITTER_BUFFER_INCREASE_WITH_WRAP( x, y, max ) ( PEER_CONNECTION_JITTER_BUFFER_WRAP( ( x ) + ( y ),\
                                                                                                            max ) )
#define PEER_CONNECTION_JITTER_BUFFER_DECREASE_WITH_WRAP( x, y, max ) ( PEER_CONNECTION_JITTER_BUFFER_WRAP( ( x ) - ( y ),\
                                                                                                            max ) )

static void DiscardPacket( PeerConnectionJitterBuffer_t * pJitterBuffer,
                           PeerConnectionJitterBufferPacket_t * pPacket );

static void DiscardPackets( PeerConnectionJitterBuffer_t * pJitterBuffer,
                            uint16_t startSeq,
                            uint16_t endSeq,
                            PeerConnectionJitterBufferPacket_t * pNextPacket )
{
    uint16_t i, index;
    PeerConnectionJitterBufferPacket_t * pPacket;

    if( pJitterBuffer != NULL )
    {
        /* Free from start sequence to end sequence number. */
        for( i = startSeq; i != ( uint16_t )( endSeq + 1 ); i++ )
        {
            index = PEER_CONNECTION_JITTER_BUFFER_WRAP( i,
                                                        PEER_CONNECTION_JITTER_BUFFER_MAX_ENTRY_NUM );
            pPacket = &pJitterBuffer->rtpPackets[index];
            DiscardPacket( pJitterBuffer,
                           pPacket );
        }

        if( pNextPacket != NULL )
        {
            pJitterBuffer->oldestReceivedSequenceNumber = pNextPacket->sequenceNumber;
            pJitterBuffer->lastPopRtpTimestamp = pNextPacket->rtpTimestamp;
        }
    }
}

static uint8_t IsRtpPacketExpired( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                   PeerConnectionJitterBufferPacket_t * pPacket )
{
    uint8_t isExpired = 0U;
    uint32_t earliestBufferTimestamp = 0U;

    earliestBufferTimestamp = pJitterBuffer->newestReceivedTimestamp - pJitterBuffer->tolerenceRtpTimeStamp;
    if( ( ( pJitterBuffer->newestReceivedTimestamp > earliestBufferTimestamp ) &&
          ( pPacket->rtpTimestamp < earliestBufferTimestamp ) ) ||
        ( ( pJitterBuffer->newestReceivedTimestamp < earliestBufferTimestamp ) &&
          ( pPacket->rtpTimestamp < earliestBufferTimestamp ) &&
          ( pPacket->rtpTimestamp > pJitterBuffer->newestReceivedTimestamp ) ) )
    {
        isExpired = 1U;
    }

    return isExpired;
}

static PeerConnectionResult_t ParseFramesInJitterBuffer( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                         BaseType_t isClosing )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    uint16_t i, index, prev;
    PeerConnectionJitterBufferPacket_t * pPacket;
    uint8_t isFrameDataContinuous = 1U;
    uint8_t isStart;
    int32_t popingPacketStartSeq = -1;
    int32_t firstPacketSeq = -1;

    if( pJitterBuffer == NULL )
    {
        LogError( ( "Invalid input, pJitterBuffer: %p", pJitterBuffer ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( ( pJitterBuffer->onFrameReadyCallbackFunc == NULL ) ||
             ( pJitterBuffer->onFrameDropCallbackFunc == NULL ) )
    {
        LogError( ( "Invalid input, onFrameReadyCallbackFunc: %p, onFrameDropCallbackFunc: %p",
                    pJitterBuffer->onFrameReadyCallbackFunc,
                    pJitterBuffer->onFrameDropCallbackFunc ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Note that newest sequence is probably less than oldest sequence due to wrapping. */
        firstPacketSeq = pJitterBuffer->oldestReceivedSequenceNumber;
        prev = firstPacketSeq;
        for( i = pJitterBuffer->oldestReceivedSequenceNumber; i != ( uint16_t )( pJitterBuffer->newestReceivedSequenceNumber + 1 ); i++ )
        {
            index = PEER_CONNECTION_JITTER_BUFFER_WRAP( i,
                                                        PEER_CONNECTION_JITTER_BUFFER_MAX_ENTRY_NUM );
            pPacket = &pJitterBuffer->rtpPackets[ index ];
            if( pPacket->isPushed == 0U )
            {
                isFrameDataContinuous = 0;
                if( ( isClosing == 0U ) &&
                    ( !IsRtpPacketExpired( pJitterBuffer, pPacket ) ) )
                {
                    /* Current frame is not completely received, break to wait for a while. */
                    break;
                }
            }
            else
            {
                if( pPacket->rtpTimestamp != pJitterBuffer->lastPopRtpTimestamp )
                {
                    if( ( popingPacketStartSeq != -1 ) &&
                        ( isFrameDataContinuous != 0U ) )
                    {
                        /* We now have an full frame ready between start index and end index. */
                        ret = pJitterBuffer->onFrameReadyCallbackFunc( pJitterBuffer->pOnFrameReadyCallbackContext,
                                                                       popingPacketStartSeq,
                                                                       prev );
                        DiscardPackets( pJitterBuffer,
                                        popingPacketStartSeq,
                                        prev,
                                        pPacket );
                        popingPacketStartSeq = -1;
                        firstPacketSeq = i;
                        if( ret != PEER_CONNECTION_RESULT_OK )
                        {
                            LogError( ( "Terminating parsing jitter buffer by frame ready callback function, result: %d", ret ) );
                            break;
                        }
                    }
                    else if( IsRtpPacketExpired( pJitterBuffer, pPacket ) )
                    {
                        /* data is expired, drop them */
                        ret = pJitterBuffer->onFrameDropCallbackFunc( pJitterBuffer->pOnFrameReadyCallbackContext,
                                                                      firstPacketSeq,
                                                                      prev );
                        DiscardPackets( pJitterBuffer,
                                        firstPacketSeq,
                                        prev,
                                        pPacket );
                        popingPacketStartSeq = -1;
                        firstPacketSeq = i;
                        if( ret != PEER_CONNECTION_RESULT_OK )
                        {
                            LogError( ( "Terminating parsing jitter buffer by frame drop callback function, result: %d", ret ) );
                            break;
                        }
                    }
                    else
                    {
                        /* The following packets are still in tolerence timestamp, not necessary to parse them for now. */
                        break;
                    }
                }
            }

            if( pPacket->rtpTimestamp == pJitterBuffer->lastPopRtpTimestamp )
            {
                if( ( pJitterBuffer->getPacketPropertyFunc ) &&
                    ( pJitterBuffer->getPacketPropertyFunc( pPacket,
                                                            &isStart ) == PEER_CONNECTION_RESULT_OK ) )
                {
                    if( ( popingPacketStartSeq == -1 ) &&
                        ( isStart != 0U ) )
                    {
                        popingPacketStartSeq = i;
                    }
                }
                else
                {
                    /* No get properties callback function or it returns failure. This packet is invalid, drop it. */
                    isFrameDataContinuous = 0;
                    LogInfo( ( "Fail to get property, dumping RTP payload, 0x%x 0x%x 0x%x 0x%x",
                               pPacket->pPacketBuffer[0],
                               pPacket->pPacketBuffer[1],
                               pPacket->pPacketBuffer[2],
                               pPacket->pPacketBuffer[3] ) );
                    DiscardPacket( pJitterBuffer,
                                   pPacket );
                    if( ( isClosing == 0U ) &&
                        ( !IsRtpPacketExpired( pJitterBuffer, pPacket ) ) )
                    {
                        /* Detect invalid packet */
                        break;
                    }
                }
            }

            prev = i;
        }

        /* Deal with last frame when closing */
        if( isClosing != 0U )
        {
            firstPacketSeq = pJitterBuffer->oldestReceivedSequenceNumber;
            prev = firstPacketSeq;
            for( i = pJitterBuffer->oldestReceivedSequenceNumber; i != ( uint16_t )( pJitterBuffer->newestReceivedSequenceNumber + 1 ); i++ )
            {
                index = PEER_CONNECTION_JITTER_BUFFER_WRAP( i,
                                                            PEER_CONNECTION_JITTER_BUFFER_MAX_ENTRY_NUM );
                pPacket = &pJitterBuffer->rtpPackets[ index ];
                if( pPacket->isPushed == 0U )
                {
                    ret = pJitterBuffer->onFrameDropCallbackFunc( pJitterBuffer->pOnFrameReadyCallbackContext,
                                                                  firstPacketSeq,
                                                                  prev );
                    DiscardPackets( pJitterBuffer,
                                    firstPacketSeq,
                                    prev,
                                    NULL );
                    firstPacketSeq = i;
                    if( ret != PEER_CONNECTION_RESULT_OK )
                    {
                        LogWarn( ( "Getting error return in drop frame callback function while closing jitter buffer, result: %d", ret ) );
                        continue;
                    }
                }

                prev = i;
            }

            ret = pJitterBuffer->onFrameDropCallbackFunc( pJitterBuffer->pOnFrameReadyCallbackContext,
                                                          firstPacketSeq,
                                                          prev );
            DiscardPackets( pJitterBuffer,
                            firstPacketSeq,
                            prev,
                            NULL );
            if( ret != PEER_CONNECTION_RESULT_OK )
            {
                LogWarn( ( "Getting error return in drop frame callback function while closing jitter buffer, result: %d", ret ) );
            }
        }
    }

    return ret;
}

static PeerConnectionResult_t ShouldAcceptPacket( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                  PeerConnectionJitterBufferPacket_t * pPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    uint16_t earliestTolerenceRtpSeq = 0U, lastestTolerenceRtpSeq = 0U;
    uint32_t rtpSeqOffset = 0U;

    if( ( pJitterBuffer == NULL ) || ( pPacket == NULL ) )
    {
        LogError( ( "Invalid input, pJitterBuffer: %p, pPacket: %p", pJitterBuffer, pPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        rtpSeqOffset = pJitterBuffer->capacity / 2;
        earliestTolerenceRtpSeq = ( uint16_t )( ( pJitterBuffer->newestReceivedSequenceNumber - rtpSeqOffset ) & 0xFFFF );
        lastestTolerenceRtpSeq = ( uint16_t )( ( pJitterBuffer->newestReceivedSequenceNumber + rtpSeqOffset ) & 0xFFFF );

        if( lastestTolerenceRtpSeq >= earliestTolerenceRtpSeq )
        {
            /* The most common scenario, the tolerence range is between earliest and lastest, otherwise return false. */
            if( ( pPacket->sequenceNumber < earliestTolerenceRtpSeq ) ||
                ( pPacket->sequenceNumber > lastestTolerenceRtpSeq ) )
            {
                ret = PEER_CONNECTION_RESULT_PACKET_OUTDATED;
                LogInfo( ( "Dropping packet with seq: %u because of out of tolerence seq range, earliest: %u, lastest: %u",
                           pPacket->sequenceNumber,
                           earliestTolerenceRtpSeq,
                           lastestTolerenceRtpSeq ) );
            }
        }
        else
        {
            /* In this case, the lastest tolerence sequence is wrapped. */
            if( ( pPacket->sequenceNumber < earliestTolerenceRtpSeq ) &&
                ( pPacket->sequenceNumber > lastestTolerenceRtpSeq ) )
            {
                ret = PEER_CONNECTION_RESULT_PACKET_OUTDATED;
                LogInfo( ( "Dropping packet with seq: %u because of out of tolerence seq range, earliest seq: %u, lastest seq: %u",
                           pPacket->sequenceNumber,
                           earliestTolerenceRtpSeq,
                           lastestTolerenceRtpSeq ) );
            }
        }
    }

    return ret;
}

static PeerConnectionResult_t UpdateJitterBufferAddPacket( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                           PeerConnectionJitterBufferPacket_t * pPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( ( pJitterBuffer == NULL ) || ( pPacket == NULL ) )
    {
        LogError( ( "Invalid input, pJitterBuffer: %p, pPacket: %p", pJitterBuffer, pPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Update newest sequence number. */
        if( pPacket->sequenceNumber > pJitterBuffer->newestReceivedSequenceNumber )
        {
            /* The RTP sequence number in packet is just larger than newest one. */
            pJitterBuffer->newestReceivedSequenceNumber = pPacket->sequenceNumber;
        }
        else if( ( ( pJitterBuffer->newestReceivedSequenceNumber + PEER_CONNECTION_JITTER_BUFFER_SEQ_WRAPPER_THRESHOLD ) & 0xFFFF ) <= pPacket->sequenceNumber )
        {
            /* Overflow is happening, we update the newest sequence number only if it's in the range of threshold. */
            pJitterBuffer->newestReceivedSequenceNumber = pPacket->sequenceNumber;
        }
        else
        {
            /* Do nothing for non newest packets. */
        }

        /* Update oldest sequecen number. */
        if( pPacket->sequenceNumber < pJitterBuffer->oldestReceivedSequenceNumber )
        {
            /* The RTP sequence number in packet is just older than oldest one. */
            pJitterBuffer->oldestReceivedSequenceNumber = pPacket->sequenceNumber;
        }
        else if( ( ( pJitterBuffer->oldestReceivedSequenceNumber - PEER_CONNECTION_JITTER_BUFFER_SEQ_WRAPPER_THRESHOLD ) & 0xFFFF ) <= pPacket->sequenceNumber )
        {
            /* Overflow is happening, we update the newest sequence number only if it's in the range of threshold. */
            pJitterBuffer->newestReceivedSequenceNumber = pPacket->sequenceNumber;
        }
        else
        {
            /* Do nothing for non newest packets. */
        }

        /* Update newest timestamp. */
        if( pPacket->rtpTimestamp > pJitterBuffer->newestReceivedTimestamp )
        {
            /* The RTP timestamp in packet is just larger than newest one. */
            pJitterBuffer->newestReceivedTimestamp = pPacket->rtpTimestamp;
        }
        else if( ( ( uint64_t )( pJitterBuffer->newestReceivedTimestamp + ( PEER_CONNECTION_JITTER_BUFFER_TIMESTAMP_WRAPPER_THRESHOLD_SEC * pJitterBuffer->clockRate ) ) & 0xFFFFFFFF ) <= pPacket->rtpTimestamp )
        {
            /* Overflow is happening, we update the newest timestamp only if it's in the range of threshold. */
            pJitterBuffer->newestReceivedTimestamp = pPacket->rtpTimestamp;
        }
        else
        {
            /* Do nothing for non newest packets. */
        }

        /* Update oldest timestamp. */
        if( pPacket->rtpTimestamp < pJitterBuffer->lastPopRtpTimestamp )
        {
            pJitterBuffer->lastPopRtpTimestamp = pPacket->rtpTimestamp;
        }
        else if( ( ( uint64_t )( pJitterBuffer->lastPopRtpTimestamp > pPacket->rtpTimestamp + ( PEER_CONNECTION_JITTER_BUFFER_TIMESTAMP_WRAPPER_THRESHOLD_SEC * pJitterBuffer->clockRate ) ) & 0xFFFFFFFF ) )
        {
            /* Overflow is happening, we update the last pop timestamp only if it's in the range of threshold. */
            pJitterBuffer->lastPopRtpTimestamp = pPacket->rtpTimestamp;
        }
        else
        {
            /* Empty else marker */
        }
    }

    return ret;
}

static void DiscardPacket( PeerConnectionJitterBuffer_t * pJitterBuffer,
                           PeerConnectionJitterBufferPacket_t * pPacket )
{
    ( void ) pJitterBuffer;
    if( pPacket && ( pPacket->pPacketBuffer != NULL ) )
    {
        vPortFree( pPacket->pPacketBuffer );
        memset( pPacket,
                0,
                sizeof( PeerConnectionJitterBufferPacket_t ) );
    }
}

PeerConnectionResult_t PeerConnectionJitterBuffer_Create( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                          OnJitterBufferFrameReadyCallback_t onFrameReadyCallbackFunc,
                                                          void * pOnFrameReadyCallbackContext,
                                                          OnJitterBufferFrameDropCallback_t onFrameDropCallbackFunc,
                                                          void * pOnFrameDropCallbackContext,
                                                          uint32_t tolerenceBufferSec,  // buffer time in seconds
                                                          uint32_t codec,
                                                          uint32_t clockRate )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( ( pJitterBuffer == NULL ) ||
        ( codec == 0 ) )
    {
        LogError( ( "Invalid input, pJitterBuffer: %p, codec: %lu", pJitterBuffer, codec ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( pJitterBuffer,
                0,
                sizeof( PeerConnectionJitterBuffer_t ) );
        pJitterBuffer->isStart = 0U;
        pJitterBuffer->clockRate = clockRate;
        pJitterBuffer->codec = codec;
        pJitterBuffer->capacity = PEER_CONNECTION_JITTER_BUFFER_MAX_ENTRY_NUM;
        pJitterBuffer->lastPopRtpTimestamp = 0xFFFFFFFF;
        pJitterBuffer->lastPopSequenceNumber = 0xFFFF;
        pJitterBuffer->lastPopTick = portMAX_DELAY;
        pJitterBuffer->newestReceivedSequenceNumber = 0xFFFF;
        pJitterBuffer->newestReceivedTimestamp = 0xFFFFFFFF;
        pJitterBuffer->oldestReceivedSequenceNumber = 0U;
        /* Converting tolerence buffer in seconds into RTP time stamp format. */
        pJitterBuffer->tolerenceRtpTimeStamp = tolerenceBufferSec * ( clockRate );

        pJitterBuffer->onFrameReadyCallbackFunc = onFrameReadyCallbackFunc;
        pJitterBuffer->pOnFrameReadyCallbackContext = pOnFrameReadyCallbackContext;
        pJitterBuffer->onFrameDropCallbackFunc = onFrameDropCallbackFunc;
        pJitterBuffer->pOnFrameDropCallbackContext = pOnFrameDropCallbackContext;
        LogInfo( ( "Creating jitter buffer with tolerence RTP timestamp: %lu", pJitterBuffer->tolerenceRtpTimeStamp ) );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        LogInfo( ( "Setting jitter buffer with codec: 0x%lx.", codec ) );
        /* Pick get property function based on codec. */
        if( TRANSCEIVER_IS_CODEC_ENABLED( codec,
                                          TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT ) )
        {
            pJitterBuffer->getPacketPropertyFunc = PeerConnectionH264Helper_GetH264PacketProperty;
            pJitterBuffer->fillFrameFunc = PeerConnectionH264Helper_FillFrameH264;
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( codec,
                                               TRANSCEIVER_RTC_CODEC_OPUS_BIT ) )
        {
            pJitterBuffer->getPacketPropertyFunc = PeerConnectionOpusHelper_GetOpusPacketProperty;
            pJitterBuffer->fillFrameFunc = PeerConnectionOpusHelper_FillFrameOpus;
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( codec,
                                               TRANSCEIVER_RTC_CODEC_MULAW_BIT ) )
        {
            pJitterBuffer->getPacketPropertyFunc = PeerConnectionG711Helper_GetG711PacketProperty;
            pJitterBuffer->fillFrameFunc = PeerConnectionG711Helper_FillFrameG711;
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( codec,
                                               TRANSCEIVER_RTC_CODEC_ALAW_BIT ) )
        {
            pJitterBuffer->getPacketPropertyFunc = PeerConnectionG711Helper_GetG711PacketProperty;
            pJitterBuffer->fillFrameFunc = PeerConnectionG711Helper_FillFrameG711;
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( codec,
                                               TRANSCEIVER_RTC_CODEC_H265_BIT ) )
        {
            pJitterBuffer->getPacketPropertyFunc = PeerConnectionH265Helper_GetH265PacketProperty;
            pJitterBuffer->fillFrameFunc = PeerConnectionH265Helper_FillFrameH265;
        }
        else
        {
            /* TODO: Unknown, no matching codec. */
            LogError( ( "Codec is not supported, codec bit map: 0x%x", ( int ) codec ) );
            ret = PEER_CONNECTION_RESULT_UNKNOWN_TX_CODEC;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pJitterBuffer->isInit = 1U;
    }

    return ret;
}

void PeerConnectionJitterBuffer_Free( PeerConnectionJitterBuffer_t * pJitterBuffer )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( pJitterBuffer == NULL )
    {
        LogError( ( "Invalid input, pJitterBuffer: %p",
                    pJitterBuffer ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( pJitterBuffer->isInit == 0U )
    {
        LogError( ( "Jitter buffer is not initialized yet." ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pJitterBuffer->isInit = 0U;

        ( void ) ParseFramesInJitterBuffer( pJitterBuffer,
                                            pdTRUE );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        DiscardPackets( pJitterBuffer,
                        0,
                        PEER_CONNECTION_JITTER_BUFFER_MAX_ENTRY_NUM - 1,
                        NULL );
    }
}

PeerConnectionResult_t PeerConnectionJitterBuffer_AllocateBuffer( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                                  PeerConnectionJitterBufferPacket_t ** ppOutPacket,
                                                                  size_t packetBufferSize,
                                                                  uint16_t rtpSeq )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    int index = PEER_CONNECTION_JITTER_BUFFER_WRAP( rtpSeq,
                                                    PEER_CONNECTION_JITTER_BUFFER_MAX_ENTRY_NUM );

    if( ( pJitterBuffer == NULL ) ||
        ( ppOutPacket == NULL ) )
    {
        LogError( ( "Invalid input, pJitterBuffer: %p, ppOutPacket: %p", pJitterBuffer, ppOutPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( pJitterBuffer->isInit == 0U )
    {
        LogError( ( "Jitter buffer is not initialized yet or it has been freed." ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( packetBufferSize == 0 )
    {
        LogError( ( "Invalid input, the input buffer length should be set correctly" ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        *ppOutPacket = &pJitterBuffer->rtpPackets[index];
        if( ( *ppOutPacket )->pPacketBuffer != NULL )
        {
            /* Remove old information. */
            DiscardPacket( pJitterBuffer,
                           *ppOutPacket );
        }

        ( *ppOutPacket )->pPacketBuffer = ( uint8_t * )pvPortMalloc( packetBufferSize );
        ( *ppOutPacket )->packetBufferLength = packetBufferSize;
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionJitterBuffer_GetPacket( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                             uint16_t rtpSeq,
                                                             PeerConnectionJitterBufferPacket_t ** ppOutPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    PeerConnectionJitterBufferPacket_t * pPacket = NULL;
    int index = PEER_CONNECTION_JITTER_BUFFER_WRAP( rtpSeq,
                                                    PEER_CONNECTION_JITTER_BUFFER_MAX_ENTRY_NUM );

    if( ( pJitterBuffer == NULL ) ||
        ( ppOutPacket == NULL ) )
    {
        LogError( ( "Invalid input, pJitterBuffer: %p, ppOutPacket: %p", pJitterBuffer, ppOutPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( pJitterBuffer->isInit == 0U )
    {
        LogError( ( "Jitter buffer is not initialized yet or it has been freed." ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pPacket = &pJitterBuffer->rtpPackets[index];

        if( pPacket->sequenceNumber == rtpSeq )
        {
            *ppOutPacket = pPacket;
        }
        else
        {
            LogWarn( ( "No available packets found, target seq: %u", rtpSeq ) );
            ret = PEER_CONNECTION_RESULT_FAIL_JITTER_BUFFER_SEQ_NOT_FOUND;
        }
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionJitterBuffer_Push( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                        PeerConnectionJitterBufferPacket_t * pPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( ( pJitterBuffer == NULL ) ||
        ( pPacket == NULL ) )
    {
        LogError( ( "Invalid input, pJitterBuffer: %p, pPacket: %p", pJitterBuffer, pPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( pJitterBuffer->isInit == 0U )
    {
        LogError( ( "Jitter buffer is not initialized yet or it has been freed." ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pJitterBuffer->isStart == 0U )
        {
            pJitterBuffer->isStart = 1U;
            pJitterBuffer->newestReceivedSequenceNumber = pPacket->sequenceNumber;
            pJitterBuffer->newestReceivedTimestamp = pPacket->rtpTimestamp;
            pJitterBuffer->oldestReceivedSequenceNumber = pPacket->sequenceNumber;
        }

        ret = ShouldAcceptPacket( pJitterBuffer,
                                  pPacket );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pPacket->isPushed = 1U;

        /* Update variables in jitter buffer. */
        ret = UpdateJitterBufferAddPacket( pJitterBuffer,
                                           pPacket );
    }

    if( ret != PEER_CONNECTION_RESULT_OK )
    {
        /* Remove this packet if any error happens. */
        if( pPacket && ( pPacket->pPacketBuffer != NULL ) )
        {
            vPortFree( pPacket->pPacketBuffer );
            memset( pPacket,
                    0,
                    sizeof( PeerConnectionJitterBufferPacket_t ) );
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Parse the jitter buffer to check if any frames are ready for decoding or if any packets need to be dropped. */
        ret = ParseFramesInJitterBuffer( pJitterBuffer,
                                         pdFALSE );
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionJitterBuffer_FillFrame( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                             uint16_t rtpSeqStart,
                                                             uint16_t rtpSeqEnd,
                                                             uint8_t * pOutBuffer,
                                                             size_t * pOutBufferLength,
                                                             uint32_t * pRtpTimestamp )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( ( pJitterBuffer == NULL ) ||
        ( pOutBuffer == NULL ) ||
        ( pOutBufferLength == NULL ) ||
        ( pRtpTimestamp == NULL ) )
    {
        LogError( ( "Invalid input, pJitterBuffer: %p, pOutBuffer: %p, pOutBufferLength: %p, pRtpTimestamp: %p", pJitterBuffer, pOutBuffer, pOutBufferLength, pRtpTimestamp ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( pJitterBuffer->isInit == 0U )
    {
        LogError( ( "Jitter buffer is not initialized yet or it has been freed." ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pJitterBuffer->fillFrameFunc )
        {
            ret = pJitterBuffer->fillFrameFunc( pJitterBuffer,
                                                rtpSeqStart,
                                                rtpSeqEnd,
                                                pOutBuffer,
                                                pOutBufferLength,
                                                pRtpTimestamp );
        }
        else
        {
            LogWarn( ( "No fill frame function pointer for this jitter buffer, codec: 0x%lx", pJitterBuffer->codec ) );
        }
    }

    return ret;
}
