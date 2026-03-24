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

#ifndef DATA_CHANNEL_SCTP_H
#define DATA_CHANNEL_SCTP_H

/* Standard includes. */
#include <stdbool.h>

/* libusrsctp includes. */
#define INET  1
#define INET6 1
#include <usrsctp.h>

/* DCEP includes. */
#include "dcep_api.h"

/*-----------------------------------------------------------*/

#define SCTP_SHUTDOWN_TIMEOUT_SEC           ( 2 )
#define SCTP_TEARDOWN_POLLING_INTERVAL_MSEC ( 10 )
#define MAX_DATA_CHANNEL_NAME_LEN           255
#define MAX_DATA_CHANNEL_PROTOCOL_LEN       255
#define SCTP_MAX_PACKET_LENGTH              ( DCEP_HEADER_LENGTH +                  \
                                              MAX_DATA_CHANNEL_NAME_LEN +           \
                                              MAX_DATA_CHANNEL_PROTOCOL_LEN + 2 )

/*-----------------------------------------------------------*/

typedef enum SctpUtilsResult
{
    SCTP_UTILS_RESULT_OK = 0,
    SCTP_UTILS_RESULT_BAD_PARAM,
    SCTP_UTILS_RESULT_FAIL
} SctpUtilsResult_t;

/*-----------------------------------------------------------*/

/*
 * Callback that is fired when SCTP Association wishes to send packet.
 */
typedef void ( * SctpSessionOutboundPacket_t )( void * pUserData,
                                                uint8_t * pPacket,
                                                uint32_t packetLength );

/*
 * Callback that is fired when SCTP has received a valid DATA_CHANNEL_OPEN
 * message.
 */
typedef void ( * SctpSessionDataChannelOpen_t )( void * pUserData,
                                                 uint16_t channelId,
                                                 const uint8_t * pChannelName,
                                                 uint16_t channelNameLength );

/*
 * Callback that is fired when SCTP has received a DATA_CHANNEL_ACK message.
 *
 * Return SCTP_UTILS_RESULT_OK if the required resources for the channel are
 * successfully allocated. Return SCTP_UTILS_RESULT_FAIL otherwise.
 */
typedef SctpUtilsResult_t ( * SctpSessionDataChannelAck_t )( void * pUserData,
                                                             uint16_t channelId );

/*
 * Callback that is fired when SCTP has received a DataChannel Message.
 */
typedef void ( * SctpSessionDataChannelMessage_t )( void * pUserData,
                                                    uint16_t channelId,
                                                    uint8_t isBinary,
                                                    uint8_t * pData,
                                                    uint32_t dataLength );

/*-----------------------------------------------------------*/

typedef struct SctpSessionCallbacks
{
    void * pUserData;
    SctpSessionOutboundPacket_t outboundPacketCallback;
    SctpSessionDataChannelOpen_t dataChannelOpenCallback;
    SctpSessionDataChannelAck_t dataChannelOpenAckCallback;
    SctpSessionDataChannelMessage_t dataChannelMessageCallback;
} SctpSessionCallbacks_t;

typedef struct SctpSession
{
    volatile size_t shutdownStatus;
    struct socket * socket;
    struct sctp_sendv_spa spa;

    uint8_t packet[ SCTP_MAX_PACKET_LENGTH ];
    size_t packetSize;

    SctpSessionCallbacks_t sctpSessionCallbacks;
    uint16_t currentChannelId;
} SctpSession_t;

typedef struct SctpDataChannel
{
    DcepChannelType_t channelType;
    uint32_t numRetransmissions;
    uint32_t maxLifetimeInMilliseconds;
    uint16_t channelId;
} SctpDataChannel_t;

typedef struct SctpDataChannelInitInfo
{
    DcepChannelType_t channelType;
    uint32_t numRetransmissions;
    uint32_t maxLifetimeInMilliseconds;
    const char * pChannelName;
    size_t channelNameLen;
} SctpDataChannelInitInfo_t;

/*-----------------------------------------------------------*/

SctpUtilsResult_t Sctp_Init( void );

void Sctp_DeInit( void );

SctpUtilsResult_t Sctp_CreateSession( SctpSession_t * pSctpSession,
                                      uint8_t isServer );

SctpUtilsResult_t Sctp_FreeSession( SctpSession_t * pSctpSession );

SctpUtilsResult_t Sctp_ProcessMessage( SctpSession_t * pSctpSession,
                                       uint8_t * pBuf,
                                       uint32_t bufLen );

SctpUtilsResult_t Sctp_OpenDataChannel( SctpSession_t * pSctpSession,
                                        const SctpDataChannelInitInfo_t * pDataChannelInitInfo,
                                        SctpDataChannel_t * pDataChannel );

SctpUtilsResult_t Sctp_SendMessage( SctpSession_t * pSctpSession,
                                    const SctpDataChannel_t * pDataChannel,
                                    uint8_t isBinary,
                                    uint8_t * pMessage,
                                    uint32_t messageLen );

SctpUtilsResult_t Sctp_CloseDataChannel( SctpSession_t * pSctpSession,
                                         const SctpDataChannel_t * pDataChannel );

/*-----------------------------------------------------------*/

#endif /* DATA_CHANNEL_SCTP_H */
