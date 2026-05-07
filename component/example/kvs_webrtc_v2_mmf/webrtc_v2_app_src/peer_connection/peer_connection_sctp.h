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

#ifndef PEER_CONNECTION_SCTP_H
#define PEER_CONNECTION_SCTP_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "sctp_utils.h"
#include "peer_connection.h"

#include "peer_connection_data_types.h"

#ifndef DATACHANNEL_CUSTOM_CALLBACK_HOOK
#define DATACHANNEL_CUSTOM_CALLBACK_HOOK  ( 1U )
#endif

#define DEFAULT_DATA_CHANNEL_ON_MESSAGE_BUFFER_SIZE ( 512U )


#define MASTER_DATA_CHANNEL_MESSAGE "This message is from the FreeRTOS-WebRTC-Application KVS Master"

PeerConnectionDataChannel_t * PeerConnectionSCTP_AllocateDataChannel( void );

PeerConnectionResult_t PeerConnectionSCTP_DeallocateDataChannel( PeerConnectionDataChannel_t * pChannel );

PeerConnectionResult_t PeerConnectionSCTP_CreateDataChannel( PeerConnectionSession_t * pSession,
                                                             char * pcDataChannelName,
                                                             SctpDataChannelInitInfo_t * pDataChannelInitInfo,
                                                             PeerConnectionDataChannel_t ** ppChannel );

PeerConnectionResult_t PeerConnectionSCTP_CloseDataChannel( PeerConnectionDataChannel_t * pChannel );

PeerConnectionResult_t PeerConnectionSCTP_DataChannelSend( PeerConnectionDataChannel_t * pChannel,
                                                           uint8_t isBinary,
                                                           uint8_t * pMessage,
                                                           uint32_t pMessageLen );

PeerConnectionResult_t PeerConnectionSCTP_AllocateSCTP( PeerConnectionSession_t * pSession );

PeerConnectionResult_t PeerConnectionSCTP_DeallocateSCTP( PeerConnectionSession_t * pSession );

void PeerConnectionSCTP_ProcessSCTPData( PeerConnectionSession_t * pSession,
                                         uint8_t * receiveBuffer,
                                         int readBytes );

#if ( DATACHANNEL_CUSTOM_CALLBACK_HOOK != 0 )
OnDataChannelMessageReceived_t PeerConnectionSCTP_SetChannelOnMessageCallbackHook( PeerConnectionSession_t * pPeerConnectionSession,
                                                                                   uint32_t ulChannelId,
                                                                                   const uint8_t * pucName,
                                                                                   uint32_t ulNameLen );
#endif

#ifdef __cplusplus
}
#endif
#endif /* PEER_CONNECTION_SCTP_H */
