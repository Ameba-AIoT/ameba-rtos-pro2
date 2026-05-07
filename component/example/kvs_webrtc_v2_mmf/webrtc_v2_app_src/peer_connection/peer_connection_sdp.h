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

#ifndef PEER_CONNECTION_SDP_H
#define PEER_CONNECTION_SDP_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>

#include "peer_connection_data_types.h"

PeerConnectionResult_t PeerConnectionSdp_DeserializeSdpMessage( PeerConnectionBufferSessionDescription_t * pBufferSessionDescription );
PeerConnectionResult_t PeerConnectionSdp_SetPayloadTypes( PeerConnectionSession_t * pSession,
                                                          PeerConnectionBufferSessionDescription_t * pRemoteBufferSessionDescription );
PeerConnectionResult_t PeerConnectionSdp_PopulateSessionDescription( PeerConnectionSession_t * pSession,
                                                                     PeerConnectionBufferSessionDescription_t * pRemoteBufferSessionDescription,
                                                                     PeerConnectionBufferSessionDescription_t * pLocalBufferSessionDescription,
                                                                     char * pOutputSerializedSdpMessage,
                                                                     size_t * pOutputSerializedSdpMessageLength );

#ifdef __cplusplus
}
#endif

#endif /* PEER_CONNECTION_SDP_H */
