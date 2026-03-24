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

#ifndef PEER_CONNECTION_H
#define PEER_CONNECTION_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>

#include "peer_connection_data_types.h"
#include "sdp_controller_data_types.h"

PeerConnectionResult_t PeerConnection_Init( PeerConnectionSession_t * pSession,
                                            PeerConnectionSessionConfiguration_t * pSessionConfig );
PeerConnectionResult_t PeerConnection_Start( PeerConnectionSession_t * pSession );
PeerConnectionResult_t PeerConnection_AddTransceiver( PeerConnectionSession_t * pSession,
                                                      Transceiver_t * pTransceiver );
#if ENABLE_SCTP_DATA_CHANNEL
/* Enable data channel to be included in the SDP offer. */
PeerConnectionResult_t PeerConnection_AddDataChannel( PeerConnectionSession_t * pSession );
#endif /* ENABLE_SCTP_DATA_CHANNEL */

#if ENABLE_TWCC_SUPPORT
/* Enable TWCC Bandwidth Estimation Callback to change the media bitrates along side tith the Handler Callback */
PeerConnectionResult_t PeerConnection_SetSenderBandwidthEstimationCallback( PeerConnectionSession_t * pSession,
                                                                            OnBandwidthEstimationCallback_t onBandwidthEstimationCallback,
                                                                            void * pUserContext );
#endif /* ENABLE_TWCC_SUPPORT */

PeerConnectionResult_t PeerConnection_MatchTransceiverBySsrc( PeerConnectionSession_t * pSession,
                                                              uint32_t ssrc,
                                                              const Transceiver_t ** ppTransceiver );
PeerConnectionResult_t PeerConnection_SetLocalDescription( PeerConnectionSession_t * pSession,
                                                           const PeerConnectionBufferSessionDescription_t * pBufferSessionDescription );
PeerConnectionResult_t PeerConnection_SetRemoteDescription( PeerConnectionSession_t * pSession,
                                                            const PeerConnectionBufferSessionDescription_t * pBufferSessionDescription );
PeerConnectionResult_t PeerConnection_SetVideoOnFrame( PeerConnectionSession_t * pSession,
                                                       OnFrameReadyCallback_t onFrameReadyCallbackFunc,
                                                       void * pOnFrameReadyCallbackCustomContext );
PeerConnectionResult_t PeerConnection_SetAudioOnFrame( PeerConnectionSession_t * pSession,
                                                       OnFrameReadyCallback_t onFrameReadyCallbackFunc,
                                                       void * pOnFrameReadyCallbackCustomContext );
PeerConnectionResult_t PeerConnection_AddRemoteCandidate( PeerConnectionSession_t * pSession,
                                                          const char * pDecodeMessage,
                                                          size_t decodeMessageLength );
PeerConnectionResult_t PeerConnection_CloseSession( PeerConnectionSession_t * pSession );
PeerConnectionResult_t PeerConnection_WriteFrame( PeerConnectionSession_t * pSession,
                                                  Transceiver_t * pTransceiver,
                                                  const PeerConnectionFrame_t * pFrame );
PeerConnectionResult_t PeerConnection_CreateAnswer( PeerConnectionSession_t * pSession,
                                                    PeerConnectionBufferSessionDescription_t * pOutputBufferSessionDescription,
                                                    char * pOutputSerializedSdpMessage,
                                                    size_t * pOutputSerializedSdpMessageLength );
PeerConnectionResult_t PeerConnection_CreateOffer( PeerConnectionSession_t * pSession,
                                                   PeerConnectionBufferSessionDescription_t * pOutputBufferSessionDescription,
                                                   char * pOutputSerializedSdpMessage,
                                                   size_t * pOutputSerializedSdpMessageLength );
PeerConnectionResult_t PeerConnection_SetOnLocalCandidateReady( PeerConnectionSession_t * pSession,
                                                                OnIceCandidateReadyCallback_t onLocalCandidateReadyCallbackFunc,
                                                                void * pOnLocalCandidateReadyCallbackCustomContext );
PeerConnectionResult_t PeerConnection_AddIceServerConfig( PeerConnectionSession_t * pSession,
                                                          PeerConnectionSessionConfiguration_t * pSessionConfig );
PeerConnectionResult_t PeerConnection_SetPictureLossIndicationCallback( PeerConnectionSession_t * pSession,
                                                                        OnPictureLossIndicationCallback_t onPictureLossIndicationCallback,
                                                                        void * pUserContext );

#ifdef __cplusplus
}
#endif

#endif /* PEER_CONNECTION_H */
