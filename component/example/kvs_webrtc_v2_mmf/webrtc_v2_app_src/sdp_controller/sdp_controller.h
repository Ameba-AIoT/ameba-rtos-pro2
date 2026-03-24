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

#ifndef SDP_CONTROLLER_H
#define SDP_CONTROLLER_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "sdp_controller_data_types.h"
#include "peer_connection_data_types.h"

#define SDP_CONTROLLER_ORIGIN_DEFAULT_SESSION_NAME "-"

#define SDP_CONTROLLER_MESSAGE_TEMPLATE_HEAD "{\"type\": \"%s\", \"sdp\": \""
#define SDP_CONTROLLER_MESSAGE_TEMPLATE_TAIL "\"}"

SdpControllerResult_t SdpController_DeserializeSdpMessage( const char * pSdpContent,
                                                           size_t sdpContentLength,
                                                           SdpControllerSdpDescription_t * pSdpDescription );
SdpControllerResult_t SdpController_SerializeSdpMessageByDescription( SdpControllerMessageType_t messageType,
                                                                      SdpControllerSdpDescription_t * pSdpDescription,
                                                                      char * pOutputSerializedSdpMessage,
                                                                      size_t * pOutputSerializedSdpMessageLength );
SdpControllerResult_t SdpController_PopulateSingleMedia( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                         SdpControllerPopulateMediaConfiguration_t populateConfiguration,
                                                         SdpControllerMediaDescription_t * pLocalMediaDescription,
                                                         uint32_t currentMediaIdx,
                                                         char ** ppBuffer,
                                                         size_t * pBufferLength,
                                                         TransceiverTrackKind_t trackKind );
SdpControllerResult_t SdpController_PopulateSessionDescription( SdpControllerSdpDescription_t * pRemoteSessionDescription,
                                                                SdpControllerPopulateSessionConfiguration_t populateConfiguration,
                                                                SdpControllerSdpDescription_t * pLocalSessionDescription,
                                                                char ** ppBuffer,
                                                                size_t * pBufferLength );
                                                            

#ifdef __cplusplus
}
#endif

#endif /* SDP_CONTROLLER_H */