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

#ifndef METRIC_H
#define METRIC_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <inttypes.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "semphr.h"

typedef enum MetricEvent
{
    METRIC_EVENT_NONE = 0,

    /* Media Events. */
    METRIC_EVENT_MEDIA_PORT_START,
    METRIC_EVENT_MEDIA_PORT_STOP,

    /* Signaling Events */
    METRIC_EVENT_SIGNALING_DESCRIBE_CHANNEL,
    METRIC_EVENT_SIGNALING_GET_ENDPOINTS,
    METRIC_EVENT_SIGNALING_GET_ICE_SERVER_LIST,
    METRIC_EVENT_SIGNALING_CONNECT_WSS_SERVER,
    METRIC_EVENT_SIGNALING_GET_CREDENTIALS,
    METRIC_EVENT_SIGNALING_JOIN_STORAGE_SESSION,

    /* ICE Events. */
    METRIC_EVENT_ICE_GATHER_HOST_CANDIDATES,
    METRIC_EVENT_ICE_GATHER_SRFLX_CANDIDATES,
    METRIC_EVENT_ICE_GATHER_RELAY_CANDIDATES,
    METRIC_EVENT_ICE_FIND_P2P_CONNECTION,

    /* Peer Connection Events. */
    METRIC_EVENT_PC_DTLS_HANDSHAKING,

    /* Combine case. */
    METRIC_EVENT_SENDING_FIRST_FRAME,

    METRIC_EVENT_MAX,
} MetricEvent_t;

typedef enum MetricEventState
{
    METRIC_EVENT_STATE_NONE = 0,
    METRIC_EVENT_STATE_RECORDING,
    METRIC_EVENT_STATE_RECORDED,
} MetricEventState_t;

typedef struct MetricEventRecord
{
    MetricEventState_t state;
    uint64_t startTimeUs;
    uint64_t endTimeUs;
} MetricEventRecord_t;

typedef struct MetricContext
{
    uint8_t isInit;
    MetricEventRecord_t eventRecords[ METRIC_EVENT_MAX ];
    SemaphoreHandle_t mutex;
} MetricContext_t;

void Metric_Init( void );
void Metric_StartEvent( MetricEvent_t event );
void Metric_EndEvent( MetricEvent_t event );
void Metric_PrintMetrics( void );
void Metric_ResetEvent( void );

#ifdef __cplusplus
}
#endif

#endif /* METRIC_H */