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

#include "logging.h"
#if METRIC_PRINT_ENABLED
#include "metric.h"
#endif
#include "networking_utils.h"

#define METRIC_PRINT_INTERVAL_MS ( 10000 )

MetricContext_t context;

/* Convert event ID enum into string. */
static const char * ConvertEventToString( MetricEvent_t event );

/* Calculate the duration in miliseconds from start & end time. */
static uint64_t CalculateEventDurationMs( uint64_t startTimeUs,
                                          uint64_t endTimeUs );

static const char * ConvertEventToString( MetricEvent_t event )
{
    const char * pRet = "Unknown";
    switch( event )
    {
        case METRIC_EVENT_NONE:
            pRet = "None";
            break;
        case METRIC_EVENT_MEDIA_PORT_START:
            pRet = "Start Media Port";
            break;
        case METRIC_EVENT_MEDIA_PORT_STOP:
            pRet = "Stop Media Port";
            break;
        case METRIC_EVENT_SIGNALING_DESCRIBE_CHANNEL:
            pRet = "Describe Signaling Channel";
            break;
        case METRIC_EVENT_SIGNALING_GET_ENDPOINTS:
            pRet = "Get Signaling Endpoints";
            break;
        case METRIC_EVENT_SIGNALING_GET_ICE_SERVER_LIST:
            pRet = "Get Ice Server List";
            break;
        case METRIC_EVENT_SIGNALING_CONNECT_WSS_SERVER:
            pRet = "Connect Websocket Server";
            break;
        case METRIC_EVENT_SIGNALING_GET_CREDENTIALS:
            pRet = "Get Authentication Temporary Credentials";
            break;
        case METRIC_EVENT_ICE_GATHER_HOST_CANDIDATES:
            pRet = "Gather ICE Host Candidate";
            break;
        case METRIC_EVENT_ICE_GATHER_SRFLX_CANDIDATES:
            pRet = "Gather ICE Srflx Candidate";
            break;
        case METRIC_EVENT_ICE_GATHER_RELAY_CANDIDATES:
            pRet = "Gather ICE Relay Candidate";
            break;
        case METRIC_EVENT_SIGNALING_JOIN_STORAGE_SESSION:
            pRet = "Join Storage Session";
            break;
        case METRIC_EVENT_ICE_FIND_P2P_CONNECTION:
            pRet = "Find Peer-To-Peer Connection";
            break;
        case METRIC_EVENT_PC_DTLS_HANDSHAKING:
            pRet = "DTLS Handshaking";
            break;
        case METRIC_EVENT_SENDING_FIRST_FRAME:
            pRet = "First Frame";
            break;
        default:
            pRet = "Unknown";
            break;
    }

    return pRet;
}

static uint64_t CalculateEventDurationMs( uint64_t startTimeUs,
                                          uint64_t endTimeUs )
{
    return ( endTimeUs - startTimeUs ) / 1000;
}

void Metric_Init( void )
{
    memset( &context, 0, sizeof( MetricContext_t ) );

    context.mutex = xSemaphoreCreateMutex();
    if( context.mutex == NULL )
    {
        LogError( ( "Fail to create mutex for Metric." ) );
    }
    else
    {
        context.isInit = 1U;
    }

}

void Metric_StartEvent( MetricEvent_t event )
{
    if( ( context.isInit == 1U ) && ( event < METRIC_EVENT_MAX ) &&
        ( xSemaphoreTake( context.mutex, portMAX_DELAY ) == pdTRUE ) )
    {
        MetricEventRecord_t * pEventRecord = &context.eventRecords[ event ];

        if( pEventRecord->state == METRIC_EVENT_STATE_NONE )
        {
            pEventRecord->state = METRIC_EVENT_STATE_RECORDING;
            pEventRecord->startTimeUs = NetworkingUtils_GetCurrentTimeUs( NULL );
        }

        xSemaphoreGive( context.mutex );
    }
}

void Metric_EndEvent( MetricEvent_t event )
{
    if( ( context.isInit == 1U ) && ( event < METRIC_EVENT_MAX ) &&
        ( xSemaphoreTake( context.mutex, portMAX_DELAY ) == pdTRUE ) )
    {
        MetricEventRecord_t * pEventRecord = &context.eventRecords[ event ];

        if( pEventRecord->state == METRIC_EVENT_STATE_RECORDING )
        {
            pEventRecord->state = METRIC_EVENT_STATE_RECORDED;
            pEventRecord->endTimeUs = NetworkingUtils_GetCurrentTimeUs( NULL );
        }

        xSemaphoreGive( context.mutex );
    }
}

void Metric_PrintMetrics( void )
{
    int i;
    MetricEventRecord_t * pEventRecord;
    static char runTimeStatsBuffer[ 4096 ];

    if( ( context.isInit == 1U ) &&
        ( xSemaphoreTake( context.mutex, portMAX_DELAY ) == pdTRUE ) )
    {
        LogInfo( ( "================================ Print Metrics Start ================================" ) );
        for( i = 0; i < METRIC_EVENT_MAX; i++ )
        {
            pEventRecord = &context.eventRecords[ i ];

            if( pEventRecord->state == METRIC_EVENT_STATE_RECORDED )
            {
                LogInfo( ( "Duration of %s: %llu ms",
                           ConvertEventToString( ( MetricEvent_t )i ),
                           CalculateEventDurationMs( pEventRecord->startTimeUs, pEventRecord->endTimeUs ) ) );
            }
        }

        LogInfo( ( "Remaining free heap size: %u", xPortGetFreeHeapSize() ) );

        vTaskGetRunTimeStats( runTimeStatsBuffer );
        LogInfo( ( " == Run Time Stat Start ==\n%s\n == Run Time Stat End ==", runTimeStatsBuffer ) );
        LogInfo( ( "================================ Print Metrics End ================================" ) );

        xSemaphoreGive( context.mutex );
    }
}

void Metric_ResetEvent( void )
{
    if( ( context.isInit == 1U ) &&
        ( xSemaphoreTake( context.mutex, portMAX_DELAY ) == pdTRUE ) )
    {
        for( int i = 0; i < METRIC_EVENT_MAX; i++ )
        {
            context.eventRecords[i].state = METRIC_EVENT_STATE_NONE;
            context.eventRecords[i].endTimeUs = 0;
            context.eventRecords[i].startTimeUs = 0;
        }
        xSemaphoreGive( context.mutex );
    }
}
