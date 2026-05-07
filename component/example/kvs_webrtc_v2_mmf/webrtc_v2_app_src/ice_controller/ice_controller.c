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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "logging.h"
#include "ice_controller.h"
#include "ice_controller_private.h"
#include "ice_api.h"
#include "transaction_id_store.h"
#include "core_json.h"
#include "string_utils.h"
#include "mbedtls/md.h"
#include "mbedtls/md5.h"
#include "task.h"
#if METRIC_PRINT_ENABLED
#include "metric.h"
#endif
#include "networking_utils.h"

#define ICE_CONTROLLER_MESSAGE_QUEUE_NAME "/WebrtcApplicationIceController"
#define ICE_CONTROLLER_TIMER_NAME "IceControllerTimer"
#define ICE_CONTROLLER_CANDIDATE_TYPE_HOST_STRING "host"
#define ICE_CONTROLLER_CANDIDATE_TYPE_SRFLX_STRING "srflx"
#define ICE_CONTROLLER_CANDIDATE_TYPE_PRFLX_STRING "prflx"
#define ICE_CONTROLLER_CANDIDATE_TYPE_RELAY_STRING "relay"
#define ICE_CONTROLLER_CANDIDATE_TYPE_UNKNOWN_STRING "unknown"

#define ICE_CONTROLLER_CANDIDATE_JSON_KEY "candidate"
#define MAX_QUEUE_MSG_NUM ( 30 )
#define REQUEST_QUEUE_POLL_ID ( 0 )

static const uint32_t gCrc32Table[256] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

static void OnTimerExpire( void * pContext )
{
    IceControllerContext_t * pCtx = ( IceControllerContext_t * ) pContext;
    int32_t result = 0;

    if( pCtx->onIceEventCallbackFunc )
    {
        switch( pCtx->state )
        {
            case ICE_CONTROLLER_STATE_PROCESS_CANDIDATES_AND_PAIRS:
                result = pCtx->onIceEventCallbackFunc( pCtx->pOnIceEventCustomContext,
                                                       ICE_CONTROLLER_CB_EVENT_PROCESS_ICE_CANDIDATES_AND_PAIRS,
                                                       NULL );
                if( result != 0 )
                {
                    LogDebug( ( "Failed to process ICE candidates and pairs event, result: %ld.", result ) );
                    IceController_UpdateTimerInterval( pCtx,
                                                       ICE_CONTROLLER_CONNECTIVITY_TIMER_INTERVAL_MS );
                }
                break;
            case ICE_CONTROLLER_STATE_READY:
                result = pCtx->onIceEventCallbackFunc( pCtx->pOnIceEventCustomContext,
                                                       ICE_CONTROLLER_CB_EVENT_PERIODIC_CONNECTION_CHECK,
                                                       NULL );
                if( result != 0 )
                {
                    LogDebug( ( "Failed to process ICE periodic connection check event, result: %ld.", result ) );
                    IceController_UpdateTimerInterval( pCtx,
                                                       ICE_CONTROLLER_PERIODIC_TIMER_INTERVAL_MS );
                }
                break;
            case ICE_CONTROLLER_STATE_CLOSING:
                result = pCtx->onIceEventCallbackFunc( pCtx->pOnIceEventCustomContext,
                                                       ICE_CONTROLLER_CB_EVENT_ICE_CLOSING,
                                                       NULL );
                if( result != 0 )
                {
                    LogDebug( ( "Failed to process ICE closing event, result: %ld.", result ) );
                    IceController_UpdateTimerInterval( pCtx,
                                                       ICE_CONTROLLER_CLOSING_INTERVAL_MS );
                }
                break;
            case ICE_CONTROLLER_STATE_CLOSED:
                result = pCtx->onIceEventCallbackFunc( pCtx->pOnIceEventCustomContext,
                                                       ICE_CONTROLLER_CB_EVENT_ICE_CLOSED,
                                                       NULL );
                if( result != 0 )
                {
                    LogDebug( ( "Failed to process ICE closed event, result: %ld.", result ) );
                    IceController_UpdateTimerInterval( pCtx,
                                                       ICE_CONTROLLER_CLOSING_INTERVAL_MS );
                }
                break;
            default:
                LogError( ( "Unexpected state: %d.", pCtx->state ) );
                break;
        }
    }
}

static IceResult_t IceController_MbedtlsMd5( const uint8_t * pBuffer,
                                             size_t bufferLength,
                                             uint8_t * pOutputBuffer,
                                             uint16_t * pOutputBufferLength )
{
    IceResult_t ret = ICE_RESULT_OK;
    int retMbedtls;

    if( ( pBuffer == NULL ) || ( pOutputBuffer == NULL ) || ( pOutputBufferLength == NULL ) )
    {
        LogError( ( "Invalid inputs, pBuffer=%p, pOutputBuffer=%p, pOutputBufferLength=%p", pBuffer, pOutputBuffer, pOutputBufferLength ) );

        ret = ICE_RESULT_MD5_ERROR;
    }
    else if( *pOutputBufferLength < 16U )
    {
        LogError( ( "Invalid MD5 output buffer length, pOutputBufferLength=%u", *pOutputBufferLength ) );

        ret = ICE_RESULT_MD5_ERROR;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == ICE_RESULT_OK )
    {
        retMbedtls = mbedtls_md5_ret( pBuffer, bufferLength, pOutputBuffer );
        if( retMbedtls != 0 )
        {
            LogError( ( "mbedtls_md_hmac fails, return=%d.", retMbedtls ) );
            ret = ICE_RESULT_MD5_ERROR;
        }
    }

    if( ret == ICE_RESULT_OK )
    {
        /* MD5 result is always 16 bytes. */
        *pOutputBufferLength = 16U;
    }

    return ret;
}

static IceResult_t IceController_CalculateRandom( uint8_t * pOutputBuffer,
                                                  size_t outputBufferLength )
{
    size_t i;

    for( i = 0; i < outputBufferLength; i++ )
    {
        pOutputBuffer[i] = ( uint8_t ) ( rand() % 256 );
    }

    return ICE_RESULT_OK;
}

static IceResult_t IceController_CalculateCrc32( uint32_t initialResult,
                                                 const uint8_t * pBuffer,
                                                 size_t bufferLength,
                                                 uint32_t * pCalculatedCrc32 )
{
    uint32_t c = initialResult ^ 0xFFFFFFFF, i = 0;

    if( pBuffer == NULL )
    {
        bufferLength = 0;
    }

    for( i = 0; i < bufferLength; ++i )
    {
        c = gCrc32Table[ ( c ^ pBuffer[i] ) & 0xFF ] ^ ( c >> 8 );
    }

    *pCalculatedCrc32 = ( c ^ 0xFFFFFFFF );

    return ICE_RESULT_OK;
}

static IceResult_t IceController_MbedtlsHmac( const uint8_t * pPassword,
                                              size_t passwordLength,
                                              const uint8_t * pBuffer,
                                              size_t bufferLength,
                                              uint8_t * pOutputBuffer,
                                              uint16_t * pOutputBufferLength )
{
    IceResult_t ret = ICE_RESULT_OK;
    int retMbedtls;

    if( ( pPassword == NULL ) || ( pBuffer == NULL ) || ( pOutputBuffer == NULL ) || ( pOutputBufferLength == NULL ) )
    {
        LogError( ( "Invalid inputs, pPassword=%p, pBuffer=%p, pOutputBuffer=%p, pOutputBufferLength=%p", pPassword, pBuffer, pOutputBuffer, pOutputBufferLength ) );

        ret = ICE_RESULT_HMAC_ERROR;
    }

    if( ret == ICE_RESULT_OK )
    {
        retMbedtls = mbedtls_md_hmac( mbedtls_md_info_from_type( MBEDTLS_MD_SHA1 ),
                                      pPassword,
                                      passwordLength,
                                      pBuffer,
                                      bufferLength,
                                      pOutputBuffer );
        if( retMbedtls != 0 )
        {
            LogError( ( "mbedtls_md_hmac fails, return=%d.", retMbedtls ) );
            ret = ICE_RESULT_HMAC_ERROR;
        }
    }

    if( ret == ICE_RESULT_OK )
    {
        *pOutputBufferLength = mbedtls_md_get_size( mbedtls_md_info_from_type( MBEDTLS_MD_SHA1 ) );
    }

    return ret;
}

static IceControllerResult_t parseIceCandidate( const char * pDecodeMessage,
                                                size_t decodeMessageLength,
                                                const char ** ppCandidateString,
                                                size_t * pCandidateStringLength )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    JSONStatus_t jsonResult;
    size_t start = 0, next = 0;
    JSONPair_t pair = { 0 };
    uint8_t isCandidateFound = 0;

    jsonResult = JSON_Validate( pDecodeMessage,
                                decodeMessageLength );
    if( jsonResult != JSONSuccess )
    {
        ret = ICE_CONTROLLER_RESULT_INVALID_JSON;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Check if it's SDP offer. */
        jsonResult = JSON_Iterate( pDecodeMessage,
                                   decodeMessageLength,
                                   &start,
                                   &next,
                                   &pair );

        while( jsonResult == JSONSuccess )
        {
            if( ( pair.keyLength == strlen( ICE_CONTROLLER_CANDIDATE_JSON_KEY ) ) &&
                ( strncmp( pair.key,
                           ICE_CONTROLLER_CANDIDATE_JSON_KEY,
                           pair.keyLength ) == 0 ) )
            {
                *ppCandidateString = pair.value;
                *pCandidateStringLength = pair.valueLength;
                isCandidateFound = 1;

                break;
            }

            jsonResult = JSON_Iterate( pDecodeMessage,
                                       decodeMessageLength,
                                       &start,
                                       &next,
                                       &pair );
        }
    }

    if( ( ret == ICE_CONTROLLER_RESULT_OK ) && ( isCandidateFound == 0 ) )
    {
        LogError( ( "Fail to find candidate in JSON message(%u): %.*s",
                    decodeMessageLength,
                    ( int ) decodeMessageLength,
                    pDecodeMessage ) );
        ret = ICE_CONTROLLER_RESULT_JSON_CANDIDATE_NOT_FOUND;
    }

    return ret;
}

static IceControllerSocketContext_t * FindSocketContextByLocalCandidate( IceControllerContext_t * pCtx,
                                                                         IceCandidate_t * pLocalCandidate )
{
    IceControllerSocketContext_t * pReturnContext = NULL;
    uint32_t i;

    if( pLocalCandidate != NULL )
    {
        for( i = 0; i < pCtx->socketsContextsCount; i++ )
        {
            if( pCtx->socketsContexts[i].pLocalCandidate == pLocalCandidate )
            {
                pReturnContext = &pCtx->socketsContexts[i];
            }
        }
    }

    return pReturnContext;
}

static void ProcessLocalCandidates( IceControllerContext_t * pCtx )
{
    IceControllerResult_t result = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    uint32_t i;
    uint8_t stunBuffer[ ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE + ICE_TURN_CHANNEL_DATA_MESSAGE_HEADER_LENGTH ];
    size_t stunBufferLength = ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE;
    IceControllerSocketContext_t * pSocketContext;
    #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
    char ipFromBuffer[ INET_ADDRSTRLEN ];
    #endif /* #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE */
    uint64_t currentTimeSeconds = NetworkingUtils_GetCurrentTimeSec( NULL );

    if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
    {
        for( i = 0; i < pCtx->socketsContextsCount; i++ )
        {
            pSocketContext = &( pCtx->socketsContexts[ i ] );

            if( pSocketContext->state == ICE_CONTROLLER_SOCKET_CONTEXT_STATE_CONNECTION_IN_PROGRESS )
            {
                ( void ) IceControllerNet_ExecuteTlsHandshake( pCtx, pSocketContext, 1U );
            }
            else if( pSocketContext->pLocalCandidate != NULL )
            {

                stunBufferLength = ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE;

                iceResult = Ice_CreateNextCandidateRequest( &pCtx->iceContext,
                                                            pSocketContext->pLocalCandidate,
                                                            currentTimeSeconds,
                                                            stunBuffer,
                                                            &stunBufferLength );

                if( iceResult == ICE_RESULT_NO_NEXT_ACTION )
                {
                    /*
                     * When ICE_RESULT_NO_NEXT_ACTION is returned, this candidate pair
                     * has no pending operations and can be skipped for this iteration
                     */
                    LogVerbose( ( "No next action for local candidate ID: 0x%04x, idx: %lu", pSocketContext->pLocalCandidate->candidateId, i ) );
                    continue;
                }
                else if( iceResult != ICE_RESULT_OK )
                {
                    /* Fail to create connectivity check for this round, ignore and continue next round. */
                    LogWarn( ( "Fail to create request for local candidate ID: 0x%04x, result: %d", pSocketContext->pLocalCandidate->candidateId, iceResult ) );
                    continue;
                }
                else
                {
                    /* Do nothing, coverity happy. */
                }

                LogVerbose( ( "Sending allocation/binding/refresh request from IP/port: %s/%d",
                              IceControllerNet_LogIpAddressInfo( &pSocketContext->pLocalCandidate->endpoint,
                                                                 ipFromBuffer,
                                                                 sizeof( ipFromBuffer ) ),
                              pSocketContext->pLocalCandidate->endpoint.transportAddress.port ) );
                IceControllerNet_LogStunPacket( stunBuffer,
                                                stunBufferLength );

                result = IceControllerNet_SendPacket( pCtx,
                                                      pSocketContext,
                                                      &( pSocketContext->pIceServer->iceEndpoint ),
                                                      stunBuffer,
                                                      stunBufferLength );

                if( ( result != ICE_CONTROLLER_RESULT_OK ) &&
                    ( result != ICE_CONTROLLER_RESULT_FAIL_SOCKET_CONTEXT_ALREADY_CLOSED ) )
                {
                    LogWarn( ( "Unable to send packet to remote address, result: %d", result ) );
                    continue;
                }
            }
        }

        xSemaphoreGive( pCtx->iceMutex );
    }
    else
    {
        LogError( ( "Failed to process local candidates: mutex lock acquisition." ) );
    }
}

static IceControllerResult_t HandleCandidatePairRequest( IceControllerContext_t * pCtx,
                                                         IceControllerSocketContext_t * pTargetSocketContext,
                                                         IceCandidatePair_t * pTargetCandidatePair )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    uint8_t stunBuffer[ ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE + ICE_TURN_CHANNEL_DATA_MESSAGE_HEADER_LENGTH ];
    size_t stunBufferLength = ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE;
    IceControllerSocketContext_t * pSocketContext = pTargetSocketContext;
    #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
    char ipFromBuffer[ INET_ADDRSTRLEN ];
    char ipToBuffer[ INET_ADDRSTRLEN ];
    #endif /* #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE  */
    IceEndpoint_t * pDestEndpoint = NULL;
    uint64_t currentTimeSeconds = NetworkingUtils_GetCurrentTimeSec( NULL );

    LogVerbose( ( "Candidate Pair local/remote ID:0x%04x/0x%04x state is %d",
                  pTargetCandidatePair->pLocalCandidate->candidateId,
                  pTargetCandidatePair->pRemoteCandidate->candidateId,
                  pTargetCandidatePair->state ) );

    do
    {
        stunBufferLength = ICE_CONTROLLER_STUN_MESSAGE_BUFFER_SIZE;
        iceResult = Ice_CreateNextPairRequest( &pCtx->iceContext,
                                               pTargetCandidatePair,
                                               currentTimeSeconds,
                                               stunBuffer,
                                               &stunBufferLength );

        if( iceResult == ICE_RESULT_NO_NEXT_ACTION )
        {
            /*
             * When ICE_RESULT_NO_NEXT_ACTION is returned, this candidate pair
             * has no pending operations and can be skipped for this iteration
             */
            LogVerbose( ( "No next action for candidate pair local/remote candidate ID 0x%x / 0x%x",
                          pTargetCandidatePair->pLocalCandidate->candidateId,
                          pTargetCandidatePair->pRemoteCandidate->candidateId ) );
            break;
        }
        else if( iceResult != ICE_RESULT_OK )
        {
            LogWarn( ( "Fail to create next pair request, result: %d", iceResult ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_CREATE_NEXT_PAIR_REQUEST;
            break;
        }
        else if( pTargetCandidatePair->pRemoteCandidate == NULL )
        {
            /* No remote candidate mapped to this pair, ignore and continue next round. */
            LogWarn( ( "No remote candidate available for this pair, skip this pair" ) );
            break;
        }
        else
        {
            /* Do nothing, coverity happy. */
        }

        if( pSocketContext == NULL )
        {
            pSocketContext = FindSocketContextByLocalCandidate( pCtx,
                                                                pTargetCandidatePair->pLocalCandidate );
            if( pSocketContext == NULL )
            {
                LogWarn( ( "Not able to find socket context mapping to local candidate ID: 0x%x", pTargetCandidatePair->pLocalCandidate->candidateId ) );
                break;
            }
        }

        if( pTargetCandidatePair->pLocalCandidate->candidateType == ICE_CANDIDATE_TYPE_RELAY )
        {
            pDestEndpoint = &( pSocketContext->pIceServer->iceEndpoint );
        }
        else
        {
            pDestEndpoint = &pTargetCandidatePair->pRemoteCandidate->endpoint;
        }
        LogVerbose( ( "Sending candidate pair request from IP/port: %s/%d to %s/%d",
                      IceControllerNet_LogIpAddressInfo( &pTargetCandidatePair->pLocalCandidate->endpoint,
                                                         ipFromBuffer,
                                                         sizeof( ipFromBuffer ) ),
                      pTargetCandidatePair->pLocalCandidate->endpoint.transportAddress.port,
                      IceControllerNet_LogIpAddressInfo( pDestEndpoint,
                                                         ipToBuffer,
                                                         sizeof( ipToBuffer ) ),
                      pDestEndpoint->transportAddress.port ) );
        LogDebug( ( "Sending STUN packet to candidate pair, pair state: %d, local/remote candidate ID: 0x%04x / 0x%04x",
                    pTargetCandidatePair->state,
                    pTargetCandidatePair->pLocalCandidate->candidateId,
                    pTargetCandidatePair->pRemoteCandidate->candidateId ) );

        IceControllerNet_LogStunPacket( stunBuffer,
                                        stunBufferLength );

        ret = IceControllerNet_SendPacket( pCtx,
                                           pSocketContext,
                                           pDestEndpoint,
                                           stunBuffer,
                                           stunBufferLength );

        if( ( ret != ICE_CONTROLLER_RESULT_OK ) && ( ret != ICE_CONTROLLER_RESULT_FAIL_SOCKET_CONTEXT_ALREADY_CLOSED ) )
        {
            LogWarn( ( "Unable to send packet to remote address, result: %d", ret ) );
        }
    } while( 0 );

    return ret;
}

static void ProcessCandidatePairs( IceControllerContext_t * pCtx )
{
    IceControllerResult_t result = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    uint32_t i;
    size_t count;
    IceControllerSocketContext_t * pSocketContext = NULL;
    uint8_t isLocked = 0U;

    /* Take ice lock. */
    if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
    {
        isLocked = 1U;
    }
    else
    {
        LogError( ( "Failed to process candidate pairs: mutex lock acquisition." ) );
        result = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
    }

    if( result == ICE_CONTROLLER_RESULT_OK )
    {
        /* Set the metric for first connectivity check request. */
        if( pCtx->metrics.isFirstConnectivityRequest == 1 )
        {
            pCtx->metrics.isFirstConnectivityRequest = 0;
            #if METRIC_PRINT_ENABLED
            Metric_StartEvent( METRIC_EVENT_ICE_FIND_P2P_CONNECTION );
            #endif
        }

        iceResult = Ice_GetCandidatePairCount( &pCtx->iceContext,
                                               &count );
        if( iceResult != ICE_RESULT_OK )
        {
            LogError( ( "Fail to query valid candidate pair count, result: %d", iceResult ) );
            result = ICE_CONTROLLER_RESULT_FAIL_QUERY_CANDIDATE_PAIR_COUNT;
        }
    }

    if( result == ICE_CONTROLLER_RESULT_OK )
    {
        for( i = 0; i < count; i++ )
        {
            pSocketContext = FindSocketContextByLocalCandidate( pCtx,
                                                                pCtx->iceContext.pCandidatePairs[i].pLocalCandidate );
            if( pSocketContext == NULL )
            {
                LogWarn( ( "Not able to find socket context mapping to local candidate ID: 0x%x", pCtx->iceContext.pCandidatePairs[i].pLocalCandidate->candidateId ) );
                continue;
            }

            result = HandleCandidatePairRequest( pCtx,
                                                 pSocketContext,
                                                 &pCtx->iceContext.pCandidatePairs[i] );
        }
    }

    if( isLocked != 0U )
    {
        xSemaphoreGive( pCtx->iceMutex );
    }
}

static void PrintCandidatesStatus( IceControllerContext_t * pCtx )
{
    IceControllerResult_t result = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    uint32_t i;
    size_t candidatesCount;
    uint8_t isLocked = 0U;

    if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
    {
        isLocked = 1U;
    }
    else
    {
        LogError( ( "Failed to print candidates' status: mutex lock acquisition." ) );
        result = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
    }

    if( result == ICE_CONTROLLER_RESULT_OK )
    {
        iceResult = Ice_GetLocalCandidateCount( &pCtx->iceContext,
                                                &candidatesCount );
        if( iceResult != ICE_RESULT_OK )
        {
            LogError( ( "Fail to query valid candidate count, result: %d", iceResult ) );
            result = ICE_CONTROLLER_RESULT_FAIL_QUERY_LOCAL_CANDIDATE_COUNT;
        }
    }

    if( result == ICE_CONTROLLER_RESULT_OK )
    {
        for( i = 0; i < candidatesCount; i++ )
        {
            LogInfo( ( "Local candidate ID: 0x%04x, state is %d",
                       pCtx->iceContext.pLocalCandidates[ i ].candidateId,
                       pCtx->iceContext.pLocalCandidates[ i ].state ) );
        }
    }

    if( isLocked != 0U )
    {
        xSemaphoreGive( pCtx->iceMutex );
    }
}

static void PrintCandidatePairsStatus( IceControllerContext_t * pCtx )
{
    IceControllerResult_t result = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    uint32_t i;
    size_t candidatePairsCount;
    uint8_t isLocked = 0U;

    if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
    {
        isLocked = 1U;
    }
    else
    {
        LogError( ( "Failed to print candidate pairs' status: mutex lock acquisition." ) );
        result = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
    }

    if( result == ICE_CONTROLLER_RESULT_OK )
    {
        iceResult = Ice_GetCandidatePairCount( &pCtx->iceContext,
                                               &candidatePairsCount );
        if( iceResult != ICE_RESULT_OK )
        {
            LogError( ( "Fail to query valid candidate count, result: %d", iceResult ) );
            result = ICE_CONTROLLER_RESULT_FAIL_QUERY_LOCAL_CANDIDATE_COUNT;
        }
    }

    if( result == ICE_CONTROLLER_RESULT_OK )
    {
        for( i = 0; i < candidatePairsCount; i++ )
        {
            LogInfo( ( "Local/Remote candidate ID: 0x%04x / 0x%04x, state is %d",
                       pCtx->iceContext.pCandidatePairs[ i ].pLocalCandidate->candidateId,
                       pCtx->iceContext.pCandidatePairs[ i ].pRemoteCandidate->candidateId,
                       pCtx->iceContext.pCandidatePairs[ i ].state ) );
        }
    }

    if( isLocked != 0U )
    {
        xSemaphoreGive( pCtx->iceMutex );
    }
}

static void ReleaseOtherSockets( IceControllerContext_t * pCtx,
                                 IceControllerSocketContext_t * pChosenSocketContext )
{
    uint8_t skipProcess = 0;
    int i;

    if( ( pCtx == NULL ) || ( pChosenSocketContext == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pChosenSocketContext: %p", pCtx, pChosenSocketContext ) );
        skipProcess = 1;
    }

    if( skipProcess == 0 )
    {
        LogDebug( ( "Closing sockets other than local candidate ID: 0x%04x", pChosenSocketContext->pLocalCandidate->candidateId ) );
        for( i = 0; i < pCtx->socketsContextsCount; i++ )
        {
            if( pCtx->socketsContexts[i].socketFd != pChosenSocketContext->socketFd )
            {
                if( ( pCtx->socketsContexts[i].pLocalCandidate != NULL ) && ( pCtx->socketsContexts[i].pLocalCandidate->candidateType == ICE_CANDIDATE_TYPE_RELAY ) )
                {
                    if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
                    {
                        /* If the local candidate is a relay candidate, we have to send refresh request with lifetime 0 to end the session.
                         * Thus keep the socket alive until it's terminated. */
                        Ice_CloseCandidate( &pCtx->iceContext,
                                            pCtx->socketsContexts[i].pLocalCandidate );
                        xSemaphoreGive( pCtx->iceMutex );
                        LogDebug( ( "Keep socket of local relay candidate ID: 0x%04x for terminating TURN resource", pCtx->socketsContexts[i].pLocalCandidate->candidateId ) );
                    }
                    else
                    {
                        LogError( ( "Failed to close ICE candidate: mutex lock acquisition." ) );
                    }
                }
                else
                {
                    /* Release all unused socket contexts. */
                    LogDebug( ( "Closing socket fd %d", pCtx->socketsContexts[i].socketFd ) );
                    IceControllerNet_FreeSocketContext( pCtx, &pCtx->socketsContexts[i] );
                }
            }
        }
    }

    if( skipProcess == 0 )
    {
        IceController_CloseOtherCandidatePairs( pCtx, pChosenSocketContext->pCandidatePair );
    }
}

void IceController_HandleEvent( IceControllerContext_t * pCtx,
                                IceControllerEvent_t event )
{
    if( pCtx == NULL )
    {
        LogError( ( "Invalid input, pCtx: %p", pCtx ) );
    }
    else
    {
        switch( event )
        {
            case ICE_CONTROLLER_EVENT_DTLS_HANDSHAKE_DONE:
            {
                ReleaseOtherSockets( pCtx, pCtx->pNominatedSocketContext );
                LogDebug( ( "Released all other socket contexts" ) );
                break;
            }
            default:
            {
                LogError( ( "Unknown ICE event: %d", event ) );
                break;
            }
        }
    }
}

IceControllerResult_t IceController_AddRemoteCandidate( IceControllerContext_t * pCtx,
                                                        IceRemoteCandidateInfo_t * pRemoteCandidate )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    uint8_t acceptCandidate = 0U;
    #if LIBRARY_LOG_LEVEL >= LOG_INFO
    char ipBuffer[ INET6_ADDRSTRLEN ];
    #endif /* #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE  */

    if( ( pCtx == NULL ) || ( pRemoteCandidate == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pRemoteCandidate: %p", pCtx, pRemoteCandidate ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* TODO: Skip IPv6 remote candidiate for now. */
        if( pRemoteCandidate->pEndpoint->transportAddress.family != STUN_ADDRESS_IPv4 )
        {
            LogInfo( ( "Dropping IPv6 remote candidate: %s/%u",
                       IceControllerNet_LogIpAddressInfo( pRemoteCandidate->pEndpoint,
                                                          ipBuffer,
                                                          sizeof( ipBuffer ) ),
                       pRemoteCandidate->pEndpoint->transportAddress.port ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_ADD_IPv6_REMOTE_CANDIDATE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* TODO: Skip TCP remote candidiate for now. */
        if( pRemoteCandidate->remoteProtocol != ICE_SOCKET_PROTOCOL_UDP )
        {
            LogInfo( ( "Dropping non UDP remote candidate: %s/%u, protocol: %d",
                       IceControllerNet_LogIpAddressInfo( pRemoteCandidate->pEndpoint,
                                                          ipBuffer,
                                                          sizeof( ipBuffer ) ),
                       pRemoteCandidate->pEndpoint->transportAddress.port,
                       pRemoteCandidate->remoteProtocol ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_ADD_NON_UDP_REMOTE_CANDIDATE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        switch( pRemoteCandidate->candidateType )
        {
            case ICE_CANDIDATE_TYPE_HOST:
            {
                if( ICE_CONTROLLER_IS_NAT_CONFIG_SET( pCtx, ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_ACCEPT_HOST ) )
                {
                    acceptCandidate = 1U;
                }
                break;
            }
            case ICE_CANDIDATE_TYPE_PEER_REFLEXIVE:
            {
                acceptCandidate = 1U;
                break;
            }
            case ICE_CANDIDATE_TYPE_SERVER_REFLEXIVE:
            {
                if( ICE_CONTROLLER_IS_NAT_CONFIG_SET( pCtx, ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_ACCEPT_SRFLX ) )
                {
                    acceptCandidate = 1U;
                }
                break;
            }
            case ICE_CANDIDATE_TYPE_RELAY:
            {
                if( ICE_CONTROLLER_IS_NAT_CONFIG_SET( pCtx, ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_ACCEPT_RELAY ) )
                {
                    acceptCandidate = 1U;
                }
                break;
            }
            default:
                LogWarn( ( "Unknown candidate type: %d", pRemoteCandidate->candidateType ) );
                break;
        }

        if( acceptCandidate == 0U )
        {
            LogInfo( ( "Dropping remote candidate with type: %d, NAT traversal config bitmap: 0x%x", pRemoteCandidate->candidateType, pCtx->natTraversalConfigBitmap ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_ADD_CANDIDATE_TYPE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
        {
            iceResult = Ice_AddRemoteCandidate( &pCtx->iceContext,
                                                pRemoteCandidate );

            xSemaphoreGive( pCtx->iceMutex );

            if( iceResult != ICE_RESULT_OK )
            {
                LogError( ( "Fail to add remote candidate, result: %d", iceResult ) );
                ret = ICE_CONTROLLER_RESULT_FAIL_ADD_REMOTE_CANDIDATE;
            }
            else
            {
                LogVerbose( ( "Received remote candidate with IP/port: %s/%d",
                              IceControllerNet_LogIpAddressInfo( pRemoteCandidate->pEndpoint,
                                                                 ipBuffer,
                                                                 sizeof( ipBuffer ) ),
                              pRemoteCandidate->pEndpoint->transportAddress.port ) );

                LogDebug( ( "Added new remote candidate with ID: 0x%04x", pCtx->iceContext.pRemoteCandidates[ pCtx->iceContext.numRemoteCandidates - 1 ].candidateId ) );
            }
        }
        else
        {
            LogError( ( "Failed to add remote candidate: unable to acquire lock." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
        }
    }

    return ret;
}

IceControllerResult_t IceController_ProcessIceCandidatesAndPairs( IceControllerContext_t * pCtx )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    uint64_t currentTimeMs = NetworkingUtils_GetCurrentTimeUs( NULL ) / 1000;

    if( pCtx == NULL )
    {
        LogError( ( "Invalid input, pCtx: %p", pCtx ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( pCtx->addLocalCandidates != 0U )
        {
            pCtx->addLocalCandidates = 0U;
            IceControllerNet_AddLocalCandidates( pCtx );
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Send next candidate pair request for each candidate pair. */
        ProcessCandidatePairs( pCtx );

        /* Send request for local candidates. */
        ProcessLocalCandidates( pCtx );
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Check timeout. */
        if( currentTimeMs > pCtx->connectivityCheckTimeoutMs )
        {
            LogWarn( ( "Unable to find valid connection before timeout for ICE combined name: %.*s, closing peer connection session.",
                       ( int ) pCtx->iceContext.creds.combinedUsernameLength,
                       pCtx->iceContext.creds.pCombinedUsername ) );

            /* Notify peer connection for closing the connection. */
            if( pCtx->onIceEventCallbackFunc )
            {
                pCtx->onIceEventCallbackFunc( pCtx->pOnIceEventCustomContext,
                                              ICE_CONTROLLER_CB_EVENT_ICE_CLOSE_NOTIFY,
                                              NULL );
                /* Re-set the timer. */
                IceController_UpdateTimerInterval( pCtx,
                                                   ICE_CONTROLLER_CLOSING_INTERVAL_MS );
            }
            else
            {
                LogError( ( "There is no ICE event callback function set." ) );
            }

            ret = ICE_CONTROLLER_RESULT_CONNECTIVITY_CHECK_TIMEOUT;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Re-set the timer. */
        IceController_UpdateTimerInterval( pCtx,
                                           ICE_CONTROLLER_CONNECTIVITY_TIMER_INTERVAL_MS );
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( currentTimeMs > pCtx->metrics.printCandidatePairsStatusMs )
        {
            LogInfo( ( "========== Print Candidates / Pairs States ==========" ) );
            PrintCandidatesStatus( pCtx );
            PrintCandidatePairsStatus( pCtx );
            LogInfo( ( "========== Print Candidates / Pairs States ==========" ) );

            pCtx->metrics.printCandidatePairsStatusMs = currentTimeMs + ICE_CONTROLLER_PRINT_CONNECTIVITY_CHECK_PERIOD_MS;
        }
    }

    return ret;
}

IceControllerResult_t IceController_PeriodConnectionCheck( IceControllerContext_t * pCtx )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;

    if( pCtx == NULL )
    {
        LogError( ( "Invalid input, pCtx: %p", pCtx ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( ( pCtx->pNominatedSocketContext != NULL ) &&
            ( pCtx->pNominatedSocketContext->pLocalCandidate != NULL ) )
        {
            /* Check nominated candidated pair lifetime by calling Ice_CreateNextPairRequest. */
            if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
            {
                ( void ) HandleCandidatePairRequest( pCtx, pCtx->pNominatedSocketContext, pCtx->pNominatedSocketContext->pCandidatePair );
                xSemaphoreGive( pCtx->iceMutex );
            }
            else
            {
                LogError( ( "Failed to check nominated pair request: mutex lock acquisition." ) );
            }
        }
        else
        {
            LogError( ( "Unexpected behavior, nominated pair must be set before entering ready state. pNominatedSocketContext: %p", pCtx->pNominatedSocketContext ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_FIND_NOMINATED_CONTEXT;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Check local candidates to make sure all unused TURN session are released correctly. */
        ProcessLocalCandidates( pCtx );

        /* Reset the timer. */
        IceController_UpdateTimerInterval( pCtx, ICE_CONTROLLER_PERIODIC_TIMER_INTERVAL_MS );
    }

    return ret;
}

IceControllerResult_t IceController_AddressClosing( IceControllerContext_t * pCtx )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    uint32_t i;
    uint8_t isAnySocketAlive = 0U;

    if( pCtx == NULL )
    {
        LogError( ( "Invalid input, pCtx: %p", pCtx ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }


    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        for( i = 0; i < pCtx->socketsContextsCount; i++ )
        {
            if( pCtx->socketsContexts[i].state != ICE_CONTROLLER_SOCKET_CONTEXT_STATE_NONE )
            {
                isAnySocketAlive = 1U;
                break;
            }
        }

        /* Send request for local candidates. */
        if( isAnySocketAlive != 0U )
        {
            ProcessLocalCandidates( pCtx );

            IceController_UpdateTimerInterval( pCtx, ICE_CONTROLLER_CLOSING_INTERVAL_MS );
        }
        else
        {
            LogInfo( ( "Stopping polling for Ice controller." ) );
            ( void ) IceControllerSocketListener_StopPolling( pCtx );

            /* All sockets have been closed, notify peer connection. */
            if( pCtx->onIceEventCallbackFunc )
            {
                pCtx->onIceEventCallbackFunc( pCtx->pOnIceEventCustomContext,
                                              ICE_CONTROLLER_CB_EVENT_ICE_CLOSED,
                                              NULL );
            }
            else
            {
                LogError( ( "There is no ICE event callback function set." ) );
            }

            IceController_UpdateState( pCtx, ICE_CONTROLLER_STATE_CLOSED );
        }
    }

    return ret;
}

IceControllerResult_t IceController_Destroy( IceControllerContext_t * pCtx )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    int i;
    uint8_t needReleaseTurnResource = 0U;

    if( pCtx == NULL )
    {
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        switch( pCtx->state )
        {
            case ICE_CONTROLLER_STATE_NEW:
                IceController_UpdateState( pCtx, ICE_CONTROLLER_STATE_NONE );
                break;
            case ICE_CONTROLLER_STATE_READY:
            case ICE_CONTROLLER_STATE_PROCESS_CANDIDATES_AND_PAIRS:
            case ICE_CONTROLLER_STATE_CLOSING:
                IceController_UpdateState( pCtx, ICE_CONTROLLER_STATE_CLOSING );
                break;
            case ICE_CONTROLLER_STATE_NONE:
            case ICE_CONTROLLER_STATE_CLOSED:
            default:
                ret = ICE_CONTROLLER_RESULT_CONTEXT_ALREADY_CLOSED;
                break;
        }
    }

    /* Reset socket contexts. */
    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        for( i = 0; i < pCtx->socketsContextsCount; i++ )
        {
            if( ( pCtx->socketsContexts[i].state != ICE_CONTROLLER_SOCKET_CONTEXT_STATE_NONE ) &&
                ( pCtx->socketsContexts[i].pLocalCandidate != NULL ) &&
                ( pCtx->socketsContexts[i].pLocalCandidate->candidateType == ICE_CANDIDATE_TYPE_RELAY ) )
            {
                /* Check nominated candidated pair lifetime by calling Ice_CreateNextPairRequest. */
                if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
                {
                    /* If the local candidate is a relay candidate, we have to send refresh request with lifetime 0 to end the session.
                     * Thus keep the socket alive until it's terminated. */
                    Ice_CloseCandidate( &pCtx->iceContext,
                                        pCtx->socketsContexts[i].pLocalCandidate );
                    xSemaphoreGive( pCtx->iceMutex );

                    needReleaseTurnResource = 1U;
                }
                else
                {
                    LogError( ( "Failed to close ICE candidate: mutex lock acquisition." ) );
                }
            }
            else if( pCtx->socketsContexts[i].state != ICE_CONTROLLER_SOCKET_CONTEXT_STATE_NONE )
            {
                IceControllerNet_FreeSocketContext( pCtx,
                                                    &pCtx->socketsContexts[i] );
            }
            else
            {
                /* Empty else marker. */
            }
        }
    }

    /* Stop polling */
    if( ( ret == ICE_CONTROLLER_RESULT_OK ) && ( needReleaseTurnResource == 0U ) )
    {
        LogInfo( ( "Stopping polling for Ice controller." ) );
        ( void ) IceControllerSocketListener_StopPolling( pCtx );

        /* All sockets have been closed, notify peer connection. */
        if( pCtx->onIceEventCallbackFunc )
        {
            pCtx->onIceEventCallbackFunc( pCtx->pOnIceEventCustomContext,
                                          ICE_CONTROLLER_CB_EVENT_ICE_CLOSED,
                                          NULL );
        }
        else
        {
            LogError( ( "There is no ICE event callback function set." ) );
        }

        IceController_UpdateState( pCtx, ICE_CONTROLLER_STATE_CLOSED );
    }
    else if( ( ret == ICE_CONTROLLER_RESULT_OK ) && ( needReleaseTurnResource != 0U ) )
    {
        LogInfo( ( "Waiting for TURN session to be released." ) );
        IceController_UpdateTimerInterval( pCtx, ICE_CONTROLLER_CLOSING_INTERVAL_MS );
    }
    else
    {
        /* Empty else marker. */
    }

    return ret;
}

IceControllerResult_t IceController_Init( IceControllerContext_t * pCtx,
                                          IceControllerInitConfig_t * pInitConfig )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    TimerControllerResult_t retTimer;
    int i;

    if( ( pCtx == NULL ) || ( pInitConfig == NULL ) )
    {
        LogError( ( "Invalid parameters, pCtx: %p, pInitConfig: %p", pCtx, pInitConfig ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        memset( pCtx,
                0,
                sizeof( IceControllerContext_t ) );

        IceController_UpdateState( pCtx, ICE_CONTROLLER_STATE_NEW );

        pCtx->onIceEventCallbackFunc = pInitConfig->onIceEventCallbackFunc;
        pCtx->pOnIceEventCustomContext = pInitConfig->pOnIceEventCallbackContext;

        /* Initialize metrics. */
        pCtx->metrics.isFirstConnectivityRequest = 1;

        /* Store NAT traversal config. */
        pCtx->natTraversalConfigBitmap = pInitConfig->natTraversalConfigBitmap;
    }

    /* Initialize timer for connectivity check. */
    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        retTimer = TimerController_Create( &pCtx->timerHandler,
                                           ICE_CONTROLLER_TIMER_NAME,
                                           ICE_CONTROLLER_CONNECTIVITY_TIMER_INTERVAL_MS,
                                           0U, /* No repeat. */
                                           OnTimerExpire,
                                           pCtx );
        if( retTimer != TIMER_CONTROLLER_RESULT_OK )
        {
            LogError( ( "TimerController_Create return fail, result: %d", retTimer ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_TIMER_INIT;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        for( i = 0; i < ICE_CONTROLLER_MAX_LOCAL_CANDIDATE_COUNT; i++ )
        {
            pCtx->socketsContexts[i].socketFd = -1;
            pCtx->socketsContexts[i].state = ICE_CONTROLLER_SOCKET_CONTEXT_STATE_NONE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Mutex can only be created in executing scheduler. */
        pCtx->socketMutex = xSemaphoreCreateMutex();
        if( pCtx->socketMutex == NULL )
        {
            LogError( ( "Fail to create mutex for Ice controller." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_CREATE;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Mutex can only be created in executing scheduler. */
        pCtx->iceMutex = xSemaphoreCreateMutex();
        if( pCtx->iceMutex == NULL )
        {
            LogError( ( "Fail to create ICE mutex for Ice controller." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_CREATE;
        }
    }

    /* Initialize socket listener task. */
    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        ret = IceControllerSocketListener_Init( pCtx,
                                                pInitConfig->onRecvNonStunPacketFunc,
                                                pInitConfig->pOnRecvNonStunPacketCallbackContext );
    }

    return ret;
}

IceControllerResult_t IceController_DeserializeIceCandidate( const char * pDecodeMessage,
                                                             size_t decodeMessageLength,
                                                             IceControllerCandidate_t * pCandidate )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    StringUtilsResult_t stringResult;
    const char * pCandidateString;
    size_t candidateStringLength = 0;
    const char * pCurr, * pTail, * pNext;
    size_t tokenLength;
    IceControllerCandidateDeserializerState_t deserializerState = ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_FOUNDATION;
    uint8_t isAllElementsParsed = 0;
    uint32_t port;

    if( ( pDecodeMessage == NULL ) || ( pCandidate == NULL ) )
    {
        LogError( ( "Invalid input, pDecodeMessage: %p, pCandidate: %p", pDecodeMessage, pCandidate ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* parse json message and get the candidate string. Note that it's possible the remote candidate is from media description in SDP offer/answer.
         * In this case, it's not in JSON format. */
        ret = parseIceCandidate( pDecodeMessage,
                                 decodeMessageLength,
                                 &pCandidateString,
                                 &candidateStringLength );
        if( ret == ICE_CONTROLLER_RESULT_INVALID_JSON )
        {
            pCurr = pDecodeMessage;
            pTail = pDecodeMessage + decodeMessageLength;

            /* Reset it to OK to continue parsing. */
            ret = ICE_CONTROLLER_RESULT_OK;
        }
        else
        {
            pCurr = pCandidateString;
            pTail = pCandidateString + candidateStringLength;
        }
    }

    /* deserialize candidate string into structure. */
    while( ret == ICE_CONTROLLER_RESULT_OK &&
           pCurr < pTail &&
           deserializerState <= ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_MAX )
    {
        pNext = memchr( pCurr,
                        ' ',
                        pTail - pCurr );

        if( pNext == NULL )
        {
            // If no space is found, set pNext to the end of the string
            pNext = pTail;
        }

        tokenLength = pNext - pCurr;

        switch( deserializerState )
        {
            case ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_FOUNDATION:
            case ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_COMPONENT:
                break;
            case ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_PROTOCOL:
                if( ( strncmp( pCurr,
                               "tcp",
                               tokenLength ) == 0 ) ||
                    ( strncmp( pCurr,
                               "TCP",
                               tokenLength ) == 0 ) )
                {
                    pCandidate->protocol = ICE_SOCKET_PROTOCOL_TCP;
                }
                else if( ( strncmp( pCurr,
                                    "udp",
                                    tokenLength ) == 0 ) ||
                         ( strncmp( pCurr,
                                    "UDP",
                                    tokenLength ) == 0 ) )
                {
                    pCandidate->protocol = ICE_SOCKET_PROTOCOL_UDP;
                }
                else
                {
                    LogWarn( ( "unknown protocol %.*s",
                               ( int ) tokenLength, pCurr ) );
                    ret = ICE_CONTROLLER_RESULT_JSON_CANDIDATE_INVALID_PROTOCOL;
                }
                break;
            case ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_PRIORITY:
                stringResult = StringUtils_ConvertStringToUl( pCurr,
                                                              tokenLength,
                                                              &pCandidate->priority );
                if( stringResult != STRING_UTILS_RESULT_OK )
                {
                    LogWarn( ( "Invalid priority %.*s",
                               ( int ) tokenLength, pCurr ) );
                    ret = ICE_CONTROLLER_RESULT_JSON_CANDIDATE_INVALID_PRIORITY;
                }
                break;
            case ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_IP:
                ret = IceControllerNet_ConvertIpString( pCurr,
                                                        tokenLength,
                                                        &pCandidate->iceEndpoint );
                break;
            case ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_PORT:
                stringResult = StringUtils_ConvertStringToUl( pCurr,
                                                              tokenLength,
                                                              &port );

                if( stringResult != STRING_UTILS_RESULT_OK )
                {
                    LogWarn( ( "Invalid port %.*s",
                               ( int ) tokenLength, pCurr ) );
                    ret = ICE_CONTROLLER_RESULT_JSON_CANDIDATE_INVALID_PORT;
                }
                else
                {
                    pCandidate->iceEndpoint.transportAddress.port = ( uint16_t ) port;
                }
                break;
            case ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_TYPE_ID:
                if( ( tokenLength != strlen( "typ" ) ) || ( strncmp( pCurr,
                                                                     "typ",
                                                                     tokenLength ) != 0 ) )
                {
                    ret = ICE_CONTROLLER_RESULT_JSON_CANDIDATE_INVALID_TYPE_ID;
                }
                break;
            case ICE_CONTROLLER_CANDIDATE_DESERIALIZER_STATE_TYPE_VAL:
                isAllElementsParsed = 1;

                if( strncmp( pCurr,
                             ICE_CONTROLLER_CANDIDATE_TYPE_HOST_STRING,
                             tokenLength ) == 0 )
                {
                    pCandidate->candidateType = ICE_CANDIDATE_TYPE_HOST;
                }
                else if( strncmp( pCurr,
                                  ICE_CONTROLLER_CANDIDATE_TYPE_SRFLX_STRING,
                                  tokenLength ) == 0 )
                {
                    pCandidate->candidateType = ICE_CANDIDATE_TYPE_SERVER_REFLEXIVE;
                }
                else if( strncmp( pCurr,
                                  ICE_CONTROLLER_CANDIDATE_TYPE_PRFLX_STRING,
                                  tokenLength ) == 0 )
                {
                    pCandidate->candidateType = ICE_CANDIDATE_TYPE_PEER_REFLEXIVE;
                }
                else if( strncmp( pCurr,
                                  ICE_CONTROLLER_CANDIDATE_TYPE_RELAY_STRING,
                                  tokenLength ) == 0 )
                {
                    pCandidate->candidateType = ICE_CANDIDATE_TYPE_RELAY;
                }
                else
                {
                    LogWarn( ( "unknown candidate type %.*s",
                               ( int ) tokenLength, pCurr ) );
                    ret = ICE_CONTROLLER_RESULT_JSON_CANDIDATE_INVALID_TYPE;
                }
                break;
            default:
                break;
        }

        pCurr = pNext + 1;
        deserializerState++;
    }

    if( isAllElementsParsed != 1 )
    {
        ret = ICE_CONTROLLER_RESULT_JSON_CANDIDATE_LACK_OF_ELEMENT;
    }

    return ret;
}

IceControllerResult_t IceController_Start( IceControllerContext_t * pCtx,
                                           IceControllerStartConfig_t * pStartConfig )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    IceInitInfo_t iceInitInfo;
    uint8_t i;
    uint64_t currentTimeMs = NetworkingUtils_GetCurrentTimeUs( NULL ) / 1000;

    if( ( pCtx == NULL ) ||
        ( pStartConfig == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pStartConfig: %p",
                    pCtx, pStartConfig ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }
    else if( ( pStartConfig->pLocalUserName == NULL ) || ( pStartConfig->pLocalPassword == NULL ) ||
             ( pStartConfig->pRemoteUserName == NULL ) || ( pStartConfig->pRemotePassword == NULL ) ||
             ( pStartConfig->pCombinedName == NULL ) )
    {
        LogError( ( "Invalid input, pLocalUserName: %p, pLocalPassword: %p, pRemoteUserName: %p, pRemotePassword: %p, pCombinedName: %p",
                    pStartConfig->pLocalUserName, pStartConfig->pLocalPassword, pStartConfig->pRemoteUserName, pStartConfig->pRemotePassword, pStartConfig->pCombinedName ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    /* Initialize ICE component. */
    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        TransactionIdStore_Init( &pCtx->transactionIdStore,
                                 pCtx->transactionIdsBuffer,
                                 ICE_CONTROLLER_MAX_CANDIDATE_PAIR_COUNT );

        /* Creating the Ice Initialization Info. */
        memset( &iceInitInfo,
                0,
                sizeof( IceInitInfo_t ) );
        iceInitInfo.creds.pLocalUsername = ( const uint8_t * ) pStartConfig->pLocalUserName;
        iceInitInfo.creds.localUsernameLength = pStartConfig->localUserNameLength;
        iceInitInfo.creds.pLocalPassword = ( const uint8_t * ) pStartConfig->pLocalPassword;
        iceInitInfo.creds.localPasswordLength = pStartConfig->localPasswordLength;
        iceInitInfo.creds.pRemoteUsername = ( const uint8_t * ) pStartConfig->pRemoteUserName;
        iceInitInfo.creds.remoteUsernameLength = pStartConfig->remoteUserNameLength;
        iceInitInfo.creds.pRemotePassword = ( const uint8_t * ) pStartConfig->pRemotePassword;
        iceInitInfo.creds.remotePasswordLength = pStartConfig->remotePasswordLength;
        iceInitInfo.creds.pCombinedUsername = ( const uint8_t * ) pStartConfig->pCombinedName;
        iceInitInfo.creds.combinedUsernameLength = pStartConfig->combinedNameLength;
        iceInitInfo.pLocalCandidatesArray = pCtx->localCandidatesBuffer;
        iceInitInfo.localCandidatesArrayLength = ICE_CONTROLLER_MAX_LOCAL_CANDIDATE_COUNT;
        iceInitInfo.pRemoteCandidatesArray = pCtx->remoteCandidatesBuffer;
        iceInitInfo.remoteCandidatesArrayLength = ICE_CONTROLLER_MAX_REMOTE_CANDIDATE_COUNT;
        iceInitInfo.pCandidatePairsArray = pCtx->candidatePairsBuffer;
        iceInitInfo.candidatePairsArrayLength = ICE_CONTROLLER_MAX_CANDIDATE_PAIR_COUNT;
        iceInitInfo.pTurnServerArray = pCtx->turnServersBuffer;
        iceInitInfo.turnServerArrayLength = ICE_CONTROLLER_MAX_ICE_SERVER_COUNT;
        iceInitInfo.cryptoFunctions.randomFxn = IceController_CalculateRandom;
        iceInitInfo.cryptoFunctions.crc32Fxn = IceController_CalculateCrc32;
        iceInitInfo.cryptoFunctions.hmacFxn = IceController_MbedtlsHmac;
        iceInitInfo.cryptoFunctions.md5Fxn = IceController_MbedtlsMd5;
        iceInitInfo.isControlling = pStartConfig->isControlling;
        iceInitInfo.pStunBindingRequestTransactionIdStore = &pCtx->transactionIdStore;

        if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
        {
            iceResult = Ice_Init( &pCtx->iceContext,
                                  &iceInitInfo );
            xSemaphoreGive( pCtx->iceMutex );

            if( iceResult != ICE_RESULT_OK )
            {
                LogError( ( "Fail to create ICE agent, result: %d", iceResult ) );
                ret = ICE_CONTROLLER_RESULT_FAIL_CREATE_ICE_AGENT;
            }
        }
        else
        {
            LogError( ( "Failed to initialize ICE context: mutex lock acquisition." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
        }
    }

    /* Initialize socket contexts. */
    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        for( i = 0; i < ICE_CONTROLLER_MAX_LOCAL_CANDIDATE_COUNT; i++ )
        {
            if( pCtx->socketsContexts[i].socketFd >= 0 )
            {
                /* Force close socket before next round. */
                IceControllerNet_FreeSocketContext( pCtx,
                                                    &pCtx->socketsContexts[i] );
            }
        }
        pCtx->socketsContextsCount = 0;
        pCtx->pNominatedSocketContext = NULL;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        IceController_UpdateState( pCtx, ICE_CONTROLLER_STATE_PROCESS_CANDIDATES_AND_PAIRS );
        pCtx->metrics.printCandidatePairsStatusMs = currentTimeMs + ICE_CONTROLLER_PRINT_CONNECTIVITY_CHECK_PERIOD_MS;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        pCtx->addLocalCandidates = 1U;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        /* Update the connectivity timeout before starting connectivity check. */
        pCtx->connectivityCheckTimeoutMs = currentTimeMs + ICE_CONTROLLER_CONNECTIVITY_CHECK_TIMEOUT_MS;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        ret = IceControllerSocketListener_StartPolling( pCtx );
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        OnTimerExpire( pCtx );
    }

    return ret;
}

IceControllerResult_t IceController_SendToRemotePeer( IceControllerContext_t * pCtx,
                                                      const uint8_t * pBuffer,
                                                      size_t bufferLength )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    IceResult_t iceResult;
    const uint8_t * pSendingBuffer = pBuffer;
    size_t sendingBufferLength = bufferLength;
    size_t turnBufferLength;
    IceEndpoint_t * pDestEndpoint = NULL;
    uint8_t turnSendBuffer[ ICE_CONTROLLER_MAX_MTU ];

    if( ( pCtx == NULL ) ||
        ( pBuffer == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pBuffer: %p", pCtx, pBuffer ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( ( pCtx->pNominatedSocketContext == NULL ) ||
            ( pCtx->pNominatedSocketContext->state < ICE_CONTROLLER_SOCKET_CONTEXT_STATE_SELECTED ) )
        {
            LogWarn( ( "The connection of this session is not ready." ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_CONNECTION_NOT_READY;
        }
        else if( pCtx->pNominatedSocketContext->pLocalCandidate == NULL )
        {
            LogWarn( ( "The connection of this session is not ready, local candidate pointer is NULL" ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_CONNECTION_NOT_READY;
        }
        else if( pCtx->pNominatedSocketContext->pRemoteCandidate == NULL )
        {
            LogWarn( ( "The connection of this session is not ready, remote candidate pointer is NULL" ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_CONNECTION_NOT_READY;
        }
        else if( pCtx->pNominatedSocketContext->pCandidatePair == NULL )
        {
            LogWarn( ( "The connection of this session is not ready, candidate pair pointer is NULL" ) );
            ret = ICE_CONTROLLER_RESULT_FAIL_CONNECTION_NOT_READY;
        }
        else
        {
            pDestEndpoint = &pCtx->pNominatedSocketContext->pRemoteCandidate->endpoint;
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( pCtx->pNominatedSocketContext->pLocalCandidate->candidateType == ICE_CANDIDATE_TYPE_RELAY )
        {
            if( bufferLength + ICE_TURN_CHANNEL_DATA_MESSAGE_HEADER_LENGTH > ICE_CONTROLLER_MAX_MTU )
            {
                LogError( ( "The sending buffer is larger than MTU, length: %u", sendingBufferLength ) );
                ret = ICE_CONTROLLER_RESULT_FAIL_EXCEED_MTU;
            }
            else
            {
                memcpy( turnSendBuffer + ICE_TURN_CHANNEL_DATA_MESSAGE_HEADER_LENGTH, pBuffer, bufferLength );

                if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
                {
                    turnBufferLength = ICE_CONTROLLER_MAX_MTU;
                    iceResult = Ice_CreateTurnChannelDataMessage( &pCtx->iceContext,
                                                                  pCtx->pNominatedSocketContext->pCandidatePair,
                                                                  turnSendBuffer,
                                                                  bufferLength,
                                                                  &turnBufferLength );
                    xSemaphoreGive( pCtx->iceMutex );

                    if( ( iceResult != ICE_RESULT_OK ) && ( iceResult != ICE_RESULT_TURN_CHANNEL_DATA_HEADER_NOT_REQUIRED ) )
                    {
                        LogError( ( "Fail to create TURN channel data, result: %d", iceResult ) );
                        ret = ICE_CONTROLLER_RESULT_FAIL_CREATE_TURN_CHANNEL_DATA;
                    }
                    else
                    {
                        /* Redirect the output to the TURN server instead of remote endpoint. */
                        pDestEndpoint = &( pCtx->pNominatedSocketContext->pIceServer->iceEndpoint );

                        if( iceResult == ICE_RESULT_OK )
                        {
                            /* Set sending buffer/length to turn buffer since TURN channel header has been appended successfully. */
                            pSendingBuffer = turnSendBuffer;
                            sendingBufferLength = turnBufferLength;
                        }
                    }
                }
                else
                {
                    LogError( ( "Failed to create TURN channel data message: mutex lock acquisition." ) );
                    ret = ICE_CONTROLLER_RESULT_FAIL_MUTEX_TAKE;
                }
            }
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        ret = IceControllerNet_SendPacket( pCtx,
                                           pCtx->pNominatedSocketContext,
                                           pDestEndpoint,
                                           pSendingBuffer,
                                           sendingBufferLength );
    }

    return ret;
}

IceControllerResult_t IceController_AddIceServerConfig( IceControllerContext_t * pCtx,
                                                        IceControllerIceServerConfig_t * pIceServersConfig )
{
    IceControllerResult_t ret = ICE_CONTROLLER_RESULT_OK;
    int validIceServerCount = 0;

    if( ( pCtx == NULL ) ||
        ( pIceServersConfig == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pIceServersConfig: %p", pCtx, pIceServersConfig ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }
    else if( pIceServersConfig->rootCaPathLength > ICE_CONTROLLER_MAX_PATH_LENGTH )
    {
        LogError( ( "The root CA path is longer than buffer size, length: %u", pIceServersConfig->rootCaPathLength ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }
    else if( pIceServersConfig->rootCaPemLength > ICE_CONTROLLER_MAX_PEM_LENGTH )
    {
        LogError( ( "The root CA PEM is larger than buffer size, length: %u", pIceServersConfig->rootCaPemLength ) );
        ret = ICE_CONTROLLER_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( pIceServersConfig->rootCaPathLength > 0U )
        {
            memcpy( &pCtx->rootCaPath, pIceServersConfig->pRootCaPath, pIceServersConfig->rootCaPathLength );
            pCtx->rootCaPathLength = pIceServersConfig->rootCaPathLength;
            pCtx->rootCaPath[ pIceServersConfig->rootCaPathLength ] = '\0';
        }

        if( pIceServersConfig->rootCaPemLength > 0U )
        {
            memcpy( &pCtx->rootCaPem, pIceServersConfig->pRootCaPem, pIceServersConfig->rootCaPemLength );
            pCtx->rootCaPemLength = pIceServersConfig->rootCaPemLength;
            pCtx->rootCaPem[ pIceServersConfig->rootCaPemLength ] = '\0';
        }
    }

    if( ret == ICE_CONTROLLER_RESULT_OK )
    {
        if( pIceServersConfig->iceServersCount > ICE_CONTROLLER_MAX_ICE_SERVER_COUNT )
        {
            validIceServerCount = ICE_CONTROLLER_MAX_ICE_SERVER_COUNT;
            LogInfo( ( "Ice Controller supports a maximum of %d Ice servers. The additional %d servers will be dropped",
                       ICE_CONTROLLER_MAX_ICE_SERVER_COUNT,
                       pIceServersConfig->iceServersCount - ICE_CONTROLLER_MAX_ICE_SERVER_COUNT ) );
        }
        else
        {
            validIceServerCount = pIceServersConfig->iceServersCount;
        }
        memcpy( &( pCtx->iceServers[ 0 ] ), pIceServersConfig->pIceServers, validIceServerCount * sizeof( IceControllerIceServer_t ) );
        pCtx->iceServersCount = validIceServerCount;
    }

    return ret;
}

void IceController_CloseOtherCandidatePairs( IceControllerContext_t * pCtx,
                                             IceCandidatePair_t * pCandidatePair )
{
    uint8_t skipProcess = 0;
    uint8_t isLocked = 0U;
    size_t i;
    IceResult_t iceResult;
    size_t count;

    if( ( pCtx == NULL ) || ( pCandidatePair == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pCandidatePair: %p", pCtx, pCandidatePair ) );
        skipProcess = 1;
    }

    if( skipProcess == 0 )
    {
        if( xSemaphoreTake( pCtx->iceMutex, portMAX_DELAY ) == pdTRUE )
        {
            isLocked = 1U;
        }
        else
        {
            LogError( ( "Failed to release non-selected candidate pairs: mutex lock acquisition." ) );
            skipProcess = 1U;
        }
    }

    if( skipProcess == 0 )
    {
        iceResult = Ice_GetCandidatePairCount( &pCtx->iceContext,
                                               &count );
        if( iceResult != ICE_RESULT_OK )
        {
            LogError( ( "Fail to query valid candidate pair count, result: %d", iceResult ) );
            skipProcess = 1;
        }
    }

    if( skipProcess == 0 )
    {
        for( i = 0; i < count; i++ )
        {
            if( &pCtx->iceContext.pCandidatePairs[i] != pCandidatePair )
            {
                iceResult = Ice_CloseCandidatePair( &pCtx->iceContext, &pCtx->iceContext.pCandidatePairs[i] );
                if( iceResult != ICE_RESULT_OK )
                {
                    LogWarn( ( "Fail to close candidate pair, result: %d", iceResult ) );
                    continue;
                }
            }
        }
    }

    if( isLocked != 0U )
    {
        xSemaphoreGive( pCtx->iceMutex );
    }
}

void IceController_UpdateState( IceControllerContext_t * pCtx,
                                IceControllerState_t newState )
{
    if( pCtx == NULL )
    {
        LogError( ( "Invalid input, pCtx: %p", pCtx ) );
    }
    else
    {
        pCtx->state = newState;
    }
}

void IceController_UpdateTimerInterval( IceControllerContext_t * pCtx,
                                        uint32_t newIntervalMs )
{
    TimerControllerResult_t retTimer;
    uint8_t skipProcess = 0U;

    if( pCtx == NULL )
    {
        LogError( ( "Invalid input, pCtx: %p", pCtx ) );
        skipProcess = 1U;
    }

    if( skipProcess == 0U )
    {
        retTimer = TimerController_IsTimerSet( &pCtx->timerHandler );
        if( retTimer == TIMER_CONTROLLER_RESULT_SET )
        {
            TimerController_Reset( &pCtx->timerHandler );
        }
    }

    if( skipProcess == 0U )
    {
        /* Note that this timer is configured to one-shot timer. */
        retTimer = TimerController_SetTimer( &pCtx->timerHandler,
                                             newIntervalMs,
                                             0U );
        if( retTimer != TIMER_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to re-start timer, result: %d, intervalMs: %lu", retTimer, newIntervalMs ) );
        }
        else if( newIntervalMs != pCtx->timerIntervalMs )
        {
            LogInfo( ( "Timer interval is updated from %lu to %lu", pCtx->timerIntervalMs, newIntervalMs ) );
            pCtx->timerIntervalMs = newIntervalMs;
        }
        else
        {
            LogVerbose( ( "Timer interval is %lu", pCtx->timerIntervalMs ) );
        }
    }
}
