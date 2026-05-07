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
#include "networking.h"
#include "wslay_helper.h"
#include "core_http_client.h"
#include <base64.h>

/* Inlucde mbedtls for random/base64 */
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/sha1.h>

#include "errno.h"
#include "lwip/sockets.h"
#include "lwip_netconf.h"

#define NETWORKING_WSLAY_SEND_TIMEOUT_MS ( 10000 )
#define NETWORKING_WSLAY_RECV_TIMEOUT_MS ( 10000 )
#define NETWORKING_WSLAY_USER_AGENT_NAME_MAX_LENGTH ( 128 )
#define NETWORKING_WSLAY_HOST_NAME_MAX_LENGTH ( 256 )
#define NETWORKING_WSLAY_CLIENT_KEY_RANDOM_LENGTH ( 16 )
#define NETWORKING_WSLAY_CLIENT_KEY_LENGTH ( 24 )
#define NETWORKING_WSLAY_ACCEPT_KEY_LENGTH ( 28 )
#define NETWORKING_WSLAY_STRING_CHANNEL_ARN_PARAM_NAME "X-Amz-ChannelARN"
#define NETWORKING_WSLAY_STRING_CREDENTIAL_PARAM_NAME "X-Amz-Credential"
#define NETWORKING_WSLAY_STRING_DATE_PARAM_NAME "X-Amz-Date"
#define NETWORKING_WSLAY_STRING_EXPIRES_PARAM_NAME "X-Amz-Expires"
#define NETWORKING_WSLAY_STRING_SIGNED_HEADERS_PARAM_NAME "X-Amz-SignedHeaders"
#define NETWORKING_WSLAY_STRING_SECURITY_TOKEN_PARAM_NAME "X-Amz-Security-Token"
#define NETWORKING_WSLAY_STRING_SIGNATURE_PARAM_NAME "X-Amz-Signature"
#define NETWORKING_WSLAY_STRING_SIGNED_HEADERS_VALUE "host"
#define NETWORKING_WSLAY_STRING_RFC6455_UUID "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
#define NETWORKING_WSLAY_RFC6455_UUID_LENGTH ( 36 )
#define NETWORKING_WSLAY_SHA1_LENGTH ( 20 )
#define NETWORKING_WSLAY_PING_PONG_INTERVAL_SEC ( 10 )
#define NETWORKING_WSLAY_PING_PONG_INTERVAL_TICKS ( pdMS_TO_TICKS( NETWORKING_WSLAY_PING_PONG_INTERVAL_SEC * 1000 ) ) /* 10s to ticks */

#define NETWORKING_WSLAY_STRING_CREDENTIAL_VALUE_TEMPLATE "%.*s/%.*s/%.*s/" NETWORKING_UTILS_KVS_SERVICE_NAME "/aws4_request"

#define NETWORKING_WSLAY_CREDENTIAL_PARAM_DATE_LENGTH ( 8 )
#define NETWORKING_WSLAY_STATIC_CRED_EXPIRES_SECONDS ( 604800 )

#define NETWORKING_WSLAY_URI_ENCODED_CHAR_SIZE ( 3 ) // We need 3 char spaces to translate symbols, such as from '/' to "%2F".
#define NETWORKING_WSLAY_URI_ENCODED_FORWARD_SLASH "%2F"

static void GenerateRandomBytes( uint8_t * pBuffer,
                                 size_t bufferLength );
static void HandleWslayControlMessage( void * pUserData,
                                       uint8_t opcode,
                                       const uint8_t * pData,
                                       size_t dataLength );
static void HandleWslayDataMessage( void * pUserData,
                                    const uint8_t * pData,
                                    size_t dataLength );

static int32_t SendTlsPacket( NetworkContext_t * pNetworkContext,
                              const void * pBuffer,
                              size_t bytesToSend )
{
    return TLS_FreeRTOS_send( ( TlsNetworkContext_t * ) pNetworkContext, pBuffer, bytesToSend );
}

static int32_t RecvTlsPacket( NetworkContext_t * pNetworkContext,
                              void * pBuffer,
                              size_t bytesToRecv )
{
    return TLS_FreeRTOS_recv( ( TlsNetworkContext_t * ) pNetworkContext, pBuffer, bytesToRecv );
}

static ssize_t WslaySendCallback( wslay_event_context_ptr pCtx,
                                  const uint8_t * pData,
                                  size_t dataLength,
                                  int flags,
                                  void * pUserData )
{
    NetworkingWslayContext_t * pContext = ( NetworkingWslayContext_t * ) pUserData;
    TransportSend_t sendFunction = pContext->xTransportInterface.send;
    ssize_t r = ( ssize_t ) sendFunction( ( NetworkContext_t * ) &pContext->xTlsNetworkContext, pData, dataLength );

    if( r < 0 )
    {
        wslay_event_set_error( pCtx, WSLAY_ERR_CALLBACK_FAILURE );
        LogError( ( "WslaySendCallback failed with return %d", r ) );
    }
    else if( r == 0 )
    {
        wslay_event_set_error( pCtx, WSLAY_ERR_WOULDBLOCK );
        LogDebug( ( "WslaySendCallback returns 0" ) );
    }
    else
    {
        /* Sent successfully. */
    }

    return r;
}

static ssize_t WslayRecvCallback( wslay_event_context_ptr pCtx,
                                  uint8_t * pData,
                                  size_t dataLength,
                                  int flags,
                                  void * pUserData )
{
    NetworkingWslayContext_t * pContext = ( NetworkingWslayContext_t * ) pUserData;
    TransportRecv_t recvFunction = pContext->xTransportInterface.recv;
    ssize_t r = ( ssize_t ) recvFunction( ( NetworkContext_t * ) &pContext->xTlsNetworkContext, pData, dataLength );

    if( r < 0 )
    {
        wslay_event_set_error( pCtx, WSLAY_ERR_CALLBACK_FAILURE );
        LogError( ( "WslayRecvCallback failed with return %d", r ) );
    }
    else if( r == 0 )
    {
        wslay_event_set_error( pCtx, WSLAY_ERR_WOULDBLOCK );
        LogVerbose( ( "WslayRecvCallback: No data received (would block)" ) );
    }
    else
    {
        /* Recv successfully. */
    }

    return r;
}

static int WslayGenmaskCallback( wslay_event_context_ptr pCtx,
                                 uint8_t * pBuf,
                                 size_t bufLength,
                                 void * pUserData )
{
    ( void ) pCtx;
    ( void ) pUserData;

    GenerateRandomBytes( pBuf, bufLength );
    return 0;
}

static void WslayMsgRecvCallback( wslay_event_context_ptr pCtx,
                                  const struct wslay_event_on_msg_recv_arg * pArg,
                                  void * pUserData )
{
    if( !wslay_is_ctrl_frame( pArg->opcode ) )
    {
        HandleWslayDataMessage( pUserData, pArg->msg, pArg->msg_length );
    }
    else
    {
        HandleWslayControlMessage( pUserData, pArg->opcode, pArg->msg, pArg->msg_length );
    }
}

static void HandleWslayControlMessage( void * pUserData,
                                       uint8_t opcode,
                                       const uint8_t * pData,
                                       size_t dataLength )
{
    NetworkingWslayContext_t * pContext = ( NetworkingWslayContext_t * ) pUserData;

    if( opcode == WSLAY_PONG )
    {
        LogInfo( ( "<== wss pong" ) );
    }
    else if( opcode == WSLAY_PING )
    {
        LogInfo( ( "<== wss ping, len: %u", dataLength ) );
    }
    else if( opcode == WSLAY_CONNECTION_CLOSE )
    {
        LogInfo( ( "<== connection close, msg len: %u", dataLength ) );
        pContext->connectionCloseRequested = 1U;
    }
    else
    {
        LogInfo( ( "<== ctrl msg(%u), len: %u", opcode, dataLength ) );
    }
}

static void HandleWslayDataMessage( void * pUserData,
                                    const uint8_t * pData,
                                    size_t dataLength )
{
    NetworkingWslayContext_t * pContext = ( NetworkingWslayContext_t * ) pUserData;

    ( void ) pContext->websocketRxCallback( ( char * ) pData, dataLength, pContext->pWebsocketRxCallbackContext );
}

static NetworkingWslayResult_t EncodeUriString( char * pSrc,
                                                size_t srcLength,
                                                char * pDst,
                                                size_t * pDstLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    size_t encodedLength = 0, remainLength;
    char * pCurPtr = pSrc, * pEnc = pDst;
    char ch;
    const char alpha[17] = "0123456789ABCDEF";

    if( ( pSrc == NULL ) || ( pDst == NULL ) || ( pDstLength == NULL ) )
    {
        ret = NETWORKING_WSLAY_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        // Set the remainLength length
        remainLength = *pDstLength;

        while( ( ( size_t ) ( pCurPtr - pSrc ) < srcLength ) && ( ( ch = *pCurPtr++ ) != '\0' ) )
        {
            if( ( ( ch >= 'A' ) && ( ch <= 'Z' ) ) || ( ( ch >= 'a' ) && ( ch <= 'z' ) ) || ( ( ch >= '0' ) && ( ch <= '9' ) ) || ( ch == '_' ) || ( ch == '-' ) || ( ch == '~' ) || ( ch == '.' ) )
            {
                if( remainLength < 1U )
                {
                    ret = NETWORKING_WSLAY_RESULT_URI_ENCODED_BUFFER_TOO_SMALL;
                    break;
                }

                encodedLength++;
                *pEnc++ = ch;
                remainLength--;
            }
            else if( ch == '/' )
            {
                if( remainLength < NETWORKING_WSLAY_URI_ENCODED_CHAR_SIZE )
                {
                    ret = NETWORKING_WSLAY_RESULT_URI_ENCODED_BUFFER_TOO_SMALL;
                    break;
                }

                encodedLength += NETWORKING_WSLAY_URI_ENCODED_CHAR_SIZE;
                strncpy( pEnc, NETWORKING_WSLAY_URI_ENCODED_FORWARD_SLASH, remainLength );
                pEnc += NETWORKING_WSLAY_URI_ENCODED_CHAR_SIZE;
                remainLength -= NETWORKING_WSLAY_URI_ENCODED_CHAR_SIZE;
            }
            else
            {
                if( remainLength < NETWORKING_WSLAY_URI_ENCODED_CHAR_SIZE )
                {
                    ret = NETWORKING_WSLAY_RESULT_URI_ENCODED_BUFFER_TOO_SMALL;
                    break;
                }

                encodedLength += NETWORKING_WSLAY_URI_ENCODED_CHAR_SIZE;
                *pEnc++ = '%';
                *pEnc++ = alpha[ch >> 4];
                *pEnc++ = alpha[ch & 0x0f];
                remainLength -= NETWORKING_WSLAY_URI_ENCODED_CHAR_SIZE;
            }
        }
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        *pDstLength -= remainLength;
    }

    return ret;
}

static NetworkingWslayResult_t WriteUriEncodedAlgorithm( char ** ppBuffer,
                                                         size_t * pBufferLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    int32_t writtenLength;

    writtenLength = snprintf( *ppBuffer, *pBufferLength, "X-Amz-Algorithm=AWS4-HMAC-SHA256" );

    if( writtenLength < 0 )
    {
        ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
    }
    else if( writtenLength == *pBufferLength )
    {
        ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
    }
    else
    {
        *ppBuffer += writtenLength;
        *pBufferLength -= writtenLength;
    }

    return ret;
}

static NetworkingWslayResult_t WriteUriEncodedChannelArn( char ** ppBuffer,
                                                          size_t * pBufferLength,
                                                          char * pChannelArn,
                                                          size_t channelArnLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    int32_t writtenLength;
    size_t encodedLength;

    /* X-Amz-ChannelARN query parameter. */
    writtenLength = snprintf( *ppBuffer, *pBufferLength, "&X-Amz-ChannelARN=" );

    if( writtenLength < 0 )
    {
        ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
    }
    else if( writtenLength == *pBufferLength )
    {
        ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
    }
    else
    {
        *ppBuffer += writtenLength;
        *pBufferLength -= writtenLength;
    }

    /* X-Amz-ChannelARN value (plaintext). */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        writtenLength = snprintf( *ppBuffer, *pBufferLength, "%.*s",
                                  ( int ) channelArnLength, pChannelArn );

        if( writtenLength < 0 )
        {
            ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
        }
        else if( writtenLength == *pBufferLength )
        {
            ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
        }
        else
        {
            /* Keep the pointer and *pBufferLength for URI encoded. */
        }
    }

    /* X-Amz-ChannelARN value (URI encoded). */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        encodedLength = *pBufferLength - writtenLength;
        ret = EncodeUriString( *ppBuffer, writtenLength, ( *ppBuffer ) + writtenLength, &encodedLength );

        /* Move and update pointer/remain length. */
        if( ret == NETWORKING_WSLAY_RESULT_OK )
        {
            memmove( *ppBuffer, *ppBuffer + writtenLength, encodedLength );
            *ppBuffer += encodedLength;
            *pBufferLength -= encodedLength;
        }
    }

    return ret;
}

static NetworkingWslayResult_t WriteUriEncodedCredential( NetworkingWslayContext_t * pWebsocketCtx,
                                                          const AwsCredentials_t * pAwsCredentials,
                                                          char ** ppBuffer,
                                                          size_t * pBufferLength,
                                                          const char * pDate )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    int32_t writtenLength;
    size_t encodedLength;

    /* X-Amz-Credential query parameter. */
    writtenLength = snprintf( *ppBuffer, *pBufferLength, "&" NETWORKING_WSLAY_STRING_CREDENTIAL_PARAM_NAME "=" );

    if( writtenLength < 0 )
    {
        ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
    }
    else if( writtenLength == *pBufferLength )
    {
        ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
    }
    else
    {
        *ppBuffer += writtenLength;
        *pBufferLength -= writtenLength;
    }

    /* X-Amz-Credential value (plaintext). */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        writtenLength = snprintf( *ppBuffer, *pBufferLength, NETWORKING_WSLAY_STRING_CREDENTIAL_VALUE_TEMPLATE,
                                  ( int ) pAwsCredentials->accessKeyIdLength, pAwsCredentials->pAccessKeyId,
                                  NETWORKING_WSLAY_CREDENTIAL_PARAM_DATE_LENGTH, pDate,
                                  ( int ) pAwsCredentials->regionLength, pAwsCredentials->pRegion );

        if( writtenLength < 0 )
        {
            ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
        }
        else if( writtenLength == *pBufferLength )
        {
            ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
        }
        else
        {
            /* Keep the pointer and pBufferLength for URI encoded. */
        }
    }

    /* X-Amz-Credential value (URI encoded) */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        encodedLength = *pBufferLength - writtenLength;
        ret = EncodeUriString( *ppBuffer, writtenLength, *ppBuffer + writtenLength, &encodedLength );

        /* Move and update pointer/remain length. */
        if( ret == NETWORKING_WSLAY_RESULT_OK )
        {
            memmove( *ppBuffer, *ppBuffer + writtenLength, encodedLength );
            *ppBuffer += encodedLength;
            *pBufferLength -= encodedLength;
        }
    }

    return ret;
}

static NetworkingWslayResult_t WriteUriEncodedDate( char ** ppBuffer,
                                                    size_t * pBufferLength,
                                                    const char * pDate,
                                                    size_t dateLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    int32_t writtenLength;
    size_t encodedLength;

    /* X-Amz-Date query parameter. */
    writtenLength = snprintf( *ppBuffer, *pBufferLength, "&" NETWORKING_WSLAY_STRING_DATE_PARAM_NAME "=" );

    if( writtenLength < 0 )
    {
        ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
    }
    else if( writtenLength == *pBufferLength )
    {
        ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
    }
    else
    {
        *ppBuffer += writtenLength;
        *pBufferLength -= writtenLength;
    }

    /* X-Amz-Date value (plaintext). */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        writtenLength = snprintf( *ppBuffer, *pBufferLength, "%.*s",
                                  ( int ) dateLength, pDate );

        if( writtenLength < 0 )
        {
            ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
        }
        else if( writtenLength == *pBufferLength )
        {
            ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
        }
        else
        {
            /* Keep the pointer and pBufferLength for URI encoded. */
        }
    }

    /* X-Amz-Date value (URI encoded) */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        encodedLength = *pBufferLength - writtenLength;
        ret = EncodeUriString( *ppBuffer, writtenLength, *ppBuffer + writtenLength, &encodedLength );

        /* Move and update pointer/remain length. */
        if( ret == NETWORKING_WSLAY_RESULT_OK )
        {
            memmove( *ppBuffer, *ppBuffer + writtenLength, encodedLength );
            *ppBuffer += encodedLength;
            *pBufferLength -= encodedLength;
        }
    }

    return ret;
}

static NetworkingWslayResult_t WriteUriEncodedExpires( char ** ppBuffer,
                                                       size_t * pBufferLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    int32_t writtenLength;
    size_t encodedLength;

    /* X-Amz-Expires query parameter. */
    writtenLength = snprintf( *ppBuffer, *pBufferLength, "&" NETWORKING_WSLAY_STRING_EXPIRES_PARAM_NAME "=" );

    if( writtenLength < 0 )
    {
        ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
    }
    else if( writtenLength == *pBufferLength )
    {
        ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
    }
    else
    {
        *ppBuffer += writtenLength;
        *pBufferLength -= writtenLength;
    }

    /* X-Amz-Expires value (plaintext). */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        writtenLength = snprintf( *ppBuffer, *pBufferLength, "%d", NETWORKING_WSLAY_STATIC_CRED_EXPIRES_SECONDS );

        if( writtenLength < 0 )
        {
            ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
        }
        else if( writtenLength == *pBufferLength )
        {
            ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
        }
        else
        {
            /* Keep the pointer and pBufferLength for URI encoded. */
        }
    }

    /* X-Amz-Expires value (URI encoded) */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        encodedLength = *pBufferLength - writtenLength;
        ret = EncodeUriString( *ppBuffer, writtenLength, *ppBuffer + writtenLength, &encodedLength );

        /* Move and update pointer/remain length. */
        if( ret == NETWORKING_WSLAY_RESULT_OK )
        {
            memmove( *ppBuffer, *ppBuffer + writtenLength, encodedLength );
            *ppBuffer += encodedLength;
            *pBufferLength -= encodedLength;
        }
    }

    return ret;
}

static NetworkingWslayResult_t WriteUriEncodedSignedHeaders( char ** ppBuffer,
                                                             size_t * pBufferLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    int32_t writtenLength;

    /* X-Amz-SignedHeaders query parameter. */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        writtenLength = snprintf( *ppBuffer, *pBufferLength, "&" NETWORKING_WSLAY_STRING_SIGNED_HEADERS_PARAM_NAME "=" NETWORKING_WSLAY_STRING_SIGNED_HEADERS_VALUE );

        if( writtenLength < 0 )
        {
            ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
        }
        else if( writtenLength == *pBufferLength )
        {
            ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
        }
        else
        {
            *ppBuffer += writtenLength;
            *pBufferLength -= writtenLength;
        }
    }

    return ret;
}

static NetworkingWslayResult_t WriteUriEncodeSecurityToken( NetworkingWslayContext_t * pWebsocketCtx,
                                                            const AwsCredentials_t * pAwsCredentials,
                                                            char ** ppBuffer,
                                                            size_t * pBufferLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    int writtenLength;
    size_t encodedLength;

    /* X-Amz-Security-Token query parameter. */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        writtenLength = snprintf( *ppBuffer, *pBufferLength, "&" NETWORKING_WSLAY_STRING_SECURITY_TOKEN_PARAM_NAME "=" );

        if( writtenLength < 0 )
        {
            ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
        }
        else if( writtenLength == *pBufferLength )
        {
            ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
        }
        else
        {
            *ppBuffer += writtenLength;
            *pBufferLength -= writtenLength;
        }
    }

    /* X-Amz-Security-Token value (plaintext). */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        writtenLength = snprintf( *ppBuffer, *pBufferLength, "%.*s",
                                  ( int ) pAwsCredentials->sessionTokenLength, pAwsCredentials->pSessionToken );

        if( writtenLength < 0 )
        {
            ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
        }
        else if( writtenLength == *pBufferLength )
        {
            ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
        }
        else
        {
            /* Keep the pointer and pBufferLength for URI encoded. */
        }
    }

    /* X-Amz-Security-Token value (URI encoded) */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        encodedLength = *pBufferLength - writtenLength;
        ret = EncodeUriString( *ppBuffer, writtenLength, *ppBuffer + writtenLength, &encodedLength );

        /* Move and update pointer/remain length. */
        if( ret == NETWORKING_WSLAY_RESULT_OK )
        {
            memmove( *ppBuffer, *ppBuffer + writtenLength, encodedLength );
            *ppBuffer += encodedLength;
            *pBufferLength -= encodedLength;
        }
    }

    return ret;

}

static WebsocketResult_t GenerateQueryParameters( NetworkingWslayContext_t * pWebsocketCtx,
                                                  const AwsCredentials_t * pAwsCredentials,
                                                  const char * pUrl,
                                                  size_t urlLength,
                                                  const char * pHost,
                                                  size_t hostLength,
                                                  const char * pPath,
                                                  size_t pathLength,
                                                  char * pOutput,
                                                  size_t * pOutputLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    NetworkingUtilsResult_t retUtils;
    const char * pQueryStart;
    size_t queryLength;
    char * pChannelArnValue, * pEqual;
    size_t channelArnValueLength;
    char * pCurrentWrite = pOutput;
    size_t remainLength, EncodeUriStringLength;
    int32_t writtenLength;
    char dateBuffer[ NETWORKING_UTILS_TIME_BUFFER_LENGTH ];
    NetworkingUtilsCanonicalRequest_t canonicalRequest;
    char * pSig;
    size_t sigLength;
    SigV4Credentials_t sigv4Credential;
    char * pAmpersand = NULL, * pClientId = NULL;
    size_t clientIdlength = 0;

    if( ( pUrl == NULL ) || ( pHost == NULL ) || ( pPath == NULL ) || ( pOutput == NULL ) || ( pOutputLength == NULL ) )
    {
        ret = NETWORKING_WSLAY_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        retUtils = NetworkingUtils_GetIso8601CurrentTime( dateBuffer, NETWORKING_UTILS_TIME_BUFFER_LENGTH );

        if( retUtils != NETWORKING_UTILS_RESULT_OK )
        {
            LogError( ( "Fail to get current ISO8601 date" ) );
            ret = NETWORKING_WSLAY_RESULT_FAIL_GET_DATE;
        }
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        pQueryStart = pPath + pathLength + 1; // +1 to skip '?' mark.
        queryLength = pUrl + urlLength - pQueryStart;

        if( ( queryLength < strlen( NETWORKING_WSLAY_STRING_CHANNEL_ARN_PARAM_NAME ) ) ||
            ( strncmp( pQueryStart, NETWORKING_WSLAY_STRING_CHANNEL_ARN_PARAM_NAME, strlen( NETWORKING_WSLAY_STRING_CHANNEL_ARN_PARAM_NAME ) ) != 0 ) )
        {
            /* No channel ARN exist. */
            ret = NETWORKING_WSLAY_RESULT_UNEXPECTED_WEBSOCKET_URL;
        }
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        /* Parse existing query parameters. */
        pEqual = strchr( pQueryStart, '=' );
        pAmpersand = strchr( pQueryStart, '&' );
        if( pEqual == NULL )
        {
            /* No equal found, unexpected. */
            ret = NETWORKING_WSLAY_RESULT_UNEXPECTED_WEBSOCKET_URL;
        }
        else
        {
            /* There are two possibilities in URL:
             * 1. Master: X-Amz-ChannelARN=${ARN} follow by nothing.
             * 2. Viewer: X-Amz-ChannelARN=${ARN}&X-Amz-ClientId=${ID} */
            if( pAmpersand == NULL )
            {
                /* Master case */
                pChannelArnValue = pEqual + 1;
                channelArnValueLength = pQueryStart + queryLength - pChannelArnValue;
                clientIdlength = 0;
            }
            else
            {
                /* Viewer case */
                pChannelArnValue = pEqual + 1;
                channelArnValueLength = pAmpersand - pChannelArnValue;
                pClientId = ( pAmpersand + 1 );
                clientIdlength = pQueryStart + queryLength - pClientId;
            }
        }
    }

    /* Append X-Amz-Algorithm, X-Amz-ChannelARN, X-Amz-Credential, X-Amz-Date, X-Amz-Expires, and X-Amz-SignedHeaders first
     * to generate signature. Then append X-Amz-Signature after getting it from sigv4 API.
     *
     * Note that the order of query parameters is important. */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        remainLength = *pOutputLength;

        ret = WriteUriEncodedAlgorithm( &pCurrentWrite, &remainLength );
    }

    /* X-Amz-ChannelARN query parameter. */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        ret = WriteUriEncodedChannelArn( &pCurrentWrite, &remainLength, pChannelArnValue, channelArnValueLength );
    }

    /* X-Amz-ChannelARN query parameter. */
    if( ( ret == NETWORKING_WSLAY_RESULT_OK ) &&
        ( pClientId != NULL ) )
    {
        /* Write X-Amz-ClientId for viewer when exist. */
        writtenLength = snprintf( pCurrentWrite,
                                  remainLength,
                                  "&%.*s",
                                  ( int ) clientIdlength,
                                  pClientId );
        if( ( writtenLength < 0 ) || ( writtenLength == remainLength ) )
        {
            LogError( ( "Failed to write X-Amz-ClientId key!" ) );
            ret = -1;
        }
        else
        {
            pCurrentWrite += writtenLength;
            remainLength -= writtenLength;
        }
    }

    /* X-Amz-Credential query parameter. */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        ret = WriteUriEncodedCredential( pWebsocketCtx, pAwsCredentials, &pCurrentWrite, &remainLength, dateBuffer );
    }

    /* X-Amz-Date query parameter. */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        ret = WriteUriEncodedDate( &pCurrentWrite, &remainLength, dateBuffer, NETWORKING_UTILS_TIME_BUFFER_LENGTH - 1 );
    }

    /* X-Amz-Expires query parameter. */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        ret = WriteUriEncodedExpires( &pCurrentWrite, &remainLength );
    }

    /* X-Amz-Security-Token query parameter. */
    if( ( ret == NETWORKING_WSLAY_RESULT_OK ) && ( pAwsCredentials->sessionTokenLength > 0U ) )
    {
        ret = WriteUriEncodeSecurityToken( pWebsocketCtx, pAwsCredentials, &pCurrentWrite, &remainLength );
    }

    /* X-Amz-SignedHeaders query parameter. */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        ret = WriteUriEncodedSignedHeaders( &pCurrentWrite, &remainLength );
    }

    /* Follow https://docs.aws.amazon.com/IAM/latest/UserGuide/create-signed-request.html to create canonical headers.
     * Websocket Format: "host: kinesisvideo.us-west-2.amazonaws.com\r\n"
     *
     * Note that we re-use the parsed result in pAppendHeaders from Websocket_Connect(). */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        EncodeUriStringLength = ( *pOutputLength ) - remainLength;
        writtenLength = snprintf( pCurrentWrite, remainLength,
                                  "%s: %.*s\r\n", "host", ( int ) hostLength, pHost );

        if( writtenLength < 0 )
        {
            ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
        }
        else if( writtenLength == remainLength )
        {
            ret = NETWORKING_WSLAY_RESULT_AUTH_BUFFER_TOO_SMALL;
        }
        else
        {
            /* Do nothing, Coverity happy. */
        }
    }

    /* Sign the HTTP request. */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        memset( &canonicalRequest, 0, sizeof( canonicalRequest ) );
        canonicalRequest.verb = NETWORKING_UTILS_HTTP_VERB_GET;
        canonicalRequest.pPath = ( char * ) pPath;
        canonicalRequest.pathLength = pathLength;
        canonicalRequest.pCanonicalQueryString = pOutput;
        canonicalRequest.canonicalQueryStringLength = EncodeUriStringLength;
        canonicalRequest.pCanonicalHeaders = pCurrentWrite;
        canonicalRequest.canonicalHeadersLength = writtenLength;
        canonicalRequest.pPayload = NULL;
        canonicalRequest.payloadLength = 0;

        sigv4Credential.pAccessKeyId = pAwsCredentials->pAccessKeyId;
        sigv4Credential.accessKeyIdLen = pAwsCredentials->accessKeyIdLength;
        sigv4Credential.pSecretAccessKey = pAwsCredentials->pSecretAccessKey;
        sigv4Credential.secretAccessKeyLen = pAwsCredentials->secretAccessKeyLength;

        pWebsocketCtx->sigv4AuthBufferLength = NETWORKING_META_BUFFER_LENGTH;
        retUtils = NetworkingUtils_GenrerateAuthorizationHeader( &canonicalRequest, &sigv4Credential,
                                                                 pAwsCredentials->pRegion, pAwsCredentials->regionLength, dateBuffer,
                                                                 pWebsocketCtx->sigv4AuthBuffer, &pWebsocketCtx->sigv4AuthBufferLength,
                                                                 &pSig, &sigLength );

        if( retUtils != NETWORKING_UTILS_RESULT_OK )
        {
            LogError( ( "Fail to generate authorization header, return=%d", retUtils ) );
            ret = NETWORKING_WSLAY_RESULT_FAIL_CONNECT;
        }
    }

    /* Append signature. */
    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        writtenLength = snprintf( pCurrentWrite, remainLength, "&" NETWORKING_WSLAY_STRING_SIGNATURE_PARAM_NAME "=%.*s",
                                  ( int ) sigLength, pSig );

        if( writtenLength < 0 )
        {
            ret = NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF;
        }
        else if( writtenLength == remainLength )
        {
            ret = NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL;
        }
        else
        {
            remainLength -= writtenLength;
        }
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        *pOutputLength = *pOutputLength - remainLength;
    }

    return ret;
}

static WebsocketResult_t AddHeader( HTTPRequestHeaders_t * pxRequestHeaders,
                                    const char * pName,
                                    size_t nameLength,
                                    const char * pValue,
                                    size_t valueLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    HTTPStatus_t xHttpStatus = HTTPSuccess;

    if( ( pxRequestHeaders == NULL ) || ( pName == NULL ) )
    {
        LogError( ( "Invalid input while adding header." ) );
        ret = NETWORKING_WSLAY_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        xHttpStatus = HTTPClient_AddHeader( pxRequestHeaders,
                                            pName,
                                            nameLength,
                                            pValue,
                                            valueLength );

        if( xHttpStatus != HTTPSuccess )
        {
            LogError( ( "Failed to add %.*s. Error=%s.",
                        ( int ) nameLength, pName,
                        HTTPClient_strerror( xHttpStatus ) ) );
            ret = NETWORKING_WSLAY_RESULT_FAIL_HTTP_ADD;
        }
    }

    return ret;
}

static void GenerateRandomBytes( uint8_t * pBuffer,
                                 size_t bufferLength )
{
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;

    mbedtls_entropy_init( &entropy );
    mbedtls_ctr_drbg_init( &ctr_drbg );
    if( mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy, NULL, 0 ) == 0 )
    {
        mbedtls_ctr_drbg_random( &ctr_drbg, pBuffer, bufferLength );
    }

    mbedtls_ctr_drbg_free( &ctr_drbg );
    mbedtls_entropy_free( &entropy );
}

static WebsocketResult_t GenerateWebSocketClientKey( char * pClientKey,
                                                     size_t clientKeyLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    uint8_t randomBuffer[ NETWORKING_WSLAY_CLIENT_KEY_RANDOM_LENGTH ];
    Base64Result_t retBase64;
    size_t outputLength = clientKeyLength;

    if( ( pClientKey == NULL ) || ( clientKeyLength < NETWORKING_WSLAY_CLIENT_KEY_LENGTH + 1 ) )
    {
        ret = NETWORKING_WSLAY_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        GenerateRandomBytes( randomBuffer, NETWORKING_WSLAY_CLIENT_KEY_RANDOM_LENGTH );

        retBase64 = Base64_Encode( ( const char * ) randomBuffer, NETWORKING_WSLAY_CLIENT_KEY_RANDOM_LENGTH, pClientKey, &outputLength );
        if( retBase64 != BASE64_RESULT_OK )
        {
            LogError( ( "Fail to base64 encode to generate client key, return=0x%x", retBase64 ) );

            ret = NETWORKING_WSLAY_RESULT_FAIL_BASE64_ENCODE;
        }
        else
        {
            LogInfo( ( "Base64 encode output length %u, original length %u", outputLength, clientKeyLength ) );
        }
    }

    return ret;
}

static WebsocketResult_t GenerateAcceptKey( const char * pClientKey,
                                            size_t clientKeyLength,
                                            char * pOutAcceptKey,
                                            size_t outAcceptKeyLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    char tempBuffer[ clientKeyLength + NETWORKING_WSLAY_RFC6455_UUID_LENGTH ];
    uint8_t sha1Buffer[ NETWORKING_WSLAY_SHA1_LENGTH ];
    Base64Result_t retBase64;
    size_t outputLength = outAcceptKeyLength;

    if( ( pClientKey == NULL ) || ( pOutAcceptKey == NULL ) || ( outAcceptKeyLength < NETWORKING_WSLAY_ACCEPT_KEY_LENGTH + 1 ) )
    {
        ret = NETWORKING_WSLAY_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        memset( tempBuffer, 0, sizeof( tempBuffer ) );
        memset( sha1Buffer, 0, sizeof( sha1Buffer ) );

        memcpy( tempBuffer, pClientKey, clientKeyLength );
        memcpy( tempBuffer + clientKeyLength, NETWORKING_WSLAY_STRING_RFC6455_UUID, NETWORKING_WSLAY_RFC6455_UUID_LENGTH );

        mbedtls_sha1( ( unsigned char * ) tempBuffer, clientKeyLength + NETWORKING_WSLAY_RFC6455_UUID_LENGTH, sha1Buffer );
        retBase64 = Base64_Encode( ( const char * ) sha1Buffer, NETWORKING_WSLAY_SHA1_LENGTH, pOutAcceptKey, &outputLength );
        if( retBase64 != BASE64_RESULT_OK )
        {
            LogError( ( "Fail to base64 encode to generate accept key, return=0x%x", retBase64 ) );

            ret = NETWORKING_WSLAY_RESULT_FAIL_BASE64_ENCODE;
        }
        else
        {
            LogInfo( ( "Base64 encode output length %u, original length %u", outputLength, outAcceptKeyLength ) );
        }
    }

    return ret;
}

static WebsocketResult_t AcceptClientKey( const char * pAcceptKey,
                                          size_t acceptKeyLength,
                                          const char * pClientKey,
                                          size_t clientKeyLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    char tempAcceptKey[ NETWORKING_WSLAY_ACCEPT_KEY_LENGTH + 1 ];
    //

    if( ( pAcceptKey == NULL ) || ( pClientKey == NULL ) )
    {
        ret = NETWORKING_WSLAY_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        ret = GenerateAcceptKey( pClientKey, clientKeyLength, tempAcceptKey, NETWORKING_WSLAY_ACCEPT_KEY_LENGTH + 1 );
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        if( memcmp( pAcceptKey, tempAcceptKey, acceptKeyLength ) != 0 )
        {
            LogError( ( "The accept key in response is invalid, accept key(%d)=%.*s, expected accept key(%d)=%.*s",
                        acceptKeyLength, ( int ) acceptKeyLength, pAcceptKey,
                        NETWORKING_WSLAY_ACCEPT_KEY_LENGTH, NETWORKING_WSLAY_ACCEPT_KEY_LENGTH, tempAcceptKey ) );
            ret = NETWORKING_WSLAY_RESULT_FAIL_VERIFY_ACCEPT_KEY;
        }
    }

    return ret;
}

static void ParsingConnectResponseHeader( void * pContext,
                                          const char * fieldLoc,
                                          size_t fieldLen,
                                          const char * valueLoc,
                                          size_t valueLen,
                                          uint16_t statusCode )
{
    NetworkingWslayConnectResponseContext_t * pConnectContext = ( NetworkingWslayConnectResponseContext_t * ) pContext;
    NetworkingWslayResult_t ret;

    LogDebug( ( "statusCode=%u, field(%d)=%.*s, value(%d)=%.*s", statusCode,
                fieldLen, fieldLen, fieldLoc,
                valueLen, valueLen, valueLoc ) );

    pConnectContext->statusCode = statusCode;
    if( strncmp( fieldLoc, "Connection", fieldLen ) == 0 )
    {
        if( strncmp( valueLoc, "upgrade", valueLen ) == 0 )
        {
            pConnectContext->headersParsed |= NETWORKING_WSLAY_HTTP_HEADER_CONNECTION;
        }
    }
    else if( strncmp( fieldLoc, "upgrade", fieldLen ) == 0 )
    {
        if( strncmp( valueLoc, "websocket", valueLen ) == 0 )
        {
            pConnectContext->headersParsed |= NETWORKING_WSLAY_HTTP_HEADER_UPGRADE;
        }
    }
    else if( strncmp( fieldLoc, "sec-websocket-accept", fieldLen ) == 0 )
    {
        /* Verify client key. */
        ret = AcceptClientKey( valueLoc, valueLen, pConnectContext->pClientKey, pConnectContext->clientKeyLength );
        if( ret == NETWORKING_WSLAY_RESULT_OK )
        {
            pConnectContext->headersParsed |= NETWORKING_WSLAY_HTTP_HEADER_WEBSOCKET_ACCEPT;
        }
    }
    else
    {
        /* Ignore other headers. */
    }
}

static WebsocketResult_t InitializeWslayContext( NetworkingWslayContext_t * pWebsocketCtx )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    struct wslay_event_callbacks callbacks = {
        WslayRecvCallback,    /* wslay_event_recv_callback */
        WslaySendCallback,    /* wslay_event_send_callback */
        WslayGenmaskCallback, /* wslay_event_genmask_callback */
        NULL,                   /* wslay_event_on_frame_recv_start_callback */
        NULL,                   /* wslay_event_on_frame_recv_chunk_callback */
        NULL,                   /* wslay_event_on_frame_recv_end_callback */
        WslayMsgRecvCallback /* wslay_event_on_msg_recv_callback */
    };

    wslay_event_context_client_init( &pWebsocketCtx->wslayContext, &callbacks, pWebsocketCtx );
    pWebsocketCtx->lastPingTick = xTaskGetTickCount();

    return ret;
}

static WebsocketResult_t InitializeWakeUpSocket( NetworkingWslayContext_t * pWebsocketCtx )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    uint32_t socketTimeoutMs = 1U;
    struct sockaddr * sockAddress = NULL;
    socklen_t addressLength;

    pWebsocketCtx->socketWakeUp = socket( PF_INET, SOCK_DGRAM, 0 );
    if( pWebsocketCtx->socketWakeUp < 0 )
    {
        LogError( ( "Fail to create wake up socket" ) );
        ret = NETWORKING_WSLAY_RESULT_FAIL_CREATE_SOCKET;
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        memset( &pWebsocketCtx->socketWakeUpAddr, 0, sizeof( struct sockaddr_in ) );
        pWebsocketCtx->socketWakeUpAddr.sin_family = AF_INET;
        pWebsocketCtx->socketWakeUpAddr.sin_addr.s_addr = htonl( IPADDR_ANY );
        pWebsocketCtx->socketWakeUpAddr.sin_port = 0;

        if( bind( pWebsocketCtx->socketWakeUp, ( const struct sockaddr * )&pWebsocketCtx->socketWakeUpAddr, sizeof( pWebsocketCtx->socketWakeUpAddr ) ) < 0 )
        {
            LogError( ( "Fail to bind wake up socket" ) );
            closesocket( pWebsocketCtx->socketWakeUp );
            ret = NETWORKING_WSLAY_RESULT_FAIL_BIND_SOCKET;
        }
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        setsockopt( pWebsocketCtx->socketWakeUp, SOL_SOCKET, SO_RCVTIMEO, &socketTimeoutMs, sizeof( socketTimeoutMs ) );
        setsockopt( pWebsocketCtx->socketWakeUp, SOL_SOCKET, SO_SNDTIMEO, &socketTimeoutMs, sizeof( socketTimeoutMs ) );

        sockAddress = ( struct sockaddr * ) &pWebsocketCtx->socketWakeUpAddr;
        addressLength = sizeof( struct sockaddr_in );
        if( getsockname( pWebsocketCtx->socketWakeUp, sockAddress, &addressLength ) < 0 )
        {
            LogError( ( "getsockname() failed with errno: %s", strerror( errno ) ) );
            close( pWebsocketCtx->socketWakeUp );
        }
        else
        {
            memcpy( &pWebsocketCtx->socketWakeUpAddr.sin_addr, LwIP_GetIP( 0 ), sizeof( struct in_addr ) );
            LogDebug( ( "Creating wake up socket at IP/port 0x%x/%u", pWebsocketCtx->socketWakeUpAddr.sin_addr.s_addr, ntohs( pWebsocketCtx->socketWakeUpAddr.sin_port ) ) );
        }
    }

    return ret;
}

static WebsocketResult_t ReadWebsocketMessage( NetworkingWslayContext_t * pWebsocketCtx )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    int retWslay;

    if( wslay_event_get_read_enabled( pWebsocketCtx->wslayContext ) == 1 )
    {
        retWslay = wslay_event_recv( pWebsocketCtx->wslayContext );
        if( retWslay != 0 )
        {
            LogError( ( "wslay_event_recv returns error 0x%X", retWslay ) );
            ret = NETWORKING_WSLAY_RESULT_FAIL_RECV;
        }
    }

    return ret;
}

static WebsocketResult_t SendWebsocketMessage( NetworkingWslayContext_t * pWebsocketCtx,
                                               struct wslay_event_msg * pArg )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    int retWslay;
    #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
    size_t prev = 0, mid = 0, last = 0;
    #endif /* if LIBRARY_LOG_LEVEL >= LOG_VERBOSE */

    if( wslay_event_get_write_enabled( pWebsocketCtx->wslayContext ) == 1 )
    {
        // send the message out immediately.
        #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
        /* Get the queued message count before sending message */
        prev = wslay_event_get_queued_msg_count( pWebsocketCtx->wslayContext );
        #endif /* if LIBRARY_LOG_LEVEL >= LOG_VERBOSE */

        retWslay = wslay_event_queue_msg( pWebsocketCtx->wslayContext, pArg );
        if( retWslay != 0 )
        {
            LogError( ( "Fail to enqueue new message." ) );
            ret = NETWORKING_WSLAY_RESULT_FAIL_QUEUE;
        }

        if( ret == NETWORKING_WSLAY_RESULT_OK )
        {
            #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
            /* Get the queued message count between queue and send message. */
            mid = wslay_event_get_queued_msg_count( pWebsocketCtx->wslayContext );
            #endif /* if LIBRARY_LOG_LEVEL >= LOG_VERBOSE */

            retWslay = wslay_event_send( pWebsocketCtx->wslayContext );

            if( retWslay != 0 )
            {
                LogInfo( ( "Fail to send this message at this moment, retWslay: %d.", retWslay ) );
                ret = NETWORKING_WSLAY_RESULT_FAIL_SEND;
            }
        }

        if( ret == NETWORKING_WSLAY_RESULT_OK )
        {
            #if LIBRARY_LOG_LEVEL >= LOG_VERBOSE
            /* Get the queued message count after sending message. */
            last = wslay_event_get_queued_msg_count( pWebsocketCtx->wslayContext );
            #endif /* if LIBRARY_LOG_LEVEL >= LOG_VERBOSE */

            LogVerbose( ( "Monitor wslay send queue (%u, %u, %u)", prev, mid, last ) );
        }
    }
    else
    {
        LogError( ( "Get write enable fail." ) );
        ret = NETWORKING_WSLAY_RESULT_FAIL_WRITE_ENABLE;
    }

    return ret;
}

static WebsocketResult_t SendWebsocketText( NetworkingWslayContext_t * pWebsocketCtx,
                                            uint8_t * pMessage,
                                            size_t messageLength )
{
    struct wslay_event_msg arg;

    memset( &arg, 0, sizeof( struct wslay_event_msg ) );
    arg.opcode = WSLAY_TEXT_FRAME;
    arg.msg = pMessage;
    arg.msg_length = messageLength;

    return SendWebsocketMessage( pWebsocketCtx, &arg );
}

static WebsocketResult_t SendWebsocketPing( NetworkingWslayContext_t * pWebsocketCtx )
{
    struct wslay_event_msg arg;

    memset( &arg, 0, sizeof( struct wslay_event_msg ) );
    arg.opcode = WSLAY_PING;
    arg.msg_length = 0;
    LogInfo( ( "wss ping ==>" ) );
    return SendWebsocketMessage( pWebsocketCtx, &arg );
}

static void ClearWakeUpSocketEvents( NetworkingWslayContext_t * pWebsocketCtx )
{
    char tempBuffer[ 32 ];
    struct sockaddr addr;
    socklen_t addrLength = sizeof( addr );
    int recvLength;

    while( ( recvLength = recvfrom( pWebsocketCtx->socketWakeUp, tempBuffer, sizeof( tempBuffer ), 0,
                                    ( struct sockaddr * ) &addr, ( socklen_t * )&addrLength ) ) > 0 )
    {
        LogDebug( ( "Clear %d byte on wake up socket", recvLength ) );
    }
}

static void TriggerWakeUpSocket( NetworkingWslayContext_t * pWebsocketCtx )
{
    char ch = 'a';
    int writtenLength;

    writtenLength = sendto( pWebsocketCtx->socketWakeUp, &ch, 1, 0,
                            ( struct sockaddr * ) &pWebsocketCtx->socketWakeUpAddr, sizeof( pWebsocketCtx->socketWakeUpAddr ) );
    if( writtenLength < 0 )
    {
        LogError( ( "Fail to trigger wake up socket, error=%s.", strerror( errno ) ) );
    }
}

WebsocketResult_t Websocket_Init( NetworkingWslayContext_t * pWebsocketCtx,
                                  WebsocketMessageCallback_t rxCallback,
                                  void * pRxCallbackContext )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    static uint8_t first = 0U;

    if( pWebsocketCtx == NULL )
    {
        ret = NETWORKING_WSLAY_RESULT_BAD_PARAMETER;
    }

    if( ( ret == NETWORKING_WSLAY_RESULT_OK ) && !first )
    {
        memset( &pWebsocketCtx->xTransportInterface, 0, sizeof( TransportInterface_t ) );
        memset( &pWebsocketCtx->xTlsNetworkContext, 0, sizeof( TlsNetworkContext_t ) );
        memset( &pWebsocketCtx->xTlsTransportParams, 0, sizeof( TlsTransportParams_t ) );

        /* Set transport interface. */
        pWebsocketCtx->xTransportInterface.pNetworkContext = ( NetworkContext_t * ) &pWebsocketCtx->xTlsNetworkContext;
        pWebsocketCtx->xTransportInterface.send = SendTlsPacket;
        pWebsocketCtx->xTransportInterface.recv = RecvTlsPacket;

        /* Set the pParams member of the network context with desired transport. */
        pWebsocketCtx->xTlsNetworkContext.pParams = &pWebsocketCtx->xTlsTransportParams;

        pWebsocketCtx->websocketRxCallback = rxCallback;
        pWebsocketCtx->pWebsocketRxCallbackContext = pRxCallbackContext;
    }

    if( ( ret == NETWORKING_WSLAY_RESULT_OK ) && !first )
    {
        first = 1U;
    }

    return ret;
}

WebsocketResult_t Websocket_Connect( NetworkingWslayContext_t * pWebsocketCtx,
                                     const AwsCredentials_t * pAwsCredentials,
                                     WebsocketServerInfo_t * pServerInfo )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    NetworkCredentials_t credentials;
    NetworkingUtilsResult_t retUtils;
    char * pHost, * pPath;
    size_t hostLength, pathLength;
    char host[ NETWORKING_WSLAY_HOST_NAME_MAX_LENGTH ];
    HTTPStatus_t xHttpStatus = HTTPSuccess;
    HTTPRequestHeaders_t xRequestHeaders = { 0 };
    HTTPRequestInfo_t xRequestInfo = { 0 };
    size_t queryParamsStringLength = NETWORKING_META_BUFFER_LENGTH - 2; /* Reserve first 2 bytes for "/?" */
    char clientKey[ NETWORKING_WSLAY_CLIENT_KEY_LENGTH + 1 ];
    HTTPResponse_t corehttpResponse;
    NetworkingWslayConnectResponseContext_t connectResponseContext;
    HTTPClient_ResponseHeaderParsingCallback_t headerParsingCallback = {
        .onHeaderCallback = ParsingConnectResponseHeader,
        .pContext = &connectResponseContext
    };
    TlsTransportStatus_t xNetworkStatus;
    uint8_t isTlsConnectionEstablished = 0U;

    if( ( pServerInfo == NULL ) || ( pServerInfo->pUrl == NULL ) )
    {
        ret = NETWORKING_WSLAY_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        do
        {
            /* Connection check before starting. Will skip following process if connection is already there.
             * This might happen when fail happening at join storage session request for any reason. */
            if( pWebsocketCtx->connectionEstablished != 0U )
            {
                LogDebug( ( "Websocket connection is already there, skip the connect process." ) );
                break;
            }

            /* Get host pointer & length */
            retUtils = NetworkingUtils_GetUrlHost( pServerInfo->pUrl, pServerInfo->urlLength, &pHost, &hostLength );

            if( retUtils == NETWORKING_UTILS_RESULT_OK )
            {
                if( hostLength < NETWORKING_WSLAY_HOST_NAME_MAX_LENGTH )
                {
                    memcpy( host, pHost, hostLength );
                    host[ hostLength ] = '\0';
                }
                else
                {
                    LogError( ( "uriHost buffer is not large enough to fit the host, hostLength = %u!", hostLength ) );
                    ret = NETWORKING_WSLAY_RESULT_NO_HOST_IN_URL;
                    break;
                }
            }
            else
            {
                LogError( ( "Fail to find valid host name from URL: %.*s", ( int ) pServerInfo->urlLength, pServerInfo->pUrl ) );
                ret = NETWORKING_WSLAY_RESULT_NO_HOST_IN_URL;
                break;
            }

            retUtils = NetworkingUtils_GetPathFromUrl( pServerInfo->pUrl, pServerInfo->urlLength, &pPath, &pathLength );
            if( retUtils != NETWORKING_UTILS_RESULT_OK )
            {
                LogError( ( "Fail to find valid path from URL: %.*s", ( int ) pServerInfo->urlLength, pServerInfo->pUrl ) );
                ret = NETWORKING_WSLAY_RESULT_NO_PATH_IN_URL;
                break;
            }

            ret = GenerateWebSocketClientKey( clientKey, NETWORKING_WSLAY_CLIENT_KEY_LENGTH + 1 );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to generate websocket client key, ret: %d", ret ) );
                break;
            }

            /* Try connect with websocket server. */
            {
                memset( &credentials, 0, sizeof( NetworkCredentials_t ) );
                credentials.pRootCa = pAwsCredentials->pRootCa;
                credentials.rootCaSize = pAwsCredentials->rootCaSize;

                if( pAwsCredentials->iotThingCertSize > 0 )
                {
                    credentials.pClientCert = pAwsCredentials->pIotThingCert;
                    credentials.clientCertSize = pAwsCredentials->iotThingCertSize;
                    credentials.pPrivateKey = pAwsCredentials->pIotThingPrivateKey;
                    credentials.privateKeySize = pAwsCredentials->iotThingPrivateKeySize;
                }

                LogDebug( ( "Establishing a TLS session with %s:443.",
                            host ) );

                /* Attempt to create a server-authenticated TLS connection. */
                xNetworkStatus = TLS_FreeRTOS_Connect( &pWebsocketCtx->xTlsNetworkContext,
                                                       host,
                                                       443,
                                                       &credentials,
                                                       NETWORKING_WSLAY_SEND_TIMEOUT_MS,
                                                       NETWORKING_WSLAY_RECV_TIMEOUT_MS,
                                                       0 ); /* Flag 0 - Blocking call */

                if( xNetworkStatus != TLS_TRANSPORT_SUCCESS )
                {
                    LogError( ( "Fail to connect with server with return %d", xNetworkStatus ) );
                    ret = NETWORKING_WSLAY_RESULT_FAIL_CONNECT;
                    break;
                }
                else
                {
                    isTlsConnectionEstablished = 1U;
                }
            }

            /* Follow https://docs.aws.amazon.com/IAM/latest/UserGuide/create-signed-request.html to create query parameters. */
            ret = GenerateQueryParameters( pWebsocketCtx,
                                           pAwsCredentials,
                                           pServerInfo->pUrl,
                                           pServerInfo->urlLength,
                                           pHost,
                                           hostLength,
                                           pPath,
                                           pathLength,
                                           pWebsocketCtx->metaBuffer,
                                           &queryParamsStringLength );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to get query parameters, ret: %d", ret ) );
                break;
            }

            /* Store query parameters into path buffer. */
            pWebsocketCtx->websocketTxBuffer[ 0 ] = '/';
            pWebsocketCtx->websocketTxBuffer[ 1 ] = '?';
            memcpy( &pWebsocketCtx->websocketTxBuffer[ 2 ], pWebsocketCtx->metaBuffer, queryParamsStringLength );

            pPath = pWebsocketCtx->websocketTxBuffer;
            pathLength = 2U + queryParamsStringLength;

            /* Prepare HTTP request for websocket connection */
            {
                /* Initialize Request header buffer. */
                xRequestHeaders.pBuffer = ( uint8_t * ) pWebsocketCtx->websocketTxBuffer + pathLength;
                xRequestHeaders.bufferLen = NETWORKING_WEBSOCKET_BUFFER_LENGTH - pathLength;

                /* Set HTTP request parameters to get temporary AWS IoT credentials. */
                xRequestInfo.pMethod = HTTP_METHOD_GET;
                xRequestInfo.methodLen = sizeof( HTTP_METHOD_GET ) - 1;
                xRequestInfo.pPath = pPath;
                xRequestInfo.pathLen = pathLength;
                xRequestInfo.pHost = pHost;
                xRequestInfo.hostLen = hostLength;
                xRequestInfo.reqFlags = HTTP_REQUEST_NO_USER_AGENT_FLAG;
                /* Note that host would be added to the header field by HTTPClient_InitializeRequestHeaders. */

                /* Initialize request headers. */
                xHttpStatus = HTTPClient_InitializeRequestHeaders( &xRequestHeaders, &xRequestInfo );

                if( xHttpStatus != HTTPSuccess )
                {
                    LogError( ( "Failed to initialize request headers: Error=%s.",
                                HTTPClient_strerror( xHttpStatus ) ) );
                    ret = NETWORKING_WSLAY_RESULT_FAIL_HTTP_INIT_REQUEST_HEADER;
                    break;
                }
            }

            ret = AddHeader( &xRequestHeaders,
                             "Pragma",
                             strlen( "Pragma" ),
                             "no-cache",
                             strlen( "no-cache" ) );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to append Pragma to HTTP header, ret: %d", ret ) );
                break;
            }

            ret = AddHeader( &xRequestHeaders,
                             "Cache-Control",
                             strlen( "Cache-Control" ),
                             "no-cache",
                             strlen( "no-cache" ) );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to append Cache-Control to HTTP header, ret: %d", ret ) );
                break;
            }

            ret = AddHeader( &xRequestHeaders,
                             "upgrade",
                             strlen( "upgrade" ),
                             "WebSocket",
                             strlen( "WebSocket" ) );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to append upgrade to HTTP header, ret: %d", ret ) );
                break;
            }

            ret = AddHeader( &xRequestHeaders,
                             "connection",
                             strlen( "connection" ),
                             "Upgrade",
                             strlen( "Upgrade" ) );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to append connection to HTTP header, ret: %d", ret ) );
                break;
            }

            ret = AddHeader( &xRequestHeaders,
                             "Sec-WebSocket-Key",
                             strlen( "Sec-WebSocket-Key" ),
                             clientKey,
                             NETWORKING_WSLAY_CLIENT_KEY_LENGTH );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to append Sec-WebSocket-Key to HTTP header, ret: %d", ret ) );
                break;
            }

            ret = AddHeader( &xRequestHeaders,
                             "Sec-WebSocket-Protocol",
                             strlen( "Sec-WebSocket-Protocol" ),
                             "wss",
                             strlen( "wss" ) );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to append Sec-WebSocket-Protocol to HTTP header, ret: %d", ret ) );
                break;
            }

            ret = AddHeader( &xRequestHeaders,
                             "Sec-WebSocket-Version",
                             strlen( "Sec-WebSocket-Version" ),
                             "13",
                             strlen( "13" ) );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to append Sec-WebSocket-Version to HTTP header, ret: %d", ret ) );
                break;
            }

            /* Send HTTP request to websocket server and wait for resopnse. */
            {
                memset( &corehttpResponse, 0, sizeof( HTTPResponse_t ) );
                corehttpResponse.pBuffer = ( uint8_t * ) pWebsocketCtx->websocketRxBuffer;
                corehttpResponse.bufferLen = NETWORKING_WEBSOCKET_BUFFER_LENGTH;
                corehttpResponse.pHeaderParsingCallback = &headerParsingCallback;

                memset( &connectResponseContext, 0, sizeof( NetworkingWslayConnectResponseContext_t ) );
                connectResponseContext.pClientKey = clientKey;
                connectResponseContext.clientKeyLength = NETWORKING_WSLAY_CLIENT_KEY_LENGTH;

                LogDebug( ( "Sending HTTP header: %.*s", ( int ) xRequestHeaders.headersLen, xRequestHeaders.pBuffer ) );

                /* Send the request to AWS IoT Credentials Provider to obtain temporary credentials
                 * so that the demo application can access configured S3 bucket thereafter. */
                xHttpStatus = HTTPClient_Send( &pWebsocketCtx->xTransportInterface,
                                               &xRequestHeaders,
                                               NULL,
                                               0U,
                                               &corehttpResponse,
                                               HTTP_SEND_DISABLE_CONTENT_LENGTH_FLAG );

                if( xHttpStatus != HTTPSuccess )
                {
                    LogError( ( "Failed to send Websocket connect request to %.*s for obtaining temporary credentials: Error=%s.",
                                ( int ) hostLength, pHost,
                                HTTPClient_strerror( xHttpStatus ) ) );
                    ret = NETWORKING_WSLAY_RESULT_FAIL_HTTP_SEND;
                    break;
                }
                else if( corehttpResponse.statusCode != 101 )
                {
                    LogError( ( "Websocket Connect Failed - Status Code: %u (Expected: 101), Response: %.*s",
                                corehttpResponse.statusCode,
                                ( int ) corehttpResponse.bodyLen,
                                corehttpResponse.pBody ) );
                    ret = NETWORKING_COREHTTP_RESULT_FAIL_HTTP_SEND;
                    break;
                }
                else
                {
                    LogDebug( ( "Receiving Websocket headers(%d): %.*s", corehttpResponse.headersLen, ( int ) corehttpResponse.headersLen, corehttpResponse.pHeaders ) );
                }
            }

            /* Check verified result in context. */
            {
                if( ( ( connectResponseContext.headersParsed & NETWORKING_WSLAY_HTTP_HEADER_CONNECTION ) == 0 ) ||
                    ( ( connectResponseContext.headersParsed & NETWORKING_WSLAY_HTTP_HEADER_UPGRADE ) == 0 ) ||
                    ( ( connectResponseContext.headersParsed & NETWORKING_WSLAY_HTTP_HEADER_WEBSOCKET_ACCEPT ) == 0 ) )
                {
                    LogError( ( "No valid response received, headersParsed=0x%x", connectResponseContext.headersParsed ) );
                    ret = NETWORKING_WSLAY_RESULT_FAIL_HTTP_PARSE_RESPONSE;
                    break;
                }
                else
                {
                    LogInfo( ( "Successfully connect with WSS endpoint %.*s.",
                               ( int ) pServerInfo->urlLength, pServerInfo->pUrl ) );
                }
            }

            /* Initialize wslay context. */
            ret = InitializeWslayContext( pWebsocketCtx );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to initialize wslay context, ret: %d", ret ) );
                break;
            }

            /* Initialize wake up socket for signal function. */
            ret = InitializeWakeUpSocket( pWebsocketCtx );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to initialize wake up socket handler, ret: %d", ret ) );
                break;
            }

            xNetworkStatus = TLS_FreeRTOS_ConfigureTimeout( &pWebsocketCtx->xTlsNetworkContext, 0U, 0U );
            if( xNetworkStatus != TLS_TRANSPORT_SUCCESS )
            {
                LogError( ( "Failed to configure TLS timeout: Status=%d", xNetworkStatus ) );
                ret = NETWORKING_WSLAY_RESULT_FAIL_CONNECT;
                break;
            }

            pWebsocketCtx->connectionEstablished = 1U;
            pWebsocketCtx->connectionCloseRequested = 0U;
        } while( pdFALSE );
    }

    if( ( ret != NETWORKING_WSLAY_RESULT_OK ) &&
        ( isTlsConnectionEstablished != 0U ) )
    {
        ( void ) TLS_FreeRTOS_Disconnect( &pWebsocketCtx->xTlsNetworkContext );
    }

    return ret;
}

WebsocketResult_t Websocket_Disconnect( NetworkingWslayContext_t * pWebsocketCtx )
{
    WebsocketResult_t ret = WEBSOCKET_RESULT_OK;
    TlsTransportStatus_t tlsStatus = TLS_TRANSPORT_SUCCESS;

    if( pWebsocketCtx == NULL )
    {
        ret = WEBSOCKET_RESULT_BAD_PARAMETER;
    }

    if( ret == WEBSOCKET_RESULT_OK )
    {
        if( pWebsocketCtx->wslayContext != NULL )
        {
            pWebsocketCtx->connectionCloseRequested = 0U;

            /* Shutdown WebSocket read operations */
            wslay_event_shutdown_read( pWebsocketCtx->wslayContext );

            /* Shutdown WebSocket write operations */
            wslay_event_shutdown_write( pWebsocketCtx->wslayContext );

            /* Free the wslay context */
            wslay_event_context_free( pWebsocketCtx->wslayContext );
            pWebsocketCtx->wslayContext = NULL;
        }
    }

    if( ret == WEBSOCKET_RESULT_OK )
    {
        /* Close wake-up socket if it's open */
        if( pWebsocketCtx->socketWakeUp != -1 )
        {
            if( close( pWebsocketCtx->socketWakeUp ) == -1 )
            {
                LogError( ( "Failed to close wake-up socket: errno=%d", errno ) );
            }

            pWebsocketCtx->socketWakeUp = -1;
        }
    }

    if( ret == WEBSOCKET_RESULT_OK )
    {
        /* Close TLS connection if it exists */
        if( pWebsocketCtx->connectionEstablished != 0U )
        {
            tlsStatus = TLS_FreeRTOS_Disconnect( &pWebsocketCtx->xTlsNetworkContext );
            if( tlsStatus != TLS_TRANSPORT_SUCCESS )
            {
                LogError( ( "Failed to disconnect TLS connection: Status=%d", tlsStatus ) );
                ret = WEBSOCKET_RESULT_FAIL;
            }

            pWebsocketCtx->connectionEstablished = 0U;
        }
    }

    /* Log the final status */
    if( ret == WEBSOCKET_RESULT_OK )
    {
        LogInfo( ( "WebSocket connection successfully closed" ) );
    }
    else
    {
        LogWarn( ( "WebSocket disconnect completed with errors" ) );
    }

    return ret;
}

WebsocketResult_t Websocket_Send( NetworkingWslayContext_t * pWebsocketCtx,
                                  char * pMessage,
                                  size_t messageLength )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;

    if( pMessage == NULL )
    {
        ret = NETWORKING_WSLAY_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        ret = SendWebsocketText( pWebsocketCtx, ( uint8_t * ) pMessage, messageLength );
    }

    return ret;
}

WebsocketResult_t Websocket_Recv( NetworkingWslayContext_t * pWebsocketCtx )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;
    int fd = 0, maxFd = 0;
    fd_set rfds;
    struct timeval tv;
    int retSelect;
    TickType_t currentTick;

    fd = TLS_FreeRTOS_GetSocketFd( &pWebsocketCtx->xTlsNetworkContext );

    FD_ZERO( &rfds );
    FD_SET( fd, &rfds );
    FD_SET( pWebsocketCtx->socketWakeUp, &rfds );
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    maxFd = fd > pWebsocketCtx->socketWakeUp ? fd : pWebsocketCtx->socketWakeUp;

    retSelect = select( maxFd + 1, &rfds, NULL, NULL, &tv );
    if( retSelect < 0 )
    {
        LogError( ( "select return error value %d", retSelect ) );
        ret = NETWORKING_WSLAY_RESULT_FAIL_SELECT;
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        if( FD_ISSET( fd, &rfds ) )
        {
            /* Have something to read. */
            ret = ReadWebsocketMessage( pWebsocketCtx );
            if( ret != NETWORKING_WSLAY_RESULT_OK )
            {
                LogError( ( "Fail to read websocket message, ret: %d", ret ) );
                pWebsocketCtx->connectionEstablished = 0U;
            }
        }
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        /* If connection is asked to close by server, set the return code to non-ok
           to start disconnection flow. */
        if( pWebsocketCtx->connectionCloseRequested != 0U )
        {
            ret = NETWORKING_WSLAY_RESULT_CONNETION_CLOSED;
        }
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        if( FD_ISSET( pWebsocketCtx->socketWakeUp, &rfds ) )
        {
            ClearWakeUpSocketEvents( pWebsocketCtx );
        }
    }

    if( ret == NETWORKING_WSLAY_RESULT_OK )
    {
        /* Handle ping interval. */
        currentTick = xTaskGetTickCount();
        if( currentTick - pWebsocketCtx->lastPingTick >= NETWORKING_WSLAY_PING_PONG_INTERVAL_TICKS )
        {
            ret = SendWebsocketPing( pWebsocketCtx );
            pWebsocketCtx->lastPingTick = currentTick;
        }
    }

    if( ret != NETWORKING_WSLAY_RESULT_OK )
    {
        ( void ) Websocket_Disconnect( pWebsocketCtx );
    }

    return ret;
}

WebsocketResult_t Websocket_Signal( NetworkingWslayContext_t * pWebsocketCtx )
{
    NetworkingWslayResult_t ret = NETWORKING_WSLAY_RESULT_OK;

    TriggerWakeUpSocket( pWebsocketCtx );

    return ret;
}
