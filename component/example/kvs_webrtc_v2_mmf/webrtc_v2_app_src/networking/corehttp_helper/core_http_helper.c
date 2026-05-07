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

#include <time.h>

#include "FreeRTOS_POSIX/time.h"
#include "logging.h"
#include "networking.h"
#include "core_http_helper.h"
#include "core_http_client.h"

#include "mbedtls/ssl.h"
#include "mbedtls/sha256.h"
#include "mbedtls/md.h"
#include "mbedtls/version.h"

#define NETWORKING_COREHTTP_SEND_TIMEOUT_MS ( 10000 )
#define NETWORKING_COREHTTP_RECV_TIMEOUT_MS ( 10000 )
#define NETWORKING_COREHTTP_USER_AGENT_NAME_MAX_LENGTH ( 128 )
#define NETWORKING_COREHTTP_STRING_SCHEMA_DELIMITER "://"
#define NETWORKING_COREHTTP_STRING_HOST "host"
#define NETWORKING_COREHTTP_STRING_USER_AGENT "user-agent"
#define NETWORKING_COREHTTP_STRING_AUTHORIZATION "Authorization"
#define NETWORKING_COREHTTP_STRING_CONTENT_TYPE "content-type"
#define NETWORKING_COREHTTP_STRING_CONTENT_TYPE_VALUE "application/json"
#define NETWORKING_COREHTTP_STRING_CONTENT_LENGTH "content-length"
#define NETWORKING_COREHTTP_STRING_IOT_THINGNAME "x-amzn-iot-thingname"

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

static uint32_t GetCurrentTimeMilisec( void )
{
    uint32_t timeSeconds, timeMilliseconds;

    timeSeconds = ( uint32_t ) NetworkingUtils_GetCurrentTimeSec( NULL );
    timeMilliseconds = 1000 * timeSeconds;

    return timeMilliseconds;
}

HttpResult_t Http_Init( NetworkingCorehttpContext_t * pHttpCtx )
{
    NetworkingCorehttpResult_t ret = NETWORKING_COREHTTP_RESULT_OK;
    static uint8_t first = 0U;

    if( pHttpCtx == NULL )
    {
        ret = NETWORKING_COREHTTP_RESULT_BAD_PARAMETER;
    }

    if( ( ret == NETWORKING_COREHTTP_RESULT_OK ) && !first )
    {
        memset( &pHttpCtx->xTransportInterface, 0, sizeof( TransportInterface_t ) );
        memset( &pHttpCtx->xTlsNetworkContext, 0, sizeof( TlsNetworkContext_t ) );
        memset( &pHttpCtx->xTlsTransportParams, 0, sizeof( TlsTransportParams_t ) );

        /* Set transport interface. */
        pHttpCtx->xTransportInterface.pNetworkContext = ( NetworkContext_t * ) &pHttpCtx->xTlsNetworkContext;
        pHttpCtx->xTransportInterface.send = SendTlsPacket;
        pHttpCtx->xTransportInterface.recv = RecvTlsPacket;

        /* Set the pParams member of the network context with desired transport. */
        pHttpCtx->xTlsNetworkContext.pParams = &pHttpCtx->xTlsTransportParams;
    }

    if( ( ret == NETWORKING_COREHTTP_RESULT_OK ) && !first )
    {
        first = 1U;
    }

    return ret;
}

HttpResult_t Http_Send( NetworkingCorehttpContext_t * pHttpCtx,
                        HttpRequest_t * pRequest,
                        const AwsCredentials_t * pAwsCredentials,
                        size_t timeoutMs,
                        HttpResponse_t * pResponse )
{
    NetworkingCorehttpResult_t ret = NETWORKING_COREHTTP_RESULT_OK;
    NetworkingUtilsResult_t retUtils;
    HTTPStatus_t xHttpStatus = HTTPSuccess;
    HTTPRequestHeaders_t xRequestHeaders = { 0 };
    HTTPRequestInfo_t xRequestInfo = { 0 };
    char dateBuffer[ NETWORKING_UTILS_TIME_BUFFER_LENGTH ];
    char * pPath;
    size_t pathLength;
    char * pHost;
    size_t hostLength;
    char contentLengthBuffer[ 11 ]; /* It needs 10 bytes for 32 bit integer, +1 for NULL terminator. */
    size_t contentLengthLength;
    /* Pointer to start of key-value pair buffer in request buffer. This is
     * used for Sigv4 signing */
    char * pcHeaderStart;
    size_t xHeadersLen;
    NetworkingUtilsCanonicalRequest_t canonicalRequest;
    HTTPResponse_t corehttpResponse;
    NetworkCredentials_t credentials;
    char * pSig;
    size_t sigLength;
    TlsTransportStatus_t xNetworkStatus;
    SigV4Credentials_t sigv4Credential;

    if( ( pRequest == NULL ) || ( pResponse == NULL ) )
    {
        ret = NETWORKING_COREHTTP_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_COREHTTP_RESULT_OK )
    {
        /* Get host pointer & length */
        retUtils = NetworkingUtils_GetUrlHost( pRequest->pUrl, pRequest->urlLength, &pHost, &hostLength );

        if( retUtils == NETWORKING_UTILS_RESULT_OK )
        {
            memcpy( pHttpCtx->hostName, pHost, hostLength );
            pHttpCtx->hostName[ hostLength ] = '\0';
        }
        else
        {
            LogError( ( "Fail to find valid host name from URL: %.*s", ( int ) pRequest->urlLength, pRequest->pUrl ) );
            ret = NETWORKING_COREHTTP_RESULT_NO_HOST_IN_URL;
        }
    }

    if( ret == NETWORKING_COREHTTP_RESULT_OK )
    {
        ret = NetworkingUtils_GetPathFromUrl( pRequest->pUrl, pRequest->urlLength, &pPath, &pathLength );

        if( retUtils != NETWORKING_UTILS_RESULT_OK )
        {
            LogError( ( "Fail to find valid path from URL: %.*s", ( int ) pRequest->urlLength, pRequest->pUrl ) );
            ret = NETWORKING_COREHTTP_RESULT_NO_PATH_IN_URL;
        }
    }

    if( ret == NETWORKING_COREHTTP_RESULT_OK )
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
                    pHttpCtx->hostName ) );

        /* Attempt to create a server-authenticated TLS connection. */
        xNetworkStatus = TLS_FreeRTOS_Connect( &pHttpCtx->xTlsNetworkContext,
                                               pHttpCtx->hostName,
                                               443,
                                               &credentials,
                                               NETWORKING_COREHTTP_SEND_TIMEOUT_MS,
                                               NETWORKING_COREHTTP_RECV_TIMEOUT_MS,
                                               0 ); /* Flag 0 - Blocking call */

        if( xNetworkStatus != TLS_TRANSPORT_SUCCESS )
        {
            LogError( ( "Fail to connect the host: %s:%u", pHttpCtx->hostName, 443U ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_CONNECT;
        }
    }

    if( ret == NETWORKING_COREHTTP_RESULT_OK )
    {
        /* Initialize Request header buffer. */
        xRequestHeaders.pBuffer = pHttpCtx->requestBuffer;
        xRequestHeaders.bufferLen = NETWORKING_COREHTTP_BUFFER_LENGTH;

        /* Set HTTP request parameters to get temporary AWS IoT credentials. */
        if( pRequest->isFetchingCredential != 0U )
        {
            xRequestInfo.pMethod = HTTP_METHOD_GET;
            xRequestInfo.methodLen = sizeof( HTTP_METHOD_GET ) - 1;
        }
        else
        {
            xRequestInfo.pMethod = HTTP_METHOD_POST;
            xRequestInfo.methodLen = sizeof( HTTP_METHOD_POST ) - 1;
        }

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
            ret = NETWORKING_COREHTTP_RESULT_FAIL_HTTP_INIT_REQUEST_HEADER;
        }
    }

    if( ret == NETWORKING_COREHTTP_RESULT_OK )
    {
        xHttpStatus = HTTPClient_AddHeader( &xRequestHeaders,
                                            NETWORKING_COREHTTP_STRING_USER_AGENT,
                                            strlen( NETWORKING_COREHTTP_STRING_USER_AGENT ),
                                            pAwsCredentials->pUserAgent,
                                            pAwsCredentials->userAgentLength );

        if( xHttpStatus != HTTPSuccess )
        {
            LogError( ( "Failed to add user-agent header to request headers: Error=%s.",
                        HTTPClient_strerror( xHttpStatus ) ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_USER_AGENT;
        }
    }

    if( ret == NETWORKING_COREHTTP_RESULT_OK )
    {
        retUtils = NetworkingUtils_GetIso8601CurrentTime( dateBuffer, NETWORKING_UTILS_TIME_BUFFER_LENGTH );

        if( retUtils != NETWORKING_UTILS_RESULT_OK )
        {
            LogError( ( "Fail to get current ISO8601 date" ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_GET_DATE;
        }
    }

    /* While fetching credential, append IoT Thing Name and use HTTP GET. */
    if( ( ret == NETWORKING_COREHTTP_RESULT_OK ) &&
        ( pRequest->isFetchingCredential != 0U ) &&
        ( pAwsCredentials->iotThingNameLength > 0 ) )
    {

        xHttpStatus = HTTPClient_AddHeader( &xRequestHeaders,
                                            NETWORKING_COREHTTP_STRING_IOT_THINGNAME,
                                            strlen( NETWORKING_COREHTTP_STRING_IOT_THINGNAME ),
                                            pAwsCredentials->pIotThingName,
                                            pAwsCredentials->iotThingNameLength );

        if( xHttpStatus != HTTPSuccess )
        {
            LogError( ( "Failed to add x-amzn-iot-thingname header to request headers: Error=%s.",
                        HTTPClient_strerror( xHttpStatus ) ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_IOT_THING_NAME;
        }
    }

    /* While fetching credential, append IoT Thing Name and use HTTP GET. */
    if( ( ret == NETWORKING_COREHTTP_RESULT_OK ) && ( pAwsCredentials->sessionTokenLength > 0 ) )
    {
        xHttpStatus = HTTPClient_AddHeader( &xRequestHeaders,
                                            SIGV4_HTTP_X_AMZ_SECURITY_TOKEN_HEADER,
                                            strlen( SIGV4_HTTP_X_AMZ_SECURITY_TOKEN_HEADER ),
                                            pAwsCredentials->pSessionToken,
                                            pAwsCredentials->sessionTokenLength );

        if( xHttpStatus != HTTPSuccess )
        {
            LogError( ( "Failed to add x-amzn-iot-thingname header to request headers: Error=%s.",
                        HTTPClient_strerror( xHttpStatus ) ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_IOT_THING_NAME;
        }
    }

    if( ret == NETWORKING_COREHTTP_RESULT_OK )
    {
        xHttpStatus = HTTPClient_AddHeader( &xRequestHeaders,
                                            SIGV4_HTTP_X_AMZ_DATE_HEADER,
                                            strlen( SIGV4_HTTP_X_AMZ_DATE_HEADER ),
                                            dateBuffer,
                                            NETWORKING_UTILS_TIME_BUFFER_LENGTH - 1 );

        if( xHttpStatus != HTTPSuccess )
        {
            LogError( ( "Failed to add date header to request headers: Error=%s.",
                        HTTPClient_strerror( xHttpStatus ) ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_DATE;
        }
    }

    /* Sign the HTTP request. */
    /* While fetching credential, we don't need to generate authorization header and we don't have the key pair at this moment. */
    if( ( ret == NETWORKING_COREHTTP_RESULT_OK ) && ( pRequest->isFetchingCredential == 0U ) )
    {
        /* Find the start key-value pairs for sigv4 signing. */
        NetworkingUtils_GetHeaderStartLocFromHttpRequest( &xRequestHeaders, &pcHeaderStart, &xHeadersLen );

        memset( &canonicalRequest, 0, sizeof( canonicalRequest ) );
        canonicalRequest.verb = NETWORKING_UTILS_HTTP_VERB_POST;
        canonicalRequest.pPath = pPath;
        canonicalRequest.pathLength = pathLength;
        canonicalRequest.pCanonicalQueryString = NULL;
        canonicalRequest.canonicalQueryStringLength = 0U;
        canonicalRequest.pCanonicalHeaders = pcHeaderStart;
        canonicalRequest.canonicalHeadersLength = xHeadersLen;
        canonicalRequest.pPayload = pRequest->pBody;
        canonicalRequest.payloadLength = pRequest->bodyLength;

        sigv4Credential.pAccessKeyId = pAwsCredentials->pAccessKeyId;
        sigv4Credential.accessKeyIdLen = pAwsCredentials->accessKeyIdLength;
        sigv4Credential.pSecretAccessKey = pAwsCredentials->pSecretAccessKey;
        sigv4Credential.secretAccessKeyLen = pAwsCredentials->secretAccessKeyLength;

        pHttpCtx->sigv4AuthBufferLength = NETWORKING_COREHTTP_SIGV4_METADATA_BUFFER_LENGTH;
        retUtils = NetworkingUtils_GenrerateAuthorizationHeader( &canonicalRequest, &sigv4Credential,
                                                                 pAwsCredentials->pRegion, pAwsCredentials->regionLength, dateBuffer,
                                                                 pHttpCtx->sigv4AuthBuffer, &pHttpCtx->sigv4AuthBufferLength,
                                                                 &pSig, &sigLength );

        if( retUtils != NETWORKING_UTILS_RESULT_OK )
        {
            LogError( ( "Fail to generate authorization header, return=%d", retUtils ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_CONNECT;
        }
    }

    if( ( ret == NETWORKING_COREHTTP_RESULT_OK ) && ( pRequest->isFetchingCredential == 0U ) )
    {
        /* Add the authorization header to the HTTP request headers. */
        xHttpStatus = HTTPClient_AddHeader( &xRequestHeaders,
                                            NETWORKING_COREHTTP_STRING_AUTHORIZATION,
                                            strlen( NETWORKING_COREHTTP_STRING_AUTHORIZATION ),
                                            pHttpCtx->sigv4AuthBuffer,
                                            pHttpCtx->sigv4AuthBufferLength );

        if( xHttpStatus != HTTPSuccess )
        {
            LogError( ( "Failed to add Sigv4 auth header. Error=%s.",
                        HTTPClient_strerror( xHttpStatus ) ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_AUTH;
        }
    }

    if( ( ret == NETWORKING_COREHTTP_RESULT_OK ) && ( pRequest->isFetchingCredential == 0U ) )
    {
        contentLengthLength = snprintf( contentLengthBuffer, sizeof( contentLengthBuffer ), "%u", pRequest->bodyLength );
        xHttpStatus = HTTPClient_AddHeader( &xRequestHeaders,
                                            NETWORKING_COREHTTP_STRING_CONTENT_LENGTH,
                                            strlen( NETWORKING_COREHTTP_STRING_CONTENT_LENGTH ),
                                            contentLengthBuffer,
                                            contentLengthLength );

        if( xHttpStatus != HTTPSuccess )
        {
            LogError( ( "Failed to add content type header to request headers: Error=%s.",
                        HTTPClient_strerror( xHttpStatus ) ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_CONTENT_TYPE;
        }
    }

    if( ret == NETWORKING_COREHTTP_RESULT_OK )
    {
        xHttpStatus = HTTPClient_AddHeader( &xRequestHeaders,
                                            NETWORKING_COREHTTP_STRING_CONTENT_TYPE,
                                            strlen( NETWORKING_COREHTTP_STRING_CONTENT_TYPE ),
                                            NETWORKING_COREHTTP_STRING_CONTENT_TYPE_VALUE,
                                            strlen( NETWORKING_COREHTTP_STRING_CONTENT_TYPE_VALUE ) );

        if( xHttpStatus != HTTPSuccess )
        {
            LogError( ( "Failed to add content type header to request headers: Error=%s.",
                        HTTPClient_strerror( xHttpStatus ) ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_CONTENT_TYPE;
        }
    }

    if( ret == NETWORKING_COREHTTP_RESULT_OK )
    {
        memset( &corehttpResponse, 0, sizeof( HTTPResponse_t ) );
        corehttpResponse.pBuffer = ( uint8_t * ) pResponse->pBuffer;
        corehttpResponse.bufferLen = pResponse->bufferLength;
        corehttpResponse.pHeaderParsingCallback = NULL;
        corehttpResponse.getTime = GetCurrentTimeMilisec;

        LogDebug( ( "Sending HTTP header: %.*s", ( int ) xRequestHeaders.headersLen, xRequestHeaders.pBuffer ) );
        LogDebug( ( "Sending HTTP body: %.*s", ( int ) pRequest->bodyLength, pRequest->pBody ) );

        /* Send the request to AWS IoT Credentials Provider to obtain temporary credentials
         * so that the demo application can access configured S3 bucket thereafter. */
        xHttpStatus = HTTPClient_Send( &pHttpCtx->xTransportInterface,
                                       &xRequestHeaders,
                                       ( uint8_t * ) pRequest->pBody,
                                       pRequest->bodyLength,
                                       &corehttpResponse,
                                       HTTP_SEND_DISABLE_CONTENT_LENGTH_FLAG );

        if( xHttpStatus != HTTPSuccess )
        {
            LogError( ( "Failed to send HTTP POST request to %.*s for obtaining temporary credentials: Error=%s.",
                        ( int ) pRequest->urlLength, pRequest->pUrl,
                        HTTPClient_strerror( xHttpStatus ) ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_HTTP_SEND;
        }
        else if( corehttpResponse.statusCode != 200 )
        {
            LogError( ( "HTTP Request Failed - Status Code: %u (Expected: 200), Response: %.*s",
                        corehttpResponse.statusCode,
                        ( int ) corehttpResponse.bodyLen,
                        corehttpResponse.pBody ) );
            ret = NETWORKING_COREHTTP_RESULT_FAIL_HTTP_SEND;
        }
        else
        {
            LogDebug( ( "Receiving HTTP body(%d): %.*s", corehttpResponse.bodyLen, ( int ) corehttpResponse.bodyLen, corehttpResponse.pBody ) );

            /* Return the body part for signaling controller. */
            pResponse->bufferLength = corehttpResponse.bodyLen;
            pResponse->pBuffer = ( char * ) corehttpResponse.pBody;
        }
    }

    TLS_FreeRTOS_Disconnect( &pHttpCtx->xTlsNetworkContext );

    return ret;
}
