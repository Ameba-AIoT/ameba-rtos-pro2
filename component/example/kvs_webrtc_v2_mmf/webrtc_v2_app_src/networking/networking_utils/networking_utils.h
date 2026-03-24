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

#ifndef NETWORKING_UTILS_H
#define NETWORKING_UTILS_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "transport_mbedtls.h"
#include "transport_dtls_mbedtls.h"
#include "core_http_client.h"
#include "sigv4.h"

#define NETWORKING_UTILS_TIME_BUFFER_LENGTH ( 17 ) /* length of ISO8601 format (e.g. 20111008T070709Z) with NULL terminator */
#define NETWORKING_UTILS_KVS_SERVICE_NAME "kinesisvideo"

typedef enum NetworkingUtilsResult
{
    NETWORKING_UTILS_RESULT_OK = 0,
    NETWORKING_UTILS_RESULT_BAD_PARAMETER,
    NETWORKING_UTILS_RESULT_TIME_BUFFER_OUT_OF_MEMORY,
    NETWORKING_UTILS_RESULT_FAIL_CONNECT,
    NETWORKING_UTILS_RESULT_FAIL_SIGV4_GENAUTH,
    NETWORKING_UTILS_RESULT_SCHEMA_DELIMITER_NOT_FOUND,
    NETWORKING_UTILS_RESULT_INVALID_URL,
} NetworkingUtilsResult_t;

typedef enum NetworkingHttpVerb
{
    NETWORKING_UTILS_HTTP_VERB_NONE = 0,
    NETWORKING_UTILS_HTTP_VERB_GET,
    NETWORKING_UTILS_HTTP_VERB_POST,
    NETWORKING_UTILS_HTTP_VERB_WSS,
} NetworkingHttpVerb_t;

/* Refer to https://docs.aws.amazon.com/IAM/latest/UserGuide/create-signed-request.html
 * to create a struct that needed for generating authorzation header. */
typedef struct NetworkingUtilsCanonicalRequest
{
    NetworkingHttpVerb_t verb;
    char * pPath; // For canonical URI
    size_t pathLength;
    char * pCanonicalQueryString; // Canonical query string
    size_t canonicalQueryStringLength;
    char * pCanonicalHeaders; // Canonical headers
    size_t canonicalHeadersLength;
    char * pPayload; // Un-hashed payload
    size_t payloadLength;
} NetworkingUtilsCanonicalRequest_t;

NetworkingUtilsResult_t NetworkingUtils_GetUrlHost( char * pUrl,
                                                    size_t urlLength,
                                                    char ** ppStart,
                                                    size_t * pHostLength );
NetworkingUtilsResult_t NetworkingUtils_GetPathFromUrl( char * pUrl,
                                                        size_t urlLength,
                                                        char ** ppPath,
                                                        size_t * pPathLength );
NetworkingUtilsResult_t NetworkingUtils_GenrerateAuthorizationHeader( NetworkingUtilsCanonicalRequest_t * pCanonicalRequest,
                                                                      SigV4Credentials_t * pSigv4Credential,
                                                                      const char * pAwsRegion,
                                                                      size_t awsRegionLength,
                                                                      const char * pDate,
                                                                      char * pOutput,
                                                                      size_t * pOutputLength,
                                                                      char ** ppOutSignature,
                                                                      size_t * pOutSignatureLength );
void NetworkingUtils_GetHeaderStartLocFromHttpRequest( HTTPRequestHeaders_t * pxRequestHeaders,
                                                       char ** pcStartHeaderLoc,
                                                       size_t * pxHeadersDataLen );
NetworkingUtilsResult_t NetworkingUtils_GetIso8601CurrentTime( char * pDate,
                                                               size_t dateBufferLength );

uint64_t NetworkingUtils_GetCurrentTimeSec( void * pTick );
uint64_t NetworkingUtils_GetCurrentTimeUs( void * pTick );
uint64_t NetworkingUtils_GetTimeFromIso8601( const char * pDate,
                                             size_t dateLength );
uint64_t NetworkingUtils_GetNTPTimeFromUnixTimeUs( uint64_t timeUs );

#ifdef __cplusplus
}
#endif

#endif /* NETWORKING_UTILS_H */
