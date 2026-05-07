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
#include <string.h>

#include "FreeRTOS_POSIX/time.h"
#include "sntp/sntp.h" // SNTP series APIs
#include "logging.h"
#include "networking_utils.h"

#include "mbedtls/ssl.h"
#include "mbedtls/sha256.h"
#include "mbedtls/md.h"
#include "mbedtls/version.h"

#define NETWORKING_UTILS_STRING_SCHEMA_DELIMITER "://"
#define NETWORKING_ISO8601_TIME_STRING_LENGTH ( 20 )
/* length of ISO8601 format (e.g. 2024-12-31T03:27:52Z). */
#define NETWORKING_ISO8601_TIME_STRING_LENGTH ( 20 )

/*
   NETWORKING_NTP_OFFSET (2208988800ULL) represents the number of seconds between two important epochs:
   NTP Epoch: January 1, 1900, 00:00:00 UTC
   Unix Epoch: January 1, 1970, 00:00:00 UTC
   This offset is required because:
   NTP timestamps count seconds from January 1, 1900
   Unix timestamps (what we're converting from) count seconds from January 1, 1970
   The offset (2208988800) is exactly the number of seconds between these two dates
 */
#define NETWORKING_NTP_OFFSET    2208988800ULL

/*
   The scaling (NETWORKING_NTP_TIMESCALE = 2^32 = 4294967296) is used for handling fractional seconds in NTP's timestamp format. Here's why:
   NTP timestamp format consists of two 32-bit fields:
   1. First 32 bits: whole seconds since NTP epoch
   2. Second 32 bits: fractional second in fixed-point format
   The fractional part uses a fixed-point representation where:
   1. 2^32 (4294967296) represents 1 full second
   2. Any value from 0 to 2^32-1 represents a fraction of a second */
#define NETWORKING_NTP_TIMESCALE 4294967296ULL

static int32_t Sha256Init( void * hashContext );
static int32_t Sha256Update( void * hashContext,
                             const uint8_t * pInput,
                             size_t inputLen );
static int32_t Sha256Final( void * hashContext,
                            uint8_t * pOutput,
                            size_t outputLen );

/**
 *  @brief mbedTLS Hash Context passed to SigV4 cryptointerface for generating the hash digest.
 */
static mbedtls_sha256_context xHashContext = { 0 };

/**
 * @brief CryptoInterface provided to SigV4 library for generating the hash digest.
 */
static SigV4CryptoInterface_t cryptoInterface =
{
    .hashInit = Sha256Init,
    .hashUpdate = Sha256Update,
    .hashFinal = Sha256Final,
    .pHashContext = &xHashContext,
    .hashBlockLen = 64,
    .hashDigestLen = 32,
};

static SigV4Parameters_t sigv4Params =
{
    .pCredentials = NULL,
    .pDateIso8601 = NULL,
    .pRegion = NULL,
    .regionLen = 0,
    .pService = NETWORKING_UTILS_KVS_SERVICE_NAME,
    .serviceLen = strlen( NETWORKING_UTILS_KVS_SERVICE_NAME ),
    .pCryptoInterface = &cryptoInterface,
    .pHttpParameters = NULL
};

static int32_t Sha256Init( void * hashContext )
{
    mbedtls_sha256_init( ( mbedtls_sha256_context * ) hashContext );
    mbedtls_sha256_starts( hashContext, 0 );

    return 0;
}

static int32_t Sha256Update( void * hashContext,
                             const uint8_t * pInput,
                             size_t inputLen )
{
    mbedtls_sha256_update( hashContext, pInput, inputLen );

    return 0;
}

static int32_t Sha256Final( void * hashContext,
                            uint8_t * pOutput,
                            size_t outputLen )
{
    configASSERT( outputLen >= 32 );

    ( void ) outputLen;

    mbedtls_sha256_finish( hashContext, pOutput );

    return 0;
}

NetworkingUtilsResult_t NetworkingUtils_GetUrlHost( char * pUrl,
                                                    size_t urlLength,
                                                    char ** ppStart,
                                                    size_t * pHostLength )
{
    NetworkingUtilsResult_t ret = NETWORKING_UTILS_RESULT_OK;
    char * pStart = NULL, * pEnd = pUrl + urlLength, * pCurPtr;
    uint8_t foundEndMark = 0;

    if( ( pUrl == NULL ) || ( ppStart == NULL ) || ( pHostLength == NULL ) )
    {
        ret = NETWORKING_UTILS_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_UTILS_RESULT_OK )
    {
        // Start from the schema delimiter
        pStart = strstr( pUrl, NETWORKING_UTILS_STRING_SCHEMA_DELIMITER );
        if( pStart == NULL )
        {
            ret = NETWORKING_UTILS_RESULT_SCHEMA_DELIMITER_NOT_FOUND;
        }
    }

    if( ret == NETWORKING_UTILS_RESULT_OK )
    {
        // Advance the pStart past the delimiter
        pStart += strlen( NETWORKING_UTILS_STRING_SCHEMA_DELIMITER );

        if( pStart > pEnd )
        {
            ret = NETWORKING_UTILS_RESULT_INVALID_URL;
        }
    }

    if( ret == NETWORKING_UTILS_RESULT_OK )
    {
        // Find the delimiter which would indicate end of the host - either one of "/:?"
        pCurPtr = pStart;

        while( !foundEndMark && pCurPtr <= pEnd )
        {
            switch( *pCurPtr )
            {
                case '/':
                case ':':
                case '?':
                    foundEndMark = 1;
                    break;
                default:
                    pCurPtr++;
            }
        }
    }

    if( ret == NETWORKING_UTILS_RESULT_OK )
    {
        *ppStart = pStart;
        *pHostLength = pCurPtr - pStart;
    }

    return ret;
}

NetworkingUtilsResult_t NetworkingUtils_GetPathFromUrl( char * pUrl,
                                                        size_t urlLength,
                                                        char ** ppPath,
                                                        size_t * pPathLength )
{
    NetworkingUtilsResult_t ret = NETWORKING_UTILS_RESULT_OK;
    char * pHost, * pPathEnd;
    size_t hostLength;
    char * pStart;

    if( ( pUrl == NULL ) || ( ppPath == NULL ) )
    {
        ret = NETWORKING_UTILS_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_UTILS_RESULT_OK )
    {
        /* Get host pointer & length */
        ret = NetworkingUtils_GetUrlHost( pUrl, urlLength, &pHost, &hostLength );
    }

    if( ret == NETWORKING_UTILS_RESULT_OK )
    {
        /* Find '?' as end of path if any query parameters. */
        pStart = strchr( pHost + hostLength, '?' );

        if( pStart == NULL )
        {
            /* All the remaining part after the host belongs to the path. */
            pPathEnd = pUrl + urlLength;
            *ppPath = pHost + hostLength;
            *pPathLength = pPathEnd - *ppPath;
        }
        else
        {
            /* The part after the host until '?' belongs to the path. */
            pPathEnd = pStart;
            *ppPath = pHost + hostLength;
            *pPathLength = pPathEnd - *ppPath;
        }
    }

    return ret;
}

NetworkingUtilsResult_t NetworkingUtils_GenrerateAuthorizationHeader( NetworkingUtilsCanonicalRequest_t * pCanonicalRequest,
                                                                      SigV4Credentials_t * pSigv4Credential,
                                                                      const char * pAwsRegion,
                                                                      size_t awsRegionLength,
                                                                      const char * pDate,
                                                                      char * pOutput,
                                                                      size_t * pOutputLength,
                                                                      char ** ppOutSignature,
                                                                      size_t * pOutSignatureLength )
{
    NetworkingUtilsResult_t ret = NETWORKING_UTILS_RESULT_OK;
    SigV4HttpParameters_t sigv4HttpParams;
    SigV4Status_t sigv4Status = SigV4Success;

    if( ( pCanonicalRequest == NULL ) || ( pAwsRegion == NULL ) || ( pDate == NULL ) || ( pOutput == NULL ) || ( pOutputLength == NULL ) || ( ppOutSignature == NULL ) || ( pOutSignatureLength == NULL ) )
    {
        ret = NETWORKING_UTILS_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_UTILS_RESULT_OK )
    {
        /* Setup the HTTP parameters for SigV4. */
        sigv4HttpParams.flags = 0;
        if( pCanonicalRequest->verb == NETWORKING_UTILS_HTTP_VERB_POST )
        {
            sigv4HttpParams.pHttpMethod = HTTP_METHOD_POST;
            sigv4HttpParams.httpMethodLen = strlen( HTTP_METHOD_POST );
        }
        else
        {
            sigv4HttpParams.pHttpMethod = HTTP_METHOD_GET;
            sigv4HttpParams.httpMethodLen = strlen( HTTP_METHOD_GET );
            sigv4HttpParams.flags |= SIGV4_HTTP_QUERY_IS_CANONICAL_FLAG;
        }
        sigv4HttpParams.pPath = pCanonicalRequest->pPath;
        sigv4HttpParams.pathLen = pCanonicalRequest->pathLength;
        sigv4HttpParams.pQuery = pCanonicalRequest->pCanonicalQueryString;
        sigv4HttpParams.queryLen = pCanonicalRequest->canonicalQueryStringLength;
        sigv4HttpParams.pHeaders = pCanonicalRequest->pCanonicalHeaders;
        sigv4HttpParams.headersLen = pCanonicalRequest->canonicalHeadersLength;
        sigv4HttpParams.pPayload = pCanonicalRequest->pPayload;
        sigv4HttpParams.payloadLen = pCanonicalRequest->payloadLength;

        /* Initializing sigv4Params with Http parameters required for the HTTP request. */
        sigv4Params.pHttpParameters = &sigv4HttpParams;
        sigv4Params.pRegion = pAwsRegion;
        sigv4Params.regionLen = awsRegionLength;
        sigv4Params.pCredentials = pSigv4Credential;
        sigv4Params.pDateIso8601 = pDate;

        /* Reset buffer length then generate authorization. */
        sigv4Status = SigV4_GenerateHTTPAuthorization( &sigv4Params, pOutput, pOutputLength,
                                                       ppOutSignature, pOutSignatureLength );

        if( sigv4Status != SigV4Success )
        {
            LogError( ( "Fail to generate HTTP authorization with return 0x%x", sigv4Status ) );
            ret = NETWORKING_UTILS_RESULT_FAIL_SIGV4_GENAUTH;
        }
    }

    return ret;
}

void NetworkingUtils_GetHeaderStartLocFromHttpRequest( HTTPRequestHeaders_t * pxRequestHeaders,
                                                       char ** pcStartHeaderLoc,
                                                       size_t * pxHeadersDataLen )
{
    size_t xHeaderLen = 0U;
    char * pcHeaders = NULL;
    bool xNewLineFound = false;

    if( ( pxRequestHeaders != NULL ) && ( pcStartHeaderLoc != NULL ) && ( pxHeadersDataLen != NULL ) )
    {
        xHeaderLen = pxRequestHeaders->headersLen;
        pcHeaders = ( char * ) pxRequestHeaders->pBuffer;

        while( xHeaderLen >= 2 )
        {
            /* The request line ends in \r\n. Look for \r\n. */
            if( 0 == strncmp( pcHeaders, "\r\n", strlen( "\r\n" ) ) )
            {
                xNewLineFound = true;
                break;
            }

            pcHeaders++;
            xHeaderLen--;
        }

        if( xNewLineFound == false )
        {
            LogError( ( "Failed to find starting location of HTTP headers in HTTP request: \"\\r\\n\" missing before start of HTTP headers." ) );
            *pxHeadersDataLen = 0;
            *pcStartHeaderLoc = NULL;
        }
        else
        {
            /* Moving header pointer past "\r\n" .*/
            *pxHeadersDataLen = xHeaderLen - 2;
            *pcStartHeaderLoc = pcHeaders + 2;
        }
    }
}

NetworkingUtilsResult_t NetworkingUtils_GetIso8601CurrentTime( char * pDate,
                                                               size_t dateBufferLength )
{
    NetworkingUtilsResult_t ret = NETWORKING_UTILS_RESULT_OK;
    struct timespec nowTime;
    time_t timeT;
    size_t timeLength = 0;

    if( ( pDate == NULL ) || ( dateBufferLength < NETWORKING_UTILS_TIME_BUFFER_LENGTH ) )
    {
        ret = NETWORKING_UTILS_RESULT_BAD_PARAMETER;
    }

    if( ret == NETWORKING_UTILS_RESULT_OK )
    {
        clock_gettime( CLOCK_REALTIME, &nowTime );
        timeT = nowTime.tv_sec;

        timeLength = strftime( pDate, dateBufferLength, "%Y%m%dT%H%M%SZ", gmtime( &timeT ) );

        if( timeLength <= 0 )
        {
            LogError( ( "Fail to get time, timeLength=0x%x", timeLength ) );
            ret = NETWORKING_UTILS_RESULT_TIME_BUFFER_OUT_OF_MEMORY;
        }
    }

    if( ret == NETWORKING_UTILS_RESULT_OK )
    {
        pDate[ timeLength ] = '\0';
    }

    return ret;
}

uint64_t NetworkingUtils_GetCurrentTimeSec( void * pTick )
{
    long long sec;
    long long usec;
    unsigned int tick;
    unsigned int tickDiff;

    sntp_get_lasttime( &sec, &usec, &tick );

    if( pTick == NULL )
    {
        tickDiff = xTaskGetTickCount() - tick;
    }
    else
    {
        tickDiff = ( *( uint32_t * )pTick ) - tick;
    }

    sec += tickDiff / configTICK_RATE_HZ;
    usec += ( ( tickDiff % configTICK_RATE_HZ ) / portTICK_RATE_MS ) * 1000;

    while( usec >= 1000000 )
    {
        usec -= 1000000;
        sec++;
    }

    LogDebug( ( "sec: %lld, usec: %lld, tick: %u", sec, usec, tick ) );

    return ( uint64_t ) sec;
}

uint64_t NetworkingUtils_GetCurrentTimeUs( void * pTick )
{
    long long sec;
    long long usec;
    unsigned int tick;
    unsigned int tickDiff;

    sntp_get_lasttime( &sec, &usec, &tick );

    if( pTick == NULL )
    {
        tickDiff = xTaskGetTickCount() - tick;
    }
    else
    {
        tickDiff = ( *( uint32_t * )pTick ) - tick;
    }

    sec += tickDiff / configTICK_RATE_HZ;
    usec += ( ( tickDiff % configTICK_RATE_HZ ) / portTICK_RATE_MS ) * 1000;

    while( usec >= 1000000 )
    {
        usec -= 1000000;
        sec++;
    }

    // LogDebug( ( "pTick: %p, tickDiff: %u, sec: %lld, usec: %lld, tick: %u", pTick, tickDiff, sec, usec, tick ) );

    return ( ( uint64_t )sec * 1000000 ) + usec;
}

uint64_t NetworkingUtils_GetTimeFromIso8601( const char * pDate,
                                             size_t dateLength )
{
    uint64_t ret = 0;
    char isoTimeBuffer[NETWORKING_ISO8601_TIME_STRING_LENGTH + 1];
    int year, month, day, hour, minute, second;

    if( ( dateLength == NETWORKING_ISO8601_TIME_STRING_LENGTH ) && ( pDate != NULL ) )
    {
        memcpy( isoTimeBuffer, pDate, dateLength );
        isoTimeBuffer[dateLength] = '\0';

        if( sscanf( isoTimeBuffer, "%d-%d-%dT%d:%d:%dZ", &year, &month, &day, &hour, &minute, &second ) == 6 )
        {
            /* Convert the date and time fields to seconds since the epoch. */
            year -= 1900; /* tm_year is years since 1900. */
            month -= 1;   /* tm_mon is zero-based. */

            /* Days since epoch (1970-01-01) to the given date. */
            uint64_t daysSinceEpoch = 0;
            for( int y = 1970; y < 1900 + year; y++ )
            {
                daysSinceEpoch += ( ( y % 4 == 0 && y % 100 != 0 ) || ( y % 400 == 0 ) ) ? 366 : 365;
            }

            /* Add days for the current year. */
            static const int daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
            for( int m = 0; m < month; m++ )
            {
                daysSinceEpoch += daysInMonth[m];
                if( ( m == 1 ) && ( ( ( year % 4 == 0 ) && ( year % 100 != 0 ) ) || ( year % 400 == 0 ) ) )
                {
                    daysSinceEpoch += 1; /* Add a day for leap years. */
                }
            }

            daysSinceEpoch += day - 1; /* Add days in the current month. */

            /* Calculate total seconds since epoch. */
            ret = ( daysSinceEpoch * 24 * 3600 ) + ( hour * 3600 ) + ( minute * 60 ) + second;
        }
    }

    return ret;
}

/*
   Example walkthrough of ConvertMicrosecondsToNTP function
   Input: January 1, 2023 12:30:45.500000

   Using microseconds since Unix epoch (1970)
   timeUs = 1672527045500000ULL;  // Our input value in microseconds

   Step 1: Convert to seconds
   sec = timeUs / 1000000ULL;
   1672527045500000 / 1000000 = 1672527045 seconds

   Step 2: Get remainder (fractional part in microseconds)
   usec = timeUs % 1000000ULL;
   1672527045500000 % 1000000 = 500000 (0.5 seconds in microseconds)

   Step 3: Add NTP offset to seconds
   ntp_sec = sec + NETWORKING_NTP_OFFSET;
   1672527045 + 2208988800 = 3881515845

   Step 4: Convert fractional part to NTP scale
   NETWORKING_NTP_TIMESCALE is 2^32 = 4294967296
   500000 * 4294967296 / 1000000 = 2147483648

   Step 5: Combine into final NTP timestamp
   final_ntp = (ntp_sec << 32U | ntp_frac);
   3881515845 << 32 | 2147483648 = 16677181839663572288
 */
uint64_t NetworkingUtils_GetNTPTimeFromUnixTimeUs( uint64_t timeUs )
{
    uint64_t sec = timeUs / 1000000ULL;   // Convert microseconds to seconds
    uint64_t usec = timeUs % 1000000ULL;  // Get microsecond remainder

    uint64_t ntp_sec = sec + NETWORKING_NTP_OFFSET;
    uint64_t ntp_frac = ( usec * NETWORKING_NTP_TIMESCALE ) / 1000000ULL;

    return( ntp_sec << 32U | ntp_frac );
}
