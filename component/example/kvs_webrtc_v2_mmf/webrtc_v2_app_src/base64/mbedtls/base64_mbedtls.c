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
#include "base64.h"
#include "mbedtls/base64.h"

static Base64Result_t Base64_InterpretReturnValue( int errorCode )
{
    Base64Result_t ret = BASE64_RESULT_OK;

    switch( errorCode )
    {
        case MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL:
            ret = BASE64_RESULT_BUFFER_TOO_SMALL;
            break;
        case MBEDTLS_ERR_BASE64_INVALID_CHARACTER:
        default:
            ret = BASE64_RESULT_INVALID_INPUT;
            break;
    }

    return ret;
}

Base64Result_t Base64_Encode( const char * pInputData,
                              size_t inputDataLength,
                              char * pOutputData,
                              size_t * pOutputDataLength )
{
    Base64Result_t ret = BASE64_RESULT_OK;
    size_t olen = 0;
    int retBase64;

    retBase64 = mbedtls_base64_encode( ( unsigned char * ) pOutputData, *pOutputDataLength, &olen, ( const unsigned char * ) pInputData, inputDataLength );
    if( retBase64 == 0 )
    {
        /* Update output length for user. */
        *pOutputDataLength = olen;
    }
    else
    {
        ret = Base64_InterpretReturnValue( retBase64 );
    }

    return ret;
}

Base64Result_t Base64_Decode( const char * pInputData,
                              size_t inputDataLength,
                              char * pOutputData,
                              size_t * pOutputDataLength )
{
    Base64Result_t ret = BASE64_RESULT_OK;
    size_t olen = 0;
    int retBase64;

    retBase64 = mbedtls_base64_decode( ( unsigned char * ) pOutputData, *pOutputDataLength, &olen, ( const unsigned char * ) pInputData, inputDataLength );
    if( retBase64 == 0 )
    {
        /* Update output length for user. */
        *pOutputDataLength = olen;
    }
    else
    {
        ret = Base64_InterpretReturnValue( retBase64 );
    }

    return ret;
}
