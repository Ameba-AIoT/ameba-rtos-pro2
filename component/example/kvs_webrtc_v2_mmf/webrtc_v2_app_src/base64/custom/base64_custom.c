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

/**
 * Padding values for mod3 indicating how many '=' to append
 */
uint8_t base64EncodePadding[3] = {0, 2, 1};

/**
 * Padding values for mod4 indicating how many '=' has been padded. NOTE: value for 1 is invalid = 0xff
 */
uint8_t base64DecodePadding[4] = {0, 0xff, 2, 1};

/**
 * Base64 encoding alphabet
 */
uint8_t base64EecodeAlpha[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/**
 * Base64 decoding alphabet - an array of 256 values corresponding to the encoded base64 indexes
 * maps A -> 0, B -> 1, etc..
 */
uint8_t base64DecodeAlpha[256] =
{
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 10
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 20
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 30
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 40
    0,  0,  0,  62, 0,  0,  0,  63, 52, 53, // 50
    54, 55, 56, 57, 58, 59, 60, 61, 0,  0,  // 60
    0,  0,  0,  0,  0,  0,  1,  2,  3,  4,  // 70
    5,  6,  7,  8,  9,  10, 11, 12, 13, 14, // 80
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24, // 90
    25, 0,  0,  0,  0,  0,  0,  26, 27, 28, // 100
    29, 30, 31, 32, 33, 34, 35, 36, 37, 38, // 110
    39, 40, 41, 42, 43, 44, 45, 46, 47, 48, // 120
    49, 50, 51, 0,  0,  0,  0,  0,  0,  0,  // 130
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 140
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 150
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 160
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 170
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 180
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 190
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 200
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 210
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 220
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 230
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 240
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 250
    0,  0,  0,  0,  0,  0,
};

Base64Result_t Base64_Encode( const char * pInputData,
                              size_t inputDataLength,
                              char * pOutputData,
                              size_t * pOutputDataLength )
{
    Base64Result_t ret = BASE64_RESULT_OK;
    uint32_t padding, i;
    size_t mod3;
    const char * pInput = pInputData;
    char * pOutput = pOutputData;
    size_t outputLength = 0;
    uint8_t b0, b1, b2;

    if( ( pInputData == NULL ) || ( pOutputData == NULL ) || ( pOutputDataLength == NULL ) )
    {
        ret = BASE64_RESULT_BAD_PARAMETER;
    }

    if( ret == BASE64_RESULT_OK )
    {
        mod3 = inputDataLength % 3;
        padding = base64EncodePadding[mod3];
        outputLength = 4 * ( inputDataLength + padding ) / 3;

        if( outputLength > *pOutputDataLength )
        {
            ret = BASE64_RESULT_BUFFER_TOO_SMALL;
        }
    }

    if( ret == BASE64_RESULT_OK )
    {
        // Need to have at least a triade to process in the loop
        if( inputDataLength >= 3 )
        {
            for( i = 0; i <= inputDataLength - 3; i += 3 )
            {
                b0 = *pInput++;
                b1 = *pInput++;
                b2 = *pInput++;

                *pOutput++ = base64EecodeAlpha[ b0 >> 2 ];
                *pOutput++ = base64EecodeAlpha[ ( ( 0x03 & b0 ) << 4 ) + ( b1 >> 4 ) ];
                *pOutput++ = base64EecodeAlpha[ ( ( 0x0f & b1 ) << 2 ) + ( b2 >> 6 ) ];
                *pOutput++ = base64EecodeAlpha[ 0x3f & b2 ];
            }
        }
    }

    if( ret == BASE64_RESULT_OK )
    {
        // Process the padding
        if( padding == 1 )
        {
            *pOutput++ = base64EecodeAlpha[ *pInput >> 2 ];
            *pOutput++ = base64EecodeAlpha[ ( ( 0x03 & *pInput ) << 4 ) + ( *( pInput + 1 ) >> 4 ) ];
            *pOutput++ = base64EecodeAlpha[ ( 0x0f & *( pInput + 1 ) ) << 2 ];
            *pOutput++ = '=';
        }
        else if( padding == 2 )
        {
            *pOutput++ = base64EecodeAlpha[ *pInput >> 2 ];
            *pOutput++ = base64EecodeAlpha[ ( 0x03 & *pInput ) << 4 ];
            *pOutput++ = '=';
            *pOutput++ = '=';
        }

        // Set the correct size
        *pOutputDataLength = outputLength;
    }

    return ret;
}

Base64Result_t Base64_Decode( const char * pInputData,
                              size_t inputDataLength,
                              char * pOutputData,
                              size_t * pOutputDataLength )
{
    Base64Result_t ret = BASE64_RESULT_OK;
    const char * pInput = pInputData;
    char * pOutput = pOutputData;
    uint32_t padding, i;
    size_t outputLength = 0;
    uint8_t b0, b1, b2, b3;

    if( ( pInputData == NULL ) || ( pOutputData == NULL ) || ( pOutputDataLength == NULL ) )
    {
        ret = BASE64_RESULT_BAD_PARAMETER;
    }

    if( ret == BASE64_RESULT_OK )
    {
        // Check the size - should have more than 2 chars
        if( inputDataLength < 2U )
        {
            ret = BASE64_RESULT_INVALID_INPUT;
        }
    }

    if( ret == BASE64_RESULT_OK )
    {
        // Check the padding twice
        if( pInputData[ inputDataLength - 1 ] == '=' )
        {
            inputDataLength--;
        }

        if( pInputData[ inputDataLength - 1 ] == '=' )
        {
            inputDataLength--;
        }

        // Calculate the padding
        padding = base64DecodePadding[ inputDataLength % 4 ];

        // Mod4 can't be 1 which means the padding can never be 0xff
        if( padding == 0xff )
        {
            ret = BASE64_RESULT_INVALID_INPUT;
        }
    }

    if( ret == BASE64_RESULT_OK )
    {
        // Calculate the output length
        outputLength = 3 * inputDataLength / 4;

        // Check against the buffer size that's been supplied
        if( *pOutputDataLength < outputLength )
        {
            ret = BASE64_RESULT_BUFFER_TOO_SMALL;
        }
    }

    if( ret == BASE64_RESULT_OK )
    {
        if( inputDataLength >= 4 )
        {
            for( i = 0; i <= inputDataLength - 4; i += 4 )
            {
                b0 = base64DecodeAlpha[ ( uint8_t ) *pInput++ ];
                b1 = base64DecodeAlpha[ ( uint8_t ) *pInput++ ];
                b2 = base64DecodeAlpha[ ( uint8_t ) *pInput++ ];
                b3 = base64DecodeAlpha[ ( uint8_t ) *pInput++ ];

                *pOutput++ = ( b0 << 2 ) | ( b1 >> 4 );
                *pOutput++ = ( b1 << 4 ) | ( b2 >> 2 );
                *pOutput++ = ( b2 << 6 ) | b3;
            }
        }

        // Process the padding
        if( padding == 1 )
        {
            b0 = base64DecodeAlpha[ ( uint8_t ) *pInput++ ];
            b1 = base64DecodeAlpha[ ( uint8_t ) *pInput++ ];
            b2 = base64DecodeAlpha[ ( uint8_t ) *pInput++ ];

            *pOutput++ = ( b0 << 2 ) | ( b1 >> 4 );
            *pOutput++ = ( b1 << 4 ) | ( b2 >> 2 );
        }
        else if( padding == 2 )
        {
            b0 = base64DecodeAlpha[( uint8_t ) *pInput++];
            b1 = base64DecodeAlpha[( uint8_t ) *pInput++];

            *pOutput++ = ( b0 << 2 ) | ( b1 >> 4 );
        }
        else
        {
            /* Do nothing, coverity happy. */
        }

        // Set the correct size
        *pOutputDataLength = outputLength;
    }

    return ret;
}
