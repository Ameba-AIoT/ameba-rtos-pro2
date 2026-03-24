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

#include "string_utils.h"

StringUtilsResult_t StringUtils_ConvertStringToUl( const char * pStr,
                                                   size_t strLength,
                                                   uint32_t * pOutUl )
{
    StringUtilsResult_t ret = STRING_UTILS_RESULT_OK;
    uint32_t i, result = 0;

    if( ( pStr == NULL ) || ( pOutUl == NULL ) )
    {
        return STRING_UTILS_RESULT_OK;
    }

    if( ret == STRING_UTILS_RESULT_OK )
    {
        for( i = 0; pStr[i] != '\0' && i < strLength; i++ )
        {
            if( ( pStr[i] >= '0' ) && ( pStr[i] <= '9' ) )
            {
                result = result * 10 + ( pStr[i] - '0' );
            }
            else if( i == 0 )
            {
                ret = STRING_UTILS_RESULT_NON_NUMBERIC_STRING;
                break;
            }
            else
            {
                break;
            }
        }
    }

    if( ret == STRING_UTILS_RESULT_OK )
    {
        *pOutUl = result;
    }

    return ret;
}

StringUtilsResult_t StringUtils_ConvertStringToHex( const char * pStr,
                                                    size_t strLength,
                                                    uint32_t * pOutUl )
{
    StringUtilsResult_t ret = STRING_UTILS_RESULT_OK;
    int result;

    if( ( pStr == NULL ) || ( pOutUl == NULL ) )
    {
        return STRING_UTILS_RESULT_OK;
    }

    if( ret == STRING_UTILS_RESULT_OK )
    {
        result = sscanf( pStr, "%lx", pOutUl );
        if( result < 1 )
        {
            ret = STRING_UTILS_RESULT_NON_NUMBERIC_STRING;
        }
    }

    return ret;
}

const char * StringUtils_StrStr( const char * pStr,
                                 size_t strLength,
                                 const char * pPattern,
                                 size_t patternLength )
{
    const char * pRet = NULL;
    const char * pCurrentStr, * pCurrentPattern;
    int i;
    size_t checkedLength = 0;

    if( ( pPattern == NULL ) || ( patternLength == 0 ) )
    {
        pRet = pStr;
    }
    else if( pStr && pPattern && ( patternLength <= strLength ) )
    {
        for( i = 0; i <= strLength - patternLength; i++ )
        {
            pCurrentStr = &pStr[i];
            pCurrentPattern = pPattern;
            checkedLength = 0;
            while( pCurrentStr[ checkedLength ] == pCurrentPattern[ checkedLength ] && checkedLength < patternLength )
            {
                checkedLength++;
            }

            if( checkedLength == patternLength )
            {
                /* Found pattern. */
                pRet = &pStr[i];
                break;
            }
        }
    }
    else
    {
        pRet = NULL;
    }

    return pRet;
}
