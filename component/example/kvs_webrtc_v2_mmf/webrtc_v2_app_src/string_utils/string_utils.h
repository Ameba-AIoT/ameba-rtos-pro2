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

#ifndef STRING_UTILS_H
#define STRING_UTILS_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>

typedef enum StringUtilsResult
{
    STRING_UTILS_RESULT_OK = 0,
    STRING_UTILS_RESULT_BAD_PARAMETER,
    STRING_UTILS_RESULT_NON_NUMBERIC_STRING,
} StringUtilsResult_t;

StringUtilsResult_t StringUtils_ConvertStringToUl( const char * pStr,
                                                   size_t strLength,
                                                   uint32_t * pOutUl );
StringUtilsResult_t StringUtils_ConvertStringToHex( const char * pStr,
                                                    size_t strLength,
                                                    uint32_t * pOutUl );
const char * StringUtils_StrStr( const char * pStr,
                                 size_t strLength,
                                 const char * pPattern,
                                 size_t patternLength );

#ifdef __cplusplus
}
#endif

#endif /* STRING_UTILS_H */
