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

#ifndef BASE64_H
#define BASE64_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>

typedef enum Base64Result
{
    BASE64_RESULT_OK = 0,
    BASE64_RESULT_BAD_PARAMETER,
    BASE64_RESULT_INVALID_INPUT,
    BASE64_RESULT_BUFFER_TOO_SMALL,
} Base64Result_t;

Base64Result_t Base64_Decode( const char * pInputData,
                              size_t inputDataLength,
                              char * pOutputData,
                              size_t * pOutputDataLength );
Base64Result_t Base64_Encode( const char * pInputData,
                              size_t inputDataLength,
                              char * pOutputData,
                              size_t * pOutputDataLength );

#ifdef __cplusplus
}
#endif

#endif /* BASE64_H */