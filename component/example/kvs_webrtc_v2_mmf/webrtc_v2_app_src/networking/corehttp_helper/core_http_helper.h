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

#ifndef CORE_HTTP_HELPER_H
#define CORE_HTTP_HELPER_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include "sigv4.h"
#include "networking_utils.h"

/* Transport interface implementation include header for TLS. */
#include "transport_mbedtls.h"


#define NETWORKING_COREHTTP_DEFAULT_REGION "us-west-2"

#define NETWORKING_COREHTTP_USER_AGENT_NAME_MAX_LENGTH ( 128 )
#define NETWORKING_COREHTTP_HOST_NAME_MAX_LENGTH ( 256 )
#define NETWORKING_COREHTTP_BUFFER_LENGTH ( 10000 )
#define NETWORKING_COREHTTP_SIGV4_METADATA_BUFFER_LENGTH ( 4096 )

typedef enum NetworkingCorehttpResult
{
    NETWORKING_COREHTTP_RESULT_OK = 0,
    NETWORKING_COREHTTP_RESULT_BAD_PARAMETER,
    NETWORKING_COREHTTP_RESULT_USER_AGENT_NAME_TOO_LONG,
    NETWORKING_COREHTTP_RESULT_FAIL_CONNECT,
    NETWORKING_COREHTTP_RESULT_FAIL_HTTP_INIT_REQUEST_HEADER,
    NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_HOST,
    NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_USER_AGENT,
    NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_DATE,
    NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_CONTENT_TYPE,
    NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_IOT_THING_NAME,
    NETWORKING_COREHTTP_RESULT_FAIL_HTTP_ADD_HEADER_AUTH,
    NETWORKING_COREHTTP_RESULT_FAIL_HTTP_SEND,
    NETWORKING_COREHTTP_RESULT_FAIL_SIGV4_GENERATE_AUTH,
    NETWORKING_COREHTTP_RESULT_FAIL_GET_DATE,
    NETWORKING_COREHTTP_RESULT_NO_HOST_IN_URL,
    NETWORKING_COREHTTP_RESULT_NO_PATH_IN_URL,
} NetworkingCorehttpResult_t;

#ifdef __cplusplus
}
#endif

#endif /* CORE_HTTP_HELPER_H */
