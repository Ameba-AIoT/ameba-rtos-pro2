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

#ifndef WSLAY_HELPER_H
#define WSLAY_HELPER_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include "sigv4.h"
#include "transport_mbedtls.h"
#include "networking_utils.h"

#include "lwip/sockets.h"
#include "wslay/wslay.h"

/* FreeRTOS includes. */
#include "task.h"

#define NETWORKING_WEBSOCKET_BUFFER_LENGTH ( 10000 )
#define NETWORKING_META_BUFFER_LENGTH ( 4096 )

typedef enum NetworkingWslayResult
{
    NETWORKING_WSLAY_RESULT_OK = 0,
    NETWORKING_WSLAY_RESULT_BAD_PARAMETER,
    NETWORKING_WSLAY_RESULT_CONNETION_CLOSED,
    NETWORKING_WSLAY_RESULT_FAIL_CONNECT,
    NETWORKING_WSLAY_RESULT_FAIL_SNPRINTF,
    NETWORKING_WSLAY_RESULT_FAIL_HTTP_INIT_REQUEST_HEADER,
    NETWORKING_WSLAY_RESULT_FAIL_GET_DATE,
    NETWORKING_WSLAY_RESULT_FAIL_HTTP_ADD,
    NETWORKING_WSLAY_RESULT_FAIL_HTTP_SEND,
    NETWORKING_WSLAY_RESULT_FAIL_HTTP_PARSE_RESPONSE,
    NETWORKING_WSLAY_RESULT_FAIL_BASE64_ENCODE,
    NETWORKING_WSLAY_RESULT_FAIL_VERIFY_ACCEPT_KEY,
    NETWORKING_WSLAY_RESULT_FAIL_SEND,
    NETWORKING_WSLAY_RESULT_FAIL_SELECT,
    NETWORKING_WSLAY_RESULT_FAIL_RECV,
    NETWORKING_WSLAY_RESULT_FAIL_QUEUE,
    NETWORKING_WSLAY_RESULT_FAIL_WRITE_ENABLE,
    NETWORKING_WSLAY_RESULT_FAIL_CREATE_SOCKET,
    NETWORKING_WSLAY_RESULT_FAIL_BIND_SOCKET,
    NETWORKING_WSLAY_RESULT_FAIL_FCNTL,
    NETWORKING_WSLAY_RESULT_FAIL_BASE64_DECODE,
    NETWORKING_WSLAY_RESULT_USER_AGENT_NAME_LENGTH_TOO_LONG,
    NETWORKING_WSLAY_RESULT_NO_HOST_IN_URL,
    NETWORKING_WSLAY_RESULT_NO_PATH_IN_URL,
    NETWORKING_WSLAY_RESULT_UNEXPECTED_WEBSOCKET_URL,
    NETWORKING_WSLAY_RESULT_QUERY_PARAM_BUFFER_TOO_SMALL,
    NETWORKING_WSLAY_RESULT_URI_ENCODED_BUFFER_TOO_SMALL,
    NETWORKING_WSLAY_RESULT_AUTH_BUFFER_TOO_SMALL,
    NETWORKING_WSLAY_RESULT_UNKNOWN_MESSAGE,
} NetworkingWslayResult_t;

typedef enum NetworkingWslayHttpHeader
{
    NETWORKING_WSLAY_HTTP_HEADER_CONNECTION = 1,
    NETWORKING_WSLAY_HTTP_HEADER_UPGRADE = 2,
    NETWORKING_WSLAY_HTTP_HEADER_WEBSOCKET_ACCEPT = 4,
} NetworkingWslayHttpHeader_t;

typedef struct NetworkingWslayConnectResponseContext
{
    /* user-agent */
    char * pClientKey;
    size_t clientKeyLength;

    uint8_t headersParsed; //bitmap with NetworkingWslayHttpHeader_t value.
    uint16_t statusCode; //bitmap with NetworkingWslayHttpHeader_t value.
} NetworkingWslayConnectResponseContext_t;

#ifdef __cplusplus
}
#endif

#endif /* WSLAY_HELPER_H */
