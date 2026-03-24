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

#ifndef TRANSPORT_DTLS_PORT_H
#define TRANSPORT_DTLS_PORT_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// include for retransmission timer
#include "timers.h"

// need for x509 cert generation
#include "time.h"

void mbedtls_timing_set_delay( void * data,
                               uint32_t int_ms,
                               uint32_t fin_ms );

int mbedtls_timing_get_delay( void * data );

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_DTLS_PORT_H */
