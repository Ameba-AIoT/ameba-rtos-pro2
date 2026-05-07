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

#include "transport_dtls_mbedtls.h"
#include "transport_dtls_mbedtls_port.h"

void mbedtls_timing_set_delay( void * data,
                               uint32_t int_ms,
                               uint32_t fin_ms ) {
    // LogDebug(("mbedtls_timing_set_delay: int_ms %li,fin_ms %li", int_ms, fin_ms)); ;
    DtlsSessionTimer_t * ctx = ( DtlsSessionTimer_t * ) data;
    ctx->start_ticks = xTaskGetTickCount();

    ctx->int_ms = ctx->start_ticks + int_ms;
    ctx->fin_ms = ctx->start_ticks + fin_ms;

    // LogDebug(("mbedtls_timing_set_delay start_ticks: %lli",ctx->start_ticks));
}

int mbedtls_timing_get_delay( void * data ) {
    // LogDebug(("mbedtls_timing_get_delay"));
    DtlsSessionTimer_t * ctx = ( DtlsSessionTimer_t * ) data;
    int64_t elapsed_ticks = xTaskGetTickCount() - ctx->start_ticks;
    int64_t elapsed_ms = elapsed_ticks * portTICK_PERIOD_MS;
    // LogDebug(("mbedtls_timing_get_delay elapsed_ticks: %lli",elapsed_ticks));
    if( ctx->fin_ms == 0 )
    {
        return -1;
    }

    if( elapsed_ms >= ctx->fin_ms )
    {
        return 2;
    }

    if( elapsed_ms >= ctx->int_ms )
    {
        return 1;
    }

    return 0;
}
