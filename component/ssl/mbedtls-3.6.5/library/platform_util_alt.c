/*
 * Common and shared functions used by multiple modules in the Mbed TLS
 * library.
 *
 *  Copyright The Mbed TLS Contributors
 *  SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later
 */

#include "common.h"

#include "mbedtls/platform_util.h"
#include "mbedtls/platform.h"

#if defined(MBEDTLS_HAVE_TIME) && defined(MBEDTLS_PLATFORM_MS_TIME_ALT)
#include "FreeRTOS_POSIX.h"
#include "FreeRTOS_POSIX/time.h"

mbedtls_ms_time_t mbedtls_ms_time(void)
{

#if defined(CONFIG_BUILD_SECURE) && (CONFIG_BUILD_SECURE == 1)
    extern mbedtls_ms_time_t ns_ms_time_call(void);
    return ns_ms_time_call();
#else
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return tv.tv_sec * 1000 + tv.tv_nsec / 1000000;
#endif
}
#endif /* MBEDTLS_HAVE_TIME && MBEDTLS_PLATFORM_MS_TIME_ALT */
