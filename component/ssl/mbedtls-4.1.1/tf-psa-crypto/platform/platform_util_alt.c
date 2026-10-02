/*
 * Common and shared functions used by multiple modules in the Mbed TLS
 * library.
 *
 *  Copyright The Mbed TLS Contributors
 *  SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later
 */

#include "tf_psa_crypto_common.h"

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

#if defined(MBEDTLS_PSA_DRIVER_GET_ENTROPY)
#if defined(CONFIG_BUILD_NONSECURE) && (CONFIG_BUILD_NONSECURE == 1)
#include "osdep_service.h"
#else
#include "hal_trng_sec.h"
#endif
#include "diag.h"
int mbedtls_platform_get_entropy(psa_driver_get_entropy_flags_t flags,
                                 size_t *estimate_bits,
                                 unsigned char *output, size_t output_size)
{
    /* We don't implement any flags yet. */
    if (flags != 0) {
        return PSA_ERROR_NOT_SUPPORTED;
    }

#if defined(CONFIG_BUILD_NONSECURE) && (CONFIG_BUILD_NONSECURE == 1)
    rtw_get_random_bytes(output, output_size);
#else
    uint32_t r;
    uint32_t r_count = output_size / sizeof(uint32_t);

    hal_trng_sec_init();

    for (int i = 0; i < r_count; i ++) {
        r = hal_trng_sec_get_rand();
        memcpy(output + i * sizeof(uint32_t), &r, sizeof(uint32_t));
    }

    if (output_size > (r_count * sizeof(uint32_t))) {
        r = hal_trng_sec_get_rand();
        memcpy(output + r_count * sizeof(uint32_t), &r, output_size - r_count * sizeof(uint32_t));
    }
#endif

    *estimate_bits = 8 * output_size;

    return 0;
}
#endif /* MBEDTLS_PSA_DRIVER_GET_ENTROPY */
