/*
 *  Elliptic curve Diffie-Hellman
 *
 *  Copyright The Mbed TLS Contributors
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*
 * References:
 *
 * SEC1 http://www.secg.org/index.php?action=secg,docs_secg
 * RFC 4492
 */

#include "common.h"

#if defined(MBEDTLS_ECDH_C)

#include "mbedtls/ecdh.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"

#include <string.h>

#include "ecdsa_api.h"
#include "device_lock.h"

/* Parameter validation macros based on platform_util.h */
#define ECDH_VALIDATE_RET( cond )    \
    MBEDTLS_INTERNAL_VALIDATE_RET( cond, MBEDTLS_ERR_ECP_BAD_INPUT_DATA )

static uint32_t ecdsa_pub_x[8] = {0};
static uint32_t ecdsa_pub_y[8] = {0};
static uint32_t ecdsa_err = 0;
static volatile uint32_t ecdsa_finish_flag = 0;

static void ecdsa_irq_callback( void *data )
{
    ecdsa_t *pecdsa_in = (ecdsa_t *) data;

    ecdsa_err = ecdsa_get_err_sta( pecdsa_in );

    memset( ecdsa_pub_x, 0, sizeof( ecdsa_pub_x ) );
    memset( ecdsa_pub_y, 0, sizeof( ecdsa_pub_y ) );
    ecdsa_get_pbk( pecdsa_in, ecdsa_pub_x, ecdsa_pub_y );

    ecdsa_finish_flag = 1;
}

#if defined(MBEDTLS_ECDH_GEN_PUBLIC_ALT)
/*
 * Generate public key (restartable version)
 *
 * Note: this internal function relies on its caller preserving the value of
 * the output parameter 'd' across continuation calls. This would not be
 * acceptable for a public function but is OK here as we control call sites.
 */
static int ecdh_gen_public_restartable( mbedtls_ecp_group *grp,
                    mbedtls_mpi *d, mbedtls_ecp_point *Q,
                    int (*f_rng)(void *, unsigned char *, size_t),
                    void *p_rng,
                    mbedtls_ecp_restart_ctx *rs_ctx )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;

    /* If multiplication is in progress, we already generated a privkey */
#if defined(MBEDTLS_ECP_RESTARTABLE)
    if( rs_ctx == NULL || rs_ctx->rsm == NULL )
#endif
        MBEDTLS_MPI_CHK( mbedtls_ecp_gen_privkey( grp, d, f_rng, p_rng ) );

    if( grp->id == MBEDTLS_ECP_DP_SECP256R1 )
    {
        device_mutex_lock( RT_DEV_LOCK_CRYPTO );

        ecdsa_t ecdsa_obj;
        ecdsa_init( &ecdsa_obj );
        ecdsa_cb_irq_handler( &ecdsa_obj, (ecdsa_irq_user_cb_t) ecdsa_irq_callback, &ecdsa_obj );
        ecdsa_curve_para_t curve_para;
        curve_para.curve = ECDSA_P256;
        ecdsa_set_curve( &ecdsa_obj, &curve_para );

        ecdsa_finish_flag = 0;
        ecdsa_gen_public_key( &ecdsa_obj, d->p, grp->G.X.p, grp->G.Y.p );

        uint32_t ecdsa_timeout = 20000000;
        while( ecdsa_finish_flag == 0 )
        {
            ecdsa_timeout --;
            if( ecdsa_timeout == 0 )
                break;
        }

        ecdsa_deinit( &ecdsa_obj );

        if( ( ecdsa_finish_flag == 1 ) && ( ecdsa_err == 0 ) )
        {
            mbedtls_mpi_read_binary_le( &Q->X, (uint8_t *) ecdsa_pub_x, 32 );
            mbedtls_mpi_read_binary_le( &Q->Y, (uint8_t *) ecdsa_pub_y, 32 );
            mbedtls_mpi_lset( &Q->Z, 1 );
            ret = 0;
        }
        else
        {
            ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
        }

        device_mutex_unlock( RT_DEV_LOCK_CRYPTO );
    }
    else
    {
        MBEDTLS_MPI_CHK( mbedtls_ecp_mul_restartable( grp, Q, d, &grp->G,
                                                      f_rng, p_rng, rs_ctx ) );
    }

cleanup:
    return( ret );
}

/*
 * Generate public key
 */
int mbedtls_ecdh_gen_public( mbedtls_ecp_group *grp, mbedtls_mpi *d, mbedtls_ecp_point *Q,
                     int (*f_rng)(void *, unsigned char *, size_t),
                     void *p_rng )
{
    ECDH_VALIDATE_RET( grp != NULL );
    ECDH_VALIDATE_RET( d != NULL );
    ECDH_VALIDATE_RET( Q != NULL );
    ECDH_VALIDATE_RET( f_rng != NULL );
    return( ecdh_gen_public_restartable( grp, d, Q, f_rng, p_rng, NULL ) );
}
#endif /* MBEDTLS_ECDH_GEN_PUBLIC_ALT */

#if defined(MBEDTLS_ECDH_COMPUTE_SHARED_ALT)
/*
 * Compute shared secret (SEC1 3.3.1)
 */
static int ecdh_compute_shared_restartable( mbedtls_ecp_group *grp,
                         mbedtls_mpi *z,
                         const mbedtls_ecp_point *Q, const mbedtls_mpi *d,
                         int (*f_rng)(void *, unsigned char *, size_t),
                         void *p_rng,
                         mbedtls_ecp_restart_ctx *rs_ctx )
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    mbedtls_ecp_point P;

    mbedtls_ecp_point_init( &P );

    if( grp->id == MBEDTLS_ECP_DP_SECP256R1 )
    {
        device_mutex_lock( RT_DEV_LOCK_CRYPTO );

        ecdsa_t ecdsa_obj;
        ecdsa_init( &ecdsa_obj );
        ecdsa_cb_irq_handler( &ecdsa_obj, (ecdsa_irq_user_cb_t) ecdsa_irq_callback, &ecdsa_obj );
        ecdsa_curve_para_t curve_para;
        curve_para.curve = ECDSA_P256;
        ecdsa_set_curve( &ecdsa_obj, &curve_para );

        ecdsa_finish_flag = 0;
        ecdsa_gen_public_key( &ecdsa_obj, d->p, Q->X.p, Q->Y.p );

        uint32_t ecdsa_timeout = 20000000;
        while( ecdsa_finish_flag == 0 )
        {
            ecdsa_timeout --;
            if( ecdsa_timeout == 0 )
                break;
        }

        ecdsa_deinit( &ecdsa_obj );

        if( ( ecdsa_finish_flag == 1 ) && ( ecdsa_err == 0 ) )
        {
            mbedtls_mpi_read_binary_le( &P.X, (uint8_t *) ecdsa_pub_x, 32 );
            mbedtls_mpi_read_binary_le( &P.Y, (uint8_t *) ecdsa_pub_y, 32 );
            mbedtls_mpi_lset( &P.Z, 1 );
            device_mutex_unlock( RT_DEV_LOCK_CRYPTO );
            ret = 0;
        }
        else
        {
            device_mutex_unlock( RT_DEV_LOCK_CRYPTO );
            ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
            goto cleanup;
        }
    }
    else
    {
        MBEDTLS_MPI_CHK( mbedtls_ecp_mul_restartable( grp, &P, d, Q,
                                                      f_rng, p_rng, rs_ctx ) );
    }

    if( mbedtls_ecp_is_zero( &P ) )
    {
        ret = MBEDTLS_ERR_ECP_BAD_INPUT_DATA;
        goto cleanup;
    }

    MBEDTLS_MPI_CHK( mbedtls_mpi_copy( z, &P.X ) );

cleanup:
    mbedtls_ecp_point_free( &P );

    return( ret );
}

/*
 * Compute shared secret (SEC1 3.3.1)
 */
int mbedtls_ecdh_compute_shared( mbedtls_ecp_group *grp, mbedtls_mpi *z,
                         const mbedtls_ecp_point *Q, const mbedtls_mpi *d,
                         int (*f_rng)(void *, unsigned char *, size_t),
                         void *p_rng )
{
    ECDH_VALIDATE_RET( grp != NULL );
    ECDH_VALIDATE_RET( Q != NULL );
    ECDH_VALIDATE_RET( d != NULL );
    ECDH_VALIDATE_RET( z != NULL );
    return( ecdh_compute_shared_restartable( grp, z, Q, d,
                                             f_rng, p_rng, NULL ) );
}
#endif /* MBEDTLS_ECDH_COMPUTE_SHARED_ALT */

#endif /* MBEDTLS_ECDH_C */
