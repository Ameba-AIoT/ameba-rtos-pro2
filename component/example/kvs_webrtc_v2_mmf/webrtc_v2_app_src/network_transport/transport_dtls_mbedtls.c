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

#include "logging.h"

/* Standard includes. */
#include <string.h>

#include "FreeRTOS.h"
#include "mbedtls/config.h"
#include "mbedtls/pem.h"
#include "mbedtls/sha256.h"
#include "mbedtls/ssl.h"

#ifdef MBEDTLS_PSA_CRYPTO_C
/* MbedTLS PSA Includes */
#include "psa/crypto.h"
#include "psa/crypto_values.h"
#endif /* MBEDTLS_PSA_CRYPTO_C */

#include "mbedtls/debug.h"

/* DTLS transport header. */
#include "transport_dtls_mbedtls.h"

/* OS specific port header. */
#include "transport_dtls_mbedtls_port.h"

/*-----------------------------------------------------------*/

/**  https://tools.ietf.org/html/rfc5764#section-4.1.2 */
mbedtls_ssl_srtp_profile DTLS_SRTP_SUPPORTED_PROFILES[] = {
#if ( MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100 )
    MBEDTLS_TLS_SRTP_AES128_CM_HMAC_SHA1_80,
    MBEDTLS_TLS_SRTP_AES128_CM_HMAC_SHA1_32,
    MBEDTLS_TLS_SRTP_UNSET,
#else
    MBEDTLS_SRTP_AES128_CM_HMAC_SHA1_80,
    MBEDTLS_SRTP_AES128_CM_HMAC_SHA1_32,
#endif
};

/**
 * @brief Utility for converting the high-level code in an mbedTLS error to
 * string, if the code-contains a high-level code; otherwise, using a default
 * string.
 */
#define mbedtlsHighLevelCodeOrDefault( mbedTlsCode ) "mbedTLS high level Error"

/**
 * @brief Utility for converting the level-level code in an mbedTLS error to
 * string, if the code-contains a level-level code; otherwise, using a default
 * string.
 */
#define mbedtlsLowLevelCodeOrDefault( mbedTlsCode ) "mbedTLS low level Error"

/*-----------------------------------------------------------*/

/**
 * @brief Initialize the mbed DTLS structures in a network connection.
 *
 * @param[in] pSslContext The DTLS SSL context to initialize.
 */
static void DtlsSslContextInit( DtlsSSLContext_t * pSslContext );

/**
 * @brief Free the mbed DTLS structures in a network connection.
 *
 * @param[in] pSslContext The SSL context to free.
 */
static void DtlsSslContextFree( DtlsSSLContext_t * pSslContext );

/**
 * @brief Passes DTLS credentials to the OpenSSL library.
 *
 * Provides the root CA certificate, client certificate, and private key to the
 * OpenSSL library. If the client certificate or private key is not NULL, mutual
 * authentication is used when performing the DTLS handshake.
 *
 * @param[out] pSslContext SSL context to which the credentials are to be
 * imported.
 * @param[in] pNetworkCredentials DTLS credentials to be imported.
 *
 * @return 0 on success; otherwise, failure;
 */
static int32_t setCredentials( DtlsSSLContext_t * pSslContext,
                               DtlsNetworkCredentials_t * pNetworkCredentials );

/**
 * @brief Setup DTLS by initializing contexts and setting configurations.
 *
 * @param[in] pDtlsNetworkContext Network context.
 * @param[in] pHostName Remote host name, used for server name indication.
 * @param[in] pNetworkCredentials DTLS setup parameters.
 * @param[in] isServer The role of DTLS is server or not.
 *
 * @return #DTLS_SUCCESS, #DTLS_TRANSPORT_INSUFFICIENT_MEMORY,
 * #DTLS_TRANSPORT_INVALID_CREDENTIALS, or #DTLS_TRANSPORT_INTERNAL_ERROR.
 */
static DtlsTransportStatus_t dtlsSetup( DtlsNetworkContext_t * pDtlsNetworkContext,
                                        DtlsNetworkCredentials_t * pNetworkCredentials,
                                        uint8_t isServer );

/**
 * @brief Initialize mbedTLS.
 *
 * @param[out] entropyContext mbed DTLS entropy context for generation of random
 * numbers.
 * @param[out] ctrDrbgContext mbed DTLS CTR DRBG context for generation of
 * random numbers.
 *
 * @return #DTLS_SUCCESS, or #DTLS_TRANSPORT_INTERNAL_ERROR.
 */
static DtlsTransportStatus_t initMbedtls( mbedtls_entropy_context * pEntropyContext,
                                          mbedtls_ctr_drbg_context * pCtrDrbgContext );

/*-----------------------------------------------------------*/

#ifdef MBEDTLS_DEBUG_C
void dtls_mbedtls_string_printf( void * sslContext,
                                 int level,
                                 const char * file,
                                 int line,
                                 const char * str )
{
    if( ( str != NULL ) && ( file != NULL ) )
    {
        LogDebug( ( "%s:%d: [%d] %s", file, line, level, str ) );
    }
}
#endif /* MBEDTLS_DEBUG_C */

/*-----------------------------------------------------------*/

static int DtlsUdpSendWrap( void * pCustomCtx,
                            const unsigned char * pBuf,
                            size_t len )
{
    int ret = 0;
    DtlsTransportParams_t * pDtlsTransportParams = ( DtlsTransportParams_t * ) pCustomCtx;

    if( pDtlsTransportParams == NULL )
    {
        LogError( ( "Invalid parameter: pCustomCtx is NULL." ) );
        return -1;
    }

    if( ret == 0 )
    {
        if( pDtlsTransportParams->onDtlsSendHook != NULL )
        {
            ret = pDtlsTransportParams->onDtlsSendHook( pDtlsTransportParams->pOnDtlsSendCustomContext, pBuf, len );
        }
        else
        {
            LogError( ( "No send function to send DTLS packet." ) );
            ret = -2;
        }
    }

    return ret;
}
/*-----------------------------------------------------------*/

static int DtlsUdpRecvWrap( void * pCustomCtx,
                            unsigned char * pBuf,
                            size_t len )
{
    int ret = 0;
    DtlsTransportParams_t * pDtlsTransportParams = ( DtlsTransportParams_t * ) pCustomCtx;

    if( pDtlsTransportParams == NULL )
    {
        LogError( ( "Invalid parameter: pCustomCtx is NULL." ) );
        ret = -1;
    }
    else if( pBuf == NULL )
    {
        LogError( ( "Invalid parameter: pBuf is NULL." ) );
        ret = -2;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == 0 )
    {
        if( pDtlsTransportParams->pReceivedPacket == NULL )
        {
            ret = MBEDTLS_ERR_SSL_WANT_READ;
        }
    }

    if( ret == 0 )
    {
        if( len > pDtlsTransportParams->receivedPacketLength )
        {
            ret = pDtlsTransportParams->receivedPacketLength;
        }
        else
        {
            ret = len;
        }

        /* Copy the buffer content to mbedtls buffer. */
        memcpy( pBuf, pDtlsTransportParams->pReceivedPacket + pDtlsTransportParams->receivedPacketOffset, ret );

        if( ret + pDtlsTransportParams->receivedPacketOffset >= pDtlsTransportParams->receivedPacketLength )
        {
            /* Received packet is all addressed. */
            pDtlsTransportParams->pReceivedPacket = NULL;
            pDtlsTransportParams->receivedPacketLength = 0;
            pDtlsTransportParams->receivedPacketOffset = 0;
        }
        else
        {
            /* Received packet is partially addressed. */
            pDtlsTransportParams->receivedPacketOffset += ret;
        }
    }

    return ret;
}
/*-----------------------------------------------------------*/

static void DtlsSslContextInit( DtlsSSLContext_t * pSslContext )
{
    configASSERT( pSslContext != NULL );

    mbedtls_ssl_config_init( &( pSslContext->config ) );
    mbedtls_ssl_init( &( pSslContext->context ) );
    #ifdef MBEDTLS_DEBUG_C
    mbedtls_debug_set_threshold( 1 );
    mbedtls_ssl_conf_dbg( &( pSslContext->config ),
                          dtls_mbedtls_string_printf,
                          NULL );
    #endif /* MBEDTLS_DEBUG_C */
}
/*-----------------------------------------------------------*/

static void DtlsSslContextFree( DtlsSSLContext_t * pSslContext )
{
    configASSERT( pSslContext != NULL );

    mbedtls_ssl_config_free( &( pSslContext->config ) );
    mbedtls_ssl_free( &( pSslContext->context ) );
    mbedtls_entropy_free( &( pSslContext->entropyContext ) );
    mbedtls_ctr_drbg_free( &( pSslContext->ctrDrbgContext ) );
}
/*-----------------------------------------------------------*/
#if (MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100)
int dtlsSessionKeyDerivationCallback( void * customData,
                                       mbedtls_ssl_key_export_type type,
                                       const unsigned char * pMasterSecret,
                                       size_t secret_len,
                                       const unsigned char clientRandom[MAX_DTLS_RANDOM_BYTES_LEN],
                                       const unsigned char serverRandom[MAX_DTLS_RANDOM_BYTES_LEN],
                                       mbedtls_tls_prf_types tlsProfile )
{
    ( void ) type;
    ( void ) secret_len;
    DtlsSSLContext_t * pSslContext = ( DtlsSSLContext_t * )customData;
    TlsKeys * pKeys = ( TlsKeys * ) &pSslContext->tlsKeys;

    memcpy( pKeys->masterSecret, pMasterSecret, sizeof( pKeys->masterSecret ) );
    memcpy( pKeys->randBytes, clientRandom, MAX_DTLS_RANDOM_BYTES_LEN );
    memcpy( pKeys->randBytes + MAX_DTLS_RANDOM_BYTES_LEN, serverRandom, MAX_DTLS_RANDOM_BYTES_LEN );
    pKeys->tlsProfile = tlsProfile;

    return 0;
}
#else
int dtlsSessionKeyDerivationCallback( void * customData,
                                      const unsigned char * pMasterSecret,
                                      const unsigned char * pKeyBlock,
                                      size_t maclen,
                                      size_t keylen,
                                      size_t ivlen,
                                      const unsigned char clientRandom[MAX_DTLS_RANDOM_BYTES_LEN],
                                      const unsigned char serverRandom[MAX_DTLS_RANDOM_BYTES_LEN],
                                      mbedtls_tls_prf_types tlsProfile )
{
    ( ( void ) pKeyBlock );
    ( ( void ) maclen );
    ( ( void  )keylen );
    ( ( void ) ivlen );
    DtlsSSLContext_t * pSslContext = ( DtlsSSLContext_t * )customData;
    TlsKeys * pKeys = ( TlsKeys * ) &pSslContext->tlsKeys;

    memcpy( pKeys->masterSecret,
            pMasterSecret,
            sizeof( pKeys->masterSecret ) );
    memcpy( pKeys->randBytes,
            clientRandom,
            MAX_DTLS_RANDOM_BYTES_LEN );
    memcpy( pKeys->randBytes + MAX_DTLS_RANDOM_BYTES_LEN,
            serverRandom,
            MAX_DTLS_RANDOM_BYTES_LEN );
    pKeys->tlsProfile = tlsProfile;

    return 0;
}
#endif
/*-----------------------------------------------------------*/
static int32_t setCredentials( DtlsSSLContext_t * pSslContext,
                               DtlsNetworkCredentials_t * pNetworkCredentials )
{
    int32_t mbedtlsError = 0;

    configASSERT( pSslContext != NULL );
    configASSERT( pNetworkCredentials != NULL );

    /* Set up the certificate security profile, starting from the default value.
     */
    pSslContext->certProfile = mbedtls_x509_crt_profile_default;

    /* Set SSL authmode and the RNG context. */
    mbedtls_ssl_conf_authmode( &( pSslContext->config ),
                               MBEDTLS_SSL_VERIFY_OPTIONAL );
    mbedtls_ssl_conf_rng( &( pSslContext->config ),
                          mbedtls_ctr_drbg_random,
                          &( pSslContext->ctrDrbgContext ) );
    mbedtls_ssl_conf_cert_profile( &( pSslContext->config ),
                                   &( pSslContext->certProfile ) );

    if( pNetworkCredentials->pClientCert != NULL )
    {
        if( pNetworkCredentials->pPrivateKey != NULL )
        {
            if( mbedtlsError == 0 )
            {
                mbedtlsError = mbedtls_ssl_conf_own_cert( &( pSslContext->config ),
                                                          pNetworkCredentials->pClientCert,
                                                          pNetworkCredentials->pPrivateKey );
            }

            if( mbedtlsError == 0 )
            {
                mbedtls_ssl_conf_dtls_cookies( &( pSslContext->config ),
                                               NULL,
                                               NULL,
                                               NULL );
            }

            if( mbedtlsError == 0 )
            {
#if (MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100)
                mbedtlsError = mbedtls_ssl_conf_dtls_srtp_protection_profiles( &pSslContext->config,
                                                                            DTLS_SRTP_SUPPORTED_PROFILES );
#else
                mbedtlsError = mbedtls_ssl_conf_dtls_srtp_protection_profiles( &pSslContext->config,
                                                                               DTLS_SRTP_SUPPORTED_PROFILES,
                                                                               ARRAY_SIZE( DTLS_SRTP_SUPPORTED_PROFILES ) );
#endif
                if( mbedtlsError != 0 )
                {
                    LogError( ( "mbedtls_ssl_conf_dtls_srtp_protection_profiles failed" ) );
                    MBEDTLS_ERROR_DESCRIPTION( mbedtlsError );
                }
            }
            if( mbedtlsError == 0 )
            {
#if (MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100)
                mbedtls_ssl_set_export_keys_cb( &pSslContext->context,
                                                 (mbedtls_ssl_export_keys_t *)dtlsSessionKeyDerivationCallback,
                                                 pSslContext );
#else
                mbedtls_ssl_conf_export_keys_ext_cb( &pSslContext->config,
                                                     dtlsSessionKeyDerivationCallback,
                                                     pSslContext );
#endif
            }
        }
        else
        {
            LogError( ( "pNetworkCredentials->pPrivateKey == NULL" ) );
            mbedtlsError = -1;
        }
    }
    else
    {
        LogError( ( "pNetworkCredentials->pClientCert == NULL" ) );
        mbedtlsError = -1;
    }

    return mbedtlsError;
}
/*-----------------------------------------------------------*/

static DtlsTransportStatus_t dtlsSetup( DtlsNetworkContext_t * pDtlsNetworkContext,
                                        DtlsNetworkCredentials_t * pNetworkCredentials,
                                        uint8_t isServer )
{
    DtlsTransportParams_t * pDtlsTransportParams = NULL;
    DtlsTransportStatus_t returnStatus = DTLS_SUCCESS;
    int32_t mbedtlsError = 0;

    configASSERT( pDtlsNetworkContext != NULL );
    configASSERT( pDtlsNetworkContext->pParams != NULL );
    configASSERT( pNetworkCredentials != NULL );
    // configASSERT( pNetworkCredentials->pRootCa != NULL );

    pDtlsTransportParams = pDtlsNetworkContext->pParams;
    /* Initialize the mbed DTLS context structures. */
    DtlsSslContextInit( &( pDtlsTransportParams->dtlsSslContext ) );

    mbedtlsError = mbedtls_ssl_config_defaults( &( pDtlsTransportParams->dtlsSslContext.config ),
                                                isServer? MBEDTLS_SSL_IS_SERVER : MBEDTLS_SSL_IS_CLIENT,
                                                MBEDTLS_SSL_TRANSPORT_DATAGRAM,
                                                MBEDTLS_SSL_PRESET_DEFAULT );

    if( mbedtlsError != 0 )
    {
        LogError( ( "Failed to set default SSL configuration: mbedTLSError= %s : %s.", mbedtlsHighLevelCodeOrDefault( mbedtlsError ), mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );

        /* Per mbed DTLS docs, mbedtls_ssl_config_defaults only fails on memory
         * allocation. */
        returnStatus = DTLS_TRANSPORT_INSUFFICIENT_MEMORY;
    }

    if( returnStatus == DTLS_SUCCESS )
    {
        mbedtlsError = setCredentials( &( pDtlsTransportParams->dtlsSslContext ),
                                       pNetworkCredentials );

        if( mbedtlsError != 0 )
        {
            returnStatus = DTLS_TRANSPORT_INVALID_CREDENTIALS;
        }
    }

    if( returnStatus == DTLS_SUCCESS )
    {
        pDtlsTransportParams = pDtlsNetworkContext->pParams;

        /* Initialize the mbed DTLS secured connection context. */
        mbedtlsError = mbedtls_ssl_setup( &( pDtlsTransportParams->dtlsSslContext.context ),
                                          &( pDtlsTransportParams->dtlsSslContext.config ) );
        if( mbedtlsError != 0 )
        {
            LogError( ( "Failed to set up mbed DTLS SSL context: mbedTLSError=-0x%lx %s : %s.",
                        mbedtlsError,
                        mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                        mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
            returnStatus = DTLS_TRANSPORT_INTERNAL_ERROR;
        }
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

static DtlsTransportStatus_t initMbedtls( mbedtls_entropy_context * pEntropyContext,
                                          mbedtls_ctr_drbg_context * pCtrDrbgContext )
{
    DtlsTransportStatus_t returnStatus = DTLS_SUCCESS;
    int32_t mbedtlsError = 0;

    #if defined( MBEDTLS_THREADING_ALT )
    /* Set the mutex functions for mbed DTLS thread safety. */
    mbedtls_platform_threading_init();
    #endif

    /* Initialize contexts for random number generation. */
    mbedtls_entropy_init( pEntropyContext );
    mbedtls_ctr_drbg_init( pCtrDrbgContext );

    if( mbedtlsError != 0 )
    {
        LogError( ( "Failed to add entropy source: mbedTLSError= %s : %s.", mbedtlsHighLevelCodeOrDefault( mbedtlsError ), mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
        returnStatus = DTLS_TRANSPORT_INTERNAL_ERROR;
    }

#ifdef MBEDTLS_PSA_CRYPTO_C
    if( returnStatus == DTLS_SUCCESS )
    {
        mbedtlsError = psa_crypto_init();

        if( mbedtlsError != PSA_SUCCESS )
        {
            LogError( ( "Failed to initialize PSA Crypto implementation: %s", ( int )mbedtlsError ) );
            returnStatus = DTLS_TRANSPORT_INTERNAL_ERROR;
        }
    }
#endif /* MBEDTLS_PSA_CRYPTO_C */

    if( returnStatus == DTLS_SUCCESS )
    {
        /* Seed the random number generator. */
        mbedtlsError = mbedtls_ctr_drbg_seed( pCtrDrbgContext,
                                              mbedtls_entropy_func,
                                              pEntropyContext,
                                              NULL,
                                              0 );

        if( mbedtlsError != 0 )
        {
            LogError( ( "Failed to seed PRNG: mbedTLSError= %s : %s.", mbedtlsHighLevelCodeOrDefault( mbedtlsError ), mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
            returnStatus = DTLS_TRANSPORT_INTERNAL_ERROR;
        }
    }

    if( returnStatus == DTLS_SUCCESS )
    {
        LogDebug( ( "Successfully initialized mbedTLS." ) );
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

void DTLS_Disconnect( DtlsNetworkContext_t * pNetworkContext )
{
    DtlsTransportParams_t * pDtlsTransportParams = NULL;
    BaseType_t dtlsStatus = 0;

    if( ( pNetworkContext != NULL ) && ( pNetworkContext->pParams != NULL ) )
    {
        pDtlsTransportParams = pNetworkContext->pParams;
        /* Attempting to terminate DTLS connection. */
        dtlsStatus = ( BaseType_t )mbedtls_ssl_close_notify( &( pDtlsTransportParams->dtlsSslContext.context ) );

        /* Ignore the WANT_READ and WANT_WRITE return values. */
        if( ( dtlsStatus != ( BaseType_t )MBEDTLS_ERR_SSL_WANT_READ ) && ( dtlsStatus != ( BaseType_t )MBEDTLS_ERR_SSL_WANT_WRITE ) )
        {
            if( dtlsStatus == 0 )
            {
                LogDebug( ( "(Network connection %p) DTLS close-notify sent.", pNetworkContext ) );
            }
            else
            {
                LogError( ( "(Network connection %p) Failed to send DTLS close-notify: "
                            "mbedTLSError= %s : %s.",
                            pNetworkContext,
                            mbedtlsHighLevelCodeOrDefault( dtlsStatus ),
                            mbedtlsLowLevelCodeOrDefault( dtlsStatus ) ) );
            }
        }
        else
        {
            /* WANT_READ and WANT_WRITE can be ignored. Logging for debugging purposes. */
            LogDebug( ( "(Network connection %p) DTLS close-notify sent; "
                        "received %s as the DTLS status can be ignored for close-notify.",
                        pNetworkContext,
                        ( dtlsStatus == MBEDTLS_ERR_SSL_WANT_READ ) ? "WANT_READ" : "WANT_WRITE" ) );
        }

        /* Free mbed DTLS contexts. */
        DtlsSslContextFree( &( pDtlsTransportParams->dtlsSslContext ) );
    }
}
/*-----------------------------------------------------------*/

int32_t DTLS_Send( DtlsNetworkContext_t * pNetworkContext,
                   const void * pBuffer,
                   size_t bytesToSend )
{
    DtlsTransportParams_t * pDtlsTransportParams = NULL;
    int32_t dtlsStatus = 0;

    if( ( pNetworkContext == NULL ) || ( pNetworkContext->pParams == NULL ) )
    {
        LogError( ( "invalid input, pNetworkContext=%p", pNetworkContext ) );
        dtlsStatus = -1;
    }
    else if( pBuffer == NULL )
    {
        LogError( ( "invalid input, pBuffer == NULL" ) );
        dtlsStatus = -1;
    }
    else if( bytesToSend == 0 )
    {
        LogError( ( "invalid input, bytesToSend == 0" ) );
        dtlsStatus = -1;
    }
    else
    {
        pDtlsTransportParams = pNetworkContext->pParams;

        dtlsStatus = ( int32_t ) mbedtls_ssl_write( &( pDtlsTransportParams->dtlsSslContext.context ),
                                                    pBuffer,
                                                    bytesToSend );

        if( ( dtlsStatus == MBEDTLS_ERR_SSL_TIMEOUT ) || ( dtlsStatus == MBEDTLS_ERR_SSL_WANT_READ ) || ( dtlsStatus == MBEDTLS_ERR_SSL_WANT_WRITE ) )
        {
            LogDebug( ( "Failed to send data. However, send can be retried on "
                        "this error. "
                        "mbedTLSError= %s : %s.",
                        mbedtlsHighLevelCodeOrDefault( dtlsStatus ),
                        mbedtlsLowLevelCodeOrDefault( dtlsStatus ) ) );

            /* Mark these set of errors as a timeout. The libraries may retry
             * send on these errors. */
            dtlsStatus = 0;
        }
        else if( dtlsStatus < 0 )
        {
            LogError( ( "Failed to send DTLS data: -0x%lx mbedTLSError= %s : %s.", -dtlsStatus, mbedtlsHighLevelCodeOrDefault( dtlsStatus ), mbedtlsLowLevelCodeOrDefault( dtlsStatus ) ) );
        }
        else
        {
            /* Empty else marker. */
        }
    }

    return dtlsStatus;
}
/*-----------------------------------------------------------*/

int32_t dtlsFillPseudoRandomBits( uint8_t * pBuf,
                                  size_t bufSize )
{
    int32_t retStatus = 0;
    uint32_t i;

    if( ( bufSize >= DTLS_CERT_MIN_SERIAL_NUM_SIZE ) && ( bufSize <= DTLS_CERT_MAX_SERIAL_NUM_SIZE ) )
    {
        if( pBuf != NULL )
        {
            for( i = 0; i < bufSize; i++ )
            {
                *pBuf++ = ( uint8_t )( rand() & 0xFF );
            }
        }
        else
        {
            retStatus = DTLS_INVALID_PARAMETER;
        }
    }
    else
    {
        retStatus = DTLS_INVALID_PARAMETER;
        LogError( ( "invalid input, bufSize >= DTLS_CERT_MIN_SERIAL_NUM_SIZE && "
                    "bufSize <= DTLS_CERT_MAX_SERIAL_NUM_SIZE " ) );
    }
    return retStatus;
}
/*-----------------------------------------------------------*/

int32_t DTLS_CreateCertificateFingerprint( const mbedtls_x509_crt * pCert,
                                           char * pBuff,
                                           const size_t bufLen )
{
    int32_t retStatus = 0;
    uint8_t fingerprint[MBEDTLS_MD_MAX_SIZE];
    int32_t sslRet, i, size;
    // const is not pure C, but mbedtls_md_info_from_type requires the param to
    // be const
    const mbedtls_md_info_t * pMdInfo;

    if( pBuff == NULL )
    {
        LogError( ( "invalid input, pBuff == NULL " ) );
        retStatus = -1;
    }
    else
    {
        /* Empty else marker. */
    }

    if( retStatus == 0 )
    {
        pMdInfo = mbedtls_md_info_from_type( MBEDTLS_MD_SHA256 );
        if( pMdInfo == NULL )
        {
            LogError( ( "invalid input, pMdInfo == NULL " ) );
            retStatus = -1;
        }
    }

    if( retStatus == 0 )
    {
        sslRet = mbedtls_sha256_ret( pCert->raw.p,
                                     pCert->raw.len,
                                     fingerprint,
                                     0 );
        if( sslRet != 0 )
        {
            LogError( ( "Failed to calculate the SHA-256 checksum: mbedTLSError= %s : %s.", mbedtlsHighLevelCodeOrDefault( sslRet ), mbedtlsLowLevelCodeOrDefault( sslRet ) ) );
            retStatus = -1;
        }
    }

    if( retStatus == 0 )
    {
        size = mbedtls_md_get_size( pMdInfo );

        if( bufLen < 3 * size )
        {
            LogError( ( "buffer to store fingerprint too small buffer: %i size: %li", bufLen, size ) );
            retStatus = -1;
        }
    }

    if( retStatus == 0 )
    {
        for( i = 0; i < size; i++ )
        {
            sprintf( pBuff,
                     "%.2X:",
                     fingerprint[i] );
            pBuff += 3;
        }
        *( pBuff - 1 ) = '\0';
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

int32_t DTLS_GetLocalCertificateFingerprint( DtlsSSLContext_t * pSslContext,
                                             void * pBuff,
                                             size_t buffLen )
{
    int32_t retStatus = 0;

    if( ( pSslContext == NULL ) || ( pBuff == NULL ) )
    {
        LogError( ( "invalid input, pSslContext || pBuff == NULL " ) );
        retStatus = -1;
    }
    else if( buffLen < CERTIFICATE_FINGERPRINT_LENGTH )
    {
        LogError( ( "buffLen < CERTIFICATE_FINGERPRINT_LENGTH " ) );
        retStatus = -1;
    }
    else
    {
        DTLS_CreateCertificateFingerprint( &pSslContext->clientCert,
                                           pBuff,
                                           buffLen );
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

int32_t DTLS_VerifyRemoteCertificateFingerprint( DtlsSSLContext_t * pSslContext,
                                                 char * pExpectedFingerprint,
                                                 const size_t fingerprintMaxLen )
{
    int32_t retStatus = 0;
    char actualFingerprint[ CERTIFICATE_FINGERPRINT_LENGTH ];
    mbedtls_x509_crt * pRemoteCertificate = NULL;

    if( ( pSslContext == NULL ) || ( pExpectedFingerprint == NULL ) || ( CERTIFICATE_FINGERPRINT_LENGTH < fingerprintMaxLen ) )
    {
        LogError( ( "invalid input, pSslContext || pExpectedFingerprint == NULL || CERTIFICATE_FINGERPRINT_LENGTH < fingerprintMaxLen(%u)", fingerprintMaxLen ) );
        retStatus = -1;
    }

    if( retStatus == 0 )
    {
        pRemoteCertificate = ( mbedtls_x509_crt * )mbedtls_ssl_get_peer_cert( &pSslContext->context );
        if( pRemoteCertificate == NULL )
        {
            LogError( ( "pRemoteCertificate == NULL" ) );
            retStatus = -1;
        }
    }

    if( retStatus == 0 )
    {
        if( DTLS_CreateCertificateFingerprint( pRemoteCertificate,
                                               actualFingerprint,
                                               CERTIFICATE_FINGERPRINT_LENGTH ) != 0 )
        {
            LogError( ( "Failed to calculate certificate fingerprint" ) );
            retStatus = -1;
        }
    }

    if( retStatus == 0 )
    {
        if( strncmp( pExpectedFingerprint,
                     actualFingerprint,
                     fingerprintMaxLen ) != 0 )
        {
            LogError( ( "DTLS_SSL_REMOTE_CERTIFICATE_VERIFICATION_FAILED \nexpected fingerprint:\n %s \nactual fingerprint:\n %s", pExpectedFingerprint, actualFingerprint ) );
            retStatus = DTLS_SSL_REMOTE_CERTIFICATE_VERIFICATION_FAILED;
        }
    }

    return retStatus;
}
/*-----------------------------------------------------------*/

int32_t DTLS_PopulateKeyingMaterial( DtlsSSLContext_t * pSslContext,
                                     pDtlsKeyingMaterial_t pDtlsKeyingMaterial )
{
    int32_t retStatus = 0;
    uint32_t offset = 0;

    TlsKeys * pKeys = NULL;
    uint8_t keyingMaterialBuffer[MAX_SRTP_MASTER_KEY_LEN * 2 + MAX_SRTP_SALT_KEY_LEN * 2];
#if ( MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100 )
    mbedtls_dtls_srtp_info negotiatedSRTPProfile;
#else /* #if ( MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100 ) */
    mbedtls_ssl_srtp_profile negotiatedSRTPProfile;
#endif /* #if ( MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100 ) */

    if( ( pSslContext == NULL ) || ( pDtlsKeyingMaterial == NULL ) )
    {
        LogError( ( "invalid input, pSslContext || pDtlsKeyingMaterial == NULL " ) );
        retStatus = -1;
    }
    else
    {
        /* Empty else marker. */
    }

    if( retStatus == 0 )
    {
        pKeys = ( TlsKeys * ) &pSslContext->tlsKeys;

        // https://mbed-tls.readthedocs.io/en/latest/kb/how-to/tls_prf/
        retStatus = mbedtls_ssl_tls_prf( pKeys->tlsProfile,
                                         pKeys->masterSecret,
                                         ARRAY_SIZE( pKeys->masterSecret ),
                                         KEYING_EXTRACTOR_LABEL,
                                         pKeys->randBytes,
                                         ARRAY_SIZE( pKeys->randBytes ),
                                         keyingMaterialBuffer,
                                         ARRAY_SIZE( keyingMaterialBuffer ) );
        if( retStatus != 0 )
        {
            LogError( ( "Failed TLS-PRF function for key derivation, funct: %d", pKeys->tlsProfile ) );
            MBEDTLS_ERROR_DESCRIPTION( retStatus );
            retStatus = -1;
        }
        else
        {
            /* Empty else marker. */
        }
    }

    if( retStatus == 0 )
    {
        pDtlsKeyingMaterial->key_length = MAX_SRTP_MASTER_KEY_LEN + MAX_SRTP_SALT_KEY_LEN;

        memcpy( pDtlsKeyingMaterial->clientWriteKey,
                &keyingMaterialBuffer[offset],
                MAX_SRTP_MASTER_KEY_LEN );
        offset += MAX_SRTP_MASTER_KEY_LEN;

        memcpy( pDtlsKeyingMaterial->serverWriteKey,
                &keyingMaterialBuffer[offset],
                MAX_SRTP_MASTER_KEY_LEN );
        offset += MAX_SRTP_MASTER_KEY_LEN;

        memcpy( pDtlsKeyingMaterial->clientWriteKey + MAX_SRTP_MASTER_KEY_LEN,
                &keyingMaterialBuffer[offset],
                MAX_SRTP_SALT_KEY_LEN );
        offset += MAX_SRTP_SALT_KEY_LEN;

        memcpy( pDtlsKeyingMaterial->serverWriteKey + MAX_SRTP_MASTER_KEY_LEN,
                &keyingMaterialBuffer[offset],
                MAX_SRTP_SALT_KEY_LEN );

#if ( MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100 )
        mbedtls_ssl_get_dtls_srtp_negotiation_result( &pSslContext->context, &negotiatedSRTPProfile );
        switch( negotiatedSRTPProfile.chosen_dtls_srtp_profile )
        {
            case MBEDTLS_TLS_SRTP_AES128_CM_HMAC_SHA1_80:
#else /* #if ( MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100 ) */
        negotiatedSRTPProfile = mbedtls_ssl_get_dtls_srtp_protection_profile( &pSslContext->context );
        switch( negotiatedSRTPProfile )
        {
            case MBEDTLS_SRTP_AES128_CM_HMAC_SHA1_80:
#endif /* #if ( MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100 ) */
                pDtlsKeyingMaterial->srtpProfile = KVS_SRTP_PROFILE_AES128_CM_HMAC_SHA1_80;
                break;

#if ( MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100 )
            case MBEDTLS_TLS_SRTP_AES128_CM_HMAC_SHA1_32:
#else /* #if ( MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100 ) */
            case MBEDTLS_SRTP_AES128_CM_HMAC_SHA1_32:
#endif /* #if ( MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100 ) */
                pDtlsKeyingMaterial->srtpProfile = KVS_SRTP_PROFILE_AES128_CM_HMAC_SHA1_32;
                break;
            default:
                LogError( ( "DTLS_SSL_UNKNOWN_SRTP_PROFILE" ) );
                retStatus = DTLS_SSL_UNKNOWN_SRTP_PROFILE;
        }
    }
    return retStatus;
}
/*-----------------------------------------------------------*/

/**
 * DTLS_CreateCertificateAndKey generates a new certificate and a key
 * If generateRSACertificate is true, RSA is going to be used for the key
 * generation. Otherwise, ECDSA is going to be used. certificateBits is only
 * being used when generateRSACertificate is true.
 */
int32_t DTLS_CreateCertificateAndKey( int32_t certificateBits,
                                      BaseType_t generateRSACertificate,
                                      mbedtls_x509_crt * pCert,
                                      mbedtls_pk_context * pKey )
{
    int32_t retStatus = 0;
    BaseType_t initialized = pdFALSE;
    char * pCertBuf = NULL;
    char notBeforeBuf[MBEDTLS_X509_RFC5280_UTC_TIME_LEN + 1], notAfterBuf[MBEDTLS_X509_RFC5280_UTC_TIME_LEN + 1];
    // TODO RTC needs to be solved: uint64_t now, notAfter;
    int32_t len;
    mbedtls_entropy_context * pEntropy = NULL;
    mbedtls_ctr_drbg_context * pCtrDrbg = NULL;
    mbedtls_mpi serial;
    mbedtls_x509write_cert * pWriteCert = NULL;
    uint8_t certSn[DTLS_CERT_MAX_SERIAL_NUM_SIZE];

    if( ( pCert != NULL ) && ( pKey != NULL ) )
    {
        if( ( pCertBuf = ( char * )pvPortMalloc( GENERATED_CERTIFICATE_MAX_SIZE ) ) )
        {
            if( ( NULL != ( pEntropy = ( mbedtls_entropy_context * )pvPortMalloc( sizeof( mbedtls_entropy_context ) ) ) ) )
            {
                if( ( NULL != ( pCtrDrbg = ( mbedtls_ctr_drbg_context * )pvPortMalloc( sizeof( mbedtls_ctr_drbg_context ) ) ) ) )
                {
                    if( ( NULL != ( pWriteCert = ( mbedtls_x509write_cert * )pvPortMalloc( sizeof( mbedtls_x509write_cert ) ) ) ) )
                    {
                        if( dtlsFillPseudoRandomBits( certSn,
                                                      sizeof( certSn ) ) == 0 )
                        {
                            // initialize to sane values
                            mbedtls_entropy_init( pEntropy );
                            mbedtls_ctr_drbg_init( pCtrDrbg );
                            mbedtls_mpi_init( &serial );
                            mbedtls_x509write_crt_init( pWriteCert );
                            mbedtls_x509_crt_init( pCert );
                            mbedtls_pk_init( pKey );
                            initialized = pdTRUE;
                            if( mbedtls_ctr_drbg_seed( pCtrDrbg,
                                                       mbedtls_entropy_func,
                                                       pEntropy,
                                                       NULL,
                                                       0 ) == 0 )
                            {
                                LogDebug( ( "mbedtls_ctr_drbg_seed successful" ) );

                                // generate a RSA key
                                if( generateRSACertificate )
                                {
                                    LogWarn( ( "generateRSACertificate this will take about 10mins" ) );

                                    if( mbedtls_pk_setup( pKey,
                                                          mbedtls_pk_info_from_type( MBEDTLS_PK_RSA ) ) == 0 )
                                    {
                                        LogDebug( ( "mbedtls_pk_setup successful" ) );
                                        if( mbedtls_rsa_gen_key( mbedtls_pk_rsa( *pKey ),
                                                                 mbedtls_ctr_drbg_random,
                                                                 pCtrDrbg,
                                                                 certificateBits,
                                                                 DTLS_RSA_F4 ) == 0 )
                                        {
                                            LogDebug( ( "mbedtls_rsa_gen_key successful" ) );
                                        }
                                        else
                                        {
                                            retStatus = DTLS_GENERATE_KEY_FAILURE;
                                            LogError( ( "mbedtls_rsa_gen_key failed" ) );
                                        }
                                    }
                                    else
                                    {
                                        retStatus = DTLS_INITIALIZE_PK_FAILURE;
                                        LogError( ( "mbedtls_pk_setup DTLS_INITIALIZE_PK_FAILURE" ) );
                                    }
                                }
                                else     // generate ECDSA
                                {

                                    if( ( mbedtls_pk_setup( pKey,
                                                            mbedtls_pk_info_from_type( MBEDTLS_PK_ECKEY ) ) == 0 ) &&
                                        ( mbedtls_ecp_gen_key( MBEDTLS_ECP_DP_SECP256R1,
                                                               mbedtls_pk_ec( *pKey ),
                                                               mbedtls_ctr_drbg_random,
                                                               pCtrDrbg ) == 0 ) )
                                    {
                                        LogDebug( ( "mbedtls_pk_setup && mbedtls_ecp_gen_key successful" ) );
                                    }
                                    else
                                    {
                                        retStatus = DTLS_GENERATE_KEY_FAILURE;
                                        LogError( ( "mbedtls_pk_setup or mbedtls_ecp_gen_key failed" ) );
                                    }
                                }
                            }

                            // generate a new certificate
                            int mbedtlsRet = mbedtls_mpi_read_binary( &serial,
                                                                      certSn,
                                                                      sizeof( certSn ) );
                            if( mbedtlsRet == 0 )
                            {
                                LogDebug( ( "mbedtls_mpi_read_binary successful" ) );
                                struct timespec nowTime;
                                time_t timeT;
                                clock_gettime( CLOCK_REALTIME,
                                               &nowTime );
                                timeT = nowTime.tv_sec;

                                if( strftime( notBeforeBuf,
                                              sizeof( notBeforeBuf ),
                                              "%Y%m%d%H%M%S",
                                              gmtime( &timeT ) ) != MBEDTLS_X509_RFC5280_UTC_TIME_LEN )
                                {
                                    LogDebug( ( "notBefore: %s", notBeforeBuf ) );

                                    timeT = nowTime.tv_sec + ( GENERATED_CERTIFICATE_DAYS * DTLS_SECONDS_IN_A_DAY );
                                    if( strftime( notAfterBuf,
                                                  sizeof( notAfterBuf ),
                                                  "%Y%m%d%H%M%S",
                                                  gmtime( &timeT ) ) != MBEDTLS_X509_RFC5280_UTC_TIME_LEN )
                                    {
                                        LogDebug( ( "notAfter: %s", notAfterBuf ) );

                                        if( mbedtls_x509write_crt_set_serial( pWriteCert,
                                                                              &serial ) == 0 )
                                        {
                                            if( mbedtls_x509write_crt_set_validity( pWriteCert,
                                                                                    notBeforeBuf,
                                                                                    notAfterBuf ) == 0 )
                                            {
                                                if( mbedtls_x509write_crt_set_subject_name( pWriteCert,
                                                                                            "O"
                                                                                            "=" GENERATED_CERTIFICATE_NAME ",CN"
                                                                                                                           "=" GENERATED_CERTIFICATE_NAME ) == 0 )
                                                {
                                                    if( mbedtls_x509write_crt_set_issuer_name( pWriteCert,
                                                                                               "O"
                                                                                               "=" GENERATED_CERTIFICATE_NAME ",CN"
                                                                                                                              "=" GENERATED_CERTIFICATE_NAME ) != 0 )
                                                    {
                                                        retStatus = DTLS_SET_CERT_ISSUER_NAME_FAILURE;
                                                        LogError( ( "mbedtls_x509write_crt_set_issuer_name failed" ) );
                                                    }
                                                }
                                            }
                                            else
                                            {
                                                retStatus = DTLS_SET_CERT_VALIDITY_FAILURE;
                                                LogError( ( "mbedtls_x509write_crt_set_validity failed" ) );
                                            }

                                            // void functions, it must succeed
                                            mbedtls_x509write_crt_set_version( pWriteCert,
                                                                               MBEDTLS_X509_CRT_VERSION_3 );
                                            mbedtls_x509write_crt_set_subject_key( pWriteCert,
                                                                                   pKey );
                                            mbedtls_x509write_crt_set_issuer_key( pWriteCert,
                                                                                  pKey );
                                            mbedtls_x509write_crt_set_md_alg( pWriteCert,
                                                                              MBEDTLS_MD_SHA1 );

                                            memset( pCertBuf,
                                                    0,
                                                    GENERATED_CERTIFICATE_MAX_SIZE );
                                            len = mbedtls_x509write_crt_der( pWriteCert,
                                                                             ( void * )pCertBuf,
                                                                             GENERATED_CERTIFICATE_MAX_SIZE,
                                                                             mbedtls_ctr_drbg_random,
                                                                             pCtrDrbg );
                                            LogDebug( ( "mbedtls_x509write_crt_der, len: %li", len ) );
                                            if( len <= 0 )
                                            {
                                                retStatus = DTLS_WRITE_CERT_CRT_DER_FAILURE;
                                                LogError( ( "mbedtls_x509write_crt_der failed" ) );
                                            }

                                            // mbedtls_x509write_crt_der starts
                                            // writing from behind, so we need to
                                            // use the return len to figure out
                                            // where the data actually starts:
                                            //
                                            //         -----------------------------------------
                                            //         |  padding      | certificate
                                            //         |
                                            //         -----------------------------------------
                                            //         ^               ^
                                            //       pCertBuf   pCertBuf +
                                            //       (sizeof(pCertBuf) - len)
                                            if( mbedtls_x509_crt_parse_der( pCert,
                                                                            ( void * )( pCertBuf + GENERATED_CERTIFICATE_MAX_SIZE - len ),
                                                                            len ) != 0 )
                                            {
                                                retStatus = DTLS_PARSE_CERT_DER_FAILURE;
                                                LogError( ( "mbedtls_x509_crt_parse_der failed" ) );
                                            }
                                        }
                                        else
                                        {
                                            retStatus = DTLS_SET_CERT_SERIAL_FAILURE;
                                            LogError( ( "mbedtls_x509write_crt_set_serial failed" ) );
                                        }
                                    }
                                    else
                                    {
                                        retStatus = DTLS_GENERATE_TIMESTAMP_STRING_FAILURE;
                                        LogError( ( "generateTimestampStr failed" ) );
                                    }
                                }
                                else
                                {
                                    retStatus = DTLS_GENERATE_TIMESTAMP_STRING_FAILURE;
                                    LogError( ( "generateTimestampStr failed" ) );
                                }
                            }
                            else
                            {
                                retStatus = DTLS_READ_BINARY_FAILURE;
                                LogError( ( "mbedtls_mpi_read_binary failed, ret: %d", mbedtlsRet ) );
                                MBEDTLS_ERROR_DESCRIPTION( mbedtlsRet );
                            }
                        }
                        else
                        {
                            retStatus = DTLS_GENERATE_RANDOM_BITS_FAILURE;
                            LogError( ( "dtlsFillPseudoRandomBits failed" ) );
                        }
                    }
                    else
                    {
                        retStatus = DTLS_OUT_OF_MEMORY;
                        LogError( ( "mbedtls_x509write_cert alloc failed" ) );
                    }
                }
                else
                {
                    retStatus = DTLS_OUT_OF_MEMORY;
                    LogError( ( "mbedtls_ctr_drbg_context alloc failed" ) );
                }
            }
            else
            {
                retStatus = DTLS_OUT_OF_MEMORY;
                LogError( ( "mbedtls_entropy_context alloc failed" ) );
            }
        }
        else
        {
            retStatus = DTLS_OUT_OF_MEMORY;
            LogError( ( "pCertBuf alloc failed" ) );
        }
    }
    else
    {
        LogError( ( "pCert != NULL && pKey != NULL" ) );
        retStatus = DTLS_INVALID_PARAMETER;
    }

    if( initialized && ( 0 != retStatus ) )
    {
        mbedtls_x509write_crt_free( pWriteCert );
        mbedtls_mpi_free( &serial );
        mbedtls_ctr_drbg_free( pCtrDrbg );
        mbedtls_entropy_free( pEntropy );

        if( 0 != retStatus )
        {
            DTLS_FreeCertificateAndKey( pCert,
                                        pKey );
        }
    }
    vPortFree( pCertBuf );
    vPortFree( pEntropy );
    vPortFree( pCtrDrbg );
    vPortFree( pWriteCert );

    return retStatus;
}
/*-----------------------------------------------------------*/

int32_t DTLS_FreeCertificateAndKey( mbedtls_x509_crt * pCert,
                                    mbedtls_pk_context * pKey )
{
    int32_t dtlsStatus = DTLS_SUCCESS;

    if( pCert != NULL )
    {
        mbedtls_x509_crt_free( pCert );
    }
    else
    {
        dtlsStatus = DTLS_INVALID_PARAMETER;
    }

    if( pKey != NULL )
    {
        mbedtls_pk_free( pKey );
    }
    else
    {
        dtlsStatus = DTLS_INVALID_PARAMETER;
    }

    return dtlsStatus;
}
/*-----------------------------------------------------------*/

DtlsTransportStatus_t DTLS_Init( DtlsNetworkContext_t * pNetworkContext,
                                 DtlsNetworkCredentials_t * pNetworkCredentials,
                                 uint8_t isServer )
{
    DtlsTransportStatus_t returnStatus = DTLS_SUCCESS;
    DtlsTransportParams_t * pDtlsTransportParams = NULL;

    if( ( pNetworkContext == NULL ) || ( pNetworkCredentials == NULL ) )
    {
        LogError( ( "Invalid input parameter(s): Arguments cannot be NULL. "
                    "pNetworkContext=%p, "
                    "pNetworkCredentials=%p.",
                    pNetworkContext,
                    pNetworkCredentials ) );
        returnStatus = DTLS_INVALID_PARAMETER;
    }
    else if( NULL == pNetworkCredentials->pClientCert )
    {
        LogError( ( "NULL == pNetworkCredentials->pClientCert" ) );
        returnStatus = DTLS_INVALID_PARAMETER;
    }
    else if( NULL == pNetworkCredentials->pPrivateKey )
    {
        LogError( ( "NULL == pNetworkCredentials->pPrivateKey" ) );
        returnStatus = DTLS_INVALID_PARAMETER;
    }
    else if( pNetworkContext->pParams == NULL )
    {
        LogError( ( "pNetworkContext->pParams == NULL" ) );
        returnStatus = DTLS_INVALID_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    /* Initialize mbedtls. */
    if( returnStatus == DTLS_SUCCESS )
    {
        pNetworkContext->state = DTLS_STATE_NEW;
        pDtlsTransportParams = pNetworkContext->pParams;

        returnStatus = initMbedtls( &( pDtlsTransportParams->dtlsSslContext.entropyContext ),
                                    &( pDtlsTransportParams->dtlsSslContext.ctrDrbgContext ) );
    }

    /* Initialize DTLS contexts and set credentials. */
    if( returnStatus == DTLS_SUCCESS )
    {
        returnStatus = dtlsSetup( pNetworkContext,
                                  pNetworkCredentials,
                                  isServer );
    }

    if( returnStatus == DTLS_SUCCESS )
    {
        memset( &pNetworkContext->pParams->mbedtlsTimer,
                0,
                sizeof( mbedtls_timing_delay_context ) );

        /* Set the timer functions for mbed DTLS. */
        mbedtls_ssl_set_timer_cb( &pNetworkContext->pParams->dtlsSslContext.context,
                                  &pNetworkContext->pParams->mbedtlsTimer,
                                  &mbedtls_timing_set_delay,
                                  &mbedtls_timing_get_delay );

        /* Set the bio functions provided by user. */
        mbedtls_ssl_set_bio( &( pDtlsTransportParams->dtlsSslContext.context ),
                             ( void * ) pDtlsTransportParams,
                             DtlsUdpSendWrap,
                             DtlsUdpRecvWrap,
                             NULL );
    }

    if( returnStatus != DTLS_SUCCESS )
    {
        LogWarn( ( "Fail to initialize DTLS Context: %p with return: %d", pNetworkContext, returnStatus ) );
    }
    else
    {
        pNetworkContext->state = DTLS_STATE_HANDSHAKING;
        LogInfo( ( "Initialized DTLS Context: %p successfully.", pNetworkContext ) );
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

DtlsTransportStatus_t DTLS_ProcessPacket( DtlsNetworkContext_t * pNetworkContext,
                                          void * pDtlsPacket,
                                          size_t dtlsPacketLength,
                                          uint8_t * readBuffer,
                                          size_t * pReadBufferSize )
{
    DtlsTransportStatus_t returnStatus = DTLS_SUCCESS;
    DtlsTransportParams_t * pDtlsTransportParams = NULL;
    int32_t mbedtlsError = MBEDTLS_ERR_SSL_WANT_READ;
    int32_t readOffset = 0;

    if( ( pNetworkContext == NULL ) || ( pDtlsPacket == NULL ) || ( readBuffer == NULL ) || ( pReadBufferSize == NULL ) )
    {
        LogError( ( "Invalid input parameter(s): Arguments cannot be NULL. "
                    "pNetworkContext=%p, pDtlsPacket=%p, readBuffer=%p, pReadBufferSize=%p.",
                    pNetworkContext,
                    pDtlsPacket,
                    readBuffer,
                    pReadBufferSize ) );
        returnStatus = DTLS_INVALID_PARAMETER;
    }

    if( returnStatus == DTLS_SUCCESS )
    {
        pDtlsTransportParams = pNetworkContext->pParams;

        /* Store the processing packet into transport params. */
        pDtlsTransportParams->pReceivedPacket = pDtlsPacket;
        pDtlsTransportParams->receivedPacketLength = dtlsPacketLength;
        pDtlsTransportParams->receivedPacketOffset = 0;

        while( mbedtlsError == MBEDTLS_ERR_SSL_WANT_READ && pDtlsTransportParams->pReceivedPacket != NULL )
        {
            /* Perform read function. Mbedtls would execute mbedtls_ssl_handshake inside if the handshake is not done. */
            mbedtlsError = mbedtls_ssl_read( &( pDtlsTransportParams->dtlsSslContext.context ),
                                             readBuffer + readOffset,
                                             *pReadBufferSize - readOffset );

            if( ( mbedtlsError == MBEDTLS_ERR_SSL_TIMEOUT ) || ( mbedtlsError == MBEDTLS_ERR_SSL_WANT_READ ) || ( mbedtlsError == MBEDTLS_ERR_SSL_WANT_WRITE ) )
            {
                LogDebug( ( "Failed to read data. However, a read can be retried on "
                            "this error. "
                            "mbedTLSError=-0x%lx %s : %s.",
                            -mbedtlsError,
                            mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                            mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
            }
            else if( mbedtlsError == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY )
            {
                LogInfo( ( "DTLS connection has been closed. mbedTLSError=-0x%lx %s : %s.",
                           -mbedtlsError,
                           mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                           mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
                returnStatus = DTLS_CONNECTION_HAS_BEEN_CLOSED;
            }
            else if( mbedtlsError < 0 )
            {
                LogError( ( "Failed to read data: mbedTLSError=-0x%lx, %s : %s.",
                            -mbedtlsError,
                            mbedtlsHighLevelCodeOrDefault( mbedtlsError ), mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
                returnStatus = DTLS_TRANSPORT_PROCESS_FAILURE;
            }
            else
            {
                readOffset += mbedtlsError;
            }
        }
    }

    if( returnStatus == DTLS_SUCCESS )
    {
        /* Update recv buffer length for user. */
        *pReadBufferSize = ( size_t ) readOffset;

        /* Check handshake status. */
        if( pNetworkContext->state == DTLS_STATE_HANDSHAKING )
        {
            /* Check if handshake is done. */
            if( pDtlsTransportParams->dtlsSslContext.context.state == MBEDTLS_SSL_HANDSHAKE_OVER )
            {
                /* Update the state to connected. */
                pNetworkContext->state = DTLS_STATE_READY;
                returnStatus = DTLS_HANDSHAKE_COMPLETE;
            }
        }
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

DtlsTransportStatus_t DTLS_ExecuteHandshake( DtlsNetworkContext_t * pNetworkContext )
{
    DtlsTransportStatus_t returnStatus = DTLS_SUCCESS;
    DtlsTransportParams_t * pDtlsTransportParams = NULL;
    int32_t mbedtlsError = 0;

    if( pNetworkContext == NULL )
    {
        LogError( ( "Invalid input parameter(s): Arguments cannot be NULL. pNetworkContext=%p.",
                    pNetworkContext ) );
        returnStatus = DTLS_INVALID_PARAMETER;
    }

    /* Check handshake status. */
    if( returnStatus == DTLS_SUCCESS )
    {
        if( pNetworkContext->state == DTLS_STATE_READY )
        {
            /* If handshake is already done, we should let user know. */
            returnStatus = DTLS_HANDSHAKE_ALREADY_COMPLETE;
        }
    }

    /* Execute DTLS handshake if that's not finished yet. */
    if( returnStatus == DTLS_SUCCESS )
    {
        pDtlsTransportParams = pNetworkContext->pParams;

        /* Continuously loop while the local side should trigger the handshake. */
        do
        {
            mbedtlsError = mbedtls_ssl_handshake( &( pDtlsTransportParams->dtlsSslContext.context ) );
        } while( mbedtlsError == MBEDTLS_ERR_SSL_WANT_WRITE );

        if( mbedtlsError == MBEDTLS_ERR_SSL_WANT_READ )
        {
            /* DTLS session is waiting for receiving data. */
        }
        else if( mbedtlsError < 0 )
        {
            LogError( ( "Unexpected error during DTLS handshaking, error= -0x%lx %s : %s.",
                        -mbedtlsError,
                        mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                        mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
            returnStatus = DTLS_TRANSPORT_HANDSHAKE_FAILED;
        }
        else
        {
            pNetworkContext->state = DTLS_STATE_READY;
            returnStatus = DTLS_HANDSHAKE_COMPLETE;
        }
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/
