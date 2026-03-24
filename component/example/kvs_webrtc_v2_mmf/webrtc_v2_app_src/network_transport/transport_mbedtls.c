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

/* FreeRTOS includes. */
#include "FreeRTOS.h"

#include "mbedtls/config.h"
#include "mbedtls/version.h"

#ifdef MBEDTLS_PSA_CRYPTO_C
/* MbedTLS PSA Includes */
#include "psa/crypto.h"
#include "psa/crypto_values.h"
#endif /* MBEDTLS_PSA_CRYPTO_C */

#ifdef MBEDTLS_DEBUG_C
#include "mbedtls/debug.h"
#endif /* MBEDTLS_DEBUG_C */

/* MBedTLS Bio TCP sockets wrapper include. */
#include "mbedtls_bio_tcp_sockets_wrapper.h"

/* TLS transport header. */
#include "transport_mbedtls.h"

/*-----------------------------------------------------------*/

/**
 * @brief Utility for converting the high-level code in an mbedTLS error to string,
 * if the code-contains a high-level code; otherwise, using a default string.
 */
#define mbedtlsHighLevelCodeOrDefault( mbedTlsCode )       "mbedTLS high level Error"

/**
 * @brief Utility for converting the level-level code in an mbedTLS error to string,
 * if the code-contains a level-level code; otherwise, using a default string.
 */
#define mbedtlsLowLevelCodeOrDefault( mbedTlsCode )       "mbedTLS low level Error"

#define TRANSPORT_MBEDTLS_MAX_FILE_PATH_LENGTH ( 1024 )

/*-----------------------------------------------------------*/

/**
 * @brief Initialize the mbed TLS structures in a network connection.
 *
 * @param[in] pSslContext The SSL context to initialize.
 */
static void sslContextInit( SSLContext_t * pSslContext );

/**
 * @brief Free the mbed TLS structures in a network connection.
 *
 * @param[in] pSslContext The SSL context to free.
 */
static void sslContextFree( SSLContext_t * pSslContext );

/**
 * @brief Add X509 certificate to the trusted list of root certificates.
 *
 * OpenSSL does not provide a single function for reading and loading certificates
 * from files into stores, so the file API must be called. Start with the
 * root certificate.
 *
 * @param[out] pSslContext SSL context to which the trusted server root CA is to be added.
 * @param[in] pRootCa PEM-encoded string of the trusted server root CA.
 * @param[in] rootCaSize Size of the trusted server root CA.
 *
 * @return 0 on success; otherwise, failure;
 */
static int32_t setRootCa( SSLContext_t * pSslContext,
                          const uint8_t * pRootCa,
                          size_t rootCaSize );

/**
 * @brief Set X509 certificate as client certificate for the server to authenticate.
 *
 * @param[out] pSslContext SSL context to which the client certificate is to be set.
 * @param[in] pClientCert PEM-encoded string of the client certificate.
 * @param[in] clientCertSize Size of the client certificate.
 *
 * @return 0 on success; otherwise, failure;
 */
static int32_t setClientCertificate( SSLContext_t * pSslContext,
                                     const uint8_t * pClientCert,
                                     size_t clientCertSize );

/**
 * @brief Set private key for the client's certificate.
 *
 * @param[out] pSslContext SSL context to which the private key is to be set.
 * @param[in] pPrivateKey PEM-encoded string of the client private key.
 * @param[in] privateKeySize Size of the client private key.
 *
 * @return 0 on success; otherwise, failure;
 */
static int32_t setPrivateKey( SSLContext_t * pSslContext,
                              const uint8_t * pPrivateKey,
                              size_t privateKeySize );

/**
 * @brief Passes TLS credentials to the OpenSSL library.
 *
 * Provides the root CA certificate, client certificate, and private key to the
 * OpenSSL library. If the client certificate or private key is not NULL, mutual
 * authentication is used when performing the TLS handshake.
 *
 * @param[out] pSslContext SSL context to which the credentials are to be imported.
 * @param[in] pNetworkCredentials TLS credentials to be imported.
 *
 * @return 0 on success; otherwise, failure;
 */
static int32_t setCredentials( SSLContext_t * pSslContext,
                               const NetworkCredentials_t * pNetworkCredentials );

/**
 * @brief Set optional configurations for the TLS connection.
 *
 * This function is used to set SNI and ALPN protocols.
 *
 * @param[in] pSslContext SSL context to which the optional configurations are to be set.
 * @param[in] pHostName Remote host name, used for server name indication.
 * @param[in] pNetworkCredentials TLS setup parameters.
 */
static void setOptionalConfigurations( SSLContext_t * pSslContext,
                                       const char * pHostName,
                                       const NetworkCredentials_t * pNetworkCredentials );

/**
 * @brief Setup TLS by initializing contexts and setting configurations.
 *
 * @param[in] pTlsNetworkContext Network context.
 * @param[in] pHostName Remote host name, used for server name indication.
 * @param[in] pNetworkCredentials TLS setup parameters.
 *
 * @return #TLS_TRANSPORT_SUCCESS, #TLS_TRANSPORT_INSUFFICIENT_MEMORY, #TLS_TRANSPORT_INVALID_CREDENTIALS,
 * or #TLS_TRANSPORT_INTERNAL_ERROR.
 */
static TlsTransportStatus_t tlsSetup( TlsNetworkContext_t * pTlsNetworkContext,
                                      const char * pHostName,
                                      const NetworkCredentials_t * pNetworkCredentials );

/**
 * @brief Perform the TLS handshake on a TCP connection.
 *
 * @param[in] pTlsNetworkContext Network context.
 * @param[in] pNetworkCredentials TLS setup parameters.
 * @param[in] flags Flags to configure additional behaviors, example, TLS_CONNECT_NON_BLOCKING_HANDSHAKE
 *
 * @return #TLS_TRANSPORT_SUCCESS, #TLS_TRANSPORT_HANDSHAKE_FAILED, or #TLS_TRANSPORT_INTERNAL_ERROR.
 */
static TlsTransportStatus_t tlsHandshake( TlsNetworkContext_t * pTlsNetworkContext,
                                          const NetworkCredentials_t * pNetworkCredentials,
                                          uint32_t flags );

/**
 * @brief Initialize mbedTLS.
 *
 * @param[out] entropyContext mbed TLS entropy context for generation of random numbers.
 * @param[out] ctrDrbgContext mbed TLS CTR DRBG context for generation of random numbers.
 *
 * @return #TLS_TRANSPORT_SUCCESS, or #TLS_TRANSPORT_INTERNAL_ERROR.
 */
static TlsTransportStatus_t initMbedtls( mbedtls_entropy_context * pEntropyContext,
                                         mbedtls_ctr_drbg_context * pCtrDrbgContext );

/*-----------------------------------------------------------*/

#ifdef MBEDTLS_DEBUG_C
void mbedtls_string_printf( void * sslContext,
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

static void sslContextInit( SSLContext_t * pSslContext )
{
    configASSERT( pSslContext != NULL );

    mbedtls_ssl_config_init( &( pSslContext->config ) );
    mbedtls_x509_crt_init( &( pSslContext->rootCa ) );
    mbedtls_pk_init( &( pSslContext->privKey ) );
    mbedtls_x509_crt_init( &( pSslContext->clientCert ) );
    mbedtls_ssl_init( &( pSslContext->context ) );
    #ifdef MBEDTLS_DEBUG_C
    mbedtls_debug_set_threshold( 1 );
    mbedtls_ssl_conf_dbg( &( pSslContext->config ),
                          mbedtls_string_printf,
                          NULL );
    #endif /* MBEDTLS_DEBUG_C */
}
/*-----------------------------------------------------------*/

static void sslContextFree( SSLContext_t * pSslContext )
{
    configASSERT( pSslContext != NULL );

    mbedtls_ssl_free( &( pSslContext->context ) );
    mbedtls_x509_crt_free( &( pSslContext->rootCa ) );
    mbedtls_x509_crt_free( &( pSslContext->clientCert ) );
    mbedtls_pk_free( &( pSslContext->privKey ) );
    mbedtls_entropy_free( &( pSslContext->entropyContext ) );
    mbedtls_ctr_drbg_free( &( pSslContext->ctrDrbgContext ) );
    mbedtls_ssl_config_free( &( pSslContext->config ) );
}
/*-----------------------------------------------------------*/

static int32_t setRootCa( SSLContext_t * pSslContext,
                          const uint8_t * pRootCa,
                          size_t rootCaSize )
{
    int32_t mbedtlsError = -1;

    configASSERT( pSslContext != NULL );
    configASSERT( pRootCa != NULL );

    /* Parse the server root CA certificate into the SSL context. */
    mbedtlsError = mbedtls_x509_crt_parse( &( pSslContext->rootCa ),
                                           pRootCa,
                                           rootCaSize );

    if( mbedtlsError != 0 )
    {
        LogError( ( "Failed to parse server root CA certificate: %.*s, result: -0x%lx mbedTLSError= %s : %s.",
                    ( int ) rootCaSize, pRootCa,
                    mbedtlsError,
                    mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                    mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
    }
    else
    {
        mbedtls_ssl_conf_ca_chain( &( pSslContext->config ),
                                   &( pSslContext->rootCa ),
                                   NULL );
    }

    return mbedtlsError;
}
/*-----------------------------------------------------------*/

static int32_t setRootCaPath( SSLContext_t * pSslContext,
                              const char * pRootCaPath,
                              size_t rootCaPathLength )
{
    int32_t mbedtlsError = -1;
    /* Allocate one extra space for NULL terminator. */
    static char caFilePath[ TRANSPORT_MBEDTLS_MAX_FILE_PATH_LENGTH + 1 ];

    configASSERT( pSslContext != NULL );
    configASSERT( pRootCaPath != NULL );
    configASSERT( rootCaPathLength < TRANSPORT_MBEDTLS_MAX_FILE_PATH_LENGTH );

    memcpy( caFilePath, pRootCaPath, rootCaPathLength );
    caFilePath[ rootCaPathLength ] = '\0';

    /* Parse the server root CA certificate into the SSL context. */
    mbedtlsError = mbedtls_x509_crt_parse_file( &( pSslContext->rootCa ),
                                                caFilePath );

    if( mbedtlsError != 0 )
    {
        LogError( ( "Failed to parse server root CA certificate: %s, result: -0x%lx mbedTLSError= %s : %s.",
                    caFilePath,
                    mbedtlsError,
                    mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                    mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
    }
    else
    {
        mbedtls_ssl_conf_ca_chain( &( pSslContext->config ),
                                   &( pSslContext->rootCa ),
                                   NULL );
    }

    return mbedtlsError;
}
/*-----------------------------------------------------------*/

static int32_t setClientCertificate( SSLContext_t * pSslContext,
                                     const uint8_t * pClientCert,
                                     size_t clientCertSize )
{
    int32_t mbedtlsError = -1;

    configASSERT( pSslContext != NULL );
    configASSERT( pClientCert != NULL );

    /* Setup the client certificate. */
    mbedtlsError = mbedtls_x509_crt_parse( &( pSslContext->clientCert ),
                                           pClientCert,
                                           clientCertSize );

    if( mbedtlsError != 0 )
    {
        LogError( ( "Failed to parse the client certificate: mbedTLSError= %s : %s.",
                    mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                    mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
    }

    return mbedtlsError;
}
/*-----------------------------------------------------------*/

static int32_t setPrivateKey( SSLContext_t * pSslContext,
                              const uint8_t * pPrivateKey,
                              size_t privateKeySize )
{
    int32_t mbedtlsError = -1;

    configASSERT( pSslContext != NULL );
    configASSERT( pPrivateKey != NULL );

    #if MBEDTLS_VERSION_NUMBER < 0x03000000
    mbedtlsError = mbedtls_pk_parse_key( &( pSslContext->privKey ),
                                         pPrivateKey,
                                         privateKeySize,
                                         NULL, 0 );
    #else
    mbedtlsError = mbedtls_pk_parse_key( &( pSslContext->privKey ),
                                         pPrivateKey,
                                         privateKeySize,
                                         NULL, 0,
                                         mbedtls_ctr_drbg_random,
                                         &( pSslContext->ctrDrbgContext ) );
    #endif /* if MBEDTLS_VERSION_NUMBER < 0x03000000 */

    if( mbedtlsError != 0 )
    {
        LogError( ( "Failed to parse the client key: mbedTLSError= %s : %s.",
                    mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                    mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
    }

    return mbedtlsError;
}
/*-----------------------------------------------------------*/

static int32_t setCredentials( SSLContext_t * pSslContext,
                               const NetworkCredentials_t * pNetworkCredentials )
{
    int32_t mbedtlsError = 0;

    configASSERT( pSslContext != NULL );
    configASSERT( pNetworkCredentials != NULL );

    /* Set up the certificate security profile, starting from the default value. */
    pSslContext->certProfile = mbedtls_x509_crt_profile_default;

    /* Set SSL authmode and the RNG context. */
    mbedtls_ssl_conf_authmode( &( pSslContext->config ),
                               MBEDTLS_SSL_VERIFY_REQUIRED );
    mbedtls_ssl_conf_rng( &( pSslContext->config ),
                          mbedtls_ctr_drbg_random,
                          &( pSslContext->ctrDrbgContext ) );
    mbedtls_ssl_conf_cert_profile( &( pSslContext->config ),
                                   &( pSslContext->certProfile ) );

    if( pNetworkCredentials->pRootCa != NULL )
    {
        mbedtlsError = setRootCa( pSslContext,
                                  pNetworkCredentials->pRootCa,
                                  pNetworkCredentials->rootCaSize );
    }
    else
    {
        mbedtlsError = setRootCaPath( pSslContext,
                                      ( const char * ) pNetworkCredentials->pRootCaPath,
                                      pNetworkCredentials->rootCaPathLength );
    }

    if( ( pNetworkCredentials->pClientCert != NULL ) &&
        ( pNetworkCredentials->pPrivateKey != NULL ) )
    {
        if( mbedtlsError == 0 )
        {
            mbedtlsError = setClientCertificate( pSslContext,
                                                 pNetworkCredentials->pClientCert,
                                                 pNetworkCredentials->clientCertSize );
        }

        if( mbedtlsError == 0 )
        {
            mbedtlsError = setPrivateKey( pSslContext,
                                          pNetworkCredentials->pPrivateKey,
                                          pNetworkCredentials->privateKeySize );
        }

        if( mbedtlsError == 0 )
        {
            mbedtlsError = mbedtls_ssl_conf_own_cert( &( pSslContext->config ),
                                                      &( pSslContext->clientCert ),
                                                      &( pSslContext->privKey ) );
        }
    }

    return mbedtlsError;
}
/*-----------------------------------------------------------*/

static void setOptionalConfigurations( SSLContext_t * pSslContext,
                                       const char * pHostName,
                                       const NetworkCredentials_t * pNetworkCredentials )
{
    int32_t mbedtlsError = -1;

    configASSERT( pSslContext != NULL );
    configASSERT( pHostName != NULL );
    configASSERT( pNetworkCredentials != NULL );

    if( pNetworkCredentials->pAlpnProtos != NULL )
    {
        /* Include an application protocol list in the TLS ClientHello
         * message. */
        mbedtlsError = mbedtls_ssl_conf_alpn_protocols( &( pSslContext->config ),
                                                        pNetworkCredentials->pAlpnProtos );

        if( mbedtlsError != 0 )
        {
            LogError( ( "Failed to configure ALPN protocol in mbed TLS: mbedTLSError= %s : %s.",
                        mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                        mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
        }
    }

    /* Enable SNI if requested. */
    if( pNetworkCredentials->disableSni == pdFALSE )
    {
        mbedtlsError = mbedtls_ssl_set_hostname( &( pSslContext->context ),
                                                 pHostName );

        if( mbedtlsError != 0 )
        {
            LogError( ( "Failed to set server name: mbedTLSError= %s : %s.",
                        mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                        mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
        }
    }
}
/*-----------------------------------------------------------*/

static TlsTransportStatus_t tlsSetup( TlsNetworkContext_t * pTlsNetworkContext,
                                      const char * pHostName,
                                      const NetworkCredentials_t * pNetworkCredentials )
{
    TlsTransportParams_t * pTlsTransportParams = NULL;
    TlsTransportStatus_t returnStatus = TLS_TRANSPORT_SUCCESS;
    int32_t mbedtlsError = 0;

    configASSERT( pTlsNetworkContext != NULL );
    configASSERT( pTlsNetworkContext->pParams != NULL );
    configASSERT( pHostName != NULL );
    configASSERT( pNetworkCredentials != NULL );
    configASSERT( pNetworkCredentials->pRootCa != NULL || pNetworkCredentials->pRootCaPath != NULL );

    pTlsTransportParams = pTlsNetworkContext->pParams;
    /* Initialize the mbed TLS context structures. */
    sslContextInit( &( pTlsTransportParams->sslContext ) );

    mbedtlsError = mbedtls_ssl_config_defaults( &( pTlsTransportParams->sslContext.config ),
                                                MBEDTLS_SSL_IS_CLIENT,
                                                MBEDTLS_SSL_TRANSPORT_STREAM,
                                                MBEDTLS_SSL_PRESET_DEFAULT );

    if( mbedtlsError != 0 )
    {
        LogError( ( "Failed to set default SSL configuration: mbedTLSError= %s : %s.",
                    mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                    mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );

        /* Per mbed TLS docs, mbedtls_ssl_config_defaults only fails on memory allocation. */
        returnStatus = TLS_TRANSPORT_INSUFFICIENT_MEMORY;
    }

    if( returnStatus == TLS_TRANSPORT_SUCCESS )
    {
        mbedtlsError = setCredentials( &( pTlsTransportParams->sslContext ),
                                       pNetworkCredentials );

        if( mbedtlsError != 0 )
        {
            returnStatus = TLS_TRANSPORT_INVALID_CREDENTIALS;
        }
        else
        {
            /* Optionally set SNI and ALPN protocols. */
            setOptionalConfigurations( &( pTlsTransportParams->sslContext ),
                                       pHostName,
                                       pNetworkCredentials );
        }
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

static TlsTransportStatus_t tlsHandshake( TlsNetworkContext_t * pTlsNetworkContext,
                                          const NetworkCredentials_t * pNetworkCredentials,
                                          uint32_t flags )
{
    TlsTransportParams_t * pTlsTransportParams = NULL;
    TlsTransportStatus_t returnStatus = TLS_TRANSPORT_SUCCESS;
    int32_t mbedtlsError = 0;

    configASSERT( pTlsNetworkContext != NULL );
    configASSERT( pTlsNetworkContext->pParams != NULL );
    configASSERT( pNetworkCredentials != NULL );

    pTlsTransportParams = pTlsNetworkContext->pParams;
    /* Initialize the mbed TLS secured connection context. */
    mbedtlsError = mbedtls_ssl_setup( &( pTlsTransportParams->sslContext.context ),
                                      &( pTlsTransportParams->sslContext.config ) );

    if( mbedtlsError != 0 )
    {
        LogError( ( "Failed to set up mbed TLS SSL context: mbedTLSError= %s : %s.",
                    mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                    mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );

        returnStatus = TLS_TRANSPORT_INTERNAL_ERROR;
    }
    else
    {
        /* Set the underlying IO for the TLS connection. */

        /* MISRA Rule 11.2 flags the following line for casting the second
         * parameter to void *. This rule is suppressed because
         * #mbedtls_ssl_set_bio requires the second parameter as void *.
         */
        /* coverity[misra_c_2012_rule_11_2_violation] */

        /* These two macros MBEDTLS_SSL_SEND and MBEDTLS_SSL_RECV need to be
         * defined in mbedtls_config.h according to which implementation you use.
         */
        mbedtls_ssl_set_bio( &( pTlsTransportParams->sslContext.context ),
                             ( void * ) pTlsTransportParams->tcpSocket,
                             xMbedTLSBioTCPSocketsWrapperSend,
                             xMbedTLSBioTCPSocketsWrapperRecv,
                             NULL );
    }

    if( returnStatus == TLS_TRANSPORT_SUCCESS )
    {
        /* Perform the TLS handshake. */
        if( ( flags & TLS_CONNECT_NON_BLOCKING_HANDSHAKE ) == 0 )
        {
            do
            {
                returnStatus = TLS_FreeRTOS_ContinueHandshake( pTlsNetworkContext );
            } while( returnStatus == TLS_TRANSPORT_HANDSHAKE_IN_PROGRESS );
        }
        else
        {
            returnStatus = TLS_FreeRTOS_ContinueHandshake( pTlsNetworkContext );
        }

        if( returnStatus == TLS_TRANSPORT_HANDSHAKE_FAILED )
        {
            LogError( ( "Failed to perform TLS handshake." ) );
        }
        else if( returnStatus == TLS_TRANSPORT_HANDSHAKE_IN_PROGRESS )
        {
            LogDebug( ( "TLS handshake is in progress." ) );
        }
        else
        {
            LogInfo( ( "(Network connection %p) TLS handshake successful.",
                       pTlsNetworkContext ) );
        }
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

static TlsTransportStatus_t initMbedtls( mbedtls_entropy_context * pEntropyContext,
                                         mbedtls_ctr_drbg_context * pCtrDrbgContext )
{
    TlsTransportStatus_t returnStatus = TLS_TRANSPORT_SUCCESS;
    int32_t mbedtlsError = 0;

    #if defined( MBEDTLS_THREADING_ALT )
    /* Set the mutex functions for mbed TLS thread safety. */
    mbedtls_platform_threading_init();
    #endif

    /* Initialize contexts for random number generation. */
    mbedtls_entropy_init( pEntropyContext );
    mbedtls_ctr_drbg_init( pCtrDrbgContext );

    if( mbedtlsError != 0 )
    {
        LogError( ( "Failed to add entropy source: mbedTLSError= %s : %s.",
                    mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                    mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
        returnStatus = TLS_TRANSPORT_INTERNAL_ERROR;
    }

    #ifdef MBEDTLS_PSA_CRYPTO_C
    if( returnStatus == TLS_TRANSPORT_SUCCESS )
    {
        mbedtlsError = psa_crypto_init();

        if( mbedtlsError != PSA_SUCCESS )
        {
            LogError( ( "Failed to initialize PSA Crypto implementation: %s", ( int ) mbedtlsError ) );
            returnStatus = TLS_TRANSPORT_INTERNAL_ERROR;
        }
    }
    #endif /* MBEDTLS_PSA_CRYPTO_C */

    if( returnStatus == TLS_TRANSPORT_SUCCESS )
    {
        /* Seed the random number generator. */
        mbedtlsError = mbedtls_ctr_drbg_seed( pCtrDrbgContext,
                                              mbedtls_entropy_func,
                                              pEntropyContext,
                                              NULL,
                                              0 );

        if( mbedtlsError != 0 )
        {
            LogError( ( "Failed to seed PRNG: mbedTLSError= %s : %s.",
                        mbedtlsHighLevelCodeOrDefault( mbedtlsError ),
                        mbedtlsLowLevelCodeOrDefault( mbedtlsError ) ) );
            returnStatus = TLS_TRANSPORT_INTERNAL_ERROR;
        }
    }

    if( returnStatus == TLS_TRANSPORT_SUCCESS )
    {
        LogDebug( ( "Successfully initialized mbedTLS." ) );
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

TlsTransportStatus_t TLS_FreeRTOS_ContinueHandshake( TlsNetworkContext_t * pTlsNetworkContext )
{
    TlsTransportParams_t * pTlsTransportParams = NULL;
    TlsTransportStatus_t returnStatus = TLS_TRANSPORT_SUCCESS;
    int32_t mbedtlsError = 0;

    if( ( pTlsNetworkContext == NULL ) ||
        ( pTlsNetworkContext->pParams == NULL ) )
    {
        returnStatus = TLS_TRANSPORT_INVALID_PARAMETER;
    }

    if( returnStatus == TLS_TRANSPORT_SUCCESS )
    {
        pTlsTransportParams = pTlsNetworkContext->pParams;
        mbedtlsError = mbedtls_ssl_handshake( &( pTlsTransportParams->sslContext.context ) );

        if( ( mbedtlsError == MBEDTLS_ERR_SSL_WANT_READ ) ||
            ( mbedtlsError == MBEDTLS_ERR_SSL_WANT_WRITE ) )
        {
            returnStatus = TLS_TRANSPORT_HANDSHAKE_IN_PROGRESS;
        }
        else if( mbedtlsError != 0 )
        {
            returnStatus = TLS_TRANSPORT_HANDSHAKE_FAILED;
        }
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

TlsTransportStatus_t TLS_FreeRTOS_Connect( TlsNetworkContext_t * pTlsNetworkContext,
                                           const char * pHostName,
                                           uint16_t port,
                                           const NetworkCredentials_t * pNetworkCredentials,
                                           uint32_t receiveTimeoutMs,
                                           uint32_t sendTimeoutMs,
                                           uint32_t flags )
{
    TlsTransportParams_t * pTlsTransportParams = NULL;
    TlsTransportStatus_t returnStatus = TLS_TRANSPORT_SUCCESS;
    BaseType_t socketStatus = 0;
    BaseType_t isSocketConnected = pdFALSE, isTlsSetup = pdFALSE;

    if( ( pTlsNetworkContext == NULL ) ||
        ( pTlsNetworkContext->pParams == NULL ) ||
        ( pHostName == NULL ) ||
        ( pNetworkCredentials == NULL ) )
    {
        LogError( ( "Invalid input parameter(s): Arguments cannot be NULL. pTlsNetworkContext=%p, "
                    "pHostName=%p, pNetworkCredentials=%p.",
                    pTlsNetworkContext,
                    pHostName,
                    pNetworkCredentials ) );
        returnStatus = TLS_TRANSPORT_INVALID_PARAMETER;
    }
    else if( ( pNetworkCredentials->pRootCa == NULL ) &&
             ( pNetworkCredentials->pRootCaPath == NULL ) )
    {
        LogError( ( "pRootCa & pRootCaPath cannot both be NULL." ) );
        returnStatus = TLS_TRANSPORT_INVALID_PARAMETER;
    }
    else
    {
        /* Empty else for MISRA 15.7 compliance. */
    }

    /* Establish a TCP connection with the server. */
    if( returnStatus == TLS_TRANSPORT_SUCCESS )
    {
        pTlsTransportParams = pTlsNetworkContext->pParams;

        /* Initialize tcpSocket. */
        pTlsTransportParams->tcpSocket = NULL;

        socketStatus = TCP_Sockets_Connect( &( pTlsTransportParams->tcpSocket ),
                                            pHostName,
                                            port,
                                            receiveTimeoutMs,
                                            sendTimeoutMs );

        if( socketStatus != 0 )
        {
            LogError( ( "Failed to connect to %s with error %ld.",
                        pHostName,
                        socketStatus ) );
            returnStatus = TLS_TRANSPORT_CONNECT_FAILURE;
        }
    }

    /* Initialize mbedtls. */
    if( returnStatus == TLS_TRANSPORT_SUCCESS )
    {
        isSocketConnected = pdTRUE;

        returnStatus = initMbedtls( &( pTlsTransportParams->sslContext.entropyContext ),
                                    &( pTlsTransportParams->sslContext.ctrDrbgContext ) );
    }

    /* Initialize TLS contexts and set credentials. */
    if( returnStatus == TLS_TRANSPORT_SUCCESS )
    {
        returnStatus = tlsSetup( pTlsNetworkContext, pHostName, pNetworkCredentials );
    }

    /* Perform TLS handshake. */
    if( returnStatus == TLS_TRANSPORT_SUCCESS )
    {
        isTlsSetup = pdTRUE;

        returnStatus = tlsHandshake( pTlsNetworkContext, pNetworkCredentials, flags );
    }

    /* Clean up on failure. */
    if( ( returnStatus != TLS_TRANSPORT_SUCCESS ) &&
        ( returnStatus != TLS_TRANSPORT_HANDSHAKE_IN_PROGRESS ) )
    {
        /* Free SSL context if it's setup. */
        if( isTlsSetup == pdTRUE )
        {
            sslContextFree( &( pTlsTransportParams->sslContext ) );
        }

        /* Call Sockets_Disconnect if socket was connected. */
        if( isSocketConnected == pdTRUE )
        {
            TCP_Sockets_Disconnect( pTlsTransportParams->tcpSocket );
            pTlsTransportParams->tcpSocket = NULL;
        }
    }
    else
    {
        LogDebug( ( "(Network connection %p) Connection to %s established.",
                    pTlsNetworkContext,
                    pHostName ) );
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

TlsTransportStatus_t TLS_FreeRTOS_Disconnect( TlsNetworkContext_t * pTlsNetworkContext )
{
    TlsTransportParams_t * pTlsTransportParams = NULL;
    BaseType_t tlsStatus = 0;
    TlsTransportStatus_t returnStatus = TLS_TRANSPORT_SUCCESS;

    if( ( pTlsNetworkContext != NULL ) && ( pTlsNetworkContext->pParams != NULL ) )
    {
        pTlsTransportParams = pTlsNetworkContext->pParams;
        /* Attempting to terminate TLS connection. */
        tlsStatus = ( BaseType_t ) mbedtls_ssl_close_notify( &( pTlsTransportParams->sslContext.context ) );

        /* Ignore the WANT_READ and WANT_WRITE return values. */
        if( ( tlsStatus != ( BaseType_t ) MBEDTLS_ERR_SSL_WANT_READ ) &&
            ( tlsStatus != ( BaseType_t ) MBEDTLS_ERR_SSL_WANT_WRITE ) )
        {
            if( tlsStatus == 0 )
            {
                LogDebug( ( "(Network connection %p) TLS close-notify sent.",
                            pTlsNetworkContext ) );
            }
            else
            {
                LogError( ( "(Network connection %p) Failed to send TLS close-notify: mbedTLSError= %s : %s.",
                            pTlsNetworkContext,
                            mbedtlsHighLevelCodeOrDefault( tlsStatus ),
                            mbedtlsLowLevelCodeOrDefault( tlsStatus ) ) );

                returnStatus = TLS_TRANSPORT_INTERNAL_ERROR;
            }
        }
        else
        {
            /* WANT_READ and WANT_WRITE can be ignored. Logging for debugging purposes. */
            LogDebug( ( "(Network connection %p) TLS close-notify sent; "
                        "received %s as the TLS status can be ignored for close-notify.",
                        pTlsNetworkContext,
                        ( tlsStatus == MBEDTLS_ERR_SSL_WANT_READ ) ? "WANT_READ" : "WANT_WRITE" ) );
        }

        /* Call socket shutdown function to close connection. */
        TCP_Sockets_Disconnect( pTlsTransportParams->tcpSocket );

        /* Free mbed TLS contexts. */
        sslContextFree( &( pTlsTransportParams->sslContext ) );
    }
    else
    {
        LogError( ( "Invalid input parameter(s): Arguments cannot be NULL. pTlsNetworkContext=%p.",
                    pTlsNetworkContext ) );
        returnStatus = TLS_TRANSPORT_INVALID_PARAMETER;
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

int32_t TLS_FreeRTOS_recv( TlsNetworkContext_t * pTlsNetworkContext,
                           void * pBuffer,
                           size_t bytesToRecv )
{
    TlsTransportParams_t * pTlsTransportParams = NULL;
    int32_t tlsStatus = 0;

    if( ( pTlsNetworkContext == NULL ) || ( pTlsNetworkContext->pParams == NULL ) )
    {
        LogError( ( "invalid input, pTlsNetworkContext=%p", pTlsNetworkContext ) );
        tlsStatus = -1;
    }
    else if( pBuffer == NULL )
    {
        LogError( ( "invalid input, pBuffer == NULL" ) );
        tlsStatus = -1;
    }
    else if( bytesToRecv == 0 )
    {
        LogError( ( "invalid input, bytesToRecv == 0" ) );
        tlsStatus = -1;
    }
    else
    {
        pTlsTransportParams = pTlsNetworkContext->pParams;

        tlsStatus = ( int32_t ) mbedtls_ssl_read( &( pTlsTransportParams->sslContext.context ),
                                                  pBuffer,
                                                  bytesToRecv );

        if( ( tlsStatus == MBEDTLS_ERR_SSL_TIMEOUT ) ||
            ( tlsStatus == MBEDTLS_ERR_SSL_WANT_READ ) ||
            ( tlsStatus == MBEDTLS_ERR_SSL_WANT_WRITE ) )
        {
            /* Mark these set of errors as a timeout. The libraries may retry read
             * on these errors. */
            tlsStatus = 0;
        }
        else if( tlsStatus < 0 )
        {
            LogError( ( "Failed to read data: mbedTLSError= %s : %s.",
                        mbedtlsHighLevelCodeOrDefault( tlsStatus ),
                        mbedtlsLowLevelCodeOrDefault( tlsStatus ) ) );
        }
        else
        {
            /* Empty else marker. */
        }
    }

    return tlsStatus;
}
/*-----------------------------------------------------------*/

int32_t TLS_FreeRTOS_send( TlsNetworkContext_t * pTlsNetworkContext,
                           const void * pBuffer,
                           size_t bytesToSend )
{
    TlsTransportParams_t * pTlsTransportParams = NULL;
    int32_t tlsStatus = 0;

    if( ( pTlsNetworkContext == NULL ) || ( pTlsNetworkContext->pParams == NULL ) )
    {
        LogError( ( "invalid input, pTlsNetworkContext=%p", pTlsNetworkContext ) );
        tlsStatus = -1;
    }
    else if( pBuffer == NULL )
    {
        LogError( ( "invalid input, pBuffer == NULL" ) );
        tlsStatus = -1;
    }
    else if( bytesToSend == 0 )
    {
        LogError( ( "invalid input, bytesToSend == 0" ) );
        tlsStatus = -1;
    }
    else
    {
        pTlsTransportParams = pTlsNetworkContext->pParams;

        tlsStatus = ( int32_t ) mbedtls_ssl_write( &( pTlsTransportParams->sslContext.context ),
                                                   pBuffer,
                                                   bytesToSend );

        if( ( tlsStatus == MBEDTLS_ERR_SSL_TIMEOUT ) ||
            ( tlsStatus == MBEDTLS_ERR_SSL_WANT_READ ) ||
            ( tlsStatus == MBEDTLS_ERR_SSL_WANT_WRITE ) )
        {
            LogDebug( ( "Failed to send data. However, send can be retried on this error. "
                        "mbedTLSError= %s : %s.",
                        mbedtlsHighLevelCodeOrDefault( tlsStatus ),
                        mbedtlsLowLevelCodeOrDefault( tlsStatus ) ) );

            /* Mark these set of errors as a timeout. The libraries may retry send
             * on these errors. */
            tlsStatus = 0;
        }
        else if( tlsStatus < 0 )
        {
            LogError( ( "Failed to send TLS data: mbedTLSError=-0x%lx %s : %s.",
                        -tlsStatus,
                        mbedtlsHighLevelCodeOrDefault( tlsStatus ),
                        mbedtlsLowLevelCodeOrDefault( tlsStatus ) ) );
        }
        else
        {
            /* Sent data successfully. */
        }
    }

    return tlsStatus;
}
/*-----------------------------------------------------------*/

int32_t TLS_FreeRTOS_GetSocketFd( TlsNetworkContext_t * pTlsNetworkContext )
{
    int32_t ret = -1;

    if( ( pTlsNetworkContext != NULL ) && ( pTlsNetworkContext->pParams != NULL ) )
    {
        ret = TCP_Sockets_GetSocketFd( pTlsNetworkContext->pParams->tcpSocket );
    }

    return ret;
}
/*-----------------------------------------------------------*/

TlsTransportStatus_t TLS_FreeRTOS_ConfigureTimeout( TlsNetworkContext_t * pTlsNetworkContext,
                                                    uint32_t receiveTimeoutMs,
                                                    uint32_t sendTimeoutMs )
{
    TlsTransportStatus_t returnStatus = TLS_TRANSPORT_SUCCESS;
    BaseType_t tcpResult = TCP_SOCKETS_ERRNO_NONE;

    if( ( pTlsNetworkContext == NULL ) ||
        ( pTlsNetworkContext->pParams == NULL ) )
    {
        returnStatus = TLS_TRANSPORT_INVALID_PARAMETER;
    }

    if( returnStatus == TLS_TRANSPORT_SUCCESS )
    {
        tcpResult = TCP_Sockets_ConfigureTimeout( pTlsNetworkContext->pParams->tcpSocket, receiveTimeoutMs, sendTimeoutMs );
        if( tcpResult != TCP_SOCKETS_ERRNO_NONE )
        {
            LogError( ( "Failed to configure timeout: TCP_Sockets_ConfigureTimeout returned error %ld.",
                        tcpResult ) );
            returnStatus = TLS_TRANSPORT_INTERNAL_ERROR;
        }
    }

    return returnStatus;
}
