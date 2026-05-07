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

#ifndef USING_MBEDTLS
#define USING_MBEDTLS

#include "mbedtls/config.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ssl.h"
#include "mbedtls/threading.h"
#include "mbedtls/x509.h"
#include "mbedtls/error.h"

/* Include header that defines log levels. */
#include "logging.h"

/* TCP Sockets Wrapper include.*/
#include "tcp_sockets_wrapper.h"

/* Flags to be used in TLS_FreeRTOS_Connect. */
#define TLS_CONNECT_NON_BLOCKING_HANDSHAKE   ( 1 << 0 )

/**
 * @brief Secured connection context.
 */
typedef struct SSLContext
{
    mbedtls_ssl_config config;               /**< @brief SSL connection configuration. */
    mbedtls_ssl_context context;             /**< @brief SSL connection context */
    mbedtls_x509_crt_profile certProfile;    /**< @brief Certificate security profile for this connection. */
    mbedtls_x509_crt rootCa;                 /**< @brief Root CA certificate context. */
    mbedtls_x509_crt clientCert;             /**< @brief Client certificate context. */
    mbedtls_pk_context privKey;              /**< @brief Client private key context. */
    mbedtls_entropy_context entropyContext;  /**< @brief Entropy context for random number generation. */
    mbedtls_ctr_drbg_context ctrDrbgContext; /**< @brief CTR DRBG context for random number generation. */
} SSLContext_t;

/**
 * @brief Parameters for the network context of the transport interface
 * implementation that uses mbedTLS and FreeRTOS+TCP sockets.
 */
typedef struct TlsTransportParams
{
    Socket_t tcpSocket;
    SSLContext_t sslContext;
} TlsTransportParams_t;

typedef struct TlsNetworkContext
{
    TlsTransportParams_t * pParams;
} TlsNetworkContext_t;

/**
 * @brief Contains the credentials necessary for tls connection setup.
 */
typedef struct NetworkCredentials
{
    /**
     * @brief To use ALPN, set this to a NULL-terminated list of supported
     * protocols in decreasing order of preference.
     *
     * See [this link]
     * (https://aws.amazon.com/blogs/iot/mqtt-with-tls-client-authentication-on-port-443-why-it-is-useful-and-how-it-works/)
     * for more information.
     */
    const char ** pAlpnProtos;

    /**
     * @brief Disable server name indication (SNI) for a TLS session.
     */
    BaseType_t disableSni;

    const uint8_t * pRootCa;     /**< @brief String representing a trusted server root certificate. */
    size_t rootCaSize;           /**< @brief Size associated with #NetworkCredentials.pRootCa. */
    const uint8_t * pRootCaPath; /**< @brief String representing a trusted server root certificate path. */
    size_t rootCaPathLength;     /**< @brief Length associated with #NetworkCredentials.pRootCaPath. */
    const uint8_t * pClientCert; /**< @brief String representing the client certificate. */
    size_t clientCertSize;       /**< @brief Size associated with #NetworkCredentials.pClientCert. */
    const uint8_t * pPrivateKey; /**< @brief String representing the client certificate's private key. */
    size_t privateKeySize;       /**< @brief Size associated with #NetworkCredentials.pPrivateKey. */
} NetworkCredentials_t;

typedef struct TlsSession
{
    TlsNetworkContext_t xTlsNetworkContext;
    TlsTransportParams_t xTlsTransportParams;
} TlsSession_t;

/**
 * @brief TLS Connect / Disconnect return status.
 */
typedef enum TlsTransportStatus
{
    TLS_TRANSPORT_SUCCESS = 0,              /**< Function successfully completed. */
    TLS_TRANSPORT_INVALID_PARAMETER,        /**< At least one parameter was invalid. */
    TLS_TRANSPORT_INSUFFICIENT_MEMORY,      /**< Insufficient memory required to establish connection. */
    TLS_TRANSPORT_INVALID_CREDENTIALS,      /**< Provided credentials were invalid. */
    TLS_TRANSPORT_HANDSHAKE_FAILED,         /**< Performing TLS handshake with server failed. */
    TLS_TRANSPORT_HANDSHAKE_IN_PROGRESS,    /**< TLS handshake with server is in-progress. */
    TLS_TRANSPORT_INTERNAL_ERROR,           /**< A call to a system API resulted in an internal error. */
    TLS_TRANSPORT_CONNECT_FAILURE           /**< Initial connection to the server failed. */
} TlsTransportStatus_t;

/**
 * @brief Continue the TLS handshake that was started in TLS_FreeRTOS_Connect.
 *
 * @param[in] pTlsNetworkContext The Network context.
 *
 * @return #TLS_TRANSPORT_SUCCESS, #TLS_TRANSPORT_INVALID_PARAMETER,
 * #TLS_TRANSPORT_HANDSHAKE_FAILED, or #TLS_TRANSPORT_HANDSHAKE_IN_PROGRESS.
 */
TlsTransportStatus_t TLS_FreeRTOS_ContinueHandshake( TlsNetworkContext_t * pTlsNetworkContext );

/**
 * @brief Create a TLS connection with FreeRTOS sockets.
 *
 * @param[out] pTlsNetworkContext Pointer to a network context to contain the
 * initialized socket handle.
 * @param[in] pHostName The hostname of the remote endpoint.
 * @param[in] port The destination port.
 * @param[in] pNetworkCredentials Credentials for the TLS connection.
 * @param[in] receiveTimeoutMs Receive socket timeout.
 * @param[in] sendTimeoutMs Send socket timeout.
 * @param[in] flags Flags to configure additional behaviors, example, TLS_CONNECT_NON_BLOCKING_HANDSHAKE
 *
 * @return #TLS_TRANSPORT_SUCCESS, #TLS_TRANSPORT_INSUFFICIENT_MEMORY, #TLS_TRANSPORT_INVALID_CREDENTIALS,
 * #TLS_TRANSPORT_HANDSHAKE_FAILED, #TLS_TRANSPORT_INTERNAL_ERROR, or #TLS_TRANSPORT_CONNECT_FAILURE,
 * or #TLS_TRANSPORT_HANDSHAKE_IN_PROGRESS.
 */
TlsTransportStatus_t TLS_FreeRTOS_Connect( TlsNetworkContext_t * pTlsNetworkContext,
                                           const char * pHostName,
                                           uint16_t port,
                                           const NetworkCredentials_t * pNetworkCredentials,
                                           uint32_t receiveTimeoutMs,
                                           uint32_t sendTimeoutMs,
                                           uint32_t flags );

/**
 * @brief Gracefully disconnect an established TLS connection.
 *
 * @param[in] pTlsNetworkContext Network context.
 */
TlsTransportStatus_t TLS_FreeRTOS_Disconnect( TlsNetworkContext_t * pTlsNetworkContext );

/**
 * @brief Receives data from an established TLS connection.
 *
 * @note This is the TLS version of the transport interface's
 * #TransportRecv_t function.
 *
 * @param[in] pTlsNetworkContext The Network context.
 * @param[out] pBuffer Buffer to receive bytes into.
 * @param[in] bytesToRecv Number of bytes to receive from the network.
 *
 * @return Number of bytes (> 0) received if successful;
 * 0 if the socket times out without reading any bytes;
 * negative value on error.
 */
int32_t TLS_FreeRTOS_recv( TlsNetworkContext_t * pTlsNetworkContext,
                           void * pBuffer,
                           size_t bytesToRecv );

/**
 * @brief Sends data over an established TLS connection.
 *
 * @note This is the TLS version of the transport interface's
 * #TransportSend_t function.
 *
 * @param[in] pTlsNetworkContext The network context.
 * @param[in] pBuffer Buffer containing the bytes to send.
 * @param[in] bytesToSend Number of bytes to send from the buffer.
 *
 * @return Number of bytes (> 0) sent on success;
 * 0 if the socket times out without sending any bytes;
 * else a negative value to represent error.
 */
int32_t TLS_FreeRTOS_send( TlsNetworkContext_t * pTlsNetworkContext,
                           const void * pBuffer,
                           size_t bytesToSend );

/**
 * @brief Get the socket FD for this network context.
 *
 * @param[in] pTlsNetworkContext The network context.
 *
 * @return The socket descriptor if value >= 0. It returns -1 when failure.
 */
int32_t TLS_FreeRTOS_GetSocketFd( TlsNetworkContext_t * pTlsNetworkContext );

/**
 * @brief Configures receive and send timeouts for an existing TLS connection.
 *
 * @param[in] pTlsNetworkContext Pointer to the network context containing the TLS connection.
 * @param[in] receiveTimeoutMs Timeout in milliseconds for receiving data.
 * @param[in] sendTimeoutMs Timeout in milliseconds for sending data.
 *
 * @return Returns TLS_TRANSPORT_SUCCESS if timeouts are configured successfully,
 *         otherwise returns appropriate error code indicating failure.
 */
TlsTransportStatus_t TLS_FreeRTOS_ConfigureTimeout( TlsNetworkContext_t * pTlsNetworkContext,
                                                    uint32_t receiveTimeoutMs,
                                                    uint32_t sendTimeoutMs );

#ifdef MBEDTLS_DEBUG_C

/**
 * @brief Write an MBedTLS Debug message to the LogDebug() function
 *
 * @param[in] sslContext Pointer of the SSL Context that is being used
 * @param[in] level The severity level of the debug message from MBedTLS
 * @param[in] file Name of the file that the debug message is from
 * @param[in] line The line number that the debug message is from
 * @param[in] str The full string debug message from MBedTLS
 *
 * @return void
 */
void mbedtls_string_printf( void * sslContext,
                            int level,
                            const char * file,
                            int line,
                            const char * str );
#endif /* MBEDTLS_DEBUG_C */

#endif /* ifndef USING_MBEDTLS */
