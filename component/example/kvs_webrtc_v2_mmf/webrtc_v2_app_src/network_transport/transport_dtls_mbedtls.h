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

#ifndef TRANSPORT_DTLS_MBEDTLS_H
#define TRANSPORT_DTLS_MBEDTLS_H

#include "mbedtls/config.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/error.h"
#include "mbedtls/ssl.h"
#include "mbedtls/version.h"
#if (MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100)
#include "mbedtls/compat-2.x.h"
#endif
#include "mbedtls/threading.h"
#include "mbedtls/x509.h"
#include "mbedtls/timing.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE( array ) ( sizeof( array ) / sizeof *( array ) )
#endif

#define MBEDTLS_ERROR_STRING_BUFFER_SIZE 512

#define MBEDTLS_ERROR_DESCRIPTION( err ) \
    do { \
        char _error_string[MBEDTLS_ERROR_STRING_BUFFER_SIZE]; \
        mbedtls_strerror( err, \
                          _error_string, \
                          sizeof( _error_string ) ); \
        LogError( ( "Error 0x%04x: %s\n", ( unsigned int )-( err ), _error_string ) ); \
    } while( 0 )

/* Include header that defines log levels. */
#include "logging.h"

/* SRTP */
#define CERTIFICATE_FINGERPRINT_LENGTH 160
#define MAX_SRTP_MASTER_KEY_LEN 16
#define MAX_SRTP_SALT_KEY_LEN 14
#define MAX_DTLS_RANDOM_BYTES_LEN 32
#define MAX_DTLS_MASTER_KEY_LEN 48

typedef int32_t (* OnTransportDtlsSendHook_t)( void * pCustomContext,
                                               const uint8_t * pInputBuffer,
                                               size_t inputBufferLength );

/*
 * For code readability use a typedef for DTLS-SRTP profiles
 *
 * Use_srtp extension protection profiles values as defined in
 * http://www.iana.org/assignments/srtp-protection/srtp-protection.xhtml
 *
 * Reminder: if this list is expanded mbedtls_ssl_check_srtp_profile_value
 * must be updated too.
 */
#if !(MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100)
    #define MBEDTLS_TLS_SRTP_AES128_CM_HMAC_SHA1_80     ( ( uint16_t ) 0x0001 )
    #define MBEDTLS_TLS_SRTP_AES128_CM_HMAC_SHA1_32     ( ( uint16_t ) 0x0002 )
    #define MBEDTLS_TLS_SRTP_NULL_HMAC_SHA1_80          ( ( uint16_t ) 0x0005 )
    #define MBEDTLS_TLS_SRTP_NULL_HMAC_SHA1_32          ( ( uint16_t ) 0x0006 )

    /* This one is not iana defined, but for code readability. */
    #define MBEDTLS_TLS_SRTP_UNSET                      ( ( uint16_t ) 0x0000 )
#endif

typedef enum
{
#if (MBEDTLS_VERSION_NUMBER == 0x03000000 || MBEDTLS_VERSION_NUMBER == 0x03020100)
    KVS_SRTP_PROFILE_AES128_CM_HMAC_SHA1_80 = MBEDTLS_TLS_SRTP_AES128_CM_HMAC_SHA1_80,
    KVS_SRTP_PROFILE_AES128_CM_HMAC_SHA1_32 = MBEDTLS_TLS_SRTP_AES128_CM_HMAC_SHA1_32,
#else
    KVS_SRTP_PROFILE_AES128_CM_HMAC_SHA1_80 = MBEDTLS_SRTP_AES128_CM_HMAC_SHA1_80,
    KVS_SRTP_PROFILE_AES128_CM_HMAC_SHA1_32 = MBEDTLS_SRTP_AES128_CM_HMAC_SHA1_32,
#endif
} KVS_SRTP_PROFILE;

typedef struct
{
    uint8_t masterSecret[MAX_DTLS_MASTER_KEY_LEN];
    // client random bytes + server random bytes
    uint8_t randBytes[2 * MAX_DTLS_RANDOM_BYTES_LEN];
    mbedtls_tls_prf_types tlsProfile;
} TlsKeys;

/**
 * @brief Secured connection context.
 */
typedef struct DtlsSSLContext
{
    mbedtls_ssl_config config;               /**< @brief SSL connection configuration. */
    mbedtls_ssl_context context;             /**< @brief SSL connection context */
    mbedtls_x509_crt_profile certProfile;    /**< @brief Certificate security profile for this connection. */
    mbedtls_x509_crt rootCa;                 /**< @brief Root CA certificate context. */
    mbedtls_x509_crt clientCert;             /**< @brief Client certificate context. */
    mbedtls_pk_context privKey;              /**< @brief Client private key context. */
    TlsKeys tlsKeys;                         /**< @brief Client private key context. */
    mbedtls_entropy_context entropyContext;  /**< @brief Entropy context for random number generation. */
    mbedtls_ctr_drbg_context ctrDrbgContext; /**< @brief CTR DRBG context for random number generation. */
} DtlsSSLContext_t;

typedef void (* mbedtls_set_delay_fptr)( void *,
                                         uint32_t,
                                         uint32_t );
typedef int (* mbedtls_get_delay_fptr)( void * );

typedef struct DtlsSessionTimer
{
    uint32_t int_ms;                  // Intermediate delay in milliseconds
    uint32_t fin_ms;                  // Final delay in milliseconds
    int64_t start_ticks;              // Start tick count
    mbedtls_set_delay_fptr set_delay; // Function pointer to set delay
    mbedtls_get_delay_fptr get_delay; // Function pointer to get delay
} DtlsSessionTimer_t;

typedef struct DtlsRetransmissionParams
{
    DtlsSessionTimer_t transmissionTimer;
    uint32_t dtlsSessionStartTime;
    uint32_t dtlsSessionSetupTime;
} DtlsRetransmission_t;

/**
 * @brief Parameters for the network context of the transport interface
 * implementation that uses mbedTLS and UDP sockets.
 */
typedef struct DtlsTransportParams
{
    DtlsSSLContext_t dtlsSslContext;
    mbedtls_timing_delay_context mbedtlsTimer;
    OnTransportDtlsSendHook_t onDtlsSendHook;
    void * pOnDtlsSendCustomContext;

    /* Store the processing packet here. */
    uint8_t * pReceivedPacket;
    size_t receivedPacketLength;
    uint32_t receivedPacketOffset;
} DtlsTransportParams_t;

typedef enum DtlsState
{
    DTLS_STATE_NONE = 0,
    DTLS_STATE_NEW,
    DTLS_STATE_HANDSHAKING,
    DTLS_STATE_READY,
} DtlsState_t;

/**
 * @brief Each compilation unit that consumes the NetworkContext must define it.
 * It should contain a single pointer as seen below whenever the header file
 * of this transport implementation is included to your project.
 *
 * @note When using multiple transports in the same compilation unit,
 *       define this pointer as void *.
 */
struct DtlsNetworkContext
{
    DtlsState_t state;
    DtlsTransportParams_t * pParams;
};
typedef struct DtlsNetworkContext DtlsNetworkContext_t;


// DtlsKeyingMaterial is information extracted via https://tools.ietf.org/html/rfc5705
// also includes the use_srtp value from Handshake
typedef struct
{
    uint8_t clientWriteKey[MAX_SRTP_MASTER_KEY_LEN + MAX_SRTP_SALT_KEY_LEN];
    uint8_t serverWriteKey[MAX_SRTP_MASTER_KEY_LEN + MAX_SRTP_SALT_KEY_LEN];
    uint8_t key_length;

    KVS_SRTP_PROFILE srtpProfile;
} DtlsKeyingMaterial, * pDtlsKeyingMaterial_t;


/**
 * @brief Contains the credentials necessary for tls connection setup.
 */
typedef struct DtlsNetworkCredentials
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
     * @brief Disable server name indication (SNI) for a (D)TLS session.
     */
    int disableSni;

    const uint8_t * pRootCa;     /**< @brief String representing a trusted server root certificate. */
    size_t rootCaSize;          /**< @brief Size associated with #NetworkCredentials.pRootCa. */
    mbedtls_x509_crt * pClientCert;             /**< @brief Client certificate context. */
    mbedtls_pk_context * pPrivateKey;

    DtlsKeyingMaterial dtlsKeyingMaterial; /**< @brief derivated SRTP keys */
} DtlsNetworkCredentials_t;

typedef struct DtlsSession {
    DtlsNetworkContext_t xNetworkContext;
    DtlsTransportParams_t xDtlsTransportParams;
    DtlsNetworkCredentials_t xNetworkCredentials;
    uint8_t isServer;
} DtlsSession_t;

/**
 * @brief DTLS Connect / Disconnect return status.
 */
typedef enum DtlsTransportStatus
{
    DTLS_SUCCESS = 0,                                /**< Function successfully completed. */

    /* Common error code. */
    DTLS_INVALID_PARAMETER,                          /**< At least one parameter was invalid. */
    DTLS_OUT_OF_MEMORY,                              /**< Fail to allocate memory by malloc. */

    /* Transport error code */
    DTLS_TRANSPORT_INSUFFICIENT_MEMORY,              /**< Insufficient memory required to establish connection. */
    DTLS_TRANSPORT_INVALID_CREDENTIALS,              /**< Provided credentials were invalid. */
    DTLS_TRANSPORT_HANDSHAKE_FAILED,                 /**< Performing TLS handshake with server failed. */
    DTLS_TRANSPORT_INTERNAL_ERROR,                   /**< A call to a system API resulted in an internal error. */
    DTLS_TRANSPORT_CONNECT_FAILURE,                  /**< Initial connection to the server failed. */
    DTLS_TRANSPORT_PROCESS_FAILURE,                  /**< Fail while processing received packet. */

    /* Error code for key and certificate generation. */
    DTLS_INITIALIZE_PK_FAILURE,                      /**< Fail to initialize SSL context before generating RSA key. */
    DTLS_GENERATE_KEY_FAILURE,                       /**< Fail to generate SSL key. */
    DTLS_SET_CERT_ISSUER_NAME_FAILURE,               /**< Fail to set issuer name. */
    DTLS_SET_CERT_VALIDITY_FAILURE,                  /**< Fail to set validity. */
    DTLS_WRITE_CERT_CRT_DER_FAILURE,                 /**< Fail to write X509 crt der. */
    DTLS_PARSE_CERT_DER_FAILURE,                     /**< Fail to parse X509 der. */
    DTLS_SET_CERT_SERIAL_FAILURE,                    /**< Fail to set cert serial. */
    DTLS_GENERATE_TIMESTAMP_STRING_FAILURE,          /**< Fail to generate timestamp string. */
    DTLS_READ_BINARY_FAILURE,                        /**< Fail to read binary. */
    DTLS_GENERATE_RANDOM_BITS_FAILURE,               /**< Fail to generate random bits. */

    DTLS_SSL_REMOTE_CERTIFICATE_VERIFICATION_FAILED, /**< The remote certificate failed verification. */
    DTLS_SSL_UNKNOWN_SRTP_PROFILE,                   /**< The SRTP profile is unknown. */

    /* User info. */
    DTLS_HANDSHAKE_COMPLETE,                         /**< Just complete the DTLS handshaking. */
    DTLS_HANDSHAKE_ALREADY_COMPLETE,                 /**< DTLS handshaking is done before calling. */
    DTLS_CONNECTION_HAS_BEEN_CLOSED,                 /**< The DTLS connection has been closed. */
} DtlsTransportStatus_t;

#define DTLS_RSA_F4 0x10001L

#define PRIVATE_KEY_PCS_PEM_SIZE  228

#define GENERATED_CERTIFICATE_MAX_SIZE 4096
#define GENERATED_CERTIFICATE_BITS 2048
#define DTLS_CERT_MIN_SERIAL_NUM_SIZE 8
#define DTLS_CERT_MAX_SERIAL_NUM_SIZE 20
#define GENERATED_CERTIFICATE_DAYS 365
#define DTLS_SECONDS_IN_A_DAY ( 86400 )
#define GENERATED_CERTIFICATE_NAME "KVS-WebRTC-Client"
#define KEYING_EXTRACTOR_LABEL "EXTRACTOR-dtls_srtp"

/////////////////////////////////////////////////////
/// DTLS related status codes
/////////////////////////////////////////////////////

/**
 * @brief Initialise DTLS network context with provided credentials
 *
 * @param[in] pNetworkContext The DTLS network context.
 * @param[in] pNetworkCredentials The DTLS network credential.
 * @param[in] isServer Boolean flag indicating the DTLS role:
 *                     - 0: Initialize as DTLS client
 *                     - 1: Initialize as DTLS server
 *
 * @return DtlsTransportStatus_t Returns the status of the initialization:
 *         - DTLS_SUCCESS if initialization is successful
 *         - Other specific error codes in case of failure
 */
DtlsTransportStatus_t DTLS_Init( DtlsNetworkContext_t * pNetworkContext,
                                 DtlsNetworkCredentials_t * pNetworkCredentials,
                                 uint8_t isServer );

/**
 * @brief Gracefully disconnect an established DTLS connection.
 *
 * @param[in] pNetworkContext Network context.
 */
void DTLS_Disconnect( DtlsNetworkContext_t * pNetworkContext );

/**
 * @brief Sends data over an established DTLS connection.
 *
 * @note This is the DTLS version of the transport interface's
 * #TransportSend_t function.
 *
 * @param[in] pNetworkContext The network context.
 * @param[in] pBuffer Buffer containing the bytes to send.
 * @param[in] bytesToSend Number of bytes to send from the buffer.
 *
 * @return Number of bytes (> 0) sent on success;
 * 0 if the socket times out without sending any bytes;
 * else a negative value to represent error.
 */
int32_t DTLS_Send( DtlsNetworkContext_t * pNetworkContext,
                   const void * pBuffer,
                   size_t bytesToSend );

/**
 * @brief Get the socket FD for this network context.
 *
 * @param[in] pNetworkContext The network context.
 *
 * @return The socket descriptor if value >= 0. It returns -1 when failure.
 */
int32_t DTLS_GetSocketFd( DtlsNetworkContext_t * pNetworkContext );

/**
 * @brief Process a received packet in an established DTLS session.
 *
 * @param[in] pNetworkContext The DTLS network context.
 * @param[in] pDtlsPacket Pointer to the received DTLS packet.
 * @param[in] dtlsPacketLength The length of the received DTLS encrypted packet.
 * @param[out] readBuffer The buffer to store the decrypted DTLS packet.
 * @param[in,out] pReadBufferSize The size of the buffer. It will be updated
 * to the size of the decrypted packet.
 *
 * @return DtlsTransportStatus_t Returns the status of the initialization:
 *         - DTLS_SUCCESS if the packet was successfully processed
 *         - DTLS_HANDSHAKE_COMPLETE if the handshake is completed
 *         - Other specific error codes in case of failure
 */
DtlsTransportStatus_t DTLS_ProcessPacket( DtlsNetworkContext_t * pNetworkContext,
                                          void * pDtlsPacket,
                                          size_t dtlsPacketLength,
                                          uint8_t * readBuffer,
                                          size_t * pReadBufferSize );

/**
 * @brief Execute DTLS handshaking.
 *
 * @param[in] pNetworkContext The DTLS network context.
 *
 * @return DtlsTransportStatus_t Returns the status of handshaking:
 *         - DTLS_SUCCESS if DTLS handshaking is still in-progress
 *         - DTLS_HANDSHAKE_COMPLETE if handshake is just completed
 *         - DTLS_HANDSHAKE_ALREADY_COMPLETE if handshake has been completed before invoking.
 *         - Other specific error codes in case of failure
 */
DtlsTransportStatus_t DTLS_ExecuteHandshake( DtlsNetworkContext_t * pNetworkContext );

/**
 * @brief Generates a new certificate and a key.
 *
 * @param[in] certificateBits The bit number to generate RSA certificate.
 *            It's used only when generating RSA certificate.
 * @param[in] generateRSACertificate To generate RSA or ECDSA certificate.
 *            - 1 if generating RSA certificate
 *            - 0 if generating ECDSA certificate
 * @param[out] pCert The DTLS certificate generated.
 * @param[out] pKey The DTLS key generated.
 *
 * @return DtlsTransportStatus_t Returns the status of the initialization:
 *         - DTLS_SUCCESS if DTLS handshaking is still in-progress
 *         - DTLS_HANDSHAKE_COMPLETE if the handshake is completed
 *         - Other specific error codes in case of failure
 */
int32_t DTLS_CreateCertificateAndKey( int32_t certificateBits,
                                      BaseType_t generateRSACertificate,
                                      mbedtls_x509_crt * pCert,
                                      mbedtls_pk_context * pKey );

/**
 * @brief Free certificate and key
 *
 * @param[in] pCert The DTLS certificate to be freed.
 * @param[in] pKey The DTLS key to be freed.
 *
 * @return DtlsTransportStatus_t Returns the status of the initialization:
 *         - DTLS_SUCCESS if DTLS certificate and key are freed.
 *         - Other specific error codes in case of failure
 */
int32_t DTLS_FreeCertificateAndKey( mbedtls_x509_crt * pCert,
                                    mbedtls_pk_context * pKey );

/**
 * @brief Generates a fingerprint of the certificate.
 *
 * @param[in] pCert The DTLS certificate.
 * @param[out] pBuff The buffer of generated fingerprint.
 * @param[in] bufLen The maximum size of input buffer.
 *
 * @return DtlsTransportStatus_t Returns the status of the initialization:
 *         - DTLS_SUCCESS if fingerprint is created successfully.
 *         - Other specific error codes in case of failure
 */
int32_t DTLS_CreateCertificateFingerprint( const mbedtls_x509_crt * pCert,
                                           char * pBuff,
                                           const size_t bufLen );

/**
 * @brief Verify the fingerprint of certificate.
 *
 * @param[in] pSslContext The DTLS SSL context storing remote certificate.
 * @param[in] pExpectedFingerprint The expected certificate fingerprint.
 * @param[in] fingerprintMaxLen The size of expected certificate fingerprint.
 *
 * @return DtlsTransportStatus_t Returns the status of the initialization:
 *         - DTLS_SUCCESS if the fingerprint of remote certificate matches the expected fingerprint.
 *         - Other specific error codes in case of failure
 */
int32_t DTLS_VerifyRemoteCertificateFingerprint( DtlsSSLContext_t * pSslContext,
                                                 char * pExpectedFingerprint,
                                                 const size_t fingerprintMaxLen );

/**
 * @brief Populate key material of DTLS session.
 *
 * @param[in] pSslContext The target DTLS SSL context passing handshake.
 * @param[out] pDtlsKeyingMaterial The key material.
 *
 * @return DtlsTransportStatus_t Returns the status of the initialization:
 *         - DTLS_SUCCESS if the key is retrieved successfully.
 *         - Other specific error codes in case of failure
 */
int32_t DTLS_PopulateKeyingMaterial( DtlsSSLContext_t * pSslContext,
                                     pDtlsKeyingMaterial_t pDtlsKeyingMaterial );

#endif /* ifndef TRANSPORT_DTLS_MBEDTLS_H */
