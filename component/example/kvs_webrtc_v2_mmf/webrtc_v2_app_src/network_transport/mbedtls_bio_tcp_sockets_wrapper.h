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

#ifndef MBEDTLS_BIO_TCP_SOCKETS_WRAPPER
#define MBEDTLS_BIO_TCP_SOCKETS_WRAPPER

/**
 * @brief Sends data over TCP socket.
 *
 * @param[in] ctx The network context containing the socket handle.
 * @param[in] buf Buffer containing the bytes to send.
 * @param[in] len Number of bytes to send from the buffer.
 *
 * @return Number of bytes sent on success; else a negative value.
 */
int xMbedTLSBioTCPSocketsWrapperSend( void * ctx,
                                      const unsigned char * buf,
                                      size_t len );

/**
 * @brief Receives data from TCP socket.
 *
 * @param[in] ctx The network context containing the socket handle.
 * @param[out] buf Buffer to receive bytes into.
 * @param[in] len Number of bytes to receive from the network.
 *
 * @return Number of bytes received if successful; Negative value on error.
 */
int xMbedTLSBioTCPSocketsWrapperRecv( void * ctx,
                                      unsigned char * buf,
                                      size_t len );


#endif /* MBEDTLS_BIO_TCP_SOCKETS_WRAPPER */
