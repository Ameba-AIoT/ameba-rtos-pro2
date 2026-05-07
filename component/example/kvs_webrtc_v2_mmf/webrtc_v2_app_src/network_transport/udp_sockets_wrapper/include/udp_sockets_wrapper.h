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

#ifndef UDP_SOCKETS_WRAPPER_H
#define UDP_SOCKETS_WRAPPER_H


/* Standard includes. */
#include <stdint.h>

/* FreeRTOS Kernel includes. */
#include "FreeRTOS.h"

/* Error codes. */
#define UDP_SOCKETS_ERRNO_NONE                ( 0 )   /*!< No error. */
#define UDP_SOCKETS_ERRNO_ERROR               ( -1 )  /*!< Catch-all sockets error code. */
#define UDP_SOCKETS_ERRNO_EWOULDBLOCK         ( -2 )  /*!< A resource is temporarily unavailable. */
#define UDP_SOCKETS_ERRNO_ENOMEM              ( -3 )  /*!< Memory allocation failed. */
#define UDP_SOCKETS_ERRNO_EINVAL              ( -4 )  /*!< Invalid argument. */
#define UDP_SOCKETS_ERRNO_ENOPROTOOPT         ( -5 )  /*!< A bad option was specified . */
#define UDP_SOCKETS_ERRNO_ENOTCONN            ( -6 )  /*!< The supplied socket is not connected. */
#define UDP_SOCKETS_ERRNO_EISCONN             ( -7 )  /*!< The supplied socket is already connected. */
#define UDP_SOCKETS_ERRNO_ECLOSED             ( -8 )  /*!< The supplied socket has already been closed. */
#define UDP_SOCKETS_ERRNO_PERIPHERAL_RESET    ( -9 )  /*!< Communications peripheral has been reset. */
#define UDP_SOCKETS_ERRNO_ENOSPC              ( -10 ) /*!< No space left on device */
#define UDP_SOCKETS_ERRNO_EINTR               ( -11 ) /*!< Interrupted system call */

#ifndef SOCKET_T_TYPEDEFED
struct xSOCKET;
typedef struct xSOCKET * Socket_t;     /**< @brief Socket handle data type. */
#endif

/**
 * @brief Create an UDP socket descriptor.
 *
 * @param[out] pUdpSocket The output parameter to return the created socket descriptor.
 * @param[in] assignFd The existing socket descriptor to be assigned to pUdpSocket.
 *
 * @return Non-zero value on error, 0 on success.
 */
BaseType_t UDP_Sockets_CreateAndAssign( Socket_t * pUdpSocket,
                                        int assignFd );

/**
 * @brief Establish a connection to server.
 *
 * @param[out] pUdpSocket The output parameter to return the created socket descriptor.
 * @param[in] pHostName Server hostname to connect to.
 * @param[in] pServerInfo Server port to connect to.
 * @param[in] receiveTimeoutMs Timeout (in milliseconds) for transport receive.
 * @param[in] sendTimeoutMs Timeout (in milliseconds) for transport send.
 *
 * @note A timeout of 0 means infinite timeout.
 *
 * @return Non-zero value on error, 0 on success.
 */
BaseType_t UDP_Sockets_Connect( Socket_t * pUdpSocket,
                                const char * pHostName,
                                uint16_t port,
                                uint32_t receiveTimeoutMs,
                                uint32_t sendTimeoutMs );

/**
 * @brief End connection to server.
 *
 * @param[in] udpSocket The socket descriptor.
 */
void UDP_Sockets_Disconnect( Socket_t udpSocket );

/**
 * @brief Transmit data to the remote socket.
 *
 * The socket must have already been created using a call to UDP_Sockets_Connect().
 *
 * @param[in] xSocket The handle of the sending socket.
 * @param[in] pvBuffer The buffer containing the data to be sent.
 * @param[in] xDataLength The length of the data to be sent.
 *
 * @return
 * * On success, the number of bytes actually sent is returned.
 * * If an error occurred, a negative value is returned. @ref SocketsErrors
 */
int32_t UDP_Sockets_Send( Socket_t xSocket,
                          const void * pvBuffer,
                          size_t xDataLength );

/**
 * @brief Receive data from a UDP socket.
 *
 * The socket must have already been created using a call to UDP_Sockets_Connect().
 *
 * @param[in] xSocket The handle of the socket from which data is being received.
 * @param[out] pvBuffer The buffer into which the received data will be placed.
 * @param[in] xBufferLength The maximum number of bytes which can be received.
 * pvBuffer must be at least xBufferLength bytes long.
 *
 * @return
 * * If the receive was successful then the number of bytes received (placed in the
 *   buffer pointed to by pvBuffer) is returned.
 * * If a timeout occurred before data could be received then 0 is returned (timeout
 *   is set using @ref SOCKETS_SO_RCVTIMEO).
 * * If an error occurred, a negative value is returned. @ref SocketsErrors
 */
int32_t UDP_Sockets_Recv( Socket_t xSocket,
                          void * pvBuffer,
                          size_t xBufferLength );

/**
 * @brief Get socket descriptor from Socket_t structure.
 *
 * @param[in] xSocket The handle of the socket
 *
 * @return
 * * Return socket descriptor with value >= 0. Otherwise, it returns -1.
 */
int32_t UDP_Sockets_GetSocketFd( Socket_t xSocket );

#endif /* ifndef UDP_SOCKETS_WRAPPER_H */
