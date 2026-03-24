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

/* Include header that defines log levels. */
#include "logging.h"

/* Standard includes. */
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"

/* LWIP includes */
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "errno.h"

/* TCP Sockets Wrapper include.*/
#include "tcp_sockets_wrapper.h"

/* configASSERT() using stuff in task.h */
#include "task.h"

static BaseType_t ConfigureTimeout( Socket_t xSocket,
                                    uint32_t receiveTimeoutMs,
                                    uint32_t sendTimeoutMs )
{
    BaseType_t xRet = TCP_SOCKETS_ERRNO_NONE;
    int fcntlFlags = 0;
    uint32_t realReceiveTimeoutMs = receiveTimeoutMs;
    uint32_t realSendTimeoutMs = sendTimeoutMs;
    int setsockoptResult = 0;

    if( ( receiveTimeoutMs == 0 ) &&
        ( sendTimeoutMs == 0 ) )
    {
        fcntlFlags = fcntl( xSocket->xFd, F_GETFL, 0 );
        if( fcntlFlags < 0 )
        {
            LogError( ( "fcntl() failed with errno: %d", errno ) );
            xRet = TCP_SOCKETS_ERRNO_ERROR;
        }
        else
        {
            fcntlFlags |= O_NONBLOCK;
            if( fcntl( xSocket->xFd, F_SETFL, fcntlFlags ) < 0 )
            {
                LogError( ( "fcntl() failed with errno: %d", errno ) );
                xRet = TCP_SOCKETS_ERRNO_ERROR;
            }
        }
    }
    else
    {
        /* If the timeout is not 0, disable the non-blocking flag in socket handler. */
        fcntlFlags = fcntl( xSocket->xFd, F_GETFL, 0 );
        if( fcntlFlags < 0 )
        {
            LogError( ( "fcntl() failed with errno: %d", errno ) );
            xRet = TCP_SOCKETS_ERRNO_ERROR;
        }
        else
        {
            fcntlFlags &= ~O_NONBLOCK;
            if( fcntl( xSocket->xFd, F_SETFL, fcntlFlags ) < 0 )
            {
                LogError( ( "fcntl() failed with errno: %d", errno ) );
                xRet = TCP_SOCKETS_ERRNO_ERROR;
            }
        }

        if( ( receiveTimeoutMs == TCP_SOCKETS_TIMEOUT_INFINITE ) &&
            ( sendTimeoutMs == TCP_SOCKETS_TIMEOUT_INFINITE ) )
        {
            /* Set the timeout to 0 as waiting infinitely. */
            realReceiveTimeoutMs = 0U;
            realSendTimeoutMs = 0U;
        }

        setsockoptResult = setsockopt( xSocket->xFd, SOL_SOCKET, SO_RCVTIMEO, &realReceiveTimeoutMs, sizeof( realReceiveTimeoutMs ) );
        if( setsockoptResult < 0 )
        {
            LogError( ( "setsockopt() failed to set receive timeout. Error code: %d", errno ) );
            xRet = TCP_SOCKETS_ERRNO_ERROR;
        }

        setsockoptResult = setsockopt( xSocket->xFd, SOL_SOCKET, SO_SNDTIMEO, &realSendTimeoutMs, sizeof( realSendTimeoutMs ) );
        if( setsockoptResult < 0 )
        {
            LogError( ( "setsockopt() failed to set send timeout. Error code: %d", errno ) );
            xRet = TCP_SOCKETS_ERRNO_ERROR;
        }
    }

    return xRet;
}

/**
 * @brief Establish a connection to server.
 *
 * @param[out] pTcpSocket The output parameter to return the created socket descriptor.
 * @param[in] pHostName Server hostname to connect to.
 * @param[in] pServerInfo Server port to connect to.
 * @param[in] receiveTimeoutMs Timeout (in milliseconds) for transport receive.
 * @param[in] sendTimeoutMs Timeout (in milliseconds) for transport send.
 *
 * @note A timeout of 0 means infinite timeout.
 *
 * @return Non-zero value on error, 0 on success.
 */
BaseType_t TCP_Sockets_Connect( Socket_t * pTcpSocket,
                                const char * pHostName,
                                uint16_t port,
                                uint32_t receiveTimeoutMs,
                                uint32_t sendTimeoutMs )
{
    int xFd = -1;
    BaseType_t xRet = TCP_SOCKETS_ERRNO_NONE;
    struct addrinfo xHints, * pxAddrList, * pxCur;
    char xPortStr[6];

    memset( &xHints, 0, sizeof( xHints ) );
    xHints.ai_family = AF_UNSPEC;
    xHints.ai_socktype = SOCK_STREAM;
    xHints.ai_protocol = IPPROTO_TCP;
    ( void ) snprintf( xPortStr, sizeof( xPortStr ), "%d", port );
    if( getaddrinfo( pHostName, xPortStr, &xHints, &pxAddrList ) != 0 )
    {
        LogError( ( "Failed to connect to server: DNS resolution failed: Hostname=%s.",
                    pHostName ) );
        return TCP_SOCKETS_ERRNO_ERROR;
    }

    /* Try the sockaddrs until a connection succeeds */
    xRet = TCP_SOCKETS_ERRNO_ERROR;
    for( pxCur = pxAddrList; pxCur != NULL; pxCur = pxCur->ai_next )
    {
        xFd = socket( pxCur->ai_family, pxCur->ai_socktype,
                      pxCur->ai_protocol );
        if( xFd < 0 )
        {
            LogError( ( "Failed to create new socket, Error code: %d", errno ) );
            xRet = TCP_SOCKETS_ERRNO_ENOMEM;
            continue;
        }

        if( connect( xFd, pxCur->ai_addr, pxCur->ai_addrlen ) == 0 )
        {
            xRet = TCP_SOCKETS_ERRNO_NONE;
            LogDebug( ( "Established TCP connection with %s.", pHostName ) );
            break;
        }

        close( xFd );
        xRet = TCP_SOCKETS_ERRNO_ERROR;
    }

    freeaddrinfo( pxAddrList );

    if( xRet == TCP_SOCKETS_ERRNO_NONE )
    {
        *pTcpSocket = pvPortMalloc( sizeof( struct xSOCKET ) );
        if( *pTcpSocket == NULL )
        {
            LogError( ( "Failed to allow new socket context." ) );
            ( void )close( xFd );
            xRet = TCP_SOCKETS_ERRNO_ENOMEM;
        }
        else
        {
            ( *pTcpSocket )->xFd = xFd;
        }
    }

    if( xRet == TCP_SOCKETS_ERRNO_NONE )
    {
        xRet = ConfigureTimeout( *pTcpSocket, receiveTimeoutMs, sendTimeoutMs );
    }

    return xRet;
}

/**
 * @brief End connection to server.
 *
 * @param[in] tcpSocket The socket descriptor.
 */
void TCP_Sockets_Disconnect( Socket_t tcpSocket )
{
    ( void )shutdown( tcpSocket->xFd, SHUT_RDWR );
    ( void )close( tcpSocket->xFd );
    vPortFree( tcpSocket );
}

/**
 * @brief Transmit data to the remote socket.
 *
 * The socket must have already been created using a call to TCP_Sockets_Connect().
 *
 * @param[in] xSocket The handle of the sending socket.
 * @param[in] pvBuffer The buffer containing the data to be sent.
 * @param[in] xDataLength The length of the data to be sent.
 *
 * @return
 * * On success, the number of bytes actually sent is returned.
 * * If an error occurred, a negative value is returned. @ref SocketsErrors
 */
int32_t TCP_Sockets_Send( Socket_t xSocket,
                          const void * pvBuffer,
                          size_t xBufferLength )
{
    int xWriteRet;
    int xReturnStatus;

    configASSERT( xSocket != NULL );
    configASSERT( pvBuffer != NULL );

    xWriteRet = write( xSocket->xFd, pvBuffer, xBufferLength );
    if( xWriteRet >= 0 )
    {
        xReturnStatus = xWriteRet;
    }
    else
    {
        switch( errno )
        {
            case EAGAIN:
            case EINTR:
            case ENOMEM:
            case ENOSPC:
                xReturnStatus = TCP_SOCKETS_ERRNO_EWOULDBLOCK;
                break;
            case EPIPE:
            case ECONNRESET:
                xReturnStatus = TCP_SOCKETS_ERRNO_ENOTCONN;
                break;
            default:
                LogInfo( ( "error code %d, %s", errno, strerror( errno ) ) );
                xReturnStatus = TCP_SOCKETS_ERRNO_ERROR;
                break;
        }
    }
    return xReturnStatus;
}

/**
 * @brief Receive data from a TCP socket.
 *
 * The socket must have already been created using a call to TCP_Sockets_Connect().
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
int32_t TCP_Sockets_Recv( Socket_t xSocket,
                          void * pvBuffer,
                          size_t xBufferLength )
{
    int xReadRet;
    int xReturnStatus;

    configASSERT( xSocket != NULL );
    configASSERT( pvBuffer != NULL );

    xReadRet = read( xSocket->xFd, pvBuffer, xBufferLength );
    if( xReadRet >= 0 )
    {
        xReturnStatus = xReadRet;
    }
    else
    {
        switch( errno )
        {
            case EWOULDBLOCK:
            case EINTR:
                xReturnStatus = 0;
                break;
            case EPIPE:
            case ECONNRESET:
                xReturnStatus = TCP_SOCKETS_ERRNO_ENOTCONN;
                break;
            default:
                LogInfo( ( "error code %d, %s", errno, strerror( errno ) ) );
                xReturnStatus = TCP_SOCKETS_ERRNO_ERROR;
                break;
        }
    }
    return xReturnStatus;
}

int32_t TCP_Sockets_GetSocketFd( Socket_t xSocket )
{
    int32_t ret = -1;

    if( xSocket != NULL )
    {
        ret = xSocket->xFd;
    }

    return ret;
}

BaseType_t TCP_Sockets_ConfigureTimeout( Socket_t xSocket,
                                         uint32_t receiveTimeoutMs,
                                         uint32_t sendTimeoutMs )
{
    BaseType_t xRet = TCP_SOCKETS_ERRNO_NONE;

    if( xSocket == NULL )
    {
        xRet = TCP_SOCKETS_ERRNO_EINVAL;
    }

    if( xRet == TCP_SOCKETS_ERRNO_NONE )
    {
        xRet = ConfigureTimeout( xSocket, receiveTimeoutMs, sendTimeoutMs );
    }

    return xRet;
}
