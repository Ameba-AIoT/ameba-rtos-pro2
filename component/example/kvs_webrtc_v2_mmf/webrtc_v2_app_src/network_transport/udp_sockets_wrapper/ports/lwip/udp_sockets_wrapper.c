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

/* UDP Sockets Wrapper include.*/
#include "udp_sockets_wrapper.h"

/* configASSERT() using stuff in task.h */
#include "task.h"

struct xSOCKET
{
    int xFd;
};

BaseType_t UDP_Sockets_CreateAndAssign( Socket_t * pUdpSocket,
                                        int assignFd )
{
    BaseType_t xRet = UDP_SOCKETS_ERRNO_NONE;

    if( pUdpSocket == NULL )
    {
        xRet = UDP_SOCKETS_ERRNO_EINVAL;
        LogError( ( "Invalid input, pUdpSocket: %p", pUdpSocket ) );
    }

    if( xRet == UDP_SOCKETS_ERRNO_NONE )
    {
        *pUdpSocket = pvPortMalloc( sizeof( struct xSOCKET ) );
        if( *pUdpSocket == NULL )
        {
            LogError( ( "Failed to allow new socket context." ) );
            xRet = UDP_SOCKETS_ERRNO_ENOMEM;
        }
        else
        {
            ( *pUdpSocket )->xFd = assignFd;
        }
    }

    return xRet;
}

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
                                uint32_t sendTimeoutMs )
{
    int xFd = -1;
    BaseType_t xRet = UDP_SOCKETS_ERRNO_NONE;
    struct addrinfo xHints, * pxAddrList, * pxCur;
    char xPortStr[6];

    memset( &xHints, 0, sizeof( xHints ) );
    xHints.ai_family = AF_UNSPEC;
    xHints.ai_socktype = SOCK_DGRAM;
    xHints.ai_protocol = IPPROTO_UDP;
    ( void ) snprintf( xPortStr, sizeof( xPortStr ), "%d", port );
    if( getaddrinfo( pHostName, xPortStr, &xHints, &pxAddrList ) != 0 )
    {
        LogError( ( "Failed to connect to server: DNS resolution failed: Hostname=%s.",
                    pHostName ) );
        return UDP_SOCKETS_ERRNO_ERROR;
    }

    /* Try the sockaddrs until a connection succeeds */
    xRet = UDP_SOCKETS_ERRNO_ERROR;
    for( pxCur = pxAddrList; pxCur != NULL; pxCur = pxCur->ai_next )
    {
        xFd = ( *pUdpSocket )->xFd;
        if( xFd < 0 )
        {
            LogError( ( "Failed to create new socket, Error code: %d", errno ) );
            xRet = UDP_SOCKETS_ERRNO_ENOMEM;
            continue;
        }

        if( connect( xFd, pxCur->ai_addr, pxCur->ai_addrlen ) == 0 )
        {
            xRet = UDP_SOCKETS_ERRNO_NONE;
            LogInfo( ( "Established UDP connection with %s.", pHostName ) );
            break;
        }
        else
        {
            LogInfo( ( "Connecting failed with %s.", pHostName ) );
        }

        // close(xFd);
        xRet = UDP_SOCKETS_ERRNO_ERROR;
    }

    freeaddrinfo( pxAddrList );

    // if (xRet == UDP_SOCKETS_ERRNO_NONE)
    // {
    //     *pUdpSocket = pvPortMalloc(sizeof(*pUdpSocket));
    //     if (*pUdpSocket == NULL)
    //     {
    //         LogError(("Failed to allow new socket context."));
    //         (void)close(xFd);
    //         xRet = UDP_SOCKETS_ERRNO_ENOMEM;
    //     }
    //     else
    //     {
    //         (*pUdpSocket)->xFd = xFd;
    //     }
    // }

    // if (xRet == UDP_SOCKETS_ERRNO_NONE)
    // {
    //     setsockopt( xFd, SOL_SOCKET, SO_RCVTIMEO, &receiveTimeoutMs, sizeof( receiveTimeoutMs ) );
    //     setsockopt( xFd, SOL_SOCKET, SO_SNDTIMEO, &sendTimeoutMs, sizeof( sendTimeoutMs ) );
    // }

    return xRet;
}

/**
 * @brief End connection to server.
 *
 * @param[in] udpSocket The socket descriptor.
 */
void UDP_Sockets_Disconnect( Socket_t udpSocket )
{
    ( void )shutdown( udpSocket->xFd, SHUT_RDWR );
    vPortFree( udpSocket );
}

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
            case EPIPE:
            case ECONNRESET:
                xReturnStatus = UDP_SOCKETS_ERRNO_ENOTCONN;
                break;
            default:
                xReturnStatus = UDP_SOCKETS_ERRNO_ERROR;
                break;
        }
    }
    return xReturnStatus;
}

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
                xReturnStatus = UDP_SOCKETS_ERRNO_ENOTCONN;
                break;
            default:
                xReturnStatus = UDP_SOCKETS_ERRNO_ERROR;
                break;
        }
    }
    return xReturnStatus;
}

int32_t UDP_Sockets_GetSocketFd( Socket_t xSocket )
{
    int32_t ret = -1;

    if( xSocket != NULL )
    {
        ret = xSocket->xFd;
    }

    return ret;
}
