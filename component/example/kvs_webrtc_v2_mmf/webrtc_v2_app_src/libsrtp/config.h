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

#ifndef LIBSRTP_CONFIG_H
#define LIBSRTP_CONFIG_H
/* clang-format off */

/* Define to the full name and version of this package. */
#define PACKAGE_VERSION "3.0.0-pre"

/* Define to the version of this package. */
#define PACKAGE_STRING "libsrtp3 3.0.0-pre"

/* Define to enabled debug logging for all mudules. */
/* #undef ENABLE_DEBUG_LOGGING */

/* Logging statments will be writen to this file. */
/* #undef ERR_REPORTING_FILE */

/* Define to redirect logging to stdout. */
/* #undef ERR_REPORTING_STDOUT */

/* Define this to use OpenSSL crypto. */
/* #undef OPENSSL */

/* Define this to use AES-GCM. */
#define GCM 1

/* Define this to use MBEDTLS. */
#define MBEDTLS 1

/* Define if building for a CISC machine (e.g. Intel). */
/* #undef CPU_CISC */

/* Define if building for a RISC machine (assume slow byte access). */
#define CPU_RISC 1

/* Define to use X86 inlined assembly code */
/* #undef HAVE_X86 */

/* Define WORDS_BIGENDIAN to 1 if your processor stores words with the most
   significant byte first (like Motorola and SPARC, unlike Intel). */
/* #undef WORDS_BIGENDIAN */

/* Define to 1 if you have the <arpa/inet.h> header file. */
/* #undef HAVE_ARPA_INET_H */

/* Define to 1 if you have the <byteswap.h> header file. */
/* #undef HAVE_BYTESWAP_H */

/* Define to 1 if you have the <inttypes.h> header file. */
/* #undef HAVE_INTTYPES_H */

/* Define to 1 if you have the <machine/types.h> header file. */
/* #undef HAVE_MACHINE_TYPES_H */

/* Define to 1 if you have the <netinet/in.h> header file. */
/* #undef HAVE_NETINET_IN_H */

/* Define to 1 if you have the <stdint.h> header file. */
#define HAVE_STDINT_H 1

/* Define to 1 if you have the <stdlib.h> header file. */
#define HAVE_STDLIB_H 1

/* Define to 1 if you have the <sys/int_types.h> header file. */
/* #undef HAVE_SYS_INT_TYPES_H */

/* Define to 1 if you have the <sys/socket.h> header file. */
/* #undef HAVE_SYS_SOCKET_H */

/* Define to 1 if you have the <sys/types.h> header file. */
/* #undef HAVE_SYS_TYPES_H */

/* Define to 1 if you have the <unistd.h> header file. */
/* #undef HAVE_UNISTD_H */

/* Define to 1 if you have the <windows.h> header file. */
/* #undef HAVE_WINDOWS_H */

/* Define to 1 if you have the <winsock2.h> header file. */
/* #undef HAVE_WINSOCK2_H */

/* Define to 1 if you have the `inet_aton' function. */
/* #undef HAVE_INET_ATON */

/* Define to 1 if you have the `sigaction' function. */
/* #undef HAVE_SIGACTION */

/* Define to 1 if you have the `usleep' function. */
/* #undef HAVE_USLEEP */

/* Define to 1 if the system has the type `uint8_t'. */
#define HAVE_UINT8_T 1

/* Define to 1 if the system has the type `uint16_t'. */
#define HAVE_UINT16_T 1

/* Define to 1 if the system has the type `uint32_t'. */
#define HAVE_UINT32_T 1

/* Define to 1 if the system has the type `uint64_t'. */
#define HAVE_UINT64_T 1

/* Define to 1 if the system has the type `int32_t'. */
#define HAVE_INT32_T 1

/* The size of `unsigned long', as computed by sizeof. */
#define SIZEOF_UNSIGNED_LONG 4

/* The size of `unsigned long long', as computed by sizeof. */
#define SIZEOF_UNSIGNED_LONG_LONG 8

/* Define inline to what is supported by compiler  */
#define HAVE_INLINE 1
/* #undef HAVE___INLINE */
#ifndef HAVE_INLINE
  #ifdef HAVE___INLINE
    #define inline __inline
  #else
    #define inline
  #endif
#endif

/* Remove debugging. */
#define debug_print0( mod, format )
#define debug_print( mod, format, arg )
#define debug_print2( mod, format, arg1, arg2 )

#include <lwip/def.h>

/* Map malloc and free to FreeRTOS versions. */
#include "FreeRTOS.h"
#define malloc  pvPortMalloc
#define free    vPortFree

#endif /* #ifndef LIBSRTP_CONFIG_H */
