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

#ifndef LOGGING_H
#define LOGGING_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Standard Include. */
#include <stdio.h>
#include <string.h>
#include "log_service.h"

#include "FreeRTOS.h"

#define LOG_NONE     0
#define LOG_ERROR    1
#define LOG_WARN     2
#define LOG_INFO     3
#define LOG_DEBUG    4
#define LOG_VERBOSE  5

#ifndef LIBRARY_LOG_LEVEL
    #define LIBRARY_LOG_LEVEL LOG_INFO
#endif

/* Metadata information to prepend to every log message. */
#ifndef LOG_METADATA_FORMAT
    #define LOG_METADATA_FORMAT    "[%s: %s: %d] "            /**< @brief Format of metadata prefix in log messages. */
#endif

#ifndef LOG_METADATA_ARGS
    #define LOG_METADATA_ARGS    pcTaskGetName(xTaskGetCurrentTaskHandle()), __FUNCTION__, __LINE__  /**< @brief Arguments into the metadata logging prefix format. */
#endif

#ifndef SdkLog
    #define SdkLog( message ) printf message
#endif

/**
 * Disable definition of logging interface macros when generating doxygen output,
 * to avoid conflict with documentation of macros at the end of the file.
 */
/* Check that LIBRARY_LOG_LEVEL is defined and has a valid value. */
#if !defined( LIBRARY_LOG_LEVEL ) ||       \
    ( ( LIBRARY_LOG_LEVEL != LOG_NONE ) && \
    ( LIBRARY_LOG_LEVEL != LOG_ERROR ) &&  \
    ( LIBRARY_LOG_LEVEL != LOG_WARN ) &&   \
    ( LIBRARY_LOG_LEVEL != LOG_INFO ) &&   \
    ( LIBRARY_LOG_LEVEL != LOG_DEBUG ) &&   \
    ( LIBRARY_LOG_LEVEL != LOG_VERBOSE ) )
    #error "Please define LIBRARY_LOG_LEVEL as either LOG_NONE, LOG_ERROR, LOG_WARN, LOG_INFO, or LOG_DEBUG."
#else
    #if LIBRARY_LOG_LEVEL == LOG_VERBOSE
/* All log level messages will logged. */
        #define LogError( message )    SdkLog( ( "[ERROR]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogWarn( message )     SdkLog( ( "[WARN]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogInfo( message )     SdkLog( ( "[INFO]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogDebug( message )    SdkLog( ( "[DEBUG]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogVerbose( message )    SdkLog( ( "[VERBOSE]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )

    #elif LIBRARY_LOG_LEVEL == LOG_DEBUG
/* All log level messages will logged. */
        #define LogError( message )    SdkLog( ( "[ERROR]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogWarn( message )     SdkLog( ( "[WARN]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogInfo( message )     SdkLog( ( "[INFO]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogDebug( message )    SdkLog( ( "[DEBUG]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogVerbose( message )

    #elif LIBRARY_LOG_LEVEL == LOG_INFO
/* Only INFO, WARNING and ERROR messages will be logged. */
        #define LogError( message )    SdkLog( ( "[ERROR]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogWarn( message )     SdkLog( ( "[WARN]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogInfo( message )     SdkLog( ( "[INFO]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogDebug( message )
        #define LogVerbose( message )

    #elif LIBRARY_LOG_LEVEL == LOG_WARN
/* Only WARNING and ERROR messages will be logged.*/
        #define LogError( message )    SdkLog( ( "[ERROR]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogWarn( message )     SdkLog( ( "[WARN]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogInfo( message )
        #define LogDebug( message )
        #define LogVerbose( message )

    #elif LIBRARY_LOG_LEVEL == LOG_ERROR
/* Only ERROR messages will be logged. */
        #define LogError( message )    SdkLog( ( "[ERROR]"LOG_METADATA_FORMAT, LOG_METADATA_ARGS ) ); SdkLog( message ); SdkLog( ( "\r\n" ) )
        #define LogWarn( message )
        #define LogInfo( message )
        #define LogDebug( message )
        #define LogVerbose( message )

    #else /* if LIBRARY_LOG_LEVEL == LOG_ERROR */

        #define LogError( message )
        #define LogWarn( message )
        #define LogInfo( message )
        #define LogDebug( message )
        #define LogVerbose( message )

    #endif /* if LIBRARY_LOG_LEVEL == LOG_ERROR */
#endif /* if !defined( LIBRARY_LOG_LEVEL ) || ( ( LIBRARY_LOG_LEVEL != LOG_NONE ) && ( LIBRARY_LOG_LEVEL != LOG_ERROR ) && ( LIBRARY_LOG_LEVEL != LOG_WARN ) && ( LIBRARY_LOG_LEVEL != LOG_INFO ) && ( LIBRARY_LOG_LEVEL != LOG_DEBUG ) ) */

#ifdef __cplusplus
}
#endif

#endif /* LOGGING_H */