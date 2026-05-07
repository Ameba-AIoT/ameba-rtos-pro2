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

#ifndef TIMER_CONTROLLER_H
#define TIMER_CONTROLLER_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include "timers.h"

typedef enum TimerControllerResult
{
    TIMER_CONTROLLER_RESULT_OK = 0,
    TIMER_CONTROLLER_RESULT_SET,
    TIMER_CONTROLLER_RESULT_NOT_SET,
    TIMER_CONTROLLER_RESULT_BAD_PARAMETER,
    TIMER_CONTROLLER_RESULT_FAIL_TIMER_CREATE,
    TIMER_CONTROLLER_RESULT_FAIL_TIMER_SET,
    TIMER_CONTROLLER_RESULT_FAIL_GETTIME,
} TimerControllerResult_t;

typedef void (* TimerControllerTimerExpireCallback)( void * pUserContext );

typedef struct TimerHandler
{
    TimerHandle_t timer;
    TimerControllerTimerExpireCallback onTimerExpire;
    void * pUserContext;
} TimerHandler_t;

TimerControllerResult_t TimerController_Create( TimerHandler_t * pTimerHandler,
                                                const char * pTimerName,
                                                uint32_t initialTimeMs,
                                                uint32_t repeatTimeMs,
                                                TimerControllerTimerExpireCallback onTimerExpire,
                                                void * pUserContext );
TimerControllerResult_t TimerController_SetTimer( TimerHandler_t * pTimerHandler,
                                                  uint32_t initialTimeMs,
                                                  uint32_t repeatTimeMs );
void TimerController_Reset( TimerHandler_t * pTimerHandler );
void TimerController_Delete( TimerHandler_t * pTimerHandler );
TimerControllerResult_t TimerController_IsTimerSet( TimerHandler_t * pTimerHandler );

#ifdef __cplusplus
}
#endif

#endif /* TIMER_CONTROLLER_H */
