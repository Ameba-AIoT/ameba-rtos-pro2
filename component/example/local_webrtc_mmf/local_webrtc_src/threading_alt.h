/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 AmebaPro2 WebRTC Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THREADING_ALT_H
#define THREADING_ALT_H

#include "FreeRTOS.h"
#include "semphr.h"

typedef struct {
	SemaphoreHandle_t mutex;
	char is_valid;
} mbedtls_threading_mutex_t;

extern void mbedtls_threading_set_alt(void (*mutex_init)(mbedtls_threading_mutex_t *),
									  void (*mutex_free)(mbedtls_threading_mutex_t *),
									  int (*mutex_lock)(mbedtls_threading_mutex_t *),
									  int (*mutex_unlock)(mbedtls_threading_mutex_t *));

#endif /* THREADING_ALT_H */
