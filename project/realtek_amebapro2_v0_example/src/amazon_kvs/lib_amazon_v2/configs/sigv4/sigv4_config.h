/*
 * SigV4 Library v1.3.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file sigv4_config.h
 * @brief The default values for configuration macros used by the SigV4 Library.
 *
 * @note This file should NOT be modified. If custom values are needed for any
 * configuration macros, a sigv4_config.h file should be provided to the SigV4
 * Library to override the default values defined in this file. To use
 * the custom config file, the preprocessor macro SIGV4_DO_NOT_USE_CUSTOM_CONFIG
 * must NOT be set.
 */

#ifndef SIGV4_CONFIG_H_
#define SIGV4_CONFIG_H_

#define SIGV4_PROCESSING_BUFFER_LENGTH    2048U

#endif /* ifndef SIGV4_CONFIG_H_ */
