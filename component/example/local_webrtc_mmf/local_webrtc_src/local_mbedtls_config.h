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

/*
 * Custom mbedTLS configuration used in place of the SDK's default config.
 *
 * HOW IT IS ACTIVATED
 * -------------------
 * No source file includes this file directly. Instead, local_webrtc_mmf.cmake
 * adds -DMBEDTLS_CONFIG_FILE="local_mbedtls_config.h" to the compile definitions
 * for all compilation units in this example. mbedTLS's own build_info.h detects
 * MBEDTLS_CONFIG_FILE and #include-s this file in place of its normal config.
 *
 * This is the standard mbedTLS hook for injecting custom configuration.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * MBEDTLS_FS_IO is disabled because <dirent.h> is not supported on ARM
 * bare-metal newlib.
 *
 * NOTE: we include "mbedtls/mbedtls_config.h" directly rather than the Realtek
 * "mbedtls/config.h" wrapper. The wrapper defines RTL_CRYPTO_FRAGMENT and
 * CONFIG_SSL_RESUME — these are provided via cmake compile definitions
 * instead (see local_webrtc_mmf.cmake) to avoid pulling in the entire Realtek
 * config context, which was found to cause a heap corruption boot crash
 * (imprecise data bus error, R2=0xdeadbeef) when combined with the
 * MBEDTLS_CONFIG_FILE mechanism.
 */

#include "mbedtls/mbedtls_config.h"

#undef MBEDTLS_FS_IO
