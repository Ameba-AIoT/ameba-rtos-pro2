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

#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

/*
 * Minimal demo configuration for local_webrtc_mmf example.
 * This file provides the necessary defines for the shared WebRTC
 * protocol stack components (peer_connection, ice_controller, etc.).
 */

/* Video codec */
#define USE_VIDEO_CODEC_H264 1
#define USE_VIDEO_CODEC_H265 0

/* Audio codec (only one should be set) */
#define AUDIO_OPUS       1
#define AUDIO_G711_MULAW 0
#define AUDIO_G711_ALAW  0

/* Features */
#define ENABLE_TWCC_SUPPORT       0
#define MEDIA_PORT_ENABLE_AUDIO_RECV 1
#define ENABLE_SCTP_DATA_CHANNEL  0
#define METRIC_PRINT_ENABLED      0

/* Max concurrent viewers */
#define AWS_MAX_VIEWER_NUM        1

/* AWS placeholders (unused — no cloud services required) */
#define AWS_REGION            "local"
#define AWS_KVS_CHANNEL_NAME  "local"
#define AWS_KVS_AGENT_NAME    "LocalWebRTC"

/* CA Cert placeholder (unused — required only by shared code that references the macro).
 * The local_webrtc example does NOT use this value: it zeros pcConfig via memset,
 * so pRootCaPem remains NULL and mbedtls_x509_crt_parse() is never called on it.
 * If it were parsed, mbedTLS would return MBEDTLS_ERR_PEM_NO_HEADER_FOOTER_PRESENT
 * on empty input — not success. Keep the macro defined for compilation only. */
#define AWS_CA_CERT_PEM \
    ""

/*
 * IPv6 compatibility for lwIP-2.1.2 (AmebaPro2 SDK).
 * The shared ICE controller code uses INET6_ADDRSTRLEN unconditionally,
 * but this SDK's lwIP doesn't define it. We provide it here.
 * Also ensure LWIP_IPV6 is 1 so struct sockaddr_in6 is defined.
 */
#ifndef LWIP_IPV6
#define LWIP_IPV6 1
#endif
#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 46
#endif

#endif /* DEMO_CONFIG_H */
