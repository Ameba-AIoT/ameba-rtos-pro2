# SPDX-License-Identifier: Apache-2.0
#
# Copyright 2026 AmebaPro2 WebRTC Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.6.3)

# ========================================================================== #
# Local WebRTC MMF Example — Build Configuration
# ========================================================================== #
#
# This example reuses the WebRTC protocol stack from
# component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/ and the
# open-source Amazon KVS WebRTC protocol libraries.
#
# Prerequisites:
#   Run component/example/kvs_webrtc_v2_mmf/fetch_aws_lib.sh first to download
#   the Amazon KVS WebRTC protocol libraries.
#
# Build:
#   cd project/realtek_amebapro2_v0_example/GCC-RELEASE
#   mkdir build && cd build
#   cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
#            -DEXAMPLE=local_webrtc_mmf
#   cmake --build . --target flash -j4
# ========================================================================== #

# --- Paths to shared WebRTC app sources (from kvs_webrtc_v2_mmf) ---------
set(WEBRTC_APP_SRC  "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src")
set(WEBRTC_V2_EX    "${sdk_root}/component/example/kvs_webrtc_v2_mmf")
set(LOCAL_SRC       "${sdk_root}/component/example/local_webrtc_mmf")
set(LOCAL_APP_SRC   "${LOCAL_SRC}/local_webrtc_src")
set(AMAZON_LIB      "${prj_root}/src/amazon_kvs/lib_amazon_v2")

# --- Ensure the Amazon KVS protocol libraries are downloaded -----------------
# These are open-source protocol implementations (ICE, STUN, SRTP, RTP, SDP)
# from the Amazon KVS WebRTC SDK (https://github.com/awslabs/amazon-kinesis-
# video-streams-webrtc-sdk-c), used WITHOUT connecting to any AWS service.
#
# The libraries directory is gitignored, so a fresh clone won't have them.
# Auto-fetch at cmake configure time; skip the example gracefully on failure
# so the overall build (or BVT for other examples) is not blocked.
if(NOT EXISTS "${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-ice/source")
    message(STATUS "Amazon KVS WebRTC libraries not found."
                   " Attempting auto-fetch from upstream...")
    execute_process(
        COMMAND bash "${sdk_root}/component/example/kvs_webrtc_v2_mmf/fetch_aws_lib.sh"
        WORKING_DIRECTORY "${sdk_root}"
        RESULT_VARIABLE _aws_fetch_result
        ERROR_VARIABLE  _aws_fetch_error
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _aws_fetch_result EQUAL 0)
        message(WARNING
            "\n"
            "==========================================================================\n"
            "Amazon KVS WebRTC protocol libraries could not be auto-fetched.\n"
            "To download them manually:\n"
            "\n"
            "  cd component/example/kvs_webrtc_v2_mmf\n"
            "  ./fetch_aws_lib.sh\n"
            "\n"
            "The local_webrtc_mmf example will be SKIPPED in this build.\n"
            "==========================================================================\n")
        return()
    endif()
endif()

# --- Check that mbedTLS version is 3.x ---------------------------------------
# The shared WebRTC DTLS code calls mbedtls 3.x specific APIs:
#   mbedtls_ssl_config_free(), mbedtls_ssl_conf_dtls_srtp_protection_profiles(),
#   mbedtls_ssl_get_dtls_srtp_protection_profile(), etc.
# These do not exist in mbedtls-2.x.
# The user should set mbedtls to "mbedtls-3.0.0" in config.cmake.
# We do NOT modify config.cmake — we check and skip gracefully.
if(NOT "${mbedtls}" MATCHES "^mbedtls-3")
    message(WARNING
        "\n"
        "==========================================================================\n"
        "mbedTLS version ${mbedtls} is not supported.\n"
        "The local_webrtc_mmf example requires mbedtls-3.0.0 for DTLS-SRTP support.\n"
        "\n"
        "To fix, set the following in project/realtek_amebapro2_v0_example/"
        "GCC-RELEASE/config.cmake:\n"
        "\n"
        "  set(mbedtls \"mbedtls-3.0.0\")\n"
        "\n"
        "The local_webrtc_mmf example will be SKIPPED in this build.\n"
        "==========================================================================\n")
    return()
endif()

# --- FreeRTOS TCP sockets wrapper config ----------------------------------
add_definitions(-DHAVE_SA_LEN -DHAVE_SIN_LEN -DHAVE_SCONN_LEN -DSCTP_USE_LWIP)

# --- Compile definitions --------------------------------------------------
list(APPEND app_example_flags
    ENABLE_SCTP_DATA_CHANNEL=0
    ENABLE_TWCC_SUPPORT=0
    METRIC_PRINT_ENABLED=0
    ENABLE_STREAMING_LOOPBACK=0
    HAVE_CONFIG_H
    AUDIO_OPUS=1
    AUDIO_G711_MULAW=0
    AUDIO_G711_ALAW=0
    USE_VIDEO_CODEC_H264=1
    USE_VIDEO_CODEC_H265=0
    MEDIA_PORT_ENABLE_AUDIO_RECV=1
    LWIP_IPV6=1
    # Needed by mbedtls-3.0.0/library/aes_alt.c for hardware AES DMA.
    # Normally defined in the Realtek "mbedtls/config.h" wrapper, but we
    # use MBEDTLS_CONFIG_FILE to provide our own config and do NOT include
    # the Realtek wrapper (which also defines CONFIG_SSL_RESUME, causing a
    # heap corruption boot crash on this platform).
    RTL_CRYPTO_FRAGMENT=65536
    # DTLS cookie support in the SDK's mbedtls-3.0.0 fork is guarded by
    # CONFIG_SSL_RESUME. Without it, mbedtls_ssl_conf_dtls_cookies() is
    # compiled out and the DTLS handshake fails with INVALID_MAC.
    CONFIG_SSL_RESUME=1
)

# mbedTLS compatibility: provide old macro names for mbedtls-3.x
add_definitions(
    -DMBEDTLS_SRTP_AES128_CM_HMAC_SHA1_80=MBEDTLS_TLS_SRTP_AES128_CM_HMAC_SHA1_80
    -DMBEDTLS_SRTP_AES128_CM_HMAC_SHA1_32=MBEDTLS_TLS_SRTP_AES128_CM_HMAC_SHA1_32
)

# --------------------------------------------------------------------------
# SDK configuration: non-destructive overrides
#
# The following use CMake-native mechanisms (configure_file, compile
# definitions) instead of modifying SDK source files in-place. This avoids
# side effects on other projects that build against the same SDK.
# --------------------------------------------------------------------------

# --- lwIP: LWIP_IPV6 must be 1 for WebRTC ICE controller -------------------
# The SDK's lwipopts.h has a plain "#define LWIP_IPV6 0" (no #ifndef guard),
# so -DLWIP_IPV6=1 from compile definitions is overridden by the header.
# Solution: create a patched copy in the build directory and ensure it is
# found before the SDK original via include_directories(BEFORE ...).
set(_lwipopts_src  "${sdk_root}/component/lwip/api/lwipopts.h")
set(_lwipopts_patched "${CMAKE_CURRENT_BINARY_DIR}/lwip_patched/lwipopts.h")
file(READ "${_lwipopts_src}" _lwipopts_content)
string(REGEX REPLACE "#define LWIP_IPV6[ \t]+0" "#define LWIP_IPV6 1" _lwipopts_content "${_lwipopts_content}")
file(WRITE "${_lwipopts_patched}" "${_lwipopts_content}")
# Prepend the patched directory so lwIP picks up our lwipopts.h first
include_directories(BEFORE "${CMAKE_CURRENT_BINARY_DIR}/lwip_patched")

# --- mbedTLS: disable MBEDTLS_FS_IO (no <dirent.h> on ARM bare-metal) ------
# MBEDTLS_FS_IO pulls in <dirent.h> which is not supported on ARM bare-metal
# newlib. We use a custom config file that wraps the default config and
# undefines MBEDTLS_FS_IO, passed via MBEDTLS_CONFIG_FILE.
# fs_io_stubs.c provides stubs for any remaining FS_IO symbol references.
list(APPEND app_example_flags
    "MBEDTLS_CONFIG_FILE=\"local_mbedtls_config.h\""
)


# --------------------------------------------------------------------------
# Build the Amazon KVS protocol libraries as CMake targets
# --------------------------------------------------------------------------

# --- ICE library ---
set(CMAKE_ICE_DIR "${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-ice")
file(GLOB ICE_SOURCES "${CMAKE_ICE_DIR}/source/*.c")
set(ICE_INCLUDE_PUBLIC_DIRS "${CMAKE_ICE_DIR}/source/include")
add_library(ice ${ICE_SOURCES})
target_include_directories(ice PUBLIC ${ICE_INCLUDE_PUBLIC_DIRS})
target_link_libraries(ice PUBLIC stun corejson)

# --- STUN library ---
set(CMAKE_STUN_DIR "${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-stun")
file(GLOB STUN_SOURCES "${CMAKE_STUN_DIR}/source/*.c")
set(STUN_INCLUDE_PUBLIC_DIRS "${CMAKE_STUN_DIR}/source/include")
add_library(stun ${STUN_SOURCES})
target_include_directories(stun PUBLIC ${STUN_INCLUDE_PUBLIC_DIRS})

# --- SDP library ---
set(CMAKE_SDP_DIR "${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-sdp")
file(GLOB SDP_SOURCES "${CMAKE_SDP_DIR}/source/*.c")
set(SDP_INCLUDE_PUBLIC_DIRS "${CMAKE_SDP_DIR}/source/include")
add_library(sdp ${SDP_SOURCES})
target_include_directories(sdp PUBLIC ${SDP_INCLUDE_PUBLIC_DIRS})
target_compile_definitions(sdp PUBLIC SDP_DO_NOT_USE_CUSTOM_CONFIG)

# --- STUN library (also needed as direct dependency) ---
# (already defined above as 'stun')

# --- RTP library ---
set(CMAKE_RTP_DIR "${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-rtp")
file(GLOB_RECURSE RTP_MAIN_SOURCES "${CMAKE_RTP_DIR}/source/*.c")
file(GLOB RTP_CODEC_SOURCES "${CMAKE_RTP_DIR}/codec_packetizers/*/*.c")
set(RTP_SOURCES ${RTP_MAIN_SOURCES} ${RTP_CODEC_SOURCES})
set(RTP_INCLUDE_PUBLIC_DIRS
    "${CMAKE_RTP_DIR}/source/include"
    "${CMAKE_RTP_DIR}/codec_packetizers/g711/include"
    "${CMAKE_RTP_DIR}/codec_packetizers/h264/include"
    "${CMAKE_RTP_DIR}/codec_packetizers/h265/include"
    "${CMAKE_RTP_DIR}/codec_packetizers/opus/include"
    "${CMAKE_RTP_DIR}/codec_packetizers/vp8/include"
)
add_library(rtp ${RTP_SOURCES})
target_include_directories(rtp PUBLIC ${RTP_INCLUDE_PUBLIC_DIRS})

# --- coreJSON library (ICE candidate parsing) ---
set(CMAKE_COREJSON_DIR "${AMAZON_LIB}/libraries/coreJSON")
file(GLOB COREJSON_SOURCES "${CMAKE_COREJSON_DIR}/source/*.c")
set(COREJSON_INCLUDE_PUBLIC_DIRS "${CMAKE_COREJSON_DIR}/source/include")
add_library(corejson ${COREJSON_SOURCES})
target_include_directories(corejson PUBLIC ${COREJSON_INCLUDE_PUBLIC_DIRS})
target_compile_definitions(corejson PUBLIC LIBRARY_LOG_LEVEL=LOG_NONE)

# --- RTCP library ---
set(CMAKE_RTCP_DIR "${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-rtcp")
file(GLOB RTCP_SOURCES "${CMAKE_RTCP_DIR}/source/*.c")
set(RTCP_INCLUDE_PUBLIC_DIRS "${CMAKE_RTCP_DIR}/source/include")
add_library(rtcp ${RTCP_SOURCES})
target_include_directories(rtcp PUBLIC ${RTCP_INCLUDE_PUBLIC_DIRS})

# --- libsrtp library ---
set(LIBSRTP_DIR "${AMAZON_LIB}/libraries/libsrtp")
file(GLOB LIBSRTP_GLOB_SOURCES
    "${LIBSRTP_DIR}/crypto/kernel/*.c"
    "${LIBSRTP_DIR}/crypto/math/*.c"
    "${LIBSRTP_DIR}/crypto/replay/*.c"
    "${LIBSRTP_DIR}/srtp/*.c"
)
set(LIBSRTP_SOURCES
    ${LIBSRTP_GLOB_SOURCES}
    "${LIBSRTP_DIR}/crypto/cipher/aes.c"
    "${LIBSRTP_DIR}/crypto/cipher/aes_gcm_mbedtls.c"
    "${LIBSRTP_DIR}/crypto/cipher/aes_icm_mbedtls.c"
    "${LIBSRTP_DIR}/crypto/cipher/cipher.c"
    "${LIBSRTP_DIR}/crypto/cipher/cipher_test_cases.c"
    "${LIBSRTP_DIR}/crypto/cipher/null_cipher.c"
    "${LIBSRTP_DIR}/crypto/hash/auth_test_cases.c"
    "${LIBSRTP_DIR}/crypto/hash/auth.c"
    "${LIBSRTP_DIR}/crypto/hash/hmac.c"
    "${LIBSRTP_DIR}/crypto/hash/hmac_mbedtls.c"
    "${LIBSRTP_DIR}/crypto/hash/null_auth.c"
    "${LIBSRTP_DIR}/crypto/hash/sha1.c"
)
set(LIBSRTP_INCLUDE_PUBLIC_DIRS
    "${WEBRTC_APP_SRC}/libsrtp"
    "${LIBSRTP_DIR}/include"
    "${LIBSRTP_DIR}/crypto/include"
)
# Need ${inc_path} for SDK headers (lwIP, FreeRTOS, mbedTLS, etc.)
include(${prj_root}/GCC-RELEASE/includepath.cmake)

add_library(libsrtp ${LIBSRTP_SOURCES})
target_compile_definitions(libsrtp PRIVATE HAVE_CONFIG_H)
target_include_directories(libsrtp PUBLIC
    ${LIBSRTP_INCLUDE_PUBLIC_DIRS}
    ${inc_path}
    ${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure
)

# --- DCEP library (data channel) ---
set(CMAKE_DCEP_DIR "${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-dcep")
file(GLOB DCEP_SOURCES "${CMAKE_DCEP_DIR}/source/*.c")
set(DCEP_INCLUDE_PUBLIC_DIRS "${CMAKE_DCEP_DIR}/source/include")
add_library(dcep ${DCEP_SOURCES})
target_include_directories(dcep PUBLIC ${DCEP_INCLUDE_PUBLIC_DIRS})

# --- usrsctp library (SCTP data channels) ---
set(USRSCTP_DIR "${AMAZON_LIB}/libraries/usrsctp/usrsctplib")
set(USRSCTP_SOURCES
    ${USRSCTP_DIR}/netinet/sctp_asconf.c
    ${USRSCTP_DIR}/netinet/sctp_auth.c
    ${USRSCTP_DIR}/netinet/sctp_bsd_addr.c
    ${USRSCTP_DIR}/netinet/sctp_callout.c
    ${USRSCTP_DIR}/netinet/sctp_cc_functions.c
    ${USRSCTP_DIR}/netinet/sctp_crc32.c
    ${USRSCTP_DIR}/netinet/sctp_indata.c
    ${USRSCTP_DIR}/netinet/sctp_input.c
    ${USRSCTP_DIR}/netinet/sctp_output.c
    ${USRSCTP_DIR}/netinet/sctp_pcb.c
    ${USRSCTP_DIR}/netinet/sctp_peeloff.c
    ${USRSCTP_DIR}/netinet/sctp_sha1.c
    ${USRSCTP_DIR}/netinet/sctp_ss_functions.c
    ${USRSCTP_DIR}/netinet/sctp_sysctl.c
    ${USRSCTP_DIR}/netinet/sctp_timer.c
    ${USRSCTP_DIR}/netinet/sctp_userspace.c
    ${USRSCTP_DIR}/netinet/sctp_usrreq.c
    ${USRSCTP_DIR}/netinet/sctputil.c
    ${USRSCTP_DIR}/netinet6/sctp6_usrreq.c
    ${USRSCTP_DIR}/user_environment.c
    ${USRSCTP_DIR}/user_mbuf.c
    ${USRSCTP_DIR}/user_recv_thread.c
    ${USRSCTP_DIR}/user_socket.c
)
set(USRSCTP_INCLUDE_DIRS
    ${USRSCTP_DIR}
    ${USRSCTP_DIR}/netinet
    ${USRSCTP_DIR}/netinet6
)
add_library(usrsctp STATIC ${USRSCTP_SOURCES})
target_compile_definitions(usrsctp PUBLIC
    __Userspace__
    SCTP_SIMPLE_ALLOCATOR
    SCTP_PROCESS_LEVEL_LOCKS
    SCTP_USE_MBEDTLS_SHA1
    SCTP_USE_LWIP
    SCTP_USE_RTOS
    INET
    HAVE_SA_LEN HAVE_SIN_LEN HAVE_SCONN_LEN
    KVS_PLAT_RTK_FREERTOS
)
target_include_directories(usrsctp PRIVATE
    ${AMAZON_LIB}/gcc_include
    ${sdk_root}/component/os/freertos/freertos_posix/lib/include
)
target_include_directories(usrsctp PUBLIC
    ${USRSCTP_INCLUDE_DIRS}
    ${inc_path}
    ${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure
)

# --- Amazon library list ------------------------------------------------
list(APPEND app_example_lib
    ice stun sdp rtp rtcp libsrtp dcep usrsctp corejson
)

# --- Include paths -------------------------------------------------------
include_directories(
    # Local
    ${LOCAL_SRC}
    ${LOCAL_APP_SRC}

    # Shared WebRTC app sources
    ${WEBRTC_APP_SRC}
    ${WEBRTC_APP_SRC}/peer_connection
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_codec_helper
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_codec_helper/include
    ${WEBRTC_APP_SRC}/network_transport
    ${WEBRTC_APP_SRC}/network_transport/tcp_sockets_wrapper/include
    ${WEBRTC_APP_SRC}/network_transport/udp_sockets_wrapper/include
    ${WEBRTC_APP_SRC}/networking
    ${WEBRTC_APP_SRC}/networking/corehttp_helper
    ${WEBRTC_APP_SRC}/networking/wslay_helper
    ${WEBRTC_APP_SRC}/networking/networking_utils
    ${WEBRTC_APP_SRC}/logging
    ${WEBRTC_APP_SRC}/message_queue
    ${WEBRTC_APP_SRC}/base64
    ${WEBRTC_APP_SRC}/sdp_controller
    ${WEBRTC_APP_SRC}/string_utils
    ${WEBRTC_APP_SRC}/ice_controller
    ${WEBRTC_APP_SRC}/timer_controller
    ${WEBRTC_APP_SRC}/app_media_source
    ${WEBRTC_APP_SRC}/app_media_source/port/ameba_pro2
    ${WEBRTC_V2_EXAMPLE}

    # Video example headers
    ${prj_root}/src/mmfv2_video_example
)

# NOTE: Amazon library include paths are NOT in global include_directories()
# to avoid shadowing system headers for other SDK components.
# They are handled via:
#   - target_include_directories() on each library target (for compiling the library)
#   - app_example_inc_path (for compiling the example app sources)

# --- App Example include paths (duplicated for the build system) ----------
list(APPEND app_example_inc_path
    ${LOCAL_SRC}
    ${LOCAL_APP_SRC}
    ${WEBRTC_APP_SRC}
    ${WEBRTC_APP_SRC}/peer_connection
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_codec_helper
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_codec_helper/include
    ${WEBRTC_APP_SRC}/network_transport
    ${WEBRTC_APP_SRC}/network_transport/tcp_sockets_wrapper/include
    ${WEBRTC_APP_SRC}/network_transport/udp_sockets_wrapper/include
    ${WEBRTC_APP_SRC}/networking
    ${WEBRTC_APP_SRC}/networking/corehttp_helper
    ${WEBRTC_APP_SRC}/networking/wslay_helper
    ${WEBRTC_APP_SRC}/networking/networking_utils
    ${WEBRTC_APP_SRC}/logging
    ${WEBRTC_APP_SRC}/message_queue
    ${WEBRTC_APP_SRC}/base64
    ${WEBRTC_APP_SRC}/sdp_controller
    ${WEBRTC_APP_SRC}/string_utils
    ${WEBRTC_APP_SRC}/ice_controller
    ${WEBRTC_APP_SRC}/timer_controller
    ${WEBRTC_APP_SRC}/app_media_source
    ${WEBRTC_APP_SRC}/app_media_source/port/ameba_pro2
    ${WEBRTC_V2_EXAMPLE}
    # gcc_include NOT added globally — added PRIVATE to usrsctp target only

    # coreHTTP headers (needed by networking_utils.h in shared code)
    ${AMAZON_LIB}/libraries/coreHTTP/source/include
    ${AMAZON_LIB}/libraries/coreHTTP/source/interface
    ${AMAZON_LIB}/libraries/coreJSON/source/include
    ${AMAZON_LIB}/libraries/crypto/SigV4-for-AWS-IoT-embedded-sdk/source/include

    # Config dirs (needed for coreHTTP, sigv4, etc.)
    ${AMAZON_LIB}/configs/corehttp
    ${AMAZON_LIB}/configs/sigv4
    ${AMAZON_LIB}/configs/wslay

    # wslay (WebSocket library — needed by some shared headers)
    ${AMAZON_LIB}/libraries/wslay/lib/
    ${AMAZON_LIB}/libraries/wslay/lib/includes
    ${AMAZON_LIB}/libraries/wslay/lib/includes/wslay

    # Amazon KVS protocol library headers
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-ice/source/include
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-sdp/source/include
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-sdp/test/coverity
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-stun/source/include
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-rtp/source/include
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-rtp/codec_packetizers/h264/include
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-rtp/codec_packetizers/h265/include
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-rtp/codec_packetizers/opus/include
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-rtp/codec_packetizers/g711/include
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-rtp/codec_packetizers/vp8/include
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-rtcp/source/include
    ${AMAZON_LIB}/libraries/components/amazon-kinesis-video-streams-dcep/source/include
    ${AMAZON_LIB}/libraries/libsrtp/include
    ${AMAZON_LIB}/libraries/libsrtp/crypto/include
    ${AMAZON_LIB}/libraries/usrsctp/usrsctplib
    ${AMAZON_LIB}/libraries/usrsctp/usrsctplib/netinet
    ${AMAZON_LIB}/libraries/usrsctp/usrsctplib/netinet6
    ${prj_root}/src/mmfv2_video_example
)

# --- Source files ---------------------------------------------------------

# App example entry point
list(APPEND app_example_sources
    ${LOCAL_SRC}/app_example.c
    ${LOCAL_SRC}/example_local_webrtc_mmf.c
)

# Local WebRTC module & application
list(APPEND out_sources
    ${LOCAL_SRC}/module_local_webrtc.c
    ${LOCAL_APP_SRC}/local_master.c
    ${LOCAL_APP_SRC}/local_app_common.c
    ${LOCAL_APP_SRC}/ameba_pro2_local_media_port.c
    ${LOCAL_APP_SRC}/fs_io_stubs.c
)

# --- Shared WebRTC peer connection layer (from kvs_webrtc_v2_mmf) --------
list(APPEND out_sources
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection.c
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_jitter_buffer.c
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_rolling_buffer.c
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_sctp.c
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_sdp.c
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_srtcp.c
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_srtp.c
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_codec_helper/peer_connection_g711_helper.c
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_codec_helper/peer_connection_h264_helper.c
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_codec_helper/peer_connection_h265_helper.c
    ${WEBRTC_APP_SRC}/peer_connection/peer_connection_codec_helper/peer_connection_opus_helper.c
)

# --- Shared network transport (DTLS with mbedTLS) ------------------------
list(APPEND out_sources
    ${WEBRTC_APP_SRC}/network_transport/transport_dtls_mbedtls.c
    ${WEBRTC_APP_SRC}/network_transport/transport_dtls_mbedtls_port.c
    ${WEBRTC_APP_SRC}/network_transport/transport_mbedtls.c
    ${WEBRTC_APP_SRC}/network_transport/mbedtls_bio_tcp_sockets_wrapper.c
    ${WEBRTC_APP_SRC}/network_transport/mbedtls_bio_udp_sockets_wrapper.c
    ${WEBRTC_APP_SRC}/network_transport/tcp_sockets_wrapper/ports/lwip/tcp_sockets_wrapper.c
    ${WEBRTC_APP_SRC}/network_transport/udp_sockets_wrapper/ports/lwip/udp_sockets_wrapper.c
)

# --- Shared utilities ----------------------------------------------------
list(APPEND out_sources
    # NOTE: core_http_helper.c and wslay_helper.c are excluded intentionally.
    # They pull in the wslay WebSocket library (wslayver.h) which is only
    # needed for AWS KVS signaling — not used in this local WebRTC example.
    ${WEBRTC_APP_SRC}/networking/networking_utils/networking_utils.c
    ${WEBRTC_APP_SRC}/message_queue/message_queue.c
    ${WEBRTC_APP_SRC}/base64/mbedtls/base64_mbedtls.c
    ${WEBRTC_APP_SRC}/sdp_controller/sdp_controller.c
    ${WEBRTC_APP_SRC}/string_utils/string_utils.c
    ${WEBRTC_APP_SRC}/ice_controller/ice_controller.c
    ${WEBRTC_APP_SRC}/ice_controller/ice_controller_net.c
    ${WEBRTC_APP_SRC}/ice_controller/ice_controller_socket_listener.c
    ${WEBRTC_APP_SRC}/timer_controller/timer_controller.c
    ${WEBRTC_APP_SRC}/app_media_source/app_media_source.c
    ${WEBRTC_APP_SRC}/libusrsctp/sctp_utils.c
)
