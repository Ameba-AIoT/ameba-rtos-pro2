cmake_minimum_required(VERSION 3.6.3)

option(ENABLE_STREAMING_LOOPBACK "Loopback the received frames to the remote peer" OFF)

# Option to control linking with usrsctp
option( BUILD_USRSCTP_LIBRARY "Enable linking with usrsctp" ON )

# Option to enable metric logging
option( METRIC_PRINT_ENABLED "Enable Metric print logging" OFF )

# Option to choose the target type, either master or viewer application
option( BUILD_VIEWER_APPLICATION "Build Viewer Application" OFF )

if( BUILD_VIEWER_APPLICATION )
	set( WEBRTC_APPLICATION_DEMO_TYPE "viewer" CACHE STRING "Build WebRTC Viewer Application" )
else()
	set( WEBRTC_APPLICATION_DEMO_TYPE "master" CACHE STRING "Build WebRTC Master Application" )
endif()
 
include_directories(
    "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/demo_config"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/configs/corehttp"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/configs/sigv4"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/configs/wslay"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/wslay/lib/"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/wslay/lib/includes"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/wslay/lib/includes/wslay"

    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-dcep/source/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-ice/source/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-rtcp/source/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-rtp/source/include"

    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-rtp/codec_packetizers/g711/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-rtp/codec_packetizers/h264/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-rtp/codec_packetizers/h265/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-rtp/codec_packetizers/opus/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-rtp/codec_packetizers/vp8/include"

    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-sdp/source/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-sdp/test/coverity"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-signaling/source/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-stun/source/include"

    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/coreHTTP/source/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/coreHTTP/source/interface"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/coreJSON/source/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/crypto/SigV4-for-AWS-IoT-embedded-sdk/source/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/include"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet"
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet6"
)

# Include dependencies
# Include coreHTTP
include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/coreHTTP.cmake )

# Include sigV4
include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/sigV4.cmake )

## Include coreJSON
include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/coreJSON.cmake )

## Include Signaling
include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/signaling.cmake )

# Include wslay
include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/wslay.cmake )

# Include SDP
include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/sdp.cmake )

# Include STUN
include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/stun.cmake )

# Include RTP
include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/rtp.cmake )

# Include RTCP
include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/rtcp.cmake )

# Include ICE
include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/ice.cmake )

# Include libsrtp
include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/libsrtp.cmake )

list(
	APPEND app_example_lib
	corehttp
     sigv4
	corejson
     signaling
	wslay
	sdp
     stun
     rtp
     rtcp
     ice
	libsrtp
)

if(BUILD_USRSCTP_LIBRARY)
     add_definitions(-DHAVE_SA_LEN -DHAVE_SIN_LEN -DHAVE_SCONN_LEN -DSCTP_USE_LWIP)
     # Include DCEP
     include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/dcep.cmake )
     # Include usrsctp
     include( ${prj_root}/src/amazon_kvs/lib_amazon_v2/CMake/usrsctp.cmake )

     list(
          APPEND app_example_flags
          ENABLE_SCTP_DATA_CHANNEL=1
     )
else()
     list(
          APPEND app_example_flags
          ENABLE_SCTP_DATA_CHANNEL=0
     )
endif()

### add header files ###
list (
	APPEND app_example_inc_path
	"${prj_root}/src/amazon_kvs/lib_amazon_v2/gcc_include"
	"${prj_root}/src/mmfv2_video_example"
	"${sdk_root}/component/example/kvs_webrtc_v2_mmf"
	"${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_codec_helper"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_codec_helper/include"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/signaling_controller"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/network_transport"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/network_transport/tcp_sockets_wrapper/include"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/network_transport/udp_sockets_wrapper/include"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/networking"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/networking/corehttp_helper"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/networking/wslay_helper"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/networking/networking_utils"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/logging"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/message_queue"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/base64"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/sdp_controller"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/string_utils"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/ice_controller"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/timer_controller"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/app_media_source"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/app_media_source/port/ameba_pro2"
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/demo_config"
)

### add source file ###
list(
	APPEND app_example_sources
	${sdk_root}/component/example/kvs_webrtc_v2_mmf/app_example.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/example_kvs_webrtc_v2_mmf.c
)

list(
	APPEND out_sources
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/module_kvs_webrtc_v2.c

     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_jitter_buffer.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_rolling_buffer.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_sctp.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_sdp.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_srtcp.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_srtp.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_codec_helper/peer_connection_g711_helper.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_codec_helper/peer_connection_h264_helper.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_codec_helper/peer_connection_h265_helper.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/peer_connection/peer_connection_codec_helper/peer_connection_opus_helper.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/signaling_controller/signaling_controller.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/network_transport/mbedtls_bio_tcp_sockets_wrapper.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/network_transport/mbedtls_bio_udp_sockets_wrapper.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/network_transport/transport_dtls_mbedtls.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/network_transport/transport_dtls_mbedtls_port.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/network_transport/transport_mbedtls.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/network_transport/tcp_sockets_wrapper/ports/lwip/tcp_sockets_wrapper.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/network_transport/udp_sockets_wrapper/ports/lwip/udp_sockets_wrapper.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/networking/corehttp_helper/core_http_helper.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/networking/networking_utils/networking_utils.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/networking/wslay_helper/wslay_helper.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/message_queue/message_queue.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/base64/mbedtls/base64_mbedtls.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/sdp_controller/sdp_controller.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/string_utils/string_utils.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/ice_controller/ice_controller.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/ice_controller/ice_controller_net.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/ice_controller/ice_controller_socket_listener.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/timer_controller/timer_controller.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/app_media_source/app_media_source.c
     ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/app_media_source/port/ameba_pro2/ameba_pro2_media_port.c
)

if( BUILD_USRSCTP_LIBRARY )
    file( GLOB USRSCTP_SRC_FILES ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/libusrsctp/sctp_utils.c)
    list( APPEND out_sources ${USRSCTP_SRC_FILES} )
    list( APPEND app_example_inc_path "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/libusrsctp" )
endif()

    
if( ${WEBRTC_APPLICATION_DEMO_TYPE} STREQUAL "master" )
    message( STATUS "Building Master Application" )
    file( GLOB WEBRTC_APPLICATION_MASTER_SOURCE_FILES
          ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/master/master.c
          ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/app_common/app_common.c)
    list( APPEND out_sources
          ${WEBRTC_APPLICATION_MASTER_SOURCE_FILES} )
    list( APPEND app_example_inc_path
          "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/master"
          "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/app_common" )
elseif( ${WEBRTC_APPLICATION_DEMO_TYPE} STREQUAL "viewer" )
    message( STATUS "Building Viewer Application" )
    file( GLOB WEBRTC_APPLICATION_VIEWER_SOURCE_FILES
          ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/viewer/viewer.c
          ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/app_common/app_common.c)
    list( APPEND out_sources
          ${WEBRTC_APPLICATION_VIEWER_SOURCE_FILES} )
    list( APPEND app_example_inc_path
          "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/viewer"
          "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/app_common" )
endif()
 
if( METRIC_PRINT_ENABLED )
     file( GLOB METRIC_SRC_FILES ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/metric/metric.c)
     list( APPEND out_sources ${METRIC_SRC_FILES} )
     list( APPEND app_example_inc_path "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/metric" )
endif()

if(METRIC_PRINT_ENABLED)
     list(
          APPEND app_example_flags
          METRIC_PRINT_ENABLED=1
     )
else()
     list(
          APPEND app_example_flags
          METRIC_PRINT_ENABLED=0
     )
endif()

if( ENABLE_STREAMING_LOOPBACK )
     list(
          APPEND app_example_flags
          ENABLE_STREAMING_LOOPBACK
     )
endif()