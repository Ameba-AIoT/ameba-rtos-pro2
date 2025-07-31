cmake_minimum_required(VERSION 3.6)

project(kvs_webrtc)

set(kvs_webrtc kvs_webrtc)

file(GLOB WEBRTC_STATE_MACHINE_SOURCE_FILES ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/state_machine/*.c)
file(GLOB WEBRTC_UTILS_SOURCE_FILES ${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/utils/*.c)
file(
	GLOB
	WEBRTC_CLIENT_SOURCE_FILES
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/crypto/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/ice/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Json/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/net/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/PeerConnection/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtcp/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtp/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtp/Codecs/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Sdp/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/srtp/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/stun/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/sctp/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Metrics/*.c
)
list(FILTER WEBRTC_CLIENT_SOURCE_FILES EXCLUDE REGEX ".*_openssl\\.c")
file(
	GLOB
	WEBRTC_SIGNALING_CLIENT_SOURCE_FILES
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/credential/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/api_call/*.c
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/signaling/*.c
)

list(
	APPEND kvs_webrtc_sources
	${WEBRTC_STATE_MACHINE_SOURCE_FILES}
	${WEBRTC_UTILS_SOURCE_FILES}
	${WEBRTC_CLIENT_SOURCE_FILES}
	${WEBRTC_SIGNALING_CLIENT_SOURCE_FILES}
)

add_library(
	${kvs_webrtc} STATIC
	${kvs_webrtc_sources}
)

list(
	APPEND webrtc_netio_func_rename_flag
	NetIo_create=webrtc_NetIo_create
	NetIo_terminate=webrtc_NetIo_terminate
	NetIo_connect=webrtc_NetIo_connect
	NetIo_connectWithX509=webrtc_NetIo_connectWithX509
	NetIo_connectWithX509Path=webrtc_NetIo_connectWithX509Path
	NetIo_disconnect=webrtc_NetIo_disconnect
	NetIo_send=webrtc_NetIo_send
	NetIo_recv=webrtc_NetIo_recv
	NetIo_isDataAvailable=webrtc_NetIo_isDataAvailable
	NetIo_setRecvTimeout=webrtc_NetIo_setRecvTimeout
	NetIo_setSendTimeout=webrtc_NetIo_setSendTimeout
	NetIo_getSocket=webrtc_NetIo_getSocket
)

list(
	APPEND webrtc_netio_func_rename_flag
	wss_client_close=webrtc_wss_client_close
)

list(
	APPEND webrtc_kvs_string_rename_flag
	strnchr=webrtc_strnchr
)

list(
	APPEND kvs_webrtc_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	CONFIG_SYSTEM_TIME64=1

	KVS_USE_MBEDTLS
	KVS_PLAT_RTK_FREERTOS
	BUILD_CLIENT
	ENABLE_STREAMING
	${wslay_flags}

	${webrtc_netio_func_rename_flag}
	${webrtc_kvs_string_rename_flag}
)
if(USE_DATA_CHANNEL)
list(
	APPEND kvs_webrtc_flags
	ENABLE_DATA_CHANNEL
	${usrsctp_flags}
)
endif()

target_compile_definitions(${kvs_webrtc} PRIVATE ${kvs_webrtc_flags} )
target_compile_options(${kvs_webrtc} PRIVATE ${LIBS_WARN_ERR_FLAGS} )

include(../includepath.cmake)
target_include_directories(
	${kvs_webrtc}
	PUBLIC

	${inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure

	${prj_root}/src/amazon_kvs/lib_amazon/gcc_include
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/include
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/api_call
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/credential
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/crypto
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/ice
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Json
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Metrics
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/net
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/PeerConnection
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtcp
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtp
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/sctp
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Sdp
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/signaling
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/srtp
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/state_machine
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/stun
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/utils
	${prj_root}/src/amazon_kvs/lib_amazon/amazon-kinesis-video-streams-webrtc-sdk-c/src/source/Rtp/Codecs
	${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/include
	${prj_root}/src/amazon_kvs/lib_amazon/llhttp/include
	${prj_root}/src/amazon_kvs/lib_amazon/wslay/lib/includes
	${prj_root}/src/amazon_kvs/lib_amazon/usrsctp/usrsctplib
	${sdk_root}/component/example/kvs_webrtc_mmf
)