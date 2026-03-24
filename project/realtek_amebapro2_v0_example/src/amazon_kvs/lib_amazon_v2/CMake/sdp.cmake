# This cmake file is used to include SDP as static library.
set(CMAKE_SDP_DIRECTORY ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-sdp)
include( ${CMAKE_SDP_DIRECTORY}/sdpFilePaths.cmake )

add_library( sdp )

target_sources( sdp
    PRIVATE
        ${SDP_SOURCES}
    PUBLIC
        ${SDP_INCLUDE_PUBLIC_DIRS}
)

target_include_directories( sdp PUBLIC
                            ${SDP_INCLUDE_PUBLIC_DIRS} )

target_compile_definitions( sdp PUBLIC SDP_DO_NOT_USE_CUSTOM_CONFIG )

### add linked library ###
list(
    APPEND app_example_lib
    sdp
)
