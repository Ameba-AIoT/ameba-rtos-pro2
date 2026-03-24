# This cmake file is used to include rtcp as static library.
include(${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-rtcp/rtcpFilePaths.cmake)

add_library( rtcp )

target_sources( rtcp
    PRIVATE
        ${RTCP_SOURCES}
    PUBLIC
        ${RTCP_INCLUDE_PUBLIC_DIRS}
)

target_include_directories( rtcp PUBLIC
                            ${RTCP_INCLUDE_PUBLIC_DIRS} )

### add linked library ###
list(
    APPEND app_example_lib
    rtcp
)
