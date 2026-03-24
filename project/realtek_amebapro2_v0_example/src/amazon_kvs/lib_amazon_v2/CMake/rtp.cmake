# This cmake file is used to include rtp as static library.
include(${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-rtp/rtpFilePaths.cmake)

add_library( rtp )

target_sources( rtp
    PRIVATE
        ${RTP_SOURCES}
    PUBLIC
        ${RTP_INCLUDE_PUBLIC_DIRS}
)

target_include_directories( rtp PUBLIC
                            ${RTP_INCLUDE_PUBLIC_DIRS} )

### add linked library ###
list(
    APPEND app_example_lib
    rtp
)
