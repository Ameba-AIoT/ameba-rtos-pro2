# This cmake file is used to include Signaling as static library.
set(CMAKE_SIGNALING_DIRECTORY ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-signaling)
include( ${CMAKE_SIGNALING_DIRECTORY}/signalingFilePaths.cmake )

add_library( signaling )

target_sources( signaling
    PRIVATE
        ${SIGNALING_SOURCES}
    PUBLIC
        ${SIGNALING_INCLUDE_PUBLIC_DIRS}
)

target_include_directories( signaling PUBLIC
                            ${SIGNALING_INCLUDE_PUBLIC_DIRS} )

target_link_libraries( signaling PRIVATE
                       corejson )

target_compile_definitions( signaling PUBLIC SIGNALING_DO_NOT_USE_CUSTOM_CONFIG )

### add linked library ###
list(
    APPEND app_example_lib
    signaling
)
