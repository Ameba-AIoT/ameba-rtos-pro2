# This cmake file is used to include coreHTTP as static library.
set(CMAKE_corehttp_DIRECTORY ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/coreHTTP)
include( ${CMAKE_corehttp_DIRECTORY}/httpFilePaths.cmake )

add_library( corehttp )

target_sources( corehttp
    PRIVATE
        ${HTTP_SOURCES}
    PUBLIC
        ${HTTP_INCLUDE_PUBLIC_DIRS}
)

target_include_directories( corehttp PUBLIC
                            ${HTTP_INCLUDE_PUBLIC_DIRS}
                            ${prj_root}/src/amazon_kvs/lib_amazon_v2/configs/corehttp
                            ${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/demo_config # to get demo_config.h definition
)

# Suppress warnings for some Libraries
file(GLOB_RECURSE HTTP_WARNING_SUPPRESSED_SOURCES
    "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/coreHTTP/source/dependency/3rdparty/llhttp/src/llhttp.c"
)

set_source_files_properties(
    ${HTTP_WARNING_SUPPRESSED_SOURCES}
    PROPERTIES
    COMPILE_FLAGS "-w"
)

### add linked library ###
list(
    APPEND app_example_lib
    corehttp
)
