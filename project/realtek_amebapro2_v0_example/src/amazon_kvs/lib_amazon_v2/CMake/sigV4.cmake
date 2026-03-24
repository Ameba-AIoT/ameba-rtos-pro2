# This cmake file is used to include SigV4 as static library.
set(CMAKE_SIGV4_DIRECTORY ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/crypto/SigV4-for-AWS-IoT-embedded-sdk)
include( ${CMAKE_SIGV4_DIRECTORY}/sigv4FilePaths.cmake )

add_library( sigv4
             ${SIGV4_SOURCES} )

target_sources( sigv4
    PRIVATE
        ${SIGV4_SOURCES}
    PUBLIC
        ${prj_root}/src/amazon_kvs/lib_amazon_v2/configs/sigv4
        ${SIGV4_INCLUDE_PUBLIC_DIRS}
)

target_include_directories( sigv4 PUBLIC
                            ${prj_root}/src/amazon_kvs/lib_amazon_v2/configs/sigv4
                            ${SIGV4_INCLUDE_PUBLIC_DIRS} )

### add linked library ###
list(
    APPEND app_example_lib
    sigv4
)
