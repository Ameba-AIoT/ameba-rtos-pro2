# This cmake file is used to include Wslay as static library.
include( ${prj_root}/GCC-RELEASE/includepath.cmake )

file(
  GLOB
  WSLAY_SOURCE_FILES
  "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/wslay/lib/*.c" )

configure_file(${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/wslay/lib/includes/wslay/wslayver.h.in
               ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/wslay/lib/includes/wslay/wslayver.h @ONLY)

set( WSLAY_INCLUDE_DIRS
     "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/wslay/lib/"
     "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/wslay/lib/includes"
     "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/wslay/lib/includes/wslay"
     "${prj_root}/src/amazon_kvs/lib_amazon_v2/configs/wslay" )

add_library( wslay )

target_sources( wslay
    PRIVATE
        ${WSLAY_SOURCE_FILES}
)

target_include_directories( wslay PUBLIC
                            ${WSLAY_INCLUDE_DIRS}
                            ${inc_path}
                            ${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure )

target_compile_definitions( wslay PUBLIC HAVE_ARPA_INET_H )

### add linked library ###
list(
    APPEND app_example_lib
    wslay
)
