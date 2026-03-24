cmake_minimum_required(VERSION 3.6)
include( ${prj_root}/GCC-RELEASE/includepath.cmake )

project(libsrtp)
set(libsrtp libsrtp)

# libsrtp library source files.
file(GLOB LIBSRTP_GLOB_SOURCES
          ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/kernel/*.c
          ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/math/*.c
          ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/replay/*.c
          ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/srtp/*.c )
set(LIBSRTP_SOURCES
          ${LIBSRTP_GLOB_SOURCES}
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/cipher/aes.c"
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/cipher/aes_gcm_mbedtls.c"
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/cipher/aes_icm_mbedtls.c"
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/cipher/cipher.c"
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/cipher/cipher_test_cases.c"
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/cipher/null_cipher.c"
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/hash/auth_test_cases.c"
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/hash/auth.c"
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/hash/hmac.c"
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/hash/hmac_mbedtls.c"
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/hash/null_auth.c"
          "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/hash/sha1.c"
          )

# libsrtp library Public Include directories.
set( LIBSRTP_INCLUDE_PUBLIC_DIRS
     "${sdk_root}/component/example/kvs_webrtc_v2_mmf/webrtc_v2_app_src/libsrtp"
     "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/include"
     "${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/libsrtp/crypto/include" )

add_library( ${libsrtp} )

target_sources( ${libsrtp}
    PRIVATE
        ${LIBSRTP_SOURCES}
)

list(
    APPEND libsrtp_flags
    HAVE_CONFIG_H
)

target_compile_definitions(${libsrtp} PRIVATE ${libsrtp_flags})

target_include_directories(
    ${libsrtp}
    PUBLIC
    ${LIBSRTP_INCLUDE_PUBLIC_DIRS}
    ${inc_path}
    ${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure
)

### add linked library ###
list(
    APPEND app_example_lib
    ${libsrtp}
)
