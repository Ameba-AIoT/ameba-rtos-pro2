cmake_minimum_required(VERSION 3.6)

project(srtp2)

set(srtp2 srtp2)

list(
    APPEND srtp2_sources

##crypto
#cipher
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/cipher/aes.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/cipher/aes_gcm_mbedtls.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/cipher/aes_icm_mbedtls.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/cipher/cipher.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/cipher/null_cipher.c
#hash
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/hash/auth.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/hash/hmac.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/hash/hmac_mbedtls.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/hash/null_auth.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/hash/sha1.c
#kernal
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/kernel/alloc.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/kernel/crypto_kernel.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/kernel/err.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/kernel/key.c
#math
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/math/datatypes.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/math/stat.c
#reply
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/replay/rdb.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/replay/rdbx.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/replay/ut_sim.c
##srtp
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/srtp/ekt.c
    ${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/srtp/srtp.c
)

add_library(
    ${srtp2} STATIC
    ${srtp2_sources}
)

list(
	APPEND srtp2_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	CONFIG_SYSTEM_TIME64=1
    HAVE_CONFIG_H
    KVS_PLAT_RTK_FREERTOS
)

target_compile_definitions(${srtp2} PRIVATE ${srtp2_flags} )
target_compile_options(${srtp2} PRIVATE ${LIBS_WARN_ERR_FLAGS} )

include(../includepath.cmake)
target_include_directories(
	${srtp2}
	PUBLIC

    ${prj_root}/src/amazon_kvs/lib_amazon/gcc_include
	${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/crypto/include
	${prj_root}/src/amazon_kvs/lib_amazon/libsrtp/include
	${prj_root}/src/amazon_kvs/lib_amazon/libsrtp
    
    ${inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure
)