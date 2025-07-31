cmake_minimum_required(VERSION 3.6)

project(llhttp)

set(llhttp llhttp)

list(
	APPEND llhttp_sources

###llhttp src
	${prj_root}/src/amazon_kvs/lib_amazon/llhttp/src/api.c
	${prj_root}/src/amazon_kvs/lib_amazon/llhttp/src/http.c
	${prj_root}/src/amazon_kvs/lib_amazon/llhttp/src/llhttp.c
)


add_library(
	${llhttp} STATIC
	${llhttp_sources}
)

list(
	APPEND llhttp_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	CONFIG_SYSTEM_TIME64=1
)

target_compile_definitions(${llhttp} PRIVATE ${llhttp_flags} )
target_compile_options(${llhttp} PRIVATE ${LIBS_WARN_ERR_FLAGS} )

include(../includepath.cmake)
target_include_directories(
	${llhttp}
	PUBLIC

	${inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure

	${prj_root}/src/amazon_kvs/lib_amazon/gcc_include
	${prj_root}/src/amazon_kvs/lib_amazon/llhttp/include
)