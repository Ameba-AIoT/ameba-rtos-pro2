cmake_minimum_required(VERSION 3.6)

project(wslay)

set(wslay wslay)

list(
	APPEND wslay_sources

###wslay src
	${prj_root}/src/amazon_kvs/lib_amazon/wslay/lib/wslay_event.c
	${prj_root}/src/amazon_kvs/lib_amazon/wslay/lib/wslay_frame.c
	${prj_root}/src/amazon_kvs/lib_amazon/wslay/lib/wslay_net.c
	${prj_root}/src/amazon_kvs/lib_amazon/wslay/lib/wslay_queue.c
)


add_library(
	${wslay} STATIC
	${wslay_sources}
)

list(
	APPEND wslay_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	CONFIG_SYSTEM_TIME64=1

	HAVE_ARPA_INET_H
)

target_compile_definitions(${wslay} PRIVATE ${wslay_flags} )
target_compile_options(${wslay} PRIVATE ${LIBS_WARN_ERR_FLAGS} )

include(../includepath.cmake)
target_include_directories(
	${wslay}
	PUBLIC

	${inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure

	${prj_root}/src/amazon_kvs/lib_amazon/gcc_include
	${prj_root}/src/amazon_kvs/lib_amazon/wslay/lib
	${prj_root}/src/amazon_kvs/lib_amazon/wslay/lib/includes
	${prj_root}/src/amazon_kvs/lib_amazon/wslay/lib/includes/wslay
)