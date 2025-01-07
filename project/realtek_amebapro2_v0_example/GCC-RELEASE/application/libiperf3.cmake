cmake_minimum_required(VERSION 3.6)

project(iperf3)

include(../includepath.cmake)

set(iperf3 iperf3)

list(
    APPEND iperf3_sources

	${sdk_root}/component/network/iperf3/dscp.c
	${sdk_root}/component/network/iperf3/iperf_api.c
	${sdk_root}/component/network/iperf3/iperf_auth.c
	${sdk_root}/component/network/iperf3/iperf_client_api.c
	${sdk_root}/component/network/iperf3/iperf_error.c
	${sdk_root}/component/network/iperf3/iperf_locale.c
	${sdk_root}/component/network/iperf3/iperf_sctp.c
	${sdk_root}/component/network/iperf3/iperf_server_api.c
	${sdk_root}/component/network/iperf3/iperf_tcp.c
	${sdk_root}/component/network/iperf3/iperf_udp.c
	${sdk_root}/component/network/iperf3/iperf_util.c

	${sdk_root}/component/network/iperf3/net.c
	${sdk_root}/component/network/iperf3/tcp_info.c
	${sdk_root}/component/network/iperf3/tcp_window_size.c
	${sdk_root}/component/network/iperf3/timer.c
	${sdk_root}/component/network/iperf3/units.c

	${sdk_root}/component/network/iperf3/iperf3.c
	${sdk_root}/component/network/iperf3/t_uuid.c
	${sdk_root}/component/network/iperf3/t_timer.c
	${sdk_root}/component/network/iperf3/t_units.c	
)


add_library(
    ${iperf3} STATIC
    ${iperf3_sources}
)

list(
	APPEND iperf3_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
)

target_compile_definitions(${iperf3} PRIVATE ${iperf3_flags} )

include(../includepath.cmake)
target_include_directories(
	${iperf3}
	PUBLIC
	#${inc_path}
	${app_example_inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/non_secure
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/secure
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/voe_bin
	${sdk_root}/component/video/driver/RTL8735B
	
	${prj_root}/src/test_model
	${prj_root}/src	
	
	${sdk_root}/component/soc/8735b/cmsis/cmsis-core/include
	${sdk_root}/component/soc/8735b/cmsis/rtl8735b/lib/include
	${sdk_root}/component/soc/8735b/cmsis/rtl8735b/include
	${sdk_root}/component/network/cJSON
	${sdk_root}/component/os/os_dep/include
	${sdk_root}/component/os/freertos
	${sdk_root}/component/os/freertos/${freertos}/Source/include
	${prj_root}/inc
	${sdk_root}/component/stdlib
	${sdk_root}/component/lwip/api
	${sdk_root}/component/lwip/${lwip}/src/include
	${sdk_root}/component/lwip/${lwip}/src/include/lwip
	#${sdk_root}/component/lwip/${lwip}/src/include/compat/posix
	${sdk_root}/component/lwip/${lwip}/port/realtek
	${sdk_root}/component/lwip/${lwip}/port/realtek/freertos
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure
	${sdk_root}/component/network/iperf3
)