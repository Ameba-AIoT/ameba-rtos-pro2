cmake_minimum_required(VERSION 3.6)

project(app_s)

enable_language(C CXX ASM)

set(app_s application.s)

include(./libsoc_s.cmake OPTIONAL)	

if(BUILD_LIB)
	message(STATUS "build libraries")
else()
	message(STATUS "use released libraries")
	link_directories(${prj_root}/GCC-RELEASE/application/output)
endif()


ADD_LIBRARY (hal_pmc_lib STATIC IMPORTED )
SET_PROPERTY ( TARGET hal_pmc_lib PROPERTY IMPORTED_LOCATION ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/lib/hal_pmc.a )

#HAL
list(
    APPEND out_s_sources
	
	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_hkdf.c
	#${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_otp_nsc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_wdt.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_rtc.c

	
	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_flash_sec.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_pinmux_nsc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_rtc_nsc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_trng_sec.c


	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_adc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_comp.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_crypto.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_dram_init.c	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_dram_scan.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_eddsa.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_eth.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_flash.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_gdma.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_gpio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_i2c.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_i2s.c
	#${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_otp.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_pwm.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_rsa.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_sdhost.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_ssi.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_snand.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_spic.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_timer.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_trng.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_uart.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_i2s.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_sgpio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_sport.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_ssi.c
	
)

#MBED
list(
	APPEND app_s_sources
	${sdk_root}/component/mbed/targets/hal/rtl8735b/crypto_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/flash_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/efuse_api.c
)

#ssl
if(${mbedtls} STREQUAL "mbedtls-3.4.0" OR ${mbedtls} STREQUAL "mbedtls-3.6.0")
file(GLOB MBEDTLS_SRC CONFIGURE_DEPENDS ${sdk_root}/component/ssl/${mbedtls}/library/*.c)
list(
	APPEND out_s_sources
	${MBEDTLS_SRC}
	#nsc
	${sdk_root}/component/ssl/${mbedtls}/library_s/mbedtls_nsc.c
	${sdk_root}/component/ssl/${mbedtls}/library_s/mbedtls_ext_nsc.c
	#ssl_ram_map
	${sdk_root}/component/ssl/ssl_ram_map/rom/rom_ssl_ram_map.c
	${sdk_root}/component/ssl/ssl_func_stubs/ssl_func_stubs.c
)
list(
	REMOVE_ITEM out_s_sources
	${sdk_root}/component/ssl/${mbedtls}/library/net_sockets.c
)
endif()

#RTOS
list(
    APPEND out_s_sources

	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/secure/secure_context.c
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/secure/secure_context_port.c
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/secure/secure_heap.c
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/secure/secure_init.c
	
)

#CMSIS
list(
    APPEND out_s_sources
	${sdk_root}/component/soc/8735b/cmsis/rtl8735b/source/ram/mpu_config.c
	${sdk_root}/component/soc/8735b/cmsis/rtl8735b/source/ram_s/app_start.c
)

#USER
list(
	APPEND app_s_sources
	${prj_root}/src/main_s.c
	${sdk_root}/component/example/secure_storage/example_secure_storage_s.c
)

#MISC
list(
	APPEND app_s_sources

	#${sdk_root}/component/soc/8735b/misc/driver/low_level_io.c
	${sdk_root}/component/soc/8735b/misc/utilities/source/ram/libc_wrap.c
	${sdk_root}/component/soc/8735b/app/shell/ram_s/consol_cmds.c
)

#BLUETOOTH
list(
	APPEND out_s_sources

	${sdk_root}/component/bluetooth/driver/platform/amebapro2/hci/hci_platform_s.c
)

if(PICOLIBC)
list(
	APPEND app_s_sources

	${sdk_root}/component/soc/8735b/misc/driver/picolibc/getentropy.c	
	${sdk_root}/component/soc/8735b/misc/driver/picolibc/gettimeofday.c	
)
endif()


#SW ROM WRAP
#faac_func_stubs.c
#ssl_func_stubs.c
#crypto_internal-modexp.c

#LIB
list(
	APPEND app_s_sources
	

)

add_library(outsrc_s ${out_s_sources})

if(SIMULATION)
	if(BUILD_PXP)
		set(sim_rom_path ${prj_root}/GCC-RELEASE/ROM/GCC/PXP)
	endif()
	if(BUILD_FPGA)
		set(sim_rom_path ${prj_root}/GCC-RELEASE/ROM/GCC/FPGA)
	endif()

	#add command for build PXP or FPGA
	add_custom_command(
		OUTPUT rom.o
		COMMAND ${CMAKE_AS} -mthumb -march=armv8-m.main -mfpu=fpv5-sp-d16 -mfloat-abi=softfp ${sim_rom_path}/rom.s -I${sim_rom_path} -o rom.o
	)

	add_custom_command(
		OUTPUT dtcm_rom.o
		COMMAND ${CMAKE_AS} -mthumb -march=armv8-m.main -mfpu=fpv5-sp-d16 -mfloat-abi=softfp ${sim_rom_path}/dtcm_rom.s -I${sim_rom_path} -o dtcm_rom.o
	)

	add_custom_command(
		OUTPUT itcm_rom.o
		COMMAND ${CMAKE_AS} -mthumb -march=armv8-m.main -mfpu=fpv5-sp-d16 -mfloat-abi=softfp ${sim_rom_path}/itcm_rom.s -I${sim_rom_path} -o itcm_rom.o
	)
	
	add_executable(
		${app_s}
		${app_s_sources}
		#$<TARGET_OBJECTS:rom> 
		#$<TARGET_OBJECTS:soc> 
		$<TARGET_OBJECTS:outsrc_s>
		rom.o
		dtcm_rom.o
		itcm_rom.o
	)
	
	set( ld_script ${CMAKE_CURRENT_SOURCE_DIR}/rtl8735b_ram_s_sim.ld )
else()

	add_executable(
		${app_s}
		${app_s_sources}
		$<TARGET_OBJECTS:outsrc_s>
		#$<TARGET_OBJECTS:rom> 
		#$<TARGET_OBJECTS:soc> 
	)
	
	set( ld_script ${CMAKE_CURRENT_SOURCE_DIR}/rtl8735b_ram_s.ld )
endif()




list(
	APPEND app_s_flags
	CONFIG_BUILD_SECURE=1
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
)

target_compile_definitions(${app_s} PRIVATE ${app_s_flags} )
target_compile_definitions(outsrc_s PRIVATE ${app_s_flags} )

target_compile_options(${app_s} PRIVATE $<$<COMPILE_LANGUAGE:C>:${WARN_ERR_FLAGS}>)
target_compile_options(outsrc_s PRIVATE $<$<COMPILE_LANGUAGE:C>:${OUTSRC_WARN_ERR_FLAGS}>)

include(../includepath.cmake)
list(

	APPEND app_s_inc_path
	${inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/non_secure
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/secure
)

target_include_directories(${app_s} PUBLIC ${app_s_inc_path})
target_include_directories(outsrc_s PUBLIC ${app_s_inc_path})

target_link_libraries(
	${app_s} 
	
	-Wl,--whole-archive
	soc_s
	hal_pmc_lib
	-Wl,--no-whole-archive
	
	c
	gcc
	nosys
)

target_link_options(
	${app_s} 
	PUBLIC
	"LINKER:SHELL:--out-implib=${CMAKE_CURRENT_BINARY_DIR}/import_lib.o"
	"LINKER:SHELL:--cmse-implib"
	"LINKER:SHELL:-L ${sdk_root}/component/soc/8735b/cmsis/rtl8735b/source/GCC"
	"LINKER:SHELL:-L ${CMAKE_CURRENT_BINARY_DIR}"
	"LINKER:SHELL:-T ${ld_script}"
	"LINKER:SHELL:-Map=${CMAKE_CURRENT_BINARY_DIR}/${app_s}.map"
)

set_target_properties(${app_s} PROPERTIES LINK_DEPENDS ${ld_script})

add_custom_target(
	import_lib.o
	DEPENDS ${app_s}
)

add_custom_command(TARGET ${app_s} POST_BUILD 
	COMMAND ${CMAKE_NM} $<TARGET_FILE:${app_s}> | sort > ${app_s}.nm.map
	COMMAND ${CMAKE_OBJEDUMP} -d $<TARGET_FILE:${app_s}> > ${app_s}.asm
	COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${app_s}> ${app_s}.axf

	COMMAND ${CMAKE_COMMAND} -E make_directory  output
	COMMAND ${CMAKE_COMMAND} -E copy ${app_s}.nm.map output 
	COMMAND ${CMAKE_COMMAND} -E copy ${app_s}.map output 
	COMMAND ${CMAKE_COMMAND} -E copy ${app_s}.asm output 
	COMMAND ${CMAKE_COMMAND} -E copy ${app_s}.axf output
	
	COMMAND ${PLAT_COPY} *.a output 
)
