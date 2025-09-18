cmake_minimum_required(VERSION 3.6)

project(app)

enable_language(C CXX ASM)

if(BUILD_TZ)
set(app application.ns)
else()
set(app application.ntz)
endif()

include(../includepath.cmake)

if(BUILD_TZ)
	include(./libsoc_ns.cmake OPTIONAL)
else()
	include(./libsoc_ntz.cmake OPTIONAL)
endif()
include(./libwlan.cmake OPTIONAL)
include(./libwps.cmake OPTIONAL)
if(BUILD_TZ)
	include(./libvideo_ns.cmake OPTIONAL)
else()
	include(./libvideo_ntz.cmake OPTIONAL)
endif()
include(./libmmf.cmake OPTIONAL)
include(./libg711.cmake OPTIONAL)
if(BUILD_NEWAEC)
#AEC FILE
include(./libctaec.cmake OPTIONAL)
else()
include(./libaec.cmake OPTIONAL)
endif()
include(./libhttp.cmake OPTIONAL)
include(./libsdcard.cmake OPTIONAL)
include(./libfaac.cmake OPTIONAL)
include(./libhaac.cmake OPTIONAL)
include(./libfdkaac.cmake OPTIONAL)
include(./libmuxer.cmake OPTIONAL)
include(./libusbd.cmake OPTIONAL)
include(./libopus.cmake OPTIONAL)
include(./libopusenc.cmake OPTIONAL)
include(./libopusfile.cmake OPTIONAL)
include(./libhmp3.cmake OPTIONAL)
include(./libnn.cmake OPTIONAL)
include(./libqrcode.cmake OPTIONAL)
include(./libfmp4.cmake OPTIONAL)
include(./libispfeature.cmake OPTIONAL)
include(./libmd.cmake OPTIONAL)
include(./libfaultlog.cmake OPTIONAL)
include(./libeap.cmake OPTIONAL)
include(./libiperf3.cmake OPTIONAL)

if(BUILD_LIB)
	message(STATUS "build libraries")
else()
	message(STATUS "use released libraries")
	link_directories(${prj_root}/GCC-RELEASE/application/output)
endif()

ADD_LIBRARY (bt_upperstack_lib STATIC IMPORTED )
SET_PROPERTY ( TARGET bt_upperstack_lib PROPERTY IMPORTED_LOCATION ${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/lib/btgap.a )

# FACEALIGNMENT NK
ADD_LIBRARY (libface STATIC IMPORTED )
SET_PROPERTY ( TARGET libface PROPERTY IMPORTED_LOCATION ${prj_root}/src/test_model/face_alignment/libface.a )

if(NOT BUILD_TZ)
ADD_LIBRARY (hal_pmc_lib STATIC IMPORTED )
SET_PROPERTY ( TARGET hal_pmc_lib PROPERTY IMPORTED_LOCATION ${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/lib/hal_pmc.a )
endif()

#HAL
if(NOT BUILD_TZ)
#build TZ, move to secure project
list(
    APPEND out_sources
	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_flash_sec.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_hkdf.c
	#${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_otp_nsc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_pinmux_nsc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_wdt.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_rtc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_s/hal_trng_sec.c
)
endif()

list(
    APPEND out_sources
	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_audio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_adc.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_comp.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_crypto.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_dram_init.c	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_dram_scan.c	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_eddsa.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_ecdsa.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_flash.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_gdma.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_gpio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_i2c.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_i2s.c
	#${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_otp.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_pwm.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_rsa.c
	#${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_sdhost.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_ssi.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_spic.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_snand.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_timer.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_trng.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_uart.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_sgpio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_sport.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_i2s.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_sgpio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_sport.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_ssi.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_audio.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/hal_eth.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram/rtl8735b_eth.c
)

list(
    APPEND out_sources
	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_ns/hal_flash_ns.c
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/source/ram_ns/hal_spic_ns.c
)

#MBED
list(
    APPEND out_sources
	${sdk_root}/component/mbed/targets/hal/rtl8735b/audio_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/crypto_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/dma_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/ecdsa_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/flash_api.c
	${sdk_root}/component/soc/8735b/misc/driver/flash_api_ext.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/i2c_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/i2s_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/pwmout_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/sgpio_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/spi_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/timer_api.c
	${sdk_root}/component/soc/8735b/mbed-drivers/source/wait_api.c
	${sdk_root}/component/soc/8735b/mbed-drivers/source/us_ticker_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/us_ticker.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/gpio_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/gpio_irq_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/serial_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/wdt_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/rtc_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/analogin_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/pinmap_common.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/pinmap.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/ethernet_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/trng_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/power_mode_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/snand_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/sys_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/ethernet_api.c
	${sdk_root}/component/mbed/targets/hal/rtl8735b/efuse_api.c
)

#RTOS
list(
    APPEND out_sources
	${sdk_root}/component/os/freertos/${freertos}/Source/croutine.c
	${sdk_root}/component/os/freertos/${freertos}/Source/event_groups.c
	${sdk_root}/component/os/freertos/${freertos}/Source/list.c
	${sdk_root}/component/os/freertos/${freertos}/Source/queue.c
	${sdk_root}/component/os/freertos/${freertos}/Source/stream_buffer.c
	${sdk_root}/component/os/freertos/${freertos}/Source/tasks.c
	${sdk_root}/component/os/freertos/${freertos}/Source/timers.c
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/MemMang/heap_4_2.c
	
	${sdk_root}/component/os/freertos/freertos_cb.c
	${sdk_root}/component/os/freertos/freertos_service.c
	${sdk_root}/component/os/freertos/cmsis_os.c
	
	${sdk_root}/component/os/os_dep/osdep_service.c
	${sdk_root}/component/os/os_dep/device_lock.c
	${sdk_root}/component/os/os_dep/timer_service.c
	#posix
	${sdk_root}/component/os/freertos/freertos_posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_clock.c
	${sdk_root}/component/os/freertos/freertos_posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_mqueue.c
	${sdk_root}/component/os/freertos/freertos_posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread.c
	${sdk_root}/component/os/freertos/freertos_posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread_barrier.c
	${sdk_root}/component/os/freertos/freertos_posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread_cond.c
	${sdk_root}/component/os/freertos/freertos_posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_pthread_mutex.c
	${sdk_root}/component/os/freertos/freertos_posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_sched.c
	${sdk_root}/component/os/freertos/freertos_posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_semaphore.c
	${sdk_root}/component/os/freertos/freertos_posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_timer.c
	${sdk_root}/component/os/freertos/freertos_posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_unistd.c
	${sdk_root}/component/os/freertos/freertos_posix/lib/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_utils.c
)

if(BUILD_TZ)
list(
    APPEND out_sources
	#FREERTOS
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/non_secure/port.c
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/non_secure/portasm.c
	#CMSIS
	${sdk_root}/component/soc/8735b/cmsis/rtl8735b/source/ram_ns/app_start.c
	${sdk_root}/component/soc/8735b/cmsis/rtl8735b/source/ram_ns/system_ns.c
)
else()
list(
    APPEND out_sources
	#FREERTOS
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure/port.c
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c
	#CMSIS
	${sdk_root}/component/soc/8735b/cmsis/rtl8735b/source/ram_s/app_start.c
)
endif()

#CMSIS
list(
    APPEND out_sources
	${sdk_root}/component/soc/8735b/cmsis/rtl8735b/source/ram/mpu_config.c

	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/BasicMathFunctions/arm_add_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/TransformFunctions/arm_bitreversal2.S
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/TransformFunctions/arm_cfft_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/TransformFunctions/arm_cfft_radix8_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/ComplexMathFunctions/arm_cmplx_mag_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/CommonTables/arm_common_tables.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/CommonTables/arm_const_structs.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/StatisticsFunctions/arm_max_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/BasicMathFunctions/arm_mult_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/TransformFunctions/arm_rfft_fast_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/TransformFunctions/arm_rfft_fast_init_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/BasicMathFunctions/arm_scale_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/BasicMathFunctions/arm_dot_prod_f32.c
	${sdk_root}/component/soc/8735b/cmsis/cmsis-dsp/source/BasicMathFunctions/arm_offset_f32.c
)

#at_cmd
list(
	APPEND app_sources
	${sdk_root}/component/at_cmd/atcmd_sys.c
	${sdk_root}/component/at_cmd/atcmd_wifi.c
	${sdk_root}/component/at_cmd/atcmd_bt.c
	${sdk_root}/component/at_cmd/atcmd_mp.c
	${sdk_root}/component/at_cmd/atcmd_mp_ext2.c
	${sdk_root}/component/at_cmd/atcmd_isp.c
	${sdk_root}/component/at_cmd/atcmd_ftl.c
	${sdk_root}/component/at_cmd/atcmd_ethernet.c
	${sdk_root}/component/at_cmd/log_service.c
	${sdk_root}/component/soc/8735b/misc/driver/rtl_console.c
	${sdk_root}/component/soc/8735b/misc/driver/console_auth.c
	${sdk_root}/component/soc/8735b/misc/driver/low_level_io.c
)

#wifi
list(
	APPEND app_sources
	#api
	${sdk_root}/component/wifi/api/wifi_conf.c
	${sdk_root}/component/wifi/api/wifi_conf_wowlan.c
	${sdk_root}/component/wifi/api/wifi_conf_inter.c
	${sdk_root}/component/wifi/api/wifi_conf_ext.c
	${sdk_root}/component/wifi/api/wifi_ind.c
	${sdk_root}/component/wifi/api/wlan_network.c
	#promisc
	${sdk_root}/component/wifi/promisc/wifi_conf_promisc.c
	${sdk_root}/component/wifi/promisc/wifi_promisc.c
	#fast_connect
	${sdk_root}/component/wifi/wifi_fast_connect/wifi_fast_connect.c
	#wpa_supplicant
	${sdk_root}/component/wifi/wpa_supplicant/wpa_supplicant/wifi_wps_config.c
	#wpa_supplicant
	${sdk_root}/component/wifi/wpa_supplicant/src/crypto/tls_polarssl.c		
	#wpa_supplicant
	${sdk_root}/component/wifi/wpa_supplicant/wpa_supplicant/wifi_eap_config.c	
	#option
	${sdk_root}/component/wifi/driver/src/core/option/rtw_opt_crypto_ssl.c
	${sdk_root}/component/wifi/driver/src/core/option/rtw_opt_skbuf_rtl8735b.c
)

#ethernet
list(
	APPEND app_sources
	${sdk_root}/component/ethernet_mii/ethernet_mii.c
	${sdk_root}/component/ethernet_mii/ethernet_usb.c
)

#network
list(
	APPEND app_sources
	#dhcp
	${sdk_root}/component/network/dhcp/dhcps.c
	#ping
	${sdk_root}/component/network/ping/ping_test.c
	#iperf
	${sdk_root}/component/network/iperf/iperf.c
	#sntp
	${sdk_root}/component/network/sntp/sntp.c
	#http
	${sdk_root}/component/network/httpc/httpc_tls.c
	${sdk_root}/component/network/httpd/httpd_tls.c
	#cJSON
	${sdk_root}/component/network/cJSON/cJSON.c
	#ssl_client
	${sdk_root}/component/example/ssl_client/ssl_client.c
	${sdk_root}/component/example/ssl_client/ssl_client_ext.c
	#mqtt
	${sdk_root}/component/network/mqtt/MQTTClient/MQTTClient.c
	${sdk_root}/component/network/mqtt/MQTTClient/MQTTFreertos.c
	${sdk_root}/component/network/mqtt/MQTTPacket/MQTTConnectClient.c
	${sdk_root}/component/network/mqtt/MQTTPacket/MQTTConnectServer.c
	${sdk_root}/component/network/mqtt/MQTTPacket/MQTTDeserializePublish.c
	${sdk_root}/component/network/mqtt/MQTTPacket/MQTTFormat.c
	${sdk_root}/component/network/mqtt/MQTTPacket/MQTTPacket.c
	${sdk_root}/component/network/mqtt/MQTTPacket/MQTTSerializePublish.c
	${sdk_root}/component/network/mqtt/MQTTPacket/MQTTSubscribeClient.c
	${sdk_root}/component/network/mqtt/MQTTPacket/MQTTSubscribeServer.c
	${sdk_root}/component/network/mqtt/MQTTPacket/MQTTUnsubscribeClient.c
	${sdk_root}/component/network/mqtt/MQTTPacket/MQTTUnsubscribeServer.c
	#websocket
	${sdk_root}/component/network/websocket/libwsclient.c
	#${sdk_root}/component/network/websocket/ws_server_msg.c
	${sdk_root}/component/network/websocket/wsclient_api.c
	${sdk_root}/component/network/websocket/wsclient_tls.c
	#${sdk_root}/component/network/websocket/wsserver_api.c
	#${sdk_root}/component/network/websocket/wsserver_tls.c
	#ota
	${sdk_root}/component/soc/8735b/misc/platform/ota_8735b.c
	${sdk_root}/component/soc/8735b/misc/platform/ota_8735b_fwfs.c
	${sdk_root}/component/soc/8735b/misc/platform/dfu_8735b.c
	#httplite
	${sdk_root}/component/network/httplite/http_client.c
	#tftp
	${sdk_root}/component/network/tftp/tftp_client.c
	${sdk_root}/component/network/tftp/tftp_server.c
	#coap
	${sdk_root}/component/network/coap/sn_coap_ameba_port.c
	${sdk_root}/component/network/coap/sn_coap_builder.c
	${sdk_root}/component/network/coap/sn_coap_header_check.c
	${sdk_root}/component/network/coap/sn_coap_parser.c
	${sdk_root}/component/network/coap/sn_coap_protocol.c
)

#lwip
list(
	APPEND out_sources
	#api
	${sdk_root}/component/lwip/api/lwip_netconf.c
	#lwip - api
	${sdk_root}/component/lwip/${lwip}/src/api/api_lib.c
	${sdk_root}/component/lwip/${lwip}/src/api/api_msg.c
	${sdk_root}/component/lwip/${lwip}/src/api/err.c
	${sdk_root}/component/lwip/${lwip}/src/api/netbuf.c
	${sdk_root}/component/lwip/${lwip}/src/api/netdb.c
	${sdk_root}/component/lwip/${lwip}/src/api/netifapi.c
	${sdk_root}/component/lwip/${lwip}/src/api/sockets.c
	${sdk_root}/component/lwip/${lwip}/src/api/tcpip.c
	#lwip - core
	${sdk_root}/component/lwip/${lwip}/src/core/def.c
	${sdk_root}/component/lwip/${lwip}/src/core/dns.c
	${sdk_root}/component/lwip/${lwip}/src/core/inet_chksum.c
	${sdk_root}/component/lwip/${lwip}/src/core/init.c
	${sdk_root}/component/lwip/${lwip}/src/core/ip.c
	${sdk_root}/component/lwip/${lwip}/src/core/mem.c
	${sdk_root}/component/lwip/${lwip}/src/core/memp.c
	${sdk_root}/component/lwip/${lwip}/src/core/netif.c
	${sdk_root}/component/lwip/${lwip}/src/core/pbuf.c
	${sdk_root}/component/lwip/${lwip}/src/core/raw.c
	${sdk_root}/component/lwip/${lwip}/src/core/stats.c
	${sdk_root}/component/lwip/${lwip}/src/core/sys.c
	${sdk_root}/component/lwip/${lwip}/src/core/tcp.c
	${sdk_root}/component/lwip/${lwip}/src/core/tcp_in.c
	${sdk_root}/component/lwip/${lwip}/src/core/tcp_out.c
	${sdk_root}/component/lwip/${lwip}/src/core/timeouts.c
	${sdk_root}/component/lwip/${lwip}/src/core/udp.c
	#lwip - core - ipv4
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/autoip.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/dhcp.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/etharp.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/icmp.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/igmp.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/ip4.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/ip4_addr.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/ip4_frag.c
	#lwip - core - ipv6
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/dhcp6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/ethip6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/icmp6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/inet6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/ip6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/ip6_addr.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/ip6_frag.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/mld6.c
	${sdk_root}/component/lwip/${lwip}/src/core/ipv6/nd6.c
	#lwip - netif
	${sdk_root}/component/lwip/${lwip}/src/netif/ethernet.c
	#lwip - port
	${sdk_root}/component/lwip/${lwip}/port/realtek/freertos/ethernetif.c
	${sdk_root}/component/wifi/driver/src/osdep/lwip_intf.c
	${sdk_root}/component/lwip/${lwip}/port/realtek/freertos/sys_arch.c
)

if(${lwip} STREQUAL "lwip_v2.2.0")
list(
	APPEND out_sources
	${sdk_root}/component/lwip/${lwip}/src/core/ipv4/acd.c
)
endif()

#ssl
if(${mbedtls} STREQUAL "mbedtls-2.4.0")
list(
	APPEND out_sources
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/aesni.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/blowfish.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/camellia.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ccm.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/certs.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/cipher.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/cipher_wrap.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/cmac.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/debug.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/gcm.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/havege.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/md2.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/md4.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/memory_buffer_alloc.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/net_sockets.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/padlock.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/pkcs11.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/pkcs12.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/pkcs5.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/pkparse.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/platform.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ripemd160.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_cache.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_ciphersuites.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_cli.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_cookie.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_srv.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_ticket.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/ssl_tls.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/threading.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/timing.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/version.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/version_features.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509_create.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509_crl.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509_crt.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509_csr.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509write_crt.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/x509write_csr.c
	${sdk_root}/component/ssl/mbedtls-2.4.0/library/xtea.c
	#ssl_ram_map
	${sdk_root}/component/ssl/ssl_ram_map/rom/rom_ssl_ram_map.c
	${sdk_root}/component/ssl/ssl_func_stubs/ssl_func_stubs.c
)
else()
file(GLOB MBEDTLS_SRC CONFIGURE_DEPENDS ${sdk_root}/component/ssl/${mbedtls}/library/*.c)
list(
	APPEND out_sources
	${MBEDTLS_SRC}
	#ssl_ram_map
	${sdk_root}/component/ssl/ssl_ram_map/rom/rom_ssl_ram_map.c
	${sdk_root}/component/ssl/ssl_func_stubs/ssl_func_stubs.c
)
endif()

#FATFS
list(
	APPEND out_sources
	${sdk_root}/component/file_system/fatfs/disk_if/src/sdcard.c
	${sdk_root}/component/file_system/fatfs/disk_if/src/flash_fatfs.c
	${sdk_root}/component/file_system/fatfs/fatfs_ext/src/ff_driver.c
	
	${sdk_root}/component/file_system/fatfs/r0.14/diskio.c
	${sdk_root}/component/file_system/fatfs/r0.14/ff.c
	${sdk_root}/component/file_system/fatfs/r0.14/ffsystem.c
	${sdk_root}/component/file_system/fatfs/r0.14/ffunicode.c
	${sdk_root}/component/file_system/fatfs/fatfs_flash_api.c
	${sdk_root}/component/file_system/fatfs/fatfs_reent.c
	${sdk_root}/component/file_system/fatfs/fatfs_sdcard_api.c
	${sdk_root}/component/file_system/fatfs/fatfs_ramdisk_api.c
)

#Littlefs
list(
	APPEND out_sources
	${sdk_root}/component/file_system/littlefs/r2.41/lfs.c
	${sdk_root}/component/file_system/littlefs/r2.41/lfs_util.c
	${sdk_root}/component/file_system/littlefs/lfs_reent.c
)

#vfs
list(
	APPEND app_sources
	${sdk_root}/component/file_system/vfs/vfs.c
	${sdk_root}/component/file_system/vfs/vfs_fatfs.c
	${sdk_root}/component/file_system/vfs/vfs_littlefs.c
	${sdk_root}/component/file_system/vfs/vfs_wrap.c
)

#FTL_COMMON
list(
	APPEND app_sources
	${sdk_root}/component/file_system/ftl_common/ftl_common_api.c
	${sdk_root}/component/file_system/ftl_common/ftl_nand_api.c
	${sdk_root}/component/file_system/ftl_common/ftl_nor_api.c
	#${sdk_root}/component/file_system/ftl_common/nand_task.c
)

#system_data
list(
	APPEND app_sources
	${sdk_root}/component/file_system/system_data/system_data_api.c
)

#USER
list(
    APPEND app_sources
	${sdk_root}/component/soc/8735b/misc/driver/mpu_protect.c	
)

#RTSP
list(
	APPEND app_sources	
	${sdk_root}/component/network/rtsp/rtp_api.c
	${sdk_root}/component/network/rtsp/rtsp_api.c
	${sdk_root}/component/network/rtsp/sdp.c
)

#VIDEO
list(
	APPEND app_sources		
	${sdk_root}/component/video/driver/RTL8735B/video_api.c
	${sdk_root}/component/video/driver/RTL8735B/video_snapshot.c
	${sdk_root}/component/video/osd2/isp_osd_lite.c
	${sdk_root}/component/video/eip/eip_auto_wdr.c
	${sdk_root}/component/media/framework/AL3042.c
	${sdk_root}/component/media/framework/ambient_light_sensor.c
	${sdk_root}/component/media/framework/ir_ctrl.c
	${sdk_root}/component/media/framework/ir_cut.c
	${sdk_root}/component/media/framework/sensor_service.c
)

#MISC
list(
	APPEND app_sources
	
	${sdk_root}/component/soc/8735b/misc/utilities/source/ram/libc_wrap.c
	${sdk_root}/component/soc/8735b/app/shell/cmd_shell.c	
)

#LIB
list(
	APPEND app_sources
	
)

#FTL
list(
	APPEND app_sources
	
	${sdk_root}/component/file_system/ftl/ftl.c
)

#FWFS
list(
	APPEND app_sources
	
	${sdk_root}/component/file_system/fwfs/fwfs.c
)

#NN FILE OP
list(
	APPEND app_sources
	
	${sdk_root}/component/file_system/nn/nn_file_op.c
)

#BLUETOOTH
list(
	APPEND out_sources

	${sdk_root}/component/bluetooth/driver/hci/hci_process/hci_process.c
	${sdk_root}/component/bluetooth/driver/hci/hci_process/hci_standalone.c
	${sdk_root}/component/bluetooth/driver/hci/hci_transport/hci_h4.c
	${sdk_root}/component/bluetooth/driver/hci/hci_if_rtk.c
	${sdk_root}/component/bluetooth/driver/platform/amebapro2/hci/bt_mp_patch.c
	${sdk_root}/component/bluetooth/driver/platform/amebapro2/hci/bt_normal_patch.c
	${sdk_root}/component/bluetooth/driver/platform/amebapro2/hci/hci_dbg.c
	${sdk_root}/component/bluetooth/driver/platform/amebapro2/hci/hci_platform.c
	${sdk_root}/component/bluetooth/driver/platform/amebapro2/hci/hci_uart.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/src/vendor_cmd/vendor_cmd.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/src/platform_utils.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/src/rtk_coex.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/src/trace_uart.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/src/uart_bridge.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/common/src/cycle_queue.c
	${sdk_root}/component/bluetooth/rtk_stack/platform/common/src/trace_task.c
	${sdk_root}/component/bluetooth/rtk_stack/src/ble/privacy/privacy_mgnt.c
	${sdk_root}/component/bluetooth/os/freertos/osif_freertos.c

	${sdk_root}/component/bluetooth/rtk_stack/src/ble/profile/server/simple_ble_service.c
	${sdk_root}/component/bluetooth/rtk_stack/src/ble/profile/server/bas.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_peripheral/app_task.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_peripheral/ble_app_main.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_peripheral/peripheral_app.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_peripheral/ble_peripheral_at_cmd.c

	${sdk_root}/component/bluetooth/rtk_stack/src/ble/profile/client/gcs_client.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central/ble_central_app_main.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central/ble_central_app_task.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central/ble_central_client_app.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central/ble_central_link_mgr.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central/ble_central_at_cmd.c

	${sdk_root}/component/bluetooth/rtk_stack/example/ble_scatternet/ble_scatternet_app_main.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_scatternet/ble_scatternet_app_task.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_scatternet/ble_scatternet_app.c
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_scatternet/ble_scatternet_link_mgr.c

	${sdk_root}/component/bluetooth/rtk_stack/example/bt_beacon/bt_beacon_app.c
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_beacon/bt_beacon_app_main.c
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_beacon/bt_beacon_app_task.c

	${sdk_root}/component/bluetooth/rtk_stack/example/bt_config/bt_config_app_main.c
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_config/bt_config_app_task.c
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_config/bt_config_peripheral_app.c
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_config/bt_config_service.c
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_config/bt_config_wifi.c
)

if(PICOLIBC)
list(
	APPEND app_sources

	${sdk_root}/component/soc/8735b/misc/driver/picolibc/getentropy.c
)
endif()

if(DEFINED SCENARIO AND SCENARIO AND NOT "${SCENARIO}" STREQUAL "standard")
    message(STATUS "SCENARIO = ${SCENARIO}")
    if(EXISTS ${prj_root}/scenario/${SCENARIO})
        if(EXISTS ${prj_root}/scenario/${SCENARIO}/scenario.cmake)
            message(STATUS "Found SCENARIO ${SCENARIO} and start to build up ${SCENARIO} project")
            include(${prj_root}/scenario/${SCENARIO}/scenario.cmake)
    endif()
    else()
        message(ERROR SCENARIO "${SCENARIO} Not Found")
        endif()
	if(NOT DEBUG)
        set(SCENARIO OFF CACHE STRING INTERNAL FORCE)
	endif()
else()
#If users do not choose the scenario, the SDK will use the standard scenario, which means using sdk original src folder
#Todo: maybe some libraries and source files for applcaition also not need to include in every scenario
    message(STATUS "Set SCENARIO standard and start to build up standard project")
    include(${prj_root}/scenario.cmake)
endif()

if(CONSOLE STREQUAL "RTT")
	include(./console_segger_rtt.cmake OPTIONAL)
endif()

include(./console_xmodem.cmake OPTIONAL)
include(./console_remote_sh.cmake OPTIONAL)

if(UNITEST)
	include(${prj_root}/src/internal/unitest/unitest.cmake OPTIONAL)
endif()

add_library(outsrc ${out_sources})

if(BUILD_TZ)
	add_library(secure_object OBJECT IMPORTED)
	set_target_properties( secure_object PROPERTIES IMPORTED_OBJECTS "${CMAKE_CURRENT_BINARY_DIR}/import_lib.o")

	add_executable(
		${app}
		${app_sources}
		${scn_sources}
		$<TARGET_OBJECTS:outsrc>
		$<TARGET_OBJECTS:secure_object>
	)
	add_dependencies(${app} import_lib.o)
	# add noddr ld??
	set( soclib soc_ns)
	set( videolib video_ns)
	set( ld_script ${CMAKE_CURRENT_SOURCE_DIR}/rtl8735b_ram_ns.ld )
else()
	add_executable(
		${app}
		${app_sources}
		${scn_sources}
		$<TARGET_OBJECTS:outsrc>
	)

	set( soclib soc_ntz)
	set( videolib video_ntz)
	if(NODDR)
		message(STATUS "WITHOUT DDR")
		set( ld_script ${CMAKE_CURRENT_SOURCE_DIR}/rtl8735b_ram_noddr.ld ) 
	else()
		message(STATUS "WITH DDR")
		set( ld_script ${CMAKE_CURRENT_SOURCE_DIR}/rtl8735b_ram.ld )
	endif()	
endif()


list(
	APPEND app_flags
	${scn_flags}
	CONFIG_BUILD_RAM=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	CONFIG_SYSTEM_TIME64=1
)

if (BUILD_NEWAEC)
list(
	APPEND app_flags
    CONFIG_NEWAEC=1
)    
endif()
if(BUILD_WLANMP)	
list(
	APPEND app_flags
	CONFIG_MP_INCLUDED
)
endif()

if(BUILD_TZ)
list(
	APPEND app_flags
	CONFIG_BUILD_NONSECURE=1
	ENABLE_SECCALL_PATCH
)
endif()

target_compile_options(${app} PRIVATE $<$<COMPILE_LANGUAGE:C>:${WARN_ERR_FLAGS}>)
target_compile_options(outsrc PRIVATE $<$<COMPILE_LANGUAGE:C>:${OUTSRC_WARN_ERR_FLAGS}>)

target_compile_definitions(${app} PRIVATE ${app_flags})
target_compile_definitions(outsrc PRIVATE ${app_flags})

# HEADER FILE PATH
list(
	APPEND app_inc_path
	
	${inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/non_secure
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33/secure
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/voe_bin
	${sdk_root}/component/video/driver/RTL8735B
	
    ${scn_inc_path}
	
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/nn
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/nn/model_itp
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/nn/nn_api
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/nn/nn_postprocess
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/nn/nn_preprocess
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/nn/run_facerecog
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/nn/run_itp	
	
	${sdk_root}/component/soc/8735b/misc/platform

	${sdk_root}/component/media/mmfv2
	${sdk_root}/component/media/rtp_codec
	${sdk_root}/component/audio/3rdparty/AEC
	${sdk_root}/component/mbed/hal_ext
	${sdk_root}/component/file_system/ftl
	${sdk_root}/component/file_system/system_data
	${sdk_root}/component/file_system/fwfs
	${sdk_root}/component/file_system/nn

	${sdk_root}/component/bluetooth/driver
	${sdk_root}/component/bluetooth/driver/hci
	${sdk_root}/component/bluetooth/driver/inc
	${sdk_root}/component/bluetooth/driver/inc/hci
	${sdk_root}/component/bluetooth/driver/platform/amebapro2/inc
	${sdk_root}/component/bluetooth/os/osif
	${sdk_root}/component/bluetooth/rtk_stack/example
	${sdk_root}/component/bluetooth/rtk_stack/inc/app
	${sdk_root}/component/bluetooth/rtk_stack/inc/bluetooth/gap
	${sdk_root}/component/bluetooth/rtk_stack/inc/bluetooth/profile
	${sdk_root}/component/bluetooth/rtk_stack/inc/bluetooth/profile/client
	${sdk_root}/component/bluetooth/rtk_stack/inc/bluetooth/profile/server
	${sdk_root}/component/bluetooth/rtk_stack/inc/framework/bt
	${sdk_root}/component/bluetooth/rtk_stack/inc/framework/remote
	${sdk_root}/component/bluetooth/rtk_stack/inc/framework/sys
	${sdk_root}/component/bluetooth/rtk_stack/inc/os
	${sdk_root}/component/bluetooth/rtk_stack/inc/platform
	${sdk_root}/component/bluetooth/rtk_stack/inc/stack
	${sdk_root}/component/bluetooth/rtk_stack/src/ble/privacy
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/inc
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/lib
	${sdk_root}/component/bluetooth/rtk_stack/platform/amebapro2/src/vendor_cmd
	${sdk_root}/component/bluetooth/rtk_stack/platform/common/inc
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_central
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_peripheral
	${sdk_root}/component/bluetooth/rtk_stack/example/ble_scatternet
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_beacon
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_config
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_airsync_config
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_mesh/provisioner
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_mesh/device
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_mesh_multiple_profile/provisioner_multiple_profile
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_mesh_multiple_profile/device_multiple_profile
	${sdk_root}/component/bluetooth/rtk_stack/example/bt_mesh_test

	${sdk_root}/component/wifi/wpa_supplicant/src
	${sdk_root}/component/network/mqtt/MQTTClient
	${sdk_root}/component/network/mqtt/MQTTPacket
	${sdk_root}/component/network/tftp
	${sdk_root}/component/network/ftp

	${sdk_root}/component/example/media_framework/inc
	${sdk_root}/component/wifi/wpa_supplicant/src
	${sdk_root}/component/wifi/driver/src/core/option
	${sdk_root}/component/ssl/ssl_ram_map/rom
	${sdk_root}/component/audio/3rdparty/faac/libfaac
	${prj_root}/component/file_system/fatfs/r0.14
	${sdk_root}/component/soc/8735b/fwlib/rtl8735b/lib/source/ram/video/osd
	${sdk_root}/component/video/osd2
	${sdk_root}/component/video/eip
	${sdk_root}/component/video/md
	${sdk_root}/component/wifi/wifi_config
	
	${sdk_root}/component/media/framework
	${sdk_root}/component/soc/8735b/misc/driver
	${sdk_root}/component/soc/8735b/misc/driver/xmodem
	${sdk_root}/component/soc/8735b/misc/driver/mqtt

	${sdk_root}/component/network/coap/include
	
	${sdk_root}/component/usb/common_new/
	${sdk_root}/component/usb/host_new/
	${sdk_root}/component/usb/host_new/cdc_ecm
	${sdk_root}/component/usb/host_new/core
	${sdk_root}/component/usb/device_new/core
	${sdk_root}/component/usb/

	${sdk_root}/component/wifi/wpa_supplicant/src/
	${sdk_root}/component/wifi/wpa_supplicant/src/crypto
)

target_include_directories( ${app} PUBLIC ${app_inc_path} )
target_include_directories( outsrc PUBLIC ${app_inc_path} )


if(NOT BUILD_WLANMP)	
	set( wlanlib wlan)
else()
	set( wlanlib wlan_mp)
endif()

if(NOT BUILD_TZ)
target_link_libraries(
	${app}
	hal_pmc_lib
)
endif()

if(BUILD_NEWAEC)
	set( aeclib ctaec)
	set( aeclib_ex ${sdk_root}/component/audio/3rdparty/AEC/CTAEC/libVQE.a)

else()
	set( aeclib aec)
	unset( aeclib_ex )
endif()

list(
	APPEND libs
	${wlanlib}
	wps
	opusenc
	opusfile
	opus
	hmp3
	g711
	http
 	${aeclib}
	${videolib}
	mmf
	sdcard
	faac
	haac
	fdkaac
	muxer
	fmp4
	usbd
	qrcode
	iperf3
	nn
	libface
	ispfeature
	md
	eap	
	faultlog
	${soclib}
)

list(
	APPEND out_libs 
	${aeclib_ex}
	bt_upperstack_lib
)


target_link_libraries(
	${app}
	#${wlanlib}
	${scn_libs}
	
	${libs}
	${out_libs}
	
    stdc++
	m
	c
	gcc
)



if(NOT PICOLIBC)
target_link_libraries(
	${app} 
	nosys
)
endif()

target_link_options(
	${app} 
	PUBLIC
	"LINKER:SHELL:-L ${sdk_root}/component/soc/8735b/cmsis/rtl8735b/source/GCC"
	"LINKER:SHELL:-L ${CMAKE_CURRENT_BINARY_DIR}"
	"LINKER:SHELL:-T ${ld_script}"
	"LINKER:SHELL:-Map=${CMAKE_CURRENT_BINARY_DIR}/${app}.map"
	"LINKER:-wrap,realloc"
	"LINKER:-wrap,_realloc_r"
	#"SHELL:${CMAKE_CURRENT_SOURCE_DIR}/build/import.lib"
)

if(BUILD_TZ)
target_link_options(
	${app} 
	PUBLIC
	"LINKER:SHELL:-wrap,hal_crypto_engine_init_platform"
	"LINKER:SHELL:-wrap,hal_pinmux_register"
	"LINKER:SHELL:-wrap,hal_pinmux_unregister"
	"LINKER:SHELL:-wrap,hal_otp_byte_rd_syss"
	"LINKER:SHELL:-wrap,hal_otp_byte_wr_syss"
	"LINKER:SHELL:-wrap,hal_sys_get_video_info"
	"LINKER:SHELL:-wrap,hal_sys_peripheral_en"
	"LINKER:SHELL:-wrap,hal_sys_set_clk"
	"LINKER:SHELL:-wrap,hal_sys_get_clk"
	"LINKER:SHELL:-wrap,hal_sys_lxbus_shared_en"
	"LINKER:SHELL:-wrap,bt_power_on"
	"LINKER:SHELL:-wrap,hal_pll_98p304_ctrl"
	"LINKER:SHELL:-wrap,hal_pll_45p158_ctrl"
	"LINKER:SHELL:-wrap,hal_osc4m_cal"
	"LINKER:SHELL:-wrap,hal_sdm_32k_enable"
	"LINKER:SHELL:-wrap,hal_sys_get_rom_ver"
	"LINKER:SHELL:-wrap,hal_otp_init"
	"LINKER:SHELL:-wrap,hal_otp_sb_key_get"
	"LINKER:SHELL:-wrap,hal_otp_sb_key_write"
	"LINKER:SHELL:-wrap,hal_otp_ssz_lock"
	"LINKER:SHELL:-wrap,hal_sys_spic_boot_finish"
	"LINKER:SHELL:-wrap,hal_sys_spic_ddr_ctrl"
	"LINKER:SHELL:-wrap,hal_sys_spic_phy_en"
	"LINKER:SHELL:-wrap,hal_sys_spic_set_phy_delay"
	"LINKER:SHELL:-wrap,hal_sys_spic_read_phy_delay"
	"LINKER:SHELL:-wrap,hal_sys_bt_uart_mux"
	"LINKER:SHELL:-wrap,hal_pwm_clock_init"
	"LINKER:SHELL:-wrap,hal_pwm_clk_sel"
	"LINKER:SHELL:-wrap,hal_timer_clock_init"
	"LINKER:SHELL:-wrap,hal_timer_group_sclk_sel"
	"LINKER:SHELL:-wrap,hal_sys_get_ld_fw_idx"
	"LINKER:SHELL:-wrap,hal_sys_get_boot_select"
	"LINKER:SHELL:-wrap,hal_sys_dbg_port_cfg"
	"LINKER:SHELL:-wrap,hal_otp_byte_rd_sys"
	"LINKER:SHELL:-wrap,hal_crypto_engine_init_s4ns"
	"LINKER:SHELL:-wrap,hal_sys_get_chip_id"
	"LINKER:SHELL:-wrap,hal_sys_get_video_img_ld_offset"
	"LINKER:SHELL:-wrap,hal_sys_cust_pws_val_ctrl"
	"LINKER:SHELL:-wrap,hal_32k_s1_sel"
	"LINKER:SHELL:-wrap,hal_xtal_divider_enable"
	"LINKER:SHELL:-wrap,hal_sys_clear_prefw_boot_fail_sts"
)
endif()

set_target_properties(${app} PROPERTIES LINK_DEPENDS ${ld_script})


add_custom_command(TARGET ${app} POST_BUILD 
	COMMAND ${CMAKE_NM} $<TARGET_FILE:${app}> | sort > ${app}.nm.map
	COMMAND ${CMAKE_OBJEDUMP} -d $<TARGET_FILE:${app}> > ${app}.asm
	COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${app}> ${app}.axf
	COMMAND ${CMAKE_OBJCOPY} -j .bluetooth_trace.text -Obinary ${app}.axf APP.trace
	COMMAND ${CMAKE_OBJCOPY} -R .bluetooth_trace.text ${app}.axf 
	COMMAND ${CMAKE_READELF} -s -W $<TARGET_FILE:${app}>  > ${app}.symbols
	
	#COMMAND [ -d output ] || mkdir output
	COMMAND ${CMAKE_COMMAND} -E remove_directory output && ${CMAKE_COMMAND} -E make_directory  output
	COMMAND ${CMAKE_COMMAND} -E copy ${app}.nm.map output
	COMMAND ${CMAKE_COMMAND} -E copy ${app}.asm output
	COMMAND ${CMAKE_COMMAND} -E copy ${app}.map output 
	COMMAND ${CMAKE_COMMAND} -E copy ${app}.axf output
	COMMAND ${CMAKE_COMMAND} -E copy APP.trace output 
	
	COMMAND ${PLAT_COPY} *.a output
)
