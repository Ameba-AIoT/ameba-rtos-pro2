cmake_minimum_required(VERSION 3.6)
include( ${prj_root}/GCC-RELEASE/includepath.cmake )

project(usrsctp)

set(usrsctp usrsctp)

list(
    APPEND usrsctp_sources

###netinet
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_asconf.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_auth.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_bsd_addr.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_callout.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_cc_functions.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_crc32.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_indata.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_input.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_output.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_pcb.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_peeloff.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_sha1.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_ss_functions.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_sysctl.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_timer.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_userspace.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctp_usrreq.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet/sctputil.c
###netinet6
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet6/sctp6_usrreq.c

    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/user_environment.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/user_mbuf.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/user_recv_thread.c
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/user_socket.c
)

list(
    APPEND usrsctp_include
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet
    ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/usrsctp/usrsctplib/netinet6
)

add_library( 
    ${usrsctp} STATIC
    ${usrsctp_sources}
)

list(
    APPEND usrsctp_flags
    CONFIG_BUILD_RAM=1 
    CONFIG_BUILD_LIB=1 
    CONFIG_PLATFORM_8735B
    CONFIG_RTL8735B_PLATFORM=1
    CONFIG_SYSTEM_TIME64=1
    
    __Userspace__
    SCTP_SIMPLE_ALLOCATOR
    SCTP_PROCESS_LEVEL_LOCKS
    SCTP_USE_MBEDTLS_SHA1
    SCTP_USE_LWIP
    SCTP_USE_RTOS
    SCTP_DEBUG
    INET
    # INET6
    
    # HAVE_STDATOMIC_H
    HAVE_SA_LEN
    HAVE_SIN_LEN
    #HAVE_SIN6_LEN
    HAVE_SCONN_LEN

    KVS_PLAT_RTK_FREERTOS
)

target_compile_definitions( ${usrsctp} PUBLIC ${usrsctp_flags} )

target_include_directories( ${usrsctp}
    PRIVATE
        ${prj_root}/src/amazon_kvs/lib_amazon_v2/gcc_include
    PUBLIC
        ${inc_path}
        ${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure
        ${usrsctp_include}
)

### add linked library ###
list(
    APPEND app_example_lib
    usrsctp
)
