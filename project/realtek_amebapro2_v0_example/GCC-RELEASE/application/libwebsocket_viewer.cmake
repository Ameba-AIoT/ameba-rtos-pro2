cmake_minimum_required(VERSION 3.6)

project(websocket_viewer)

include(../includepath.cmake)

set(websocket_viewer websocket_viewer)

list(
    APPEND websocket_viewer_sources

	${sdk_root}/component/application/websocket_viewer/httpd_service.c
	${sdk_root}/component/application/websocket_viewer/htdocs_bin.c
    ${sdk_root}/component/application/websocket_viewer/web_service.c

    # minihttpd sources
    ${sdk_root}/component/application/websocket_viewer/minihttpd/minihttpd.cpp
    ${sdk_root}/component/application/websocket_viewer/minihttpd/ameba/httpd.cpp

    # ministd sources
    ${sdk_root}/component/application/websocket_viewer/minihttpd/ministd/avl_bf.c

    # websocket sources
    ${sdk_root}/component/application/websocket_viewer/wsfs/src/base64.c
    ${sdk_root}/component/application/websocket_viewer/wsfs/src/sha1.c
    ${sdk_root}/component/application/websocket_viewer/wsfs/src/utf8.c
    ${sdk_root}/component/application/websocket_viewer/wsfs/src/wsfs_connection.c
	${sdk_root}/component/application/websocket_viewer/wsfs/src/wsfs_frame_reader.c
	${sdk_root}/component/application/websocket_viewer/wsfs/src/wsfs_mem.c
	${sdk_root}/component/application/websocket_viewer/wsfs/src/wsfs_server.c
)

# include(libversion_file.cmake)

add_library(
    ${websocket_viewer} STATIC
    ${websocket_viewer_sources}
)

list(
	APPEND http_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
)

target_compile_definitions(${websocket_viewer} PRIVATE ${websocket_viewer_flags} )
target_compile_options(${websocket_viewer} PRIVATE ${LIBS_WARN_ERR_FLAGS} )

target_include_directories(
	${websocket_viewer}
	PUBLIC

	${inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure
	${sdk_root}/component/application/websocket_viewer
    ${sdk_root}/component/application/websocket_viewer/minihttpd
    ${sdk_root}/component/application/websocket_viewer/minihttpd/ministd
    ${sdk_root}/component/application/websocket_viewer/minihttpd/ameba/sys_include
    ${sdk_root}/component/application/websocket_viewer/wsfs/include
)

# include(libversion_exec.cmake)