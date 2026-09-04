### add lib ###
list(
	APPEND app_example_lib
)

### add flags ###
list(
	APPEND app_example_flags
	PSRAM_BSS_SECTION=SDRAM_BSS_SECTION
)

### add header files ###
list (
	APPEND app_example_inc_path
	${CMAKE_CURRENT_LIST_DIR}/../../usb/host_new/uvc
	${CMAKE_CURRENT_LIST_DIR}/../../image/3rdparty/stb
	${CMAKE_CURRENT_LIST_DIR}/../../video/driver/common/include
)

### add source file ###
list(
	APPEND app_example_sources
	app_example.c
	example_usbh_uvc_new.c
	uvc_to_rgb.c
)
list(TRANSFORM app_example_sources PREPEND ${CMAKE_CURRENT_LIST_DIR}/ REGEX "^(app_example|example_|uvc_)")

