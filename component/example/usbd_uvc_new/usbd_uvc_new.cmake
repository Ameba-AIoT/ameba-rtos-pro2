### example mode ###
# 0: built-in H.264 clip (default), 1: sensor single camera with format switching,
# 2: sensor dual camera (H264 1080p30 + NV12 640x360 15fps). See example_usbd_uvc_new.h.
# Cached: pass -DUSBD_UVC_NEW_MODE=<n> once, it sticks until changed.
set(USBD_UVC_NEW_MODE 0 CACHE STRING "usbd_uvc_new example mode: 0 sample clip, 1 sensor single, 2 sensor dual")
# USB DFU firmware update built in (usbd_dfu_new): console USBDFU=1 hands the USB port to DFU
option(USBD_UVC_NEW_DFU "Build USB DFU (usbd_dfu_new) into usbd_uvc_new" ON)
if(USBD_UVC_NEW_DFU)
	set(USBD_UVC_NEW_DFU_VAL 1)
else()
	set(USBD_UVC_NEW_DFU_VAL 0)
endif()
message(STATUS "usbd_uvc_new mode ${USBD_UVC_NEW_MODE}, DFU ${USBD_UVC_NEW_DFU_VAL}")

### add header files ###
list (
    APPEND app_example_inc_path
    "${sdk_root}/component/example/${EXAMPLE}"
    # mmf2_pro2_video_config.h for the sensor modes
    "${sdk_root}/project/realtek_amebapro2_v0_example/src/mmfv2_video_example"
)

### add source file ###
set(EXAMPLE_SOURCE_PATH)
file(GLOB EXAMPLE_SOURCE_PATH ${CMAKE_CURRENT_LIST_DIR}/*.c)
list(
	APPEND app_example_sources

    ${EXAMPLE_SOURCE_PATH}
)
message(STATUS "${EXAMPLE_SOURCE_PATH}")

# app_inc_path (application.cmake) unconditionally includes
# component/usb/device_new/core, which also has a usbd.h/usbd_config_t (the
# legacy driver's, different shape) - without this, #include "usbd_uvc.h"
# (from component/usb/usb_stack/${usb_stack_ver}/device/uvc) resolves its own #include "usbd.h" to
# that legacy one instead of the ported stack's. Same -iquote fix as libusbd.cmake,
# scoped to just this example's sources.
set_source_files_properties(
	${EXAMPLE_SOURCE_PATH}
	PROPERTIES COMPILE_OPTIONS
	"-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/hal;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/common;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/uvc;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_acm;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_ecm"
	COMPILE_DEFINITIONS
	"USBD_UVC_NEW_MODE=${USBD_UVC_NEW_MODE};USBD_UVC_NEW_DFU=${USBD_UVC_NEW_DFU_VAL}"
)

### built-in USB DFU ###
# example_usbd_dfu_new.c without its app_example.c; DFU_NEW_AUTO_START=0: the cameras own the USB
# port until USBDFU=1 (usbd_dfu_new_usb_release/restore are implemented by this example).
if(USBD_UVC_NEW_DFU)
	set(USBD_UVC_NEW_DFU_SRC ${sdk_root}/component/example/usbd_dfu_new/example_usbd_dfu_new.c)
	list(
		APPEND app_example_sources
		${USBD_UVC_NEW_DFU_SRC}
	)
	list(
		APPEND app_example_inc_path
		"${sdk_root}/component/example/usbd_dfu_new"
	)
	set_source_files_properties(
		${USBD_UVC_NEW_DFU_SRC}
		PROPERTIES COMPILE_OPTIONS
		"-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/hal;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/common;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/dfu"
		COMPILE_DEFINITIONS
		"DFU_NEW_AUTO_START=0"
	)
endif()
