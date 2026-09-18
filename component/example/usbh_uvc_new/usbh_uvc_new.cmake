### add flags ###
list(
	APPEND app_example_flags
	PSRAM_BSS_SECTION=SDRAM_BSS_SECTION
)

# UVC application mode selection.
# Default: RGB (convert frames to RGB888, save as PPM to SD card).
# Override on cmake command line: -DUSBH_UVC_APP_MODE=EMPTY  (throughput only)
#                                 -DUSBH_UVC_APP_MODE=FATFS  (save raw JPEG/YUY2)
#                                 -DUSBH_UVC_APP_MODE=RGB    (save PPM, default)
if(NOT DEFINED USBH_UVC_APP_MODE)
	set(USBH_UVC_APP_MODE "RGB" CACHE STRING "UVC app mode: EMPTY, FATFS, or RGB (default)")
endif()

if(USBH_UVC_APP_MODE STREQUAL "EMPTY")
	list(APPEND app_example_flags CONFIG_USBH_UVC_APP=1)
elseif(USBH_UVC_APP_MODE STREQUAL "FATFS")
	list(APPEND app_example_flags CONFIG_USBH_UVC_APP=2)
else()
	# RGB is the default; also handles any unrecognised value gracefully.
	list(APPEND app_example_flags CONFIG_USBH_UVC_APP=3)
endif()
message(STATUS "USBH_UVC_APP_MODE=${USBH_UVC_APP_MODE} -> CONFIG_USBH_UVC_APP=${CONFIG_USBH_UVC_APP}")

### add header files ###
list(
	APPEND app_example_inc_path
	"${sdk_root}/component/example/${EXAMPLE}"
	"${sdk_root}/component/image/3rdparty/stb"
	"${sdk_root}/component/video/driver/common/include"
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
# component/usb/host_new/core, which also has a usbh.h/usbh_config_t (the
# legacy driver's, different shape) - without this, #include "usbh_uvc.h"
# (from component/usb/usb_stack/${usb_stack_ver}/host/uvc) resolves its own #include "usbh.h" to
# that legacy one instead of the ported stack's. Same -iquote fix as libusbd.cmake,
# scoped to just this example's sources.
set_source_files_properties(
	${EXAMPLE_SOURCE_PATH}
	PROPERTIES COMPILE_OPTIONS
	"-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/hal;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/common;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_acm;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_ecm;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/uvc"
)
