### add header files ###
list (
    APPEND app_example_inc_path
    "${sdk_root}/component/example/${EXAMPLE}"
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
# legacy driver's, different shape) - without this, #include "usbd_hid.h"
# (from component/usb/usb_stack/${usb_stack_ver}/device/hid) resolves its own #include "usbd.h" to
# that legacy one instead of the ported stack's. Same -iquote fix as libusbd.cmake,
# scoped to just this example's sources.
set_source_files_properties(
	${EXAMPLE_SOURCE_PATH}
	PROPERTIES COMPILE_OPTIONS
	"-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/hal;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/common;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/hid;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_acm;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_ecm"
)

### add flags ###
# CONFIG_USBD_HID_MOUSE selects usbd_hid.c/.h's mouse report descriptor and
# 1-endpoint (INTR IN only) config descriptor branch over its default
# keyboard branch (which also needs CONFIG_USBD_HID_KEYBOARD for the INTR
# OUT endpoint - not defined here, matching this example's mouse-only
# scope). component/usb/usb_stack/${usb_stack_ver}/device/hid/usbd_hid.c is unconditionally
# compiled (application.cmake), so this flag must be set here to steer that
# shared compilation unit's #ifdef branch for this example's build.
list(
	APPEND app_example_flags
	CONFIG_USBD_HID_MOUSE=1
)
