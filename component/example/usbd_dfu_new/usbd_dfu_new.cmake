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

# app_inc_path also carries component/usb/device_new/core (legacy usbd.h), so resolve
# "usbd_dfu.h" -> "usbd.h" to the usb_stack/${usb_stack_ver} headers with -iquote, scoped to this
# example's sources (same as the other *_new examples).
set_source_files_properties(
	${EXAMPLE_SOURCE_PATH}
	PROPERTIES COMPILE_OPTIONS
	"-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/hal;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/common;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/dfu"
)

### add flags ###
# DFU feature switches (defaults in usbd_dfu.h). usbd_dfu.c is a shared compilation unit, so set
# them here to change them, e.g.:
#   CONFIG_USBD_DFU_WILL_DETACH=1
#   CONFIG_USBD_DFU_MANIFESTATION_TOLERANT=1
list(
	APPEND app_example_flags
)
