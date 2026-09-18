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
# component/usb/host_new/core, which also has a usbh.h/usbh_config_t (the
# legacy driver's, different shape) - without this, #include "usbh_msc.h"
# (from component/usb/usb_stack/${usb_stack_ver}/host/msc) resolves its own #include "usbh.h" to
# that legacy one instead of the ported stack's. Same -iquote fix as libusbd.cmake,
# scoped to just this example's sources.
set_source_files_properties(
	${EXAMPLE_SOURCE_PATH}
	PROPERTIES COMPILE_OPTIONS
	"-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/hal;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/common;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_acm;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_ecm;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/msc"
)
