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

	# usbd_composite_hid.c/usbd_composite_msc_hid.c only reference macros
	# (USBD_COMP_HID_*) defined by usbd_composite_config.h's HID-including
	# branches - only one CONFIG_USBD_COMPOSITE_* combo may be active per
	# build (see the #if/#elif chain there), so these two are
	# EXAMPLE-conditional rather than unconditionally compiled in
	# application.cmake alongside the rest of the ported-stack class drivers.
	${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/composite/usbd_composite_hid.c
	${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/composite/usbd_composite_msc_hid.c
)
message(STATUS "${EXAMPLE_SOURCE_PATH}")

# app_inc_path (application.cmake) unconditionally includes
# component/usb/device_new/core, which also has a usbd.h/usbd_config_t (the
# legacy driver's, different shape) - without this, #include
# "usbd_composite_msc_hid.h" (from component/usb/usb_stack/${usb_stack_ver}/device/composite)
# resolves its own #include "usbd.h" to that legacy one instead of the ported stack's.
# Same -iquote fix as libusbd.cmake, scoped to just this example's sources.
set_source_files_properties(
	${EXAMPLE_SOURCE_PATH}
	${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/composite/usbd_composite_hid.c
	${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/composite/usbd_composite_msc_hid.c
	PROPERTIES COMPILE_OPTIONS
	"-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/hal;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/common;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/cdc_acm;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/msc;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/composite;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_acm;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_ecm"
)

### add flags ###
# CONFIG_USBD_COMPOSITE_MSC_HID selects the MSC + HID branch of
# component/usb/usb_stack/${usb_stack_ver}/device/composite/usbd_composite_config.h. Must match
# the composite dispatcher sources chosen above - same macro, same header,
# both this example's sources and the unconditionally-compiled
# usbd_composite_msc.c/usbd_composite_scsi.c (application.cmake) need to
# agree on which CONFIG_USBD_COMPOSITE_* branch is active.
list(
	APPEND app_example_flags
	CONFIG_USBD_COMPOSITE_MSC_HID=1
)
