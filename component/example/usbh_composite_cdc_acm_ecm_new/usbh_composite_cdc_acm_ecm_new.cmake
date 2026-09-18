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
# legacy driver's, different shape) - without this, #include "usbh_cdc_acm.h"
# (from component/usb/usb_stack/${usb_stack_ver}/host/cdc_acm) resolves its own #include "usbh.h"
# to that legacy one instead of the ported stack's. Same -iquote fix as libusbd.cmake,
# scoped to just this example's sources.
set_source_files_properties(
	${EXAMPLE_SOURCE_PATH}
	PROPERTIES COMPILE_OPTIONS
	"-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/hal;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/common;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/device/cdc_acm;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/core;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_acm;-iquote;${sdk_root}/component/usb/usb_stack/${usb_stack_ver}/host/cdc_ecm"
)

### add flags ###
# CONFIG_ETHERNET must be project-wide (target_compile_definitions(${app}...)
# in application.cmake applies app_example_flags globally, including to
# component/lwip/api/lwip_netconf.c) so NET_IF_NUM/xnetif[] sizing and the
# LwIP_Init()-registered ethernetif_mii_init() netif agree with what this
# example expects. platform_opts.h's CONFIG_ETHERNET is #ifndef-guarded
# specifically so this override doesn't collide with it (default stays 0 for
# every other build). CONFIG_USBH_CDC_ACM_4G_DONGLE changes
# usbh_cdc_acm_cb_t/usbh_cdc_acm_param_t's layout (see
# component/usb/usb_stack/${usb_stack_ver}/host/cdc_acm/usbh_cdc_acm.h) - usbh_cdc_acm.c is
# unconditionally compiled (application.cmake), so this example must set the
# same value it expects when calling into that driver, or they'll disagree
# on struct layout.
list(
	APPEND app_example_flags
	CONFIG_ETHERNET=1
	CONFIG_USBH_CDC_ACM_4G_DONGLE=1
)
