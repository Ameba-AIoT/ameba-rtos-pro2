##################################################################################
#                                                                                #
#               Example example_usbh_composite_cdc_acm_ecm_new                   #
#                                                                                #
##################################################################################

Date: 2026-09-17

Table of Contents
~~~~~~~~~~~~~~~~~
 - Description
 - Setup Guide
 - Parameter Setting and Configuration
 - Result description
 - Supported List


Description
~~~~~~~~~~~
        With USB host v2.0 interface, AmebaPro2 can be designed to a composite USB host that
        simultaneously communicates with a 4G LTE dongle exposing two interfaces:
          - CDC ACM interface: used for AT command diagnostics (cellular dial-up control).
          - CDC ECM interface: used as a USB Ethernet link to route network traffic through
            the 4G dongle via the LwIP network stack.

        This example uses the new USB stack (usb_stack/v1.0.0) and currently supports
        Quectel EG915 only.

        Note: USB host mode requires a hardware rework on the DEV board to supply VBUS power
        to the USB OTG connector (see the EVB hardware guide).

Setup Guide
~~~~~~~~~~~
        In order to run this application successfully, the hardware setup should be confirmed
        before moving on.
        1. Apply the USB host VBUS rework on the AmebaPro2 DEV board (see EVB hardware guide).
        2. Connect a Quectel EG915 4G dongle to the micro USB connector on the AmebaPro2
           DEV board.
        3. Build and flash:
               cd project/realtek_amebapro2_v0_example/GCC-RELEASE
               mkdir build
               cd build
               cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
                   -Dusb_stack=new -DEXAMPLE=usbh_composite_cdc_acm_ecm_new
               cmake --build . --target flash

Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        CONFIG_ETHERNET=1 and CONFIG_USBH_CDC_ACM_4G_DONGLE=1 are set automatically by
        usbh_composite_cdc_acm_ecm_new.cmake.

Result description
~~~~~~~~~~~~~~~~~~
        On the serial console, USB composite host enumeration, AT command handshake, and
        DHCP log will be printed.  After a successful DHCP lease via the ECM interface,
        the board routes network traffic through the 4G dongle.

Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported : Ameba-Pro2
        Source code not in project: Ameba-1, Ameba-z, Ameba-D, Ameba-D2
