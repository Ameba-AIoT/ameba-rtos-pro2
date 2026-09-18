##################################################################################
#                                                                                #
#                         Example example_usbh_cdc_ecm_new                       #
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
        With USB host v2.0 interface, AmebaPro2 can be designed to a USB CDC ECM (Ethernet
        Control Model) host.  In this application, AmebaPro2 boots up as a USB host that
        connects to a USB-Ethernet adapter (CDC ECM device), obtains an IP address via DHCP,
        and bridges network traffic between the USB Ethernet interface and the LwIP stack.

        This is a standalone (non-composite) CDC ECM host example using the new USB stack
        (usb_stack/v1.0.0).

        Note: USB host mode requires a hardware rework on the DEV board to supply VBUS power
        to the USB OTG connector (see the EVB hardware guide).

Setup Guide
~~~~~~~~~~~
        In order to run this application successfully, the hardware setup should be confirmed
        before moving on.
        1. Apply the USB host VBUS rework on the AmebaPro2 DEV board (see EVB hardware guide).
        2. Connect a USB-Ethernet adapter (CDC ECM class) to the micro USB connector on the
           AmebaPro2 DEV board.
        3. Build and flash:
               cd project/realtek_amebapro2_v0_example/GCC-RELEASE
               mkdir build
               cd build
               cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
                   -Dusb_stack=new -DEXAMPLE=usbh_cdc_ecm_new
               cmake --build . --target flash

Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        CONFIG_ETHERNET=1 is set automatically by usbh_cdc_ecm_new.cmake.

        Note: when both Wi-Fi and the USB Ethernet interface are active, the default route
        is switched to the USB Ethernet interface after ECM link-up and DHCP success, which
        will affect the existing Wi-Fi outbound traffic path.

Result description
~~~~~~~~~~~~~~~~~~
        On the serial console, USB CDC ECM host loading and DHCP log will be printed.
        After a successful DHCP lease, the board can send and receive network traffic through
        the USB-Ethernet adapter.  USB hotplug (detach/re-attach) is supported.

Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported : Ameba-Pro2
        Source code not in project: Ameba-1, Ameba-z, Ameba-D, Ameba-D2
