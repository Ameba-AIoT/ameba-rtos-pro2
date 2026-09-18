##################################################################################
#                                                                                #
#                          Example example_usbd_hid_new                          #
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
        With USB device v2.0 interface, AmebaPro2 can be designed to a USB HID (Human Interface
        Device) class device.  In this application, AmebaPro2 boots up as a standard USB mouse.
        Mouse movement and button clicks are driven via the MOUSE serial console command.

        This example uses the new USB stack (usb_stack/v1.0.0) ported from AmebaPro3.
        The HID driver is built in mouse mode (CONFIG_USBD_HID_MOUSE=1).

Setup Guide
~~~~~~~~~~~
        In order to run this application successfully, the hardware setup should be confirmed
        before moving on.
        1. Connect AmebaPro2 to a USB host (e.g. Windows PC) with a USB cable.
        2. Build and flash:
               cd project/realtek_amebapro2_v0_example/GCC-RELEASE
               mkdir build
               cd build
               cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
                   -Dusb_stack=new -DEXAMPLE=usbd_hid_new
               cmake --build . --target flash

Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        CONFIG_USBD_HID_MOUSE is set automatically by usbd_hid_new.cmake; no manual flag needed.

        MOUSE console command (after USB enumeration):
            MOUSE=<left>[,<right>,<middle>,<x_axis>,<y_axis>,<wheel>]
            Example: MOUSE=0,0,0,50,0,0   (move cursor right 50 pixels)
                     MOUSE=1,0,0,0,0,0    (left button press)
                     MOUSE=0,0,0,0,0,0    (release all buttons)
                     MOUSE=0,0,0,0,0,5    (scroll wheel up)

Result description
~~~~~~~~~~~~~~~~~~
        On the serial console, HID device loading log will be printed:
            [HID-I] INIT
            [HID-I] USBD HID demo start

        After successful enumeration, the USB host recognizes AmebaPro2 as a HID-compliant
        mouse (Device Manager shows "HID-compliant mouse", no error code 10).
        Use the MOUSE command to verify cursor movement and button clicks.  The console prints
        the result, e.g.:
            MOUSE=0,0,0,50,0,0
            [HID-I] MOUSE: buttons=f8 x=50 y=0 wheel=0, ret=0   (ret=0: success)

        USB hotplug (detach/re-attach) is supported; the device re-enumerates after replug.

Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported : Ameba-Pro2
        Source code not in project: Ameba-1, Ameba-z, Ameba-D, Ameba-D2
