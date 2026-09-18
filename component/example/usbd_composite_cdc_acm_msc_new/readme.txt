##################################################################################
#                                                                                #
#                  Example example_usbd_composite_cdc_acm_msc_new                #
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
        With USB device v2.0 interface, AmebaPro2 can be designed to a composite USB device
        exposing both a CDC ACM virtual COM port and a Mass Storage Class (MSC) disk at the
        same time.  The CDC ACM interface operates as a loopback VCOM, and the MSC interface
        exposes an SD card (default) or a RAM disk as a removable drive to the USB host.

        This example uses the new USB stack (usb_stack/v1.0.0) ported from AmebaPro3.

Setup Guide
~~~~~~~~~~~
        In order to run this application successfully, the hardware setup should be confirmed
        before moving on.
        1. Connect AmebaPro2 to a USB host (e.g. Windows PC) with a USB cable.
        2. For SD card mode (default): connect an SD card to the AmebaPro2 DEV board.
        3. Build and flash (SD card mode, default):
               cd project/realtek_amebapro2_v0_example/GCC-RELEASE
               mkdir build
               cd build
               cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
                   -Dusb_stack=new -DEXAMPLE=usbd_composite_cdc_acm_msc_new
               cmake --build . --target flash
        4. To use RAM disk instead of SD card, add -DUSBD_MSC_SD_MODE=OFF:
               cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
                   -Dusb_stack=new -DEXAMPLE=usbd_composite_cdc_acm_msc_new \
                   -DUSBD_MSC_SD_MODE=OFF
               cmake --build . --target flash

Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        USBD_MSC_SD_MODE (cmake option, default ON):
            ON  - MSC storage backend uses SD card (requires SD card connected to DEV board).
            OFF - MSC storage backend uses an internal RAM disk (no SD card required).

Result description
~~~~~~~~~~~~~~~~~~
        On the serial console, USB composite device loading log will be printed.
        After successful enumeration, the USB host will recognize:
          - A virtual COM port (CDC ACM): loopback, data sent from host is echoed back.
          - A removable drive (MSC): backed by SD card or RAM disk depending on build config.

Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported : Ameba-Pro2
        Source code not in project: Ameba-1, Ameba-z, Ameba-D, Ameba-D2
