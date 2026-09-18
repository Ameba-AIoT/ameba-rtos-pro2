##################################################################################
#                                                                                #
#                          Example example_usbd_msc_new                          #
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
        With USB device v2.0 interface, AmebaPro2 can be designed to a USB Mass Storage
        Class (MSC) device.  In this application, AmebaPro2 boots up as a USB mass storage
        device; the USB host (e.g. a Windows PC) can recognize it and read/write files
        through the USB interface.

        The storage backend is an SD card by default.  A RAM disk backend is also available
        for quick testing without an SD card (see Parameter Setting below).

        This example uses the new USB stack (usb_stack/v1.0.0) standalone MSC driver ported
        from AmebaPro3.  USB hotplug (detach/re-attach) is supported.

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
                   -Dusb_stack=new -DEXAMPLE=usbd_msc_new
               cmake --build . --target flash
        4. To use RAM disk instead of SD card, add -DUSBD_MSC_SD_MODE=OFF:
               cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
                   -Dusb_stack=new -DEXAMPLE=usbd_msc_new -DUSBD_MSC_SD_MODE=OFF
               cmake --build . --target flash

Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        USBD_MSC_SD_MODE (cmake option, default ON):
            ON  - Storage backend uses SD card via fatfs_sdcard_api.
                  Requires an SD card connected to the DEV board.
            OFF - Storage backend uses an internal RAM disk (no SD card required).
                  Useful for quick functional verification without hardware.

Result description
~~~~~~~~~~~~~~~~~~
        On the serial console, USB MSC loading log will be printed; confirm there are no
        errors.  After the MSC driver is successfully loaded, the USB host recognizes
        AmebaPro2 as a removable drive.  Files can be read from and written to the SD card
        (or RAM disk) via the standard OS file manager.

        USB hotplug is supported: unplug and replug the USB cable; the console prints
        DETACHED -> ATTACHED and the drive re-appears on the host without a board reset.

Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported : Ameba-Pro2
        Source code not in project: Ameba-1, Ameba-z, Ameba-D, Ameba-D2
