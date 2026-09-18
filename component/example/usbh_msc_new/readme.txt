##################################################################################
#                                                                                #
#                         Example example_usbh_msc_new                           #
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
        With USB host v2.0 interface, AmebaPro2 can be designed to a USB Mass
        Storage Class (MSC) host. In this application, AmebaPro2 boots up as a
        USB host that connects to a USB flash disk, mounts its FAT filesystem,
        and runs read/write throughput tests on test files.

        This is a standalone (non-composite) MSC host example using the new USB
        stack (usb_stack/v1.0.0).

        Note: USB host mode requires a hardware rework on the DEV board to supply
        VBUS power to the USB OTG connector (see the EVB hardware guide).

Setup Guide
~~~~~~~~~~~
        In order to run this application successfully, the hardware setup should be confirmed
        before moving on.
        1. Apply the USB host VBUS rework on the AmebaPro2 DEV board (see EVB hardware guide).
        2. Connect a FAT-formatted USB flash disk to the micro USB connector on the
           AmebaPro2 DEV board.
        3. Build and flash:
               cd project/realtek_amebapro2_v0_example/GCC-RELEASE
               mkdir build
               cd build
               cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
                   -Dusb_stack=new -DEXAMPLE=usbh_msc_new
               cmake --build . --target flash

Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        CONFIG_USBH_MSC_HOTPLUG is set to 0 by default. After the throughput test
        finishes, unplug and replug the USB flash disk to run the test again.

        Set CONFIG_USBH_MSC_HOTPLUG to 1 in example_usbh_msc_new.c to continuously
        repeat the test and rebuild the USB stack after a device detach.

Result description
~~~~~~~~~~~~~~~~~~
        On the serial console, USB MSC host mount and read/write throughput logs
        will be printed. After the test completes, unplug and replug the USB flash
        disk to run the test again. USB hotplug handling is supported when
        CONFIG_USBH_MSC_HOTPLUG is enabled.

Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported : Ameba-Pro2
        Source code not in project: Ameba-1, Ameba-z, Ameba-D, Ameba-D2
