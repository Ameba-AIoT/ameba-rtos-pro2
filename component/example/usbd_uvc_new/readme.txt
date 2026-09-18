##################################################################################
#                                                                                #
#                          Example example_usbd_uvc_new                          #
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
        With USB device v2.0 interface, AmebaPro2 can be designed to a USB UVC (USB Video
        Class) device, presenting itself as a USB camera to the host PC.  In this application,
        AmebaPro2 streams a fixed 1920x1080@30fps H.264 sample to the PC host; any UVC viewer
        that supports H.264 elementary stream (e.g. PotPlayer) can display the video.

        This example uses the new USB stack (usb_stack/v1.0.0) device UVC driver ported from
        AmebaPro3.  Note: this driver is independent of the legacy usb_class UVC driver and
        uses rename macros to avoid symbol conflicts.

        There is no USB hotplug support in this example because the UVC driver does not
        implement the .status_changed callback.

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
                   -Dusb_stack=new -DEXAMPLE=usbd_uvc_new
               cmake --build . --target flash
        3. Open a UVC viewer on the host PC that supports H.264 (e.g. PotPlayer ->
           open a device).

Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        None.  The sample H.264 stream is embedded in example_usbd_uvc_new_sample_h264.h.

Result description
~~~~~~~~~~~~~~~~~~
        On the serial console, UVC device loading log will be printed after reset.
        The host PC should enumerate AmebaPro2 as a UVC camera.  A UVC viewer that supports
        H.264 elementary stream should display the fixed 1920x1080@30fps sample video.

Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported : Ameba-Pro2
        Source code not in project: Ameba-1, Ameba-z, Ameba-D, Ameba-D2
