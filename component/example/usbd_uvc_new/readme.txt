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
 - Playing on Linux with VLC
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

        Besides the sample clip, the example has two sensor modes built on the new MMF sink
        usbd_uvc_module (module_usbd_uvc.c in this example). They do not use the legacy
        module_uvcd or the usb_class UVC driver.

        Mode (USBD_UVC_NEW_MODE)   Host sees                          Source
        0 SAMPLE (default)         1 camera, H.264 1920x1080 30 fps   built-in clip, no sensor
        1 SINGLE                   1 camera, the host picks:          V1, restarted with the
                                   H.264 / HEVC / MJPEG 1920x1080       format the host commits
                                   30/15/10 fps, NV12 / NV16 1920x1080   (like media_uvcd)
                                   5 fps
        2 DUAL                     2 cameras at the same time:
                                   UVC0 H.264 1920x1080 30 fps        V1, EP1 IN 1 x 1024 B/uframe
                                   UVC1 NV12  640x360   15 fps        V2, EP3 IN 3 x 1024 B/uframe

        Modes 1 and 2 use usbd_uvc_init_multi() of the UVC class: one UVC function (IAD + VC +
        VS) per camera, the configuration descriptor is generated from the format table.
        Raw 1080p is limited to 5 fps by USB (3 x 1024 B per microframe = 24.5 MB/s).

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
                   -Dusb_stack=new -DEXAMPLE=usbd_uvc_new -DUSBD_UVC_NEW_MODE=<0|1|2>
               cmake --build . --target flash
           USBD_UVC_NEW_MODE is cached: it stays until it is passed again.
        3. Open a UVC viewer on the host PC that supports H.264 (e.g. PotPlayer ->
           open a device).

Playing on Linux with VLC
~~~~~~~~~~~~~~~~~~~~~~~~~~
        The board enumerates as a standard UVC device, so the in-kernel uvcvideo driver
        handles it and no vendor driver is needed.

        Start VLC - Media - Open Capture Device.  The "Video device name" list holds one
        "Realtek UVC" entry per camera: in mode 2 the first is UVC0 (H.264 1920x1080 30 fps)
        and the second is UVC1 (NV12 640x360 15 fps).  Pick one and press Play.  Tools -
        Codec Information shows which one is playing.  The device number after the name
        (/dev/videoN) differs between PCs and can be ignored.

        UVC0 plays as it is.  For UVC1 the chroma must be set explicitly or the window stays
        black, because VLC otherwise asks for YUYV or MJPEG - open Tools - Preferences -
        All - Input / Codecs and set the v4l2 chroma to NV12, or start VLC from a shell:
               cvlc v4l2:///dev/video0 :v4l2-width=1920 :v4l2-height=1080 \
                      :v4l2-fps=30 :v4l2-chroma=H264 :live-caching=0 --no-audio
               cvlc v4l2:///dev/video2 :v4l2-width=640 :v4l2-height=360 \
                      :v4l2-fps=15 :v4l2-chroma=NV12 :live-caching=0 --no-audio
        Replace /dev/video0 and /dev/video2 with the numbers the dialog shows.

        To see both cameras at once, open one VLC instance per camera.
        The serial console reports what the host actually receives:
               [UVC] UVC0(H264) on fps=30 drop=0 | UVC1(NV12) on fps=15 drop=0
        A camera without a viewer reports off / fps=0; both channels always run and simply
        drop their frames.

Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Mode 0: the sample H.264 stream is embedded in example_usbd_uvc_new_sample_h264.h.
        Modes 1/2: format tables, video parameters and USB FIFO sizes are at the top of
        example_usbd_uvc_new_media.c.  Dual mode FIFO: RX 912 + EP0 256 + {EP1 256, EP3 768,
        EP5 16} + 8 = 2216 of 2232 DWORD.

Result description
~~~~~~~~~~~~~~~~~~
        On the serial console, UVC device loading log will be printed after reset.
        The host PC should enumerate AmebaPro2 as a UVC camera.  A UVC viewer that supports
        H.264 elementary stream should display the fixed 1920x1080@30fps sample video.

        Modes 1/2 print the frames sent per camera every 5 s while the host streams, e.g.
               [UVC-I] UVC0(H264) on fps=30 drop=0 | UVC1(NV12) on fps=15 drop=0
        Mode 1 logs every format switch:
               [UVC-I] Host streams NV12 1920x1080 5 fps
               [UVC-I] Switch V1 to NV12 1920x1080 5 fps
        Note (mode 2): 640x360 is 16:9 like the sensor image, so the decoded stream is not scaled.

        Console command USBUVC, all modes (full stop / start of the cameras):
               USBUVC=0          stop: worker threads, UVC class and USB core; modes 1/2 also
                                 release the links, video channels, MMF modules and VOE buffers
               USBUVC=1          start again (mode 1: V1 back to H.264 1080p30, mode 0: clip from start)
               USBUVC=2,<n>      n stop/start rounds, prints heap and task count each round
               USBUVC            status, free heap and task count
        While USB DFU owns the port (USBDFU=1), USBUVC=1 is refused; USBDFU=0 brings the cameras
        back only if they were running.

Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported : Ameba-Pro2
        Source code not in project: Ameba-1, Ameba-z, Ameba-D, Ameba-D2
