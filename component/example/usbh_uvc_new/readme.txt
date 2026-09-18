##################################################################################
#                                                                                #
#                         Example example_usbh_uvc                          #
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
        With USB host v2.0 interface, AmebaPro2 can be designed to a USB UVC (USB Video
        Class) host.  In this application, AmebaPro2 boots up as a USB host that connects
        to a USB camera and processes captured frames in one of three modes:

          USBH_UVC_APP_EMPTY  - capture frames and discard, report throughput
          USBH_UVC_APP_FATFS  - capture raw JPEG or YUY2 frames, save to SD card
          USBH_UVC_APP_RGB    - capture frames, convert to RGB888, save as PPM to SD card
                                [default]

        This example uses the new USB stack (usb_stack/v1.0.0) host UVC driver ported from
        AmebaPro3.  The driver always uses the software combine-thread path
        (USBH_UVC_USE_HW=0); hardware UVC decode is not available on AmebaPro2/Pro3.

        Note: USB host mode requires a hardware rework on the DEV board to supply VBUS power
        to the USB OTG connector (see the EVB hardware guide).

Setup Guide
~~~~~~~~~~~
        In order to run this application successfully, the hardware setup should be confirmed
        before moving on.
        1. Apply the USB host VBUS rework on the AmebaPro2 DEV board (see EVB hardware guide).
        2. Connect a USB camera (UVC-compatible) to the micro USB connector on the
           AmebaPro2 DEV board.
        3. For FATFS / RGB mode: connect an SD card to the AmebaPro2 DEV board.
        4. Build and flash (RGB mode, default):
               cd project/realtek_amebapro2_v0_example/GCC-RELEASE
               mkdir build
               cd build
               cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
                   -Dusb_stack=new -DEXAMPLE=usbh_uvc_new
               cmake --build . --target flash
        5. To select a different mode, add -DUSBH_UVC_APP_MODE=<mode>:
               cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
                   -Dusb_stack=new -DEXAMPLE=usbh_uvc_new -DUSBH_UVC_APP_MODE=EMPTY
               cmake .. ... -DUSBH_UVC_APP_MODE=FATFS
               cmake .. ... -DUSBH_UVC_APP_MODE=RGB    (default)
        5. Verify with a USB camera known to work on another platform before testing a
           new one.

Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        USBH_UVC_APP_MODE (cmake option, set via -DUSBH_UVC_APP_MODE=<mode>):
            EMPTY  - throughput measurement only.               -> CONFIG_USBH_UVC_APP=1
            FATFS  - save raw JPEG/YUY2 to SD card.            -> CONFIG_USBH_UVC_APP=2
            RGB    - convert to RGB888, save PPM. [default]    -> CONFIG_USBH_UVC_APP=3
            (Default fallback also defined in example_usbh_uvc_new.c via #ifndef guard)

        Detail per mode:
            USBH_UVC_APP_EMPTY  (1) - capture and discard, measure throughput.
                                      After CONFIG_USBH_UVC_LOOP frames, prints throughput
                                      and waits for the next camera attach/re-setup.
                                      On truncated frame (frame size >= buffer size),
                                      the task exits and is re-created on next attach.
            USBH_UVC_APP_FATFS  (2) - save raw JPEG (.jpg) or YUY2 (.yuy2) to SD card.
                                      Saves every USBH_UVC_SAVE_EVERY_N-th frame after
                                      USBH_UVC_WARMUP_FRAMES warmup frames are discarded.
                                      Stops after USBH_UVC_MAX_IMAGES images are saved.
            USBH_UVC_APP_RGB    (3) - convert to RGB888, save as PPM (.ppm) to SD card.
                                      Same cadence as FATFS mode.  [default]

        CONFIG_USBH_UVC_FORMAT_TYPE:
            USBH_UVC_FORMAT_MJPEG (default) or USBH_UVC_FORMAT_YUV.

        CONFIG_USBH_UVC_WIDTH / CONFIG_USBH_UVC_HEIGHT / CONFIG_USBH_UVC_FRAME_RATE:
            Target resolution and frame rate. Default: 640x480@15fps.
            If the camera does not support these values, the driver automatically
            negotiates the closest supported parameters; check the log to confirm
            the actual values applied.

        CONFIG_USBH_UVC_FRAME_BUF_SIZE:
            Per-frame DMA buffer size in bytes (default 640 KB).
            Increase if the console shows "truncated" frame errors.

        CONFIG_USBH_UVC_LOOP:
            Number of frames to capture per round in EMPTY mode (default 200).

        USBH_UVC_MAX_IMAGES:
            Total images to save before stopping in FATFS/RGB mode (default 500).

        USBH_UVC_WARMUP_FRAMES:
            Number of frames discarded at start before saving begins (default 30).
            Allows the camera's auto-exposure and white-balance to stabilise.

        USBH_UVC_SAVE_EVERY_N:
            Save every N-th captured frame in FATFS/RGB mode (default 15).

Result description
~~~~~~~~~~~~~~~~~~
        EMPTY mode:
            Captures CONFIG_USBH_UVC_LOOP frames and prints throughput statistics:
                [UVC] TP <KB/s> KB/s @ <ms> ms, fps <N>/s
                [UVC] TP <X>.<YY> MB/s (<bytes> bytes)
            On completion, waits for the next camera setup event (hotplug re-attach).
            On truncated frame, the task exits; a new task is created on next attach.

        FATFS mode:
            After USBH_UVC_WARMUP_FRAMES warmup frames, saves every USBH_UVC_SAVE_EVERY_N-th
            frame to the SD card root as img0.jpg (MJPEG) or img0.yuy2 (YUY2), img1, ...
            Console prints filename and byte count for each saved file.
            Stops automatically after USBH_UVC_MAX_IMAGES images.

        RGB mode:
            Same as FATFS mode but converts each frame to RGB888 before saving.
            MJPEG frames are decoded with stb_image (standard Huffman tables injected
            automatically if the camera omits the DHT segment).
            YUY2 frames are converted inline.
            Output files: img0.ppm, img1.ppm ... (PPM P6 format, viewable in GIMP or
            ImageMagick).  Console prints decoded resolution and byte count.

        USB hotplug (detach/re-attach) is supported in all modes.

Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported : Ameba-Pro2
        Source code not in project: Ameba-1, Ameba-z, Ameba-D, Ameba-D2
