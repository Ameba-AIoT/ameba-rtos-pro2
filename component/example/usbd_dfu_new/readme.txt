##################################################################################
#                                                                                #
#                           example_usbd_dfu_new                                 #
#                                                                                #
##################################################################################

Table of Contents
~~~~~~~~~~~~~~~~~
 - Description
 - Setup Guide
 - Result description
 - Note
 - Supported List


Description
~~~~~~~~~~~
        USB DFU 1.1 firmware update on the usb_stack/v1.0.0 device stack
        (class driver: component/usb/usb_stack/v1.0.0/device/dfu/usbd_dfu.c).

        It combines the AmebaPro3 usbd_dfu flow with the AmebaPro2 usb_dfu_ota image handling:
        1. The device enumerates in DFU Run-Time mode (appIDLE) as 0BDA:8D00 "Realtek DFU Device".
        2. dfu-util sends DFU_DETACH, the device re-enumerates by itself in DFU mode (dfuIDLE).
        3. ota.bin is downloaded block by block (wTransferSize 4096) and written to the firmware
           slot that is not running (FW1 <-> FW2). Blocks are programmed from a task, the host
           sees dfuDNBUSY meanwhile. Block 0 must carry the "RTL8735B" image label, anything
           else is rejected at once (errWRITE).
        4. Power-loss safety: block 0 (the image header) is held back in RAM and written as 0xFF
           until the end, so an interrupted download leaves a slot with a blank header, which
           the bootloader skips.
        5. After the last block, the byte-sum of the received data is compared with the checksum
           in the last 4 bytes of ota.bin. Only then is the header written and the slot closed
           (on NAND this commits the partition table), and the slot is read back once more.
        6. On success the record of an earlier load failure of that slot (AON register, which
           would make the bootloader skip it until power-off) is cleared and the board reboots
           into the new firmware. On failure the slot header is voided, the old firmware keeps
           booting, and dfu-util reports errVERIFY (or errWRITE for a flash write failure).

        Set DFU_NEW_DFU_MODE_ONLY to 1 in example_usbd_dfu_new.c to enumerate directly in DFU
        mode like the legacy usb_dfu_ota example (no DFU_DETACH step).


Setup Guide
~~~~~~~~~~~
        1. Build:
               cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
                   -Dusb_stack=new -DEXAMPLE=usbd_dfu_new
               cmake --build . --target flash
        2. Download the image with the Image Tool and reset the board.
        3. Build the new firmware to update to; its build output contains ota.bin.
        4. Windows: bind WinUSB to "Realtek DFU Device" (USB ID 0BDA 8D00) with Zadig
           (Options -> List All Devices). Bind it in Run-Time mode and again after the first
           DFU_DETACH if Windows lists the DFU-mode device separately. Do not use libusb-win32
           (libusb0): it rejects zero-length control OUT requests, so DFU_DETACH and the final
           zero-length DFU_DNLOAD fail.
        5. On the host:
               dfu-util -l
               dfu-util -d 0bda:8d00 -D ota.bin

        Console command:
               USBDFU=1        start USB DFU (started automatically at boot)
               USBDFU=0        stop USB DFU


Result description
~~~~~~~~~~~~~~~~~~
        [DFU] Init in Run-Time mode, xfer 4096
        [DFU] Started, run: dfu-util -d 0bda:8d00 -D ota.bin
        [DFU] DFU_DETACH
        [DFU] Re-enumerate in DFU mode
        [DFU] Re-enumerating
        [DFU] Init in DFU mode, xfer 4096
        [DFU] Running FW1, update FW2
        [DFU] Written ... bytes
        [DFU] Received ... bytes, checksum file 0x..., received 0x...
        [DFU] Header written, verify FW2
        [DFU] Checksum file 0x..., flash 0x...
        [DFU] Verify OK, ready to reboot
        [DFU] Manifest done
        [DFU] Reboot
        === Load FW2 ===

        "=== Load FWx ===" alone does not prove the boot: on a load failure the bootloader prints
        "[BOOT Err] load img(x) fail!" and falls back to the other slot.


Note
~~~~
        - "Invalid DFU suffix signature" printed by dfu-util can be ignored, ota.bin carries no
          DFU suffix.
        - Restarting dfu-util -D starts a new session from block 0; an unfinished session is
          aborted and its slot header voided. USBDFU=0 does the same.
        - DFU_UPLOAD is disabled (CONFIG_USBD_DFU_CAN_UPLOAD=0).
        - Feature switches CONFIG_USBD_DFU_WILL_DETACH / CONFIG_USBD_DFU_MANIFESTATION_TOLERANT /
          CONFIG_USBD_DFU_CAN_UPLOAD default in usbd_dfu.h and can be overridden in
          usbd_dfu_new.cmake.


Supported List
~~~~~~~~~~~~~~
        AmebaPro2
