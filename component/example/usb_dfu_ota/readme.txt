##################################################################################
#                                                                                #
#                           example_usb_dfu_ota                                  #
#                                                                                #
##################################################################################

Table of Contents
~~~~~~~~~~~~~~~~~
 - Description
 - Setup Guide
 - Result description
 - Supported List

 
Description
~~~~~~~~~~~
        This example shows how to use dfu to update OTA FW

	
Setup Guide
~~~~~~~~~~~
        1.cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=usb_dfu_ota
        2.It need to install the libusb.
	3.After installing the libusb, the device manager will show the usb_dfu_class
	4.dfu-util.exe -d 1d6b:0202 -a 0 -D ota.bin

	
Result description
~~~~~~~~~~~~~~~~~~
        It will update the OTA firmware from the USB DFU OTA procedure, the message will show "Ready to reboot".


Supported List
~~~~~~~~~~~~~~
[Supported List]
	Supported :
	    AmebaPro2
			   
Note:The detail can reference the dfu_tool.zip