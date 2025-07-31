##################################################################################
#                                                                                #
#                           example_media_uvc                                 	 #
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
        An example of transferring video to a PC via USB.

	
Setup Guide
~~~~~~~~~~~
        1.cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=media_uvcd
        2.Connect the USB device port from the PRO2 to the PC side and burn the firmware into EVB board.
		3.Use the Pot Player to see the image, the fast key ALT+D to open the network camera(USB UVC CLASS).

	
Result description
~~~~~~~~~~~~~~~~~~
        It can see the image from Pot player.


Supported List
~~~~~~~~~~~~~~
[Supported List]
	Supported :
	    AmebaPro2