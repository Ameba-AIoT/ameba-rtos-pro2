##################################################################################
#                                                                                #
#               example_media_video_to_storage                                 	 #
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
        This example is for video to save example.It show how to use the example to record the mp4 video to sd card

	
Setup Guide
~~~~~~~~~~~
        1.cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=media_video_to_storage
        2.Insert the sdcard and burn the image to into the EVB board.
	3.Enter the MSTA to record the 10s video.
	4.Enter the MQUI to quit the video
	
Result description
~~~~~~~~~~~~~~~~~~
        Use the upper steps to see the mp4 files at the sdcard.


Supported List
~~~~~~~~~~~~~~
[Supported List]
	Supported :
	    AmebaPro2