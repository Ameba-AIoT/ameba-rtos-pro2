##################################################################################
#                                                                                #
#               example_media_mp4_demuxer                                 	     #
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
        It show how to use the example to demuxer the mp4 video.

	
Setup Guide
~~~~~~~~~~~
        1.cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=media_mp4_demuxer
        2.Record the mp4 file from mmf2_video_example_av_mp4_init and rename to AMEBA_PRO.mp4. 
        2.Insert the sdcard with MP4 video.The name must be AMEBA_PRO.mp4 and burn the firmware into the EVB board.
	3.Execute the example to extract the video and audio files.
	
Result description
~~~~~~~~~~~~~~~~~~
        You can see the ameba_video.h264 and ameba_audio.aac files at sd card. 


Supported List
~~~~~~~~~~~~~~
[Supported List]
	Supported :
	    AmebaPro2

Note: It only support the AmebaPro2 record video.