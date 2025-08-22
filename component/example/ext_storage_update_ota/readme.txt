##################################################################################
#                                                                                #
#                      OTA OVER EXT_STORAGE UPDATING EXAMPLE                     #
#                                                                                #
##################################################################################

Date: 2024-12-13

Table of Contents
~~~~~~~~~~~~~~~~~
 - Description
 - Setup Guide
 - Parameter Setting and Configuration
 - Result description
 - Supported List


Description
~~~~~~~~~~~
    This example shows how to download ota.bin from external storage device such as sd card or emmc if target file presented.
    To perform this example, please make sure the target OTA file is within external storage device file system.
    Also, we recommend to rename the OTA file as some vendor specified name, e.g. ota_is_realtek.bin.


Setup Guide
~~~~~~~~~~~
        1. Enable EXT_STORAGE_OTA_UPDATE in [ota_8735b.h]
        /* For ext_storage ota update example */
        #define EXT_STORAGE_OTA_UPDATE


Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Modify FILENAME as the target OTA file in SD card or emmc.
        eg: #define FILENAME    "emmc:/ota_is_realtek.bin"	
	    #define FILENAME    "sd:/ota_is_realtek.bin"

	Define EXT_STORAGE to VFS_INF_EMMC or VFS_INF_SD as the target external storage device
	eg: #define EXT_STORAGE   VFS_INF_EMMC 
	    #define EXT_STORAGE   VFS_INF_SD    

Result description
~~~~~~~~~~~~~~~~~~
    A OTA over ext_storage example thread will be started automatically when booting.


Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported :
            Ameba-pro2, 
        Source code not in project:
            Ameba-1, Ameba-z, Ameba-pro, AmebaD



