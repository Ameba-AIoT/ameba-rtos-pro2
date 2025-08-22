##################################################################################
#                                                                                #
#                               Fmp4 EXAMPLE                                     #
#                                                                                #
##################################################################################

Date: 2023-12-26

Table of Contents
~~~~~~~~~~~~~~~~~
 - Description
 - Setup Guide
 - Parameter Setting and Configuration
 - Result description
 - Supported List


Description
~~~~~~~~~~~
    Fmp4(Fragmented MP4)
    This is an example for recording a streaming data(H264/AAC) as a fmp4 file in RAM file system.


Setup Guide
~~~~~~~~~~~
    1. Build Fmp4 demo
    ```
    cd project/realtek_amebapro2_v0_example/GCC-RELEASE
    mkdir build
    cd build
    cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=media_fmp4
    make flash -j4
    ```
    2.  Download firmware to AmebaPro2 and run


Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Insert a SD card.


Result description
~~~~~~~~~~~~~~~~~~
    The example will demo how to record a 15s fmp4 file to RAM file system, and then write the file to SD card.
    Then, user can validate the fmp4 file in SD card


Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported :
            Ameba-pro2
