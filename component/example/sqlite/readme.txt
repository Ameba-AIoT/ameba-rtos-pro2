##################################################################################
#                                                                                #
#                               SQLite EXAMPLE                                   #
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
    SQLite is a lightweight database engine. This is an example for SQLite simple operation


Setup Guide
~~~~~~~~~~~
    1. If using Nand flash, please set enough flash file system space in "project/realtek_amebapro2_v0_example/inc/platform_opts.h" to run this demo
    ```
    #define FLASH_FILESYS_SIZE      (1*1024*1024)
    ```
    2. Build SQLite demo
    ```
    cd project/realtek_amebapro2_v0_example/GCC-RELEASE
    mkdir build
    cd build
    cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=sqlite
    make flash -j4
    ```
    3.  Download firmware to AmebaPro2 and run


Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Insert a SD card.


Result description
~~~~~~~~~~~~~~~~~~
    Example will demonstrate SQLite operation on
        1. SD filesystem
        2. Flash filesystem
        3. RAM filesystem
    If everything works fine, user should see the following log
        ```
        === FINISH SQLite TEST ===
        run sqlite test successfully [3]
        ...
        ...
        =========================================
        TEST RESULT (TOTAL: 3): 3 PASS, 0 FAIL
        =========================================
        ```


Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported :
            Ameba-pro2
