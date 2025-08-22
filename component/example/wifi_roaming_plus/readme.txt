##################################################################################
#                                                                                #
#                        EXAMPLE_WIFI_ROAMING_PLUS                               #
#                                                                                #
##################################################################################

Table of Contents
~~~~~~~~~~~~~~~~~
 - Description
 - Setup Guide
 - Parameter Setting and Configuration
 - Result description 
 - Supported list


Description
~~~~~~~~~~~
    This example realizes wifi roaming among local area network with the same SSID. 
    The roaming strategy is prescan before roaming and store the connected channel 
        to flash for prescan. Also, it can store the sta info to flash for fast DHCP.
    Enable CONFIG_EXAMPLE_WLAN_FAST_CONNECT for fast connection.
    There is 4KB flash for Wi-Fi to store the AP and station infomation for roaming.


Setup Guide
~~~~~~~~~~~
	1. Comment out the following line of code in wlan_initialize()
				wifi_fast_connect_enable(1);
	2. the following instruction to compile this example for RTL8735B
	3. cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=wifi_roaming_plus
	4. cmake --build . --target flash


Parameter Setting and Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    1. Set the RSSI_SCAN_THRESHOLD and RSSI_ROAMING_THRESHOLD and FIND_BETTER_RSSI_DELTA according to the WLAN envrionment.
    2. Config MAX_CH_NUM and MAX_AP_NUM for your application usage. But with more scan MAX_CH_NUM, pre-scan time may takes longer.
    3. IF support 5G channels. enable SUPPORT_SCAN_5G_CHANNEL and set the roaming_channel_plan.
    4. The duration of this example is 85-90seconds. After 85-95 seconds, this example will stop.



Result description
~~~~~~~~~~~~~~~~~~
    The device can connect to a better RSSI AP and store the connection message to flash.


Supported List
~~~~~~~~~~~~~~
[Supported List]
        Supported IC:
            RTL8730A, RTL872XE, RTL8735B