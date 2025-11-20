1.The example only supports the H.264 format.

2.Load the websocket viewer file.There are two choices.
2.1.Use the array to load the data from htdocs.h.(Default) 
2.2.Enable the websocket marco and copy htdocs.bin to the root directory of the SD card.
	Format the SD card as FAT32 using Windows OS.
	#define WEBSOCKET_FROM_SD

3. Apply patch and build SDK
	- cmake ../ -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=media_websocket_viewer
	- make -j4 flash

4. Please upgrade flash_ntz.bin
    - Regarding how to upgrade firmware, please refer to 'AN0700 Realtek AmebaPro2 application note.en.pdf'

5. Please insert sd card before boot

6. Connect to your AP
	- ATW0=SSID
	- ATW1=PASS
	- ATWC
	- ATW? (Find your IP)

7. Connect to AmebaPro2 with Google Chrome
	- http://your_ip_address/
	
Note:If you want to change the websocket source file,please reference the websocket viewer source file.
	
