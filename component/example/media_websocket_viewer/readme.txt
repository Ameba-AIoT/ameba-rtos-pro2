1.The example only supports the H.264 format.

2.Please copy htdocs.bin to the root directory of the SD card.
	Format the SD card as FAT32 using Windows OS.

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
