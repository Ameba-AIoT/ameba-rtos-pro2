# Prepare Environment #

- install mingw with ASDK
	- Get Latest version from WD1 share folder or link from build server
	- http://172.21.35.7:8080/userContent/msys64-1210.7z
	- Current version is 20201210
	- extract to proper location
- install cmake
	- https://github.com/Kitware/CMake/releases/download/v3.20.0-rc1/cmake-3.20.0-rc1-windows-x86_64.msi
	- Add location of cmake.exe to PATH
- [optional] install ninja
	- Good for build speed
	- Necessary if want to build from cmd.exe
	- https://github.com/ninja-build/ninja/releases/download/v1.10.2/ninja-win.zip
	- Extract and place to proper location
	- set this location to PATH
	
# Important Note for ASIC
- default build change to ASIC 
- if want to use PXP or FPGA 
	- FPGA : `cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake -DBUILD_FPGA=ON -DBUILD_PXP=OFF`
	- FPGA : `cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake -DBUILD_FPGA=OFF -DBUILD_PXP=ON`
- if has an existing build folder
	- remove build folder and reconfig again by 
		-`cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake'
	- or `cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake -DBUILD_FPGA=OFF -DBUILD_PXP=OFF`

# Build Step (MinGW) #
- double click **msys2_shell.cmd** from mysys64 folder
- setup cmake program path to ~/.bashrc by 
	- vim ~/.bashrc and append path of cmake.exe to environment variable PATH
	- or use notepad.exe ~/.bashrc 
	- or other editor to edit ~/.bashrc
- change directory to project/realtek_amebapro2_v0_example/GCC-RELEASE/[target want to built]
- enter command in popuped terminal

1. `mkdir build`
2. `cd build`
3. `cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake -DBUILD_FPGA=ON -DBUILD_PXP=OFF`
	- if error relative to CMAKE_MAKE_PROGRAM, please add -DCMAKE_MAKE_PROGRAM=make to fix this issue
4. `cmake --build .`

# Build Step (CMD.exe) #	

- before starting, please make sure arm-none-eabi-gcc.exe can be found in system PATH. if not, please setup PATH to location of arm-none-eabi-gcc.exe
- change directory to project/realtek_amebapro2_v0_example/GCC-RELEASE/[target want to built]
- execute cmd.exe and enter commands listed below

1. `mkdir build`
2. `cd build`
3. `cmake .. -G"Ninja" -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake  -DBUILD_FPGA=ON -DBUILD_PXP=OFF`
4. `cmake --build .`

# Build Step (Cygwin)

- Not test, please try yourself
- If still want to use cygwin, please following next step
- install cmake by using cygwin installer program, and remove standalone version
- setup mingw style windows volume by using symbolic link from /cygdriver/[c,d,e,f,g] to /[c,d,e,f,g] 
	- that is because asdk toolchain only support windows and mingw style path, not cygwin style
- build project by following MinGW step

# Option for build setup #

- For Internal
	- Toolchain defination
		- Use -DCMAKE_TOOLCHAIN_FILE=[paht of toolchain.cmake]
		- toolchain.cmake path is related path and it will be different depend on project path
	- Build Libraries
		- Default is OFF
		- Add -DBUILD_LIB=ON on cmake command
			- **cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake -D BUILD_LIB=ON**
		- Turn OFF BUILD_LIB please use **-D BUILD_LIB=OFF** or **-U BUILD_LIB**
	- Build for PXP or FPGA
		- Add -DBUILD_PXP=ON or -DBUILD_FPGA=ON
		- Change to ASIC from PXP or FPGA, use -D BUILD_PXP=OFF or -D BUILD_FPGA=OFF
			- this step is necessary if want to change build config 
	- Build for ASIC
		- if this is first time configurate this project, build for ASIC is default setting
		- if this is configurated project
			- Run  `cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake -DBUILD_FPGA=OFF -DBUILD_PXP=OFF`
			- Or remove CMakeCache.txt and Run `cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake`

	- To check current build config
		- Run **cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake**
		- and check
		-  ```
			-- Build libraries OFF
			-- Build FPGA OFF
			-- Build PXP OFF
	
			```
	
	- Use ram model
		- only for FPGA or PXP simulation
		- in ram_model folder
		- ``` 
			mkdir build && cd build
			cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../../toolchain.cmake
			cmake --build .
		  ```
		  
	- Build NTZ flash binary
		- in GCC-RELEASE folder
		- create build folder and enter build folder just like before
		- Run **cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake**
		- Run **cmake --build . --target flash** to build and generate flash binary
		- If target is not DDR-built-in device
			- Run **cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DNODDR=ON**
			- only support NTZ project
		- If last build is for TZ, please re-run cmake to configurate BUILD_TZ=OFF
			- **cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DBUILD_TZ=OFF**
		
	- Build TZ flash binary
		- in GCC-RELEASE folder
		- create build folder and enter build folder just like before
		- Run **cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DBUILD_TZ=ON**
		- Run **cmake --build . --target flash** to build and generate flash binary 
		- If want to build NTZ binary after TZ build
		- 	please configurate project by running **cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DBUILD_TZ=OFF**
		
	- Build for test chip
		- currently, test chip is default setting 
		- if want to switch from other cut configuration, use `-DCUTVER=TEST` when run cmake configuration
			- for example, run `cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DCUTVER=TEST`
			- check configuration message `MPCHIP OFF : TEST-CUT` 
	
	- Build for MP (A/B/C) cut
		- if want to switch from other cut configuration, use `-DCUTVER=A` or `-DCUTVER=B` or `-DCUTVER=C` when run cmake configuration
			- for example, run `cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DCUTVER=B`
			- check configuration message `MPCHIP ON : B-CUT` 

	- Use GCC 10.3.0 with picolibc support
		- Install Realtek ASDK 10.3.0 and set `asdk-10.3.0/mingw32/picolibc/bin` to PATH
			- NOTE: make sure this path is set before other ASDK version
		- change toolchain.cmake to toolchain_picolibc.cmake and re-run configuration

# test MAC Loopback in ram_model
- Define CONFIG_MAC_LOOPBACK_DRIVER_RTL8735B of autoconf.h to 1
- Define RX_INDICATE of rtk_wlan_if.c to 0
- Enable '#LOOPBACK TEST' app_sources in project\realtek_amebapro2_v0_example\GCC-RELEASE\ram_model\CMakeLists.txt
- Use main_loopback.c instead of main.c

# test two MAC in ram_model
- enable console_init() and wlan_network() in main.c
- wlan_network.c does NOT wifi_on(RTW_MODE_STA);
- autoconf.h #define CONFIG_TWO_MAC_DRIVER
- platform_conf.h #define CONFIG_EXRAM_LPDDR_EN (0)
- heap_4_2.c #define USE_ERAM 0
- CMakeList.txt changes to use rtl8735b_twomac.ld
- freertos_intfs.c changes #if SUPPORT_5G_CHANNEL to #if 0
- freertos_intfs.c rtw_if1_init() changes mac_addr[5] after rtw_macaddr_cfg()
	rtw_macaddr_cfg(padapter->eeprompriv.mac_addr);
	if(mode == RTW_MODE_STA)
		padapter->eeprompriv.mac_addr[5] = 0x01;
	if(mode == RTW_MODE_AP)
		padapter->eeprompriv.mac_addr[5] = 0x02;
- start soft AP mode in FA, STA mode in FB

# enable LWIP+WLAN in ram_model
- enable wlan_network() in main.c

# test application with UVC in ram_model
- Enable '#UVCD AUTO TEST' app_sources in project\realtek_amebapro2_v0_example\GCC-RELEASE\ram_model\CMakeLists.txt
- Use main_uvc.c instead of main.c