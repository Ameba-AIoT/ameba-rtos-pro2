cmake_minimum_required(VERSION 3.6)

# CMakeLists.txt
option(USE_ATCMD_MQTT "Enable AT command over MQTT" ON)  # Switch Options

if(USE_ATCMD_MQTT)
    add_definitions(-DUSE_ATCMD_MQTT)      # Define flag
    list(APPEND app_sources                 
		${sdk_root}/component/soc/8735b/misc/driver/mqtt/atcmd_mqtt.c
    )
	message(STATUS "Remote AT command over MQTT enabled")
else()
	list(APPEND app_sources                
		${sdk_root}/component/soc/8735b/misc/driver/telnetd/telnetd.c
	)
	message(STATUS "Remote AT command over Telnet enabled")
endif()

list(
	APPEND app_inc_path
)

list(
	APPEND app_flags
)