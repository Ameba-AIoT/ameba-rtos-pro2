### add .cmkae need if neeeded ###
if(BUILD_LIB)
	message(STATUS "build audio save libraries")
endif()

ADD_LIBRARY (skynet_lib STATIC IMPORTED )
SET_PROPERTY ( TARGET skynet_lib PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/skynet/skynet_iot/GCC/libSkynetAPI_iot.a )

list(
    APPEND app_example_lib
	skynet_lib
)

### add flags ###
list(
	APPEND app_example_flags
	SAVE_AUDIO_DATA=1
)

### add header files ###
list (
    APPEND app_example_inc_path
    "${CMAKE_CURRENT_LIST_DIR}"
	${CMAKE_CURRENT_LIST_DIR}/skynet
)

### add source file ###
## add the file under the folder
#[[
set(EXAMPLE_SOURCE_PATH)
file(GLOB EXAMPLE_SOURCE_PATH ${CMAKE_CURRENT_LIST_DIR}/*.c)
list(
	APPEND app_example_sources
    
    ${EXAMPLE_SOURCE_PATH}
)
]]

## add source file individulay
#AUDIO TEST
list(
	APPEND app_example_sources
    
    #${EXAMPLE_SOURCE_PATH}
    app_example.c
    audio_tool_command.c
	module_null.c
	module_afft.c
	module_tone.c
	audio_tool_init.c
	pcm8K_music.c
	pcm16K_music.c
	audio_cjson_generater.c
	audio_http_server.c
	skynet/skynet_device.c
	skynet/module_p2p_audio.c
)	
#pcm8K_std1ktone.c
#pcm16K_std1ktone.c
list(TRANSFORM app_example_sources PREPEND ${CMAKE_CURRENT_LIST_DIR}/)

