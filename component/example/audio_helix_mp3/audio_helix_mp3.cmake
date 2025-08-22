### add .cmkae need if neeeded ###
if(BUILD_LIB)
	message(STATUS "build MMF libraries")
endif()

list(
    APPEND app_example_lib
)

### add flags ###
list(
	APPEND app_example_flags
)

### add header files ###
list (
    APPEND app_example_inc_path
    "${CMAKE_CURRENT_LIST_DIR}"
    "${sdk_root}/component/audio/3rdparty/hmp3"
    "${sdk_root}/component/example"
)

### add source file ###
## add the file under the folder
list(
	APPEND app_example_sources
    
    example_audio_helix_mp3.c
    app_example.c
)
list(TRANSFORM app_example_sources PREPEND ${CMAKE_CURRENT_LIST_DIR}/)
