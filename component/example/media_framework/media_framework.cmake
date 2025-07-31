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
	${sdk_root}/component/media/samples
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
    example_media_framework.c
	mmf2_example_a_init.c
	mmf2_example_aacloop_init.c
	mmf2_example_audioloop_init.c
	mmf2_example_g711loop_init.c
	mmf2_example_2way_audio_init.c
	mmf2_example_aac_array_rtsp_init.c
	mmf2_example_opus_array_rtsp_init.c
	mmf2_example_pcmu_array_rtsp_init.c
	mmf2_example_rtp_aad_init.c
    mmf2_example_i2s_audio_init.c
    mmf2_example_2way_audio_opus_init.c
    mmf2_example_a_opus_init.c
    mmf2_example_opusloop_init.c
    mmf2_example_rtp_opusd_init.c
	mmf2_example_pcm_array_audio_init.c
	mmf2_example_2way_audio_g711_doorbell_init.c
	
)
list(TRANSFORM app_example_sources PREPEND ${CMAKE_CURRENT_LIST_DIR}/)

