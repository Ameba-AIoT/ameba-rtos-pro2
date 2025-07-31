cmake_minimum_required(VERSION 3.6)

enable_language(C CXX ASM)

ADD_LIBRARY (skynet_lib STATIC IMPORTED )
SET_PROPERTY ( TARGET skynet_lib PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/src/skynet_iot/GCC/libSkynetAPI_iot.a )

#MMF_MODULE
list(
    APPEND app_sources
    ${sdk_root}/component/media/mmfv2/module_video.c
    ${sdk_root}/component/media/mmfv2/module_rtsp2.c
    ${sdk_root}/component/media/mmfv2/module_audio.c
    ${sdk_root}/component/media/mmfv2/module_aac.c
    ${sdk_root}/component/media/mmfv2/module_g711.c
    ${sdk_root}/component/media/mmfv2/module_rtp.c
)

#USER
list(
    APPEND scn_sources
    ${CMAKE_CURRENT_LIST_DIR}/src/main.c
    ${CMAKE_CURRENT_LIST_DIR}/src/skynet_device.c
    ${CMAKE_CURRENT_LIST_DIR}/src/module_p2p.c
    ${CMAKE_CURRENT_LIST_DIR}/src/module_p2p_aac.c
    ${CMAKE_CURRENT_LIST_DIR}/src/doorbell_chime.c
)

#ENTRY for the project
list(
    APPEND scn_sources
    ${CMAKE_CURRENT_LIST_DIR}/src/main.c
)

list(
    APPEND scn_inc_path
    ${CMAKE_CURRENT_LIST_DIR}/src
)

list(
    APPEND scn_flags
)

list(
    APPEND scn_libs
    skynet_lib
)


