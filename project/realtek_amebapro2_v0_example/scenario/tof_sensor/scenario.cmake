cmake_minimum_required(VERSION 3.6)

enable_language(C CXX ASM)

list(
    APPEND app_sources
    ${sdk_root}/component/media/framework/tof_sensor/tof_sens_ctrl_api.c
	${sdk_root}/component/media/framework/tof_sensor/vl53l5cx_uld_driver/src/vl53l5cx_api.c
	${sdk_root}/component/media/framework/tof_sensor/vl53l5cx_uld_driver/platform/platform.c
    ${sdk_root}/component/media/framework/tof_sensor/vl53l5cx_uld_driver/platform/amebapro2/amebapro2_i2c_wrapper.c
)

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
    ${CMAKE_CURRENT_LIST_DIR}/src/tof_sens_example_dist_array_init.c
    ${CMAKE_CURRENT_LIST_DIR}/src/tof_sens_example_osd_init.c
    ${sdk_root}/component/video/osd2/osd_render.c
)

#ENTRY for the project
list(
    APPEND scn_sources
    ${CMAKE_CURRENT_LIST_DIR}/src/main.c
)

list(
    APPEND scn_inc_path
    ${CMAKE_CURRENT_LIST_DIR}/src
	${sdk_root}/component/media/framework/tof_sensor
	${sdk_root}/component/media/framework/tof_sensor/vl53l5cx_uld_driver/inc
	${sdk_root}/component/media/framework/tof_sensor/vl53l5cx_uld_driver/platform
    ${sdk_root}/component/media/framework/tof_sensor/vl53l5cx_uld_driver/platform/amebapro2
	${sdk_root}/component/media/framework/tof_sensor/vl53l5cx_uld_driver/src
    ${sdk_root}/project/realtek_amebapro2_v0_example/scenario/tof_sensor/src
)

list(
    APPEND scn_flags
)

list(
    APPEND scn_libs
)
