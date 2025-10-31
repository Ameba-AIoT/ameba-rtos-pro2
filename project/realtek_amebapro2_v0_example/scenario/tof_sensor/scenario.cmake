cmake_minimum_required(VERSION 3.6)

enable_language(C CXX ASM)

list(
    APPEND app_sources
    ${sdk_root}/component/media/framework/tof_sensor/tof_sens_ctrl_api.c
	${sdk_root}/component/media/framework/tof_sensor/vl53l5cx_uld_driver/src/vl53l5cx_api.c
	${sdk_root}/component/media/framework/tof_sensor/vl53l5cx_uld_driver/platform/platform.c
    ${sdk_root}/component/media/framework/tof_sensor/vl53l5cx_uld_driver/platform/amebapro2/amebapro2_i2c_wrapper.c
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
)

list(
    APPEND scn_flags
)

list(
    APPEND scn_libs
)
