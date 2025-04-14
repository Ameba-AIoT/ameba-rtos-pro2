cmake_minimum_required(VERSION 3.6)

enable_language(C CXX ASM)

#Find the uart command library
set(UARTCMD_CMAKE_PATH "${CMAKE_CURRENT_LIST_DIR}/src/common_basics/libcommbasics.cmake")

#message(STATUS "${UARTCMD_CMAKE_PATH}")
if(EXISTS "${UARTCMD_CMAKE_PATH}")
    include("${UARTCMD_CMAKE_PATH}")
    message(STATUS "Using libcommonbasics.cmake")

else()
    message(STATUS "Using libcommonbasics.a")

    set(LIBUARTCMD_PATH "${CMAKE_CURRENT_LIST_DIR}/src/output/libcommonbasics.a")

    add_library(commonbasics STATIC IMPORTED)
    set_target_properties(commonbasics PROPERTIES IMPORTED_LOCATION "${LIBUARTCMD_PATH}")

endif()

list(
    APPEND scn_sources
    ${sdk_root}/component/media/mmfv2/module_video.c
    ${sdk_root}/component/media/mmfv2/module_audio.c
    ${sdk_root}/component/media/mmfv2/module_aac.c
    ${sdk_root}/component/media/mmfv2/module_i2s.c
    ${sdk_root}/component/media/mmfv2/module_mp4.c
    ${sdk_root}/component/media/mmfv2/module_filesaver.c
)

list(
    APPEND scn_sources
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/gyrosensor_api.c

    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/mpu6050/src/driver_mpu6050.c
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/mpu6050/interface/driver_mpu6050_interface_template.c
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/mpu6050/example/driver_mpu6050_basic.c
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/mpu6050/example/driver_mpu6050_fifo.c

    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/icm42670p/src/imu/inv_imu_driver.c
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/icm42670p/src/imu/inv_imu_transport.c
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/icm42670p/src/inv_time.c
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/icm42670p/src/icm42670p_hal.c
)

#USER
list(
    APPEND scn_sources
    ${CMAKE_CURRENT_LIST_DIR}/src/ai_glass_initialize.c
    ${CMAKE_CURRENT_LIST_DIR}/src/ai_glass_media_params.c
    ${CMAKE_CURRENT_LIST_DIR}/src/ai_snapshot_initialize.c
    ${CMAKE_CURRENT_LIST_DIR}/src/lifetime_recording_initialize.c
    ${CMAKE_CURRENT_LIST_DIR}/src/lifetime_snapshot_initialize.c
    ${CMAKE_CURRENT_LIST_DIR}/src/media_filesystem.c
    ${CMAKE_CURRENT_LIST_DIR}/src/nv12tojpg.c
    ${CMAKE_CURRENT_LIST_DIR}/src/wlan_scenario.c
)

#ENTRY for the project
list(
    APPEND scn_sources
    ${CMAKE_CURRENT_LIST_DIR}/src/main.c
)

list(
    APPEND scn_inc_path
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/mpu6050/src
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/mpu6050/interface
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/mpu6050/example
    ${CMAKE_CURRENT_LIST_DIR}/src/gyrosensor/icm42670p/src
    ${CMAKE_CURRENT_LIST_DIR}/src/common_basics
)

list(
    APPEND scn_flags
)

list(
    APPEND scn_libs
    commonbasics
)

list(
    APPEND _wrapper
    "-Wl,-wrap,get_fattime"
)

list(JOIN _wrapper " " function_wrapper)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${function_wrapper}" CACHE INTERNAL "")


