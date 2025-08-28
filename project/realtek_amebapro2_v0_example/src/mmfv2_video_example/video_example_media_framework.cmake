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
	UNITEST_VIDEO_EXAMPLE
	VIDEO_EXAMPLE_ON
)

### add header files ###
list (
    APPEND app_example_inc_path
    "${CMAKE_CURRENT_LIST_DIR}"
)


# remosaic library for mmf2_video_example_v1_snapshot_hr_init
ADD_LIBRARY (librtsremosaic STATIC IMPORTED )
SET_PROPERTY ( TARGET librtsremosaic PROPERTY IMPORTED_LOCATION ${sdk_root}/component/video/driver/RTL8735B/librtsremosaic.a )

list(
	APPEND libs
	librtsremosaic
)

set(EXAMPLE_SOURCE_PATH)
file(GLOB EXAMPLE_SOURCE_PATH ${CMAKE_CURRENT_LIST_DIR}/*.c)
#message(STATUS "${EXAMPLE_SOURCE_PATH}")

### add source file ###
#VIDEO TEST
list(
	APPEND app_example_sources
    
    ${EXAMPLE_SOURCE_PATH}
	${sdk_root}/component/video/osd2/isp_osd_example.c
	${sdk_root}/component/video/osd2/osd_render.c
)


