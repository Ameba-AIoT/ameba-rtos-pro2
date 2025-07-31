### add lib ###
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
	"${prj_root}/src/mmfv2_video_example"
	"${prj_root}/src/fast_inf_example"
)

set(EXAMPLE_SOURCE_PATH)
file(GLOB EXAMPLE_SOURCE_PATH ${CMAKE_CURRENT_LIST_DIR}/*.c)

### add source file ###
list(
	APPEND app_example_sources
    ${EXAMPLE_SOURCE_PATH}
	${sdk_root}/component/video/osd2/osd_render.c
)


