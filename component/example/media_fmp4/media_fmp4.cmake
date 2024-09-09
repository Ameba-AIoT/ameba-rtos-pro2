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
	"${sdk_root}/component/media/3rdparty/fmp4/libmov/include"
	"${sdk_root}/component/media/3rdparty/fmp4/libflv/include"
)

### add source file ###
list(
	APPEND app_example_sources
	${CMAKE_CURRENT_LIST_DIR}/app_example.c
	${CMAKE_CURRENT_LIST_DIR}/example_media_fmp4.c
)
