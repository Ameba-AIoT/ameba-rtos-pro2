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
	"${sdk_root}/project/realtek_amebapro2_v0_example/src/mmfv2_video_example"
)

### add source file ###
list(
	APPEND app_example_sources
	app_example.c
	example_qr_code_scanner.c
)
list(TRANSFORM app_example_sources PREPEND ${CMAKE_CURRENT_LIST_DIR}/)


