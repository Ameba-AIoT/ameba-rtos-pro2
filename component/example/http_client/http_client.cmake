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
	${sdk_root}/component/network/httplite
	${sdk_root}/component/example
)

### add source file ###
list(
	APPEND app_example_sources
	app_example.c
	example_http_client.c
)
list(TRANSFORM app_example_sources PREPEND ${CMAKE_CURRENT_LIST_DIR}/)
