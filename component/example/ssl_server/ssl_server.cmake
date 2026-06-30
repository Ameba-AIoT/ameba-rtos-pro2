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
)

### add source file ###
list(
	APPEND app_example_sources
	app_example.c
	example_ssl_server.c
)

if("${mbedtls}" MATCHES "^mbedtls-4")
list(
	APPEND app_example_sources
	../../ssl/${mbedtls}/tests/src/certs.c
)
list (
	APPEND inc_path_re
	${sdk_root}/component/ssl/${mbedtls}/tests/include
)
endif()

list(TRANSFORM app_example_sources PREPEND ${CMAKE_CURRENT_LIST_DIR}/)
