### include .cmake need if neeeded ###
include(${sdk_root}/component/example/sqlite/libsqlite.cmake)

### add lib ###
list(
	APPEND app_example_lib
	sqlite
)

### add flags ###
list(
	APPEND app_example_flags
)

### add header files ###
list (
	APPEND app_example_inc_path
	"${sdk_root}/component/application/sqlite/sqlite_3.40.0"
	"${sdk_root}/component/application/sqlite/sqlite_port_8735b"
)

### add source file ###
list(
	APPEND app_example_sources
	${sdk_root}/component/example/sqlite/app_example.c
	${sdk_root}/component/example/sqlite/example_sqlite.c
)

