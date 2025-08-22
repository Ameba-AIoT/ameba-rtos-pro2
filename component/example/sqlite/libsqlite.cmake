cmake_minimum_required(VERSION 3.6)

project(sqlite)

set(sqlite sqlite)

list(
	APPEND sqlite_sources
# lib src
	${sdk_root}/component/application/sqlite/sqlite_3.40.0/sqlite3.c
# port
	${sdk_root}/component/application/sqlite/sqlite_port_8735b/ameba_sqlite_mutex.c
	${sdk_root}/component/application/sqlite/sqlite_port_8735b/ameba_sqlite_io_methods.c
	${sdk_root}/component/application/sqlite/sqlite_port_8735b/ameba_sqlite_vfs.c
)

add_library(
	${sqlite} STATIC
	${sqlite_sources}
)

list(
	APPEND sqlite_flags
	CONFIG_BUILD_RAM=1 
	CONFIG_BUILD_LIB=1 
	CONFIG_PLATFORM_8735B
	CONFIG_RTL8735B_PLATFORM=1
	CONFIG_SYSTEM_TIME64=1

# SQLITE configuration
	SQLITE_OMIT_LOAD_EXTENSION=1
	SQLITE_OMIT_WAL=1
	SQLITE_OMIT_AUTOINIT=1
	SQLITE_OMIT_SHARED_CACHE=1
	SQLITE_OMIT_DEPRECATED=1
	SQLITE_THREADSAFE=1
	NDEBUG
	# SQLITE_DEBUG
	SQLITE_OS_OTHER=1
	SQLITE_OS_FREERTOS=1
	SQLITE_MUTEX_FREERTOS=1
)

target_compile_definitions(${sqlite} PRIVATE ${sqlite_flags} )
target_compile_options(${sqlite} PRIVATE ${LIBS_WARN_ERR_FLAGS} )

target_include_directories(
	${sqlite}
	PUBLIC

	${inc_path}
	${sdk_root}/component/os/freertos/${freertos}/Source/portable/GCC/ARM_CM33_NTZ/non_secure

	${sdk_root}/component/application/sqlite/sqlite_3.40.0
)