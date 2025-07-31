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
    #OPE_EXPORT=extern
)

### add header files ###
list (
    APPEND app_example_inc_path
    "${CMAKE_CURRENT_LIST_DIR}"
)

### add source file ###
## add the file under the folder
set(EXAMPLE_SOURCE_PATH)
file(GLOB EXAMPLE_SOURCE_PATH ${CMAKE_CURRENT_LIST_DIR}/*.c)
list(
	APPEND app_example_sources
    
    ${EXAMPLE_SOURCE_PATH}
)


