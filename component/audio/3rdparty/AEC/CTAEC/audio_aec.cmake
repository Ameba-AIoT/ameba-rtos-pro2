### add .cmkae need if neeeded ###
message(STATUS "build pc AEC libraries")

#include(${sdk_root}/component/audio/3rdparty/AEC/libaec.cmake)
#ADD_LIBRARY (testaec STATIC IMPORTED )
#SET_PROPERTY ( TARGET testaec PROPERTY IMPORTED_LOCATION ${sdk_root}/component/audio/3rdparty/AEC/test_aec.a )

list(
    APPEND aec_lib
	#testaec
	${sdk_root}/component/audio/3rdparty/AEC/CTAEC/libVQE.a
)

### add flags ###
list(
	APPEND aec_flags
)

### add header files ###
list (
    APPEND aec_inc_path
    "${CMAKE_CURRENT_LIST_DIR}"
)

### add source file ###
## add the file under the folder
list(
	APPEND aec_sources
	AEC_CT.c
    
)

list(TRANSFORM aec_sources PREPEND ${CMAKE_CURRENT_LIST_DIR}/../)
