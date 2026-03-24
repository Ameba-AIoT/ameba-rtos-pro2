# This cmake file is used to include ICE as static library.
set(CMAKE_ICE_DIRECTORY ${prj_root}/src/amazon_kvs/lib_amazon_v2/libraries/components/amazon-kinesis-video-streams-ice)

# ICE library source files.
file(
  GLOB
  ICE_SOURCES
  "${CMAKE_ICE_DIRECTORY}/source/*.c" )

# ICE library public include directories.
set( ICE_INCLUDE_PUBLIC_DIRS
     "${CMAKE_ICE_DIRECTORY}/source/include" )

add_library( ice
             ${ICE_SOURCES} )

target_include_directories( ice PUBLIC
                            ${ICE_INCLUDE_PUBLIC_DIRS} )

target_link_libraries( ice PRIVATE
                       stun )

### add linked library ###
list(
    APPEND app_example_lib
    ice
    stun
)
