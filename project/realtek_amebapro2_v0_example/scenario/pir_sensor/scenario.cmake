cmake_minimum_required(VERSION 3.6)

enable_language(C CXX ASM)

#ENTRY for the project
list(
    APPEND scn_sources
    ${CMAKE_CURRENT_LIST_DIR}/src/main.c
)

list(
    APPEND scn_inc_path
    ${CMAKE_CURRENT_LIST_DIR}/src
)

list(
    APPEND scn_flags
)

list(
    APPEND scn_libs
)
