project(ompl_drrtsfn)
cmake_minimum_required(VERSION 2.8)

set(RRTFND_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
include_directories(${RRTFND_INCLUDE_DIR})

add_subdirectory(src)


