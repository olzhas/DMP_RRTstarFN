project(staubli_dart)
cmake_minimum_required(VERSION 2.8)

find_package(DART REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_search_module(YAMLCPP REQUIRED yaml-cpp)

include(FindBoost)

find_package(Boost ${MIN_BOOST_VERSTION} REQUIRED system regex thread chrono serialization filesystem)

find_package(OMPL)
find_library(FLANN_LIBRARIES flann)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${BULLET_INCLUDE_DIR})

aux_source_directory(. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "./Point2DPlanning.cpp")
add_executable(${PROJECT_NAME} ${SRC_LIST})
add_executable(2d_demo Point2DPlanning.cpp)

target_link_libraries(${PROJECT_NAME}
    dart
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${YAMLCPP_LIBRARIES}
    pthread
    )

target_link_libraries(2d_demo
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${BULLET_LIBRARIES}
    )



