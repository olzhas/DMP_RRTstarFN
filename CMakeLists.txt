project(staubli_dart)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -mtune=native -g -Og")

find_package(Threads REQUIRED)
find_package(DART REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(urdfdom REQUIRED)

pkg_search_module(YAMLCPP REQUIRED yaml-cpp)

find_package(Boost ${MIN_BOOST_VERSTION} REQUIRED system serialization filesystem)
find_package(OMPL REQUIRED)
find_library(FLANN_LIBRARIES flann)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${BULLET_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OMPL_INCLUDE_DIRS})

set(RRTFND_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ompl/src/")
include_directories(${RRTFND_INCLUDE_DIR})

add_subdirectory(ompl)
add_subdirectory(examples)
add_subdirectory(manipulator)

aux_source_directory(. SRC_LIST)

list(REMOVE_ITEM SRC_LIST "./Point2DPlanning.cpp")
list(REMOVE_ITEM SRC_LIST "./Dubins2D.cpp")
list(REMOVE_ITEM SRC_LIST "./car_dart.cpp")
list(REMOVE_ITEM SRC_LIST "./simpledynamicsetuptest.cpp")

FILE(GLOB_RECURSE LibFiles "*.hpp" "*.h")
add_custom_target(headers SOURCES ${LibFiles})

#add_executable(${PROJECT_NAME} ${SRC_LIST} ompl/geometric/planners/rrt/src/RRTstarFND.cpp ompl/base/spaces/src/WeightedRealVectorStateSpace.cpp)
#add_executable(2d_demo examples/Point2DPlanning.cpp ompl/geometric/planners/rrt/src/RRTstarFND.cpp)
#add_executable(2d_dubins examples/Dubins2D.cpp ompl/geometric/planners/rrt/src/RRTstarFND.cpp examples/model.cpp)


#target_link_libraries(${PROJECT_NAME}
#    dart
#    ${Boost_LIBRARIES}
#    ${OMPL_LIBRARIES}
#    ${FLANN_LIBRARIES}
#    ${BULLET_LIBRARIES}
#    ${YAMLCPP_LIBRARIES}
#    pthread
#    )

#target_link_libraries(2d_demo
#    dart
#    ${Boost_LIBRARIES}
#    ${OMPL_LIBRARIES}
#    ${FLANN_LIBRARIES}
#    ${BULLET_LIBRARIES}
#    pthread
#    )

#target_link_libraries(2d_dubins
#    dart
#    ${Boost_LIBRARIES}
#    ${OMPL_LIBRARIES}
#    ${FLANN_LIBRARIES}
#    ${BULLET_LIBRARIES}
#    pthread
#    )

