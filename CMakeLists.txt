project(staubli_dart)
cmake_minimum_required(VERSION 2.8)

find_package(DART REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(urdfdom REQUIRED)

pkg_search_module(YAMLCPP REQUIRED yaml-cpp)

include(FindBoost)

find_package(Boost ${MIN_BOOST_VERSTION} REQUIRED system serialization filesystem)

find_package(OMPL)
find_library(FLANN_LIBRARIES flann)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -mtune=native -g -Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${BULLET_INCLUDE_DIR})

set(RRTFND_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${RRTFND_INCLUDE_DIR})


aux_source_directory(. SRC_LIST)

list(REMOVE_ITEM SRC_LIST "./Point2DPlanning.cpp")
list(REMOVE_ITEM SRC_LIST "./Dubins2D.cpp")
list(REMOVE_ITEM SRC_LIST "./car_dart.cpp")
list(REMOVE_ITEM SRC_LIST "./simpledynamicsetuptest.cpp")

#add_executable(${PROJECT_NAME} ${SRC_LIST} ompl/geometric/planners/rrt/src/RRTstarFND.cpp ompl/base/spaces/src/WeightedRealVectorStateSpace.cpp)
#add_executable(2d_demo examples/Point2DPlanning.cpp ompl/geometric/planners/rrt/src/RRTstarFND.cpp)
#add_executable(2d_dubins examples/Dubins2D.cpp ompl/geometric/planners/rrt/src/RRTstarFND.cpp examples/model.cpp)
add_executable(testdynamic examples/simpledynamicsetuptest.cpp ompl/geometric/planners/rrt/src/RRTstarFND.cpp ompl/geometric/src/DynamicSimpleSetup.cpp examples/model.cpp ompl/base/src/DynamicPlanner.cpp)

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

target_link_libraries(testdynamic
    dart
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${BULLET_LIBRARIES}
    pthread
    )
