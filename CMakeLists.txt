project(staubli_dart)

find_package(DART REQUIRED)
find_package(DARTCore REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Bullet REQUIRED)

include(FindBoost)

find_package(Boost ${MIN_BOOST_VERSTION} REQUIRED system filesystem regex thread chrono)

find_package(OMPL)
find_library(FLANN_LIBRARIES flann)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${BULLET_INCLUDE_DIR})

cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME}
    dart
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${FLANN_LIBRARIES}
    pthread
    )

