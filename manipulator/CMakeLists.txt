project(staubli)
cmake_minimum_required(VERSION 2.8)

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME}
    dart
    ompl_drrtsfn
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${YAMLCPP_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )

