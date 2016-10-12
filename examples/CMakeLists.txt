

add_executable(testdynamic simpledynamicsetuptest.cpp model.cpp)

target_link_libraries(testdynamic
    dart
    ompl_drrtsfn
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )

