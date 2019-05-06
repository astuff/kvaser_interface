cmake_minimum_required(VERSION 2.8.3)
project(kvaser_interface)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  can_msgs
  roslint
)

catkin_package(
  CATKIN_DEPENDS roscpp can_msgs
  INCLUDE_DIRS include
  LIBRARIES ros_${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ros_${PROJECT_NAME}
  src/kvaser_interface.cpp
)

target_link_libraries(ros_${PROJECT_NAME}
  canlib
  ${catkin_LIBRARIES}
)

add_executable(kvaser_can_bridge
  src/kvaser_can_bridge.cpp
)

target_link_libraries(kvaser_can_bridge
  ros_${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(canmonitor
  tools/canmonitor.cpp
)

target_link_libraries(canmonitor
  ros_${PROJECT_NAME}
)

add_executable(list_channels
  tools/list_channels.cpp
)

target_link_libraries(list_channels
  ros_${PROJECT_NAME}
)

set(ROSLINT_CPP_OPTS "--filter=-runtime/threadsafe_fn,-build/namespaces,-build/c++11")
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()

  find_package(catkin REQUIRED COMPONENTS rosunit)
  catkin_add_gtest(${PROJECT_NAME}_tests
    tests/${PROJECT_NAME}_tests.cpp)
  target_link_libraries(${PROJECT_NAME}_tests
    ros_${PROJECT_NAME}
    ${catkin_LIBRARIES})

  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(tests/reader_writer_1000hz.test)

  add_rostest_gtest(${PROJECT_NAME}_ros_tests
    tests/can_bridge_tests.test
    tests/${PROJECT_NAME}_ros_tests.cpp)
  target_link_libraries(${PROJECT_NAME}_ros_tests
    ros_${PROJECT_NAME}
    ${catkin_LIBRARIES})
endif()

install(
  TARGETS
  ros_${PROJECT_NAME}
  kvaser_can_bridge
  canmonitor
  list_channels
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
