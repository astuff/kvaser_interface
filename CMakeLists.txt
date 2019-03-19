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
  LIBRARIES ros_linuxcan
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ros_linuxcan
  src/linuxcan.cpp
)

target_link_libraries(ros_linuxcan
  canlib
  ${catkin_LIBRARIES}
)

add_executable(kvaser_can_bridge
  src/kvaser_can_bridge.cpp
)

target_link_libraries(kvaser_can_bridge
  ros_linuxcan
  ${catkin_LIBRARIES}
)

add_executable(canmonitor
  tools/canmonitor.cpp
)

target_link_libraries(canmonitor
  ros_linuxcan
)

add_executable(list_channels
  tools/list_channels.cpp
)

target_link_libraries(list_channels
  ros_linuxcan
)

set(ROSLINT_CPP_OPTS "--filter=-runtime/threadsafe_fn,-build/namespaces,-build/c++11")
roslint_cpp(
  include/kvaser_interface/kvaser_interface.h
  src/linuxcan.cpp
  src/kvaser_can_bridge.cpp
  tools/canmonitor.cpp
  tools/list_channels.cpp
)

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()

install(
  TARGETS
  ros_linuxcan
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
