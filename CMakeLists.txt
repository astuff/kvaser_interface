cmake_minimum_required(VERSION 2.8.3)
project(kvaser_interface)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  can_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp can_msgs
  INCLUDE_DIRS include
  LIBRARIES ros_linuxcan
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(ros_linuxcan
  src/linuxcan.cpp
  src/utils.cpp
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

install(TARGETS ros_linuxcan
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS kvaser_can_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
