cmake_minimum_required(VERSION 2.8.3)
project(can_interface)

find_package(catkin)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_linuxcan
)

include_directories(include/${PROJECT_NAME}
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

install(TARGETS ros_linuxcan
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
