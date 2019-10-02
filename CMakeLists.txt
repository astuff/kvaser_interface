cmake_minimum_required(VERSION 3.5)
project(kvaser_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(can_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${can_msgs_INCLUDE_DIRS}
)

link_directories(
  ${rclcpp_LIB_DIRS}
)

set(node_plugins "")

set(LIB_NAME "ros_${PROJECT_NAME}")

add_library(
  ${LIB_NAME} SHARED
  src/kvaser_interface.cpp
)

target_compile_definitions(
  ${LIB_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

target_link_libraries(${LIB_NAME}
  canlib
)

add_executable(
  kvaser_can_bridge
  src/kvaser_can_bridge.cpp
)

ament_target_dependencies(
  kvaser_can_bridge
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "can_msgs"
)

target_link_libraries(
  kvaser_can_bridge
  ${LIB_NAME}
)

add_executable(
  canmonitor
  tools/canmonitor.cpp
)

ament_target_dependencies(
  canmonitor
  "rclcpp"
  "can_msgs"
)

target_link_libraries(
  canmonitor
  ${LIB_NAME}
)

add_executable(
  list_channels
  tools/list_channels.cpp
)

ament_target_dependencies(
  list_channels
  "rclcpp"
  "can_msgs"
)

target_link_libraries(
  list_channels
  ${LIB_NAME}
)

install(
  TARGETS
  ${LIB_NAME}
  kvaser_can_bridge
  canmonitor
  list_channels
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    ${PROJECT_NAME}_tests
    tests/${PROJECT_NAME}_tests.cpp
  )
  target_link_libraries(
    ${PROJECT_NAME}_tests
    ${LIB_NAME}
  )

#  TODO(jwhitleyastuff): Use launch_testing for tests.
#  find_package(catkin REQUIRED COMPONENTS rostest)
#  add_rostest(tests/reader_writer_1000hz.test)

#add_rostest_gtest(${PROJECT_NAME}_ros_tests
#    tests/can_bridge_tests.test
#    tests/${PROJECT_NAME}_ros_tests.cpp)
#  target_link_libraries(${PROJECT_NAME}_ros_tests
#    ros_${PROJECT_NAME}
#    ${catkin_LIBRARIES})

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
