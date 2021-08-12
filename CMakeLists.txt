cmake_minimum_required(VERSION 3.5)
project(kvaser_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(LIB_NAME "ros_${PROJECT_NAME}")

# kvaser_interface Library
ament_auto_add_library(
  ${LIB_NAME} SHARED
  src/kvaser_interface.cpp
)
target_link_libraries(${LIB_NAME}
  canlib
)

# kvaser_reader ROS Node
ament_auto_add_library(
  kvaser_reader_node SHARED
  src/kvaser_reader_node.cpp
)
target_compile_definitions(
  kvaser_reader_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(
  kvaser_reader_node
  ${LIB_NAME}
)
rclcpp_components_register_nodes(
  kvaser_reader_node "kvaser_interface::KvaserReaderNode"
  EXECUTABLE kvaser_reader_node_exe
)

# kvaser_writer ROS Node
ament_auto_add_library(
  kvaser_writer_node SHARED
  src/kvaser_writer_node.cpp
)
target_compile_definitions(
  kvaser_writer_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(
  kvaser_writer_node
  ${LIB_NAME}
)
rclcpp_components_register_nodes(
  kvaser_writer_node "kvaser_interface::KvaserWriterNode"
  EXECUTABLE kvaser_writer_node_exe
)

# CAN Bridge Node
ament_auto_add_executable(
  kvaser_can_bridge
  src/kvaser_can_bridge.cpp
)
target_link_libraries(
  kvaser_can_bridge
  kvaser_reader_node
  kvaser_writer_node
)

# canmonitor Tool
ament_auto_add_executable(
  canmonitor
  tools/canmonitor.cpp
)
target_link_libraries(
  canmonitor
  ${LIB_NAME}
)

# list_channels Tool
ament_auto_add_executable(
  list_channels
  tools/list_channels.cpp
)
target_link_libraries(
  list_channels
  ${LIB_NAME}
)

if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Remove cxxopts.hpp from linter tests
  file(GLOB_RECURSE SOURCE_FILES
    "*.cpp"
    "*.hpp"
  )

  find_package(ament_cmake_cpplint)
  find_package(ament_cmake_uncrustify)

  get_filename_component(
    CXXOPTS_ABS
    "include/kvaser_interface/cxxopts.hpp"
    ABSOLUTE
  )

  list(REMOVE_ITEM SOURCE_FILES ${CXXOPTS_ABS})

  ament_cpplint(${SOURCE_FILES})
  ament_uncrustify(${SOURCE_FILES})

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
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
