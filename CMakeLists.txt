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
find_package(lifecycle_msgs REQUIRED)

include_directories(
  include
)

set(LIB_NAME "ros_${PROJECT_NAME}")

# kvaser_interface Library
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

# kvaser_reader ROS Node
add_library(
  kvaser_reader_node SHARED
  src/kvaser_reader_node.cpp
)
ament_target_dependencies(
  kvaser_reader_node
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "can_msgs"
)
target_link_libraries(
  kvaser_reader_node
  ${LIB_NAME}
)

# kvaser_writer ROS Node
add_library(
  kvaser_writer_node SHARED
  src/kvaser_writer_node.cpp
)
ament_target_dependencies(
  kvaser_writer_node
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "can_msgs"
  "lifecycle_msgs"
)
target_link_libraries(
  kvaser_writer_node
  ${LIB_NAME}
)

rclcpp_components_register_nodes(
  kvaser_reader_node
  "AS::CAN::KvaserReaderNode"
)

rclcpp_components_register_nodes(
  kvaser_writer_node
  "AS::CAN::KvaserWriterNode"
)

# CAN Bridge Node
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
  kvaser_reader_node
  kvaser_writer_node
)

# canmonitor Tool
add_executable(
  canmonitor
  tools/canmonitor.cpp
)
target_link_libraries(
  canmonitor
  ${LIB_NAME}
)

# list_channels Tool
add_executable(
  list_channels
  tools/list_channels.cpp
)
target_link_libraries(
  list_channels
  ${LIB_NAME}
)

ament_export_include_directories(include)
ament_export_interfaces(
  export_${LIB_NAME} HAS_LIBRARY_TARGET
  export_kvaser_reader_node HAS_LIBRARY_TARGET
  export_kvaser_writer_node HAS_LIBRARY_TARGET
)
ament_export_libraries(
  ${LIB_NAME}
  kvaser_reader_node
  kvaser_writer_node
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

ament_package()

install(
  TARGETS ${LIB_NAME}
  EXPORT export_${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS kvaser_reader_node
  EXPORT export_kvaser_reader_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS kvaser_writer_node
  EXPORT export_kvaser_writer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
  kvaser_can_bridge
  canmonitor
  list_channels
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
