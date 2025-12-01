cmake_minimum_required(VERSION 3.5)
project(bw_ros2_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(serial_port src/serial_port.cpp)
add_library(bws_parser src/bws_parser.cpp)
add_library(f3_parser src/f3_parser.cpp)

add_executable(bw_node_dmc src/bw_node_dmc.cpp)
add_executable(bw_node src/bw_node.cpp)
ament_target_dependencies(bw_node_dmc
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
ament_target_dependencies(bw_node
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(bw_node_dmc
  bws_parser
  serial_port
  f3_parser
)
target_link_libraries(bw_node
  bws_parser
  serial_port
  f3_parser
)

install(TARGETS
  serial_port
  bws_parser
  f3_parser
  bw_node_dmc
  bw_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
