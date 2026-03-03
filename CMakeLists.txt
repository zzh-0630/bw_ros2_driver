cmake_minimum_required(VERSION 3.8)
project(bw_ros2_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_library(serial_port src/serial_port.cpp)
add_library(standard_parser src/standard_parser.cpp)
add_library(nova_parser src/nova_parser.cpp)
add_library(can_port src/can_port.cpp)

set_target_properties(serial_port standard_parser nova_parser can_port
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
)

add_executable(bw_node_standard src/bw_node_standard.cpp)
add_executable(bw_node_nova src/bw_node_nova.cpp)
add_executable(bw_node_can src/bw_node_can.cpp)

set_target_properties(bw_node_standard bw_node_nova bw_node_can
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
)

target_link_libraries(bw_node_standard
  standard_parser
  serial_port
  nova_parser
)

target_link_libraries(bw_node_nova
  serial_port
  nova_parser
)

target_link_libraries(bw_node_can
  can_port
)

ament_target_dependencies(bw_node_standard rclcpp sensor_msgs tf2)
ament_target_dependencies(bw_node_nova rclcpp sensor_msgs tf2)
ament_target_dependencies(bw_node_can rclcpp sensor_msgs tf2)

ament_export_include_directories(include)

install(TARGETS
  serial_port
  standard_parser
  nova_parser
  can_port
  bw_node_standard
  bw_node_nova
  bw_node_can
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config can assets
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
