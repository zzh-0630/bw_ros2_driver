cmake_minimum_required(VERSION 3.8)
project(bw_ros2_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BewisInspva.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

include_directories(include)

add_library(serial_port src/serial_port.cpp)
add_library(standard_parser src/standard_parser.cpp)
add_library(nova_parser src/nova_parser.cpp)
add_library(can_port src/can_port.cpp)
add_library(inspva_parser src/inspva_parser.cpp)

set_target_properties(serial_port standard_parser nova_parser can_port inspva_parser
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
)

add_executable(bw_node_standard src/bw_node_standard.cpp)
add_executable(bw_node_nova src/bw_node_nova.cpp)
add_executable(bw_node_can src/bw_node_can.cpp)
add_executable(bw_node_gixxx src/bw_node_gixxx.cpp)

set_target_properties(bw_node_standard bw_node_nova bw_node_can bw_node_gixxx
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

target_link_libraries(bw_node_gixxx
  serial_port
  inspva_parser
)


ament_target_dependencies(bw_node_standard rclcpp sensor_msgs tf2)
ament_target_dependencies(bw_node_nova rclcpp sensor_msgs tf2)
ament_target_dependencies(bw_node_can rclcpp sensor_msgs tf2)
ament_target_dependencies(bw_node_gixxx geometry_msgs rclcpp sensor_msgs std_msgs tf2)

rosidl_target_interfaces(bw_node_gixxx
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

install(TARGETS
  serial_port
  standard_parser
  nova_parser
  inspva_parser
  can_port
  bw_node_standard
  bw_node_nova
  bw_node_gixxx
  bw_node_can
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config can msg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
