cmake_minimum_required(VERSION 3.8)
project(livox_avia_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

add_executable(custom_to_pointcloud2_node
  src/custom_to_pointcloud2_node.cpp
)
ament_target_dependencies(custom_to_pointcloud2_node
  rclcpp
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)
target_link_libraries(custom_to_pointcloud2_node "${cpp_typesupport_target}")

install(TARGETS custom_to_pointcloud2_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
