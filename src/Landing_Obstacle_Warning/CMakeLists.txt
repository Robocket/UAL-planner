cmake_minimum_required(VERSION 3.8)
project(landing_evaluator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LandingStatus.msg"
  "msg/HeightMap.msg"
  DEPENDENCIES std_msgs
)

add_library(landing_evaluator_core src/evaluator.cpp)
target_include_directories(landing_evaluator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(landing_evaluator_node src/landing_evaluator_node.cpp)
target_include_directories(landing_evaluator_node PRIVATE include)
ament_target_dependencies(landing_evaluator_node rclcpp sensor_msgs std_msgs std_srvs)
target_link_libraries(landing_evaluator_node landing_evaluator_core)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(landing_evaluator_node "${cpp_typesupport_target}")

install(TARGETS landing_evaluator_core landing_evaluator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/avia_landing_test.py
  scripts/landing_rviz_visualizer.py
  scripts/odom_gravity_adapter.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_evaluator test/test_evaluator.cpp)
  target_link_libraries(test_evaluator landing_evaluator_core)
  target_include_directories(test_evaluator PRIVATE include)
  ament_add_pytest_test(test_odom_gravity_adapter
    test/test_odom_gravity_adapter.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(landing_evaluator_core)
ament_package()
