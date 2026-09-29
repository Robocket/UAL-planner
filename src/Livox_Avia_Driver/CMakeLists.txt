cmake_minimum_required(VERSION 3.0.2)
project(livox_avia_driver)

add_compile_options(-Wall -Wextra -Wpedantic)
find_package(catkin REQUIRED COMPONENTS
  message_generation roscpp sensor_msgs std_msgs)

add_message_files(FILES CustomPoint.msg CustomMsg.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs)

add_executable(custom_to_pointcloud2_node src/custom_to_pointcloud2_node.cpp)
target_compile_features(custom_to_pointcloud2_node PRIVATE cxx_std_17)
target_include_directories(custom_to_pointcloud2_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(custom_to_pointcloud2_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_to_pointcloud2_node ${catkin_LIBRARIES})
install(TARGETS custom_to_pointcloud2_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
