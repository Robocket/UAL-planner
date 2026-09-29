cmake_minimum_required(VERSION 3.0.2)
project(landing_evaluator)

add_compile_options(-Wall -Wextra -Wpedantic)
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  ual_ros1
  visualization_msgs
)

add_message_files(FILES HeightMap.msg LandingStatus.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES landing_evaluator_core
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs std_srvs ual_ros1)

add_library(landing_evaluator_core src/evaluator.cpp)
target_compile_features(landing_evaluator_core PUBLIC cxx_std_17)
target_include_directories(landing_evaluator_core PUBLIC include)

add_executable(landing_evaluator_node src/landing_evaluator_node.cpp)
target_compile_features(landing_evaluator_node PRIVATE cxx_std_17)
target_include_directories(landing_evaluator_node PRIVATE include ${catkin_INCLUDE_DIRS})
add_dependencies(landing_evaluator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(landing_evaluator_node landing_evaluator_core ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/avia_landing_test.py
  scripts/landing_rviz_visualizer.py
  scripts/odom_gravity_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_evaluator test/test_evaluator.cpp)
  if(TARGET test_evaluator)
    target_compile_features(test_evaluator PRIVATE cxx_std_17)
    target_include_directories(test_evaluator PRIVATE include)
    target_link_libraries(test_evaluator landing_evaluator_core gtest_main)
  endif()
  catkin_add_nosetests(test/test_odom_gravity_adapter.py)
endif()

install(TARGETS landing_evaluator_core landing_evaluator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE)
