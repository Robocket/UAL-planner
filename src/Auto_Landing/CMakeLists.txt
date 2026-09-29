cmake_minimum_required(VERSION 3.0.2)
project(auto_landing)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  ual_ros1
  visualization_msgs
)

add_message_files(FILES LandingRegion.msg LandingRegionArray.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS
  geometry_msgs message_runtime sensor_msgs std_msgs ual_ros1 visualization_msgs)

catkin_install_python(PROGRAMS
  scripts/evaluate.py
  scripts/track_landing_point.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "__pycache__" EXCLUDE)
