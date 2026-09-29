cmake_minimum_required(VERSION 3.0.2)
project(ual_planner_bringup)

find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
