cmake_minimum_required(VERSION 3.0.2)
project(ins_seg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  sensor_msgs
  std_msgs
  nav_msgs
  message_generation
  geometry_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
  tf2_ros
  tf2_sensor_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   ObjectDetection.msg
#   ObjectDetections.msg
#   ObjectCoordinate.msg
#   ObjectCoordinates.msg
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy
    cv_bridge 
    sensor_msgs 
    std_msgs 
    message_runtime
    geometry_msgs
    visualization_msgs
    tf2_ros
    tf2_sensor_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/projection.py
  scripts/seg.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

