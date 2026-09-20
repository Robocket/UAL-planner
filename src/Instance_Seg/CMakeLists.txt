cmake_minimum_required(VERSION 3.8)
project(ins_seg)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InstanceInfo.msg"
  "msg/ProjectedInstanceInfo.msg"
  "msg/SegInfo.msg"
  "msg/SegmentationResult.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

install(PROGRAMS
  scripts/graph.py
  scripts/projection.py
  scripts/seg.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
