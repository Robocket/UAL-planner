cmake_minimum_required(VERSION 3.8)
project(auto_landing)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LandingRegion.msg"
  "msg/LandingRegionArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

install(PROGRAMS
  scripts/evaluate.py
  scripts/track_landing_point.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
