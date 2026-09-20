cmake_minimum_required(VERSION 3.8)
project(ual_planner_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../launch/"
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../rviz/"
  DESTINATION share/${PROJECT_NAME}/rviz
)

# The repository-root config directory is the single integrated source of truth.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../config/"
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
