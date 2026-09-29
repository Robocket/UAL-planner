cmake_minimum_required(VERSION 3.14)
project(ins_seg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  diagnostic_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  ual_ros1
)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)

add_message_files(FILES
  InstanceInfo.msg
  ProjectedInstanceInfo.msg
  SegInfo.msg
  SegmentationResult.msg)
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)
catkin_package(CATKIN_DEPENDS
  cv_bridge diagnostic_msgs geometry_msgs message_runtime nav_msgs roscpp
  sensor_msgs std_msgs ual_ros1)

include(FetchContent)
set(SAM3_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SAM3_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GGML_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GGML_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GGML_BUILD_SERVER OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  sam3cpp
  GIT_REPOSITORY https://github.com/PABannier/sam3.cpp.git
  GIT_TAG 416186c501d060df7ca02989d49b38080f5f81f3
  GIT_SHALLOW TRUE
  GIT_SUBMODULES ggml)
FetchContent_MakeAvailable(sam3cpp)

add_executable(seg_q4_node src/seg_q4_node.cpp)
target_compile_features(seg_q4_node PRIVATE cxx_std_17)
target_include_directories(seg_q4_node PRIVATE
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_dependencies(seg_q4_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(seg_q4_node
  sam3 CURL::libcurl ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/compressed_image_adapter.py
  scripts/graph.py
  scripts/projection.py
  scripts/seg.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS seg_q4_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "__pycache__" EXCLUDE)
