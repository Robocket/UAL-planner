cmake_minimum_required(VERSION 3.8)
project(ins_seg)

# sam3.cpp uses optimized kernels and selects Release when no build type is
# supplied. Set it before ament creates symlink-install scripts so every
# generated install fragment uses the same configuration suffix.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SegInfo.msg"
  "msg/SegmentationResult.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

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
  GIT_SUBMODULES ggml
  GIT_SUBMODULES_RECURSE TRUE
)
FetchContent_MakeAvailable(sam3cpp)

add_executable(seg_q4_node src/seg_q4_node.cpp)
target_compile_features(seg_q4_node PRIVATE cxx_std_17)
target_link_libraries(
  seg_q4_node sam3 CURL::libcurl OpenSSL::Crypto ${OpenCV_LIBRARIES})
target_include_directories(seg_q4_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(
  seg_q4_node
  cv_bridge
  diagnostic_msgs
  rclcpp
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(seg_q4_node "${cpp_typesupport_target}")

install(TARGETS seg_q4_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/compressed_image_adapter.py
  scripts/seg.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
