cmake_minimum_required(VERSION 3.8)
project(pointcloud_voxels)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/mask_voxels_node
  scripts/path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Sources live in the corresponding repository-root directories. Installing
# them with the algorithm package makes the same launches usable via ros2 launch.
foreach(asset_dir launch config rviz)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../${asset_dir}/"
    DESTINATION share/${PROJECT_NAME}/${asset_dir}
    PATTERN "__pycache__" EXCLUDE
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_voxel_core ../../tests/test_pointcloud_voxels.py)
  ament_add_pytest_test(test_detector_core ../../tests/test_detector_core.py)
  ament_add_pytest_test(test_voxel_transport ../../tests/test_voxel_transport.py)
  ament_add_pytest_test(test_planning ../../tests/test_planning.py)
  ament_add_pytest_test(test_pointcloud_bbox ../../tests/test_pointcloud_bbox.py)
  ament_add_pytest_test(test_voxel_ros ../../tests/test_ros_voxel_node.py
    TIMEOUT 90
    ENV UAL_RUN_ROS_TESTS=1 ROS_DOMAIN_ID=74 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
  ament_add_pytest_test(test_ros_path_planner ../../tests/test_ros_path_planner.py
    TIMEOUT 90
    ENV UAL_RUN_ROS_TESTS=1 ROS_DOMAIN_ID=75 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
endif()

ament_package()
