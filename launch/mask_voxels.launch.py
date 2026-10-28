#!/usr/bin/env python3
"""Select LiDAR points with camera detections/masks and visualize voxels."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('pointcloud_voxels')
    common_config = LaunchConfiguration('common_config')
    sensor_config = LaunchConfiguration('sensor_config')
    voxel_config = LaunchConfiguration('voxel_config')
    detector_config = LaunchConfiguration('detector_config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'common_config',
            default_value=os.path.join(share, 'config', 'common.yaml'),
            description='Instance-segmentation algorithm parameters',
        ),
        DeclareLaunchArgument(
            'sensor_config',
            default_value=os.path.join(share, 'config', 'scene_water.yaml'),
            description='Sensor topics, calibration and synchronization',
        ),
        DeclareLaunchArgument(
            'voxel_config',
            default_value=os.path.join(share, 'config', 'pointcloud_voxels.yaml'),
            description='Point selection and 3D voxel classification',
        ),
        DeclareLaunchArgument(
            'detector_config',
            default_value=os.path.join(share, 'config', 'detector.yaml'),
            description='YOLO OBB/detection model and topic parameters',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'ship_voxels.rviz'),
        ),
        DeclareLaunchArgument(
            'selection_mode', default_value='detection',
            choices=['detection', 'mask', 'cluster', 'bbox'],
            description='Camera bbox/OBB, exact instance mask, or legacy LiDAR cluster',
        ),
        DeclareLaunchArgument('start_segmentation', default_value='false'),
        DeclareLaunchArgument('start_detection', default_value='true'),
        DeclareLaunchArgument('start_path_planner', default_value='true'),
        DeclareLaunchArgument(
            'geometry_config', default_value=sensor_config,
            description='Optional voxel-only geometry override; defaults to sensor profile',
        ),
        DeclareLaunchArgument(
            'segmentation_executable', default_value='seg_q4_node',
            description='seg_q4_node (GGML Q4_0) or seg.py (TorchAO)',
        ),
        DeclareLaunchArgument(
            'detector_python', default_value='python3',
            description='Python interpreter containing Ultralytics (e.g. .venv-yolo/bin/python)',
        ),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        Node(
            package='ins_seg',
            executable='yolo_detector.py',
            name='ship_detector',
            output='screen',
            prefix=LaunchConfiguration('detector_python'),
            parameters=[detector_config, sensor_config],
            sigterm_timeout='30',
            sigkill_timeout='5',
            condition=IfCondition(LaunchConfiguration('start_detection')),
        ),
        Node(
            package='ins_seg',
            executable=LaunchConfiguration('segmentation_executable'),
            name='sam3_segmentation',
            output='screen',
            parameters=[common_config, sensor_config],
            sigterm_timeout='45',
            sigkill_timeout='5',
            condition=IfCondition(LaunchConfiguration('start_segmentation')),
        ),
        Node(
            package='pointcloud_voxels',
            executable='mask_voxels_node',
            name='mask_voxels',
            output='screen',
            parameters=[
                voxel_config,
                sensor_config,
                LaunchConfiguration('geometry_config'),
                {'selection_mode': LaunchConfiguration('selection_mode')},
            ],
        ),
        Node(
            package='pointcloud_voxels',
            executable='path_planner_node',
            name='ship_path_planner',
            output='screen',
            parameters=[voxel_config, sensor_config],
            condition=IfCondition(LaunchConfiguration('start_path_planner')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='ship_voxels_rviz',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            additional_env={'GTK_PATH': '', 'GIO_MODULE_DIR': '', 'GTK_MODULES': ''},
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ])
