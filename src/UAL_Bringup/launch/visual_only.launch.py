#!/usr/bin/env python3
"""Launch the camera-only UAL processing path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ual_planner_bringup')
    common_config = LaunchConfiguration('common_config')
    camera_config = LaunchConfiguration('camera_config')
    rviz_config = LaunchConfiguration('rviz_config')
    parameter_files = [common_config, camera_config]

    return LaunchDescription([
        DeclareLaunchArgument(
            'common_config',
            default_value=os.path.join(share, 'config', 'common.yaml'),
            description='Parameters shared by all camera profiles',
        ),
        DeclareLaunchArgument(
            'camera_config',
            default_value=os.path.join(share, 'config', 'camera.yaml'),
            description='Camera topics, calibration and odometry interface',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'visual_only.rviz'),
        ),
        DeclareLaunchArgument('start_segmentation', default_value='true'),
        DeclareLaunchArgument(
            'segmentation_executable',
            default_value='seg_q4_node',
            description=(
                'seg_q4_node for GGML Q4_0; seg.py for the TorchAO fallback'
            ),
        ),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        Node(
            package='ins_seg',
            executable=LaunchConfiguration('segmentation_executable'),
            name='sam3_segmentation',
            output='screen',
            parameters=parameter_files,
            sigterm_timeout='45',
            sigkill_timeout='5',
            condition=IfCondition(LaunchConfiguration('start_segmentation')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'rviz.launch.py')
            ),
            launch_arguments={'rviz_config': rviz_config}.items(),
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ])
