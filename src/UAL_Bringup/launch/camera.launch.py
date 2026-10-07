#!/usr/bin/env python3
"""Launch the generic raw-camera profile."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('ual_planner_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'common_config',
            default_value=os.path.join(share, 'config', 'common.yaml'),
        ),
        DeclareLaunchArgument(
            'camera_config',
            default_value=os.path.join(share, 'config', 'camera.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'visual_only.rviz'),
        ),
        DeclareLaunchArgument('start_segmentation', default_value='true'),
        DeclareLaunchArgument(
            'segmentation_executable', default_value='seg_q4_node'
        ),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'visual_only.launch.py')
            ),
            launch_arguments={
                'common_config': LaunchConfiguration('common_config'),
                'camera_config': LaunchConfiguration('camera_config'),
                'rviz_config': LaunchConfiguration('rviz_config'),
                'start_segmentation': LaunchConfiguration(
                    'start_segmentation'
                ),
                'segmentation_executable': LaunchConfiguration(
                    'segmentation_executable'
                ),
                'start_rviz': LaunchConfiguration('start_rviz'),
            }.items(),
        ),
    ])
