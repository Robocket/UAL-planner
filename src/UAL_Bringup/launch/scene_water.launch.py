#!/usr/bin/env python3
"""Launch UAL-planner against the converted Scene_Water rosbag topics."""

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
            'sensor_config',
            default_value=os.path.join(share, 'config', 'scene_water.yaml'),
            description='Scene_Water converted rosbag profile',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'scene_water.rviz'),
            description='Scene_Water segmentation/projection view',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'ual_planner.launch.py')
            ),
            launch_arguments={
                'sensor_config': LaunchConfiguration('sensor_config'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items(),
        ),
    ])
