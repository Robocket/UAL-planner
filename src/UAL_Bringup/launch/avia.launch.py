#!/usr/bin/env python3

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
            default_value=os.path.join(share, 'config', 'avia.yaml'),
            description='Livox Avia radar/camera profile',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'ual_planner.launch.py')
            ),
            launch_arguments={
                'sensor_config': LaunchConfiguration('sensor_config'),
            }.items(),
        ),
    ])
