#!/usr/bin/env python3
"""Start the camera-only visualization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ual_planner_bringup')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'visual_only.rviz'),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='ual_planner_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
