#!/usr/bin/env python3
"""Start the visualization shared by all UAL-planner sensor profiles."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ual_planner_bringup')
    common_config = LaunchConfiguration('common_config')
    sensor_config = LaunchConfiguration('sensor_config')
    rviz_config = LaunchConfiguration('rviz_config')
    parameter_files = [common_config, sensor_config]

    return LaunchDescription([
        DeclareLaunchArgument(
            'common_config',
            default_value=os.path.join(share, 'config', 'common.yaml'),
        ),
        DeclareLaunchArgument(
            'sensor_config',
            default_value=os.path.join(share, 'config', 'd1.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'ual_planner.rviz'),
        ),
        Node(
            package='landing_evaluator',
            executable='landing_rviz_visualizer.py',
            name='landing_rviz_visualizer',
            output='screen',
            parameters=parameter_files,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='ual_planner_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=parameter_files,
        ),
    ])
