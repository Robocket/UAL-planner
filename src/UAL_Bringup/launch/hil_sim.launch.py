#!/usr/bin/env python3

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
    sensor_config = LaunchConfiguration('sensor_config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_config',
            default_value=os.path.join(share, 'config', 'hil_sim.yaml'),
            description='Avia HIL radar/camera profile',
        ),
        DeclareLaunchArgument(
            'use_ground_truth_attitude', default_value='true',
            description='Convert HIL odometry attitude to a gravity IMU topic',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'ual_planner.launch.py')
            ),
            launch_arguments={'sensor_config': sensor_config}.items(),
        ),
        Node(
            package='landing_evaluator',
            executable='odom_gravity_adapter.py',
            name='landing_odom_gravity_adapter',
            output='screen',
            parameters=[sensor_config],
            condition=IfCondition(
                LaunchConfiguration('use_ground_truth_attitude')
            ),
        ),
    ])
