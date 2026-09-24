#!/usr/bin/env python3
"""Launch UAL-planner against the original Scene_Water rosbag topics."""

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
            default_value=os.path.join(share, 'config', 'scene_water.yaml'),
            description='Scene_Water original rosbag profile',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'scene_water.rviz'),
            description='Scene_Water segmentation/projection view',
        ),
        DeclareLaunchArgument(
            'start_image_adapter',
            default_value='true',
            description='Convert the bag CompressedImage stream to Image',
        ),
        DeclareLaunchArgument(
            'start_livox_adapter',
            default_value='true',
            description='Convert the bag Livox CustomMsg stream to PointCloud2',
        ),
        Node(
            package='ins_seg',
            executable='compressed_image_adapter.py',
            name='scene_water_image_adapter',
            output='screen',
            parameters=[sensor_config],
            condition=IfCondition(LaunchConfiguration('start_image_adapter')),
        ),
        Node(
            package='livox_avia_driver',
            executable='custom_to_pointcloud2_node',
            name='livox_custom_to_pointcloud2',
            output='screen',
            parameters=[sensor_config],
            condition=IfCondition(LaunchConfiguration('start_livox_adapter')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'ual_planner.launch.py')
            ),
            launch_arguments={
                'sensor_config': sensor_config,
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items(),
        ),
    ])
