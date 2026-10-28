#!/usr/bin/env python3
"""Run the migrated ws_ual point-cloud-only selection for diagnostics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('pointcloud_voxels')
    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_config',
            default_value=os.path.join(share, 'config', 'scene_water.yaml'),
        ),
        DeclareLaunchArgument(
            'voxel_config',
            default_value=os.path.join(share, 'config', 'pointcloud_voxels.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'ship_voxels.rviz'),
        ),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'mask_voxels.launch.py')
            ),
            launch_arguments={
                'sensor_config': LaunchConfiguration('sensor_config'),
                'voxel_config': LaunchConfiguration('voxel_config'),
                'rviz_config': LaunchConfiguration('rviz_config'),
                'start_rviz': LaunchConfiguration('start_rviz'),
                'selection_mode': 'cluster',
                'start_segmentation': 'false',
                'start_detection': 'false',
            }.items(),
        ),
    ])
