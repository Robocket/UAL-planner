#!/usr/bin/env python3
"""Run camera detection/mask selected LiDAR voxels on Scene_Water."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('pointcloud_voxels')
    sensor_config = LaunchConfiguration('sensor_config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_config',
            default_value=os.path.join(share, 'config', 'scene_water.yaml'),
            description='Scene_Water original rosbag profile',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'ship_voxels.rviz'),
            description='Scene_Water mask and classified voxel view',
        ),
        DeclareLaunchArgument(
            'common_config',
            default_value=os.path.join(share, 'config', 'common.yaml'),
        ),
        DeclareLaunchArgument(
            'voxel_config',
            default_value=os.path.join(share, 'config', 'pointcloud_voxels.yaml'),
        ),
        DeclareLaunchArgument(
            'detector_config',
            default_value=os.path.join(share, 'config', 'detector.yaml'),
        ),
        DeclareLaunchArgument('selection_mode', default_value='detection'),
        DeclareLaunchArgument('geometry_config', default_value=sensor_config),
        DeclareLaunchArgument('start_segmentation', default_value='false'),
        DeclareLaunchArgument('start_detection', default_value='true'),
        DeclareLaunchArgument('start_path_planner', default_value='true'),
        DeclareLaunchArgument(
            'segmentation_executable', default_value='seg_q4_node',
        ),
        DeclareLaunchArgument(
            'detector_python', default_value='python3',
            description='Python interpreter containing Ultralytics',
        ),
        DeclareLaunchArgument('start_rviz', default_value='true'),
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
                os.path.join(share, 'launch', 'mask_voxels.launch.py')
            ),
            launch_arguments={
                'sensor_config': sensor_config,
                'geometry_config': LaunchConfiguration('geometry_config'),
                'common_config': LaunchConfiguration('common_config'),
                'voxel_config': LaunchConfiguration('voxel_config'),
                'detector_config': LaunchConfiguration('detector_config'),
                'rviz_config': LaunchConfiguration('rviz_config'),
                'selection_mode': LaunchConfiguration('selection_mode'),
                'start_segmentation': LaunchConfiguration('start_segmentation'),
                'start_detection': LaunchConfiguration('start_detection'),
                'start_path_planner': LaunchConfiguration('start_path_planner'),
                'segmentation_executable': LaunchConfiguration(
                    'segmentation_executable'
                ),
                'detector_python': LaunchConfiguration('detector_python'),
                'start_rviz': LaunchConfiguration('start_rviz'),
            }.items(),
        ),
    ])
