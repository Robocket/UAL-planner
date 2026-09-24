#!/usr/bin/env python3
"""Launch each UAL-planner algorithm as a peer node."""

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
    sensor_config = LaunchConfiguration('sensor_config')
    rviz_config = LaunchConfiguration('rviz_config')
    parameter_files = [common_config, sensor_config]

    return LaunchDescription([
        DeclareLaunchArgument(
            'common_config',
            default_value=os.path.join(share, 'config', 'common.yaml'),
            description='Parameters shared by every sensor profile',
        ),
        DeclareLaunchArgument(
            'sensor_config',
            default_value=os.path.join(share, 'config', 'd1.yaml'),
            description='Radar/camera topics, frames, extrinsics and overrides',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'ual_planner.rviz'),
            description='RViz display configuration for the selected profile',
        ),
        DeclareLaunchArgument('start_segmentation', default_value='true'),
        DeclareLaunchArgument(
            'segmentation_executable',
            default_value='seg_q4_node',
            description=(
                'SAM 3 backend: seg_q4_node for GGML Q4_0; '
                'seg.py for the TorchAO fallback'
            ),
        ),
        DeclareLaunchArgument('start_projection', default_value='true'),
        DeclareLaunchArgument('start_instance_graph', default_value='true'),
        DeclareLaunchArgument('start_landing_evaluator', default_value='true'),
        DeclareLaunchArgument(
            'start_rviz', default_value='false',
            description='Start the unified UAL-planner RViz view',
        ),
        Node(
            package='ins_seg',
            executable=LaunchConfiguration('segmentation_executable'),
            name='sam3_segmentation',
            output='screen',
            parameters=parameter_files,
            condition=IfCondition(LaunchConfiguration('start_segmentation')),
        ),
        Node(
            package='ins_seg',
            executable='projection.py',
            name='pointcloud_projection',
            output='screen',
            parameters=parameter_files,
            condition=IfCondition(LaunchConfiguration('start_projection')),
        ),
        Node(
            package='ins_seg',
            executable='graph.py',
            name='instance_graph',
            output='screen',
            parameters=parameter_files,
            condition=IfCondition(LaunchConfiguration('start_instance_graph')),
        ),
        Node(
            package='landing_evaluator',
            executable='landing_evaluator_node',
            name='landing_evaluator',
            output='screen',
            parameters=parameter_files,
            condition=IfCondition(
                LaunchConfiguration('start_landing_evaluator')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'rviz.launch.py')
            ),
            launch_arguments={
                'common_config': common_config,
                'sensor_config': sensor_config,
                'rviz_config': rviz_config,
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ])
