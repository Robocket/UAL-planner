#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='yolo11n-seg.pt'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='ins_seg',
            executable='seg.py',
            name='yolo_segmentation_tracker',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_path': model_path,
                'device': device,
            }],
        ),
        Node(
            package='ins_seg',
            executable='projection.py',
            name='pointcloud_projection',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='ins_seg',
            executable='graph.py',
            name='instance_graph',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
