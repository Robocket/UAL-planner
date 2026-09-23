#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    text_prompt = LaunchConfiguration('text_prompt')
    device = LaunchConfiguration('device')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('text_prompt', default_value='large ship'),
        DeclareLaunchArgument('device', default_value='auto'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='ins_seg',
            executable='seg.py',
            name='sam3_segmentation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'text_prompt': text_prompt,
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
