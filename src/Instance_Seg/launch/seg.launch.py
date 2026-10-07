#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    text_prompt = LaunchConfiguration('text_prompt')
    image_topic = LaunchConfiguration('image_topic')
    segmentation_executable = LaunchConfiguration(
        'segmentation_executable'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('text_prompt', default_value='large ship'),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/color/image_raw'
        ),
        DeclareLaunchArgument(
            'segmentation_executable', default_value='seg_q4_node'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='ins_seg',
            executable=segmentation_executable,
            name='sam3_segmentation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'text_prompt': text_prompt,
                'image_topic': image_topic,
            }],
            sigterm_timeout='45',
            sigkill_timeout='5',
        ),
    ])
