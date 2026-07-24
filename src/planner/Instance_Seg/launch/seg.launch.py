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
                'image_topic': '/camera/color/image_raw',
                'model_path': model_path,
                'confidence_threshold': 0.5,
                'iou_threshold': 0.5,
                'tracker_type': 'botsort.yaml',
                'device': device,
                'show_result': False,
                'publish_annotated': True,
            }],
        ),
        Node(
            package='ins_seg',
            executable='projection.py',
            name='pointcloud_projection',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'color_mode': 'instance',
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
