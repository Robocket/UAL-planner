#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto_landing',
            executable='evaluate.py',
            name='landing_evaluator',
            output='screen',
            parameters=[{
                'input_topic': '/projected_cloud',
                'drone_radius': 0.5,
                'safety_margin': 0.15,
                'grid_resolution': 0.10,
                'max_slope_deg': 10.0,
                'max_roughness': 0.05,
                'max_height_range': 0.15,
            }],
        ),
        Node(
            package='auto_landing',
            executable='track_landing_point.py',
            name='landing_point_tracker',
            output='screen',
            parameters=[{
                'regions_topic': '/landing/regions',
                'landing_point_topic': '/landing/point',
                'publish_rate_hz': 10.0,
                'max_tracking_distance': 1.0,
                'min_initial_confidence': 0.0,
            }],
        ),
    ])
