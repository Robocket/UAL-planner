#!/usr/bin/env python3
"""Play the converted bag and display raw camera images and LiDAR in RViz."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start_player(context):
    bag = Path(LaunchConfiguration('bag').perform(context)).expanduser().resolve()
    if not (bag/'metadata.yaml').is_file():
        raise RuntimeError(f'ROS 2 bag metadata.yaml not found: {bag}')
    command = [
        'ros2', 'bag', 'play', str(bag), '--clock', '--delay', '3',
        '--disable-keyboard-controls',
        '--rate', LaunchConfiguration('rate').perform(context),
        '--start-offset', LaunchConfiguration('start_offset').perform(context),
        '--playback-duration', LaunchConfiguration('playback_duration').perform(context),
    ]
    if LaunchConfiguration('loop').perform(context).lower() in ('true', '1', 'yes'):
        command.append('--loop')
    return [ExecuteProcess(cmd=command, output='screen')]


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag', default_value=str(root/'data/Scene_Water_20260805_013651_ros2_jazzy'),
        ),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start_offset', default_value='0.0'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('playback_duration', default_value='-1.0'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=str(root/'rviz/scene_water_playback.rviz'),
        ),
        Node(
            package='rviz2', executable='rviz2', name='scene_water_playback_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}], output='screen',
            # A VS Code Snap terminal can inject GTK/GIO plugins linked to
            # core20's libc into the system Qt process. Keep those plugins
            # out of RViz while preserving the ROS library paths and display.
            additional_env={'GTK_PATH': '', 'GIO_MODULE_DIR': '', 'GTK_MODULES': ''},
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
        OpaqueFunction(function=_start_player),
    ])
