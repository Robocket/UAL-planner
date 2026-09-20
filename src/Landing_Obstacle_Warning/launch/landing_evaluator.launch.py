from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("landing_evaluator"), "config", "landing.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        Node(
            package="landing_evaluator",
            executable="landing_evaluator_node",
            name="landing_evaluator",
            output="screen",
            parameters=[LaunchConfiguration("config")],
        ),
    ])
