import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    evaluator_share = get_package_share_directory("landing_evaluator")
    default_config = os.path.join(evaluator_share, "config", "d1_test.yaml")
    visualization_launch = os.path.join(
        evaluator_share, "launch", "landing_visualization.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("evaluator_config", default_value=default_config),
        DeclareLaunchArgument(
            "driver_config", default_value="",
            description="Absolute Seyond config path when start_driver=true"),
        DeclareLaunchArgument(
            "start_driver", default_value="false",
            description="Optionally start seyond_node; no package dependency by default"),
        DeclareLaunchArgument("visualize", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("sensor_frame", default_value="seyond"),
        DeclareLaunchArgument("publish_sensor_tf", default_value="true"),
        DeclareLaunchArgument("extrinsic_x", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_y", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_z", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_roll", default_value="0.0"),
        DeclareLaunchArgument(
            "extrinsic_pitch", default_value="1.5707963267948966"),
        DeclareLaunchArgument("extrinsic_yaw", default_value="0.0"),
        Node(
            package="seyond",
            executable="seyond_node",
            name="seyond_d1",
            output="screen",
            parameters=[{"config_path": LaunchConfiguration("driver_config")}],
            condition=IfCondition(LaunchConfiguration("start_driver")),
        ),
        Node(
            package="landing_evaluator",
            executable="landing_evaluator_node",
            name="landing_evaluator",
            output="screen",
            parameters=[LaunchConfiguration("evaluator_config")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch),
            launch_arguments={
                "use_rviz": LaunchConfiguration("use_rviz"),
                "pointcloud_topic": "/iv_points",
                "publish_sensor_tf": LaunchConfiguration("publish_sensor_tf"),
                "base_frame": "base_link",
                "sensor_frame": LaunchConfiguration("sensor_frame"),
                "extrinsic_x": LaunchConfiguration("extrinsic_x"),
                "extrinsic_y": LaunchConfiguration("extrinsic_y"),
                "extrinsic_z": LaunchConfiguration("extrinsic_z"),
                "extrinsic_roll": LaunchConfiguration("extrinsic_roll"),
                "extrinsic_pitch": LaunchConfiguration("extrinsic_pitch"),
                "extrinsic_yaw": LaunchConfiguration("extrinsic_yaw"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("visualize")),
        ),
    ])
