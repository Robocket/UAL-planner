from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    evaluator_share = get_package_share_directory("landing_evaluator")
    default_config = os.path.join(
        evaluator_share, "config", "avia_test.yaml")
    visualization_launch = os.path.join(
        evaluator_share, "launch", "landing_visualization.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument(
            "visualize", default_value="true",
            description="Start landing markers/TF and optionally RViz"),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Start RViz; set false when RViz runs on another computer"),
        DeclareLaunchArgument(
            "pointcloud_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument(
            "convert_custom", default_value="false",
            description="Convert a remapped Livox CustomMsg bag to PointCloud2"),
        DeclareLaunchArgument(
            "custom_topic", default_value="/livox/lidar_custom",
            description="CustomMsg input used when convert_custom=true"),
        DeclareLaunchArgument("sensor_frame", default_value="avia_frame"),
        DeclareLaunchArgument("publish_sensor_tf", default_value="true"),
        DeclareLaunchArgument("extrinsic_x", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_y", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_z", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_roll", default_value="0.0"),
        DeclareLaunchArgument(
            "extrinsic_pitch", default_value="1.5707963267948966"),
        DeclareLaunchArgument("extrinsic_yaw", default_value="0.0"),
        Node(
            package="landing_evaluator",
            executable="landing_evaluator_node",
            name="landing_evaluator",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {"input_topic": LaunchConfiguration("pointcloud_topic")},
            ],
        ),
        Node(
            package="livox_avia_driver",
            executable="custom_to_pointcloud2_node",
            name="livox_custom_to_pointcloud2",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("custom_topic"),
                "output_topic": LaunchConfiguration("pointcloud_topic"),
            }],
            condition=IfCondition(LaunchConfiguration("convert_custom")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch),
            launch_arguments={
                "use_rviz": LaunchConfiguration("use_rviz"),
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
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
