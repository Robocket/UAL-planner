from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("landing_evaluator"),
        "rviz", "landing_evaluator.rviz")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool)
    publish_sensor_tf = LaunchConfiguration("publish_sensor_tf")
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("publish_sensor_tf", default_value="true"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("sensor_frame", default_value="avia_frame"),
        DeclareLaunchArgument("extrinsic_x", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_y", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_z", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_roll", default_value="0.0"),
        DeclareLaunchArgument("extrinsic_pitch", default_value="1.5707963267948966"),
        DeclareLaunchArgument("extrinsic_yaw", default_value="0.0"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="landing_lidar_static_tf",
            output="screen",
            arguments=[
                "--x", LaunchConfiguration("extrinsic_x"),
                "--y", LaunchConfiguration("extrinsic_y"),
                "--z", LaunchConfiguration("extrinsic_z"),
                "--roll", LaunchConfiguration("extrinsic_roll"),
                "--pitch", LaunchConfiguration("extrinsic_pitch"),
                "--yaw", LaunchConfiguration("extrinsic_yaw"),
                "--frame-id", LaunchConfiguration("base_frame"),
                "--child-frame-id", LaunchConfiguration("sensor_frame"),
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(publish_sensor_tf),
        ),
        Node(
            package="landing_evaluator",
            executable="landing_rviz_visualizer.py",
            name="landing_rviz_visualizer",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="landing_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[
                ("/livox/lidar", LaunchConfiguration("pointcloud_topic")),
            ],
            condition=IfCondition(use_rviz),
        ),
    ])
