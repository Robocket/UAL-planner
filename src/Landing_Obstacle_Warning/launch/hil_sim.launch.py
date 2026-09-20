import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    evaluator_share = get_package_share_directory("landing_evaluator")
    default_config = os.path.join(evaluator_share, "config", "hil_sim.yaml")
    visualization_launch = os.path.join(
        evaluator_share, "launch", "landing_visualization.launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    base_frame = LaunchConfiguration("base_frame")

    translation = ParameterValue([
        [LaunchConfiguration("extrinsic_x")],
        [LaunchConfiguration("extrinsic_y")],
        [LaunchConfiguration("extrinsic_z")],
    ], value_type=List[float])
    rotation_rpy = ParameterValue([
        [LaunchConfiguration("extrinsic_roll")],
        [LaunchConfiguration("extrinsic_pitch")],
        [LaunchConfiguration("extrinsic_yaw")],
    ], value_type=List[float])
    attitude_rotation_rpy = ParameterValue([
        [LaunchConfiguration("attitude_extrinsic_roll")],
        [LaunchConfiguration("attitude_extrinsic_pitch")],
        [LaunchConfiguration("attitude_extrinsic_yaw")],
    ], value_type=List[float])

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use the simulation /clock topic"),
        DeclareLaunchArgument(
            "pointcloud_topic", default_value="/livox/avia/points",
            description="sensor_msgs/msg/PointCloud2 input"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "sensor_frame",
            default_value="981-A/livox_avia_link/livox_avia",
            description="Must exactly match PointCloud2 header.frame_id"),
        DeclareLaunchArgument(
            "use_ground_truth_attitude", default_value="true",
            description="Convert /gt/odom orientation into a gravity vector"),
        DeclareLaunchArgument("odometry_topic", default_value="/gt/odom"),
        DeclareLaunchArgument(
            "gravity_imu_topic", default_value="/landing/sim_gravity_imu"),
        DeclareLaunchArgument(
            "world_up_z", default_value="1.0",
            description="+1 for Gazebo/ENU; -1 only for a verified NED world"),
        DeclareLaunchArgument(
            "expected_odom_frame", default_value="981-A/odom"),
        DeclareLaunchArgument(
            "expected_body_frame", default_value="981-A/base_footprint"),
        DeclareLaunchArgument(
            "attitude_extrinsic_roll", default_value="0.0",
            description="Odometry child axes -> evaluator body roll [rad]"),
        DeclareLaunchArgument(
            "attitude_extrinsic_pitch", default_value="0.0"),
        DeclareLaunchArgument(
            "attitude_extrinsic_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "landing_radius", default_value="1.25",
            description="Required aircraft landing radius including margin [m]"),
        DeclareLaunchArgument(
            "startup_enabled", default_value="true",
            description="Start evaluating immediately; false waits for enable topic/service"),
        DeclareLaunchArgument(
            "enable_topic", default_value="/landing_evaluator/enable"),
        DeclareLaunchArgument(
            "visualize", default_value="true",
            description="Publish landing markers/TF and optionally start RViz"),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Start local RViz; set false when RViz runs remotely"),
        DeclareLaunchArgument(
            "publish_sensor_tf", default_value="true",
            description="Publish base_frame -> sensor_frame static transform"),
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
                {
                    "use_sim_time": ParameterValue(
                        use_sim_time, value_type=bool),
                    "input_topic": pointcloud_topic,
                    "output_frame": base_frame,
                    "landing_radius": ParameterValue(
                        LaunchConfiguration("landing_radius"),
                        value_type=float),
                    "startup_enabled": ParameterValue(
                        LaunchConfiguration("startup_enabled"),
                        value_type=bool),
                    "enable_topic": LaunchConfiguration("enable_topic"),
                    "extrinsics.translation": translation,
                    "extrinsics.rotation_rpy": rotation_rpy,
                    "imu.enabled": ParameterValue(
                        LaunchConfiguration("use_ground_truth_attitude"),
                        value_type=bool),
                    "imu.required": ParameterValue(
                        LaunchConfiguration("use_ground_truth_attitude"),
                        value_type=bool),
                    "imu.topic": LaunchConfiguration("gravity_imu_topic"),
                    "imu.extrinsics.rotation_rpy": attitude_rotation_rpy,
                },
            ],
        ),
        Node(
            package="landing_evaluator",
            executable="odom_gravity_adapter.py",
            name="landing_odom_gravity_adapter",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool),
                "input_topic": LaunchConfiguration("odometry_topic"),
                "output_topic": LaunchConfiguration("gravity_imu_topic"),
                "world_up_z": ParameterValue(
                    LaunchConfiguration("world_up_z"), value_type=float),
                "expected_parent_frame": LaunchConfiguration(
                    "expected_odom_frame"),
                "expected_child_frame": LaunchConfiguration(
                    "expected_body_frame"),
            }],
            condition=IfCondition(
                LaunchConfiguration("use_ground_truth_attitude")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visualization_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_rviz": LaunchConfiguration("use_rviz"),
                "pointcloud_topic": pointcloud_topic,
                "publish_sensor_tf": LaunchConfiguration("publish_sensor_tf"),
                "base_frame": base_frame,
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
