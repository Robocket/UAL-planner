#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def body_up_from_orientation(orientation, world_up_z):
    """Return world vertical expressed in the Odometry child/body frame.

    nav_msgs/Odometry defines pose.orientation as the orientation of the child
    frame in the header/world frame.  If R rotates body vectors into world,
    the required gravity-up vector is R^T * [0, 0, world_up_z].
    """
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-9:
        return None

    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return (
        world_up_z * 2.0 * (x * z - w * y),
        world_up_z * 2.0 * (y * z + w * x),
        world_up_z * (1.0 - 2.0 * (x * x + y * y)),
    )


class OdomGravityAdapter(Node):
    def __init__(self):
        super().__init__("landing_odom_gravity_adapter")
        self.input_topic = self.declare_parameter(
            "input_topic", "/gt/odom").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/landing/sim_gravity_imu").value
        self.gravity_magnitude = float(self.declare_parameter(
            "gravity_magnitude", 9.80665).value)
        self.world_up_z = float(self.declare_parameter(
            "world_up_z", 1.0).value)
        self.expected_parent_frame = self.declare_parameter(
            "expected_parent_frame", "").value
        self.expected_child_frame = self.declare_parameter(
            "expected_child_frame", "").value
        self.output_frame = self.declare_parameter(
            "output_frame", "").value

        if self.gravity_magnitude <= 0.0:
            raise ValueError("gravity_magnitude must be positive")
        if self.world_up_z not in (-1.0, 1.0):
            raise ValueError("world_up_z must be +1.0 (ENU) or -1.0 (NED)")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(Imu, self.output_topic, qos)
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self.on_odometry, qos)
        self.parent_mismatch_reported = False
        self.child_mismatch_reported = False
        self.invalid_quaternion_reported = False

        convention = "+Z-up (ENU/Gazebo)" if self.world_up_z > 0.0 else "-Z-up (NED)"
        self.get_logger().info(
            f"Odometry gravity adapter ready: {self.input_topic} -> "
            f"{self.output_topic}, {convention}")

    def on_odometry(self, odometry):
        parent = odometry.header.frame_id
        child = odometry.child_frame_id
        if (self.expected_parent_frame and
                parent != self.expected_parent_frame and
                not self.parent_mismatch_reported):
            self.get_logger().warning(
                f"Odometry parent frame is '{parent}', expected "
                f"'{self.expected_parent_frame}'")
            self.parent_mismatch_reported = True
        if (self.expected_child_frame and
                child != self.expected_child_frame and
                not self.child_mismatch_reported):
            self.get_logger().warning(
                f"Odometry child frame is '{child}', expected "
                f"'{self.expected_child_frame}'")
            self.child_mismatch_reported = True

        up = body_up_from_orientation(
            odometry.pose.pose.orientation, self.world_up_z)
        if up is None:
            if not self.invalid_quaternion_reported:
                self.get_logger().warning(
                    "Ignoring Odometry with a zero or non-finite quaternion")
                self.invalid_quaternion_reported = True
            return
        self.invalid_quaternion_reported = False

        output = Imu()
        output.header.stamp = odometry.header.stamp
        # The vector is expressed in the Odometry child/body frame.  A named
        # output override is useful when an equivalent body-frame alias is
        # required; otherwise preserve the actual child frame in the header.
        output.header.frame_id = self.output_frame or child
        output.orientation_covariance[0] = -1.0
        output.angular_velocity_covariance[0] = -1.0
        output.linear_acceleration.x = self.gravity_magnitude * up[0]
        output.linear_acceleration.y = self.gravity_magnitude * up[1]
        output.linear_acceleration.z = self.gravity_magnitude * up[2]
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = OdomGravityAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # A launch service can deliver another SIGINT while Jazzy is
            # destroying entities; shutdown is already in progress.
            pass


if __name__ == "__main__":
    main()
