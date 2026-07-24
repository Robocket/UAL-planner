#!/usr/bin/env python3
"""Lock the best landing region and continuously publish its tracked center."""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker

from auto_landing.msg import LandingRegionArray


class LandingPointTracker(Node):
    """Select once, then track the selected region by spatial proximity."""

    def __init__(self):
        super().__init__('landing_point_tracker')
        defaults = {
            'regions_topic': '/landing/regions',
            'landing_point_topic': '/landing/point',
            'marker_topic': '/landing/selected_marker',
            'publish_rate_hz': 10.0,
            'max_tracking_distance': 1.0,
            'min_initial_confidence': 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.publish_rate = float(
            self.get_parameter('publish_rate_hz').value
        )
        self.max_tracking_distance = float(
            self.get_parameter('max_tracking_distance').value
        )
        self.min_initial_confidence = float(
            self.get_parameter('min_initial_confidence').value
        )
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate_hz 必须大于 0')
        if self.max_tracking_distance <= 0.0:
            raise ValueError('max_tracking_distance 必须大于 0')
        if not 0.0 <= self.min_initial_confidence <= 1.0:
            raise ValueError('min_initial_confidence 必须在 [0, 1] 范围内')

        self._point = None
        self._frame_id = ''
        self._last_region_id = None

        self.point_pub = self.create_publisher(
            PointStamped,
            self.get_parameter('landing_point_topic').value,
            10,
        )
        self.marker_pub = self.create_publisher(
            Marker, self.get_parameter('marker_topic').value, 10
        )
        self.subscription = self.create_subscription(
            LandingRegionArray,
            self.get_parameter('regions_topic').value,
            self.regions_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_landing_point
        )
        self.get_logger().info(
            '降落点跟踪节点已启动，等待锁定最高置信度区域'
        )

    @staticmethod
    def _distance(first, second):
        return math.sqrt(
            (first.x - second.x) ** 2
            + (first.y - second.y) ** 2
            + (first.z - second.z) ** 2
        )

    def regions_callback(self, message):
        if not message.regions:
            return

        if self._point is None:
            selected = max(message.regions, key=lambda region: region.confidence)
            if selected.confidence < self.min_initial_confidence:
                return
            self._update_target(selected, message.header.frame_id)
            self.get_logger().info(
                f'已锁定降落区域 ID={selected.id}, '
                f'confidence={selected.confidence:.3f}'
            )
            return

        if message.header.frame_id != self._frame_id:
            self.get_logger().warning(
                '收到不同坐标系的降落区域，保留当前降落点且不更新'
            )
            return

        selected = min(
            message.regions,
            key=lambda region: self._distance(region.center, self._point),
        )
        distance = self._distance(selected.center, self._point)
        if distance <= self.max_tracking_distance:
            self._update_target(selected, message.header.frame_id)
        else:
            self.get_logger().warning(
                f'已锁定区域暂时丢失（最近候选距离 {distance:.2f} m），'
                '继续发布最后一次中心'
            )

    def _update_target(self, region, frame_id):
        # Copy values instead of retaining a mutable ROS message reference.
        point = PointStamped()
        point.point.x = region.center.x
        point.point.y = region.center.y
        point.point.z = region.center.z
        self._point = point.point
        self._frame_id = frame_id
        self._last_region_id = region.id

    def publish_landing_point(self):
        if self._point is None:
            return

        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.point.x = self._point.x
        message.point.y = self._point.y
        message.point.z = self._point.z
        self.point_pub.publish(message)

        marker = Marker()
        marker.header = message.header
        marker.ns = 'selected_landing_point'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = message.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 0.9
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LandingPointTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
