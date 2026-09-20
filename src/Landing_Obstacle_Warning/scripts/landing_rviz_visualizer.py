#!/usr/bin/env python3
"""Render landing plane, airspace occupancy and status for RViz."""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from landing_evaluator.msg import HeightMap, LandingStatus


class LandingRvizVisualizer(Node):
    def __init__(self):
        super().__init__("landing_rviz_visualizer")
        self.declare_parameter("status_topic", "/landing/status")
        self.declare_parameter("height_map_topic", "/landing/height_map")
        self.declare_parameter("marker_topic", "/landing/markers")
        self.declare_parameter("diagnostics_topic", "/landing/diagnostics")
        self.declare_parameter("text_height", 0.18)
        self.declare_parameter("plane_thickness", 0.025)

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, latched)
        self.diag_pub = self.create_publisher(
            DiagnosticArray, self.get_parameter("diagnostics_topic").value, latched)
        self.latest_map = None
        self.latest_status = None
        self.map_sub = self.create_subscription(
            HeightMap, self.get_parameter("height_map_topic").value,
            self.on_map, reliable)
        self.status_sub = self.create_subscription(
            LandingStatus, self.get_parameter("status_topic").value,
            self.on_status, reliable)
        self.get_logger().info("Landing RViz visualizer ready")

    def on_map(self, msg):
        self.latest_map = msg
        if self.latest_status is not None and self.same_stamp(
                msg.header.stamp, self.latest_status.header.stamp):
            self.publish_markers(self.latest_status, msg)

    @staticmethod
    def same_stamp(left, right):
        return (left.sec == right.sec and left.nanosec == right.nanosec)

    @staticmethod
    def color_for_status(status, alpha=0.75):
        if status == LandingStatus.LANDABLE:
            return ColorRGBA(r=0.1, g=0.9, b=0.2, a=alpha)
        if status == LandingStatus.NOT_LANDABLE:
            return ColorRGBA(r=0.95, g=0.1, b=0.1, a=alpha)
        return ColorRGBA(r=1.0, g=0.75, b=0.05, a=alpha)

    @staticmethod
    def status_name(status):
        return {
            LandingStatus.UNKNOWN: "UNKNOWN",
            LandingStatus.LANDABLE: "LANDABLE",
            LandingStatus.NOT_LANDABLE: "NOT_LANDABLE",
        }.get(status, f"INVALID({status})")

    @staticmethod
    def fitted_z(height_map, x, y):
        if abs(height_map.plane_c) < 1e-6:
            return math.nan
        return -(
            height_map.plane_a * x +
            height_map.plane_b * y +
            height_map.plane_d) / height_map.plane_c

    def on_status(self, status):
        self.latest_status = status
        if status.reason_mask & LandingStatus.REASON_CLOUD_TIMEOUT:
            self.publish_timeout_markers(status)
            self.publish_diagnostics(status)
            return
        height_map = self.latest_map
        if height_map is not None and self.same_stamp(
                height_map.header.stamp, status.header.stamp):
            self.publish_markers(status, height_map)
        self.publish_diagnostics(status)

    def publish_timeout_markers(self, status):
        markers = MarkerArray()
        clear = Marker()
        clear.header = status.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        text = Marker()
        text.header = status.header
        text.ns = "landing_status"
        text.id = 4
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.pose.position.z = 0.5
        text.scale.z = float(self.get_parameter("text_height").value)
        text.color = self.color_for_status(LandingStatus.UNKNOWN, 1.0)
        text.text = "UNKNOWN\nreason: cloud_timeout\nLiDAR stream unavailable"
        markers.markers.append(text)
        self.marker_pub.publish(markers)

    def base_marker(self, height_map, marker_id, marker_type, name):
        marker = Marker()
        marker.header = height_map.header
        marker.ns = name
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish_markers(self, status, height_map):
        markers = MarkerArray()
        clear = Marker()
        clear.header = height_map.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        plane = self.base_marker(height_map, 1, Marker.CUBE_LIST, "landing_plane")
        plane.scale.x = height_map.resolution * 0.92
        plane.scale.y = height_map.resolution * 0.92
        plane.scale.z = float(self.get_parameter("plane_thickness").value)
        plane_color = self.color_for_status(status.status)

        occupied = self.base_marker(height_map, 2, Marker.CUBE_LIST, "airspace_occupied")
        occupied.scale.x = height_map.resolution * 0.70
        occupied.scale.y = height_map.resolution * 0.70
        occupied.scale.z = 0.08
        occupied.color = ColorRGBA(r=1.0, g=0.0, b=0.8, a=0.95)

        highest_ground = -math.inf
        cell_count = int(height_map.width) * int(height_map.height)
        safe_count = min(
            cell_count, len(height_map.ground_point_count),
            len(height_map.airspace_occupied))
        for index in range(safe_count):
            row, col = divmod(index, int(height_map.width))
            x = height_map.origin_x + (col + 0.5) * height_map.resolution
            y = height_map.origin_y + (row + 0.5) * height_map.resolution
            if height_map.ground_point_count[index] > 0:
                z = self.fitted_z(height_map, x, y)
                if math.isfinite(z):
                    plane.points.append(Point(x=x, y=y, z=z))
                    plane.colors.append(plane_color)
                    highest_ground = max(highest_ground, z)
            if height_map.airspace_occupied[index]:
                z = (height_map.highest_point[index]
                     if index < len(height_map.highest_point) else math.nan)
                if math.isfinite(z):
                    occupied.points.append(Point(x=x, y=y, z=z))

        markers.markers.extend([plane, occupied])

        boundary = self.base_marker(
            height_map, 3, Marker.LINE_STRIP, "landing_boundary")
        boundary.scale.x = 0.025
        boundary.color = self.color_for_status(status.status, 1.0)
        radius = max(abs(height_map.origin_x), abs(height_map.origin_y))
        for step in range(65):
            angle = 2.0 * math.pi * step / 64.0
            x, y = radius * math.cos(angle), radius * math.sin(angle)
            z = self.fitted_z(height_map, x, y)
            if math.isfinite(z):
                boundary.points.append(Point(x=x, y=y, z=z + 0.02))
        markers.markers.append(boundary)

        text = self.base_marker(height_map, 4, Marker.TEXT_VIEW_FACING, "landing_status")
        text.pose.position.x = 0.0
        text.pose.position.y = 0.0
        text.pose.position.z = (
            highest_ground + 0.55 if math.isfinite(highest_ground) else 0.5)
        text.scale.z = float(self.get_parameter("text_height").value)
        text.color = self.color_for_status(status.status, 1.0)
        text.text = (
            f"{self.status_name(status.status)}\n"
            f"reason: {status.reason}\n"
            f"height: {status.plane_height:.2f} m | slope: {status.slope_deg:.1f} deg\n"
            f"coverage: {status.coverage_ratio:.2f} | inliers: {status.inlier_ratio:.2f}\n"
            f"occupied: {status.occupied_cell_ratio:.2f} | step: {status.max_height_step:.2f} m\n"
            f"ROI points: {status.roi_point_count} | CPU: {status.processing_time_ms:.1f} ms")
        markers.markers.append(text)
        self.marker_pub.publish(markers)

    def publish_diagnostics(self, status):
        array = DiagnosticArray()
        array.header = status.header
        item = DiagnosticStatus()
        item.name = "landing_evaluator/surface_and_airspace"
        item.hardware_id = "landing_evaluator"
        if status.status == LandingStatus.LANDABLE:
            item.level = DiagnosticStatus.OK
        elif status.status == LandingStatus.NOT_LANDABLE:
            item.level = DiagnosticStatus.ERROR
        else:
            item.level = DiagnosticStatus.WARN
        item.message = self.status_name(status.status) + ": " + status.reason
        values = {
            "status": self.status_name(status.status),
            "raw_status": self.status_name(status.raw_status),
            "reason": status.reason,
            "plane_height_m": f"{status.plane_height:.3f}",
            "slope_deg": f"{status.slope_deg:.3f}",
            "coverage_ratio": f"{status.coverage_ratio:.3f}",
            "inlier_ratio": f"{status.inlier_ratio:.3f}",
            "occupied_cell_ratio": f"{status.occupied_cell_ratio:.3f}",
            "max_airspace_intrusion_m": f"{status.max_obstacle_height:.3f}",
            "max_height_step_m": f"{status.max_height_step:.3f}",
            "roughness_rms_m": f"{status.roughness_rms:.3f}",
            "roughness_p95_m": f"{status.roughness_p95:.3f}",
            "roi_point_count": str(status.roi_point_count),
            "processing_time_ms": f"{status.processing_time_ms:.3f}",
        }
        item.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        array.status.append(item)
        self.diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = LandingRvizVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
