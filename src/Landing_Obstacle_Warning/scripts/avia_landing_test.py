#!/usr/bin/env python3
"""Runtime acceptance test for Avia landing evaluation output."""

import math
import statistics
import sys
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from landing_evaluator.msg import LandingStatus


class AviaLandingTest(Node):
    def __init__(self):
        super().__init__("avia_landing_test")
        self.declare_parameter("status_topic", "/landing/status")
        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("duration_sec", 15.0)
        self.declare_parameter("min_messages", 50)
        self.declare_parameter("min_negative_plane_ratio", 0.90)
        self.declare_parameter("max_processing_ms", 100.0)

        self.duration = float(self.get_parameter("duration_sec").value)
        topic = str(self.get_parameter("status_topic").value)
        self.status_topic = topic
        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(
            LandingStatus, topic, self.on_status, qos)
        self.lidar_subscription = self.create_subscription(
            PointCloud2, self.lidar_topic, self.on_lidar,
            qos_profile_sensor_data)
        self.started = time.monotonic()
        self.arrivals = []
        self.lidar_arrivals = []
        self.processing = []
        self.plane_heights = []
        self.roi_counts = []
        self.statuses = Counter()
        self.reasons = Counter()
        self.timer = self.create_timer(0.1, self.check_done)
        self.done = False
        self.passed = False
        self.get_logger().info(
            f"Collecting {topic} for {self.duration:.1f} s")

    def on_lidar(self, _msg):
        self.lidar_arrivals.append(time.monotonic())

    def on_status(self, msg):
        now = time.monotonic()
        self.arrivals.append(now)
        self.processing.append(float(msg.processing_time_ms))
        self.roi_counts.append(int(msg.roi_point_count))
        self.statuses[int(msg.status)] += 1
        for reason in msg.reason.split(","):
            if reason and reason not in ("none", "stabilizing"):
                self.reasons[reason] += 1
        # A zero height is the evaluator's no-plane placeholder, not evidence
        # for validating the mounting direction.
        if math.isfinite(msg.plane_height) and abs(msg.plane_height) > 1e-4:
            self.plane_heights.append(float(msg.plane_height))

    def check_done(self):
        if time.monotonic() - self.started >= self.duration:
            self.finish()

    def finish(self):
        if self.done:
            return
        self.done = True
        count = len(self.arrivals)
        lidar_count = len(self.lidar_arrivals)
        min_messages = int(self.get_parameter("min_messages").value)
        min_negative = float(
            self.get_parameter("min_negative_plane_ratio").value)
        max_processing = float(self.get_parameter("max_processing_ms").value)
        frequency = 0.0
        if count > 1 and self.arrivals[-1] > self.arrivals[0]:
            frequency = (count - 1) / (self.arrivals[-1] - self.arrivals[0])
        negative_ratio = (
            sum(value < 0.0 for value in self.plane_heights) /
            len(self.plane_heights) if self.plane_heights else 0.0)
        mean_processing = statistics.fmean(self.processing) if count else math.inf
        peak_processing = max(self.processing, default=math.inf)
        mean_roi = statistics.fmean(self.roi_counts) if count else 0.0

        checks = {
            "lidar_input_available": lidar_count >= min_messages,
            "enough_messages": count >= min_messages,
            "valid_planes": bool(self.plane_heights),
            "ground_below_aircraft": negative_ratio >= min_negative,
            "processing_deadline": peak_processing <= max_processing,
        }
        self.passed = all(checks.values())
        publisher_count = self.count_publishers(self.status_topic)
        lidar_publisher_count = self.count_publishers(self.lidar_topic)
        status_ratio = count / lidar_count if lidar_count else 0.0
        unknown_ratio = (
            self.statuses[LandingStatus.UNKNOWN] / count if count else 1.0)
        names = {0: "UNKNOWN", 1: "LANDABLE", 2: "NOT_LANDABLE"}
        status_text = ", ".join(
            f"{names.get(key, str(key))}={value}" for key, value in sorted(self.statuses.items()))
        reason_text = ", ".join(
            f"{key}={value}" for key, value in self.reasons.most_common()) or "none"
        self.get_logger().info(
            "Avia test summary: messages=%d, frequency=%.2f Hz, mean_roi=%.1f, "
            "negative_plane_ratio=%.3f, processing_mean=%.2f ms, "
            "processing_peak=%.2f ms" % (
                count, frequency, mean_roi, negative_ratio,
                mean_processing, peak_processing))
        self.get_logger().info(
            "Input summary: lidar_messages=%d, status/lidar=%.3f, "
            "unknown_ratio=%.3f" % (lidar_count, status_ratio, unknown_ratio))
        self.get_logger().info(
            f"Detected publishers on {self.status_topic}: {publisher_count}")
        self.get_logger().info(
            f"Detected publishers on {self.lidar_topic}: {lidar_publisher_count}")
        if lidar_count == 0 and lidar_publisher_count > 0:
            self.get_logger().error(
                "A LiDAR publisher exists but no PointCloud2 was decoded. The bag "
                "contains incompatible/corrupt serialized frames or a QoS/type mismatch.")
        if lidar_count > 0 and count == 0 and publisher_count > 0:
            self.get_logger().error(
                "PointCloud2 is arriving but no LandingStatus was decoded. Restart "
                "evaluator and test from the same landing_evaluator install space; "
                "the LandingStatus interface versions likely differ.")
        if count and unknown_ratio > 0.90:
            self.get_logger().warning(
                "More than 90% of decisions are UNKNOWN. The data chain works, but "
                "ROI/coverage thresholds or mounting parameters need calibration.")
        self.get_logger().info(f"Statuses: {status_text or 'none'}")
        self.get_logger().info(f"Reasons: {reason_text}")
        for name, passed in checks.items():
            if passed:
                self.get_logger().info(f"PASS: {name}")
            else:
                self.get_logger().error(f"FAIL: {name}")
        if not self.passed:
            self.get_logger().error(
                "Avia acceptance test FAILED; do not use LANDABLE for descent")
        else:
            self.get_logger().info("Avia acceptance test PASSED")


def main(args=None):
    rclpy.init(args=args)
    node = AviaLandingTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.finish()
    passed = node.passed
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
