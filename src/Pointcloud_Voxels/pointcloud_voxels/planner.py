#!/usr/bin/env python3
"""Plan and visualize a local 3D path from the LiDAR origin to a bbox target.

The target is the 3D bounding-box center of the points selected inside the
camera bbox.  All outputs stay in the selected point cloud's LiDAR frame; this
is a local planning demonstration, not a closed-loop flight controller.
"""

import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from .core import filter_points
from .planning import plan_path, point_bounds
from .transport import CloudCache, read_xyz, stamp_ns


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('ship_path_planner')
        defaults = {
            'target_cloud_topic': '/ship/points',
            'target_cloud_reliability': 'best_effort',
            'bbox_target_topic': '/ship/bbox_target',
            'bbox_target_reliability': 'best_effort',
            'obstacle_topic': '/livox/lidar/points',
            'pointcloud_reliability': 'best_effort',
            'path_topic': '/ship/path',
            'target_topic': '/ship/target',
            'marker_topic': '/ship/path_markers',
            'status_topic': '/ship/path_status',
            'lidar_frame': 'avia_frame',
            'start_point': [0.0, 0.0, 0.0],
            'target_offset': [0.0, 0.0, 0.0],
            'resolution': 1.0,
            'obstacle_inflation': 0.5,
            'target_exclusion_margin': 0.5,
            'start_clearance': 0.5,
            'goal_clearance': 0.5,
            'margin_xy': 5.0,
            'margin_z': 3.0,
            'max_grid_cells': 200000,
            'smooth_path': True,
            'sync_slop': 0.2,
            'obstacle_cache_size': 20,
            'timestamp_reset_threshold_sec': 30.0,
            'roi_min': [0.5, -10.0, -25.0],
            'roi_max': [80.0, 10.0, 25.0],
            'marker_lifetime': 0.0,
            'marker_size': 0.5,
            'path_width': 0.08,
            'label_height': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.cfg = p

        if p['target_cloud_reliability'] not in ('best_effort', 'reliable'):
            raise ValueError('target_cloud_reliability must be best_effort or reliable')
        if p['pointcloud_reliability'] not in ('best_effort', 'reliable'):
            raise ValueError('pointcloud_reliability must be best_effort or reliable')
        if p['bbox_target_reliability'] not in ('best_effort', 'reliable'):
            raise ValueError('bbox_target_reliability must be best_effort or reliable')
        for name in ('resolution', 'marker_size', 'path_width', 'label_height'):
            if not np.isfinite(float(p[name])) or float(p[name]) <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('obstacle_inflation', 'target_exclusion_margin',
                     'start_clearance', 'goal_clearance', 'margin_xy',
                     'margin_z', 'sync_slop', 'marker_lifetime'):
            if not np.isfinite(float(p[name])) or float(p[name]) < 0.0:
                raise ValueError(f'{name} must be finite and non-negative')
        if int(p['max_grid_cells']) <= 0 or int(p['obstacle_cache_size']) <= 0:
            raise ValueError('max_grid_cells and obstacle_cache_size must be positive')
        if not np.isfinite(float(p['timestamp_reset_threshold_sec'])) or float(p['timestamp_reset_threshold_sec']) <= 0:
            raise ValueError('timestamp_reset_threshold_sec must be finite and positive')
        self.start_point = np.asarray(p['start_point'], dtype=float)
        self.target_offset = np.asarray(p['target_offset'], dtype=float)
        if self.start_point.shape != (3,) or not np.isfinite(self.start_point).all():
            raise ValueError('start_point must contain three finite values')
        if self.target_offset.shape != (3,) or not np.isfinite(self.target_offset).all():
            raise ValueError('target_offset must contain three finite values')
        # Validate ROI and planning parameters before any sensor message.
        filter_points(np.empty((0, 3)), p['roi_min'], p['roi_max'])
        plan_path(self.start_point, self.start_point + np.array([1.0, 0.0, 0.0]), [],
                  p['resolution'], p['obstacle_inflation'], p['margin_xy'],
                  p['margin_z'], p['start_clearance'], p['goal_clearance'],
                  p['max_grid_cells'], p['smooth_path'])

        self.cache = CloudCache(int(p['obstacle_cache_size']),
                                float(p['timestamp_reset_threshold_sec']))
        self.last_target = None
        self.bbox_target = None
        self.last_status = 'waiting_for_target'
        self.last_log = 0.0

        self.path_pub = self.create_publisher(Path, p['path_topic'], 1)
        self.target_pub = self.create_publisher(PointStamped, p['target_topic'], 1)
        self.marker_pub = self.create_publisher(MarkerArray, p['marker_topic'], 1)
        self.status_pub = self.create_publisher(String, p['status_topic'], 1)
        self.target_sub = self.create_subscription(
            PointCloud2, p['target_cloud_topic'], self._target_callback,
            QoSProfile(depth=1, reliability=(
                ReliabilityPolicy.RELIABLE
                if p['target_cloud_reliability'] == 'reliable'
                else ReliabilityPolicy.BEST_EFFORT
            )),
        )
        self.bbox_target_sub = self.create_subscription(
            PointStamped, p['bbox_target_topic'], self._bbox_target_callback,
            QoSProfile(depth=1, reliability=(
                ReliabilityPolicy.RELIABLE
                if p['bbox_target_reliability'] == 'reliable'
                else ReliabilityPolicy.BEST_EFFORT
            )),
        )
        if str(p['obstacle_topic']).strip():
            self.obstacle_sub = self.create_subscription(
                PointCloud2, p['obstacle_topic'], self._obstacle_callback,
                QoSProfile(depth=3, reliability=(
                    ReliabilityPolicy.RELIABLE
                    if p['pointcloud_reliability'] == 'reliable'
                    else ReliabilityPolicy.BEST_EFFORT
                )),
            )
        self.get_logger().info(
            f"target={p['target_cloud_topic']} obstacle={p['obstacle_topic']} -> "
            f"{p['path_topic']}, {p['marker_topic']}"
        )

    def _warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def _obstacle_callback(self, message):
        try:
            self.cache.append(message)
            if self.last_target is not None and self.last_status == 'no_obstacle_cloud':
                self._plan_target(self.last_target)
        except (ValueError, TypeError, OverflowError) as error:
            self._warn(f'Obstacle cloud rejected: {error}')

    def _target_callback(self, message):
        self.last_target = message
        self._plan_target(message)

    def _bbox_target_callback(self, message):
        self.bbox_target = message
        if self.last_target is None:
            return
        try:
            if abs(stamp_ns(message) - stamp_ns(self.last_target)) <= (
                    float(self.cfg['sync_slop']) * 1e9):
                self._plan_target(self.last_target)
        except (ValueError, TypeError, OverflowError):
            pass

    def _matching_bbox_target(self, message):
        if self.bbox_target is None:
            return None
        frame = message.header.frame_id
        if (self.bbox_target.header.frame_id and frame
                and self.bbox_target.header.frame_id != frame):
            return None
        try:
            if abs(stamp_ns(self.bbox_target) - stamp_ns(message)) > (
                    float(self.cfg['sync_slop']) * 1e9):
                return None
        except (ValueError, TypeError, OverflowError):
            return None
        point = self.bbox_target.point
        return np.array([float(point.x), float(point.y), float(point.z)])

    def _plan_target(self, message):
        target = None
        bounds = None
        try:
            target_points = filter_points(
                read_xyz(message), self.cfg['roi_min'], self.cfg['roi_max'])
            if len(target_points) == 0:
                self._publish_failure(message.header, 'empty_target_cloud', None, None)
                return
            frame = message.header.frame_id
            if self.cfg['lidar_frame'] and frame != self.cfg['lidar_frame']:
                raise ValueError('target cloud frame must match lidar_frame')
            lower, upper, center = point_bounds(target_points)
            bbox_target = self._matching_bbox_target(message)
            target = (bbox_target if bbox_target is not None else center) + self.target_offset
            bounds = (lower, upper)
            obstacles = np.empty((0, 3), dtype=float)
            if str(self.cfg['obstacle_topic']).strip():
                obstacle_message = self.cache.nearest(
                    stamp_ns(message), float(self.cfg['sync_slop']))
                if obstacle_message is None:
                    self._publish_failure(
                        message.header, 'no_obstacle_cloud', target, (lower, upper))
                    return
                if (obstacle_message.header.frame_id and frame
                        and obstacle_message.header.frame_id != frame):
                    raise ValueError('obstacle and target cloud frames differ')
                obstacles = filter_points(
                    read_xyz(obstacle_message), self.cfg['roi_min'], self.cfg['roi_max'])
                margin = float(self.cfg['target_exclusion_margin'])
                keep = ~np.all(
                    (obstacles >= lower - margin) & (obstacles <= upper + margin),
                    axis=1,
                )
                obstacles = obstacles[keep]
            started = time.perf_counter()
            result = plan_path(
                self.start_point, target, obstacles,
                float(self.cfg['resolution']),
                float(self.cfg['obstacle_inflation']),
                float(self.cfg['margin_xy']),
                float(self.cfg['margin_z']),
                float(self.cfg['start_clearance']),
                float(self.cfg['goal_clearance']),
                int(self.cfg['max_grid_cells']),
                bool(self.cfg['smooth_path']),
            )
            elapsed = time.perf_counter() - started
            if result is None:
                self._publish_failure(message.header, 'no_path', target, (lower, upper))
                self._warn('No 3D path found for the current bbox target')
                return
            self._publish_success(
                message.header, target, lower, upper, result, elapsed)
        except (ValueError, TypeError, OverflowError) as error:
            self._publish_failure(message.header, f'blocked: {error}', target, bounds)
            self._warn(f'Path planning rejected: {error}')

    def _publish_failure(self, header, status, target, bounds):
        self.last_status = status
        self._publish_path(header, np.empty((0, 3)))
        if target is not None:
            self._publish_target(header, target)
        self._publish_markers(header, target, bounds, np.empty((0, 3)), status, 0.0)
        self._publish_status(status)
        self._warn(f'Path planner: {status}')

    def _publish_success(self, header, target, lower, upper, result, elapsed):
        self.last_status = 'ok'
        self._publish_path(header, result.waypoints)
        self._publish_target(header, target)
        self._publish_markers(
            header, target, (lower, upper), result.waypoints, 'ok', result.length)
        self._publish_status(
            f'ok waypoints={len(result.waypoints)} length={result.length:.2f}m')
        now = time.monotonic()
        if now - self.last_log >= 2.0:
            self.get_logger().info(
                f'path ok: target=({target[0]:.2f},{target[1]:.2f},{target[2]:.2f}) '
                f'points={len(result.waypoints)} length={result.length:.2f}m '
                f'visited={result.visited} plan={elapsed * 1000:.1f}ms'
            )
            self.last_log = now

    def _publish_path(self, header, waypoints):
        message = Path()
        message.header = header
        for point in waypoints:
            pose = PoseStamped()
            pose.header = header
            pose.pose.position = Point(
                x=float(point[0]), y=float(point[1]), z=float(point[2]))
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def _publish_target(self, header, target):
        message = PointStamped()
        message.header = header
        message.point = Point(
            x=float(target[0]), y=float(target[1]), z=float(target[2]))
        self.target_pub.publish(message)

    def _publish_status(self, status):
        self.status_pub.publish(String(data=status))

    def _publish_markers(self, header, target, bounds, waypoints, status, length):
        p = self.cfg
        markers = [Marker(header=header, action=Marker.DELETEALL)]
        lifetime_ns = int(float(p['marker_lifetime']) * 1e9)
        lifetime = Duration(sec=lifetime_ns // 10**9, nanosec=lifetime_ns % 10**9)

        def marker(namespace, marker_id, marker_type, rgba, scale):
            item = Marker(header=header, ns=namespace, id=marker_id,
                          type=marker_type, action=Marker.ADD)
            item.pose.orientation.w = 1.0
            item.scale.x, item.scale.y, item.scale.z = scale
            item.color = ColorRGBA(
                r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
            item.lifetime = lifetime
            return item

        start = marker('path_start', 0, Marker.SPHERE, (0.1, 1.0, 0.2, 1.0),
                       (float(p['marker_size']),) * 3)
        start.pose.position = Point(
            x=float(self.start_point[0]), y=float(self.start_point[1]),
            z=float(self.start_point[2]))
        markers.append(start)
        if target is not None:
            target_marker = marker(
                'path_target', 0, Marker.SPHERE, (1.0, 0.2, 0.1, 1.0),
                (float(p['marker_size']),) * 3)
            target_marker.pose.position = Point(
                x=float(target[0]), y=float(target[1]), z=float(target[2]))
            markers.append(target_marker)
        if bounds is not None:
            lower, upper = bounds
            corners = np.array([
                [lower[0], lower[1], lower[2]], [upper[0], lower[1], lower[2]],
                [upper[0], upper[1], lower[2]], [lower[0], upper[1], lower[2]],
                [lower[0], lower[1], upper[2]], [upper[0], lower[1], upper[2]],
                [upper[0], upper[1], upper[2]], [lower[0], upper[1], upper[2]],
            ])
            edges = ((0, 1), (1, 2), (2, 3), (3, 0),
                     (4, 5), (5, 6), (6, 7), (7, 4),
                     (0, 4), (1, 5), (2, 6), (3, 7))
            bbox = marker('path_bbox', 0, Marker.LINE_LIST, (1.0, 0.85, 0.1, 1.0),
                          (0.06, 0.06, 0.06))
            bbox.points = [Point(x=float(corners[index][0]),
                                 y=float(corners[index][1]),
                                 z=float(corners[index][2]))
                           for edge in edges for index in edge]
            markers.append(bbox)
        if len(waypoints):
            path_marker = marker('path_line', 0, Marker.LINE_STRIP,
                                 (0.0, 0.9, 1.0, 1.0),
                                 (float(p['path_width']),) * 3)
            path_marker.points = [Point(x=float(point[0]), y=float(point[1]),
                                        z=float(point[2])) for point in waypoints]
            markers.append(path_marker)
        label = marker('path_label', 0, Marker.TEXT_VIEW_FACING,
                       (1.0, 1.0, 1.0, 1.0), (0.0, 0.0, 0.5))
        anchor = target if target is not None else self.start_point
        label.pose.position = Point(
            x=float(anchor[0]), y=float(anchor[1]),
            z=float(anchor[2] + float(p['label_height'])))
        label.text = f'PATH {status} len={length:.1f}m'
        markers.append(label)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PathPlannerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
