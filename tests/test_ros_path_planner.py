"""Native ROS 2 checks for the bbox-target A* path planner."""

import os
import time
import unittest
import uuid

import numpy as np
import yaml

ENABLED = os.environ.get('UAL_RUN_ROS_TESTS') == '1'
if ENABLED:
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Path
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String
    from visualization_msgs.msg import MarkerArray
    from pointcloud_voxels.planner import PathPlannerNode


@unittest.skipUnless(ENABLED, 'Requires installed Jazzy packages and UAL_RUN_ROS_TESTS=1')
class NativePathPlannerTests(unittest.TestCase):
    def start(self, resolution=0.5, inflation=0.2, margin=2.0):
        prefix = '/planner_test_' + uuid.uuid4().hex
        params = {
            'target_cloud_topic': prefix+'/target',
            'bbox_target_topic': prefix+'/bbox_target',
            'obstacle_topic': prefix+'/obstacle',
            'path_topic': prefix+'/path',
            'target_topic': prefix+'/point',
            'marker_topic': prefix+'/markers',
            'status_topic': prefix+'/status',
            'lidar_frame': 'avia_frame',
            'start_point': [0.0, 0.0, 0.0],
            'target_offset': [0.0, 0.0, 0.0],
            'resolution': resolution,
            'obstacle_inflation': inflation,
            'target_exclusion_margin': 0.3,
            'start_clearance': 0.3,
            'goal_clearance': 0.3,
            'margin_xy': margin,
            'margin_z': margin,
            'max_grid_cells': 100000,
            'sync_slop': 0.2,
        }
        args = ['--ros-args']
        for key, value in params.items():
            encoded = yaml.safe_dump(value, default_flow_style=True).replace('\n...', '').strip()
            args += ['-p', f'{key}:={encoded}']
        rclpy.init(args=args)
        self.planner = PathPlannerNode()
        self.probe = rclpy.create_node('planner_test_probe_'+uuid.uuid4().hex)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.planner)
        self.executor.add_node(self.probe)
        self.paths, self.markers, self.statuses, self.points = [], [], [], []
        self.subs = [
            self.probe.create_subscription(Path, prefix+'/path', self.paths.append, 10),
            self.probe.create_subscription(MarkerArray, prefix+'/markers', self.markers.append, 10),
            self.probe.create_subscription(String, prefix+'/status', self.statuses.append, 10),
            self.probe.create_subscription(PointStamped, prefix+'/point', self.points.append, 10),
        ]
        self.target_pub = self.probe.create_publisher(PointCloud2, prefix+'/target', 10)
        self.obstacle_pub = self.probe.create_publisher(PointCloud2, prefix+'/obstacle', 10)
        self.bbox_target_pub = self.probe.create_publisher(
            PointStamped, prefix+'/bbox_target', 10)
        self.wait(lambda: self.target_pub.get_subscription_count()
                  and self.obstacle_pub.get_subscription_count()
                  and self.bbox_target_pub.get_subscription_count())

    def tearDown(self):
        if hasattr(self, 'executor'):
            self.executor.shutdown()
            self.planner.destroy_node()
            self.probe.destroy_node()
        if ENABLED and rclpy.ok():
            rclpy.shutdown()

    def wait(self, predicate, timeout=5):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
            if predicate():
                return
        self.fail('Timed out waiting for planner messages')

    def cloud(self, points, seconds=100):
        return point_cloud2.create_cloud_xyz32(
            Header(stamp=Time(sec=seconds), frame_id='avia_frame'),
            np.asarray(points, dtype=np.float32).reshape(-1, 3),
        )

    def test_path_uses_bbox_center_and_visualizes_it(self):
        self.start()
        target_points = [[4.5, 0.0, 1.0], [5.5, 0.0, 1.0], [5.0, 0.5, 1.0]]
        self.obstacle_pub.publish(self.cloud([[20.0, 0.0, 0.0]]))
        self.target_pub.publish(self.cloud(target_points))
        self.wait(lambda: self.paths and len(self.paths[-1].poses) >= 2
                  and self.markers and self.statuses
                  and self.statuses[-1].data.startswith('ok') and self.points)
        path = self.paths[-1]
        self.assertEqual(path.header.frame_id, 'avia_frame')
        self.assertGreaterEqual(len(path.poses), 2)
        start = path.poses[0].pose.position
        end = path.poses[-1].pose.position
        self.assertAlmostEqual(start.x, 0.0)
        self.assertAlmostEqual(start.y, 0.0)
        self.assertAlmostEqual(start.z, 0.0)
        self.assertAlmostEqual(end.x, 5.0)
        self.assertAlmostEqual(end.y, 0.25)
        self.assertAlmostEqual(end.z, 1.0)
        namespaces = {marker.ns for marker in self.markers[-1].markers}
        self.assertIn('path_target', namespaces)
        self.assertIn('path_bbox', namespaces)
        self.assertIn('path_line', namespaces)
        self.assertTrue(self.statuses[-1].data.startswith('ok'))
        self.assertAlmostEqual(self.points[-1].point.x, 5.0)

    def test_explicit_point_bbox_target_overrides_point_aabb(self):
        self.start()
        self.obstacle_pub.publish(self.cloud([[20.0, 0.0, 0.0]]))
        bbox = PointStamped(
            header=Header(stamp=Time(sec=100), frame_id='avia_frame'))
        bbox.point.x, bbox.point.y, bbox.point.z = 7.0, 2.0, 3.0
        self.bbox_target_pub.publish(bbox)
        self.target_pub.publish(self.cloud(
            [[4.5, 0.0, 1.0], [5.5, 0.0, 1.0], [5.0, 0.5, 1.0]]))
        self.wait(lambda: self.paths and len(self.paths[-1].poses) >= 2
                  and self.statuses and self.statuses[-1].data.startswith('ok'))
        end = self.paths[-1].poses[-1].pose.position
        self.assertAlmostEqual(end.x, 7.0)
        self.assertAlmostEqual(end.y, 2.0)
        self.assertAlmostEqual(end.z, 3.0)

    def test_target_without_obstacle_waits_then_replans(self):
        self.start()
        self.target_pub.publish(self.cloud([[5.0, 0.0, 1.0]]))
        self.wait(lambda: self.statuses and self.statuses[-1].data == 'no_obstacle_cloud')
        self.assertEqual(len(self.paths[-1].poses), 0)
        self.obstacle_pub.publish(self.cloud([[20.0, 0.0, 0.0]]))
        self.wait(lambda: self.paths and len(self.paths[-1].poses) >= 2
                  and self.statuses[-1].data.startswith('ok'))
        self.assertGreaterEqual(len(self.paths[-1].poses), 2)

    def test_wall_without_detour_publishes_empty_path(self):
        self.start(resolution=0.5, inflation=0.5, margin=1.0)
        wall = [[2.5, y, z] for y in np.arange(-2.0, 2.01, 0.25)
                for z in np.arange(-2.0, 2.01, 0.25)]
        self.obstacle_pub.publish(self.cloud(wall))
        self.target_pub.publish(self.cloud([[5.0, 0.0, 0.0]]))
        self.wait(lambda: self.statuses and self.statuses[-1].data == 'no_path')
        self.assertEqual(len(self.paths[-1].poses), 0)
        self.assertTrue(self.markers)
        namespaces = {marker.ns for marker in self.markers[-1].markers}
        self.assertIn('path_target', namespaces)
        self.assertIn('path_bbox', namespaces)

    def test_empty_target_clears_previous_path(self):
        self.start()
        self.obstacle_pub.publish(self.cloud([[20.0, 0.0, 0.0]]))
        self.target_pub.publish(self.cloud([[5.0, 0.0, 1.0]]))
        self.wait(lambda: self.paths and len(self.paths[-1].poses) >= 2)
        self.target_pub.publish(self.cloud(np.empty((0, 3)), seconds=101))
        self.wait(lambda: self.statuses and self.statuses[-1].data == 'empty_target_cloud')
        self.assertEqual(len(self.paths[-1].poses), 0)


if __name__ == '__main__':
    unittest.main()
