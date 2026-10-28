"""Native ROS 2/DDS checks. Enable with UAL_RUN_ROS_TESTS=1 after building."""

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
    from geometry_msgs.msg import Point32, PointStamped
    from ins_seg.msg import (
        Detection, DetectionResult, SegInfo, SegmentationResult,
    )
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Image, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from visualization_msgs.msg import Marker, MarkerArray
    from pointcloud_voxels.node import MaskVoxelsNode


@unittest.skipUnless(ENABLED, 'Requires installed Jazzy packages and UAL_RUN_ROS_TESTS=1')
class NativeVoxelNodeTests(unittest.TestCase):
    def start(self, confirmed=True, approximate=False, reset_threshold=30.0,
              mode='mask', target_class=''):
        prefix = '/voxel_test_' + uuid.uuid4().hex
        params = {
            'selection_mode': mode,
            'target_class_name': target_class,
            'pointcloud_topic': prefix+'/cloud', 'segmentation_topic': prefix+'/mask',
            'detection_topic': prefix+'/detections',
            'bbox_topic': prefix+'/bboxes',
            'bbox_target_topic': prefix+'/bbox_target',
            'marker_topic': prefix+'/voxels', 'selected_cloud_topic': prefix+'/selected',
            'camera_info_topic': '', 'image_width': 8, 'image_height': 8,
            'camera_optical_frame': 'test_optical', 'calibration_confirmed': confirmed,
            'allow_approximate_calibration': approximate,
            'timestamp_reset_threshold_sec': reset_threshold,
            'camera_matrix': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        }
        args = ['--ros-args']
        for key, value in params.items():
            encoded = "''" if value == '' else yaml.safe_dump(value, default_flow_style=True).replace('\n...', '').strip()
            args += ['-p', f'{key}:={encoded}']
        rclpy.init(args=args)
        self.algorithm = MaskVoxelsNode()
        self.probe = rclpy.create_node('voxel_test_probe_'+uuid.uuid4().hex)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.algorithm)
        self.executor.add_node(self.probe)
        self.clouds, self.markers = [], []
        self.bboxes, self.bbox_targets = [], []
        self.subs = [
            self.probe.create_subscription(PointCloud2, prefix+'/selected', self.clouds.append, 10),
            self.probe.create_subscription(MarkerArray, prefix+'/voxels', self.markers.append, 10),
            self.probe.create_subscription(MarkerArray, prefix+'/bboxes', self.bboxes.append, 10),
            self.probe.create_subscription(PointStamped, prefix+'/bbox_target', self.bbox_targets.append, 10),
        ]
        self.cloud_pub = self.probe.create_publisher(PointCloud2, prefix+'/cloud', 10)
        self.mask_pub = self.probe.create_publisher(SegmentationResult, prefix+'/mask', 10)
        self.detection_pub = self.probe.create_publisher(
            DetectionResult, prefix+'/detections', 10
        )
        self.wait(
            lambda: self.cloud_pub.get_subscription_count()
            and (self.mask_pub.get_subscription_count()
                 if mode == 'mask' else self.detection_pub.get_subscription_count())
        )

    def tearDown(self):
        if hasattr(self, 'executor'):
            self.executor.shutdown()
            self.algorithm.destroy_node()
            self.probe.destroy_node()
        if ENABLED and rclpy.ok():
            rclpy.shutdown()

    def wait(self, predicate, timeout=4):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
            if predicate():
                return
        self.fail('Timed out waiting for native ROS messages')

    def cloud(self, points, seconds=100):
        return point_cloud2.create_cloud_xyz32(
            Header(stamp=Time(sec=seconds), frame_id='avia_frame'),
            np.asarray(points, dtype=np.float32).reshape(-1, 3),
        )

    def mask(self, pixels=None, seconds=100):
        labels = np.zeros((8, 8), dtype=np.int32)
        for row, column in (pixels or []):
            labels[row, column] = 7
        header = Header(stamp=Time(sec=seconds), frame_id='test_optical')
        image = Image(header=header, width=8, height=8, encoding='32SC1',
                      is_bigendian=0, step=32, data=labels.tobytes())
        return SegmentationResult(header=header, instance_id_map=image,
                                  instances=[SegInfo(track_id=7, class_name='ship', confidence=1.0)] if pixels else [])

    def detections(self, polygon, seconds=100, track_id=7, class_name='ship'):
        header = Header(stamp=Time(sec=seconds), frame_id='test_optical')
        item = Detection(
            track_id=track_id, class_id=0, class_name=class_name,
            confidence=1.0, oriented=False,
        )
        item.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in polygon]
        return DetectionResult(
            header=header, image_width=8, image_height=8, detections=[item],
        )

    def test_delayed_mask_uses_old_cloud_and_preserves_holes(self):
        self.start()
        points = [[1, 1, 1], [2, 1, 1], [3, 1, 1]]
        self.cloud_pub.publish(self.cloud(points))
        self.cloud_pub.publish(self.cloud([[5, 1, 1]], seconds=105))
        self.wait(lambda: len(self.algorithm.cache.messages) == 2)
        self.mask_pub.publish(self.mask([(1, 1), (1, 3)]))
        self.wait(lambda: self.clouds and self.markers)
        selected = self.clouds[-1]
        self.assertEqual(selected.width, 2)
        self.assertEqual(selected.header.stamp.sec, 100)
        self.assertEqual(selected.header.frame_id, 'avia_frame')
        np.testing.assert_allclose(point_cloud2.read_points_numpy(selected), [points[0], points[2]])

    def test_mask_before_cloud_waits_for_matching_acquisition(self):
        self.start()
        self.mask_pub.publish(self.mask([(1, 1)]))
        self.wait(lambda: len(self.algorithm.pending) == 1)
        self.assertEqual(self.clouds, [])
        self.cloud_pub.publish(self.cloud([[1, 1, 1]]))
        self.wait(lambda: self.clouds)
        self.assertEqual(self.clouds[-1].width, 1)

    def test_unconfirmed_geometry_does_not_publish_fake_selection(self):
        self.start(confirmed=False)
        self.cloud_pub.publish(self.cloud([[1, 1, 1]]))
        self.mask_pub.publish(self.mask([(1, 1)]))
        self.wait(lambda: self.clouds and self.markers)
        self.assertEqual(self.clouds[-1].width, 0)
        self.assertEqual([m.action for m in self.markers[-1].markers], [Marker.DELETEALL])

    def test_explicit_approximate_opt_in_keeps_confirmed_false(self):
        self.start(confirmed=False, approximate=True)
        self.cloud_pub.publish(self.cloud([[1, 1, 1]]))
        self.mask_pub.publish(self.mask([(1, 1)]))
        self.wait(lambda: self.clouds)
        self.assertFalse(self.algorithm.cfg['calibration_confirmed'])
        self.assertEqual(self.clouds[-1].width, 1)

    def test_exact_four_empty_faces_are_green_in_native_markers(self):
        self.start()
        self.cloud_pub.publish(self.cloud([[x, 0.25, 2.25] for x in (0.75, 1.25, 1.75, 2.25, 2.75)]))
        self.mask_pub.publish(self.mask([(0, 0), (0, 1)]))
        self.wait(lambda: self.markers)
        by_ns = {item.ns: item for item in self.markers[-1].markers}
        self.assertEqual(len(by_ns['ship_voxel_edge'].points), 3)
        self.assertEqual(len(by_ns['ship_voxel_corner'].points), 2)
        self.assertEqual(len(by_ns['ship_voxel_plane'].points), 0)
        self.assertAlmostEqual(by_ns['ship_voxel_plane'].color.a, 0.22)
        self.assertAlmostEqual(by_ns['ship_voxel_edge'].color.a, 1.0)

    def test_empty_mask_clears_previous_selection(self):
        self.start()
        self.cloud_pub.publish(self.cloud([[1, 1, 1]]))
        self.mask_pub.publish(self.mask([(1, 1)]))
        self.wait(lambda: self.clouds)
        self.cloud_pub.publish(self.cloud([[1, 1, 1]], seconds=101))
        self.mask_pub.publish(self.mask(seconds=101))
        self.wait(lambda: len(self.clouds) >= 2 and len(self.markers) >= 2)
        self.assertEqual(self.clouds[-1].width, 0)
        self.assertEqual([m.action for m in self.markers[-1].markers], [Marker.DELETEALL])

    def test_detection_polygon_selects_only_inside_points(self):
        self.start(mode='detection')
        points = [[-1, 1, 1], [1, 1, 1], [2, 1, 1], [3, 1, 1]]
        self.cloud_pub.publish(self.cloud(points))
        self.wait(lambda: len(self.algorithm.cache.messages) == 1)
        # The polygon reaches outside the image on the left; the point at
        # pixel u=-1 must still be excluded, while u=1 and u=2 are selected.
        self.detection_pub.publish(self.detections(
            [[-2, 0.5], [-2, 2.5], [2.5, 2.5], [2.5, 0.5]]
        ))
        self.wait(lambda: self.clouds and self.markers and self.bboxes and self.bbox_targets)
        selected = point_cloud2.read_points_numpy(self.clouds[-1])
        np.testing.assert_allclose(selected, [points[1], points[2]])
        self.assertEqual(selected.shape[1], 3)
        self.assertIn('ship_point_bbox',
                      {marker.ns for marker in self.bboxes[-1].markers})
        self.assertAlmostEqual(self.bbox_targets[-1].point.z, 1.0)

    def test_oriented_detection_uses_polygon_not_outer_bbox(self):
        self.start(mode='detection')
        points = [[2, 2, 1], [3, 1, 1], [2, 3, 1]]
        self.cloud_pub.publish(self.cloud(points))
        self.wait(lambda: len(self.algorithm.cache.messages) == 1)
        # Diamond between (2, 1) and (5, 4): (3, 1) is inside the outer bbox
        # but outside the oriented polygon.
        self.detection_pub.publish(self.detections(
            [[2, 1], [5, 2.5], [2, 4], [-1, 2.5]], track_id=4
        ))
        self.wait(lambda: self.clouds)
        selected = point_cloud2.read_points_numpy(self.clouds[-1])
        np.testing.assert_allclose(selected, [points[0], points[2]])

    def test_detection_target_class_filter_can_clear_selection(self):
        self.start(mode='detection', target_class='ship')
        self.cloud_pub.publish(self.cloud([[1, 1, 1]]))
        self.wait(lambda: len(self.algorithm.cache.messages) == 1)
        self.detection_pub.publish(self.detections(
            [[0, 0], [3, 0], [3, 3], [0, 3]], class_name='car'
        ))
        self.wait(lambda: self.clouds)
        self.assertEqual(self.clouds[-1].width, 0)

    def test_short_loop_reset_accepts_repeated_mask_acquisition(self):
        self.start(reset_threshold=0.5)
        self.cloud_pub.publish(self.cloud([[1, 1, 1]], seconds=100))
        self.mask_pub.publish(self.mask([(1, 1)], seconds=100))
        self.wait(lambda: self.clouds)
        self.cloud_pub.publish(self.cloud([[1, 1, 1]], seconds=102))
        self.wait(lambda: len(self.algorithm.cache.messages) == 2)
        self.cloud_pub.publish(self.cloud([[1, 1, 1]], seconds=100))
        self.wait(lambda: len(self.algorithm.cache.messages) == 1 and self.algorithm.last_processed_ns is None)
        self.mask_pub.publish(self.mask([(1, 1)], seconds=100))
        self.wait(lambda: self.algorithm.last_processed_ns is not None)
        self.wait(lambda: len(self.clouds) >= 3)
        self.assertEqual(self.clouds[-1].width, 1)


if __name__ == '__main__':
    unittest.main()
