"""Point-cloud bbox generation and persistence tests; no ROS required."""

from pathlib import Path
import sys
import unittest

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'Pointcloud_Voxels'))
from pointcloud_voxels.bbox import (  # noqa: E402
    BBoxObservation, PointCloudBBox, PointCloudBBoxTracker, bbox_volume,
    pointcloud_bbox,
)


def make_bbox(center=(0, 0, 0), dimensions=(4, 2, 1)):
    center = np.asarray(center, dtype=float)
    dimensions = np.asarray(dimensions, dtype=float)
    signs = np.array([
        [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
        [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1],
    ], dtype=float)
    corners = center - dimensions / 2 + signs * dimensions
    return PointCloudBBox(center, dimensions, corners, 100, 'yz')


def observation(confidence, center=(0, 0, 0), dimensions=(4, 2, 1), class_id=1):
    return BBoxObservation(class_id, 'ship', confidence,
                           make_bbox(center, dimensions))


class BBoxGenerationTests(unittest.TestCase):
    def test_dominant_cluster_gets_oriented_3d_bbox(self):
        y, z = np.meshgrid(np.arange(0.0, 5.1, 0.5), np.arange(0.0, 3.1, 0.5))
        large = np.column_stack((
            np.full(y.size, 10.0), y.ravel(), z.ravel()))
        y2, z2 = np.meshgrid(np.arange(10.0, 11.1, 0.5), np.arange(0.0, 1.1, 0.5))
        small = np.column_stack((
            np.full(y2.size, 15.0), y2.ravel(), z2.ravel()))
        bbox = pointcloud_bbox(
            np.vstack((large, small)), projection_plane='yz', voxel_size=0.25,
            cluster_tolerance=0.8, min_cluster_points=10, box_type='axis_aligned',
            min_box_height=0.2,
        )
        self.assertIsNotNone(bbox)
        self.assertAlmostEqual(bbox.center[0], 10.0, places=6)
        self.assertGreater(bbox.dimensions[0], bbox.dimensions[1])
        self.assertAlmostEqual(bbox.dimensions[0], 5.0, places=6)
        self.assertAlmostEqual(bbox.dimensions[2], 0.2, places=6)
        self.assertEqual(bbox.point_count, len(large))

    def test_fallback_aabb_and_empty_input(self):
        points = np.array([[1, 2, 3], [3, 4, 5]], dtype=float)
        bbox = pointcloud_bbox(points, min_cluster_points=10, min_box_height=0.1)
        self.assertIsNotNone(bbox)
        np.testing.assert_allclose(bbox.center, [2, 3, 4])
        np.testing.assert_allclose(bbox.dimensions, [2, 2, 2])
        self.assertIsNone(pointcloud_bbox(np.empty((0, 3))))

    def test_invalid_bbox_parameters_are_rejected(self):
        points = np.array([[1, 2, 3]], dtype=float)
        for kwargs in ({'projection_plane': 'ab'}, {'box_type': 'bad'},
                       {'voxel_size': 0}, {'min_box_size': 0},
                       {'min_cluster_points': 0}, {'max_clusters': 0},
                       {'cluster_selection': 'bad'}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                pointcloud_bbox(points, **kwargs)


class PointCloudBBoxTrackerTests(unittest.TestCase):
    def test_lower_confidence_keeps_old_bbox_and_id(self):
        tracker = PointCloudBBoxTracker(timeout_sec=2.0)
        first = tracker.update([observation(0.9)], 0.0)
        self.assertEqual([item.track_id for item in first], [1])
        second = tracker.update([observation(0.5, center=(0.2, 0, 0))], 0.1)
        self.assertEqual([item.track_id for item in second], [1])
        self.assertEqual(second[0].confidence, 0.9)
        np.testing.assert_allclose(second[0].center, first[0].center)
        self.assertGreater(bbox_volume(make_bbox()), 0.0)
        self.assertEqual([item.track_id for item in tracker.update([], 1.0)], [1])
        self.assertEqual(tracker.update([], 2.6), [])

    def test_higher_confidence_updates_3d_geometry(self):
        tracker = PointCloudBBoxTracker()
        tracker.update([observation(0.9)], 0.0)
        updated = tracker.update([observation(0.95, center=(0.5, 0, 0))], 0.1)
        self.assertEqual(updated[0].track_id, 1)
        self.assertAlmostEqual(updated[0].confidence, 0.95)
        self.assertAlmostEqual(updated[0].center[0], 0.5)

    def test_new_area_and_class_get_new_ids_without_duplicate(self):
        tracker = PointCloudBBoxTracker()
        tracker.update([observation(0.9)], 0.0)
        result = tracker.update([
            observation(0.8, center=(20, 0, 0)),
            observation(0.7, class_id=2),
            observation(0.5, center=(0.1, 0, 0)),
        ], 0.1)
        self.assertEqual([item.track_id for item in result], [1, 2, 3])
        self.assertEqual(result[0].confidence, 0.9)
        self.assertEqual(result[1].center[0], 20.0)
        self.assertEqual(result[2].class_id, 2)

    def test_timeout_max_tracks_and_invalid_configuration(self):
        tracker = PointCloudBBoxTracker(timeout_sec=0.5)
        tracker.update([observation(0.9)], 0.0)
        self.assertEqual(tracker.update([], 0.6), [])
        limited = PointCloudBBoxTracker(max_tracks=1)
        result = limited.update([
            observation(0.9), observation(0.5, center=(20, 0, 0))], 0.0)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].confidence, 0.9)
        for kwargs in ({'match_iou': -1}, {'center_ratio': -1},
                       {'confidence_margin': -1}, {'timeout_sec': -1},
                       {'max_tracks': 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                PointCloudBBoxTracker(**kwargs)


if __name__ == '__main__':
    unittest.main()
