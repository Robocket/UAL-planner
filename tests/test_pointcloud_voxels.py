"""Numerical regression checks; no ROS, camera device or model is required."""

from pathlib import Path
import sys
import unittest

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'Pointcloud_Voxels'))
from pointcloud_voxels.core import (  # noqa: E402
    filter_points, project_instance_ids, project_pixels, select_bbox_points,
    validate_transform, voxelize,
)
from pointcloud_voxels.detection import (  # noqa: E402
    DetectionBox, points_in_polygon, select_detection_groups,
)


class MaskProjectionTests(unittest.TestCase):
    def setUp(self):
        self.mask = np.zeros((7, 7), dtype=np.int32)
        self.intrinsics = np.array([[2, 0, 3], [0, 2, 3], [0, 0, 1]], dtype=float)

    def project(self, points, **kwargs):
        return project_instance_ids(points, self.mask, self.intrinsics, np.eye(4), **kwargs)

    def test_exact_mask_holes_do_not_become_bbox_foreground(self):
        self.mask[2:5, 2:5] = 8
        self.mask[3, 3] = 0
        ids = self.project([[0, 0, 2], [1, 0, 2], [0, 1, 2]])
        np.testing.assert_array_equal(ids, [0, 8, 8])

    def test_projection_rejects_background_behind_outside_and_nonfinite(self):
        self.mask[3, 3] = 9
        ids = self.project([[0, 0, 2], [0, 0, -2], [100, 0, 2], [np.nan, 0, 2], [0, 0, 0]])
        np.testing.assert_array_equal(ids, [9, 0, 0, 0, 0])

    def test_camera_transform_has_source_to_optical_direction(self):
        self.mask[3, 4] = 3
        transform = np.array([[0, -1, 0, 1], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        ids = project_instance_ids([[0, 0, 2], [0, 1, 2]], self.mask, self.intrinsics, transform)
        np.testing.assert_array_equal(ids, [3, 0])

    def test_mask_boundaries_do_not_wrap_around(self):
        self.mask[:, 0], self.mask[:, -1] = 4, 5
        ids = self.project([[-3, 0, 2], [3, 0, 2], [-4, 0, 2], [4, 0, 2]])
        np.testing.assert_array_equal(ids, [4, 5, 0, 0])

    def test_plumb_bob_radial_and_tangential_distortion(self):
        self.mask[3, 6] = 7
        np.testing.assert_array_equal(self.project([[1, 0, 1]], distortion=[0.5, 0, 0, 0, 0]), [7])
        self.mask[5, 5] = 6
        # x=y=.5, p1=p2=.5: distorted x=y=1.25 -> pixel 5.5 (ties to 6).
        self.mask[6, 6] = 10
        np.testing.assert_array_equal(self.project([[0.5, 0.5, 1]], distortion=[0, 0, 0.5, 0.5]), [10])

    def test_rational_distortion_uses_denominator(self):
        self.mask[3, 4] = 12
        np.testing.assert_array_equal(self.project([[1, 0, 1]], distortion=[0, 0, 0, 0, 0, 1, 0, 0], distortion_model='rational_polynomial'), [12])

    def test_invalid_mask_intrinsics_and_distortion_are_rejected(self):
        for mask in (np.zeros((2, 2), dtype=float), np.array([[-1]]), np.array([[2**31]], dtype=np.uint32), np.array([], dtype=np.int32)):
            with self.subTest(mask=mask), self.assertRaises(ValueError):
                project_instance_ids([], mask, self.intrinsics, np.eye(4))
        invalid = self.intrinsics.copy()
        invalid[0, 0] = -1
        with self.assertRaises(ValueError):
            project_instance_ids([], self.mask, invalid, np.eye(4))
        for kwargs in ({'distortion': [0, 0]}, {'distortion': [np.nan]*5}, {'distortion_model': 'equidistant'}, {'near': 2, 'far': 1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.project([], **kwargs)

    def test_nonrigid_transform_is_not_accepted_as_calibration(self):
        for transform in (np.diag([2, 1, 1, 1]), np.diag([-1, 1, 1, 1]), np.zeros((4, 4))):
            with self.assertRaises(ValueError):
                validate_transform(transform)
        np.testing.assert_array_equal(validate_transform(np.eye(4).ravel()), np.eye(4))

    def test_continuous_projection_returns_pixels_and_nan_for_invalid(self):
        u, v = project_pixels(
            [[0, 0, 2], [1, 0, 2], [0, 0, -2], [np.nan, 0, 2]],
            self.intrinsics, np.eye(4),
        )
        np.testing.assert_allclose(u[:2], [3, 4])
        np.testing.assert_allclose(v[:2], [3, 3])
        self.assertTrue(np.isnan(u[2:]).all() and np.isnan(v[2:]).all())


class VoxelTests(unittest.TestCase):
    def test_exact_four_empty_faces_are_edges(self):
        grid = voxelize([[x + 0.1, 0.1, 0.1] for x in range(5)], voxel_size=1)
        np.testing.assert_array_equal(grid.empty_faces, [5, 4, 4, 4, 5])
        np.testing.assert_array_equal(grid.classes, [2, 1, 1, 1, 2])

    def test_single_layer_plane_and_corners_follow_current_thresholds(self):
        grid = voxelize([[x+0.1, y+0.1, 0.1] for x in range(5) for y in range(5)], voxel_size=1)
        self.assertEqual(np.count_nonzero(grid.classes == 1), 4)  # Four empty faces.
        self.assertEqual(np.count_nonzero(grid.classes == 2), 0)
        self.assertEqual(np.count_nonzero(grid.classes == 0), 21)

    def test_full_occupancy_is_used_before_rendering_subsets(self):
        points = [[x+0.1, y+0.1, z+0.1] for x in range(3) for y in range(3) for z in range(3)]
        grid = voxelize(points, voxel_size=1)
        center = np.all(grid.indices == [1, 1, 1], axis=1)
        self.assertEqual(grid.empty_faces[center].item(), 0)
        self.assertEqual(grid.classes[center].item(), 0)
        self.assertTrue(np.all(grid.classes == 0))

    def test_minimum_points_controls_occupied_neighbors(self):
        grid = voxelize([[0.1, 0, 0], [0.2, 0, 0], [1.1, 0, 0]], voxel_size=1, min_points=2)
        np.testing.assert_array_equal(grid.point_counts, [2])
        np.testing.assert_array_equal(grid.empty_faces, [6])
        np.testing.assert_array_equal(grid.classes, [2])

    def test_negative_coordinates_use_floor_and_centers(self):
        grid = voxelize([[-0.1, 0.1, 0.1], [0.1, 0.1, 0.1]], voxel_size=0.5)
        np.testing.assert_array_equal(grid.indices, [[-1, 0, 0], [0, 0, 0]])
        np.testing.assert_allclose(grid.centers, [[-0.25, 0.25, 0.25], [0.25, 0.25, 0.25]])

    def test_empty_grid_has_consistent_array_shapes(self):
        grid = voxelize([])
        self.assertEqual(grid.indices.shape, (0, 3))
        self.assertEqual(grid.centers.shape, (0, 3))
        self.assertEqual(grid.empty_faces.shape, (0,))

    def test_invalid_parameters_and_nonfinite_points_fail(self):
        for kwargs in ({'voxel_size': 0}, {'voxel_size': float('inf')}, {'min_points': 0}, {'edge_empty_faces': 5, 'corner_min_empty_faces': 5}, {'corner_min_empty_faces': 7}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                voxelize([], **kwargs)
        with self.assertRaises(ValueError):
            voxelize([[np.nan, 0, 0]])

    def test_roi_removes_invalid_zero_and_outside_points(self):
        points = [[0, 0, 0], [1, 2, 3], [1, 2, 4], [np.nan, 1, 1]]
        np.testing.assert_array_equal(filter_points(points, [0, 0, 0], [1, 2, 3]), [[1, 2, 3]])


class CameraDetectionTests(unittest.TestCase):
    def test_oriented_polygon_selects_points_and_excludes_holes(self):
        # A diamond whose image area covers the centre but not the corners.
        polygon = np.array([[50, 10], [90, 50], [50, 90], [10, 50]], dtype=float)
        pixels = np.array([[50, 50], [50, 10], [15, 15], [np.nan, 50]])
        np.testing.assert_array_equal(
            points_in_polygon(pixels, polygon), [True, True, False, False]
        )
        points = np.array([[1, 1, 1], [2, 2, 2], [3, 3, 3], [4, 4, 4]], dtype=float)
        box = DetectionBox(3, 'ship', 0.9, polygon, oriented=True)
        groups = select_detection_groups(points, pixels, [box])
        np.testing.assert_array_equal(groups[3], points[:2])

    def test_largest_polygon_with_points_wins_and_overlaps_stay_separate(self):
        points = np.array([[1, 1, 1], [2, 2, 2], [3, 3, 3]], dtype=float)
        pixels = np.array([[1, 1], [5, 5], [9, 9]], dtype=float)
        small = DetectionBox(1, 'ship', 0.9, np.array([[0, 0], [4, 0], [4, 4], [0, 4]]))
        large = DetectionBox(2, 'ship', 0.8, np.array([[2, 2], [10, 2], [10, 10], [2, 10]]))
        groups = select_detection_groups(points, pixels, [small, large], selection='largest')
        self.assertEqual(list(groups), [2])
        np.testing.assert_array_equal(groups[2], points[1:])
        all_groups = select_detection_groups(points, pixels, [small, large], selection='all')
        self.assertEqual(set(all_groups), {1, 2})
        np.testing.assert_array_equal(all_groups[1], points[:1])

    def test_target_filters_and_bad_polygons_are_rejected(self):
        points = np.array([[0, 0, 1]], dtype=float)
        pixels = np.array([[1, 1]], dtype=float)
        box = DetectionBox(4, 'ship', 1.0, np.array([[0, 0], [2, 0], [2, 2], [0, 2]]))
        self.assertEqual(select_detection_groups(points, pixels, [box], target_id=5), {})
        self.assertEqual(select_detection_groups(points, pixels, [box], target_class='car'), {})
        self.assertEqual(list(select_detection_groups(points, pixels, [box], target_class='ship')), [4])
        for polygon in (np.zeros((0, 2)), np.array([[0, 0], [1, 1]]), np.array([[0, 0], [1, 0], [2, 0]]), np.array([[0, 0], [np.nan, 1], [1, 0]])):
            with self.subTest(polygon=polygon), self.assertRaises(ValueError):
                select_detection_groups(
                    points, pixels, [DetectionBox(1, 'ship', 1.0, polygon)]
                )

    def test_boundary_pixel_is_inclusive_and_nonfinite_is_ignored(self):
        polygon = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=float)
        np.testing.assert_array_equal(
            points_in_polygon(np.array([[0, 0], [10, 5], [11, 5], [np.inf, 5]]), polygon),
            [True, True, False, False],
        )


class LegacyBBoxTests(unittest.TestCase):
    def test_largest_pca_area_cluster_is_selected_in_original_frame(self):
        small = [[10, y, z] for y in np.arange(0, 0.5, 0.2) for z in np.arange(0, 0.5, 0.2)]
        large = [[20, y, z] for y in np.arange(4, 6, 0.2) for z in np.arange(4, 5, 0.2)]
        selected = select_bbox_points(small + large, min_cluster_points=3)
        self.assertEqual(len(selected), len(large))
        np.testing.assert_array_equal(selected[:, 0], 20)

    def test_degenerate_and_empty_clusters_do_not_create_boxes(self):
        self.assertEqual(select_bbox_points([]).shape, (0, 3))
        self.assertEqual(len(select_bbox_points([[1, 1, 1]] * 20)), 0)


if __name__ == '__main__':
    unittest.main()
