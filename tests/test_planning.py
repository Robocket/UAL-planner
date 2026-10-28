"""Offline 3D A* path planning tests; no ROS or sensor required."""

from pathlib import Path
import sys
import unittest

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'Pointcloud_Voxels'))
from pointcloud_voxels.planning import (  # noqa: E402
    plan_path, point_bounds,
)


class PlanningTests(unittest.TestCase):
    def test_point_bounds_and_center_use_the_full_selected_cloud(self):
        lower, upper, center = point_bounds([[1, 2, 3], [5, -2, 9]])
        np.testing.assert_allclose(lower, [1, -2, 3])
        np.testing.assert_allclose(upper, [5, 2, 9])
        np.testing.assert_allclose(center, [3, 0, 6])
        with self.assertRaises(ValueError):
            point_bounds(np.empty((0, 3)))

    def test_straight_free_path_keeps_exact_endpoints(self):
        result = plan_path([0, 0, 0], [10, 0, 0], [], resolution=1.0)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result.waypoints[0], [0, 0, 0])
        np.testing.assert_allclose(result.waypoints[-1], [10, 0, 0])
        self.assertAlmostEqual(result.length, 10.0, places=6)
        self.assertGreaterEqual(result.visited, 1)

    def test_path_goes_around_an_obstacle_wall(self):
        wall = [[5.0, y, z] for y in np.arange(-1.0, 1.01, 0.25)
                for z in np.arange(-1.0, 1.01, 0.25)]
        result = plan_path(
            [0, 0, 0], [10, 0, 0], wall, resolution=0.5, inflation=0.25,
            margin_xy=3.0, margin_z=3.0, smooth=True,
        )
        self.assertIsNotNone(result)
        self.assertGreater(result.length, 10.0)
        self.assertGreater(np.max(np.abs(result.waypoints[:, 1])), 1.0)

    def test_wall_with_no_available_detour_returns_none(self):
        wall = [[5.0, y, z] for y in np.arange(-4.0, 4.01, 0.25)
                for z in np.arange(-4.0, 4.01, 0.25)]
        result = plan_path(
            [0, 0, 0], [10, 0, 0], wall, resolution=0.5, inflation=0.5,
            margin_xy=3.0, margin_z=3.0,
        )
        self.assertIsNone(result)

    def test_start_equal_target_and_grid_limit(self):
        result = plan_path([1, 2, 3], [1, 2, 3], [], resolution=1.0)
        self.assertIsNotNone(result)
        self.assertEqual(len(result.waypoints), 2)
        self.assertEqual(result.length, 0.0)
        with self.assertRaises(ValueError):
            plan_path([0, 0, 0], [100, 100, 100], [], margin_xy=10, margin_z=10,
                      resolution=0.25, max_grid_cells=100)

    def test_invalid_inputs_are_rejected(self):
        for kwargs in ({'resolution': 0}, {'inflation': -1}, {'margin_xy': -1},
                       {'margin_z': np.nan}, {'max_grid_cells': 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                plan_path([0, 0, 0], [1, 0, 0], [], **kwargs)
        with self.assertRaises(ValueError):
            plan_path([0, 0], [1, 0, 0], [])
        with self.assertRaises(ValueError):
            plan_path([0, 0, 0], [1, 0, 0], [[np.nan, 0, 0]])


if __name__ == '__main__':
    unittest.main()
