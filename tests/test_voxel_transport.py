"""Sensor binary-layout, delayed-acquisition matching and pipeline regressions."""

from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'Pointcloud_Voxels'))
from pointcloud_voxels.core import project_instance_ids, voxelize
from pointcloud_voxels.selection import select_mask_groups
from pointcloud_voxels.transport import CloudCache, read_id_map, read_xyz, stamp_ns


def stamped(seconds):
    sec = int(seconds)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=round((seconds-sec)*1e9))))


class DecodeTests(unittest.TestCase):
    def test_organized_big_endian_cloud_with_point_and_row_padding(self):
        data = b''.join(
            struct.pack('>fffI', *point, 99) + b'PADDING!'
            for point in ((1, 2, 3), (4, 5, 6))
        )
        message = NS(
            height=2, width=1, point_step=16, row_step=24,
            is_bigendian=True, data=data,
            fields=[NS(name=name, offset=axis*4, count=1, datatype=7)
                    for axis, name in enumerate(('x', 'y', 'z'))],
        )
        np.testing.assert_array_equal(read_xyz(message), [[1, 2, 3], [4, 5, 6]])
        message.data = data[:-1]
        with self.assertRaises(ValueError):
            read_xyz(message)

    def test_float64_cloud_and_unsupported_fields(self):
        message = NS(
            height=1, width=1, point_step=24, row_step=24,
            is_bigendian=False, data=struct.pack('<ddd', 1.25, 2.5, 3.75),
            fields=[NS(name=name, offset=axis*8, count=1, datatype=8)
                    for axis, name in enumerate(('x', 'y', 'z'))],
        )
        np.testing.assert_allclose(read_xyz(message), [[1.25, 2.5, 3.75]])
        message.fields[0].datatype = 6
        with self.assertRaises(ValueError):
            read_xyz(message)

    def test_id_map_respects_big_endian_padded_rows(self):
        message = NS(
            encoding='32SC1', height=2, width=2, step=12, is_bigendian=True,
            data=struct.pack('>iii', 0, 18, 999) + struct.pack('>iii', 7, 0, 999),
        )
        np.testing.assert_array_equal(read_id_map(message), [[0, 18], [7, 0]])
        message.encoding = 'mono8'
        with self.assertRaises(ValueError):
            read_id_map(message)
        message.encoding = '32SC1'
        message.data = b'\x00'
        with self.assertRaises(ValueError):
            read_id_map(message)


class TimeMatchingTests(unittest.TestCase):
    def test_delayed_mask_uses_acquisition_cloud_not_latest(self):
        cache = CloudCache(capacity=5)
        acquisition, latest = stamped(100.04), stamped(140.0)
        cache.append(acquisition)
        cache.append(latest)
        self.assertIs(cache.nearest(100_000_000_000, 0.1), acquisition)
        self.assertIsNone(cache.nearest(110_000_000_000, 0.1))

    def test_bounded_history_and_bag_loop_reset(self):
        cache = CloudCache(capacity=2, reset_threshold_sec=30)
        for value in (100, 101, 102):
            self.assertFalse(cache.append(stamped(value)))
        self.assertEqual(len(cache.messages), 2)
        self.assertIsNone(cache.nearest(100_000_000_000, 0))
        self.assertTrue(cache.append(stamped(10)))
        self.assertEqual(len(cache.messages), 1)
        self.assertIsNone(cache.nearest(102_000_000_000, 0))

    def test_invalid_or_zero_sensor_time_is_rejected(self):
        with self.assertRaises(ValueError):
            stamp_ns(stamped(0))
        invalid = stamped(1)
        invalid.header.stamp.nanosec = 1_000_000_000
        with self.assertRaises(ValueError):
            stamp_ns(invalid)


class MaskPipelineTests(unittest.TestCase):
    def test_largest_mask_not_largest_point_count_selects_original_points(self):
        labels = np.array([[1, 1, 1, 0], [2, 0, 0, 0]], dtype=np.int32)
        points = np.array([[0, 0, 1], [0, 1, 1], [0, 1, 1.5], [0, 2, 2]])
        ids = project_instance_ids(points, labels, np.eye(3), np.eye(4))
        # One point projects into mask 1, three into smaller mask 2.
        np.testing.assert_array_equal(ids, [1, 2, 2, 2])
        groups = select_mask_groups(points, ids, labels, {1: 'ship', 2: 'ship'})
        self.assertEqual(list(groups), [1])
        np.testing.assert_array_equal(groups[1], [[0, 0, 1]])
        self.assertEqual(voxelize(groups[1]).classes.item(), 2)

    def test_all_instances_remain_separate_and_background_is_excluded(self):
        points = np.array([[1, 0, 1], [2, 0, 1], [3, 0, 1]])
        ids = np.array([9, 0, 4], dtype=np.int32)
        labels = np.array([[9, 0, 4]], dtype=np.int32)
        groups = select_mask_groups(points, ids, labels, {9: 'ship', 4: 'car'}, selection='all')
        self.assertEqual(set(groups), {4, 9})
        self.assertEqual(sum(map(len, groups.values())), 2)
        filtered = select_mask_groups(points, ids, labels, {9: 'ship', 4: 'car'},
                                      selection='all', target_class='ship')
        self.assertEqual(list(filtered), [9])

    def test_valid_empty_mask_clears_selection_and_bad_metadata_fails(self):
        points = np.array([[0, 0, 1]])
        self.assertEqual(select_mask_groups(points, np.array([0]), np.zeros((2, 2), dtype=int), {}), {})
        with self.assertRaises(ValueError):
            select_mask_groups(points, np.array([8]), np.array([[8]]), {})


if __name__ == '__main__':
    unittest.main()
