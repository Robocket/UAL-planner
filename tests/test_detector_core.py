"""Offline YOLO result decoding tests; no model, GPU or ROS required."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np


sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / 'src' / 'Instance_Seg' / 'scripts')
)
from detector_core import (  # noqa: E402
    decode_result, normalize_names, polygon_area, target_class_ids,
)


def obb_box(class_id, confidence, corners):
    return SimpleNamespace(
        cls=np.array([[class_id]]), conf=np.array([confidence]),
        xyxyxyxy=np.asarray(corners, dtype=float).reshape(1, 4, 2),
    )


def detect_box(class_id, confidence, xyxy):
    return SimpleNamespace(
        cls=np.array([[class_id]]), conf=np.array([confidence]),
        xyxy=np.asarray(xyxy, dtype=float).reshape(1, 4),
    )


class DetectorCoreTests(unittest.TestCase):
    def test_obb_and_horizontal_results_become_four_corner_polygons(self):
        names = ['background', 'ship']
        obb = SimpleNamespace(names=names, obb=[obb_box(1, 0.8, [[10, 10], [20, 12], [19, 22], [9, 20]])])
        decoded = decode_result(obb, 'obb', ['ship'], 0.25)
        self.assertEqual(len(decoded), 1)
        self.assertTrue(decoded[0].oriented)
        self.assertEqual(decoded[0].class_name, 'ship')
        np.testing.assert_allclose(decoded[0].polygon, [[10, 10], [20, 12], [19, 22], [9, 20]])
        horizontal = SimpleNamespace(names=names, boxes=[detect_box(1, 0.7, [4, 5, 8, 9])])
        decoded = decode_result(horizontal, 'detect', ['ship'], 0.25)
        self.assertEqual(len(decoded), 1)
        self.assertFalse(decoded[0].oriented)
        np.testing.assert_allclose(decoded[0].polygon, [[4, 5], [8, 5], [8, 9], [4, 9]])
        self.assertAlmostEqual(polygon_area(decoded[0].polygon), 16.0)

    def test_non_target_and_low_score_boxes_are_filtered(self):
        names = {0: 'car', 1: 'ship', 2: 'warship'}
        result = SimpleNamespace(names=names, obb=[
            obb_box(0, 0.99, [[0, 0], [5, 0], [5, 5], [0, 5]]),
            obb_box(1, 0.10, [[0, 0], [5, 0], [5, 5], [0, 5]]),
            obb_box(2, 0.90, [[0, 0], [5, 0], [5, 5], [0, 5]]),
        ])
        self.assertEqual([item.class_name for item in decode_result(result, 'obb', ['ship'], 0.25)], [])
        self.assertEqual(
            [item.class_name for item in decode_result(result, 'obb', [], 0.25)],
            ['car', 'warship'],
        )
        self.assertEqual([item.class_name for item in decode_result(result, 'obb', ['ship'], 0.10)], ['ship'])

    def test_malformed_outputs_and_bad_configuration_are_rejected(self):
        names = ['background', 'ship']
        invalid = [
            SimpleNamespace(names=names, obb=[obb_box(1, 0.9, [[0, 0], [1, 0], [2, 0], [3, 0]])]),
            SimpleNamespace(names=names, obb=[obb_box(1, 1.2, [[0, 0], [5, 0], [5, 5], [0, 5]])]),
            SimpleNamespace(names=names, obb=[obb_box(9, 0.9, [[0, 0], [5, 0], [5, 5], [0, 5]])]),
            SimpleNamespace(names=names, obb=[obb_box(1, 0.9, [[0, 0], [1, 0], [2, 0], [3, 0]])]),
            SimpleNamespace(names=names, boxes=[detect_box(1, 0.9, [10, 10, 5, 5])]),
            SimpleNamespace(names=names, obb=None),
        ]
        for result in invalid:
            with self.subTest(result=result), self.assertRaises(ValueError):
                decode_result(result, 'obb', ['ship'], 0.25)
        with self.assertRaises(ValueError):
            decode_result(SimpleNamespace(names=names, obb=[]), 'segment', ['ship'], 0.25)
        with self.assertRaises(ValueError):
            target_class_ids(names, ['boat'])

    def test_name_metadata_must_be_complete(self):
        self.assertEqual(normalize_names(['a', 'b']), {0: 'a', 1: 'b'})
        self.assertEqual(normalize_names({'0': 'a'}), {0: 'a'})
        for names in ('bad', [], {0: ' '}, {-1: 'a'}):
            with self.subTest(names=names), self.assertRaises(ValueError):
                normalize_names(names)


if __name__ == '__main__':
    unittest.main()
