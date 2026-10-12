import importlib.util
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
spec = importlib.util.spec_from_file_location('ship_hybrid_trial',
    Path(__file__).resolve().parents[1] / 'scripts/ship_hybrid_trial.py')
trial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)


class ShipHybridTrialTest(unittest.TestCase):
    def test_mask_count_is_json_serializable(self):
        masks = [np.zeros((5, 6), np.uint8), np.ones((5, 6), np.uint8)]
        self.assertEqual(json.dumps({'count': trial.count_nonempty_masks(masks)}), '{"count": 1}')
        self.assertEqual(trial.count_nonempty_masks([]), 0)

    def test_box_clipping_and_empty_rejection(self):
        self.assertEqual(trial.clipped_box([-2, 1.2, 30, 8.1], (10, 20, 3)), (0, 1, 20, 9))
        with self.assertRaises(ValueError):
            trial.clipped_box([20, 0, 25, 5], (10, 20, 3))
        with self.assertRaises(ValueError):
            trial.clipped_box([0, 0, np.nan, 5], (10, 20, 3))

    def test_empty_mask_is_not_replaced_with_a_box(self):
        self.assertIsNone(trial.mask_box(np.zeros((5, 6), np.uint8)))
        mask = np.zeros((5, 6), np.uint8)
        mask[1:3, 2:5] = 1
        self.assertEqual(trial.mask_box(mask), [2, 1, 5, 3])

    def test_grabcut_native_coordinates_on_synthetic_foreground(self):
        image = np.full((60, 80, 3), 20, np.uint8)
        image[20:40, 30:50] = 220
        cfg = dict(crop_context_fraction=.2, max_crop_edge=384, iterations=3,
                   border_background_px=1, rng_seed=0)
        mask, status = trial.grabcut_mask(image, [28, 18, 52, 42], cfg)
        self.assertEqual(mask.shape, image.shape[:2])
        self.assertEqual(mask.dtype, np.uint8)
        self.assertEqual(status, 'candidate')
        self.assertTrue(mask[25:35, 35:45].all())
        self.assertFalse(mask[:10].any())


if __name__ == '__main__':
    unittest.main()
