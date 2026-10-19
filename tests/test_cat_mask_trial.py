import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
spec = importlib.util.spec_from_file_location('cat_mask_trial', ROOT / 'scripts/cat_mask_trial.py')
trial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)


class CatMaskTests(unittest.TestCase):
    def predictions(self):
        boxes = np.array([[0, 0, 4, 3], [0, 0, 4, 3]], dtype=float)
        masks = np.zeros((2, 3, 4), dtype=bool)
        masks[0, 1, 2] = True
        return boxes, np.array([0.8, 0.9]), np.array([1, 0]), masks

    def decode(self, values):
        return trial.decode_instances(*values, ['vehicle', 'ship'], {1}, (3, 4, 3), 0.1)

    def test_masks_are_model_pixels_not_filled_boxes(self):
        result = self.decode(self.predictions())
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][0]['mask_area_pixels'], 1)
        self.assertEqual(result[0][0]['class_name'], 'ship')

    def test_original_resolution_required(self):
        boxes, scores, labels, _ = self.predictions()
        with self.assertRaises(ValueError):
            self.decode((boxes, scores, labels, np.zeros((2, 6, 8), dtype=bool)))

    def test_invalid_predictions_rejected(self):
        boxes, scores, labels, masks = self.predictions()
        for values in [(boxes, scores * np.nan, labels, masks),
                       (boxes, scores, labels + 20, masks),
                       (boxes, scores, labels.astype(float), masks),
                       (boxes, scores, labels, masks.astype(float)),
                       (boxes[:, ::-1], scores, labels, masks)]:
            with self.assertRaises(ValueError):
                self.decode(values)

    def test_empty_predictions_valid(self):
        result = self.decode((np.empty((0, 4)), np.empty(0), np.empty(0, dtype=int), np.empty((0, 3, 4), dtype=bool)))
        self.assertEqual(result, [])

    def test_overlay_does_not_mutate_image(self):
        image = np.zeros((3, 4, 3), dtype=np.uint8)
        trial.overlay(image, self.decode(self.predictions()), 0.25, 0.45)
        self.assertFalse(image.any())

    def test_config_requires_valid_thresholds(self):
        config = yaml.safe_load((ROOT / 'config/cat_mask_trial.yaml').read_text())
        trial.validate_config(config)
        config['model_score_threshold'] = 0.5
        with self.assertRaises(ValueError):
            trial.validate_config(config)


if __name__ == '__main__':
    unittest.main()
