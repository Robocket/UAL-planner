import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location(
    'yolo_scene_trial', Path(__file__).resolve().parents[1] / 'scripts/yolo_scene_trial.py')
trial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)


class YoloTrialTest(unittest.TestCase):
    def test_existing_checkpoint_checksum_is_required_to_match(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'weights.pt').write_bytes(b'checksum test')
            config = dict(weights='weights.pt', expected_sha256=trial.sha256(root / 'weights.pt'))
            self.assertEqual(trial.ensure_checkpoint(root, config), root / 'weights.pt')
            config['expected_sha256'] = '0' * 64
            with self.assertRaises(ValueError):
                trial.ensure_checkpoint(root, config)

    def test_detection_diagnostic_cannot_claim_mask_export(self):
        trial.validate_task('segment', True)
        trial.validate_task('detect', False)
        with self.assertRaises(ValueError):
            trial.validate_task('detect', True)
        with self.assertRaises(ValueError):
            trial.validate_task('obb', False)

    def test_class_lookup_and_unknown_class(self):
        self.assertEqual(trial.classes_for_names({0: 'person', 8: 'boat'}, ['boat']), [8])
        self.assertIsNone(trial.classes_for_names({8: 'boat'}, None))
        with self.assertRaises(ValueError):
            trial.classes_for_names({8: 'boat'}, ['deck'])

    def test_native_mask_coordinates(self):
        mask = trial.checked_masks(np.ones((2, 3, 4)), 2, (3, 4, 3))
        self.assertEqual(mask.dtype, np.uint8)
        with self.assertRaises(ValueError):
            trial.checked_masks(np.ones((2, 6, 8)), 2, (3, 4, 3))

    def test_missing_and_invalid_masks(self):
        self.assertEqual(trial.checked_masks(None, 0, (3, 4, 3)).shape, (0, 3, 4))
        with self.assertRaises(ValueError):
            trial.checked_masks(None, 1, (3, 4, 3))
        with self.assertRaises(ValueError):
            trial.checked_masks(np.full((1, 3, 4), np.nan), 1, (3, 4, 3))

    def test_manifest_duplicate_bag_frame_is_counted_once(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for folder in ('audit', 'clips'):
                directory = root / folder
                directory.mkdir()
                (directory / 'frame.png').touch()
                (directory / 'manifest.json').write_text(json.dumps({
                    'bag_path': 'same_bag',
                    'frames': [{'file': 'frame.png', 'bag_timestamp_ns': 123}]}))
            frames = trial.load_frames(root, ['audit/manifest.json', 'clips/manifest.json'])
            self.assertEqual(len(frames), 1)


if __name__ == '__main__':
    unittest.main()
