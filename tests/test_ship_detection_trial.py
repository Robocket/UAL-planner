import copy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zipfile

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


trial = load_script('ship_detection_trial')
fetch = load_script('fetch_ship_detection_weights')


class TrialTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / 'config/ship_detection_trial.yaml').read_text())

    def test_valid_config_and_stride_rejected(self):
        trial.validate_config(self.config)
        self.config['models'][0]['image_sizes'] = [930]
        with self.assertRaises(ValueError):
            trial.validate_config(self.config)

    def test_thresholds_rejected(self):
        for value in (0, -1, 1.1):
            config = copy.deepcopy(self.config)
            config['thresholds'] = [value]
            with self.assertRaises(ValueError):
                trial.validate_config(config)

    def test_class_mapping_uses_names_not_coco_id(self):
        self.assertEqual(trial.target_ids({0: 'ship', 1: 'warship'}, ['ship', 'warship']), {0, 1})
        self.assertEqual(trial.target_ids({6: 'ship', 9: 'small-vehicle'}, ['ship']), {6})
        with self.assertRaises(ValueError):
            trial.target_ids({8: 'boat'}, ['ship'])

    def test_confidence_selection_does_not_include_non_ships(self):
        data = [{'target': True, 'confidence': 0.24}, {'target': True, 'confidence': 0.25},
                {'target': False, 'confidence': 0.99}]
        self.assertEqual(len(trial.select_targets(data, 0.25)), 1)
        self.assertEqual(len(trial.select_targets(data, 0.10)), 2)

    def test_hbb_and_obb_original_pixel_coordinates(self):
        import torch
        box = SimpleNamespace(cls=torch.tensor([6]), conf=torch.tensor([0.8]),
                              xyxy=torch.tensor([[2, 3, 20, 30]]),
                              xyxyxyxy=torch.tensor([[[10, 2], [20, 12], [10, 22], [0, 12]]]))
        result = SimpleNamespace(names={6: 'ship'}, boxes=[box], obb=[box])
        horizontal = trial.decode_detections(result, 'detect', {6})[0]
        rotated = trial.decode_detections(result, 'obb', {6})[0]
        self.assertEqual(horizontal['xyxy'], [2, 3, 20, 30])
        self.assertIsNone(horizontal['polygon'])
        self.assertEqual(rotated['xyxy'], [0, 2, 20, 22])
        self.assertEqual(len(rotated['polygon']), 4)
        self.assertTrue(rotated['target'])

    def test_missing_task_output_rejected(self):
        with self.assertRaises(ValueError):
            trial.decode_detections(SimpleNamespace(obb=None), 'obb', {6})

    def test_nonfinite_detection_rejected(self):
        import torch
        box = SimpleNamespace(cls=torch.tensor([0]), conf=torch.tensor([float('nan')]),
                              xyxy=torch.tensor([[0, 0, 10, 10]]))
        with self.assertRaises(ValueError):
            trial.decode_detections(SimpleNamespace(names={0: 'ship'}, boxes=[box]), 'detect', {0})

    def test_decoded_pixel_dedup_retains_provenance(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            image = np.zeros((4, 6, 3), dtype=np.uint8)
            cv2.imwrite(str(root / 'a.png'), image)
            cv2.imwrite(str(root / 'b.png'), image)
            (root / 'manifest.json').write_text(json.dumps({'frames': [{'file': 'a.png'}, {'file': 'b.png'}]}))
            samples = trial.collect_samples(root, {'manifests': ['manifest.json'], 'controls': []})
            self.assertEqual(len(samples), 1)
            self.assertEqual(len(samples[0]['sources']), 2)
            self.assertEqual(samples[0]['shape'], [4, 6, 3])

    def test_missing_image_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(ValueError):
                trial.collect_samples(Path(temp), {'manifests': [], 'controls': ['missing.png']})

    def test_annotate_does_not_change_input(self):
        image = np.zeros((40, 60, 3), np.uint8)
        for polygon in (None, [[2, 2], [30, 2], [30, 30], [2, 30]]):
            detection = {'xyxy': [2, 2, 30, 30], 'polygon': polygon, 'class_name': 'ship', 'confidence': 0.8}
            result = trial.annotate(image, [detection])
            self.assertEqual(result.shape, image.shape)
            self.assertTrue(result.any())
            self.assertFalse(image.any())

    def test_output_directory_is_exclusive(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(FileExistsError):
                Path(temp).mkdir(parents=True, exist_ok=False)

    def test_percentiles_and_empty_timing(self):
        self.assertIsNone(trial.distribution([]))
        self.assertEqual(trial.distribution([1, 2, 3])['median'], 2)


class FetchTests(unittest.TestCase):
    def test_metadata_names_do_not_collide(self):
        a = fetch.member_destination('models/ship/yolo11s/metadata.json')
        b = fetch.member_destination('models/ship/yolo11l/metadata.json')
        self.assertEqual(a, 'yolo11s_metadata.json')
        self.assertNotEqual(a, b)

    def test_selected_member_crc(self):
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, 'w', zipfile.ZIP_DEFLATED) as archive:
            archive.writestr('a.pt', b'model payload' * 10)
        with zipfile.ZipFile(buffer) as archive:
            info = archive.getinfo('a.pt')
        self.assertEqual(fetch.read_member(buffer, info), b'model payload' * 10)
        info.CRC ^= 1
        with self.assertRaises(ValueError):
            fetch.read_member(buffer, info)

    def test_http_full_archive_refused(self):
        with patch.object(fetch.urllib.request, 'urlopen') as opener:
            opener.return_value.__enter__.return_value.status = 200
            with self.assertRaises(RuntimeError):
                fetch.RemoteZip('https://example.invalid/archive.zip', 100).read(10)

    def test_range_seek_bounds(self):
        remote = fetch.RemoteZip('https://example.invalid/archive.zip', 100)
        self.assertEqual(remote.seek(-5, 2), 95)
        self.assertEqual(remote.tell(), 95)
        with self.assertRaises(ValueError):
            remote.seek(-1)
        self.assertEqual(remote.seek(100), 100)
        self.assertEqual(remote.read(1), b'')


if __name__ == '__main__':
    unittest.main()
