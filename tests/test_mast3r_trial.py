import importlib.util
from pathlib import Path
import unittest
import tempfile

import numpy as np

spec = importlib.util.spec_from_file_location(
    'mast3r_scene_trial', Path(__file__).resolve().parents[1] / 'scripts/mast3r_scene_trial.py')
trial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)


class PointMaskTest(unittest.TestCase):
    def test_nonfinite_confidence_and_behind_camera(self):
        points = np.array([[[1., 2., 3.], [1., 2., -1.], [np.nan, 0., 1.],
                            [0., 0., 1.], [0., 0., 1.]]])
        confidence = np.array([[3., 9., 9., 2.9, np.nan]])
        np.testing.assert_array_equal(trial.point_mask(points, confidence, 3., True),
                                      [[True, False, False, False, False]])

    def test_allow_negative_z_when_requested(self):
        points = np.array([[[0., 0., -1.]]])
        self.assertTrue(trial.point_mask(points, np.array([[3.]]), 3., False)[0, 0])

    def test_export_common_frame_and_counts(self):
        try:
            import torch
            import matplotlib  # noqa: F401
        except ImportError:
            self.skipTest('Export test requires the isolated trial environment')
        # Both point maps are already in camera 1 coordinates: no second transform.
        points = torch.tensor([[[[1., 2., 3.], [4., 5., 6.]],
                                 [[7., 8., -1.], [10., 11., 12.]]]])
        confidence = torch.tensor([[[3., 2.], [8., 4.]]])
        result = {'pred1': {'pts3d': points, 'conf': confidence},
                  'pred2': {'pts3d_in_other_view': points + 1, 'conf': confidence},
                  'view1': {'img': torch.zeros(1, 3, 2, 2)},
                  'view2': {'img': torch.zeros(1, 3, 2, 2)}}
        config = dict(confidence_threshold=3., positive_z_only=True, preview_max_points=3)
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            statistics = trial.export_result(result, folder, config)
            self.assertEqual([s['retained_points'] for s in statistics], [2, 2])
            self.assertIn('element vertex 4', (folder / 'pair_local.ply').read_text())
            with np.load(folder / 'pointmaps.npz') as arrays:
                np.testing.assert_array_equal(arrays['points2'], (points + 1)[0].numpy())
            self.assertTrue((folder / 'preview.png').is_file())
            self.assertTrue((folder / 'cloud_preview.png').is_file())


if __name__ == '__main__':
    unittest.main()
