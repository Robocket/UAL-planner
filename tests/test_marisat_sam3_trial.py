import importlib.util
import os
from pathlib import Path
import pickle
import sys
import tempfile
import unittest
import zipfile

import torch
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
spec = importlib.util.spec_from_file_location('marisat_trial', ROOT/'scripts/marisat_sam3_trial.py')
trial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)


class MariSatTests(unittest.TestCase):
    def checkpoint(self, folder):
        original = folder/'original.pt'
        expected = {'weight':torch.arange(6,dtype=torch.float16).reshape(2,3),
                    'bias':torch.ones(2)}
        torch.save(expected, original)
        with zipfile.ZipFile(original) as archive:
            archive.extractall(folder/'extracted')
        return folder/'extracted'/'original', expected

    def test_rewrap_old_timestamp_exact_roundtrip_and_safe_reuse(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name)
            source,expected=self.checkpoint(folder)
            for p in source.rglob('*'):
                if p.is_file(): os.utime(p,(0,0))
            before=trial.source_manifest(source)
            dest=folder/'restored.pt'
            record=trial.restore_checkpoint(source,dest)
            state=torch.load(dest,map_location='cpu',weights_only=True,mmap=True)
            self.assertEqual(set(state),set(expected))
            for k in expected:
                self.assertTrue(torch.equal(state[k],expected[k]))
                self.assertEqual(state[k].dtype,expected[k].dtype)
            self.assertEqual(before,trial.source_manifest(source))
            self.assertEqual(record,trial.restore_checkpoint(source,dest))

    def test_changed_input_cannot_reuse_archive(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name)
            source,_=self.checkpoint(folder)
            dest=folder/'restored.pt'
            trial.restore_checkpoint(source,dest)
            (source/'data'/'0').write_bytes(b'changed')
            with self.assertRaises(ValueError):
                trial.restore_checkpoint(source,dest)

    def test_unknown_pickle_namespace_refused_before_deserialization(self):
        class Payload:
            def __reduce__(self): return (print,('never execute',))
        with tempfile.TemporaryDirectory() as name:
            source,_=self.checkpoint(Path(name))
            (source/'data.pkl').write_bytes(pickle.dumps(Payload(),protocol=2))
            with self.assertRaises(ValueError): trial.source_manifest(source)

    def test_symlink_refused(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name)
            source,_=self.checkpoint(folder)
            (source/'linked').symlink_to(folder/'original.pt')
            with self.assertRaises(ValueError): trial.source_manifest(source)

    def test_strict_complete_keys_and_shapes_required(self):
        model=torch.nn.Linear(3,2)
        state={k:v.clone() for k,v in model.state_dict().items()}
        self.assertIs(trial.strict_state_load(model,state),model)
        for bad in [{'weight':state['weight']}, {**state,'extra':torch.zeros(1)},
                    {**state,'weight':torch.zeros(1,3)}]:
            with self.assertRaises(ValueError): trial.strict_state_load(model,bad)

    def test_stored_nonfinite_values_reported_without_repair(self):
        state={'projection':torch.tensor([[0.,float('-inf'),float('nan')]],dtype=torch.float16)}
        report=trial.audit_state(state)
        self.assertFalse(report['all_finite'])
        bad=report['invalid_tensors'][0]
        self.assertEqual(bad['nan_count'],1)
        self.assertEqual(bad['negative_inf_count'],1)
        self.assertEqual(bad['first_indices'],[[0,1],[0,2]])
        self.assertTrue(torch.isneginf(state['projection'][0,1]))
        self.assertTrue(trial.audit_state({'weight':torch.ones(2)})['all_finite'])

    def test_config_validates_author_geometry_and_classes(self):
        config=yaml.safe_load((ROOT/'config/marisat_sam3_trial.yaml').read_text())
        trial.validate_config(config)
        for key,value in [('image_size',512),('mask_probability_threshold',0),('thresholds',[float('nan')]),
                          ('large_ship_classes',['unknown'])]:
            bad={**config,key:value}
            with self.assertRaises(ValueError): trial.validate_config(bad)


if __name__=='__main__':
    unittest.main()
