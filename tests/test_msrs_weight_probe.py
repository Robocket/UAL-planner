import hashlib
import importlib.util
from pathlib import Path
import pickle
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
spec = importlib.util.spec_from_file_location('probe_msrs', ROOT / 'scripts/probe_msrs_weights.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class ProbeTests(unittest.TestCase):
    def test_static_pickle_does_not_execute(self):
        class Untrusted:
            def __reduce__(self):
                return (print, ('must not execute',))
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'model.pt'
            with zipfile.ZipFile(path, 'w') as archive:
                archive.writestr('best/data.pkl', pickle.dumps(Untrusted(), protocol=2))
            with patch('builtins.print') as printer:
                refs = probe.checkpoint_globals(path)
            printer.assert_not_called()
            self.assertIn(('__builtin__', 'print'), refs)

    def test_unsupported_stack_global_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'model.pt'
            with zipfile.ZipFile(path, 'w') as archive:
                archive.writestr('best/data.pkl', b'\x80\x04\x8c\x01a\x8c\x01b\x93.')
            with self.assertRaises(ValueError):
                probe.checkpoint_globals(path)

    def test_missing_globals_and_unknown_namespace(self):
        with patch.object(probe.importlib, 'import_module', side_effect=ModuleNotFoundError('missing')):
            self.assertEqual(len(probe.missing_globals([('ultralytics.nn.extra_modules.filc', 'FAT_Block')])), 1)
        with self.assertRaises(ValueError):
            probe.missing_globals([('os', 'system')])

    def test_standard_torch_namespace_allowed(self):
        with patch.object(probe.importlib, 'import_module') as importer:
            self.assertEqual(probe.missing_globals([('torch', 'FloatStorage')]), [])
        importer.assert_called_once_with('torch')

    def test_publisher_hash_and_size_required(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'model.pt'
            path.write_bytes(b'test weight')
            data = {'file': path.name, 'bytes': 11, 'sha256': hashlib.sha256(b'test weight').hexdigest()}
            self.assertEqual(probe.verify_weight(path, data), data['sha256'])
            data['sha256'] = '0' * 64
            with self.assertRaises(ValueError):
                probe.verify_weight(path, data)


if __name__ == '__main__':
    unittest.main()
