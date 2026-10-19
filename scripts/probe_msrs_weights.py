#!/usr/bin/env python3
"""Record missing checkpoint implementations without creating substitute layers."""
import argparse
from datetime import datetime, timezone
import gc
import importlib
import json
import os
from pathlib import Path
import pickletools
import platform
import subprocess
import sys
import traceback
import zipfile

import yaml

from ship_detection_trial import collect_samples, dump_json, sha256


def checkpoint_globals(path):
    """Inspect pickle opcodes without unpickling or executing the checkpoint."""
    with zipfile.ZipFile(path) as archive:
        entries = [name for name in archive.namelist() if name.endswith('/data.pkl') or name == 'data.pkl']
        if len(entries) != 1:
            raise ValueError('Expected one PyTorch data.pkl')
        references = set()
        for op, value, _ in pickletools.genops(archive.read(entries[0])):
            if op.name == 'GLOBAL':
                references.add(tuple(value.split(' ', 1)))
            elif op.name == 'STACK_GLOBAL':
                raise ValueError('STACK_GLOBAL unsupported by this static inspector; refusing partial audit')
    return sorted(references)


def missing_globals(references):
    missing = []
    for module, name in references:
        if module not in ('collections', 'torch') and not module.startswith(('torch.', 'dill.', 'ultralytics.')):
            raise ValueError(f'Unexpected checkpoint namespace: {module}')
        try:
            getattr(importlib.import_module(module), name)
        except (ImportError, AttributeError) as exc:
            missing.append({'module': module, 'name': name, 'error': f'{type(exc).__name__}: {exc}'})
    return missing


def verify_weight(path, spec):
    if Path(spec['file']).name != spec['file']:
        raise ValueError('Weights must use flat filenames')
    digest = sha256(path)
    if path.stat().st_size != spec['bytes'] or digest != spec['sha256']:
        raise ValueError(f'Publisher checksum/size mismatch: {path.name}')
    return digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/msrs_weight_probe.yaml')
    parser.add_argument('--attempt-load', action='store_true',
                        help='Explicitly deserialize publisher-verified full-model checkpoints on CPU; trusted files only')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / args.config).read_text())
    files = [(root / config['weights_dir'] / spec['file'], spec) for spec in config['models']]
    # Verify every file before any checkpoint deserialization.
    for path, spec in files:
        verify_weight(path, spec)
    output = root / config['output_dir']
    output.mkdir(parents=True, exist_ok=False)
    runtime = output / 'runtime'
    runtime.mkdir()
    os.environ.update(YOLO_AUTOINSTALL='false', YOLO_OFFLINE='true', YOLO_CONFIG_DIR=str(runtime))
    import torch
    import ultralytics
    from ultralytics import settings
    settings.update({'sync': False})
    samples = collect_samples(root, yaml.safe_load((root / config['sample_config']).read_text())['inputs'])
    dump_json(output / 'sample_manifest.json', samples)
    dump_json(output / 'config_snapshot.json', config)
    environment = {'experiment_date': config['experiment_date'], 'utc': datetime.now(timezone.utc).isoformat(),
                   'host': platform.node(), 'python': platform.python_version(), 'torch': torch.__version__,
                   'ultralytics': ultralytics.__version__, 'cuda_available': torch.cuda.is_available(),
                   'load_map_location': 'cpu', 'autoinstall': False,
                   'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
                   'probe_sha256': sha256(Path(__file__)), 'config_sha256': sha256(root / args.config)}
    dump_json(output / 'environment.json', environment)
    records = []
    for path, spec in files:
        refs = checkpoint_globals(path)
        missing = missing_globals(refs)
        record = {**spec, 'path': str(path.relative_to(root)), 'publisher_hash_match': True,
                  'globals': [f'{m}.{n}' for m, n in refs], 'missing_globals': missing,
                  'status': 'static_missing_implementations' if missing else 'static_resolvable',
                  'load_attempted': args.attempt_load, 'frames_inferred': 0,
                  'predict_ms': None, 'accuracy': None, 'user_acceptance': 'not_evaluated'}
        if args.attempt_load:
            try:
                checkpoint = torch.load(path, map_location='cpu', weights_only=False)
                record['status'] = 'loaded_only_not_inferred'
                del checkpoint
            except Exception as exc:
                record.update(status='load_blocked', load_error=f'{type(exc).__name__}: {exc}',
                              traceback=traceback.format_exc())
            finally:
                gc.collect()
        records.append(record)
        dump_json(output / 'checkpoint_probe.json', {'environment': environment, 'models': records,
                  'planned_samples': len(samples), 'frames_inferred': 0,
                  'note': 'Loading preflight only; zero inferred frames does not mean zero detections or failed quality acceptance.'})
        print(json.dumps({key: record.get(key) for key in ('file', 'status', 'load_error', 'frames_inferred')},
                         ensure_ascii=False), flush=True)
    if any(record['status'] == 'load_blocked' or record['missing_globals'] for record in records):
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
