#!/usr/bin/env python3
"""Restore an extracted PyTorch container and test the full MariSat SAM3 state_dict."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import pickletools
import platform
import subprocess
import sys
import time
import traceback
from unittest.mock import patch
import zipfile

import cv2
import numpy as np
import yaml

from ship_detection_trial import collect_samples, distribution, dump_json


def file_hash(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def source_manifest(source):
    if source.is_symlink() or any(p.is_symlink() for p in source.rglob('*')):
        raise ValueError('Symlinks not allowed in extracted checkpoint')
    required = ['data.pkl', 'byteorder', 'version']
    if any(not (source / name).is_file() for name in required) or not (source / 'data').is_dir():
        raise ValueError('Not an extracted PyTorch checkpoint')
    allowed = {'collections OrderedDict', 'torch._utils _rebuild_tensor_v2',
               'torch HalfStorage', 'torch FloatStorage'}
    for op, value, _ in pickletools.genops((source / 'data.pkl').read_bytes()):
        if op.name == 'STACK_GLOBAL' or (op.name == 'GLOBAL' and value not in allowed):
            raise ValueError('Unexpected pickle reference; do not deserialize')
    return [{'file': p.relative_to(source).as_posix(), 'bytes': p.stat().st_size, 'sha256': file_hash(p)}
            for p in sorted(source.rglob('*')) if p.is_file()]


def restore_checkpoint(source, destination):
    """Rewrap unchanged bytes, never modify user input or silently reuse a mismatch."""
    files = source_manifest(source)
    record_path = destination.with_suffix('.provenance.json')
    if destination.exists():
        previous = json.loads(record_path.read_text())
        if previous['files'] != files or previous['archive_sha256'] != file_hash(destination):
            raise ValueError('Existing prepared checkpoint differs from source/provenance')
        return previous
    destination.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(destination, mode='x', compression=zipfile.ZIP_STORED,
                         allowZip64=True, strict_timestamps=False) as archive:
        for entry in files:
            archive.write(source / entry['file'], 'sam3_semantic_best/' + entry['file'])
    with zipfile.ZipFile(destination) as archive:
        if archive.testzip() is not None:
            raise ValueError('Restored container CRC failure')
    record = {'source': str(source), 'files': files, 'archive_sha256': file_hash(destination),
              'source_bytes': sum(x['bytes'] for x in files), 'archive_bytes': destination.stat().st_size,
              'note': 'Locally rewrapped extracted bytes; not the original publisher ZIP hash or signature.'}
    dump_json(record_path, record)
    return record


def strict_state_load(model, state):
    expected = model.state_dict()
    if set(state) != set(expected):
        raise ValueError(f'State keys mismatch: missing={sorted(set(expected)-set(state))[:5]}, '
                         f'unexpected={sorted(set(state)-set(expected))[:5]}')
    if any(state[k].shape != expected[k].shape for k in state):
        raise ValueError('State tensor shape mismatch')
    model.load_state_dict(state, strict=True, assign=True)
    return model


def audit_state(state):
    """Inspect stored values before dtype/device conversion; do not repair them."""
    import torch
    invalid = []
    for key, tensor in state.items():
        finite = torch.isfinite(tensor)
        if not finite.all():
            indices = (~finite).nonzero()
            invalid.append({'key': key, 'dtype': str(tensor.dtype), 'shape': list(tensor.shape),
                            'nan_count': int(torch.isnan(tensor).sum()),
                            'positive_inf_count': int(torch.isposinf(tensor).sum()),
                            'negative_inf_count': int(torch.isneginf(tensor).sum()),
                            'first_indices': indices[:10].tolist()})
    return {'state_entries': len(state), 'all_finite': not invalid, 'invalid_tensors': invalid,
            'note': 'Stored checkpoint values, before half(), CUDA, or forward; nothing repaired.'}


def validate_config(config):
    if config['image_size'] != 1008:
        raise ValueError('Preserve author 1008 image / 72x72 feature geometry')
    if config['warmup'] < 1 or config['cpu_threads'] < 1 or config['max_instances_per_image'] < 1:
        raise ValueError('Positive warmup, threads and output limit required')
    if not config['thresholds'] or any(not 0 < t <= 1 for t in config['thresholds']):
        raise ValueError('Invalid score thresholds')
    if not 0 < config['mask_probability_threshold'] < 1:
        raise ValueError('Invalid mask probability threshold')
    names = [entry['name'] for entry in config['classes']]
    if not names or len(names) != len(set(names)) or any(not c['prompt'].strip() for c in config['classes']):
        raise ValueError('Unique nonempty classes/prompts required')
    if not set(config['large_ship_classes']) <= set(names):
        raise ValueError('Unknown large-ship class')
    if not 0 <= config['overlay_alpha'] <= 1 or not 0 <= config['overlay_jpeg_quality'] <= 100:
        raise ValueError('Invalid visualization configuration')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/marisat_sam3_trial.yaml')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / args.config).read_text())
    validate_config(config)
    output = root / config['output_dir']
    output.mkdir(parents=True, exist_ok=False)
    dump_json(output / 'config_snapshot.json', config)
    summary = {'status': 'initializing', 'frames_inferred': 0, 'accuracy': None, 'user_acceptance': 'not_evaluated'}
    dump_json(output / 'summary.json', summary)
    try:
        for path_key, revision_key in [('source_dir', 'source_commit'), ('clip_dir', 'clip_commit')]:
            source = root / config[path_key]
            revision = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
            if revision != config[revision_key] or subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain']):
                raise ValueError(f'Expected clean pinned source: {path_key}')
            sys.path.insert(0, str(source))
        runtime = output / 'runtime'
        runtime.mkdir()
        os.environ.update(YOLO_AUTOINSTALL='false', YOLO_OFFLINE='true', YOLO_CONFIG_DIR=str(runtime),
                          HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1')
        import torch
        import torch.nn.functional as F
        import ultralytics
        from ultralytics.models.sam import SAM3SemanticPredictor
        from ultralytics.models.sam import build_sam3
        from ultralytics.utils import ops
        from ultralytics import settings
        settings.update({'sync': False})
        torch.set_num_threads(config['cpu_threads'])
        cv2.setNumThreads(config['cpu_threads'])
        if config['device'] == 'cpu' or not torch.cuda.is_available():
            raise RuntimeError('This GPU trial requires CUDA; no silent CPU fallback')
        checkpoint = root / config['prepared_checkpoint']
        provenance = restore_checkpoint(root / config['unpacked_checkpoint'], checkpoint)
        dump_json(output / 'checkpoint_provenance.json', provenance)
        state = torch.load(checkpoint, map_location='cpu', weights_only=True, mmap=True)
        if not isinstance(state, dict) or not state or any(not isinstance(v, torch.Tensor) for v in state.values()):
            raise ValueError('Expected a nonempty tensor-only state_dict')
        environment = {'date': config['experiment_date'], 'utc': datetime.now(timezone.utc).isoformat(),
                       'host': platform.node(), 'python': platform.python_version(), 'torch': torch.__version__,
                       'ultralytics': ultralytics.__version__, 'ultralytics_path': ultralytics.__file__,
                       'opencv': cv2.__version__, 'numpy': np.__version__, 'gpu': torch.cuda.get_device_name(0),
                       'gpu_total_mib_torch': torch.cuda.get_device_properties(0).total_memory / 2**20,
                       'driver_and_physical_vram': subprocess.check_output(['nvidia-smi',
                           '--query-gpu=driver_version,memory.total', '--format=csv,noheader'], text=True).strip(),
                       'state_entries': len(state), 'dtype_counts': dict(Counter(str(v.dtype) for v in state.values())),
                       'state_bytes': sum(v.numel()*v.element_size() for v in state.values()),
                       'runner_sha256': file_hash(Path(__file__)), 'config_sha256': file_hash(root / args.config),
                       'git_head': subprocess.check_output(['git','rev-parse','HEAD'], cwd=root, text=True).strip()}
        dump_json(output / 'environment.json', environment)
        audit = audit_state(state)
        dump_json(output / 'checkpoint_numeric_audit.json', audit)
        if not audit['all_finite']:
            summary['status'] = 'invalid_checkpoint_nonfinite'
            raise ValueError('Stored checkpoint has non-finite tensor values; refusing inference without altering weights')
        print('Checkpoint tensors:', environment['state_entries'], environment['dtype_counts'], flush=True)
        # Current default loader filters detector.*; the author exported bare keys.
        # Adapt ONLY deserialization, requiring every original key/shape to match.
        def loader(model, path, interactive=False):
            if Path(path) != checkpoint or interactive:
                raise ValueError('Unexpected architecture/checkpoint loading request')
            return strict_state_load(model, state)
        with patch.object(build_sam3, '_load_checkpoint', loader):
            model = build_sam3.build_sam3_image_model(str(checkpoint))
        summary.update(exact_state_match=True, state_entries=len(state))
        dump_json(output / 'summary.json', summary)
        del state
        predictor = SAM3SemanticPredictor(overrides=dict(model=str(checkpoint), task='segment', mode='predict',
            device=config['device'], half=config['half'], imgsz=config['image_size'],
            conf=min(config['thresholds']), save=False, verbose=False))
        predictor.setup_model(model=model, verbose=True)
        samples = collect_samples(root, yaml.safe_load((root/config['sample_config']).read_text())['inputs'])
        dump_json(output / 'sample_manifest.json', samples)
        summary['planned_frames'] = len(samples)
        device = predictor.device
        def synchronize():
            torch.cuda.synchronize(device)
        def infer(image):
            predictor.set_image(image)
            instances = []
            # Author likewise processes each text class separately; share only
            # current-frame image features. No temporal memory/tracking or NMS.
            for class_id, entry in enumerate(config['classes']):
                pred = predictor._inference_features(predictor.features, text=[entry['prompt']])
                scores = (pred['pred_logits'].sigmoid() * pred['presence_logit_dec'].sigmoid().unsqueeze(1)).flatten()
                invalid = [k for k in ('pred_logits', 'pred_boxes', 'pred_masks', 'presence_logit_dec')
                           if not torch.isfinite(pred[k]).all()]
                if invalid:
                    raise ValueError(f'Non-finite network outputs for {entry["name"]}: {invalid}; not zero detections')
                keep = scores > min(config['thresholds'])
                if keep.any():
                    boxes = ops.xywh2xyxy(pred['pred_boxes'].reshape(-1,4)[keep]).float()
                    boxes[:,[0,2]] *= image.shape[1]
                    boxes[:,[1,3]] *= image.shape[0]
                    logits = pred['pred_masks'].reshape(-1,*pred['pred_masks'].shape[-2:])[keep]
                    probabilities = F.interpolate(logits.sigmoid().float().unsqueeze(0), image.shape[:2],
                                                  mode='bilinear', align_corners=False)[0]
                    masks = (probabilities > config['mask_probability_threshold']).cpu().numpy()
                    for box,score,mask in zip(boxes.cpu().tolist(),scores[keep].cpu().tolist(),masks):
                        if not np.isfinite(box).all() or not np.isfinite(score) or mask.shape != image.shape[:2]:
                            raise ValueError('Invalid original-pixel output')
                        instances.append(({'class_id':class_id, 'class_name':entry['name'],
                                           'confidence':score, 'xyxy':box, 'mask_area_pixels':int(mask.sum())}, mask))
                    del probabilities, masks, logits, boxes
                del pred, scores, keep
            predictor.reset_image()
            instances.sort(key=lambda item:item[0]['confidence'], reverse=True)
            return instances[:config['max_instances_per_image']], len(instances)
        with torch.inference_mode(), torch.autocast('cuda', dtype=torch.float16, enabled=config.get('autocast', False)):
            for _ in range(config['warmup']):
                infer(cv2.imread(str(root/samples[0]['path'])))
            synchronize()
            torch.cuda.reset_peak_memory_stats(device)
            (output/'masks').mkdir()
            for t in config['thresholds']:
                (output/f'conf_{t:.2f}').mkdir()
            records=[]
            with (output/'predictions.jsonl').open('x') as handle:
                for sample in samples:
                    start=time.perf_counter()
                    image=cv2.imread(str(root/sample['path']))
                    synchronize()
                    predict_start=time.perf_counter()
                    instances,raw_count=infer(image)
                    synchronize()
                    predict_ms=(time.perf_counter()-predict_start)*1000
                    entries=[]
                    for i,(entry,mask) in enumerate(instances):
                        path=Path('masks')/f"{sample['id']}_{i:03d}.png"
                        if not cv2.imwrite(str(output/path),mask.astype(np.uint8)*255):
                            raise OSError('Cannot save mask')
                        entries.append({**entry,'instance_index':i,'mask_path':str(path)})
                    for t in config['thresholds']:
                        canvas=image.copy()
                        for i,(entry,mask) in enumerate(instances):
                            if entry['confidence'] < t:
                                continue
                            color=np.array([(53+i*67)%256,(181+i*41)%256,255],dtype=np.uint8)
                            canvas[mask]=np.rint((1-config['overlay_alpha'])*canvas[mask]+config['overlay_alpha']*color).astype(np.uint8)
                            xy=np.rint(entry['xyxy']).astype(int)
                            cv2.rectangle(canvas,tuple(xy[:2]),tuple(xy[2:]),tuple(int(c) for c in color),2)
                            cv2.putText(canvas,f"{entry['class_name']} #{i} {entry['confidence']:.2f}",
                                        (max(0,xy[0]),max(18,xy[1])),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,220,255),1)
                        if not cv2.imwrite(str(output/f'conf_{t:.2f}'/f"{sample['id']}.jpg"),canvas,
                                           [cv2.IMWRITE_JPEG_QUALITY,config['overlay_jpeg_quality']]):
                            raise OSError('Cannot save overlay')
                    record={'sample_id':sample['id'],'group':sample['group'],'image_shape':sample['shape'],
                            'instances':entries,'raw_candidates':raw_count,'truncated':raw_count>len(entries),
                            'predict_ms':predict_ms,'decode_predict_export_ms':(time.perf_counter()-start)*1000}
                    handle.write(json.dumps(record,allow_nan=False)+'\n')
                    handle.flush()
                    records.append(record)
                    summary['frames_inferred']=len(records)
                    dump_json(output/'summary.json',summary)
                    print(f"{sample['id']}: {len(entries)} masks, {predict_ms:.1f} ms",flush=True)
        local=[r for r in records if r['group']=='scene_water']
        summary.update(status='complete',mask_files=sum(len(r['instances']) for r in records),
            local_frames=len(local),predict_ms_local=distribution([r['predict_ms'] for r in local]),
            decode_predict_export_ms_local=distribution([r['decode_predict_export_ms'] for r in local]),
            truncated_frames=sum(r['truncated'] for r in records),
            local_counts={str(t):{'frames_with_candidates':sum(any(e['confidence']>=t for e in r['instances']) for r in local),
                                  'instances':sum(e['confidence']>=t for r in local for e in r['instances']),
                                  'large_ship_instances':sum(e['confidence']>=t and e['class_name'] in config['large_ship_classes']
                                                             for r in local for e in r['instances'])} for t in config['thresholds']},
            gpu_peak_allocated_mib=torch.cuda.max_memory_allocated(device)/2**20,
            gpu_peak_reserved_mib=torch.cuda.max_memory_reserved(device)/2**20)
        dump_json(output/'summary.json',summary)
    except Exception as exc:
        if summary['status'] != 'invalid_checkpoint_nonfinite':
            summary['status'] = 'blocked_or_failed'
        summary.update(error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
        dump_json(output/'summary.json',summary)
        raise


if __name__=='__main__':
    main()
