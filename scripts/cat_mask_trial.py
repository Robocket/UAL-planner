#!/usr/bin/env python3
"""Offline CAT Mask R-CNN evaluation on existing samples; never trains."""
import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

import cv2
import numpy as np
import yaml

from ship_detection_trial import collect_samples, distribution, dump_json, sha256, target_ids


def validate_config(config):
    if config['warmup'] < 1 or config['cpu_threads'] < 1:
        raise ValueError('Positive warmup/cpu_threads required')
    if len(config['resize']) != 2 or any(s < 32 for s in config['resize']):
        raise ValueError('Resize must contain positive width/height >= 32')
    if not config['thresholds'] or any(not 0 < t <= 1 for t in config['thresholds']):
        raise ValueError('Confidence thresholds must be in (0,1]')
    if not 0 <= config['model_score_threshold'] <= min(config['thresholds']):
        raise ValueError('Model threshold must not discard exported thresholds')
    if config['nms']['type'] != 'soft_nms' or not 0 < config['nms']['iou_threshold'] <= 1:
        raise ValueError('Use the configured author soft NMS')
    if config['max_per_img'] < 1 or not 0 < config['mask_threshold'] <= 1:
        raise ValueError('Invalid mask/output limits')
    if not 0 <= config['overlay_alpha'] <= 1 or not 0 <= config['overlay_jpeg_quality'] <= 100:
        raise ValueError('Invalid visualization configuration')


def decode_instances(boxes, scores, labels, masks, classes, targets, shape, minimum):
    """Require original-pixel per-instance masks, never fill rectangles as masks."""
    count = len(scores)
    if boxes.shape != (count, 4) or labels.shape != (count,) or masks.shape != (count, *shape[:2]):
        raise ValueError('Inconsistent predictions or masks not at original image resolution')
    if masks.dtype != np.bool_:
        raise ValueError('Expected binary model masks')
    if not np.isfinite(boxes).all() or not np.isfinite(scores).all() or np.any((scores < 0) | (scores > 1)):
        raise ValueError('Invalid prediction values')
    if np.any((labels < 0) | (labels >= len(classes))) or not np.issubdtype(labels.dtype, np.integer):
        raise ValueError('Invalid class labels')
    if np.any(boxes[:, 2:] < boxes[:, :2]):
        raise ValueError('Inverted box coordinates')
    selected = []
    for index in range(count):
        label = int(labels[index])
        if label in targets and scores[index] >= minimum:
            selected.append(({'class_id': label, 'class_name': classes[label],
                              'confidence': float(scores[index]), 'xyxy': boxes[index].astype(float).tolist(),
                              'mask_area_pixels': int(masks[index].sum())}, masks[index]))
    return selected


def overlay(image, instances, threshold, alpha):
    canvas = image.copy()
    for index, (entry, mask) in enumerate(instances):
        if entry['confidence'] < threshold:
            continue
        color = np.array(((53 + index * 67) % 256, (181 + index * 41) % 256, 255), dtype=np.uint8)
        canvas[mask] = np.rint((1 - alpha) * canvas[mask] + alpha * color).astype(np.uint8)
        points = np.rint(entry['xyxy']).astype(int)
        cv2.rectangle(canvas, tuple(points[:2]), tuple(points[2:]), tuple(int(x) for x in color), 2)
        x, y = points[:2]
        cv2.putText(canvas, f"ship #{index} {entry['confidence']:.2f}", (max(0, x), max(18, y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, tuple(int(x) for x in color), 1)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/cat_mask_trial.yaml')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / args.config).read_text())
    validate_config(config)
    source = root / config['source_dir']
    commit = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != config['source_commit'] or subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain']):
        raise ValueError('CAT source must match the clean pinned author revision')
    weight = root / config['weight']['path']
    if weight.stat().st_size != config['weight']['bytes'] or sha256(weight) != config['weight']['sha256']:
        raise ValueError('CAT publisher weight size/SHA-256 mismatch')
    samples = collect_samples(root, yaml.safe_load((root / config['sample_config']).read_text())['inputs'])
    output = root / config['output_dir']
    output.mkdir(parents=True, exist_ok=False)
    dump_json(output / 'config_snapshot.json', config)
    dump_json(output / 'sample_manifest.json', samples)
    summary = {'status': 'initializing', 'planned_frames': len(samples), 'frames_inferred': 0,
               'accuracy': None, 'user_acceptance': 'not_evaluated'}
    dump_json(output / 'summary.json', summary)
    try:
        import torch
        from mmengine.config import Config
        from mmengine.dataset import Compose
        from mmdet.apis import init_detector, inference_detector
        from mmdet.utils import register_all_modules
        register_all_modules()
        sys.path.insert(0, str(source))
        torch.set_num_threads(config['cpu_threads'])
        cv2.setNumThreads(config['cpu_threads'])
        cuda = config['device'].startswith('cuda')
        if cuda and not torch.cuda.is_available():
            raise RuntimeError('CUDA requested but unavailable; no silent fallback')
        def synchronize():
            if cuda:
                torch.cuda.synchronize(config['device'])
        environment = {'date': config['experiment_date'], 'utc': datetime.now(timezone.utc).isoformat(),
                       'host': platform.node(), 'python': platform.python_version(),
                       'packages': {n: importlib.metadata.version(n) for n in
                                    ('torch', 'torchvision', 'mmcv', 'mmdet', 'mmengine', 'nncore', 'numpy')},
                       'device': config['device'], 'gpu': torch.cuda.get_device_name(config['device']) if cuda else None,
                       'gpu_driver': subprocess.check_output(['nvidia-smi', '--query-gpu=driver_version,memory.total',
                                                             '--format=csv,noheader'], text=True).strip() if cuda else None,
                       'weight_sha256': config['weight']['sha256'], 'source_commit': commit,
                       'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
                       'runner_sha256': sha256(Path(__file__)), 'config_sha256': sha256(root / args.config)}
        dump_json(output / 'environment.json', environment)
        cfg = Config.fromfile(str(source / config['model_config']))
        # Full checkpoint supplied: do not fetch an unrelated ImageNet backbone.
        cfg.model.backbone.init_cfg = None
        cfg.model.test_cfg.rcnn.update(score_thr=config['model_score_threshold'],
                                      nms=config['nms'], max_per_img=config['max_per_img'],
                                      mask_thr_binary=config['mask_threshold'])
        # No annotations exist for these images; retain author resize, omit GT loading.
        pipeline = Compose([dict(type='mmdet.LoadImageFromNDArray'),
                            dict(type='mmdet.Resize', scale=tuple(config['resize']), keep_ratio=True),
                            dict(type='mmdet.PackDetInputs', meta_keys=('img_id', 'ori_shape', 'img_shape', 'scale_factor'))])
        dump_json(output / 'effective_model_config.json', cfg.to_dict())
        model = init_detector(cfg, str(weight), device=config['device'])
        # Author checkpoint lacks class metadata. Use its pinned iSAID dataset,
        # not init_detector's generic COCO fallback (ship index differs).
        from datasets.isaid import ISAIDDataset
        classes = tuple(ISAIDDataset.METAINFO['classes'])
        if len(classes) != cfg.model.roi_head.mask_head.num_classes:
            raise ValueError('Author dataset classes do not match the model heads')
        model.dataset_meta['classes'] = classes
        state = torch.load(weight, map_location='cpu', weights_only=False)['state_dict']
        expected = model.state_dict()
        if set(state) != set(expected) or any(state[key].shape != expected[key].shape for key in state):
            raise ValueError('Checkpoint keys/shapes do not exactly match the author architecture')
        del state, expected
        targets = target_ids(dict(enumerate(classes)), config['target_classes'])
        summary.update(classes=classes, target_class_ids=sorted(targets),
                       class_mapping_source='Pinned CATNet datasets/isaid.py; checkpoint meta has no classes',
                       exact_checkpoint_key_shape_match=True)
        first = cv2.imread(str(root / samples[0]['path']))
        with torch.inference_mode():
            for _ in range(config['warmup']):
                inference_detector(model, first, test_pipeline=pipeline)
        synchronize()
        if cuda:
            torch.cuda.reset_peak_memory_stats(config['device'])
        (output / 'masks').mkdir()
        for threshold in config['thresholds']:
            (output / f'conf_{threshold:.2f}').mkdir()
        records = []
        with (output / 'predictions.jsonl').open('x') as handle:
            for sample in samples:
                start = time.perf_counter()
                image = cv2.imread(str(root / sample['path']))
                if image is None:
                    raise ValueError('Cannot decode sample')
                synchronize()
                predict_start = time.perf_counter()
                with torch.inference_mode():
                    result = inference_detector(model, image, test_pipeline=pipeline)
                synchronize()
                predict_ms = (time.perf_counter() - predict_start) * 1000
                pred = result.pred_instances.cpu().numpy()
                instances = decode_instances(pred.bboxes, pred.scores, pred.labels, pred.masks, classes,
                                             targets, image.shape, min(config['thresholds']))
                entries = []
                for index, (entry, mask) in enumerate(instances):
                    mask_path = Path('masks') / f"{sample['id']}_{index:03d}.png"
                    if not cv2.imwrite(str(output / mask_path), mask.astype(np.uint8) * 255):
                        raise OSError('Cannot save mask')
                    entries.append({**entry, 'instance_index': index, 'mask_path': str(mask_path)})
                counts = {}
                for threshold in config['thresholds']:
                    counts[str(threshold)] = sum(entry['confidence'] >= threshold for entry in entries)
                    canvas = overlay(image, instances, threshold, config['overlay_alpha'])
                    if not cv2.imwrite(str(output / f'conf_{threshold:.2f}' / f"{sample['id']}.jpg"), canvas,
                                       [cv2.IMWRITE_JPEG_QUALITY, config['overlay_jpeg_quality']]):
                        raise OSError('Cannot save overlay')
                record = {'sample_id': sample['id'], 'group': sample['group'], 'image_shape': sample['shape'],
                          'instances': entries, 'counts': counts, 'predict_ms': predict_ms,
                          'decode_predict_export_ms': (time.perf_counter() - start) * 1000}
                handle.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
                handle.flush()
                records.append(record)
                summary['frames_inferred'] = len(records)
                dump_json(output / 'summary.json', summary)
                print(f"{sample['id']}: {len(entries)} ship masks, {predict_ms:.1f} ms", flush=True)
                del result, pred, instances
        local = [r for r in records if r['group'] == 'scene_water']
        summary.update(status='complete', local_frames=len(local),
                       predict_ms_local=distribution([r['predict_ms'] for r in local]),
                       decode_predict_export_ms_local=distribution([r['decode_predict_export_ms'] for r in local]),
                       mask_files=sum(len(r['instances']) for r in records),
                       local_counts={str(t): {'frames_with_candidates': sum(r['counts'][str(t)] > 0 for r in local),
                                             'instances': sum(r['counts'][str(t)] for r in local)}
                                     for t in config['thresholds']},
                       gpu_peak_allocated_mib=torch.cuda.max_memory_allocated(config['device']) / 2**20 if cuda else None,
                       gpu_peak_reserved_mib=torch.cuda.max_memory_reserved(config['device']) / 2**20 if cuda else None)
        dump_json(output / 'summary.json', summary)
    except Exception as exc:
        summary.update(status='blocked_or_failed', error=f'{type(exc).__name__}: {exc}', traceback=traceback.format_exc())
        dump_json(output / 'summary.json', summary)
        raise


if __name__ == '__main__':
    main()
