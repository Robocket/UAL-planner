#!/usr/bin/env python3
"""Camera-only offline box comparison; does not train or produce instance masks."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import gc
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import time

import cv2
import numpy as np
import yaml


def dump_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def collect_samples(root, inputs):
    """Deduplicate decoded pixels, retaining every source manifest entry."""
    samples, by_hash = [], {}
    candidates = []
    for name in inputs['manifests']:
        manifest = root / name
        document = json.loads(manifest.read_text())
        for frame in document['frames']:
            candidates.append((manifest.parent / frame['file'], 'scene_water',
                               {'manifest': name, 'bag_path': document.get('bag_path'), **frame}))
    candidates.extend((root / name, 'control', {'file': name}) for name in inputs['controls'])
    for path, group, origin in candidates:
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f'Cannot decode input image: {path}')
        digest = hashlib.sha256(str(image.shape).encode() + image.tobytes()).hexdigest()
        source = {'path': str(path.relative_to(root)), 'metadata': origin}
        if digest in by_hash:
            by_hash[digest]['sources'].append(source)
            continue
        sample = {'id': f'{len(samples):03d}', 'path': str(path.relative_to(root)),
                  'group': group, 'shape': list(image.shape), 'pixel_sha256': digest,
                  'file_sha256': sha256(path), 'sources': [source]}
        by_hash[digest] = sample
        samples.append(sample)
    if not samples:
        raise ValueError('No samples')
    return samples


def target_ids(names, wanted):
    normalized = {int(k): str(v).strip().lower() for k, v in names.items()}
    missing = set(wanted) - set(normalized.values())
    if missing:
        raise ValueError(f'Missing target classes {missing}; actual names: {names}')
    return {key for key, name in normalized.items() if name in wanted}


def decode_detections(result, task, targets):
    detections = []
    boxes = result.obb if task == 'obb' else result.boxes
    if boxes is None:
        raise ValueError(f'Missing {task} output')
    for box in boxes:
        cls = int(box.cls.item())
        if task == 'obb':
            polygon = box.xyxyxyxy[0].detach().cpu().numpy().astype(float)
            xyxy = [*polygon.min(axis=0), *polygon.max(axis=0)]
        else:
            polygon = None
            xyxy = box.xyxy[0].detach().cpu().tolist()
        if not np.isfinite(xyxy).all() or not np.isfinite(float(box.conf.item())):
            raise ValueError('Non-finite model output')
        detections.append({'class_id': cls, 'class_name': result.names[cls],
                           'confidence': float(box.conf.item()), 'target': cls in targets,
                           'xyxy': [float(x) for x in xyxy],
                           'polygon': polygon.tolist() if polygon is not None else None})
    return detections


def select_targets(detections, threshold):
    return [item for item in detections if item['target'] and item['confidence'] >= threshold]


def annotate(image, detections):
    canvas = image.copy()
    for item in detections:
        color = (0, 220, 255)
        if item['polygon'] is not None:
            cv2.polylines(canvas, [np.rint(item['polygon']).astype(np.int32)], True, color, 2)
        else:
            x1, y1, x2, y2 = np.rint(item['xyxy']).astype(int)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        x, y = np.rint(item['xyxy'][:2]).astype(int)
        x, y = max(0, x), max(18, min(canvas.shape[0] - 5, y))
        cv2.putText(canvas, f"{item['class_name']} {item['confidence']:.2f}", (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return canvas


def distribution(values):
    if not values:
        return None
    return {'median': float(np.median(values)), 'p95': float(np.percentile(values, 95)),
            'min': float(min(values)), 'max': float(max(values))}


def make_sheet(case_dir, threshold, samples, config):
    indices = [idx for idx in config['indices'] if idx < len(samples)]
    if not indices:
        return
    cols, width, height = config['columns'], config['tile_width'], config['tile_height']
    sheet = np.full((((len(indices) + cols - 1) // cols) * height, cols * width, 3), 25, np.uint8)
    for index, sample_index in enumerate(indices):
        sample = samples[sample_index]
        image = cv2.imread(str(case_dir / f'conf_{threshold:.2f}' / f"{sample['id']}.jpg"))
        scale = min(width / image.shape[1], (height - 36) / image.shape[0])
        resized = cv2.resize(image, (round(image.shape[1] * scale), round(image.shape[0] * scale)))
        x, y = (index % cols) * width, (index // cols) * height
        sheet[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
        text = f"{sample['id']} {sample['group']} {Path(sample['path']).parent.name}"
        cv2.putText(sheet, text, (x + 4, y + height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
    if not cv2.imwrite(str(case_dir / f'contact_{threshold:.2f}.jpg'), sheet):
        raise OSError('Cannot save contact sheet')


def validate_config(config):
    if config['warmup'] < 1 or config['cpu_threads'] < 1:
        raise ValueError('warmup and cpu_threads must be positive')
    if not config['thresholds'] or any(not 0 < t <= 1 for t in config['thresholds']):
        raise ValueError('confidence thresholds must be in (0,1]')
    if not 0 < config['iou'] <= 1 or config['max_det'] < 1:
        raise ValueError('invalid NMS configuration')
    for model in config['models']:
        if model['task'] not in ('detect', 'obb'):
            raise ValueError('Only detection/OBB models are supported')
        if Path(model['file']).name != model['file']:
            raise ValueError('weights filenames must be flat')
        if any(size < 32 or size % 32 for size in model['image_sizes']):
            raise ValueError('image_sizes must be positive multiples of YOLO stride 32')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/ship_detection_trial.yaml')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    config_path = (root / args.config).resolve()
    config = yaml.safe_load(config_path.read_text())
    validate_config(config)
    samples = collect_samples(root, config['inputs'])
    weights = root / config['weights_dir']
    provenance = json.loads((weights / 'download_provenance.json').read_text())
    hashes = {entry['file']: entry['sha256'] for entry in provenance['files']}
    # Auxiliary publisher documents may still be downloading. Each selected weight
    # itself must already be recorded and pass its full SHA-256 check below.
    if any(entry['file'] not in hashes for entry in config['models']):
        raise ValueError('Selected weight download is incomplete')
    for entry in config['models']:
        if sha256(weights / entry['file']) != hashes[entry['file']]:
            raise ValueError(f"Weight hash mismatch: {entry['file']}")
    output = root / config['output_dir']
    output.mkdir(parents=True, exist_ok=False)
    dump_json(output / 'config_snapshot.json', config)
    dump_json(output / 'sample_manifest.json', samples)
    # Keep inference offline and isolate Ultralytics settings from the user's global file.
    runtime = output / 'runtime'
    runtime.mkdir()
    os.environ['YOLO_CONFIG_DIR'] = str(runtime)
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    os.environ['YOLO_OFFLINE'] = 'true'
    import torch
    import ultralytics
    from ultralytics import YOLO, settings
    settings.update({'sync': False})
    torch.set_num_threads(config['cpu_threads'])
    cv2.setNumThreads(config['cpu_threads'])
    use_cuda = config['device'] != 'cpu'
    if use_cuda and not torch.cuda.is_available():
        raise RuntimeError('CUDA requested but unavailable; no silent CPU fallback')

    def synchronize():
        if use_cuda:
            torch.cuda.synchronize(int(config['device']))

    environment = {'utc': datetime.now(timezone.utc).isoformat(), 'host': platform.node(),
                   'platform': platform.platform(), 'python': platform.python_version(),
                   'torch': torch.__version__, 'ultralytics': ultralytics.__version__,
                   'opencv': cv2.__version__, 'config_sha256': sha256(config_path),
                   'script_sha256': sha256(Path(__file__)), 'weight_provenance': provenance,
                   'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
                   'git_status': subprocess.check_output(['git', 'status', '--short'], cwd=root, text=True),
                   'cpu': next((line.strip() for line in Path('/proc/cpuinfo').read_text().splitlines()
                                if line.startswith('model name')), platform.processor())}
    if use_cuda:
        properties = torch.cuda.get_device_properties(int(config['device']))
        environment.update(gpu=properties.name, torch_visible_vram_bytes=properties.total_memory)
        environment['nvidia_smi'] = subprocess.check_output(
            ['nvidia-smi', '--query-gpu=name,driver_version,memory.total,memory.used', '--format=csv,noheader'], text=True).strip()
    dump_json(output / 'environment.json', environment)
    summaries, failures = [], []
    for spec in config['models']:
        for image_size in spec['image_sizes']:
            case_id = f"{spec['id']}_{image_size}"
            case_dir = output / case_id
            case_dir.mkdir()
            model = None
            try:
                start = time.perf_counter()
                model = YOLO(str(weights / spec['file']))
                if model.task != spec['task']:
                    raise ValueError(f'Unexpected task {model.task}')
                targets = target_ids(model.names, spec['target_classes'])
                checkpoint_train_imgsz = model.ckpt.get('train_args', {}).get('imgsz')
                cpu_load_s = time.perf_counter() - start
                options = dict(imgsz=image_size, conf=min(config['thresholds']), iou=config['iou'],
                               device=config['device'], half=config['half'], batch=1, rect=False,
                               max_det=config['max_det'], agnostic_nms=config['agnostic_nms'],
                               augment=False, save=False, verbose=False)
                first = cv2.imread(str(root / samples[0]['path']))
                start = time.perf_counter()
                for _ in range(config['warmup']):
                    model.predict(first, **options)
                synchronize()
                warmup_s = time.perf_counter() - start
                if use_cuda:
                    torch.cuda.reset_peak_memory_stats(int(config['device']))
                for threshold in config['thresholds']:
                    (case_dir / f'conf_{threshold:.2f}').mkdir()
                rows = []
                with (case_dir / 'predictions.jsonl').open('x') as prediction_file:
                    for sample in samples:
                        pipeline_start = time.perf_counter()
                        image = cv2.imread(str(root / sample['path']))
                        synchronize()
                        prediction_start = time.perf_counter()
                        result = model.predict(image, **options)[0]
                        synchronize()
                        predict_ms = (time.perf_counter() - prediction_start) * 1000
                        detections = decode_detections(result, spec['task'], targets)
                        counts = {}
                        for threshold in config['thresholds']:
                            selected = select_targets(detections, threshold)
                            counts[f'{threshold:.2f}'] = len(selected)
                            canvas = annotate(image, selected)
                            if not cv2.imwrite(str(case_dir / f'conf_{threshold:.2f}' / f"{sample['id']}.jpg"), canvas,
                                               [cv2.IMWRITE_JPEG_QUALITY, config['overlay_jpeg_quality']]):
                                raise OSError('Cannot save annotated image')
                        row = {'sample_id': sample['id'], 'group': sample['group'], 'shape': sample['shape'],
                               'detections': detections, 'target_counts': counts,
                               'predict_wall_ms': predict_ms, 'ultralytics_speed_ms': result.speed,
                               'decode_predict_draw_save_ms': (time.perf_counter() - pipeline_start) * 1000}
                        prediction_file.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
                        rows.append(row)
                summary = {'case': case_id, 'model_sha256': hashes[spec['file']], 'task': model.task,
                           'class_names': model.names, 'parameters': sum(p.numel() for p in model.model.parameters()),
                           'checkpoint_train_imgsz': checkpoint_train_imgsz,
                           'cpu_load_s': cpu_load_s, 'warmup_s': warmup_s, 'image_size': image_size,
                           'metrics_note': 'Counts are not accuracy. Predict wall excludes decode/drawing/saving. Pipeline includes both confidence overlays, not ROS or contact sheets.',
                           'groups': {}}
                for group in ('scene_water', 'control', 'all'):
                    selected = [row for row in rows if group == 'all' or row['group'] == group]
                    summary['groups'][group] = {'frames': len(selected),
                        'predict_wall_ms': distribution([r['predict_wall_ms'] for r in selected]),
                        'ultralytics_inference_ms': distribution([r['ultralytics_speed_ms']['inference'] for r in selected]),
                        'decode_predict_draw_save_ms': distribution([r['decode_predict_draw_save_ms'] for r in selected]),
                        'thresholds': {f'{t:.2f}': {'frames_with_targets': sum(r['target_counts'][f'{t:.2f}'] > 0 for r in selected),
                                                    'target_boxes': sum(r['target_counts'][f'{t:.2f}'] for r in selected),
                                                    'classes': dict(Counter(d['class_name'] for r in selected for d in select_targets(r['detections'], t)))}
                                       for t in config['thresholds']}}
                if use_cuda:
                    summary['torch_peak_allocated_mib'] = torch.cuda.max_memory_allocated(int(config['device'])) / 2**20
                    summary['torch_peak_reserved_mib'] = torch.cuda.max_memory_reserved(int(config['device'])) / 2**20
                dump_json(case_dir / 'summary.json', summary)
                for threshold in config['thresholds']:
                    make_sheet(case_dir, threshold, samples, config['contact_sheet'])
                summaries.append(summary)
                print(json.dumps({'complete': case_id, **summary['groups']['scene_water']}, ensure_ascii=False), flush=True)
            except Exception as exc:
                failure = {'case': case_id, 'error': f'{type(exc).__name__}: {exc}'}
                failures.append(failure)
                dump_json(case_dir / 'failure.json', failure)
                print(json.dumps(failure), flush=True)
            finally:
                if model is not None:
                    del model
                # Release result references before collecting CUDA allocator caches.
                if 'result' in locals():
                    del result
                gc.collect()
                if use_cuda:
                    torch.cuda.empty_cache()
            dump_json(output / 'summary.json', {'cases': summaries, 'failures': failures})
    if failures:
        raise SystemExit('One or more experiments failed; see summary.json. Existing output is retained.')


if __name__ == '__main__':
    main()
