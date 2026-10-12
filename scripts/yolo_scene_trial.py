#!/usr/bin/env python3
"""Offline YOLO segmentation or explicit detection diagnostic; no ROS or training."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import time
import urllib.request

import numpy as np
import yaml


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def ensure_checkpoint(root, model_cfg):
    weights = root / model_cfg['weights']
    if not weights.exists():
        weights.parent.mkdir(parents=True, exist_ok=True)
        partial = weights.with_suffix(weights.suffix + '.part')
        if partial.exists():
            raise FileExistsError(f'Inspect incomplete download: {partial}')
        print(f'Downloading checkpoint: {model_cfg["download_url"]}', flush=True)
        with urllib.request.urlopen(model_cfg['download_url'], timeout=30) as response, partial.open('xb') as stream:
            while block := response.read(1024 * 1024):
                stream.write(block)
        if model_cfg['expected_sha256'] and sha256(partial) != model_cfg['expected_sha256']:
            raise ValueError('Downloaded checkpoint checksum mismatch')
        partial.rename(weights)
    if model_cfg['expected_sha256'] and sha256(weights) != model_cfg['expected_sha256']:
        raise ValueError('Checkpoint checksum mismatch')
    return weights


def classes_for_names(names, requested):
    if requested is None:
        return None  # Explicit all-class diagnostic, not an empty target list.
    ids = {name: int(index) for index, name in names.items()}
    if not requested or any(name not in ids for name in requested):
        raise ValueError(f'Unknown or empty classes: {requested}')
    return [ids[name] for name in requested]


def checked_masks(masks, count, image_shape):
    if masks is None:
        if count:
            raise ValueError('Boxes returned without instance masks')
        return np.zeros((0, *image_shape[:2]), dtype=np.uint8)
    array = np.asarray(masks)
    if array.shape != (count, *image_shape[:2]):
        raise ValueError(f'Masks {array.shape} are not in original image coordinates')
    if not np.isfinite(array).all():
        raise ValueError('Non-finite masks')
    return (array > .5).astype(np.uint8)


def validate_task(task, export_masks):
    if task not in ('segment', 'detect'):
        raise ValueError(f'Unsupported trial task: {task}')
    if task == 'detect' and export_masks:
        raise ValueError('Detection-only diagnostics cannot export instance masks')


def load_frames(root, manifests, controls=()):
    frames, seen = [], set()
    for filename in manifests:
        path = root / filename
        manifest = json.loads(path.read_text())
        for record in manifest['frames']:
            image = path.parent / record['file']
            # Audit and clip manifests can contain the same bag frame.
            identity = (manifest['bag_path'], record['bag_timestamp_ns'])
            if identity in seen:
                continue
            seen.add(identity)
            if not image.is_file():
                raise FileNotFoundError(image)
            frames.append(dict(record, image_path=str(image), manifest_path=filename,
                               frame_key=f'{path.parent.name}_{image.stem}'))
    if not frames:
        raise ValueError('No input frames')
    for filename in controls:
        image = root / filename
        if not image.is_file():
            raise FileNotFoundError(image)
        frames.append(dict(image_path=str(image), frame_key=f'control_{image.stem}',
                           control_image=True, source=filename))
    if len({frame['frame_key'] for frame in frames}) != len(frames):
        raise ValueError('Frame output keys are not unique')
    return frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/yolo_scene_trial.yaml')
    args = parser.parse_args()
    path = Path(args.config).resolve()
    root = path.parent.parent
    cfg = yaml.safe_load(path.read_text())
    bench, pred, model_cfg = cfg['benchmark'], cfg['prediction'], cfg['model']
    task = model_cfg.get('task', 'segment')
    validate_task(task, cfg['export']['masks'])
    if bench['measured_runs'] < 1 or bench['warmup_runs'] < 1 or bench['torch_cpu_threads'] < 1:
        raise ValueError('Benchmark counts and thread limit must be positive')
    if not pred['sizes'] or any(s <= 0 or s % 32 for s in pred['sizes']):
        raise ValueError('Input sizes must be nonempty positive multiples of stride 32')
    if not 0 < pred['confidence'] <= 1 or not 0 < pred['nms_iou'] <= 1:
        raise ValueError('Invalid prediction thresholds')
    if task == 'segment' and not pred['retina_masks']:
        raise ValueError('This trial exports full-resolution masks; retina_masks must be true')
    output = root / bench['output_dir']
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    frames = load_frames(root, cfg['dataset']['manifests'], cfg['dataset'].get('control_images', []))
    os.environ['YOLO_CONFIG_DIR'] = str(root / bench['settings_dir'])
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    os.environ['YOLO_VERBOSE'] = 'false'
    import cv2
    import torch
    import ultralytics
    from ultralytics import YOLO, settings
    settings.update({'sync': False})
    if ultralytics.__version__ != str(model_cfg['ultralytics_version']):
        raise ValueError(f'Unexpected Ultralytics version {ultralytics.__version__}')
    if not torch.cuda.is_available() or str(model_cfg['device']) != '0':
        raise RuntimeError('This hardware trial requires CUDA device 0; no CPU fallback')
    torch.set_num_threads(bench['torch_cpu_threads'])
    weights = ensure_checkpoint(root, model_cfg)
    digest = sha256(weights)
    start = time.perf_counter()
    model = YOLO(str(weights), task=task)
    if model.task != task:
        raise ValueError(f'Checkpoint task {model.task} does not match requested {task}')
    load_s = time.perf_counter() - start
    class_ids = classes_for_names(model.names, pred['class_names'])
    params = sum(p.numel() for p in model.model.parameters())
    options = dict(device=model_cfg['device'], half=model_cfg['half'], classes=class_ids,
                   conf=pred['confidence'], iou=pred['nms_iou'], max_det=pred['max_detections'],
                   rect=pred['rectangular_padding'], retina_masks=pred['retina_masks'],
                   verbose=False, save=False, augment=False, stream=False)
    output.mkdir(parents=True)
    provenance = dict(config=cfg, checkpoint_sha256=digest, checkpoint_bytes=weights.stat().st_size,
                      ultralytics=ultralytics.__version__, torch=torch.__version__, opencv=cv2.__version__,
                      device=torch.cuda.get_device_name(0), cuda_available_memory_mib=torch.cuda.get_device_properties(0).total_memory / 2**20,
                      parameters_before_predictor_fusion=params, cpu_threads=torch.get_num_threads(),
                      class_ids=class_ids, model_load_s=load_s, frame_count=len(frames),
                      inputs_used=['RGB'], training=False, quantization=False, task=task,
                      instance_segmentation=task == 'segment',
                      masks_in_original_coordinates=task == 'segment', frame_indices_are_not_track_ids=True,
                      timing_scope='decoded BGR image -> predict (preprocess+forward+postprocess) -> CPU result; synchronized CUDA',
                      excluded_from_predict_timing=['image decode', 'overlay rendering', 'PNG/JSON export', 'ROS/RViz'])
    (output / 'provenance.json').write_text(json.dumps(provenance, indent=2))
    all_reports = []
    if not 0 <= bench['warmup_image_index'] < len(frames):
        raise ValueError('Invalid warmup_image_index')
    first = cv2.imread(frames[bench['warmup_image_index']]['image_path'])
    if first is None:
        raise ValueError('Cannot decode warmup image')
    for size in pred['sizes']:
        folder = output / f'size_{size}'
        folder.mkdir()
        start = time.perf_counter()
        for _ in range(bench['warmup_runs']):
            with torch.inference_mode():
                warmup = model.predict(first, imgsz=size, **options)[0].cpu()
        torch.cuda.synchronize()
        del warmup
        warmup_s = time.perf_counter() - start
        torch.cuda.reset_peak_memory_stats()
        records = []
        for index, frame in enumerate(frames):
            start = time.perf_counter()
            image = cv2.imread(frame['image_path'])
            decode_ms = (time.perf_counter() - start) * 1000
            if image is None:
                raise ValueError(f'Cannot decode {frame["image_path"]}')
            durations = []
            cpu_start = time.process_time()
            for _ in range(bench['measured_runs']):
                torch.cuda.synchronize()
                start = time.perf_counter()
                with torch.inference_mode():
                    result = model.predict(image, imgsz=size, **options)[0].cpu()
                torch.cuda.synchronize()
                durations.append((time.perf_counter() - start) * 1000)
            cpu_ms_per_run = (time.process_time() - cpu_start) * 1000 / bench['measured_runs']
            boxes = result.boxes.xyxy.numpy()
            scores = result.boxes.conf.numpy()
            labels = result.boxes.cls.numpy().astype(int)
            masks = (checked_masks(None if result.masks is None else result.masks.data.numpy(), len(boxes), image.shape)
                     if task == 'segment' else None)
            detections = [dict(frame_instance_index=i, class_id=int(labels[i]), class_name=model.names[int(labels[i])],
                               confidence=float(scores[i]), bbox_xyxy=boxes[i].tolist(),
                               **(dict(mask_area_px=int(masks[i].sum()), mask_area_fraction=float(masks[i].mean()))
                                  if masks is not None else {}))
                          for i in range(len(boxes))]
            record = dict(frame, decoded_shape=list(image.shape), detection_count=len(boxes),
                          detections=detections, measured_predict_ms=durations,
                          median_predict_ms=float(np.median(durations)), image_decode_ms=decode_ms,
                          cpu_process_ms_per_predict=cpu_ms_per_run, library_speed_ms=result.speed,
                          task=task, mask_shape=None if masks is None else list(masks.shape))
            records.append(record)
            sample = folder / frame['frame_key']
            sample.mkdir()
            start = time.perf_counter()
            if cfg['export']['annotated']:
                annotated = result.plot(line_width=cfg['export']['annotation_line_width'])
                if not cv2.imwrite(str(sample / 'annotated.jpg'), annotated):
                    raise OSError('Cannot write annotation')
            if cfg['export']['masks'] and masks is not None:
                for mask_index, mask in enumerate(masks):
                    if not cv2.imwrite(str(sample / f'mask_{mask_index:02d}.png'), mask * 255):
                        raise OSError('Cannot write mask')
            record['visual_export_ms'] = (time.perf_counter() - start) * 1000
            (sample / 'prediction.json').write_text(json.dumps(record, indent=2))
            if index % 10 == 0:
                print(f'{size}: frame {index + 1}/{len(frames)}, {len(boxes)} detection(s), {np.median(durations):.1f} ms', flush=True)
        timings = [r['median_predict_ms'] for r in records]
        report = dict(size=size, task=task, instance_segmentation=task == 'segment',
                      fixed_tensor_shape=[1, 3, size, size] if not pred['rectangular_padding'] else None,
                      warmup_s=warmup_s, median_predict_ms=float(np.median(timings)),
                      p95_predict_ms=float(np.quantile(timings, .95)),
                      mean_predict_ms=float(np.mean(timings)),
                      images_per_second_excluding_decode_export=1000 / float(np.mean(timings)),
                      median_decode_ms=float(np.median([r['image_decode_ms'] for r in records])),
                      median_visual_export_ms=float(np.median([r['visual_export_ms'] for r in records])),
                      peak_allocated_mib=torch.cuda.max_memory_allocated() / 2**20,
                      peak_reserved_mib=torch.cuda.max_memory_reserved() / 2**20,
                      process_peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
                      frames_with_boat_detections=sum(any(d['class_name'] == 'boat' for d in r['detections']) for r in records),
                      frames_with_detections=sum(r['detection_count'] > 0 for r in records),
                      total_detections=sum(r['detection_count'] for r in records),
                      ground_truth_available=False, accuracy_metrics=None, records=records)
        (folder / 'report.json').write_text(json.dumps(report, indent=2))
        all_reports.append({k: v for k, v in report.items() if k != 'records'})
        (output / 'summary.json').write_text(json.dumps(all_reports, indent=2))
        print(f'{size}: median {report["median_predict_ms"]:.1f} ms, p95 {report["p95_predict_ms"]:.1f} ms; '
              f'{report["frames_with_boat_detections"]}/{len(frames)} frames with boat', flush=True)
        torch.cuda.empty_cache()


if __name__ == '__main__':
    main()
