#!/usr/bin/env python3
"""Offline marine detector + GrabCut/MobileSAM, with automatic prompts only."""
import argparse
import json
import os
from pathlib import Path
import resource
import time

import cv2
import numpy as np
import yaml

from yolo_scene_trial import checked_masks, classes_for_names, ensure_checkpoint, load_frames, sha256


def clipped_box(box, shape):
    if len(box) != 4 or not np.isfinite(box).all():
        raise ValueError('Invalid box')
    height, width = shape[:2]
    x0, y0 = np.floor(box[:2]).astype(int)
    x1, y1 = np.ceil(box[2:]).astype(int)
    x0, x1 = np.clip([x0, x1], 0, width)
    y0, y1 = np.clip([y0, y1], 0, height)
    if x1 <= x0 or y1 <= y0:
        raise ValueError('Empty clipped box')
    return int(x0), int(y0), int(x1), int(y1)


def grabcut_mask(image, box, cfg):
    """A candidate foreground mask, never a semantic confirmation of boat class."""
    x0, y0, x1, y1 = clipped_box(box, image.shape)
    height, width = image.shape[:2]
    margin = cfg['crop_context_fraction']
    dx, dy = int(np.ceil((x1 - x0) * margin)), int(np.ceil((y1 - y0) * margin))
    left, top = max(0, x0 - dx), max(0, y0 - dy)
    right, bottom = min(width, x1 + dx), min(height, y1 + dy)
    crop = image[top:bottom, left:right]
    scale = min(1.0, cfg['max_crop_edge'] / max(crop.shape[:2]))
    dst = (max(3, round(crop.shape[1] * scale)), max(3, round(crop.shape[0] * scale)))
    small = cv2.resize(crop, dst, interpolation=cv2.INTER_AREA)
    sx, sy = dst[0] / crop.shape[1], dst[1] / crop.shape[0]
    ax, ay = int(np.floor((x0 - left) * sx)), int(np.floor((y0 - top) * sy))
    bx, by = int(np.ceil((x1 - left) * sx)), int(np.ceil((y1 - top) * sy))
    labels = np.full(small.shape[:2], cv2.GC_PR_BGD, np.uint8)
    labels[ay:by, ax:bx] = cv2.GC_PR_FGD
    border = cfg['border_background_px']
    if 2 * border >= min(labels.shape):
        return np.zeros((height, width), np.uint8), 'crop_too_small'
    # At clipped image edges this may remove foreground; report truncation separately.
    labels[:border] = labels[-border:] = cv2.GC_BGD
    labels[:, :border] = labels[:, -border:] = cv2.GC_BGD
    if not np.any(labels == cv2.GC_PR_FGD):
        return np.zeros((height, width), np.uint8), 'no_foreground_seed'
    cv2.setRNGSeed(cfg['rng_seed'])
    try:
        cv2.grabCut(small, labels, None, np.zeros((1, 65)), np.zeros((1, 65)),
                    cfg['iterations'], cv2.GC_INIT_WITH_MASK)
    except cv2.error as error:
        return np.zeros((height, width), np.uint8), f'grabcut_error: {error.code}'
    mask = np.zeros((height, width), np.uint8)
    foreground = np.isin(labels, [cv2.GC_FGD, cv2.GC_PR_FGD]).astype(np.uint8)
    mask[top:bottom, left:right] = cv2.resize(foreground, (right - left, bottom - top),
                                          interpolation=cv2.INTER_NEAREST)
    return mask, 'candidate' if mask.any() else 'empty'


def mask_box(mask):
    if not mask.any():
        return None
    x, y, w, h = cv2.boundingRect(mask)
    return [x, y, x + w, y + h]


def count_nonempty_masks(masks):
    # Convert numpy.bool_ before aggregation so the report remains JSON-serializable.
    return sum(bool(mask.any()) for mask in masks)


def validate_config(cfg):
    bench, gc = cfg['benchmark'], cfg['grabcut']
    if not bench['methods'] or len(set(bench['methods'])) != len(bench['methods']):
        raise ValueError('Methods must be nonempty and unique')
    if any(method not in ('grabcut', 'mobilesam') for method in bench['methods']):
        raise ValueError('Unsupported mask method')
    if min(bench[k] for k in ('warmup_runs', 'measured_runs', 'torch_cpu_threads', 'opencv_cpu_threads')) < 1:
        raise ValueError('Benchmark counts must be positive')
    if gc['iterations'] < 1 or gc['max_crop_edge'] < 3 or gc['border_background_px'] < 1 or gc['crop_context_fraction'] < 0:
        raise ValueError('Invalid GrabCut settings')
    if cfg['mobile_sam']['size'] != 1024:
        raise ValueError('This MobileSAM encoder requires native 1024 input geometry')
    if not 0 <= cfg['mobile_sam']['quality_threshold'] <= 1:
        raise ValueError('Invalid SAM quality threshold')
    det = cfg['detector']
    if det['size'] <= 0 or det['size'] % 32 or det['max_detections'] < 1:
        raise ValueError('Invalid detector size or max_detections')
    if not 0 < det['confidence'] <= 1 or not 0 < det['nms_iou'] <= 1:
        raise ValueError('Invalid detection thresholds')
    if not 0 <= cfg['export']['overlay_alpha'] <= 1 or cfg['export']['box_line_width'] < 1:
        raise ValueError('Invalid annotation settings')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/ship_hybrid_trial.yaml')
    args = parser.parse_args()
    path = Path(args.config).resolve()
    root = path.parent.parent
    cfg = yaml.safe_load(path.read_text())
    validate_config(cfg)
    bench, det = cfg['benchmark'], cfg['detector']
    output = root / bench['output_dir']
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    frames = load_frames(root, cfg['dataset']['manifests'], cfg['dataset'].get('control_images', []))
    if not 0 <= bench['warmup_image_index'] < len(frames):
        raise ValueError('Invalid warmup frame')
    os.environ['YOLO_CONFIG_DIR'] = str(root / bench['settings_dir'])
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    os.environ['YOLO_VERBOSE'] = 'false'
    import torch
    import ultralytics
    from ultralytics import SAM, YOLO, settings
    settings.update({'sync': False})
    if ultralytics.__version__ != str(bench['ultralytics_version']):
        raise ValueError('Unexpected Ultralytics version')
    if not torch.cuda.is_available() or str(bench['device']) != '0':
        raise RuntimeError('This test requires CUDA 0; no CPU fallback')
    torch.set_num_threads(bench['torch_cpu_threads'])
    cv2.setNumThreads(bench['opencv_cpu_threads'])
    weights = ensure_checkpoint(root, det)
    detector = YOLO(str(weights), task='detect')
    if detector.task != 'detect':
        raise ValueError('Expected detection checkpoint')
    options = dict(imgsz=det['size'], conf=det['confidence'], iou=det['nms_iou'],
                   classes=classes_for_names(detector.names, det['class_names']), max_det=det['max_detections'],
                   device=bench['device'], half=bench['half'], rect=False, verbose=False, save=False)
    provenance = dict(config=cfg, detector_sha256=sha256(weights), detector_bytes=weights.stat().st_size,
                      detector_parameters=sum(p.numel() for p in detector.model.parameters()),
                      device=torch.cuda.get_device_name(0), torch=torch.__version__, ultralytics=ultralytics.__version__,
                      opencv=cv2.__version__, inputs_used=['RGB'], automatic_box_prompts=True,
                      training=False, tracking=False, distance_estimation=False, ros=False,
                      ground_truth_available=False, accuracy_metrics=None,
                      timing_scope='decoded BGR -> live detector -> mask generator -> CPU masks; synchronized CUDA',
                      excluded_from_timing=['image decode', 'overlay rendering', 'mask/JSON export', 'ROS/RViz'])
    output.mkdir(parents=True)
    sam = None

    def run_pipeline(image, method):
        torch.cuda.synchronize()
        start = time.perf_counter()
        with torch.inference_mode():
            result = detector.predict(image, **options)[0].cpu()
        torch.cuda.synchronize()
        detection_ms = (time.perf_counter() - start) * 1000
        boxes, scores = result.boxes.xyxy.numpy(), result.boxes.conf.numpy()
        start_mask = time.perf_counter()
        masks, statuses, qualities = [], [], []
        if method == 'grabcut':
            for box in boxes:
                mask, status = grabcut_mask(image, box, cfg['grabcut'])
                masks.append(mask)
                statuses.append(status)
                qualities.append(None)
        elif len(boxes):
            # Never use embeddings from an earlier frame, or measure only a cached decoder.
            if sam.predictor is not None:
                sam.predictor.reset_image()
            with torch.inference_mode():
                segmented = sam.predict(image, bboxes=boxes.tolist(), imgsz=cfg['mobile_sam']['size'],
                                        conf=cfg['mobile_sam']['quality_threshold'], device=bench['device'],
                                        half=bench['half'], verbose=False, save=False)[0].cpu()
            returned = checked_masks(None if segmented.masks is None else segmented.masks.data.numpy(),
                                     len(segmented.boxes), image.shape)
            ids = segmented.boxes.cls.numpy().astype(int)
            if len(set(ids.tolist())) != len(ids) or np.any(ids < 0) or np.any(ids >= len(boxes)):
                raise ValueError('Cannot associate SAM mask indices with detector prompts')
            by_prompt = {int(k): (mask, float(quality)) for k, mask, quality in
                         zip(ids, returned, segmented.boxes.conf.numpy())}
            for index in range(len(boxes)):
                if index in by_prompt:
                    mask, quality = by_prompt[index]
                    masks.append(mask)
                    qualities.append(quality)
                    statuses.append('candidate' if mask.any() else 'empty')
                else:
                    masks.append(np.zeros(image.shape[:2], np.uint8))
                    qualities.append(None)
                    statuses.append('sam_quality_rejected')
        torch.cuda.synchronize()
        mask_ms = (time.perf_counter() - start_mask) * 1000
        pipeline_ms = (time.perf_counter() - start) * 1000
        return boxes, scores, masks, statuses, qualities, detection_ms, mask_ms, pipeline_ms

    summaries = []
    for method in bench['methods']:
        if method == 'mobilesam':
            sam_weights = ensure_checkpoint(root, cfg['mobile_sam'])
            sam = SAM(str(sam_weights))
            provenance.update(mobile_sam_sha256=sha256(sam_weights), mobile_sam_bytes=sam_weights.stat().st_size,
                              mobile_sam_parameters=sum(p.numel() for p in sam.model.parameters()))
        (output / 'provenance.json').write_text(json.dumps(provenance, indent=2))
        first = cv2.imread(frames[bench['warmup_image_index']]['image_path'])
        if first is None:
            raise ValueError('Cannot decode warmup frame')
        for _ in range(bench['warmup_runs']):
            run_pipeline(first, method)
        torch.cuda.reset_peak_memory_stats()
        folder = output / method
        folder.mkdir()
        records = []
        for index, frame in enumerate(frames):
            start = time.perf_counter()
            image = cv2.imread(frame['image_path'])
            decode_ms = (time.perf_counter() - start) * 1000
            if image is None:
                raise ValueError(f'Cannot decode {frame["image_path"]}')
            timing = []
            cpu_start = time.process_time()
            for _ in range(bench['measured_runs']):
                boxes, scores, masks, statuses, qualities, *durations = run_pipeline(image, method)
                timing.append(durations)
            cpu_ms = (time.process_time() - cpu_start) * 1000 / bench['measured_runs']
            instances = []
            for i, (box, score, mask, status, quality) in enumerate(zip(boxes, scores, masks, statuses, qualities)):
                x0, y0, x1, y1 = clipped_box(box, image.shape)
                area = int(mask.sum())
                inside = int(mask[y0:y1, x0:x1].sum())
                instances.append(dict(frame_instance_index=i, class_name='boat', detector_confidence=float(score),
                                      detector_bbox_xyxy=box.tolist(), mask_bbox_xyxy=mask_box(mask),
                                      mask_area_px=area, mask_area_fraction=float(mask.mean()),
                                      mask_outside_prompt_fraction=(area - inside) / area if area else None,
                                      mask_status=status, sam_quality_score=quality,
                                      touches_image_border=x0 == 0 or y0 == 0 or x1 == image.shape[1] or y1 == image.shape[0],
                                      semantic_class_confirmed_by_mask=False, safe_landing_surface=False))
            record = dict(frame, decoded_shape=list(image.shape), instances=instances, detection_count=len(boxes),
                          nonempty_masks=count_nonempty_masks(masks), measured_stage_ms=timing,
                          median_detection_ms=float(np.median([t[0] for t in timing])),
                          median_mask_ms=float(np.median([t[1] for t in timing])),
                          median_pipeline_ms=float(np.median([t[2] for t in timing])),
                          image_decode_ms=decode_ms, cpu_process_ms_per_pipeline=cpu_ms)
            sample = folder / frame['frame_key']
            sample.mkdir()
            start = time.perf_counter()
            annotated = image.copy()
            color, alpha = np.array(cfg['export']['color_bgr']), cfg['export']['overlay_alpha']
            for i, (mask, box) in enumerate(zip(masks, boxes)):
                if cfg['export']['masks'] and not cv2.imwrite(str(sample / f'mask_{i:02d}.png'), mask * 255):
                    raise OSError('Cannot write mask')
                if cfg['export']['annotated']:
                    selected = mask.astype(bool)
                    annotated[selected] = ((1 - alpha) * annotated[selected] + alpha * color).astype(np.uint8)
                    x0, y0, x1, y1 = clipped_box(box, image.shape)
                    cv2.rectangle(annotated, (x0, y0), (x1 - 1, y1 - 1), cfg['export']['color_bgr'], cfg['export']['box_line_width'])
                    cv2.putText(annotated, f'candidate {i} {scores[i]:.2f}', (x0, max(15, y0)),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, cfg['export']['color_bgr'], 1)
            if cfg['export']['annotated'] and not cv2.imwrite(str(sample / 'annotated.jpg'), annotated):
                raise OSError('Cannot write annotation')
            record['visual_export_ms'] = (time.perf_counter() - start) * 1000
            (sample / 'prediction.json').write_text(json.dumps(record, indent=2))
            records.append(record)
            if index % 5 == 0:
                print(f'{method}: {index + 1}/{len(frames)}, {len(boxes)} boxes, '
                      f'{record["nonempty_masks"]} masks, {record["median_pipeline_ms"]:.1f} ms', flush=True)
        timing = [r['median_pipeline_ms'] for r in records]
        report = dict(method=method, frame_count=len(frames), ground_truth_available=False, accuracy_metrics=None,
                      median_pipeline_ms=float(np.median(timing)), p95_pipeline_ms=float(np.quantile(timing, .95)),
                      median_detection_ms=float(np.median([r['median_detection_ms'] for r in records])),
                      median_mask_ms=float(np.median([r['median_mask_ms'] for r in records])),
                      median_decode_ms=float(np.median([r['image_decode_ms'] for r in records])),
                      median_visual_export_ms=float(np.median([r['visual_export_ms'] for r in records])),
                      frames_with_nonempty_masks=sum(r['nonempty_masks'] > 0 for r in records),
                      total_detections=sum(r['detection_count'] for r in records),
                      total_nonempty_masks=sum(r['nonempty_masks'] for r in records),
                      peak_allocated_mib=torch.cuda.max_memory_allocated() / 2**20,
                      peak_reserved_mib=torch.cuda.max_memory_reserved() / 2**20,
                      process_peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024, records=records)
        (folder / 'report.json').write_text(json.dumps(report, indent=2))
        summaries.append({k: v for k, v in report.items() if k != 'records'})
        (output / 'summary.json').write_text(json.dumps(summaries, indent=2))
        print(f'{method}: median {report["median_pipeline_ms"]:.1f} ms, P95 {report["p95_pipeline_ms"]:.1f} ms', flush=True)
        torch.cuda.empty_cache()


if __name__ == '__main__':
    main()
