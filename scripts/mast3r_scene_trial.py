#!/usr/bin/env python3
"""Benchmark upstream MASt3R pair inference. No SLAM, segmentation or world transform."""
import argparse
import gc
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml


def checkpoint_sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def point_mask(points, confidence, threshold, positive_z):
    mask = np.isfinite(points).all(axis=-1) & np.isfinite(confidence) & (confidence >= threshold)
    if positive_z:
        mask &= points[..., 2] > 0
    return mask


def export_result(result, folder, cfg):
    from PIL import Image
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    all_points, all_colors, statistics = [], [], []
    arrays = {}
    figure, axes = plt.subplots(2, 3, figsize=(12, 7))
    for index, key in enumerate(('pts3d', 'pts3d_in_other_view'), 1):
        points = result[f'pred{index}'][key][0].numpy()
        confidence = result[f'pred{index}']['conf'][0].numpy()
        colors = (result[f'view{index}']['img'][0].permute(1, 2, 0).numpy() * .5 + .5).clip(0, 1)
        colors = np.round(colors * 255).astype(np.uint8)
        mask = point_mask(points, confidence, cfg['confidence_threshold'], cfg['positive_z_only'])
        arrays.update({f'points{index}': points, f'confidence{index}': confidence, f'valid{index}': mask})
        Image.fromarray(colors).save(folder / f'processed_view{index}.png')
        all_points.append(points[mask])
        all_colors.append(colors[mask])
        statistics.append(dict(view=index, total_pixels=mask.size, retained_points=int(mask.sum()),
                               retained_fraction=float(mask.mean()),
                               confidence_quantiles=np.quantile(confidence, [.1, .5, .9]).tolist()))
        axes[index - 1, 0].imshow(colors)
        axes[index - 1, 0].set_title(f'Processed view {index}')
        depth = np.where(mask, points[..., 2], np.nan)
        finite = depth[np.isfinite(depth)]
        limits = np.quantile(finite, [.02, .98]) if finite.size else [0, 1]
        plot = axes[index - 1, 1].imshow(depth, vmin=limits[0], vmax=limits[1], cmap='viridis')
        axes[index - 1, 1].set_title('Predicted Z in camera 1 (unverified scale)')
        figure.colorbar(plot, ax=axes[index - 1, 1])
        axes[index - 1, 2].imshow(confidence, vmin=1, vmax=10, cmap='magma')
        axes[index - 1, 2].set_title(f'Confidence, kept {mask.mean():.1%}')
        for axis in axes[index - 1]:
            axis.axis('off')
    figure.tight_layout()
    figure.savefig(folder / 'preview.png', dpi=130)
    plt.close(figure)
    np.savez_compressed(folder / 'pointmaps.npz', **arrays)
    points, colors = np.concatenate(all_points), np.concatenate(all_colors)
    header = ('ply\nformat ascii 1.0\ncomment frame camera_1_local; scale unverified\n'
              f'element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\n'
              'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
    with (folder / 'pair_local.ply').open('w') as stream:
        stream.write(header)
        np.savetxt(stream, np.column_stack((points, colors)), fmt=['%.6g'] * 3 + ['%d'] * 3)
    figure = plt.figure(figsize=(8, 6))
    axis = figure.add_subplot(projection='3d')
    if len(points):
        selected = np.linspace(0, len(points) - 1, min(len(points), cfg['preview_max_points']), dtype=int)
        p = points[selected]
        axis.scatter(p[:, 0], p[:, 2], -p[:, 1], c=colors[selected] / 255, s=1)
    axis.set(xlabel='camera X', ylabel='camera Z', zlabel='-camera Y',
             title='Pair-local point cloud; scale NOT verified')
    figure.savefig(folder / 'cloud_preview.png', dpi=130)
    plt.close(figure)
    return statistics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/maritime_reconstruction_trial.yaml')
    parser.add_argument('--dataset', choices=['mtred', 'scene_water', 'all'], default='all')
    args = parser.parse_args()
    config_path = Path(args.config).resolve()
    root = config_path.parent.parent
    config = yaml.safe_load(config_path.read_text())
    cfg = config['benchmark']
    if cfg['measured_runs'] < 1 or cfg['warmup_runs'] < 0 or cfg['preview_max_points'] < 1:
        raise ValueError('Invalid benchmark run counts or preview size')
    if cfg['precision'] not in ('float16_amp', 'float32'):
        raise ValueError('precision must be float16_amp or float32')
    if any(size not in (224, 384, 512) for size in cfg['resolutions']):
        raise ValueError('Supported trial sizes: 224 (diagnostic crop), 384, 512')
    source = root / config['model']['source_dir']
    revision = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != config['model']['source_revision']:
        raise ValueError(f'Unexpected upstream revision: {revision}')
    sys.path.insert(0, str(source))
    import torch
    from mast3r.model import AsymmetricMASt3R
    from dust3r.inference import loss_of_one_batch
    from dust3r.utils.device import collate_with_cat, to_cpu
    from dust3r.utils.image import load_images
    device = config['model']['device']
    parameter_precision = config['model']['parameter_precision']
    if parameter_precision not in ('float32', 'float16_core_float32_heads'):
        raise ValueError('Unknown parameter_precision')
    if device != 'cuda' or not torch.cuda.is_available():
        raise RuntimeError('This GPU benchmark requires CUDA; no silent CPU fallback')
    output = root / cfg['output_dir']
    names = list(config['datasets']) if args.dataset == 'all' else [args.dataset]
    cases = [(name, size) for name in names for size in cfg['resolutions']]
    for name, size in cases:
        if (output / f'{name}_{size}').exists():
            raise FileExistsError(f'Refusing to overwrite {name}_{size}')
        paths = config['datasets'][name]['images']
        if len(paths) != 2 or not all((root / path).is_file() for path in paths):
            raise ValueError(f'{name} needs two existing images')
    checkpoint = root / config['model']['checkpoint']
    digest = checkpoint_sha(checkpoint)
    if digest != config['model']['checkpoint_sha256']:
        raise ValueError('Checkpoint SHA256 differs from recorded official download')
    provenance = dict(checkpoint_sha256=digest, source_revision=revision,
                      checkpoint_url=config['model']['checkpoint_url'], torch_version=torch.__version__,
                      device=torch.cuda.get_device_name(), total_vram_mib=torch.cuda.get_device_properties(0).total_memory / 2**20,
                      precision=cfg['precision'], parameter_dtype=parameter_precision, batch_size=1,
                      coordinate_frame='first_camera_local', metric_scale_verified=False,
                      world_alignment_verified=False, custom_cuda_rope=False,
                      timing_scope='CPU-preprocessed pair -> GPU inference -> CPU pointmaps; excludes exports',
                      configuration=config)
    print('Loading official checkpoint...', flush=True)
    model = AsymmetricMASt3R.from_pretrained(str(checkpoint)).eval()
    if parameter_precision == 'float16_core_float32_heads':
        # Upstream explicitly casts decoder tokens to FP32 and disables AMP in these heads.
        # Keep them FP32, otherwise full-model half() produces a dtype mismatch.
        model.half()
        model.downstream_head1.float()
        model.downstream_head2.float()
    model = model.to(device)
    provenance['parameter_memory_mib'] = sum(p.numel() * p.element_size() for p in model.parameters()) / 2**20
    output.mkdir(parents=True, exist_ok=True)
    for name, size in cases:
        folder = output / f'{name}_{size}'
        folder.mkdir()
        report = dict(provenance, dataset=name, requested_size=size, status='running',
                      diagnostic_center_crop=(size == 224))
        result = batch = None
        try:
            before = time.perf_counter()
            images = load_images([str(root / p) for p in config['datasets'][name]['images']], size=size, verbose=False)
            report['preprocessing_s'] = time.perf_counter() - before
            report['processed_shapes'] = [list(view['img'].shape) for view in images]
            torch.cuda.reset_peak_memory_stats()
            durations = []
            for index in range(cfg['warmup_runs'] + cfg['measured_runs']):
                result = None
                batch = collate_with_cat([tuple(images)])
                torch.cuda.synchronize()
                start = time.perf_counter()
                with torch.inference_mode(), torch.autocast(
                        device_type='cuda', enabled=cfg['precision'] == 'float16_amp',
                        cache_enabled=cfg['autocast_weight_cache']):
                    result = to_cpu(loss_of_one_batch(batch, model, None, device,
                                                     use_amp=cfg['precision'] == 'float16_amp'))
                torch.cuda.synchronize()
                elapsed = time.perf_counter() - start
                if index >= cfg['warmup_runs']:
                    durations.append(elapsed)
                batch = None
                print(f'{name}/{size} run {index + 1}: {elapsed:.3f} s', flush=True)
            report.update(status='ok', measured_pair_latency_s=durations,
                          median_pair_latency_s=float(np.median(durations)),
                          pairs_per_second=1 / float(np.median(durations)),
                          peak_allocated_mib=torch.cuda.max_memory_allocated() / 2**20,
                          peak_reserved_mib=torch.cuda.max_memory_reserved() / 2**20)
            report['point_statistics'] = export_result(result, folder, cfg)
        except torch.cuda.OutOfMemoryError as error:
            report.update(status='cuda_out_of_memory', error=str(error),
                          peak_allocated_mib=torch.cuda.max_memory_allocated() / 2**20,
                          peak_reserved_mib=torch.cuda.max_memory_reserved() / 2**20)
            print(f'{name}/{size}: CUDA OOM', flush=True)
        finally:
            (folder / 'report.json').write_text(json.dumps(report, indent=2))
            result = batch = None
            gc.collect()
            torch.cuda.empty_cache()


if __name__ == '__main__':
    main()
