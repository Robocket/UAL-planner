#!/usr/bin/env python3
"""Verify one real bag image -> SAM3 -> mask-selected voxels using installed Jazzy.

This tests interfaces and numerical output, not calibrated geometric accuracy.
The default geometry is the explicitly provisional profile. No model download.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import numpy as np
import rclpy
from ins_seg.msg import SegmentationResult
from rclpy.serialization import deserialize_message
from rclpy.duration import Duration
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import MarkerArray
import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=root/'data/Scene_Water_20260805_013651_ros2_jazzy')
    parser.add_argument('--geometry-config', type=Path, default=root/'config/scene_water_approximate.yaml')
    parser.add_argument('--report-dir', type=Path, required=True)
    parser.add_argument('--start-offset', type=float, default=45.0)
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--start-rviz', action='store_true')
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    report_path = args.report_dir/'single_frame.json'
    log_path = args.report_dir/'single_frame.log'
    if report_path.exists() or log_path.exists():
        raise FileExistsError('Use a fresh report directory; existing evidence will not be overwritten')
    common = yaml.safe_load((root/'config/common.yaml').read_text())
    common['sam3_segmentation']['ros__parameters']['q4_auto_download'] = False
    common_path = args.report_dir/'sam3_no_download.yaml'
    with common_path.open('x') as stream:
        yaml.safe_dump(common, stream)

    database = next(args.bag.glob('*.db3'))
    with sqlite3.connect(f'file:{database.resolve()}?mode=ro', uri=True) as connection:
        ids = dict(connection.execute('SELECT name,id FROM topics'))
        start = connection.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]
        image_time, raw = connection.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp>=? ORDER BY timestamp LIMIT 1',
            (ids['/left_camera/image'], start+int(args.start_offset*1e9)),
        ).fetchone()
        image = deserialize_message(raw, Image)
        candidates = connection.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp',
            (ids['/livox/lidar/points'], image_time-2*10**9, image_time+2*10**9),
        )
        clouds = [deserialize_message(data, PointCloud2) for _, data in candidates]
    def stamp(message):
        return message.header.stamp.sec*10**9+message.header.stamp.nanosec
    cloud = min(clouds, key=lambda item: abs(stamp(item)-stamp(image)))
    delta = abs(stamp(cloud)-stamp(image))/1e9
    if delta > 0.2:
        raise ValueError(f'No cloud matched the test image: delta={delta}')

    rclpy.init(args=[])
    probe = rclpy.create_node('single_frame_verification_probe')
    result_masks, result_clouds, result_markers, annotated = [], [], [], []
    subscriptions = [
        probe.create_subscription(SegmentationResult, '/sam3/segmentation', result_masks.append, 1),
        probe.create_subscription(PointCloud2, '/ship/points', result_clouds.append, 1),
        probe.create_subscription(MarkerArray, '/ship/voxels', result_markers.append, 1),
        probe.create_subscription(Image, '/sam3/annotated_image', annotated.append, 1),
    ]
    image_pub = probe.create_publisher(Image, '/left_camera/image', 1)
    cloud_pub = probe.create_publisher(PointCloud2, '/livox/lidar/points', 1)
    began = time.monotonic()
    sent_at = None
    ready_at = None
    process = None
    with log_path.open('x') as logfile:
        try:
            process = subprocess.Popen([
                'ros2', 'launch', 'pointcloud_voxels', 'scene_water.launch.py',
                'start_image_adapter:=false', 'start_livox_adapter:=false',
                'selection_mode:=mask', 'start_segmentation:=true',
                'start_detection:=false',
                f'geometry_config:={args.geometry_config.resolve()}',
                f'common_config:={common_path.resolve()}',
                f'start_rviz:={str(args.start_rviz).lower()}',
            ], stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
            next_notice = 0
            while time.monotonic()-began < args.timeout:
                rclpy.spin_once(probe, timeout_sec=0.05)
                elapsed = time.monotonic()-began
                if sent_at is None:
                    image_ready = any(info.node_name == 'sam3_segmentation'
                        for info in probe.get_subscriptions_info_by_topic('/left_camera/image'))
                    cloud_ready = any(info.node_name == 'mask_voxels'
                        for info in probe.get_subscriptions_info_by_topic('/livox/lidar/points'))
                    # A generic subscription count may only mean RViz is ready.
                    # Wait for the actual algorithms and discovery to settle.
                    if image_ready and cloud_ready and image_pub.get_subscription_count() and cloud_pub.get_subscription_count():
                        if ready_at is None:
                            ready_at = time.monotonic()
                        if time.monotonic()-ready_at >= 0.5:
                            sent_at = time.monotonic()
                            cloud_pub.publish(cloud)
                            if not cloud_pub.wait_for_all_acked(Duration(seconds=5)):
                                raise RuntimeError('Input cloud was not acknowledged')
                            image_pub.publish(image)
                            if not image_pub.wait_for_all_acked(Duration(seconds=5)):
                                raise RuntimeError('Input image was not acknowledged')
                            print(f'Input published: {image.width}x{image.height}, cloud={cloud.width}, delta={delta}s', flush=True)
                if result_masks and result_clouds and result_markers and annotated:
                    break
                if process.poll() is not None:
                    raise RuntimeError(f'Launch exited with {process.returncode}; inspect {log_path}')
                if elapsed >= next_notice:
                    print(f'Waiting {elapsed:.0f}s: masks={len(result_masks)}, clouds={len(result_clouds)}', flush=True)
                    next_notice = elapsed+10
            if not (result_masks and result_clouds and result_markers and annotated):
                raise TimeoutError(f'No complete output within {args.timeout}s; inspect {log_path}')
            mask = result_masks[-1]
            id_map = np.ndarray((mask.instance_id_map.height, mask.instance_id_map.width), dtype='<i4',
                                buffer=mask.instance_id_map.data, strides=(mask.instance_id_map.step, 4))
            assert mask.instance_id_map.encoding == '32SC1'
            assert id_map.shape == (image.height, image.width)
            assert stamp(mask) == stamp(image)
            selected = result_clouds[-1]
            assert stamp(selected) == stamp(cloud)
            assert selected.header.frame_id == cloud.header.frame_id
            marker_sizes = {m.ns: len(m.points) for m in result_markers[-1].markers if m.ns}
            geometry = yaml.safe_load(args.geometry_config.read_text())['mask_voxels']['ros__parameters']
            report = {
                'status': 'interfaces_passed', 'bag': str(args.bag.resolve()),
                'geometry_config': str(args.geometry_config.resolve()),
                'calibration_confirmed': geometry.get('calibration_confirmed', False),
                'approximate_geometry': geometry.get('allow_approximate_calibration', False),
                'image_header_ns': stamp(image), 'cloud_header_ns': stamp(cloud),
                'sensor_delta_sec': delta, 'image_size': [image.width, image.height],
                'input_points': cloud.width*cloud.height, 'instances': len(mask.instances),
                'foreground_pixels': int(np.count_nonzero(id_map)),
                'selected_points': selected.width*selected.height, 'markers': marker_sizes,
                'annotated_size': [annotated[-1].width, annotated[-1].height],
                'observed_result_latency_sec': time.monotonic()-sent_at,
                'rviz_image_subscription': any(info.node_name == 'ship_voxels_rviz'
                    for info in probe.get_subscriptions_info_by_topic('/sam3/annotated_image')),
                'accuracy_validated': False,
            }
            with report_path.open('x') as stream:
                json.dump(report, stream, indent=2)
            print(json.dumps(report, indent=2), flush=True)
        except (Exception, KeyboardInterrupt) as error:
            with (args.report_dir/'single_frame_failure.json').open('x') as stream:
                json.dump({'error': str(error), 'type': type(error).__name__,
                           'masks_received': len(result_masks), 'clouds_received': len(result_clouds),
                           'instances': [len(value.instances) for value in result_masks]}, stream, indent=2)
            raise
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=50)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)
            probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
