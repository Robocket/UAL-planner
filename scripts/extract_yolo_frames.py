#!/usr/bin/env python3
"""Extract only compressed RGB frames from configurable Scene_Water short clips."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage


def clip_offsets(starts, duration, interval):
    if not starts or starts != sorted(set(starts)) or min(starts) < 0:
        raise ValueError('clip starts must be nonnegative, sorted and unique')
    if duration <= 0 or interval <= 0:
        raise ValueError('duration and interval must be positive')
    return [(clip, start + step * interval, start + duration)
            for clip, start in enumerate(starts)
            for step in range(int(np.ceil(duration / interval)))
            if step * interval < duration]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/yolo_scene_trial.yaml')
    args = parser.parse_args()
    config_path = Path(args.config).resolve()
    root = config_path.parent.parent
    cfg = yaml.safe_load(config_path.read_text())['extraction']
    offsets = clip_offsets(cfg['clip_starts_s'], cfg['clip_duration_s'], cfg['sample_interval_s'])
    output = root / cfg['output_dir']
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=cfg['bag_path'], storage_id=cfg['storage_id']),
                rosbag2_py.ConverterOptions('', ''))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics.get(cfg['image_topic']) != 'sensor_msgs/msg/CompressedImage':
        raise ValueError('Missing or incorrect CompressedImage topic')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[cfg['image_topic']]))
    start_ns = reader.get_metadata().starting_time.nanoseconds
    output.mkdir(parents=True)
    frames, seen = [], set()
    for clip, offset, end in offsets:
        reader.seek(start_ns + round(offset * 1e9))
        if not reader.has_next():
            raise RuntimeError(f'No image at {offset} s')
        topic, raw, timestamp = reader.read_next()
        if timestamp >= start_ns + round(end * 1e9) or timestamp in seen:
            continue
        seen.add(timestamp)
        msg = deserialize_message(raw, CompressedImage)
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'Invalid compressed frame at {timestamp}')
        filename = f'frame_{len(frames):03d}.png'
        if not cv2.imwrite(str(output / filename), image):
            raise OSError(filename)
        frames.append(dict(file=filename, clip_id=clip, requested_offset_s=offset,
                           actual_offset_s=(timestamp - start_ns) / 1e9, topic=topic,
                           bag_timestamp_ns=timestamp, frame_id=msg.header.frame_id,
                           header_timestamp_ns=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                           shape=list(image.shape)))
    manifest = dict(bag_path=cfg['bag_path'], bag_start_ns=start_ns, frames=frames,
                    extraction_config=cfg, lidar_used=False, odometry_used=False)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(f'Extracted {len(frames)} unique RGB frames into {output}')


if __name__ == '__main__':
    main()
