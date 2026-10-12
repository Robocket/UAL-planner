#!/usr/bin/env python3
"""Extract compressed RGB samples only; preserve bag and header timestamps."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/maritime_reconstruction_trial.yaml')
    args = parser.parse_args()
    config_path = Path(args.config).resolve()
    root = config_path.parent.parent
    cfg = yaml.safe_load(config_path.read_text())['extraction']
    offsets = cfg['offsets_s']
    if not offsets or any(t < 0 for t in offsets) or offsets != sorted(offsets):
        raise ValueError('offsets_s must be nonempty, sorted and nonnegative')
    output = root / cfg['output_dir']
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=cfg['bag_path'], storage_id=cfg['storage_id']),
                rosbag2_py.ConverterOptions('', ''))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics.get(cfg['image_topic']) != 'sensor_msgs/msg/CompressedImage':
        raise ValueError('Configured topic is missing or not CompressedImage')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[cfg['image_topic']]))
    start_ns = reader.get_metadata().starting_time.nanoseconds
    output.mkdir(parents=True)
    records = []
    for index, offset in enumerate(offsets):
        target = start_ns + round(offset * 1e9)
        reader.seek(target)
        if not reader.has_next():
            raise RuntimeError(f'No image at offset {offset}')
        topic, raw, timestamp = reader.read_next()
        msg = deserialize_message(raw, CompressedImage)
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'Cannot decode frame {index}')
        filename = f'frame_{index:03d}.png'
        if not cv2.imwrite(str(output / filename), image):
            raise OSError(f'Cannot write {filename}')
        records.append(dict(file=filename, requested_offset_s=offset, topic=topic,
                            bag_timestamp_ns=timestamp,
                            header_timestamp_ns=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                            frame_id=msg.header.frame_id, shape=list(image.shape)))
    manifest = dict(bag_path=cfg['bag_path'], bag_start_ns=start_ns, frames=records,
                    inputs_used=['compressed_rgb'], lidar_used=False, odometry_used=False)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
