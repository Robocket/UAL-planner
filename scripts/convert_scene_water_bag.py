#!/usr/bin/env python3
"""Convert an indexed ROS 1 bag to Jazzy SQLite3 with directly viewable images.

Run with the isolated rosbags environment. Originals are opened read-only;
an existing output directory is never overwritten. No timestamps/frames change.
"""

import argparse
from collections import Counter
import hashlib
import importlib.metadata
import json
from pathlib import Path
import time

import cv2
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore


def convert(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    if destination.exists():
        raise FileExistsError(f'Refusing to overwrite existing output: {destination}')
    if not destination.parent.is_dir():
        raise FileNotFoundError(destination.parent)
    started = time.monotonic()
    source_stat = source.stat()
    store = get_typestore(Stores.ROS2_JAZZY)
    image_type = store.types['sensor_msgs/msg/Image']
    counts, expected = Counter(), Counter()
    image_shapes = Counter()
    image_count = 0
    image_pixel_hash = hashlib.sha256()
    first_image = last_image = None
    first_bag_ns = last_bag_ns = None
    input_topics = []
    cv2.setNumThreads(2)

    with Reader(source) as reader:
        image_connections = [c for c in reader.connections if c.topic == '/left_camera/image/compressed']
        if not image_connections or any(c.msgtype != 'sensor_msgs/msg/CompressedImage' for c in image_connections):
            raise ValueError('Expected /left_camera/image/compressed with CompressedImage messages')
        if any(c.topic == '/left_camera/image' for c in reader.connections):
            raise ValueError('Input already contains raw image topic; refusing ambiguous duplication')
        mapped_types = {}
        for connection in reader.connections:
            if connection.msgtype not in store.types:
                raise ValueError(f'Input type is not a standard Jazzy type: {connection.msgtype}')
            topic = ('/livox/lidar/points'
                     if connection.topic == '/livox/lidar' and connection.msgtype == 'sensor_msgs/msg/PointCloud2'
                     else connection.topic)
            if topic in mapped_types and mapped_types[topic] != connection.msgtype:
                raise ValueError(f'Multiple message types would map to {topic}')
            mapped_types[topic] = connection.msgtype
            expected[topic] += connection.msgcount
            input_topics.append({
                'input_topic': connection.topic, 'output_topic': topic,
                'type': connection.msgtype, 'expected_messages': connection.msgcount,
            })
        expected['/left_camera/image'] = sum(c.msgcount for c in image_connections)

        with Writer(destination, version=9) as writer:
            output_connections = {
                topic: writer.add_connection(topic, msgtype, typestore=store)
                for topic, msgtype in mapped_types.items()
            }
            output_image = writer.add_connection('/left_camera/image', 'sensor_msgs/msg/Image', typestore=store)
            by_input_id = {
                c.id: output_connections[
                    '/livox/lidar/points' if c.topic == '/livox/lidar' and c.msgtype == 'sensor_msgs/msg/PointCloud2'
                    else c.topic
                ] for c in reader.connections
            }
            for connection, timestamp, raw in reader.messages():
                if first_bag_ns is None:
                    first_bag_ns = timestamp
                last_bag_ns = timestamp
                # ROS2 store removes ROS1 Header.seq and applies CDR alignment.
                cdr = store.ros1_to_cdr(raw, connection.msgtype)
                output = by_input_id[connection.id]
                writer.write(output, timestamp, cdr)
                counts[output.topic] += 1
                if connection.topic != '/left_camera/image/compressed':
                    continue
                message = store.deserialize_cdr(cdr, connection.msgtype)
                image = cv2.imdecode(message.data, cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(f'Image #{image_count} at {timestamp} cannot be decoded')
                image = np.ascontiguousarray(image)
                height, width, channels = image.shape
                if channels != 3:
                    raise ValueError('Decoded image must be BGR with three channels')
                decoded = image_type(
                    header=message.header, height=height, width=width,
                    encoding='bgr8', is_bigendian=0, step=width * 3,
                    data=image.reshape(-1),
                )
                writer.write(output_image, timestamp, store.serialize_cdr(decoded, decoded.__msgtype__))
                counts[output_image.topic] += 1
                image_count += 1
                image_shapes[f'{width}x{height}'] += 1
                image_pixel_hash.update(image)
                observation = {
                    'bag_timestamp_ns': timestamp,
                    'header_timestamp_ns': message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                    'frame_id': message.header.frame_id,
                }
                if first_image is None:
                    first_image = observation
                last_image = observation
                if image_count % 200 == 0:
                    print(f'Converted {image_count}/{expected[output_image.topic]} images; '
                          f'{sum(counts.values())} output messages; '
                          f'{time.monotonic() - started:.1f}s', flush=True)

    if counts != expected:
        raise ValueError(f'Incomplete conversion: expected {dict(expected)}, got {dict(counts)}')
    unchanged = source.stat()
    if (source_stat.st_size, source_stat.st_mtime_ns) != (unchanged.st_size, unchanged.st_mtime_ns):
        raise RuntimeError('Source file changed while converting; output must not be trusted')
    report = {
        'status': 'complete', 'source': str(source), 'source_size_bytes': source_stat.st_size,
        'source_mtime_ns': source_stat.st_mtime_ns, 'output': str(destination),
        'rosbags_version': importlib.metadata.version('rosbags'), 'storage': 'sqlite3',
        'metadata_version': 9, 'input_topics': input_topics,
        'output_counts': dict(counts), 'decoded_images': image_count,
        'decoded_image_sizes': dict(image_shapes),
        'decoded_image_pixels_sha256': image_pixel_hash.hexdigest(),
        'first_image': first_image, 'last_image': last_image,
        'first_bag_timestamp_ns': first_bag_ns, 'last_bag_timestamp_ns': last_bag_ns,
        'duration_sec': (last_bag_ns-first_bag_ns)/1e9 if first_bag_ns is not None else 0.0,
        'elapsed_sec': time.monotonic() - started,
        'headers_preserved': True,
    }
    with (destination/'conversion_report.json').open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--src', type=Path, required=True)
    parser.add_argument('--dst', type=Path, required=True)
    args = parser.parse_args()
    convert(args.src, args.dst)


if __name__ == '__main__':
    main()
