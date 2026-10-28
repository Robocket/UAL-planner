#!/usr/bin/env python3
"""Verify one real bag frame through the YOLO detection -> voxel path.

This checks interfaces, timestamp matching and non-empty numerical output.  It
is not an accuracy, calibration or real-time acceptance test.  The detector
runs in the configured Python interpreter (Ultralytics is not a ROS dependency).
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from ins_seg.msg import DetectionResult
from visualization_msgs.msg import MarkerArray


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path,
                        default=root/'data/Scene_Water_20260805_013651_ros2_jazzy')
    parser.add_argument('--geometry-config', type=Path,
                        default=root/'config/scene_water_approximate.yaml')
    parser.add_argument('--detector-config', type=Path,
                        default=root/'config/detector.yaml')
    parser.add_argument('--detector-python', default=str(root/'.venv-yolo/bin/python'))
    parser.add_argument('--report-dir', type=Path, required=True)
    parser.add_argument('--start-offset', type=float, default=300.0)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--start-rviz', action='store_true')
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    report_path = args.report_dir/'detection_frame.json'
    log_path = args.report_dir/'detection_frame.log'
    if report_path.exists() or log_path.exists():
        raise FileExistsError('Use a fresh report directory; existing evidence will not be overwritten')

    database = next(args.bag.glob('*.db3'))
    with sqlite3.connect(f'file:{database.resolve()}?mode=ro', uri=True) as connection:
        topics = dict(connection.execute('SELECT name,id FROM topics'))
        start = connection.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]
        image_time, raw = connection.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp>=? '
            'ORDER BY timestamp LIMIT 1',
            (topics['/left_camera/image'], start+int(args.start_offset*1e9)),
        ).fetchone()
        image = deserialize_message(raw, Image)
        candidates = connection.execute(
            'SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? '
            'ORDER BY timestamp',
            (topics['/livox/lidar/points'], image_time-2*10**9, image_time+2*10**9),
        ).fetchall()
    clouds = [deserialize_message(data, PointCloud2) for _, data in candidates]

    def stamp(message):
        return message.header.stamp.sec*10**9+message.header.stamp.nanosec

    if not clouds:
        raise ValueError('No point cloud near the selected image')
    cloud = min(clouds, key=lambda item: abs(stamp(item)-stamp(image)))
    delta = abs(stamp(cloud)-stamp(image))/1e9
    if delta > 0.2:
        raise ValueError(f'No cloud within sync_slop of image: delta={delta}')

    rclpy.init(args=[])
    probe = rclpy.create_node('detection_verification_probe')
    detections, selected, markers, annotated = [], [], [], []
    paths, path_markers, targets, statuses = [], [], [], []
    bboxes, bbox_targets = [], []
    subscriptions = [
        probe.create_subscription(DetectionResult, '/ship/detections', detections.append, 10),
        probe.create_subscription(PointCloud2, '/ship/points', selected.append, 10),
        probe.create_subscription(MarkerArray, '/ship/voxels', markers.append, 10),
        probe.create_subscription(Image, '/ship/detections_image', annotated.append, 10),
        probe.create_subscription(RosPath, '/ship/path', paths.append, 10),
        probe.create_subscription(MarkerArray, '/ship/path_markers', path_markers.append, 10),
        probe.create_subscription(PointStamped, '/ship/target', targets.append, 10),
        probe.create_subscription(String, '/ship/path_status', statuses.append, 10),
        probe.create_subscription(MarkerArray, '/ship/bboxes', bboxes.append, 10),
        probe.create_subscription(PointStamped, '/ship/bbox_target', bbox_targets.append, 10),
    ]
    image_pub = probe.create_publisher(Image, '/left_camera/image', 1)
    cloud_pub = probe.create_publisher(PointCloud2, '/livox/lidar/points', 1)
    process = None
    with log_path.open('x') as logfile:
        try:
            process = subprocess.Popen([
                'ros2', 'launch', 'pointcloud_voxels', 'scene_water.launch.py',
                'start_image_adapter:=false', 'start_livox_adapter:=false',
                'start_segmentation:=false', 'start_detection:=true',
                'selection_mode:=detection',
                f'start_rviz:={str(args.start_rviz).lower()}',
                f'geometry_config:={args.geometry_config.resolve()}',
                f'detector_config:={args.detector_config.resolve()}',
                f'detector_python:={args.detector_python}',
            ], stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
            began = time.monotonic()
            sent_at = None
            ready_at = None
            cloud_repeats = 0
            last_cloud_publish = 0.0
            while time.monotonic()-began < args.timeout:
                rclpy.spin_once(probe, timeout_sec=0.05)
                if sent_at is None:
                    detection_ready = any(
                        info.node_name == 'ship_detector'
                        for info in probe.get_subscriptions_info_by_topic('/left_camera/image')
                    ) and bool(probe.get_publishers_info_by_topic('/ship/detections'))
                    annotated_ready = bool(probe.get_publishers_info_by_topic('/ship/detections_image'))
                    cloud_ready = any(
                        info.node_name == 'mask_voxels'
                        for info in probe.get_subscriptions_info_by_topic('/livox/lidar/points')
                    )
                    planner_target_ready = any(
                        info.node_name == 'ship_path_planner'
                        for info in probe.get_subscriptions_info_by_topic('/ship/points')
                    )
                    planner_obstacle_ready = any(
                        info.node_name == 'ship_path_planner'
                        for info in probe.get_subscriptions_info_by_topic('/livox/lidar/points')
                    )
                    planner_bbox_ready = any(
                        info.node_name == 'ship_path_planner'
                        for info in probe.get_subscriptions_info_by_topic('/ship/bbox_target')
                    )
                    rviz_path_ready = (
                        not args.start_rviz or any(
                            info.node_name == 'ship_voxels_rviz'
                            for info in probe.get_subscriptions_info_by_topic('/ship/path')
                        )
                    )
                    rviz_bbox_ready = (
                        not args.start_rviz or any(
                            info.node_name == 'ship_voxels_rviz'
                            for info in probe.get_subscriptions_info_by_topic('/ship/bboxes')
                        )
                    )
                    obstacle_matched = cloud_pub.get_subscription_count() >= 2
                    if (detection_ready and annotated_ready and cloud_ready
                            and planner_target_ready and planner_obstacle_ready
                            and planner_bbox_ready and rviz_path_ready
                            and rviz_bbox_ready and obstacle_matched):
                        if ready_at is None:
                            ready_at = time.monotonic()
                        elif time.monotonic()-ready_at >= 0.5:
                            sent_at = time.monotonic()
                            cloud_pub.publish(cloud)
                            if not cloud_pub.wait_for_all_acked(Duration(seconds=5)):
                                raise RuntimeError('Input cloud was not acknowledged')
                            image_pub.publish(image)
                            if not image_pub.wait_for_all_acked(Duration(seconds=5)):
                                raise RuntimeError('Input image was not acknowledged')
                if (detections and selected and markers and annotated and paths
                        and path_markers and targets and statuses and bboxes
                        and bbox_targets
                        and len(paths[-1].poses) >= 2
                        and statuses[-1].data.startswith('ok')):
                    break
                # The planner may have discovered its obstacle subscription
                # just after the single cloud was sent.  A real bag streams
                # clouds continuously; repeat the same-acquisition cloud a few
                # times for this one-shot verification.
                if (sent_at is not None and cloud_repeats < 5
                        and time.monotonic()-last_cloud_publish >= 0.5):
                    cloud_pub.publish(cloud)
                    cloud_repeats += 1
                    last_cloud_publish = time.monotonic()
                if process.poll() is not None:
                    raise RuntimeError(f'Launch exited with {process.returncode}; inspect {log_path}')
            if not (detections and selected and markers and annotated and paths
                    and path_markers and targets and statuses and bboxes and bbox_targets):
                raise TimeoutError(
                    f'Incomplete output within {args.timeout}s: detections={len(detections)}, '
                    f'clouds={len(selected)}, markers={len(markers)}, images={len(annotated)}, '
                    f'paths={len(paths)}, path_markers={len(path_markers)}, '
                    f'targets={len(targets)}, statuses={len(statuses)}, '
                    f'bboxes={len(bboxes)}, bbox_targets={len(bbox_targets)}'
                )
            detection = detections[-1]
            report = {
                'status': 'interfaces_passed',
                'bag': str(args.bag.resolve()),
                'offset_sec': args.start_offset,
                'image_header_ns': stamp(image),
                'cloud_header_ns': stamp(cloud),
                'sensor_delta_sec': delta,
                'image_size': [image.width, image.height],
                'input_points': cloud.width*cloud.height,
                'detections': [{
                    'track_id': item.track_id,
                    'class_id': item.class_id,
                    'class_name': item.class_name,
                    'confidence': item.confidence,
                    'oriented': item.oriented,
                    'polygon': [[point.x, point.y] for point in item.polygon.points],
                } for item in detection.detections],
                'selected_points': selected[-1].width*selected[-1].height,
                'selected_stamp_matches_cloud': stamp(selected[-1]) == stamp(cloud),
                'selected_frame': selected[-1].header.frame_id,
                'markers': {marker.ns: len(marker.points)
                            for marker in markers[-1].markers if marker.ns},
                'annotated_size': [annotated[-1].width, annotated[-1].height],
                'path_waypoints': len(paths[-1].poses),
                'path_frame': paths[-1].header.frame_id,
                'path_start': [
                    paths[-1].poses[0].pose.position.x,
                    paths[-1].poses[0].pose.position.y,
                    paths[-1].poses[0].pose.position.z,
                ],
                'path_end': [
                    paths[-1].poses[-1].pose.position.x,
                    paths[-1].poses[-1].pose.position.y,
                    paths[-1].poses[-1].pose.position.z,
                ],
                'path_length_m': sum(
                    math.dist(
                        (first.pose.position.x, first.pose.position.y, first.pose.position.z),
                        (second.pose.position.x, second.pose.position.y, second.pose.position.z),
                    )
                    for first, second in zip(paths[-1].poses, paths[-1].poses[1:])
                ),
                'path_status': statuses[-1].data,
                'path_marker_namespaces': sorted(
                    marker.ns for marker in path_markers[-1].markers if marker.ns),
                'bbox_marker_namespaces': sorted(
                    marker.ns for marker in bboxes[-1].markers if marker.ns),
                'bbox_target_point': [
                    bbox_targets[-1].point.x,
                    bbox_targets[-1].point.y,
                    bbox_targets[-1].point.z,
                ],
                'bbox_target_stamp_matches_cloud': stamp(bbox_targets[-1]) == stamp(cloud),
                'bbox_labels': [marker.text for marker in bboxes[-1].markers
                                if marker.ns == 'ship_point_bbox_labels'],
                'bbox_edge_counts': [len(marker.points) // 2
                                     for marker in bboxes[-1].markers
                                     if marker.ns == 'ship_point_bbox'],
                'path_end_matches_bbox_target': math.dist(
                    (paths[-1].poses[-1].pose.position.x,
                     paths[-1].poses[-1].pose.position.y,
                     paths[-1].poses[-1].pose.position.z),
                    (bbox_targets[-1].point.x, bbox_targets[-1].point.y,
                     bbox_targets[-1].point.z),
                ) < 1e-6,
                'rviz_path_subscription': any(
                    info.node_name == 'ship_voxels_rviz'
                    for info in probe.get_subscriptions_info_by_topic('/ship/path')
                ),
                'rviz_bbox_subscription': any(
                    info.node_name == 'ship_voxels_rviz'
                    for info in probe.get_subscriptions_info_by_topic('/ship/bboxes')
                ),
                'target_point': [
                    targets[-1].point.x, targets[-1].point.y, targets[-1].point.z],
                'target_stamp_matches_cloud': stamp(targets[-1]) == stamp(cloud),
                'path_end_matches_target': math.dist(
                    (paths[-1].poses[-1].pose.position.x,
                     paths[-1].poses[-1].pose.position.y,
                     paths[-1].poses[-1].pose.position.z),
                    (targets[-1].point.x, targets[-1].point.y, targets[-1].point.z),
                ) < 1e-6,
                'observed_result_latency_sec': time.monotonic()-sent_at,
                'accuracy_validated': False,
            }
            with report_path.open('x') as stream:
                json.dump(report, stream, indent=2)
            print(json.dumps(report, indent=2), flush=True)
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)
            probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
