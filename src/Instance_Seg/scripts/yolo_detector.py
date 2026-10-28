#!/usr/bin/env python3
"""Publish ship detections from a YOLO OBB/detection checkpoint.

The node is model-backend agnostic between Ultralytics OBB and horizontal
detection heads.  It never creates or fills an instance mask: downstream
selection projects LiDAR points and tests the reported image polygon.
"""

import hashlib
import os
from pathlib import Path
import sys
import time
import traceback

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from ins_seg.msg import Detection, DetectionResult

sys.path.insert(0, str(Path(__file__).resolve().parent))
from detector_core import decode_result, target_class_ids  # noqa: E402


DEFAULT_WEIGHTS = Path('~/.cache/ual_planner/models/yolo11n-obb.pt').expanduser()
DEFAULT_WEIGHTS_SHA256 = (
    'b62898ebf38940ca4df323863e45ee9d84a1a46d5d11ebdde529fb33aa9f3a32'
)


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


class YoloDetector(Node):
    def __init__(self):
        super().__init__('ship_detector')
        defaults = {
            'image_topic': '/left_camera/image',
            'image_reliability': 'best_effort',
            'detection_topic': '/ship/detections',
            'annotated_topic': '/ship/detections_image',
            'weights': '',
            'weights_sha256': DEFAULT_WEIGHTS_SHA256,
            'task': 'obb',
            'target_classes': ['ship'],
            'confidence_threshold': 0.25,
            'iou_threshold': 0.7,
            'max_detections': 300,
            'image_size': 640,
            'device': 'cpu',
            'half': False,
            'agnostic_nms': False,
            'cpu_threads': 4,
            'publish_annotated': True,
            'qos_depth': 1,
            'log_period_sec': 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.cfg = p

        if p['image_reliability'] not in ('best_effort', 'reliable'):
            raise ValueError('image_reliability must be best_effort or reliable')
        if p['task'] not in ('obb', 'detect'):
            raise ValueError("task must be 'obb' or 'detect'")
        if not 0.0 <= float(p['confidence_threshold']) <= 1.0:
            raise ValueError('confidence_threshold must be in [0, 1]')
        if not 0.0 < float(p['iou_threshold']) <= 1.0:
            raise ValueError('iou_threshold must be in (0, 1]')
        if int(p['max_detections']) <= 0 or int(p['image_size']) < 32:
            raise ValueError('max_detections must be positive and image_size >= 32')
        if int(p['cpu_threads']) <= 0 or int(p['qos_depth']) <= 0:
            raise ValueError('cpu_threads and qos_depth must be positive')
        if not float(p['log_period_sec']) > 0.0:
            raise ValueError('log_period_sec must be positive')
        if bool(p['half']) and str(p['device']).lower() == 'cpu':
            raise ValueError('half precision requires a CUDA device')

        self.weights = (
            Path(str(p['weights'])).expanduser() if str(p['weights']).strip()
            else DEFAULT_WEIGHTS
        ).resolve()
        if not self.weights.is_file():
            raise FileNotFoundError(
                f'YOLO weights not found: {self.weights}. Run '
                'scripts/fetch_yolo_detector_weights.py or set the weights parameter.'
            )
        expected_hash = str(p['weights_sha256']).strip().lower()
        if expected_hash and _sha256(self.weights) != expected_hash:
            raise ValueError(f'YOLO weight SHA-256 mismatch: {self.weights}')

        cache_dir = Path('~/.cache/ual_planner/yolo').expanduser()
        cache_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault('YOLO_CONFIG_DIR', str(cache_dir))
        os.environ.setdefault('YOLO_AUTOINSTALL', 'false')
        os.environ.setdefault('YOLO_OFFLINE', 'true')

        import torch
        from ultralytics import YOLO, settings

        settings.update({'sync': False})
        torch.set_num_threads(int(p['cpu_threads']))
        cv2.setNumThreads(int(p['cpu_threads']))
        device = str(p['device']).strip()
        if device.lower() == 'auto':
            device = None
        self.device = device
        self.use_cuda = device not in (None, 'cpu') and not device.startswith('cpu')
        if self.use_cuda and not torch.cuda.is_available():
            raise RuntimeError('CUDA device requested but torch.cuda is unavailable')

        start = time.perf_counter()
        self.model = YOLO(str(self.weights))
        if self.model.task != p['task']:
            raise ValueError(
                f"checkpoint task is {self.model.task}, but task is configured as {p['task']}"
            )
        # Validate configured names before the first image arrives.
        target_class_ids(self.model.names, p['target_classes'])
        self.get_logger().info(
            f'loaded {self.model.task} model {self.weights} in '
            f'{time.perf_counter() - start:.2f}s; classes={self.model.names}'
        )

        self.bridge = CvBridge()
        self.frames = 0
        self.last_log = time.monotonic()
        reliability = (
            ReliabilityPolicy.RELIABLE
            if p['image_reliability'] == 'reliable'
            else ReliabilityPolicy.BEST_EFFORT
        )
        self.detection_pub = self.create_publisher(
            DetectionResult, p['detection_topic'],
            QoSProfile(depth=int(p['qos_depth']),
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        self.image_pub = (
            self.create_publisher(Image, p['annotated_topic'], 1)
            if p['publish_annotated'] else None
        )
        self.image_sub = self.create_subscription(
            Image, p['image_topic'], self._image_callback,
            QoSProfile(depth=1, reliability=reliability),
        )
        self.get_logger().info(
            f"subscribed {p['image_topic']} -> {p['detection_topic']}, "
            f"device={p['device']}, imgsz={p['image_size']}"
        )

    def _warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def _annotate(self, image, detections):
        canvas = image.copy()
        for detection in detections:
            polygon = np.rint(detection.polygon).astype(np.int32)
            cv2.polylines(canvas, [polygon], True, (0, 220, 255), 2)
            anchor = polygon[0]
            x = max(0, int(anchor[0]))
            y = max(18, min(canvas.shape[0] - 5, int(anchor[1])))
            cv2.putText(
                canvas,
                f'{detection.class_name} {detection.confidence:.2f}',
                (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2,
            )
        return canvas

    def _image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            if image.ndim != 3 or image.shape[2] != 3:
                raise ValueError('image must decode as three-channel BGR')
            if image.shape[0] <= 0 or image.shape[1] <= 0:
                raise ValueError('image dimensions must be positive')
            started = time.perf_counter()
            options = dict(
                imgsz=int(self.cfg['image_size']),
                conf=float(self.cfg['confidence_threshold']),
                iou=float(self.cfg['iou_threshold']),
                device=self.device,
                max_det=int(self.cfg['max_detections']),
                agnostic_nms=bool(self.cfg['agnostic_nms']),
                batch=1,
                rect=False,
                augment=False,
                save=False,
                verbose=False,
            )
            # Avoid a deprecation warning from newer Ultralytics when false.
            if bool(self.cfg['half']):
                options['half'] = True
            result = self.model.predict(image, **options)[0]
            detections = decode_result(
                result, self.cfg['task'], self.cfg['target_classes'],
                float(self.cfg['confidence_threshold']),
                int(self.cfg['max_detections']),
            )
            output = DetectionResult()
            output.header = message.header
            output.image_width = int(image.shape[1])
            output.image_height = int(image.shape[0])
            for index, detection in enumerate(detections):
                item = Detection()
                item.track_id = index + 1
                item.class_id = int(detection.class_id)
                item.class_name = detection.class_name
                item.confidence = float(detection.confidence)
                item.oriented = bool(detection.oriented)
                item.polygon.points = [
                    Point32(x=float(x), y=float(y), z=0.0)
                    for x, y in detection.polygon
                ]
                output.detections.append(item)
            self.detection_pub.publish(output)
            if self.image_pub is not None:
                annotated = self.bridge.cv2_to_imgmsg(
                    self._annotate(image, detections), encoding='bgr8'
                )
                annotated.header = message.header
                self.image_pub.publish(annotated)
            self.frames += 1
            now = time.monotonic()
            if now - self.last_log >= float(self.cfg['log_period_sec']):
                elapsed = now - self.last_log
                self.get_logger().info(
                    f'processed {self.frames} frames; '
                    f'{len(detections)} target detections in last frame; '
                    f'last inference {elapsed / max(self.frames, 1):.3f}s/frame'
                )
                self.frames = 0
                self.last_log = now
            if (time.perf_counter() - started) > 2.0:
                self._warn('YOLO inference exceeded 2 s for one image')
        except (CvBridgeError, ValueError, TypeError, OverflowError) as error:
            self._warn(f'Detection frame rejected: {error}')
        except Exception as error:  # Model/runtime failures should not kill the node.
            self._warn(
                f'Detection inference failed: {type(error).__name__}: {error}\n'
                f'{traceback.format_exc()}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
