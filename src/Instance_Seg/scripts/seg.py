#!/usr/bin/env python3
"""Segment open-vocabulary concepts in ROS images with Meta SAM 3."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    import torch
    from sam3.model.sam3_image_processor import Sam3Processor
    from sam3.model_builder import build_sam3_image_model
except ImportError as exc:
    raise ImportError(
        "缺少 SAM 3/PyTorch。请按 README 安装 GPU 版 PyTorch 后执行: "
        "python3 -m pip install -r src/Instance_Seg/requirements.txt"
    ) from exc

from ins_seg.msg import SegInfo, SegmentationResult


class Sam3Segmentation(Node):
    """Text-prompted SAM 3 segmentation with lightweight ID continuity."""

    def __init__(self):
        super().__init__('sam3_segmentation')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('result_topic', '/sam3/segmentation')
        self.declare_parameter('annotated_topic', '/sam3/annotated_image')
        self.declare_parameter('text_prompt', 'large ship')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('min_mask_area_ratio', 0.0)
        self.declare_parameter('max_instances', 20)
        self.declare_parameter('tracking_iou_threshold', 0.3)
        self.declare_parameter('max_track_age_frames', 5)
        self.declare_parameter('inference_resolution', 1008)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('compile_model', False)
        self.declare_parameter('show_result', False)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('overlay_alpha', 0.45)
        self.declare_parameter('qos_depth', 1)

        image_topic = str(self.get_parameter('image_topic').value)
        result_topic = str(self.get_parameter('result_topic').value)
        annotated_topic = str(self.get_parameter('annotated_topic').value)
        self.text_prompt = str(self.get_parameter('text_prompt').value).strip()
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.min_mask_area_ratio = float(
            self.get_parameter('min_mask_area_ratio').value
        )
        self.max_instances = int(self.get_parameter('max_instances').value)
        self.tracking_iou_threshold = float(
            self.get_parameter('tracking_iou_threshold').value
        )
        self.max_track_age_frames = int(
            self.get_parameter('max_track_age_frames').value
        )
        self.inference_resolution = int(
            self.get_parameter('inference_resolution').value
        )
        configured_device = str(self.get_parameter('device').value).lower()
        self.compile_model = bool(self.get_parameter('compile_model').value)
        self.show_result = bool(self.get_parameter('show_result').value)
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated').value
        )
        self.overlay_alpha = float(self.get_parameter('overlay_alpha').value)
        qos_depth = int(self.get_parameter('qos_depth').value)
        self._validate_parameters(configured_device, qos_depth)

        self.device = self._resolve_device(configured_device)
        self.bridge = CvBridge()
        self._tracks = {}
        self._next_track_id = 1

        self.get_logger().info(
            '正在加载 Meta SAM 3；首次运行将从 facebook/sam3 自动下载权重'
        )
        try:
            model = build_sam3_image_model(
                checkpoint_path=None,
                load_from_HF=True,
                device=self.device,
                eval_mode=True,
                enable_segmentation=True,
                compile=self.compile_model,
            )
        except Exception as exc:
            raise RuntimeError(
                'SAM 3 加载失败。请先在 Hugging Face 接受 facebook/sam3 '
                '许可并执行 `hf auth login`；节点不使用本地训练权重。'
            ) from exc
        self.processor = Sam3Processor(
            model,
            resolution=self.inference_resolution,
            device=self.device,
            confidence_threshold=self.confidence_threshold,
        )

        self.result_pub = self.create_publisher(
            SegmentationResult, result_topic, qos_depth
        )
        self.annotated_pub = self.create_publisher(
            Image, annotated_topic, qos_depth
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'SAM 3 分割节点已启动: topic={image_topic}, '
            f'prompt="{self.text_prompt}", device={self.device}'
        )

    def _validate_parameters(self, configured_device, qos_depth):
        if not self.text_prompt:
            raise ValueError('text_prompt 不能为空')
        for name, value in (
            ('confidence_threshold', self.confidence_threshold),
            ('min_mask_area_ratio', self.min_mask_area_ratio),
            ('tracking_iou_threshold', self.tracking_iou_threshold),
            ('overlay_alpha', self.overlay_alpha),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f'{name} 必须在 [0, 1] 范围内')
        if self.max_instances <= 0:
            raise ValueError('max_instances 必须大于 0')
        if self.max_track_age_frames < 0:
            raise ValueError('max_track_age_frames 不能为负')
        if self.inference_resolution <= 0:
            raise ValueError('inference_resolution 必须大于 0')
        if qos_depth <= 0:
            raise ValueError('qos_depth 必须大于 0')
        if configured_device not in {'auto', 'cpu', 'cuda'}:
            raise ValueError('device 必须是 auto、cpu 或 cuda')

    @staticmethod
    def _resolve_device(configured_device):
        if configured_device == 'auto':
            return 'cuda' if torch.cuda.is_available() else 'cpu'
        if configured_device == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('配置要求 CUDA，但 PyTorch 未检测到可用 GPU')
        return configured_device

    @staticmethod
    def _to_numpy(tensor):
        if hasattr(tensor, 'detach'):
            tensor = tensor.detach()
        if hasattr(tensor, 'cpu'):
            tensor = tensor.cpu()
        return np.asarray(tensor)

    def _extract_detections(self, state, image_shape):
        masks = self._to_numpy(state.get('masks', []))
        scores = self._to_numpy(state.get('scores', [])).reshape(-1)
        if masks.size == 0 or scores.size == 0:
            return []
        if masks.ndim == 4 and masks.shape[1] == 1:
            masks = masks[:, 0]
        elif masks.ndim == 2:
            masks = masks[None]
        if masks.ndim != 3:
            raise ValueError(f'SAM 3 返回了无法识别的 mask 形状: {masks.shape}')

        count = min(len(masks), len(scores))
        image_area = float(image_shape[0] * image_shape[1])
        detections = []
        for index in range(count):
            mask = masks[index].astype(bool)
            if mask.shape != image_shape[:2]:
                mask = cv2.resize(
                    mask.astype(np.uint8),
                    (image_shape[1], image_shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
            if np.count_nonzero(mask) / image_area < self.min_mask_area_ratio:
                continue
            detections.append((mask, float(scores[index])))
        detections.sort(key=lambda item: item[1], reverse=True)
        return detections[:self.max_instances]

    @staticmethod
    def _mask_iou(first, second):
        if first.shape != second.shape:
            return 0.0
        intersection = np.count_nonzero(first & second)
        if intersection == 0:
            return 0.0
        union = np.count_nonzero(first | second)
        return float(intersection / union) if union else 0.0

    def _assign_track_ids(self, detections):
        candidates = []
        for detection_index, (mask, _) in enumerate(detections):
            for track_id, track in self._tracks.items():
                iou = self._mask_iou(mask, track['mask'])
                if iou >= self.tracking_iou_threshold:
                    candidates.append((iou, detection_index, track_id))
        candidates.sort(reverse=True)

        assignments = {}
        used_tracks = set()
        for _, detection_index, track_id in candidates:
            if detection_index in assignments or track_id in used_tracks:
                continue
            assignments[detection_index] = track_id
            used_tracks.add(track_id)

        for detection_index in range(len(detections)):
            if detection_index not in assignments:
                assignments[detection_index] = self._next_track_id
                self._next_track_id += 1

        next_tracks = {}
        for detection_index, (mask, _) in enumerate(detections):
            track_id = assignments[detection_index]
            next_tracks[track_id] = {'mask': mask, 'age': 0}
        for track_id, track in self._tracks.items():
            if track_id in used_tracks:
                continue
            age = int(track['age']) + 1
            if age <= self.max_track_age_frames:
                next_tracks[track_id] = {'mask': track['mask'], 'age': age}
        self._tracks = next_tracks
        return [assignments[index] for index in range(len(detections))]

    @staticmethod
    def _track_color(track_id):
        return (
            int((37 * track_id + 53) % 205 + 50),
            int((79 * track_id + 31) % 205 + 50),
            int((131 * track_id + 17) % 205 + 50),
        )

    def _annotate(self, image, detections, track_ids):
        annotated = image.copy()
        for (mask, score), track_id in zip(detections, track_ids):
            color = self._track_color(track_id)
            pixels = annotated[mask].astype(np.float32)
            annotated[mask] = np.clip(
                (1.0 - self.overlay_alpha) * pixels
                + self.overlay_alpha * np.asarray(color, dtype=np.float32),
                0,
                255,
            ).astype(np.uint8)
            contours, _ = cv2.findContours(
                mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            cv2.drawContours(annotated, contours, -1, color, 2)
            rows, columns = np.nonzero(mask)
            if len(columns):
                cv2.putText(
                    annotated,
                    f'{self.text_prompt} #{track_id} {score:.2f}',
                    (int(columns.min()), max(20, int(rows.min()) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2.LINE_AA,
                )
        return annotated

    def _publish_result(self, msg, image, detections):
        track_ids = self._assign_track_ids(detections)
        id_map = np.zeros(image.shape[:2], dtype=np.int32)
        instances = []
        # Higher-confidence masks win where SAM 3 masks overlap.
        for detection_index in reversed(range(len(detections))):
            mask, _ = detections[detection_index]
            id_map[mask] = track_ids[detection_index]
        for (_, score), track_id in zip(detections, track_ids):
            instance = SegInfo()
            instance.track_id = int(track_id)
            instance.class_name = self.text_prompt
            instance.confidence = float(score)
            instances.append(instance)

        output = SegmentationResult()
        output.header = msg.header
        output.instance_id_map = self.bridge.cv2_to_imgmsg(
            id_map, encoding='32SC1'
        )
        output.instance_id_map.header = msg.header
        output.instances = instances

        annotated = None
        if self.publish_annotated or self.show_result:
            annotated = self._annotate(image, detections, track_ids)
            output.annotated_image = self.bridge.cv2_to_imgmsg(
                annotated, encoding='bgr8'
            )
            output.annotated_image.header = msg.header
        if self.publish_annotated:
            self.annotated_pub.publish(output.annotated_image)
        self.result_pub.publish(output)
        if self.show_result and annotated is not None:
            cv2.imshow('SAM 3 海上大型船只分割', annotated)
            cv2.waitKey(1)

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            state = self.processor.set_image(PilImage.fromarray(rgb))
            state = self.processor.set_text_prompt(
                prompt=self.text_prompt, state=state
            )
            detections = self._extract_detections(state, image.shape)
            self._publish_result(msg, image, detections)
        except (CvBridgeError, Exception) as exc:
            self.get_logger().error(f'SAM 3 图像处理或推理失败: {exc}')

    def destroy_node(self):
        if self.show_result:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Sam3Segmentation()
        rclpy.spin(node)
    except KeyboardInterrupt:
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
