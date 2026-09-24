#!/usr/bin/env python3
"""Segment open-vocabulary concepts in ROS images with Meta SAM 3."""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    import torch
    from modelscope import snapshot_download
    from torchao.quantization import (
        Int8DynamicActivationInt4WeightConfig,
        Int8DynamicActivationInt8WeightConfig,
        Int8WeightOnlyConfig,
        quantize_,
    )
    from transformers import Sam3Model, Sam3Processor
except ImportError as exc:
    raise ImportError(
        "缺少 SAM 3/ModelScope/TorchAO 运行依赖。请按 README 安装 GPU 版 "
        "PyTorch 后执行: "
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
        self.declare_parameter('mask_threshold', 0.5)
        self.declare_parameter('min_mask_area_ratio', 0.0)
        self.declare_parameter('max_instances', 20)
        self.declare_parameter('tracking_iou_threshold', 0.3)
        self.declare_parameter('max_track_age_frames', 5)
        self.declare_parameter('inference_resolution', 1008)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('model_id', 'facebook/sam3')
        self.declare_parameter('model_revision', 'master')
        self.declare_parameter('model_cache_dir', '')
        self.declare_parameter('model_local_files_only', False)
        self.declare_parameter('model_download_max_workers', 4)
        self.declare_parameter('quantization', 'int8_dynamic')
        self.declare_parameter('int4_group_size', 128)
        self.declare_parameter('compute_dtype', 'float32')
        self.declare_parameter('compile_model', False)
        self.declare_parameter('performance_topic', '/sam3/performance')
        self.declare_parameter('performance_log_period_frames', 1)
        self.declare_parameter('performance_warn_latency_ms', 1000.0)
        self.declare_parameter('performance_ema_alpha', 0.2)
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
        self.mask_threshold = float(self.get_parameter('mask_threshold').value)
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
        self.model_id = str(self.get_parameter('model_id').value).strip()
        self.model_revision = str(
            self.get_parameter('model_revision').value
        ).strip()
        model_cache_dir = str(
            self.get_parameter('model_cache_dir').value
        ).strip()
        self.model_cache_dir = model_cache_dir or None
        self.model_local_files_only = bool(
            self.get_parameter('model_local_files_only').value
        )
        self.model_download_max_workers = int(
            self.get_parameter('model_download_max_workers').value
        )
        self.quantization = str(
            self.get_parameter('quantization').value
        ).strip().lower()
        self.int4_group_size = int(
            self.get_parameter('int4_group_size').value
        )
        self.compute_dtype = str(
            self.get_parameter('compute_dtype').value
        ).strip().lower()
        self.compile_model = bool(self.get_parameter('compile_model').value)
        self.performance_topic = str(
            self.get_parameter('performance_topic').value
        ).strip()
        self.performance_log_period_frames = int(
            self.get_parameter('performance_log_period_frames').value
        )
        self.performance_warn_latency_ms = float(
            self.get_parameter('performance_warn_latency_ms').value
        )
        self.performance_ema_alpha = float(
            self.get_parameter('performance_ema_alpha').value
        )
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
        self._processed_frames = 0
        self._latency_ema_ms = None

        self.model, self.processor = self._load_model()
        self.text_features, self.text_attention_mask = self._prepare_text_prompt()

        self.result_pub = self.create_publisher(
            SegmentationResult, result_topic, qos_depth
        )
        self.annotated_pub = self.create_publisher(
            Image, annotated_topic, qos_depth
        )
        self.performance_pub = self.create_publisher(
            DiagnosticArray, self.performance_topic, qos_depth
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'SAM 3 分割节点已启动: topic={image_topic}, '
            f'prompt="{self.text_prompt}", device={self.device}, '
            f'quantization={self.quantization}'
        )

    def _validate_parameters(self, configured_device, qos_depth):
        if not self.text_prompt:
            raise ValueError('text_prompt 不能为空')
        for name, value in (
            ('confidence_threshold', self.confidence_threshold),
            ('mask_threshold', self.mask_threshold),
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
        if not self.model_id:
            raise ValueError('model_id 不能为空')
        if not self.model_revision:
            raise ValueError('model_revision 不能为空')
        if self.model_download_max_workers <= 0:
            raise ValueError('model_download_max_workers 必须大于 0')
        if self.quantization not in {
            'int4_dynamic', 'int8_dynamic', 'int8_weight_only', 'none',
        }:
            raise ValueError(
                'quantization 必须是 int4_dynamic、int8_dynamic、'
                'int8_weight_only 或 none'
            )
        if self.int4_group_size not in {16, 32, 64, 128, 256}:
            raise ValueError(
                'int4_group_size 必须是 16、32、64、128 或 256'
            )
        if self.compute_dtype not in {'float32', 'float16'}:
            raise ValueError('compute_dtype 必须是 float32 或 float16')
        if not self.performance_topic:
            raise ValueError('performance_topic 不能为空')
        if self.performance_log_period_frames <= 0:
            raise ValueError('performance_log_period_frames 必须大于 0')
        if self.performance_warn_latency_ms <= 0.0:
            raise ValueError('performance_warn_latency_ms 必须大于 0')
        if not 0.0 < self.performance_ema_alpha <= 1.0:
            raise ValueError('performance_ema_alpha 必须在 (0, 1] 范围内')
        if qos_depth <= 0:
            raise ValueError('qos_depth 必须大于 0')
        if configured_device not in {'auto', 'cpu', 'cuda'}:
            raise ValueError('device 必须是 auto、cpu 或 cuda')

    def _load_model(self):
        mode = '仅使用本地缓存' if self.model_local_files_only else '自动下载/更新'
        self.get_logger().info(
            f'正在从魔塔加载 {self.model_id}@{self.model_revision} '
            f'({mode})'
        )
        try:
            model_dir = snapshot_download(
                self.model_id,
                revision=self.model_revision,
                cache_dir=self.model_cache_dir,
                local_files_only=self.model_local_files_only,
                allow_patterns=[
                    '*.json', '*.txt', '*.safetensors', 'LICENSE', 'README.md',
                ],
                ignore_patterns=['sam3.pt'],
                max_workers=self.model_download_max_workers,
            )
            dtype = {
                'float32': torch.float32,
                'float16': torch.float16,
            }[self.compute_dtype]
            model = Sam3Model.from_pretrained(model_dir, dtype=dtype).eval()
            if self.quantization == 'int4_dynamic':
                quantize_(
                    model,
                    Int8DynamicActivationInt4WeightConfig(
                        group_size=self.int4_group_size,
                    ),
                )
            elif self.quantization == 'int8_dynamic':
                quantize_(
                    model,
                    Int8DynamicActivationInt8WeightConfig(version=2),
                )
            elif self.quantization == 'int8_weight_only':
                # Quantize on CPU before moving the model. On Turing GPUs,
                # quantizing an FP16 model on-device produces NaN SAM outputs.
                quantize_(model, Int8WeightOnlyConfig(version=2))
            model = model.to(self.device)
            if self.device == 'cuda':
                torch.cuda.empty_cache()
            if self.compile_model:
                # Compile only the dominant vision encoder so helper methods
                # such as get_text_features remain available.
                model.vision_encoder = torch.compile(
                    model.vision_encoder,
                    mode='reduce-overhead',
                    fullgraph=False,
                )
            processor = Sam3Processor.from_pretrained(model_dir)
        except Exception as exc:
            raise RuntimeError(
                f'从魔塔加载 SAM 3 失败 ({self.model_id}@'
                f'{self.model_revision})：{exc}'
            ) from exc

        processor_size = processor.image_processor.size
        native_resolution = int(processor_size['height'])
        if (
            native_resolution != int(processor_size['width'])
            or self.inference_resolution != native_resolution
        ):
            raise ValueError(
                '当前 SAM 3 权重的二维旋转位置编码要求输入分辨率为 '
                f'{native_resolution}，但 inference_resolution='
                f'{self.inference_resolution}'
            )
        if self.device == 'cuda':
            allocated_mib = torch.cuda.memory_allocated() / (1024 * 1024)
            self.get_logger().info(
                f'SAM 3 加载完成，CUDA 已分配约 {allocated_mib:.0f} MiB'
            )
        return model, processor

    def _prepare_text_prompt(self):
        text_inputs = self.processor(
            text=self.text_prompt,
            return_tensors='pt',
        ).to(self.device)
        with torch.inference_mode():
            text_features = self.model.get_text_features(
                input_ids=text_inputs['input_ids'],
                attention_mask=text_inputs['attention_mask'],
                return_dict=True,
            )
        return text_features, text_inputs['attention_mask']

    @staticmethod
    def _diagnostic_value(key, value):
        item = KeyValue()
        item.key = key
        item.value = str(value)
        return item

    def _publish_performance(
        self,
        detection_count,
        preprocess_ms,
        inference_ms,
        postprocess_ms,
        publish_ms,
        total_ms,
    ):
        alpha = self.performance_ema_alpha
        if self._latency_ema_ms is None:
            self._latency_ema_ms = total_ms
        else:
            self._latency_ema_ms = (
                alpha * total_ms + (1.0 - alpha) * self._latency_ema_ms
            )

        allocated_mib = reserved_mib = peak_mib = 0.0
        if self.device == 'cuda':
            allocated_mib = torch.cuda.memory_allocated() / (1024 * 1024)
            reserved_mib = torch.cuda.memory_reserved() / (1024 * 1024)
            peak_mib = torch.cuda.max_memory_allocated() / (1024 * 1024)

        status = DiagnosticStatus()
        status.name = 'sam3_segmentation/performance'
        status.hardware_id = (
            torch.cuda.get_device_name(0) if self.device == 'cuda' else 'cpu'
        )
        status.level = (
            DiagnosticStatus.WARN
            if total_ms > self.performance_warn_latency_ms
            else DiagnosticStatus.OK
        )
        status.message = (
            'inference latency above configured limit'
            if status.level == DiagnosticStatus.WARN
            else 'ok'
        )
        status.values = [
            self._diagnostic_value('frame_index', self._processed_frames),
            self._diagnostic_value('detections', detection_count),
            self._diagnostic_value('preprocess_ms', f'{preprocess_ms:.2f}'),
            self._diagnostic_value('inference_ms', f'{inference_ms:.2f}'),
            self._diagnostic_value('postprocess_ms', f'{postprocess_ms:.2f}'),
            self._diagnostic_value('publish_ms', f'{publish_ms:.2f}'),
            self._diagnostic_value('total_ms', f'{total_ms:.2f}'),
            self._diagnostic_value(
                'effective_fps', f'{1000.0 / self._latency_ema_ms:.3f}'
            ),
            self._diagnostic_value(
                'cuda_allocated_mib', f'{allocated_mib:.1f}'
            ),
            self._diagnostic_value('cuda_reserved_mib', f'{reserved_mib:.1f}'),
            self._diagnostic_value('cuda_peak_mib', f'{peak_mib:.1f}'),
            self._diagnostic_value('quantization', self.quantization),
            self._diagnostic_value('int4_group_size', self.int4_group_size),
            self._diagnostic_value('compute_dtype', self.compute_dtype),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.performance_pub.publish(message)

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
            started_at = time.perf_counter()
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            inputs = self.processor(
                images=PilImage.fromarray(rgb),
                return_tensors='pt',
            )
            inputs = inputs.to(self.device)
            preprocess_done_at = time.perf_counter()
            if self.device == 'cuda':
                torch.cuda.reset_peak_memory_stats()
                torch.cuda.synchronize()
            inference_started_at = time.perf_counter()
            with torch.inference_mode():
                outputs = self.model(
                    pixel_values=inputs['pixel_values'],
                    text_embeds=self.text_features,
                    attention_mask=self.text_attention_mask,
                )
            if self.device == 'cuda':
                torch.cuda.synchronize()
            inference_done_at = time.perf_counter()
            for name in ('pred_logits', 'presence_logits', 'pred_masks'):
                if not torch.isfinite(getattr(outputs, name)).all():
                    raise RuntimeError(
                        f'SAM 3 输出 {name} 含 NaN/Inf；请使用默认的 '
                        'compute_dtype=float32'
                    )
            state = self.processor.post_process_instance_segmentation(
                outputs,
                threshold=self.confidence_threshold,
                mask_threshold=self.mask_threshold,
                target_sizes=inputs['original_sizes'].tolist(),
            )[0]
            detections = self._extract_detections(state, image.shape)
            postprocess_done_at = time.perf_counter()
            self._publish_result(msg, image, detections)
            publish_done_at = time.perf_counter()
            self._processed_frames += 1
            preprocess_ms = (preprocess_done_at - started_at) * 1000.0
            inference_ms = (inference_done_at - inference_started_at) * 1000.0
            postprocess_ms = (
                postprocess_done_at - inference_done_at
            ) * 1000.0
            publish_ms = (publish_done_at - postprocess_done_at) * 1000.0
            total_ms = (publish_done_at - started_at) * 1000.0
            self._publish_performance(
                len(detections),
                preprocess_ms,
                inference_ms,
                postprocess_ms,
                publish_ms,
                total_ms,
            )
            if self._processed_frames % self.performance_log_period_frames == 0:
                self.get_logger().info(
                    f'SAM 3 第 {self._processed_frames} 帧: '
                    f'{len(detections)} 个实例, total={total_ms:.0f} ms '
                    f'(pre={preprocess_ms:.0f}, infer={inference_ms:.0f}, '
                    f'post={postprocess_ms:.0f}, pub={publish_ms:.0f}), '
                    f'EMA FPS={1000.0 / self._latency_ema_ms:.3f}'
                )
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
