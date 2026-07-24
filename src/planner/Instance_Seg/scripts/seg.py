#!/usr/bin/env python3
"""Run YOLO instance segmentation/tracking on a ROS 2 image topic."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
try:
    from ultralytics import YOLO
except ImportError as exc:
    raise ImportError(
        "缺少 ultralytics。请执行: python3 -m pip install --user "
        "-r src/planner/Instance_Seg/requirements.txt"
    ) from exc

from ins_seg.msg import SegInfo, SegmentationResult


class YoloSegmentationTracker(Node):
    def __init__(self):
        super().__init__('yolo_segmentation_tracker')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('result_topic', '/yoloe/segmentation')
        self.declare_parameter('annotated_topic', '/yoloe/annotated_image')
        self.declare_parameter('model_path', 'yolo11n-seg.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('tracker_type', 'botsort.yaml')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('show_result', False)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('classes', [])

        image_topic = self.get_parameter('image_topic').value
        result_topic = self.get_parameter('result_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.tracker_type = self.get_parameter('tracker_type').value
        self.device = self.get_parameter('device').value
        self.show_result = self.get_parameter('show_result').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        configured_classes = self.get_parameter('classes').value
        self.classes = list(configured_classes) if configured_classes else None

        self.bridge = CvBridge()
        self.get_logger().info(f'正在加载模型: {model_path}')
        self.model = YOLO(model_path)

        self.result_pub = self.create_publisher(SegmentationResult, result_topic, 1)
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 1)
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info(f'实例分割节点已启动，订阅: {image_topic}')

    def _build_instance_id_map(self, result, image_shape):
        id_map = np.zeros(image_shape[:2], dtype=np.int32)
        instances = []

        if result.boxes is None or result.boxes.id is None:
            return id_map, instances

        track_ids = result.boxes.id.cpu().numpy().astype(int)
        class_ids = result.boxes.cls.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()
        masks = result.masks.data.cpu().numpy() if result.masks is not None else []

        for index, track_id in enumerate(track_ids):
            instance = SegInfo()
            instance.track_id = int(track_id)
            instance.class_name = str(result.names[int(class_ids[index])])
            instance.confidence = float(confidences[index])
            instances.append(instance)

            if index >= len(masks):
                continue
            mask = masks[index].astype(np.uint8)
            if mask.shape != image_shape[:2]:
                mask = cv2.resize(
                    mask, (image_shape[1], image_shape[0]),
                    interpolation=cv2.INTER_NEAREST
                )
            id_map[mask.astype(bool)] = int(track_id)

        return id_map, instances

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model.track(
                source=cv_image,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                tracker=self.tracker_type,
                device=self.device,
                classes=self.classes,
                persist=True,
                verbose=False,
            )
        except (CvBridgeError, Exception) as exc:
            self.get_logger().error(f'图像处理或推理失败: {exc}')
            return

        if not results:
            return

        result = results[0]
        id_map, instances = self._build_instance_id_map(result, cv_image.shape)
        output = SegmentationResult()
        output.header = msg.header
        output.instance_id_map = self.bridge.cv2_to_imgmsg(id_map, encoding='32SC1')
        output.instance_id_map.header = msg.header
        output.instances = instances

        annotated_image = None
        if self.publish_annotated or self.show_result:
            annotated_image = result.plot()
            output.annotated_image = self.bridge.cv2_to_imgmsg(
                annotated_image, encoding='bgr8'
            )
            output.annotated_image.header = msg.header
        if self.publish_annotated:
            self.annotated_pub.publish(output.annotated_image)

        self.result_pub.publish(output)
        if self.show_result and annotated_image is not None:
            cv2.imshow('实例分割与跟踪结果', annotated_image)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_result:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloSegmentationTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
