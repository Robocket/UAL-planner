#!/usr/bin/env python3
"""Convert a sensor_msgs/CompressedImage stream to sensor_msgs/Image."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CompressedImageAdapter(Node):
    """Expose a compressed camera stream through the standard raw-image API."""

    def __init__(self):
        super().__init__('compressed_image_adapter')

        self.declare_parameter(
            'input_topic', '/left_camera/image/compressed'
        )
        self.declare_parameter('output_topic', '/left_camera/image')
        self.declare_parameter('output_encoding', 'bgr8')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('qos_depth', 1)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.output_encoding = str(
            self.get_parameter('output_encoding').value
        )
        self.frame_id = str(self.get_parameter('frame_id').value)
        qos_depth = int(self.get_parameter('qos_depth').value)

        if not input_topic or not output_topic:
            raise ValueError('input_topic 和 output_topic 不能为空')
        if input_topic == output_topic:
            raise ValueError('压缩图像输入和原始图像输出必须使用不同话题')
        if self.output_encoding not in {'bgr8', 'rgb8', 'mono8'}:
            raise ValueError('output_encoding 仅支持 bgr8、rgb8 或 mono8')
        if qos_depth <= 0:
            raise ValueError('qos_depth 必须大于 0')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, output_topic, qos_depth
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'压缩图像适配器已启动: {input_topic} -> {output_topic}'
        )

    def _image_callback(self, message):
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
        if image is None:
            self.get_logger().warning(
                '无法解码压缩图像', throttle_duration_sec=5.0
            )
            return

        if self.output_encoding == 'bgr8':
            if image.ndim == 2:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            elif image.shape[2] == 4:
                image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        elif self.output_encoding == 'rgb8':
            if image.ndim == 2:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
            elif image.shape[2] == 4:
                image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
            else:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        output = self.bridge.cv2_to_imgmsg(
            image, encoding=self.output_encoding
        )
        output.header = message.header
        if self.frame_id:
            output.header.frame_id = self.frame_id
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
