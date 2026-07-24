#!/usr/bin/env python3
"""Maintain a spatially smoothed store of projected instances."""

import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from ins_seg.msg import ProjectedInstanceInfo


class InstanceGraph(Node):
    def __init__(self):
        super().__init__('instance_graph')
        self.declare_parameter('info_topic', '/projected_instance_info')
        self.declare_parameter('max_instances', 1000)
        self.declare_parameter('distance_threshold', 0.5)
        self.declare_parameter('ema_alpha', 0.6)

        self.instance_dict: Dict[int, Dict] = {}
        self.max_instances = self.get_parameter('max_instances').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        topic = self.get_parameter('info_topic').value

        self.subscriber = self.create_subscription(
            ProjectedInstanceInfo, topic, self.instance_callback, 10
        )
        self.log_timer = self.create_timer(1.0, self.log_status)

    @staticmethod
    def _euclidean_distance(point_a, point_b):
        return math.sqrt(sum(
            (point_a[axis] - point_b[axis]) ** 2 for axis in ('x', 'y', 'z')
        ))

    def _update_with_ema(self, old_instance, new_instance):
        alpha = self.ema_alpha
        old_coordinate = old_instance['coordinate']
        new_coordinate = new_instance['coordinate']
        old_instance['coordinate'] = {
            axis: alpha * new_coordinate[axis] + (1.0 - alpha) * old_coordinate[axis]
            for axis in ('x', 'y', 'z')
        }
        old_instance['label'] = new_instance['label'] or old_instance['label']
        old_instance['other_ids'] = new_instance['other_ids']
        old_instance['timestamp'] = new_instance['timestamp']

    def _find_same_label_match(self, instance_data) -> Optional[Tuple[int, Dict, float]]:
        candidates = [
            (instance_id, instance)
            for instance_id, instance in self.instance_dict.items()
            if instance.get('label') == instance_data['label']
            and instance_id != instance_data['id']
        ]
        if not candidates:
            return None
        best_id, best_instance = min(
            candidates,
            key=lambda item: self._euclidean_distance(
                instance_data['coordinate'], item[1]['coordinate']
            ),
        )
        distance = self._euclidean_distance(
            instance_data['coordinate'], best_instance['coordinate']
        )
        return best_id, best_instance, distance

    def instance_callback(self, msg):
        try:
            for instance in msg.instances:
                instance_data = {
                    'id': int(instance.id),
                    'label': instance.label,
                    'coordinate': {
                        'x': float(instance.coordinate.x),
                        'y': float(instance.coordinate.y),
                        'z': float(instance.coordinate.z),
                    },
                    'other_ids': list(instance.other_ids),
                    'timestamp': time.monotonic(),
                }
                instance_id = instance_data['id']
                if instance_id in self.instance_dict:
                    self._update_with_ema(self.instance_dict[instance_id], instance_data)
                    continue

                match = self._find_same_label_match(instance_data)
                if match is not None and match[2] <= self.distance_threshold:
                    self._update_with_ema(match[1], instance_data)
                    continue

                self.instance_dict[instance_id] = instance_data
                self.get_logger().info(
                    f"新增实例 {instance_id}，类别={instance_data['label']}"
                )

            while len(self.instance_dict) > self.max_instances:
                oldest_id = min(
                    self.instance_dict,
                    key=lambda key: self.instance_dict[key]['timestamp'],
                )
                del self.instance_dict[oldest_id]
        except Exception as exc:
            self.get_logger().error(f'处理实例消息失败: {exc}')

    def log_status(self):
        self.get_logger().info(f'当前存储实例数: {len(self.instance_dict)}')


def main(args=None):
    rclpy.init(args=args)
    node = InstanceGraph()
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
