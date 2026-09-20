#!/usr/bin/env python3
"""Project a registered point cloud into a segmented camera image."""

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from ins_seg.msg import InstanceInfo, ProjectedInstanceInfo, SegmentationResult


def quaternion_matrix(x, y, z, w):
    """Return a homogeneous rotation matrix for a normalized quaternion."""
    norm = x * x + y * y + z * z + w * w
    if norm < np.finfo(float).eps:
        return np.identity(4)
    scale = 2.0 / norm
    return np.array([
        [1.0 - scale * (y * y + z * z), scale * (x * y - z * w),
         scale * (x * z + y * w), 0.0],
        [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z),
         scale * (y * z - x * w), 0.0],
        [scale * (x * z - y * w), scale * (y * z + x * w),
         1.0 - scale * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


class PointCloudProjection(Node):
    def __init__(self):
        super().__init__('pointcloud_projection')
        defaults = {
            'image_width': 640,
            'image_height': 480,
            'fov_deg': 87.0,
            'fx': 385.45,
            'fy': 385.45,
            'cx': -1.0,
            'cy': -1.0,
            'near_plane': 0.1,
            'far_plane': 100.0,
            'point_size': 1,
            'color_mode': 'instance',
            'pointcloud_topic': '/cloud_registered',
            'odometry_topic': '/Odometry',
            'segmentation_topic': '/yoloe/segmentation',
            'projected_image_topic': '/projected_image',
            'projected_cloud_topic': '/projected_cloud',
            'projected_info_topic': '/projected_instance_info',
            'sync_queue_size': 10,
            'sync_slop': 0.1,
            'coordinate_ema_alpha': 0.6,
            # Input coordinates: world, base, or sensor.
            'pointcloud_frame': 'world',
            'output_frame': '',
            # Homogeneous transform from aircraft base coordinates to camera.
            'base_to_camera': [
                0.0, -1.0, 0.0, 0.0,
                0.0, 0.0, -1.0, 0.0,
                1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
            # Homogeneous transform from LiDAR sensor coordinates to base.
            'lidar_to_base': [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.near_plane = self.get_parameter('near_plane').value
        self.far_plane = self.get_parameter('far_plane').value
        self.point_size = self.get_parameter('point_size').value
        self.color_mode = self.get_parameter('color_mode').value
        self.coordinate_ema_alpha = self.get_parameter('coordinate_ema_alpha').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        if self.fx <= 0.0:
            fov = np.deg2rad(self.get_parameter('fov_deg').value)
            self.fx = self.image_width / (2.0 * np.tan(fov / 2.0))
        if self.fy <= 0.0:
            self.fy = self.fx
        configured_cx = float(self.get_parameter('cx').value)
        configured_cy = float(self.get_parameter('cy').value)
        self.cx = configured_cx if configured_cx >= 0.0 else self.image_width / 2.0
        self.cy = configured_cy if configured_cy >= 0.0 else self.image_height / 2.0
        transform = np.asarray(
            self.get_parameter('base_to_camera').value, dtype=np.float64
        )
        if transform.size != 16:
            raise ValueError('base_to_camera 必须包含 16 个数')
        self.base_to_camera = transform.reshape((4, 4))
        lidar_transform = np.asarray(
            self.get_parameter('lidar_to_base').value, dtype=np.float64
        )
        if lidar_transform.size != 16:
            raise ValueError('lidar_to_base 必须包含 16 个数')
        self.lidar_to_base = lidar_transform.reshape((4, 4))
        self.pointcloud_frame = str(
            self.get_parameter('pointcloud_frame').value
        ).lower()
        if self.pointcloud_frame not in {'world', 'base', 'sensor'}:
            raise ValueError('pointcloud_frame 必须是 world、base 或 sensor')
        self.output_frame = str(self.get_parameter('output_frame').value)
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError('图像尺寸必须大于 0')
        if self.near_plane < 0.0 or self.far_plane <= self.near_plane:
            raise ValueError('投影距离必须满足 0 <= near_plane < far_plane')
        if self.point_size <= 0:
            raise ValueError('point_size 必须大于 0')
        if not 0.0 < self.coordinate_ema_alpha <= 1.0:
            raise ValueError('coordinate_ema_alpha 必须在 (0, 1] 范围内')
        self.bridge = CvBridge()
        self.instance_coordinates = {}

        self.pc_sub = message_filters.Subscriber(
            self, PointCloud2, self.get_parameter('pointcloud_topic').value,
            qos_profile=qos_profile_sensor_data,
        )
        self.odom_sub = message_filters.Subscriber(
            self, Odometry, self.get_parameter('odometry_topic').value,
            qos_profile=qos_profile_sensor_data,
        )
        self.seg_sub = message_filters.Subscriber(
            self, SegmentationResult,
            self.get_parameter('segmentation_topic').value,
            qos_profile=qos_profile_sensor_data,
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.odom_sub, self.seg_sub],
            queue_size=self.get_parameter('sync_queue_size').value,
            slop=self.get_parameter('sync_slop').value,
        )
        self.synchronizer.registerCallback(self.callback)

        self.image_pub = self.create_publisher(
            Image, self.get_parameter('projected_image_topic').value, 1
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter('projected_cloud_topic').value,
            qos_profile_sensor_data,
        )
        self.info_pub = self.create_publisher(
            ProjectedInstanceInfo,
            self.get_parameter('projected_info_topic').value, 1,
        )
        self.get_logger().info(
            f'投影节点已启动，图像尺寸={self.image_width}x{self.image_height}'
        )

    def _read_points(self, message):
        cloud = point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True
        )
        array = np.asarray(cloud)
        if array.dtype.names:
            return np.column_stack([array[name] for name in ('x', 'y', 'z')]).astype(
                np.float64, copy=False
            )
        array = np.asarray(list(cloud), dtype=np.float64)
        return array.reshape((-1, 3))

    def _publish_black_image(self, header):
        image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        message = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        message.header = header
        self.image_pub.publish(message)

    def callback(self, pc_msg, odom_msg, seg_msg):
        try:
            self._process_messages(pc_msg, odom_msg, seg_msg)
        except Exception as exc:
            self.get_logger().error(f'处理点云失败: {exc}')

    def _process_messages(self, pc_msg, odom_msg, seg_msg):
        seg_image = np.zeros(
            (self.image_height, self.image_width, 3), dtype=np.uint8
        )
        if seg_msg.annotated_image.data:
            try:
                seg_image = self.bridge.imgmsg_to_cv2(
                    seg_msg.annotated_image, desired_encoding='bgr8'
                )
            except CvBridgeError as exc:
                self.get_logger().warning(f'标注图像转换失败: {exc}')
        if seg_image.shape[:2] != (self.image_height, self.image_width):
            seg_image = cv2.resize(
                seg_image, (self.image_width, self.image_height),
                interpolation=cv2.INTER_NEAREST,
            )

        labels = {
            int(instance.track_id): instance.class_name or 'unknown'
            for instance in seg_msg.instances
        }
        try:
            id_map = self.bridge.imgmsg_to_cv2(
                seg_msg.instance_id_map, desired_encoding='32SC1'
            )
        except CvBridgeError as exc:
            self.get_logger().warning(f'实例 ID 图转换失败: {exc}')
            id_map = np.zeros(
                (self.image_height, self.image_width), dtype=np.int32
            )
        if id_map.shape != (self.image_height, self.image_width):
            id_map = cv2.resize(
                id_map, (self.image_width, self.image_height),
                interpolation=cv2.INTER_NEAREST,
            ).astype(np.int32)

        points = self._read_points(pc_msg)
        if len(points) == 0:
            self.get_logger().warning('收到空点云')
            return

        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        base_to_world = quaternion_matrix(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        base_to_world[:3, 3] = [position.x, position.y, position.z]
        homogeneous = np.column_stack((points, np.ones(len(points))))
        if self.pointcloud_frame == 'world':
            world_points = points
            base_points_h = (np.linalg.inv(base_to_world) @ homogeneous.T).T
        elif self.pointcloud_frame == 'base':
            base_points_h = homogeneous
            world_points = (base_to_world @ base_points_h.T).T[:, :3]
        else:
            base_points_h = (self.lidar_to_base @ homogeneous.T).T
            world_points = (base_to_world @ base_points_h.T).T[:, :3]
        camera_points = (self.base_to_camera @ base_points_h.T).T[:, :3]

        visible = (
            (camera_points[:, 2] > self.near_plane)
            & (camera_points[:, 2] < self.far_plane)
        )
        camera_points = camera_points[visible]
        world_points = world_points[visible]
        if len(camera_points) == 0:
            self._publish_black_image(pc_msg.header)
            return

        u = np.rint(
            self.fx * camera_points[:, 0] / camera_points[:, 2] + self.cx
        ).astype(np.int32)
        v = np.rint(
            self.fy * camera_points[:, 1] / camera_points[:, 2] + self.cy
        ).astype(np.int32)
        inside = (
            (u >= 0) & (u < self.image_width)
            & (v >= 0) & (v < self.image_height)
        )
        u, v = u[inside], v[inside]
        camera_points, world_points = camera_points[inside], world_points[inside]
        if len(camera_points) == 0:
            self._publish_black_image(pc_msg.header)
            return

        output_header = Header()
        output_header.stamp = pc_msg.header.stamp
        output_header.frame_id = (
            self.output_frame or odom_msg.header.frame_id or pc_msg.header.frame_id
        )
        projected_cloud = point_cloud2.create_cloud_xyz32(
            output_header, world_points.astype(np.float32)
        )
        self.cloud_pub.publish(projected_cloud)

        instance_ids = id_map[v, u]
        frame_ids = [int(value) for value in np.unique(instance_ids) if value != 0]
        info_message = ProjectedInstanceInfo()
        info_message.header = pc_msg.header
        for instance_id in frame_ids:
            instance_points = world_points[instance_ids == instance_id]
            observation = np.mean(instance_points, axis=0)
            previous = self.instance_coordinates.get(instance_id, observation)
            alpha = self.coordinate_ema_alpha
            coordinate = alpha * observation + (1.0 - alpha) * previous
            self.instance_coordinates[instance_id] = coordinate

            item = InstanceInfo()
            item.id = instance_id
            item.label = labels.get(instance_id, 'unknown')
            item.coordinate.x, item.coordinate.y, item.coordinate.z = map(
                float, coordinate
            )
            item.other_ids = [value for value in frame_ids if value != instance_id]
            info_message.instances.append(item)
        self.info_pub.publish(info_message)

        depths = camera_points[:, 2]
        depth_range = np.ptp(depths)
        if depth_range > 0:
            grayscale = 255 - ((depths - depths.min()) / depth_range * 255).astype(
                np.uint8
            )
        else:
            grayscale = np.full(len(depths), 128, dtype=np.uint8)

        image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        for index, (pixel_x, pixel_y) in enumerate(zip(u, v)):
            color = seg_image[pixel_y, pixel_x]
            if self.color_mode != 'instance' or not np.any(color):
                color = (grayscale[index],) * 3
            cv2.circle(
                image, (int(pixel_x), int(pixel_y)), self.point_size,
                tuple(map(int, color)), -1,
            )
        image_message = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        image_message.header = pc_msg.header
        self.image_pub.publish(image_message)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProjection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
