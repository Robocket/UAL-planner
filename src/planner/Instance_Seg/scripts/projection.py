#!/usr/bin/env python3
import rospy
import numpy as np
import sys
import cv2

if tuple(int(x) for x in np.__version__.split('.')[:2]) >= (2, 0):
    sys.stderr.write(
        f"ERROR: cv_bridge is incompatible with NumPy {np.__version__}.\n"
        "ROS Noetic / cv_bridge requires numpy<2.0.\n"
        "Please downgrade NumPy in your Python environment, e.g. `pip install 'numpy<2'`.\n"
    )
    raise SystemExit(1)

from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices
import message_filters

class PointCloudToImage:
    def __init__(self):
        # 使用仿真时间，配合 rosbag play --clock
        rospy.set_param('/use_sim_time', True)
        # ROS节点初始化
        rospy.init_node('pointcloud_to_image_node', anonymous=True)
        
        # 参数配置
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        self.fov_deg = rospy.get_param('~fov_deg', 87)  # 水平视场角
        self.near_plane = rospy.get_param('~near_plane', 0.1)
        self.far_plane = rospy.get_param('~far_plane', 100.0)
        self.point_size = rospy.get_param('~point_size', 1)
        self.color_mode = rospy.get_param('~color_mode', 'depth')  # 'depth' or 'height'
        self.seg_image_topic = rospy.get_param('~seg_image_topic', '/yoloe/segmentation_result')
        
        # 相机内参计算
        self.fov_rad = np.deg2rad(self.fov_deg)
        self.fx = 385.45 # 焦距（像素) self.image_width / (2 * np.tan(self.fov_rad / 2))
        self.fy = self.fx  # 假设像素是正方形
        self.cx = self.image_width / 2
        self.cy = self.image_height / 2
        
        # 内参矩阵
        self.K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])
        
        # 初始化CV桥
        self.bridge = CvBridge()
        
        # 订阅话题（使用时间同步）
        self.pc_sub = message_filters.Subscriber('/cloud_registered', PointCloud2)
        self.odom_sub = message_filters.Subscriber('/Odometry', Odometry)
        self.seg_sub = message_filters.Subscriber(self.seg_image_topic, Image)
        
        # 时间同步器（近似时间同步）
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.odom_sub, self.seg_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.callback)
        
        # 发布图像话题
        self.image_pub = rospy.Publisher('/projected_image', Image, queue_size=1)
        
        rospy.loginfo("PointCloud to Image node initialized")
        rospy.loginfo(f"Image size: {self.image_width}x{self.image_height}")
        rospy.loginfo(f"FOV: {self.fov_deg} degrees")
        rospy.loginfo(f"Color mode: {self.color_mode}")
        rospy.loginfo(f"Segmentation image topic: {self.seg_image_topic}")

    def callback(self, pc_msg, odom_msg, seg_msg):
        try:
            # 1. 读取分割结果图像，并与投影图像保持一致尺寸
            try:
                seg_image = self.bridge.imgmsg_to_cv2(seg_msg, desired_encoding="bgr8")
            except CvBridgeError as e:
                rospy.logwarn(f"Unable to convert segmentation image: {e}")
                return
            if seg_image.shape[0:2] != (self.image_height, self.image_width):
                seg_image = cv2.resize(seg_image, (self.image_width, self.image_height), interpolation=cv2.INTER_NEAREST)

            # 2. 将PointCloud2转换为numpy数组
            gen = pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array(list(gen), dtype=np.float32)
            
            if points.shape[0] == 0:
                rospy.logwarn("Received empty point cloud")
                return
            
            # 3. 从里程计获取位姿
            position = odom_msg.pose.pose.position
            orientation = odom_msg.pose.pose.orientation
            
            # 4. 构建变换矩阵：世界坐标系 -> 相机坐标系
            trans = translation_matrix([position.x, position.y, position.z])
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            rot_world_to_base = quaternion_matrix(q)
            base_to_cam = np.array([
                [0, -1, 0, 0],
                [0, 0, -1, 0],
                [1, 0, 0, 0],
                [0, 0, 0, 1]
            ])
            world_to_cam = concatenate_matrices(base_to_cam, np.linalg.inv(rot_world_to_base), np.linalg.inv(trans))
            
            # 5. 将点云转换到相机坐标系
            points_hom = np.hstack((points, np.ones((points.shape[0], 1))))
            points_cam_hom = np.dot(world_to_cam, points_hom.T).T
            points_cam = points_cam_hom[:, :3]
            
            mask = (points_cam[:, 2] > self.near_plane) & (points_cam[:, 2] < self.far_plane)
            points_cam_filtered = points_cam[mask]
            
            if points_cam_filtered.shape[0] == 0:
                rospy.logwarn("No points in view frustum")
                black_image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(black_image, "bgr8"))
                return
            
            # 6. 透视投影
            u = (self.fx * points_cam_filtered[:, 0] / points_cam_filtered[:, 2]) + self.cx
            v = (self.fy * points_cam_filtered[:, 1] / points_cam_filtered[:, 2]) + self.cy
            u = np.round(u).astype(np.int32)
            v = np.round(v).astype(np.int32)
            
            mask = (u >= 0) & (u < self.image_width) & (v >= 0) & (v < self.image_height)
            u = u[mask]
            v = v[mask]
            points_cam_final = points_cam_filtered[mask]
            
            if points_cam_final.shape[0] == 0:
                rospy.logwarn("No points projected onto image")
                black_image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(black_image, "bgr8"))
                return
            
            # 7. 生成彩色图像，默认黑色背景
            image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
            
            # 根据颜色模式计算基础灰度值
            if self.color_mode == 'depth':
                depths = points_cam_final[:, 2]
                min_depth = np.min(depths)
                max_depth = np.max(depths)
                if max_depth > min_depth:
                    normalized_depth = (depths - min_depth) / (max_depth - min_depth)
                    normalized_depth = np.clip(normalized_depth, 0.0, 1.0)
                    pixel_values = 255 - (normalized_depth * 255).astype(np.uint8)
                else:
                    pixel_values = np.ones_like(depths, dtype=np.uint8) * 128
            elif self.color_mode == 'height':
                heights = points_cam_final[:, 1]
                min_height = np.min(heights)
                max_height = np.max(heights)
                if max_height > min_height:
                    normalized_height = (heights - min_height) / (max_height - min_height)
                    pixel_values = (normalized_height * 255).astype(np.uint8)
                else:
                    pixel_values = np.ones_like(heights, dtype=np.uint8) * 128
            else:
                pixel_values = np.ones_like(u, dtype=np.uint8) * 255
            
            # 8. 绘制点云点，使用分割图像颜色覆盖对应像素
            for i in range(len(u)):
                seg_color = seg_image[v[i], u[i]]
                if np.any(seg_color != 0):
                    point_color = (int(seg_color[0]), int(seg_color[1]), int(seg_color[2]))
                else:
                    gray = int(pixel_values[i])
                    point_color = (gray, gray, gray)
                cv2.circle(image, (u[i], v[i]), self.point_size, point_color, -1)
            
            # 9. 发布彩色图像
            img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            img_msg.header = pc_msg.header
            self.image_pub.publish(img_msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")

if __name__ == '__main__':
    try:
        node = PointCloudToImage()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
