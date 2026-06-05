#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS节点：订阅RGB图像话题，使用YOLO26/YOLOE进行实例分割和多目标跟踪
发布处理后的图像到/yoloe/segmentation_result话题
"""

import rospy
import cv2
import numpy as np
import sys

if tuple(int(x) for x in np.__version__.split('.')[:2]) >= (2, 0):
    sys.stderr.write(
        f"ERROR: cv_bridge is incompatible with NumPy {np.__version__}.\n"
        "ROS Noetic / cv_bridge requires numpy<2.0.\n"
        "Please downgrade NumPy in your Python environment, e.g. `pip install 'numpy<2'`.\n"
    )
    raise SystemExit(1)

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO  # 统一使用YOLO类

class YoloeSegmentationTracker:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('yoloe_seg_node', anonymous=True)
        
        # 获取ROS参数（与你的launch文件参数名一致）
        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.model_path = rospy.get_param('~model_path', '/home/k325/models/YoloE/yolo26n-seg.pt')
        self.conf_threshold = rospy.get_param('~confidence_threshold', 0.5)  # 与launch文件参数名匹配
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.5)
        self.tracker_type = rospy.get_param('~tracker_type', 'botsort.yaml')
        self.device = rospy.get_param('~device', 'cuda:0')  # 或 'cpu'
        self.show_result = rospy.get_param('~show_result', False)
        self.publish_result = rospy.get_param('~publish_result', True)
        self.classes = rospy.get_param('~classes', None)  # 例如: [0, 2, 3] 对应person, car, motorcycle
        
        # 初始化CV Bridge
        try:
            self.bridge = CvBridge()
        except Exception as e:
            rospy.logerr(
                "无法初始化 cv_bridge。这通常是由于 NumPy 2.x 与 ROS Noetic cv_bridge 不兼容。"
            )
            rospy.logerr(f"详细错误: {e}")
            rospy.signal_shutdown("cv_bridge 初始化失败")
            return
        
        # 加载模型（修正后的加载方式）
        rospy.loginfo(f"正在加载模型: {self.model_path}")
        try:
            # 所有模型统一使用YOLO()类加载，直接传入路径
            self.model = YOLO(self.model_path)
            rospy.loginfo("模型加载成功")
        except Exception as e:
            rospy.logerr(f"模型加载失败: {e}")
            rospy.signal_shutdown("模型加载失败")
            return
        
        # 设置发布者
        if self.publish_result:
            self.result_pub = rospy.Publisher('/yoloe/segmentation_result', Image, queue_size=1)
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        rospy.loginfo(f"已订阅图像话题: {self.image_topic}")
        rospy.loginfo("实例分割与跟踪节点已启动")
    
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式 (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")
            return
        
        # 执行跟踪和实例分割
        try:
            results = self.model.track(
                source=cv_image,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                tracker=self.tracker_type,
                device=self.device,
                classes=self.classes,
                persist=True,  # 保持目标ID连续性
                verbose=False  # 关闭详细输出
            )
        except Exception as e:
            rospy.logerr(f"推理错误: {e}")
            return
        
        # 处理结果
        if results and len(results) > 0:
            result = results[0]
            
            # 绘制结果（包含边界框、分割掩码和跟踪ID）
            annotated_image = result.plot()
            
            # 显示结果
            if self.show_result:
                cv2.imshow("实例分割与跟踪结果", annotated_image)
                cv2.waitKey(1)
            
            # 发布结果图像
            if self.publish_result:
                try:
                    result_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
                    result_msg.header = msg.header  # 保持时间戳和帧ID一致
                    self.result_pub.publish(result_msg)
                except CvBridgeError as e:
                    rospy.logerr(f"结果图像发布错误: {e}")
            
            # 可选：打印检测到的目标信息
            if result.boxes.id is not None:
                track_ids = result.boxes.id.cpu().numpy().astype(int)
                class_names = [result.names[int(cls)] for cls in result.boxes.cls.cpu().numpy()]
                confidences = result.boxes.conf.cpu().numpy()
                
                rospy.loginfo(f"检测到 {len(track_ids)} 个目标:")
                for track_id, class_name, confidence in zip(track_ids, class_names, confidences):
                    rospy.loginfo(f"  ID: {track_id}, 类别: {class_name}, 置信度: {confidence:.2f}")
    
    def run(self):
        rospy.spin()
        if self.show_result:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = YoloeSegmentationTracker()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点已终止")
