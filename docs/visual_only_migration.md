# 纯视觉分支迁移清单

## 已完成

- [x] 从 `master` 创建本地 `visual-only` 分支，原多传感器实现仍可从 `master` 找回。
- [x] 保留 SAM3 Q4_0 C++ 节点和 TorchAO Python 回退节点。
- [x] 保留 `SegInfo`、`SegmentationResult` 二维实例分割消息。
- [x] 保留 `CompressedImage` → `Image` 相机适配器。
- [x] 删除 Livox/D1 相关驱动接口、点云转换和雷达设备配置。
- [x] 删除点云投影、三维实例坐标和实例图代码。
- [x] 删除 Landing 评估和原自动降落包。
- [x] 新增只启动实例分割与 RViz 的通用纯视觉 launch。
- [x] Scene_Water launch 改为只启动压缩图像适配、实例分割和可选 RViz。
- [x] RViz 只显示 `/sam3/annotated_image`，不再订阅点云和投影结果。
- [x] 将 Scene_Water 的 `/fsdk/aircraft_state` 和原相机内参迁入后续视觉测距预留配置。
- [x] 为通用相机预留 `/Odometry`、`CameraInfo` 和内参回退配置。

## 本次明确未实现

- [ ] 基于实例掩膜大小变化、相机内参和里程计的距离估计算法。
- [ ] 目标跨帧关联、异常运动剔除、尺度可观性判定和滤波。
- [ ] 视觉测距输出消息、节点和可视化。
- [ ] ROS 1 Noetic 节点包装与 launch。

后续实现前需要明确目标尺寸先验、允许的相机运动、里程计坐标语义、距离定义及评测数据。
