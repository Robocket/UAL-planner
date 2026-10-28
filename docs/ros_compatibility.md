# ROS 2 Jazzy 与旧工作空间的边界

2026-10-26 master 架构更新后，后续开发、测试和部署只面向 **ROS 2 Jazzy**。
旧 Noetic 兼容方案是历史计划，已经取消；这个文件保留原路径，便于旧文档链接继续使用。

## 代码与资源

- 算法节点使用 `rclcpp/rclpy`、`ament_cmake` 和 ROS 2 消息；纯 NumPy/C++ 核心可以离线测试。
- `src/Pointcloud_Voxels` 的数值算法来自 ws_ual，但节点包装已改为 ROS 2，不导入 `rospy`。
- 启动、设备配置、RViz 分别位于根目录 `launch/`、`config/`、`rviz/`。
- 根目录资源由算法包 `pointcloud_voxels` 安装；不再使用独立 Bringup 包。
- 新入口：`ros2 launch pointcloud_voxels scene_water.launch.py`。
  历史 D1 全链路入口：`ros2 launch pointcloud_voxels d1.launch.py`。

## 构建与数据

使用 `scripts/build_jazzy.sh`，输出到 `build/jazzy`、`install/jazzy`、`log/jazzy`。
脚本清除继承的其他 ROS 发行版路径。旧 ws_ual 的 catkin 编译产物不参与本仓库构建。

标准输入为 ROS 2 Image、PointCloud2、CameraInfo、`ins_seg/DetectionResult` 及兼容的
`ins_seg/SegmentationResult`。检测节点依赖 Ultralytics/PyTorch，但不进入 ROS 构建，
由 launch 的 `detector_python` 指定独立解释器；ROS 节点本身仍只使用标准消息。
原始 Scene_Water 的 Livox CustomMsg 和压缩图像由已有 ROS 2 适配器转换。
ROS 1 `.bag` 不能直接作为 `ros2 bag play` 输入；本次没有新增桥接、Noetic 包装或全数据转换。

## 验证边界

2026-10-28 已在本机完成五包 Jazzy 构建、86项CTest、安装入口、数据回放、近似几何的
真实单帧检测/掩码/点云 3D bbox/局部路径接口验证。
图像/点云采集 header 同步通过；正式内外参及识别/几何精度尚未验收，见 [当前状态](PROJECT_STATE.md)。
