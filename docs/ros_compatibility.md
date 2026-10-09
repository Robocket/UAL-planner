# ROS 开发边界

当前分支处于学习模型选型阶段，尚未实现 ROS 节点。

- 算法层接收图像数组，返回目标框、实例掩膜和置信度，独立于 ROS。
- 训练、评测与模型导出使用独立 Python 环境，不依赖 Jazzy 或 Noetic 的消息 API。
- Jazzy 包装负责消息转换、时间戳、QoS、参数和发布。
- Noetic 使用独立 `roscpp/rospy` 包装与 `.launch`，复用同一推理核心。
- 图像、里程计和相机标定接口通过根目录 `config` 配置。

不同 launch 负责选择包装和设备配置；不能让 ROS 2 节点自动兼容 ROS 1。
后续 ROS 包构建可使用 `scripts/build_jazzy.sh`；当前没有可构建的 ROS 包。
