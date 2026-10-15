# ROS 2 Jazzy 开发约定

更新：2026-10-15。取代原 Jazzy/Noetic 双版本适配计划。

## 唯一基线

- 开发、测试和部署统一面向 Ubuntu 24.04 / ROS 2 Jazzy。
- 不再维护或规划 ROS 1/Noetic 包装、launch、桥接或跨版本兼容层。
- 使用 `ament_cmake/colcon`、`rclcpp/rclpy`、ROS 2 消息及 Python launch。
- 算法核心与 ROS 接口按可测试性分离，不为尚未实现的算法补写空接口。

## 分层要求

- 算法核心：处理图像、实例观测、位姿与标定数值，尽量不依赖 ROS API，方便离线验证。
- ROS 接口：负责消息转换、QoS、时间及订阅发布，只实现 Jazzy 接口。
- 启动和配置：节点在 `src/` 并列；参数集中根目录 `config/`，RViz 在 `rviz/`。

## 构建与边界

- 继续使用 `scripts/build_jazzy.sh`，构建目录为 `build/jazzy`、`install/jazzy`、`log/jazzy`。
- 保留脚本清理继承环境的保护，避免其他 ROS 发行版路径污染 Jazzy 构建；这不是兼容支持。
- 当前 SAM3 节点、话题和默认后端不因本约定改变；视觉测距仍未实现。
- 本机不训练的限制仍有效；独立模型环境不代表支持其他 ROS 发行版，不自动安装或启动训练。
