# Jazzy / Noetic 兼容约定

## 基线

- 新功能在 ROS 2 Jazzy 上开发和验证。
- ROS 1 Noetic 作为后续部署目标，使用独立包装和 launch。
- 不能只靠更换 launch 让 `rclcpp`/`rclpy` 节点直接在 Noetic 运行。

## 分层要求

后续视觉测距实现应拆成三层：

| 层次 | 内容 | Jazzy / Noetic 复用方式 |
| --- | --- | --- |
| 算法核心 | 图像数组、实例观测、位姿和标定的数值计算 | 完全复用，不导入 ROS API |
| ROS 包装 | 消息转换、订阅发布、时间和 QoS | `rclcpp/rclpy` 与 `roscpp/rospy` 分开 |
| 启动配置 | 话题映射、设备参数、是否启用适配器 | Jazzy `.launch.py` 与 Noetic `.launch` 分开 |

相机、分割、里程计和标定参数在不同包装中应保持相同名称与单位。ROS 消息类型、QoS、时间
API 和发行版判断不得进入算法核心。

## 当前状态

本分支当前只有 Jazzy 包装；实例分割算法仍封装在现有 ROS 2 节点内。尚未实现的视觉测距核心
必须按上述边界新建，避免后续复制两套算法。Noetic 包装完成前不提供不可运行的占位 launch。
