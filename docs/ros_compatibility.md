# Jazzy / Noetic 兼容约定

## 基线与边界

- 主开发、持续构建和新功能验证以 ROS 2 Jazzy 为基线。
- ROS 1 Noetic 只承担部署适配，不在算法实现中引入 ROS 版本分支。
- launch 负责选择节点包装、话题名、配置文件和设备预设，不能代替 ROS 1/ROS 2 API
  适配。`rclcpp` 节点不能仅通过更换 launch 在 Noetic 中运行。

## 代码分层

| 层次 | 允许依赖 | 复用方式 |
| --- | --- | --- |
| 算法核心 | C++ 标准库、NumPy/OpenCV 等算法库 | Jazzy 与 Noetic 直接复用 |
| 消息转换 | `rclcpp` 或 `roscpp/rospy` | 每个 ROS 版本单独实现 |
| 启动与配置 | Jazzy `.launch.py` / Noetic `.launch` | 使用相同算法参数名，不同接口预设 |

当前 `landing_evaluator_core` 及其 `evaluator.hpp/evaluator.cpp` 不包含 ROS 头文件，属于
共享算法核心。`landing_evaluator_node.cpp` 是 Jazzy 包装层，负责 PointCloud2/IMU 消息、
参数、状态机和发布接口。后续 Noetic 支持应新增独立 catkin 包装层并链接同一核心，不能
复制或改写评估算法。

Python 模块在继续开发时，也应把数组输入/输出的计算函数放入不导入 `rclpy`/`rospy` 的
模块；Jazzy 与 Noetic 节点只完成消息和数组之间的转换。

## Launch 与配置约定

- Jazzy：使用 `*.launch.py`，当前入口为
  `ros2 launch ual_planner_bringup d1.launch.py`；Avia/HIL 使用同包对应 launch。
- Noetic：使用独立 catkin 包和 `*.launch`；在包装层完成前不提供不可运行的占位 launch。
- 坡度、粗糙度、RANSAC、栅格、滤波和匹配阈值等算法参数保持同名、同单位。
- ROS 版本相关的 QoS、服务类型、消息包名和时间 API 不进入共享算法参数。
- 仓库根目录 `config/common.yaml` 保存算法公共参数，`config/d1.yaml`、
  `config/avia.yaml`、`config/hil_sim.yaml` 保存设备接口、外参及必要覆盖；不在算法中按
  ROS 版本或设备名称编写条件分支。

Jazzy 构建使用 `scripts/build_jazzy.sh`，固定输出到 `build/jazzy`、`install/jazzy` 和
`log/jazzy`。脚本在加载 `/opt/ros/jazzy/setup.bash` 前移除继承的 ROS/CMake/Python
前缀，从而避免不同发行版的 rosidl 生成器进入同一个 CMake 缓存。

## 当前支持状态

| 模块 | Jazzy | Noetic |
| --- | --- | --- |
| 降落评估算法核心 | 已构建、已测试 | 可复用，待 catkin 包装 |
| Landing ROS 节点 | 已实现 | 待实现消息与节点包装 |
| 实例分割/投影/实例图节点 | 已实现 | 待拆分纯算法模块与 rospy 包装 |
| 总启动文件 | `ual_planner_bringup` 的设备 launch | 待各包装层完成后提供 |

Noetic 适配验收至少应包含：相同合成输入下核心数值结果一致、参数文件逐项对应、三态与
原因位掩码一致，以及 D1 点云/IMU 回放接口测试。
