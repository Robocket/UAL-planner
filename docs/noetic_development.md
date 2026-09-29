# Noetic 开发约定

仓库根目录就是 catkin 工作空间，源码统一位于 `src/`。不要再创建嵌套的工作空间，
也不要把其他 ROS 发行版的环境叠加到同一终端。

核心算法与 ROS 封装的边界如下：

- `landing_evaluator/src/evaluator.cpp` 只依赖 C++ 标准库，是共享算法核心。
- C++ 节点负责 ROS 消息、参数、订阅、发布和服务转换。
- Python 节点通过 `ual_ros1.Node` 统一执行 rospy 的参数和通信操作；数组、OpenCV、
  推理和追踪函数仍保留在原模块中。
- 配置文件中的顶层键必须与节点名称一致，私有层级参数使用 `/`，例如
  `imu/enabled`。

提交前至少运行：

```bash
./scripts/build_noetic.sh
catkin_make run_tests
catkin_test_results
roslaunch --files ual_planner_bringup d1.launch
```
