# UAL-planner（ROS Noetic）

本分支面向 Ubuntu 20.04 / ROS Noetic，是一个可直接使用 `catkin_make` 开发的
ROS 1 工作空间。实例分割、点云投影、实例图、降落面评估、Livox 消息适配和统一启动
均已迁移到 ROS 1；算法计算流程、消息字段及阈值语义保持不变。

## 环境与构建

```bash
sudo apt install \
  ros-noetic-desktop-full ros-noetic-cv-bridge \
  ros-noetic-diagnostic-msgs ros-noetic-nav-msgs \
  ros-noetic-tf2-ros ros-noetic-visualization-msgs \
  python3-numpy python3-opencv libcurl4-openssl-dev

./scripts/build_noetic.sh
source devel/setup.bash
```

也可使用标准 catkin 命令：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

首次编译 `ins_seg` 时会获取固定版本的 `sam3.cpp`。首次运行 Q4 后端时，默认将
模型下载到 `~/.cache/ual_planner/models/sam3-q4_0.ggml`。

## 启动

```bash
# D1
roslaunch ual_planner_bringup d1.launch

# Livox Avia
roslaunch ual_planner_bringup avia.launch

# 半实物仿真
roslaunch ual_planner_bringup hil_sim.launch

# 原 Scene_Water ROS 1 bag 接口
roslaunch ual_planner_bringup scene_water.launch start_rviz:=true
```

统一入口支持按需关闭模块：

```bash
roslaunch ual_planner_bringup ual_planner.launch \
  sensor_config:=$(rospack find ual_planner_bringup)/config/d1.yaml \
  start_segmentation:=false start_rviz:=true
```

默认实例分割后端为 `seg_q4_node`。需要 Python/PyTorch 后端时：

```bash
roslaunch ual_planner_bringup d1.launch \
  segmentation_executable:=seg.py
```

## 包结构

- `ins_seg`：SAM 3 分割、点云投影、实例追踪图和图像适配。
- `landing_evaluator`：C++ 降落面核心、ROS 节点、状态可视化和仿真姿态适配。
- `auto_landing`：候选降落区域评估和降落点追踪。
- `livox_avia_driver`：Livox `CustomMsg` 定义及 `PointCloud2` 转换。
- `ual_planner_bringup`：统一配置、设备预设、launch 和 RViz 配置。
- `ual_ros1`：Python 节点共用的轻量 rospy 接口层。

## 参数约定

配置文件按节点名称分组，节点读取私有参数。例如：

```yaml
landing_evaluator:
  input_topic: /iv_points
  extrinsics/rotation_rpy: [0.0, 1.5707963267948966, 0.0]
  imu/enabled: true
  watchdog/cloud_timeout_sec: 0.60
```

角度使用弧度，距离使用米，时间使用秒。设备外参只是预设，实机使用前必须以实际标定值
替换。`LandingStatus.UNKNOWN` 必须由飞控按不安全状态处理。

## 测试

```bash
catkin_make run_tests
catkin_test_results
```

仅测试 Landing 算法核心：

```bash
catkin_make run_tests_landing_evaluator
```
