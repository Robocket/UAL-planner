# UAL Planner：纯视觉分支

`visual-only` 分支只保留相机实例分割链路，为后续使用“里程计 + 相机内参 + 目标实例尺寸变化”
估计距离准备接口。当前版本**没有实现视觉测距算法**，也不会订阅或融合雷达点云。

## 当前范围

```text
CompressedImage（可选） -> Image -> SAM3 实例分割 -> 分割消息/标注图
                                      |
                                      +-> 后续视觉测距（尚未实现）
                                          预留输入：Odometry、相机内参
```

工作空间现有两个 ROS 2 包：

- `ins_seg`：SAM3 Q4_0 C++ 节点、TorchAO Python 回退节点、压缩图像适配器及二维分割消息。
- `ual_planner_bringup`：纯视觉配置、通用相机/Scene_Water launch 和 RViz 配置。

雷达驱动、点云投影、三维实例图、降落评估和原自动降落包均未迁入本分支；完整历史仍保留在
`master` 分支。

## 构建

主开发环境为 Ubuntu 24.04 / ROS 2 Jazzy：

```bash
./scripts/build_jazzy.sh --packages-select ins_seg ual_planner_bringup
source install/jazzy/setup.bash
```

脚本会清除终端中继承的其他 ROS 发行版路径，并使用独立的 `build/jazzy`、
`install/jazzy` 和 `log/jazzy` 目录。不要在同一个构建目录中混用 Humble、Jazzy 和 Noetic。

## 启动

通用原始图像配置默认订阅 `/camera/color/image_raw`：

```bash
ros2 launch ual_planner_bringup camera.launch.py start_rviz:=true
```

也可直接使用统一入口并提供自己的相机配置：

```bash
ros2 launch ual_planner_bringup visual_only.launch.py \
  camera_config:=/absolute/path/to/camera.yaml \
  start_rviz:=true
```

Scene_Water 原始包中的海康相机发布压缩图像，专用 launch 会启动适配器：

```bash
ros2 launch ual_planner_bringup scene_water.launch.py start_rviz:=true
ros2 bag play /home/htfp/data/Scene_Water_20260805_013651
```

该链路只使用：

- `/left_camera/image/compressed` → `/left_camera/image`
- `/left_camera/image` → `/sam3/segmentation`
- `/left_camera/image` → `/sam3/annotated_image`

`/fsdk/aircraft_state` 及相机内参已写入 `config/scene_water.yaml` 的
`visual_distance_estimator` 预留配置，但当前没有节点消费这些参数。

## 实例分割模型

默认后端是 `seg_q4_node`，使用公开的完整文本提示版 `sam3-q4_0.ggml`。首次运行会自动下载到
`~/.cache/ual_planner/models/sam3-q4_0.ggml` 并校验大小与 SHA-256，无需 Hugging Face 登录。
模型地址、缓存路径、CPU 线程数、阈值、提示词和所有输出话题统一配置在
`config/common.yaml`。

Q4_0 为 CPU 后端，当前设备实测约几十秒一帧，适合验证但不满足实时性。已安装匹配 CUDA 的
PyTorch 时，可显式切换现有 TorchAO 回退后端：

```bash
ros2 launch ual_planner_bringup camera.launch.py \
  segmentation_executable:=seg.py
```

运行时可监测：

```bash
ros2 topic echo /sam3/performance --once
ros2 topic hz /sam3/segmentation --qos-reliability reliable
top -H -p "$(pgrep -n seg_q4_node)"
```

## 后续视觉测距接口

设备配置中只预留以下信息，没有自行补写算法或输出定义：

- `segmentation_topic`：带时间戳的实例掩膜与实例编号；
- `odometry_topic`：相机/载体运动信息；
- `camera_info_topic`：优先使用标准 `sensor_msgs/CameraInfo` 的接口名；
- `image_width`、`image_height`、`fx`、`fy`、`cx`、`cy`：无 CameraInfo 时的标定回退值。

Scene_Water 暂沿用原来的海康相机内参；正式实现测距前应重新标定。算法、状态缓存、尺度约束、
距离输出消息和误差处理均留待后续明确后实现。

Jazzy/Noetic 的代码边界见 [`docs/ros_compatibility.md`](docs/ros_compatibility.md)，本次迁移清单见
[`docs/visual_only_migration.md`](docs/visual_only_migration.md)。
