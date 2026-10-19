# UAL Planner：纯视觉分支

`visual-only` 分支只保留相机实例分割链路，为后续使用“里程计 + 相机内参 + 目标实例尺寸变化”
估计距离准备接口。当前版本**没有实现视觉测距算法**，也不会订阅或融合雷达点云。
后续开发、测试和部署统一使用 **ROS 2 Jazzy**，不再考虑 ROS 1/Noetic 迁移。

跨电脑或新对话接手时，先读 [开发与交接约定](AGENTS.md)、[当前项目状态](docs/PROJECT_STATE.md)
和 [决策记录](docs/DECISIONS.md)，再核查实际分支及本机环境。
交接入口当前在 `visual-only` 维护；代码同步不等于聊天、模型、数据或虚拟环境同步。
当前模型与数据集统一登记在 [待测试目录](docs/待测试/README.md)，只列资源和状态，不自动下载或训练。

2026-10-13 按用户确认的方案 1，清理此前离线试验的输出、模型权重、MASt3R 第三方源码、
试验脚本、配置、测试及相关报告/调研文档。现有 SAM3/ROS 节点、运行配置与 RViz 保留不变。
数据集、原始 bag 和虚拟环境未删除；已清理内容移入系统回收站，未清空前可恢复。
既有失败结论的简要记录保留在交接文件中，不再提供已删除试验的运行入口。

2026-10-15 按新要求完成 YOLO11n-OBB 与 YOLO-MOD 船舶 s/l 的[离线定位对照](docs/ship_detection_trial.md)。
用户验收结论为 **YOLO11n-OBB 合格、YOLO-MOD s/l 不合格**；不因此自动替换以下 SAM3/ROS 默认链路。
后续测试统一追加到上述报告，不再主动进行人工视觉核查；保留原始统计与可视化供查看。

2026-10-19 CAT Mask R-CNN/iSAID 已完成 48 张离线 GPU 测试（32 份候选实例掩膜，预测约 1.06 s，质量待验收）；
LiM-YOLO 有训练代码与 OBB 头，但缺公开训练权重，尚未推理。
候选现仅推进“有训练实现＋OBB 或像素掩膜”，淘汰仅水平框等不合门槛项，详情见[模型清单](docs/待测试/models.md)。
未改变以下 ROS 默认后端，不启动本机训练。

同日 MariSat SAM3 用户权重已尝试：全部参数严格加载通过，但原始权重有两个 `-inf`，GPU 预热掩膜数值异常，
未完成有效样本评测；[诊断](docs/ship_detection_trial.md#2026-10-19用户下载的-marisat-sam3-权重测试数值异常)保留，待正常权重再测，不擅自修补参数。

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

唯一支持的开发基线为 Ubuntu 24.04 / ROS 2 Jazzy：

```bash
./scripts/build_jazzy.sh --packages-select ins_seg ual_planner_bringup
source install/jazzy/setup.bash
```

脚本会清除终端中继承的其他 ROS 发行版路径，并使用独立的 `build/jazzy`、
`install/jazzy` 和 `log/jazzy` 目录。不要混入其他 ROS 发行版的环境或构建缓存。

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

开发约定见 [`docs/jazzy_development.md`](docs/jazzy_development.md)，纯视觉路线迁移清单见
[`docs/visual_only_migration.md`](docs/visual_only_migration.md)。
