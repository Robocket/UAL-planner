# UAL Planner

主开发环境为 Ubuntu 24.04 / ROS 2 Jazzy。统一启动层将实例分割、点云投影、
实例图维护和 C++ 降落面评估作为四个并列算法节点。部署层计划同时适配 ROS 1 Noetic；
算法实现保持中间件无关，Jazzy 与 Noetic 分别使用自己的节点包装和 launch。兼容边界见
[`docs/ros_compatibility.md`](docs/ros_compatibility.md)。

## 安装依赖

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

默认实例分割后端为 C++ `sam3.cpp` 和完整文本检测版 `sam3-q4_0.ggml`，继续使用
`large ship` 文本提示，不读取项目原有的本地训练权重。首次启动会从公开仓库自动下载
706,606,590 字节的 Q4_0 文件到
`~/.cache/ual_planner/models/sam3-q4_0.ggml`，并同时校验文件大小和 SHA-256；不需要
Hugging Face 登录。模型地址、缓存路径、摘要、CPU 线程数、NMS 和分割阈值均在
`config/common.yaml` 配置。

Q4_0 当前在 Linux 上使用 CPU 后端。i7-11800H 的 Scene_Water 实测中，8 线程单独运行约
26.6–29.6 秒/帧，完整投影链路持续运行时可接近 40 秒/帧，并能发布 4 个船只实例；
4 线程约 54 秒/帧，16 线程约 32–41 秒/帧，因此默认使用 8 线程。它满足最小完整文本
模型的要求，但不适合实时处理。
点云投影缓存同步扩展到约 75–80 秒，确保延迟分割仍能匹配原始图像时刻的点云和里程计。
分割结果在 1280×1024 下约 9 MiB，分割到投影的订阅固定使用
Reliable QoS，避免大消息分片在 Best Effort 模式下被静默丢弃。

原 Transformers/TorchAO GPU 节点保留为回退，启动时添加
`segmentation_executable:=seg.py` 即可使用。该回退才需要创建 `.venv-sam3`、安装
`src/Instance_Seg/requirements.txt` 及匹配 CUDA 的 PyTorch；T600 上 A8W8 实测约
3.5 秒/帧。

### 运行时算力监测

节点在 `/sam3/performance` 发布标准 `diagnostic_msgs/DiagnosticArray`。Q4_0 提供图像编码、
文本分割、发布、总耗时和 EMA 帧率；TorchAO 回退另外提供 CUDA 显存。查看单次诊断：

```bash
ros2 topic echo /sam3/performance --once
ros2 topic hz /sam3/segmentation --qos-reliability reliable
```

Q4_0 默认使用 CPU，另开终端查看线程利用率和内存：

```bash
top -H -p "$(pgrep -n seg_q4_node)"
```

使用 TorchAO GPU 回退时查看 GPU 核心、显存、温度和频率：

```bash
nvidia-smi dmon -s pucm -d 1
```

## 构建

推荐使用仓库提供的隔离构建入口。它不会继承当前终端中可能残留的 Humble/Noetic
路径，并将 Jazzy 产物放入独立目录：

```bash
./scripts/build_jazzy.sh
source install/jazzy/setup.bash
```

可以把标准 `colcon build` 参数继续传给脚本，例如：

```bash
./scripts/build_jazzy.sh --packages-select landing_evaluator ins_seg
```

不要在同一个构建目录或终端中混用 Noetic、Humble 与 Jazzy 的环境变量。当前目录中若有
旧的默认 `build/包名`、`install/包名`，脚本不会读取或覆盖它们；Noetic 适配层也应使用
独立 catkin 工作空间。

## 启动

默认按仓库根目录的 `config/common.yaml` 与 `config/d1.yaml` 叠加配置，启动全部
并列算法节点：

```bash
ros2 launch ual_planner_bringup d1.launch.py
```

切换雷达/相机组合时使用对应 launch，算法节点不需要修改：

```bash
ros2 launch ual_planner_bringup avia.launch.py
ros2 launch ual_planner_bringup hil_sim.launch.py
```

只调试部分链路时，可通过统一入口关闭节点：

```bash
ros2 launch ual_planner_bringup d1.launch.py \
  start_landing_evaluator:=false start_instance_graph:=false
```

SAM 3 文本提示、推理设备以及所有算法阈值均在 `config/common.yaml` 中配置；设备话题、坐标系、
雷达/相机标定和少量设备相关阈值在 `config/d1.yaml`、`config/avia.yaml` 或
`config/hil_sim.yaml` 中覆盖。也可直接调用通用入口加载自定义设备配置：

```bash
ros2 launch ual_planner_bringup ual_planner.launch.py \
  sensor_config:=/absolute/path/to/device.yaml
```

启动 RViz：

```bash
ros2 launch ual_planner_bringup rviz.launch.py
```

也可以与算法节点一起启动：

```bash
ros2 launch ual_planner_bringup d1.launch.py start_rviz:=true
```

Scene_Water 直接使用原始 bag。根据该包的 `metadata.yaml`，输入接口为：

- `/left_camera/image/compressed`：`sensor_msgs/msg/CompressedImage`
- `/livox/lidar`：`livox_avia_driver/msg/CustomMsg`
- `/livox/imu`：`sensor_msgs/msg/Imu`
- `/fsdk/aircraft_state`：`nav_msgs/msg/Odometry`

`scene_water.launch.py` 默认启动接口适配节点，将前两项分别转换为算法使用的
`/left_camera/image`（`sensor_msgs/msg/Image`）和 `/livox/lidar/points`
（`sensor_msgs/msg/PointCloud2`）。点云投影与 Landing 均订阅同一个
`/livox/lidar/points`，无需预先生成转换 bag。点云/IMU frame 为 `avia_frame`，里程计
父/子 frame 为 `map`/`base_link`；专用 RViz 的 Fixed Frame 使用 `avia_frame`，并显示
`/livox/lidar/points`、`/sam3/annotated_image` 和 `/projected_image`。

仓库中的并列包 `src/Livox_Avia_Driver` 提供与原始 bag 一致的
`livox_avia_driver/msg/CustomMsg` 接口和转换节点，因此无需 source 旧的 Humble/外部
驱动工作空间。该兼容包只负责 bag 接口和点云转换，不包含 Avia 硬件通信驱动。

```bash
source /opt/ros/jazzy/setup.bash
source /home/htfp/UAL-planner/.venv-sam3/bin/activate
source /home/htfp/UAL-planner/install/jazzy/setup.bash
```

Scene_Water 的海康工业相机镜头与 Avia 雷达镜头同向，当前在
`config/scene_water.yaml` 中将直接 LiDAR→Camera 外参设为单位旋转和零平移。相机输出尺寸
按 bag 中实测的 `1280×1024` 设置，`fx=fy=385.45` 暂沿用原值，主点暂取图像中心；获得
正式标定结果后应整体替换相机内外参。

两个已 source 当前工作空间的终端分别运行：

```bash
# 终端 1：先启动算法和适配节点
ros2 launch ual_planner_bringup scene_water.launch.py start_rviz:=true

# 终端 2：节点就绪后回放原始包
ros2 bag play /home/htfp/data/Scene_Water_20260805_013651
```

## 点云降落面评估

`Landing_Obstacle_Warning` 中的 `landing_evaluator_node` 与其他算法处于同一级，不属于
`ins_seg` 的子节点。D1 配置下，降落评估和点云投影都直接接收 `/iv_points`；投影节点按
同一设备外参完成 sensor → base → world 转换，降落评估则在局部重力对齐坐标中判定，
二者不再使用互相冲突的点云入口。

投影与实例图链路统一使用世界坐标计算：FAST-LIO 等已经位于世界系的点云配置为
`pointcloud_frame: world`；Scene_Water 等原始雷达点云配置为 `sensor`，节点先结合外参和
里程计转换到世界系。`/projected_cloud` 和 `/projected_instance_info` 的 `frame_id` 均为
对应里程计的父坐标系。Landing 独立使用机体局部重力对齐坐标，不受此约定影响。

D1 配置中的尺寸、量程和安全阈值是测试初值，不是通用实机安全结论。更换机体、安装
位置或传感器后，必须重新标定外参并核定阈值。

根目录 D1 设备配置的输入话题为：

- RGB 图像：`/camera/color/image_raw`
- 原始点云：`/iv_points`
- IMU：`/iv_imu`
- 里程计：`/Odometry`

设备驱动本身不由总 launch 启动；总 launch 只消费这些标准接口。
