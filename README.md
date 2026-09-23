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
python3 -m pip install --user -r src/Instance_Seg/requirements.txt
```

如果使用 NVIDIA GPU，还需要按显卡和 CUDA 版本安装匹配的 PyTorch；默认启动参数
使用 `device: auto`，检测到 CUDA 时优先使用 GPU。SAM 3 官方运行基线要求较新的
CUDA GPU，CPU 仅适合接口调试。

实例分割使用 Meta SAM 3 官方代码和文本提示 `large ship`，不读取本地训练权重。首次启动
会自动从 Hugging Face 的 `facebook/sam3` 下载并缓存权重。该仓库需要先接受 Meta 的模型
许可并登录一次：

```bash
hf auth login
```

之后正常启动节点即可自动下载；无需配置 `model_path`。

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

Scene_Water 转换包使用 `/left_camera/image`、`/livox/avia/points`、
`/fsdk/aircraft_state` 和 `/livox/imu`。其点云/IMU frame 为 `avia_frame`，里程计父/子
frame 为 `map`/`base_link`；分割可视化显示原始输入点云和投影图像，因此专用 RViz 的
Fixed Frame 使用 `avia_frame`：

```bash
ros2 launch ual_planner_bringup scene_water.launch.py start_rviz:=true
```

## 点云降落面评估

`Landing_Obstacle_Warning` 中的 `landing_evaluator_node` 与其他算法处于同一级，不属于
`ins_seg` 的子节点。D1 配置下，降落评估和点云投影都直接接收 `/iv_points`；投影节点按
同一设备外参完成 sensor → base → world 转换，降落评估则在局部重力对齐坐标中判定，
二者不再使用互相冲突的点云入口。

D1 配置中的尺寸、量程和安全阈值是测试初值，不是通用实机安全结论。更换机体、安装
位置或传感器后，必须重新标定外参并核定阈值。

根目录 D1 设备配置的输入话题为：

- RGB 图像：`/camera/color/image_raw`
- 原始点云：`/iv_points`
- IMU：`/iv_imu`
- 里程计：`/Odometry`

设备驱动本身不由总 launch 启动；总 launch 只消费这些标准接口。
