# UAL Planner (ROS 2 Humble)

该仓库已迁移到 Ubuntu 22.04 对应的 ROS 2 Humble。`ins_seg` 包包含实例分割、
点云投影和实例图维护三个 Python 节点。

## 安装依赖

```bash
source /opt/ros/humble/setup.bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --user -r src/planner/Instance_Seg/requirements.txt
```

如果使用 NVIDIA GPU，还需要按显卡和 CUDA 版本安装匹配的 PyTorch；默认启动参数
使用 CPU，不要求 CUDA。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果工作区中残留旧版 ROS 1 的 `build`、`devel` 目录，应先备份需要的内容，再清理
这些生成目录后重新构建。不要在同一个终端同时 source ROS 1 和 ROS 2 环境。

## 启动

默认模型使用 Ultralytics 可自动获取的 `yolo11n-seg.pt`：

```bash
ros2 launch ins_seg seg.launch.py
```

指定本地模型或 GPU：

```bash
ros2 launch ins_seg seg.launch.py \
  model_path:=/absolute/path/to/model.pt device:=cuda:0
```

启动 RViz：

```bash
ros2 launch ins_seg rviz.launch.py
```

## 点云降落区域评估

`pointcloud_projection` 会将相机视野内、保持原始世界坐标的点云发布到
`/projected_cloud`。降落评估节点要求该坐标系已经与重力对齐（Z 轴向上）：

```bash
ros2 launch auto_landing evaluate.launch.py
```

主要输出：

- `/landing/regions`：连通降落区域；包含中心、可信度、面积、等效半径、平均坡度、
  平均粗糙度和支撑点数。
- `/landing/confidence_cloud`：带 `confidence` 字段的候选中心点云。
- `/landing/markers`：RViz 圆形落点和可信度文字，绿色可信度高、红色可信度低。

可信度范围为 0–1，由坡度、粗糙度、点密度、单格高度跨度和无人机圆形占地范围内
的安全净空共同计算。默认占地安全半径为 `drone_radius + safety_margin`，即
`0.5 + 0.15 m`。可通过 ROS 2 参数覆盖，例如：

```bash
ros2 run auto_landing evaluate.py --ros-args \
  -p drone_radius:=0.7 \
  -p safety_margin:=0.2 \
  -p max_slope_deg:=8.0 \
  -p max_roughness:=0.03
```

默认输入话题为：

- RGB 图像：`/camera/color/image_raw`
- 注册点云：`/cloud_registered`
- 里程计：`/Odometry`

可在 `seg.launch.py` 中调整，也可以单独运行节点并通过 ROS 2 参数覆盖。
