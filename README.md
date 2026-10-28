# UAL Planner

当前 `master` 集成了 `ws_ual` 的三维占用栅格算法，默认使用 `visual-only`
已验收的 **YOLO11n-OBB** 目标检测框选择船体点云；同时保留实例分割掩码模式。
开发、构建和部署面向 **ROS 2 Jazzy**；原 Noetic 工作空间仅是算法来源，不作为本仓库的运行依赖。

```text
相机图像 -> ship_detector (YOLO11n-OBB) -> 原图检测多边形
                                             |
同时间戳 LiDAR 点云 -> 相机投影 -> 多边形内点云 -> 三维栅格 + bbox 中心 -> 3D A* -> RViz

兼容路径：相机图像 -> SAM3 实例分割 -> 32SC1 实例掩码 -> 精确像素查询 -> 三维栅格
```

检测模型只输出框/旋转框，**不会用外接框伪造实例掩码**。两条路径共用同一套投影、
时间同步、三维栅格和 RViz 输出：

- `selection_mode:=detection`（默认）：按 OBB/水平框多边形选点，框内背景深度由点云本身决定。
- `selection_mode:=mask`：按真实实例像素选点，掩码孔洞和背景仍被排除。
- `selection_mode:=cluster`：保留原 ws_ual 的纯雷达 PCA 聚类框选点，仅作历史对照。

图像检测框只作为选点 ROI；真正维护、发布并用于规划目标的是**点云中的三维 bbox**。
点云 bbox 按参考 `pointcloud_cluster_bbox` 逻辑生成：把选中的 3D 点投影到
`yz`（可配）平面 → 体素化并做欧氏连通聚类 → 对主簇做 PCA 有向矩形 →
沿法向取点云范围，得到 8 角点的 3D OBB 和长度/宽度/高度。投影面、聚类阈值、
最小框尺寸/法向高度和最小点数都可配置。

3D bbox 在点云坐标系中持续维护：同一类别、同一 3D 区域的新 bbox 不会替换旧 bbox，
除非其检测置信度严格更高。匹配使用 3D AABB IoU 或中心距离；当前帧没有新 bbox 时旧
bbox 继续发布，连续 `bbox_timeout_sec` 未匹配才过期。稳定 `track_id` 随
`/ship/bboxes` 和 `/ship/bbox_target` 输出，因此路径目标是点云 bbox 中心，
不再使用图像 bbox 中心。参数在
[config/pointcloud_voxels.yaml](config/pointcloud_voxels.yaml) 的 `mask_voxels` 段：
`bbox_projection_plane`、`bbox_cluster_tolerance`、`bbox_min_cluster_points`、
`bbox_confidence_margin`、`bbox_match_iou`、`bbox_timeout_sec` 等。

本机没有用户提到的 `~/ws_crl` 目录；使用的是可用的同类实现
`/home/crl/ws_ual/src/pointcloud_cluster_bbox/scripts/pointcloud_cluster_bbox.py`
（投影聚类 + PCA 有向矩形 + 法向范围）。

检测框和掩码都按原图采集时间匹配点云，不拿最新帧凑配。默认处理有有效三维点的最大
图像掩码/检测多边形，也可按 ID、类别筛选或分别处理全部实例。

维护的点云 3D bbox 中心作为路径目标；`ship_path_planner` 在 LiDAR 坐标系内规划一条
避开其他点云的局部路径，并发布 `/ship/path`、`/ship/target` 和 RViz Marker 数组。

## 目录与接口

算法包在 `src/` 并列；启动、参数和显示文件分别直接位于根目录 `launch/`、`config/`、
`rviz/`。已移除独立的 Bringup 包，这些根目录资源由 `pointcloud_voxels` 包安装。

本机数据统一放在仓库根目录 `data/`，`.gitignore` 已忽略整个目录，
包括原始 bag、转换包、图像及报告，不纳入 Git 提交或上传。

- `src/Instance_Seg`（`ins_seg`）：SAM3 掩码节点、YOLO OBB/检测节点、消息与压缩图像适配器。
- `src/Pointcloud_Voxels`（`pointcloud_voxels`）：掩码/检测多边形选点、点云 3D bbox、三维体素、3D A* 局部路径及 ROS 2 消息包装。
- `src/Livox_Avia_Driver`：原有 Livox CustomMsg 接口和 PointCloud2 转换。
- `src/Landing_Obstacle_Warning`、`src/Auto_Landing`：保留的历史算法，不在新检测/掩码栅格入口中启动。

| 输入/输出 | 话题与含义 |
| --- | --- |
| 检测输入 | `/ship/detections`，`ins_seg/msg/DetectionResult`，原图坐标四角多边形 |
| 检测标注图 | `/ship/detections_image`，`sensor_msgs/msg/Image`，仅显示 |
| 掩码输入 | `/sam3/segmentation`，`ins_seg/msg/SegmentationResult`，兼容模式 |
| 点云输入 | `/livox/lidar/points`，`sensor_msgs/msg/PointCloud2` |
| 标定输入 | `/left_camera/camera_info`，可选，必须与检测/掩码图像几何一致 |
| 船体点云 | `/ship/points`，保留输入点云的坐标系及采集时间 |
| 三维体素 | `/ship/voxels`，`visualization_msgs/msg/MarkerArray` |
| 点云 3D bbox | `/ship/bboxes`，`visualization_msgs/msg/MarkerArray`，8 角点 OBB + 标签 |
| bbox 目标 | `/ship/bbox_target`，`geometry_msgs/msg/PointStamped`，维护的点云 bbox 中心 |
| 路径目标 | `/ship/target`，`geometry_msgs/msg/PointStamped`，路径实际使用的 bbox 目标 |
| 规划路径 | `/ship/path`，`nav_msgs/msg/Path`，LiDAR frame 内的局部路径 |
| 路径显示 | `/ship/path_markers`，起点/目标/选中点 AABB/路径线和文字 |
| 规划状态 | `/ship/path_status`，`std_msgs/msg/String` |

体素边长默认 `0.5 m`，ROI 沿用雷达坐标下 `x=0.5..80`、`y=-10..10`、`z=-25..25 m`。
每个占用栅格检查六个面的相邻位置：**空面恰好 4 个为浅绿边缘，≥5 个为红色角点，其余为半透明蓝色**。
这是按用户规则实现的占用特征分类；稀疏回波、遮挡和斜面也会形成空面，颜色不等同于真实船体语义真值。
所有邻面统计在完整占用集合上完成，RViz 数量上限不会制造额外空面。多个实例分别统计，不跨船混合。

## 构建

在具有 Ubuntu 24.04 / ROS 2 Jazzy 的机器上：

Ubuntu 24.04 尚未安装 ROS 2 时，可在本机终端执行
`sudo bash scripts/install_jazzy.sh`，安装 Jazzy 基础环境、RViz 和本仓库的系统开发依赖。
脚本使用官方 ROS apt-source，不修改 `.bashrc` 或下载模型；安装需要 sudo 密码。

```bash
cd ~/UAL-planner
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
./scripts/build_jazzy.sh
source install/jazzy/setup.bash
```

隔离构建使用 `build/jazzy`、`install/jazzy`、`log/jazzy`。
默认检测后端是 YOLO11n-OBB（DOTA `ship`）。代码不提交权重；显式下载并校验：

```bash
python3 scripts/fetch_yolo_detector_weights.py
# 默认写入 ~/.cache/ual_planner/models/yolo11n-obb.pt
```

检测节点需要 Ultralytics，而 ROS 2 Jazzy 的系统 Python 默认没有它。可用独立虚拟环境
（与系统 OpenCV/NumPy 隔离）安装，并把解释器传给 launch：

```bash
python3 -m venv --system-site-packages .venv-yolo
.venv-yolo/bin/python -m pip install \
  --index-url https://download.pytorch.org/whl/cpu torch torchvision
.venv-yolo/bin/python -m pip install 'numpy<2' 'opencv-python<5' ultralytics
# CUDA 机器可改用对应 PyTorch CUDA wheel，并在 config/detector.yaml 设 device: "0"
```

注意：OpenCV 5 改变了 `CV_8UC3` 等常量，与 Jazzy 的 `cv_bridge` 不兼容，
会出现 `KeyError: 16`；必须使用 OpenCV 4（或直接复用系统 OpenCV）。
`numpy<2` 同样是 Jazzy 的 `cv_bridge` 要求。
本机 Ubuntu 未安装 `python3.12-venv` 时，`venv` 会因缺少 ensurepip 失败；
可先用 `python3 -m venv --without-pip --system-site-packages .venv-yolo` 建环境，
再执行其中的 `python -m pip`（复用系统 pip）。

本机已这样建立 `.venv-yolo`，启动时加
`detector_python:="$PWD/.venv-yolo/bin/python"`；该目录和模型均被 Git 忽略。
继续使用 SAM3 时，`seg_q4_node` 仍可能在首次启动下载约 707 MB 的 GGML 权重；
Python/TorchAO 回退使用原有 `src/Instance_Seg/requirements.txt`。

## 标定与时间匹配

先编辑 [config/scene_water.yaml](config/scene_water.yaml) 中的 `mask_voxels`：

1. 填入真实 `lidar_frame`、`camera_optical_frame` 与 **LiDAR→相机光学系** 的 `lidar_to_camera`。
   光学系为 X 右、Y 下、Z 前；镜头同向不能证明两个坐标系旋转为单位矩阵。
2. 提供与原始图像分辨率一致的 CameraInfo，或填写 `image_width/height`、`camera_matrix`、
   `distortion_model`、`distortion`。原始畸变图与校正图需要各自对应的标定，不能混用。
3. 正式标定核验后将 `calibration_confirmed` 设为 `true`。默认历史配置保持 `false`，
   节点会明确提示并暂停相机选点（检测框和实例掩码均需要几何）；不能只改开关视为完成标定。

按用户 2026-10-28 要求，另提供显式近似配置
[scene_water_approximate.yaml](config/scene_water_approximate.yaml)：
以海康 MV-CA013-20GC 的 `1280×1024`、`4.8 μm` 像元为参考，假设 6 mm 镜头，
推导 `fx=fy=1250`、`cx=640`、`cy=512`，暂忽略畸变。
这不是厂家发布的通用标定 K，也未确认本包实际相机/镜头型号。
[厂家规格书](https://www.rmaelectronics.com/content/HikRobot/CameraDatasheets/GiGeAreaScan/511095_Data%20Sheet.pdf)
支持传感器规格，焦距与主点等仍是联调假设。
外参按原点重合、共同俯视和零额外相对偏航处理，光学轴转换为
`x_camera=-y_lidar, y_camera=-z_lidar, z_camera=x_lidar`，不能直接用单位旋转。
该配置仅启用 `allow_approximate_calibration`，`calibration_confirmed` 仍为 `false`。

消息头不被重写；相机图像 frame_id 为空时依靠显式标定目标系解释，不伪造 TF。
若有 CameraInfo，其 frame、图像尺寸、ROI/binning 必须符合当前标定，不会自动缩放检测/掩码图像。
当前支持 `plumb_bob` 和 `rational_polynomial`；鱼眼模型需另行适配。
新链路直接在原始雷达坐标处理，不依赖里程计、不自行输出世界坐标。

检测/分割结果携带原始图像采集时间。节点保留最多 800 帧点云，为延迟结果匹配
`sync_slop=0.2 s` 内的点云；找不到就跳过，不拿最新点云凑配。Scene_Water 图像与 bag
时间轴历史上存在偏差，应核对各传感器 **header** 的共同时间基准；缓存不能修复时钟偏移。
RViz 默认保留最近一次匹配结果直到下次更新，这是一帧历史观测，不是当前时刻跟踪或融合地图。

## 启动

### 本机已转换数据：直接看图像与点云

2026-10-28 已将可读的 Noetic 恢复版转换为 ROS 2 SQLite3：
`data/Scene_Water_20260805_013651_ros2_jazzy`（相对于仓库根目录）。
时长约 353.63 秒，含 2797 帧 `1280×1024` 原始 BGR 图像、原压缩图像、
3777 帧标准点云，以及全部原 IMU/里程计消息；输出约 13 GiB。
原始 `.bag` 文件全部保留。两份 5.2 GB 备份的索引仍损坏，本次转换来源是 2.8 GB 恢复版。

无需编译算法或启动检测/分割模型，直接运行：

```bash
cd /home/crl/UAL-planner
./scripts/play_scene_water.sh
```

脚本会自动加载 Jazzy；实现位于仓库 `scripts/play_scene_water.sh`，也可以从任意目录直接执行。
例如 `./scripts/play_scene_water.sh loop:=false` 单次播放、`./scripts/play_scene_water.sh rate:=0.5` 半速播放。
默认循环播放，并打开 RViz 的 `Camera image` 和 `LiDAR` 显示。
使用 `loop:=false` 播放一次，或 `start_offset:=45` 跳过开头 45 秒。
只回放、不打开 RViz 时添加 `start_rviz:=false`。
该入口不会下载模型、要求相机标定或启动检测/分割算法。

也可单独回放：

```bash
ros2 bag play /home/crl/UAL-planner/data/Scene_Water_20260805_013651_ros2_jazzy --clock
```

新包已经包含 `/left_camera/image` 和 `/livox/lidar/points`。
后续启动算法的 Scene_Water 入口时，必须添加
`start_image_adapter:=false start_livox_adapter:=false`，避免重复发布原图或误把 PointCloud2 当 CustomMsg。
转换脚本为 `scripts/convert_scene_water_bag.py`，本机使用独立环境
`/home/crl/.cache/ual-bag-convert-venv`（rosbags 0.11.9，复用系统 OpenCV）；不依赖 ROS 1。
源时间戳和 frame 均未改写，原图的空 frame_id 和原始时间基准差异仍保留。
逐话题计数与像素校验记录见输出目录内的 `conversion_report.json`。

### 算法链路

默认启动检测节点 `ship_detector` 和选点节点 `mask_voxels`。Scene_Water 原始 bag
若只有压缩图像和 Livox CustomMsg，保留适配器（默认）：

```bash
source install/jazzy/setup.bash
ros2 launch pointcloud_voxels scene_water.launch.py \
  detector_python:="$PWD/.venv-yolo/bin/python"
# 另一终端（路径须指向实际 ROS 2 bag，不是旧 Noetic .bag）：
ros2 bag play /absolute/path/to/ros2_bag
```

对本机已转换包（已含 `/left_camera/image` 和 `/livox/lidar/points`）用近似几何联调：

```bash
source install/jazzy/setup.bash
ros2 launch pointcloud_voxels scene_water.launch.py \
  start_image_adapter:=false start_livox_adapter:=false \
  geometry_config:="$PWD/config/scene_water_approximate.yaml" \
  detector_python:="$PWD/.venv-yolo/bin/python"
# 另一终端只播放数据，复用上面的算法 RViz：
./scripts/play_scene_water.sh start_rviz:=false
```

检测框只在原图像素坐标中定义；投影、时间匹配和三维栅格与原掩码路径完全相同。
`config/detector.yaml` 控制权重、类别、阈值、输入尺寸、设备与标注图；
`config/pointcloud_voxels.yaml` 控制选最大/全部目标、ID/类别筛选和体素。
RViz 的 `Ship detections` 显示 `/ship/detections_image`。

切回实例分割模型（兼容路径）：

```bash
ros2 launch pointcloud_voxels scene_water.launch.py \
  selection_mode:=mask start_segmentation:=true start_detection:=false \
  start_image_adapter:=false start_livox_adapter:=false \
  geometry_config:="$PWD/config/scene_water_approximate.yaml"
```

外部节点已生成 `DetectionResult` 时添加 `start_detection:=false` 并发布到
`/ship/detections`；外部节点已生成相同掩码消息时添加 `start_segmentation:=false`。
使用 TorchAO 掩码回退则添加 `segmentation_executable:=seg.py`。

Scene_Water 的图像/点云发布端为 Reliable；该配置的订阅也使用 Reliable，
避免大型消息分片及单帧测试丢失。其他设备默认保留 Best Effort，可通过
`ship_detector.image_reliability`、`sam3_segmentation.image_reliability`、
`mask_voxels.pointcloud_reliability` 显式设置。

已有原始 Image 和标准 PointCloud2 话题时，用通用入口并选择自己的设备配置：

```bash
ros2 launch pointcloud_voxels mask_voxels.launch.py \
  sensor_config:=/absolute/path/to/device.yaml
```

单独对比原 ws_ual 的纯雷达聚类选点，不需要相机标定或检测模型：

```bash
ros2 launch pointcloud_voxels cluster_bbox.launch.py
```

这个入口消费已发布的标准 PointCloud2，不启动适配器、检测或分割。对原始 Scene_Water bag 可运行：

```bash
ros2 launch pointcloud_voxels scene_water.launch.py \
  selection_mode:=cluster start_detection:=false start_segmentation:=false
```

`selection_mode:=bbox` 仍作为 `cluster` 的历史别名保留。

历史完整融合链路仍可通过根目录 `d1.launch.py`、`avia.launch.py`、`hil_sim.launch.py`、
`ual_planner.launch.py` 使用，例如 `ros2 launch pointcloud_voxels d1.launch.py`。
它们保留原有投影/实例图/Landing 行为，独立于新的检测栅格入口。

## 路径规划

### 为什么没有直接接 EGO-Planner

本机 `/home/crl/ego-planner` 是 ROS 1 catkin 工程（`roscpp`、`quadrotor_msgs`、
catkin devel），不能直接链接到 ROS 2 Jazzy。官方 ROS 2 版本位于 ego-planner-swarm 的
`ros2_version` 分支，本工作区没有该源码；完整接入还需要局部地图/里程计/FSM、世界坐标系、
目标接口和单独的构建验证。当前 master 的检测/掩码链路工作在 LiDAR 传感器坐标系，
没有可用的世界系里程计和 TF。因此本轮采用一个自包含的简单 3D A*，不伪装成 EGO-Planner
的动力学轨迹优化，也不输出控制指令。

### 目标点与局部路径

`ship_path_planner` 订阅 `/ship/points`、`/ship/bbox_target` 和
`/livox/lidar/points`：

1. 目标点 = 维护的点云 3D bbox 中心（可选 `target_offset`）；没有 bbox target 时
   回退到选中点云的三维 AABB 中心。有当前检测时取当前选中目标对应的 bbox；
   只有没有当前观测时才从保留 bbox 中按体积/置信度选一个。
2. 起点默认是 LiDAR 原点 `[0,0,0]`，可通过 `start_point` 覆盖；没有接入里程计闭环。
   默认 `instance_selection: largest`，即一个目标；设为 `all` 时在当前观测中按
   `bbox_target_selection` 选最大体积或最高置信度的 bbox。
3. 其他 LiDAR 点作为障碍；选中目标点云 AABB 加 `target_exclusion_margin` 后从障碍中排除，
   避免把目标船体自身当成不可达障碍。
4. 在起点/目标包围盒加边距的规则网格上做 26 邻域 A*，默认分辨率 1 m、障碍膨胀 0.5 m；
   之后做视线可达的折线简化，发布两端精确等于起点/目标。
5. 输出保持在点云原始 `avia_frame`；不伪造世界坐标、TF、里程计或控制指令。

失败时仍发布目标/bbox 标记和空路径，状态包括 `empty_target_cloud`、
`no_obstacle_cloud`、`no_path` 或 `blocked: ...`。参数在
[config/pointcloud_voxels.yaml](config/pointcloud_voxels.yaml) 的 `path_planner` 段：
`start_point`、`target_offset`、`resolution`、`obstacle_inflation`、`margin_xy/z`、
`max_grid_cells` 等。

RViz 的 `Point-cloud bbox (maintained)` 显示 `/ship/bboxes` 的 3D OBB 和
尺寸/置信度标签；`Planned path` 显示 `/ship/path`；`Path target and selected AABB`
显示 `/ship/path_markers`（绿色起点、红色目标、选中点云 AABB、青色路径线、状态文字）。
路径规划随检测结果重复触发；`start_path_planner:=false` 可关闭。

这是局部坐标系内的路径演示，不是闭环飞行控制、避障安全认证或 EGO-Planner 等价实现。

## 验证与交接

```bash
python3 -m unittest discover -s tests -v
git diff --check
```

构建后可运行包含真实 ROS 2 消息/DDS 测试的 CTest：

```bash
source install/jazzy/setup.bash
colcon --log-base log/jazzy test --build-base build/jazzy --install-base install/jazzy \
  --return-code-on-test-failure
colcon test-result --test-result-base build/jazzy --verbose
```

`scripts/verify_detection_pipeline.py --report-dir data/<new-directory>` 会从转换包选一帧
真实图像和相同采集时刻点云，启动检测、选点、路径规划和 RViz，检查检测消息、选中点云、
体素、标注图、路径端点、路径 Marker 和 RViz 的 `/ship/path` 订阅；
默认使用近似几何、本机 `.venv-yolo` 和已校验权重，既有报告不会覆盖。
本机本次结果保存在
[detection_frame.json](data/validation_point_bbox_20261028/detection_frame.json)
（`data/` 被 Git 忽略）。`scripts/verify_scene_water_pipeline.py` 仍用于 SAM3 掩码兼容路径。

五包 Jazzy 构建通过，CTest 汇总86项、零错误/失败/跳过。
新增 7 项点云 bbox 生成/维护测试，覆盖投影聚类、PCA/AABB、法向高度、低置信度不更新、
高置信度替换、跨帧保留/超时、不同区域/类别新 ID 和最大数量限制。
转换数据的完整计数和2797帧像素校验、回放、安装启动均已验证。

真实单帧检测+点云 bbox+规划链路：第300秒图像 → YOLO11n-OBB 1个 `ship`（0.284）→
925点 → 466体素（蓝283/绿94/红89）→ 点云 3D bbox `ID1`，尺寸约
`32.9 × 4.1 × 5.8 m`、883点，中心 `(49.9235, 8.1008, 2.4482)` →
2个路径点、路径长 50.64 m，规划耗时约 123 ms。点云/bbox/目标/路径时间戳与图像一致，
路径终点等于点云 bbox 中心；RViz 已订阅 `/ship/path` 和 `/ship/bboxes`。
该帧路径近似直线，说明当前目标 AABB 排除策略下没有更明显的绕行；
单帧结果不代表复杂场景避障效果。
首次调用的观测结果延迟约1.6秒（CPU，含模型/launch/RViz 启动），
单帧推理在预热后约0.04秒，路径规划约0.1秒；这些都不是全 bag 精度、实时性或安全认证结论。

SAM3 兼容路径的旧证据仍保留：1个实例掩码 →1145点 →645体素，CPU Q4_0 延迟约30.26秒。
这些都只是接口/数值链路验证，不代表识别精度、安全降落面或正式标定通过。
详细状态、未验证项与后续工作见 [docs/PROJECT_STATE.md](docs/PROJECT_STATE.md)，
架构决定见 [docs/DECISIONS.md](docs/DECISIONS.md)。