# landing_evaluator

独立 ROS 2 降落面评估节点。输入标准 `sensor_msgs/msg/PointCloud2`，输出
`landing_evaluator/msg/LandingStatus`，不依赖具体雷达驱动或 PCL，便于迁移到其他工作空间。

在完整 UAL-planner 中，本节点与实例分割、点云投影和实例图节点并列，由
`ual_planner_bringup` 启动。仓库根目录 `config/` 是集成运行的统一配置源；本包内的
`config/*_test.yaml` 和测试 launch 仅保留给节点独立标定、回放与验收，不是总工作流的
权威配置。D1 集成启动命令为：

```bash
ros2 launch ual_planner_bringup d1.launch.py
```

算法实现固定外参转换、圆形降落 ROI、RANSAC 平面、坡度与残差、2.5D 网格覆盖率、
凸起及相对拟合平面的相邻网格台阶检测，并进行连续多帧确认。

节点同时发布 `/landing/height_map`。每个网格包含地面高度、最高观测点、相对拟合
地面的空域侵入高度、点数和占用标志。`airspace_clearance_height` 定义允许高于地面的
净空余量，`airspace_ceiling_z` 排除飞机自身附近回波，单格达到
`airspace_min_points` 后视为下降走廊被占用。`max_occupied_cell_ratio` 默认 0，表示任一
可靠占用网格都会禁止降落。

## D1 悬停测试

`config/d1_test.yaml` 针对 Seyond D1 驱动的 `/iv_points`。D1-R 用户手册定义的原始
点云轴为 X 向上、Y 向右、Z 向前，原点位于 RX 镜头表面中心。驱动使用
`coordinate_mode: 3` 后输出轴变为 X 前、Y 左、Z 上。

该用户手册没有定义内置 ICM-45686 的轴向，也没有给出 IMU 到 LiDAR/点云坐标系的安装
旋转。Seyond ROS 驱动源码会把 SDK 中的 IMU `x/y/z` 直接当作 LiDAR 原始轴，并对点云和
IMU 应用同一套 `coordinate_mode` 变换；这是驱动实现中的假设，不是本手册提供的标定关系。

D1 实机三轴静态测量为：手册 +Z（镜头）向下时 `imu_x=-g`，手册 +X 向下时
`imu_z=+g`，手册 +Y 向下时 `imu_y=-g`。在驱动 `coordinate_mode=3` 前提下，由此得到
IMU 消息坐标到驱动点云坐标的旋转为 `Rx(pi)`。测试安装的点云到 base 旋转是
`Ry(+pi/2)`，所以最终 IMU 外参为 `Ry(+pi/2)*Rx(pi)`，按节点的
`Rz(yaw)*Ry(pitch)*Rx(roll)` 参数顺序写作：

```yaml
imu.extrinsics.rotation_rpy: [3.141592653589793, 1.5707963267948966, 0.0]
```

测试假设是“手册原始 +Z（雷达前向轴）与重力同向”。此时驱动输出 +X 朝下，所以配置
使用 `pitch=+pi/2` 转换到飞机 Z 向上坐标，而不是 `roll=pi`。绕重力轴的安装角不会
影响圆形 ROI 和坡度，但如果下游使用 Height Map 的 X/Y 方位，仍需标定完整安装偏航角。
测试前必须以 RX 镜头表面中心为外参原点填写 `extrinsics.translation`。

```bash
ros2 launch landing_evaluator d1_test.launch.py
ros2 topic hz /iv_points
ros2 topic echo /landing/status
```

在 `coordinate_mode: 3` 且镜头垂直朝下、雷达静止时，地面点应主要位于 `/iv_points`
的 `+X` 方向，`/iv_imu.linear_acceleration.x` 应主要为负值（约 `-9.81 m/s²`），另外两轴
接近零。若不满足，应先核对驱动坐标模式和实际 IMU 符号，不能继续沿用当前 `+pi/2` 外参。
应在旋转停止后分别以四个 yaw 方位静置测试；若静置后的 slope 仍随 yaw 改变，说明 D1
内置 IMU 与点云轴并非当前假定的同轴关系，必须重新标定 `imu.extrinsics.rotation_rpy`。

该 launch 默认启动评估节点、静态外参、Marker 可视化和 RViz，不依赖或启动 Seyond
驱动，以便单独移植和编译。推荐先在另一个终端启动 D1 驱动。若确实希望合并启动，需
显式提供驱动 YAML 的绝对路径；其中应确认 `lidar_ip`、`udp_port`、
`frame_topic: /iv_points`、`frame_id: seyond` 以及 `coordinate_mode: 3`：

```bash
ros2 launch landing_evaluator d1_test.launch.py \
  start_driver:=true \
  driver_config:=/absolute/path/to/seyond/config.yaml
```

无桌面环境或 RK3588 上只运行算法：

```bash
ros2 launch landing_evaluator d1_test.launch.py visualize:=false
```

RViz 点云已自动从 `/livox/lidar` 重映射到 `/iv_points`。若驱动使用不同 frame，可传入
`sensor_frame:=实际frame`。launch 中的 `extrinsic_*` 仅用于 RViz TF，必须与
`d1_test.yaml` 的 `extrinsics.translation/rotation_rpy` 保持一致。

在平地悬停或架高静态测试时，`plane_height` 应为负值，其绝对值应接近 `base_link`
原点到地面的垂直距离，而不一定是 RX 镜头到地面的距离（两者差值由外参平移决定）。
若外参平移暂时为零，其绝对值才近似等于雷达到地面的垂直距离。如果为正值，说明轴向
或外参配置错误，禁止继续使用当前判定；应先确认驱动 `coordinate_mode` 和实际安装方向，
再重新标定外参。

## Avia 重力轴测试

`config/avia_test.yaml` 假定 Avia 自身 +X 与重力同向，采用 `pitch=+pi/2`，使
`base_z=-lidar_x`。启动实时评估并运行 15 秒验收：

```bash
ros2 launch landing_evaluator avia_test.launch.py
ros2 run landing_evaluator avia_landing_test.py
```

`avia_test.launch.py` 默认同时启动降落评估、静态雷达外参、Marker 可视化和 RViz，并将
RViz 点云显示映射到 `/livox/lidar`。无桌面环境或在 RK3588/香橙派上仅运行算法：

```bash
ros2 launch landing_evaluator avia_test.launch.py visualize:=false
```

如果 RViz 在另一台电脑运行，可让板端继续发布 TF 和降落 Marker，但不启动本地 RViz：

```bash
ros2 launch landing_evaluator avia_test.launch.py use_rviz:=false
```

若旧 rosbag 中 `/livox/lidar` 的实际类型是
`livox_avia_driver/msg/CustomMsg`，不能让它与评估所需的
`sensor_msgs/msg/PointCloud2` 共用同一个话题名。回放时先将 CustomMsg 改名：

```bash
ros2 bag play /path/to/bag --loop \
  --remap /livox/lidar:=/livox/lidar_custom
```

然后启用可选适配器：

```bash
ros2 launch landing_evaluator avia_test.launch.py convert_custom:=true
```

数据链为：

```text
/livox/lidar_custom  (livox_avia_driver/msg/CustomMsg)
    -> custom_to_pointcloud2_node
/livox/lidar         (sensor_msgs/msg/PointCloud2)
    -> landing_evaluator + RViz
```

适配器属于 `livox_avia_driver`，默认不启动，因此 `landing_evaluator` 的 C++ 节点仍只依赖
标准 PointCloud2；在不需要兼容旧 bag 的目标板上可继续单独编译和使用评估包。

点云话题、frame 或实测外参不同时，可通过 `pointcloud_topic`、`sensor_frame` 和
`extrinsic_*` 覆盖。`pointcloud_topic` 会同时设置评估节点输入、适配器输出及 RViz 显示；
`extrinsic_*` 仅控制 RViz TF，必须与 YAML 的 `extrinsics.translation/rotation_rpy`
保持一致。

验收脚本检查状态消息数量、拟合地面是否主要位于负 Z、最大处理延迟是否小于 100 ms，
并汇总三态与拒绝原因。通过只代表数据链、坐标方向和处理实时性正常，不代表全部降落阈值
已经完成实机标定。

脚本同时订阅 `/livox/lidar`，会分别报告有效 PointCloud2 数量、LandingStatus 数量及
`status/lidar` 比例。超过 90% 的输出为 UNKNOWN 时会提示检查 ROI、覆盖率和外参，但
不会把数据链测试误报为失败。bag 回放建议显式加载录制目录中的 QoS：

```bash
ros2 bag play /path/to/bag --topics /livox/lidar --loop \
  --qos-profile-overrides-path /path/to/jazzy_qos_overrides.yaml
```

若出现 `sequence size exceeds remaining buffer`，先根据验收脚本的分层计数定位：若
`lidar_messages=0`，通常是 bag 中点云类型、序列化数据或 QoS 不兼容；若点云有计数但
状态为 0，则检查 `/landing/status` 发布端和订阅端是否使用了不同版本的
`LandingStatus`。后一种情况应停止旧评估节点，在两个终端中仅 source 同一套新 install
后重新启动；仅重新编译而不重启发布进程无法更新进程内已经加载的消息类型。

## 半物理仿真

`config/hil_sim.yaml` 和 `launch/hil_sim.launch.py` 面向带 `/clock` 的半物理仿真环境。
launch 不会重复启动 PX4 bridge、仿真器或 Livox 驱动，只订阅仿真系统已经发布的
`/livox/avia/points`，并发布 `/landing/status`、`/landing/height_map` 和可视化 Marker。

```bash
ros2 launch landing_evaluator hil_sim.launch.py
```

若只需要评估算法，并且不需要 TF、Marker 或诊断可视化节点：

```bash
ros2 launch landing_evaluator hil_sim.launch.py visualize:=false
```

半物理仿真 launch 默认不启动本地 RViz，适合无显示器的仿真主机或 SSH 终端；评估、TF、
Marker 和诊断话题仍会正常发布：

```bash
ros2 launch landing_evaluator hil_sim.launch.py
```

只在本机具有正常 `DISPLAY`/Wayland 图形会话时显式启动 RViz：

```bash
ros2 launch landing_evaluator hil_sim.launch.py use_rviz:=true
```

该 launch 将点云话题、降落半径和雷达外参同时传给算法与 RViz，避免 YAML 数值与静态
TF 不一致。例如：

```bash
ros2 launch landing_evaluator hil_sim.launch.py \
  sensor_frame:=实际点云frame \
  landing_radius:=1.0 \
  extrinsic_x:=0.0 extrinsic_y:=0.0 extrinsic_z:=-0.15 \
  extrinsic_roll:=0.0 extrinsic_pitch:=1.5707963267948966 \
  extrinsic_yaw:=0.0
```

当前仿真预设暂按“水平机体时 Avia 点云 `+X` 朝下”设置 `pitch=+pi/2`。这只是待验证的
安装假设；若仿真点云已是 `base_link` 或重力对齐坐标，应使用实际外参，且点云
`header.frame_id` 已为 `base_link` 时应传 `publish_sensor_tf:=false`。启动前至少核对：

```bash
ros2 topic info /livox/avia/points -v
ros2 topic echo /livox/avia/points --once --field header
ros2 topic info /gt/odom -v
```

点云类型必须为 `sensor_msgs/msg/PointCloud2`，并包含 `FLOAT32` 的 `x/y/z` 字段；
`sensor_frame` 必须与 `header.frame_id` 完全一致，否则 RViz 无法用静态 TF 显示原始点云。

已确认 `/gt/odom` 为 `nav_msgs/msg/Odometry`，父 frame 为 `981-A/odom`、子 frame 为
`981-A/base_footprint`。仿真 launch 默认启动 `odom_gravity_adapter.py`，按 Gazebo/ENU 世界
`+Z` 向上以及标准 Odometry 的 body-to-world 四元数语义，计算
`R_body_to_world^T * [0, 0, 1]`，再以 `/landing/sim_gravity_imu` 的标准
`sensor_msgs/msg/Imu` 向量接入现有姿态补偿。该适配器只使用最新回调，不缓存里程计历史，
也不依赖 `px4_msgs`。

```bash
ros2 topic echo /landing/sim_gravity_imu --once
```

飞机水平时输出应接近 `(0, 0, +9.80665)`；机体倾斜时向量应随 roll/pitch 改变。必须确认
`base_footprint` 的四元数确实包含 roll/pitch，而不是只保留 yaw。若仿真只测试水平姿态，或
需要排除真值姿态接口，可传 `use_ground_truth_attitude:=false`；此时 `slope` 只有在机体水平
或点云已经重力对齐时才有效。PX4 `/fmu/out/vehicle_attitude` 仍未耦合进评估包。

仿真时钟暂停不会冻结点云 watchdog；超过 `watchdog.cloud_timeout_sec` 的墙钟时间仍会发布
`UNKNOWN/cloud_timeout`，这是故障安全行为。低倍速仿真时应将该参数设为至少三个实际点云
发布周期。

仿真 Avia 单帧较稀疏时，HIL 预设使用 `grid_resolution=0.20 m`。以默认
`landing_radius=1.25 m` 计算，`0.10 m` 栅格约有 490 个待覆盖单元，而 `0.20 m` 约有
四分之一；保持 `grid_min_points=2`、`min_coverage_ratio=0.80` 时不会直接放宽未观测区域比例。
这三个参数可以从 launch 临时覆盖：

```bash
ros2 launch landing_evaluator hil_sim.launch.py \
  grid_resolution:=0.20 grid_min_points:=2 min_coverage_ratio:=0.80
```

若仍为 `low_coverage`，先确认 `inlier_ratio` 较高，再逐步将仿真栅格增至 `0.25 m`。只有在
每格通常恰好一个有效回波时，才在 HIL 中尝试 `grid_min_points:=1`。不建议优先降低
`min_coverage_ratio`，因为这会让未观测的降落区域被当作安全区域；也不使用插值填充或历史
点云累计来伪造当前帧的覆盖证据。

### HIL 录包

先启动仿真和 `hil_sim.launch.py`，再在另一个终端录制。只保留能够重新运行评估算法的必要
输入时，记录仿真时钟、原始点云和真值里程计，同时保存本次评估参数：

```bash
HIL_RECORD_DIR="./hil_records/hil_input_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${HIL_RECORD_DIR}"
ros2 param dump /landing_evaluator \
  > "${HIL_RECORD_DIR}/landing_evaluator_params.yaml"

ros2 bag record -o "${HIL_RECORD_DIR}/bag" \
  /clock \
  /livox/avia/points \
  /gt/odom
```

需要同时分析评估结果和 PX4 飞行状态时，使用完整记录列表：

```bash
HIL_RECORD_DIR="./hil_records/hil_full_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${HIL_RECORD_DIR}"
ros2 param dump /landing_evaluator \
  > "${HIL_RECORD_DIR}/landing_evaluator_params.yaml"

ros2 bag record -o "${HIL_RECORD_DIR}/bag" \
  /clock \
  /livox/avia/points \
  /gt/odom \
  /landing/sim_gravity_imu \
  /landing/status \
  /landing/height_map \
  /landing/diagnostics \
  /tf_static \
  /fmu/out/vehicle_attitude \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_status
```

使用 `Ctrl-C` 正常结束录制，等待 rosbag 写完缓存和 `metadata.yaml` 后再关闭终端。原始点云
通常占据绝大部分带宽；若录制期间 `/landing/status` 频率下降或出现丢帧，应优先写入高速
磁盘，并使用上面的必要输入列表，避免直接使用 `ros2 bag record -a` 录制全部 PX4 话题。

## RViz 降落平面可视化

`landing_rviz_visualizer.py` 将拟合平面裁剪到有地面证据的 ROI 网格，显示降落边界、
空域占用点和状态文字。颜色为绿=LANDABLE、红=NOT_LANDABLE、黄=UNKNOWN，紫色表示
下降空域占用。启动：

```bash
ros2 launch landing_evaluator landing_visualization.launch.py
```

输出 `/landing/markers` (`visualization_msgs/msg/MarkerArray`) 和
`/landing/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)。RViz 标准插件不提供任意
DiagnosticArray 键值侧栏，所以专用配置在三维视图中显示完整状态文字；结构化指标仍可
由诊断工具或后续自定义 RViz Panel 直接订阅 `/landing/diagnostics`。

面向嵌入式部署，节点在读取点云时立即过滤 ROI，RANSAC 只对候选平面计数并支持
`ransac_early_exit_ratio` 提前结束，P95 使用线性选择而非全排序。默认 60 次 RANSAC；
若 `processing_time_ms` 接近雷达帧周期，可继续降低该参数，但不建议删除覆盖率、障碍或
台阶等安全判据。

## IMU 姿态补偿

设置 `imu.enabled: true` 后，节点订阅 `imu.topic`，将雷达加速度先通过固定安装外参旋转
到机体坐标，再用归一化加速度方向估计重力方向，对每帧点云进行滚转/俯仰补偿。这样飞机
倾斜不会直接被当作地面坡度；偏航不影响圆形降落 ROI，因此不参与补偿。实现仅增加一阶
低通和一个 3x3 点旋转，适合嵌入式部署。

点云外参与 IMU 外参分别由 `extrinsics.rotation_rpy` 和
`imu.extrinsics.rotation_rpy` 指定。若两者坐标轴定义不同，不能共用同一组角度；Avia/D1
配置默认相同，仅适用于两类消息使用同一传感器坐标系的情况。

点云采用回调即时处理，每帧只使用最近一次已经到达的有效 IMU 样本，不等待未来消息，也不
保留点云/IMU历史缓存。这样内存占用固定、延迟最低，但点云与 IMU 最多存在一个传感器周期
的时间差。`imu.timeout_sec` 用于限制该最近样本的有效期；Avia bag 的 IMU 与点云 header
使用不同的时钟纪元，因此不使用 header 时间戳对齐。

点云和 IMU 订阅的 ROS 队列深度均固定为 1。评估期间如果新消息到达，旧消息会被丢弃而不
会排队，优先保证实时性；这属于传输层的单帧缓冲，不是算法历史缓存。

`imu.max_angular_velocity` 和 `imu.max_accel_deviation_ratio` 会拒绝剧烈机动期间不可信的
重力观测。若 `imu.required: true`，IMU 超过 `imu.timeout_sec` 未更新或没有有效观测时，
输出强制为 `UNKNOWN`，原因包含 `imu_unavailable`。核心 `landing_evaluator_node` 不直接订阅
或融合飞控 `Odometry`；这样可以避免在飞控坐标系定义未知时引入错误的姿态修正。半物理
仿真使用上文的独立适配器转换已确认的 `/gt/odom`，实机配置不会启动它。若要接入其他姿态
源，应先明确其坐标系并通过独立、明确的外参接口转换到雷达/机体坐标系。

当 `imu.required: true` 且姿态不可用时，状态为 `UNKNOWN/imu_unavailable`，同时
`slope_deg` 输出 `NaN`。此时点云平面仍可用于诊断，但其角度处于倾斜机体系，不能解释为
相对重力的真实地面坡度。D1 实机配置将 `imu.filter_alpha` 设为 `1.0`，直接采用最近一帧
归一化加速度；需要抑制静态噪声时可逐步调低。

## 构建与运行

```bash
cd /home/htfp/fast_landing
source /opt/ros/jazzy/setup.bash
colcon build --base-paths landing_evaluator livox_avia_driver
source install/setup.bash
ros2 launch landing_evaluator landing_evaluator.launch.py
```

驱动需发布 PointCloud2（默认配置已满足）：

```bash
ros2 launch livox_avia_driver avia.launch.py output_format:=pointcloud2 pointcloud_topic:=/livox/lidar
```

## 飞行阶段启停

节点提供 `~/set_enabled` (`std_srvs/srv/SetBool`) 服务。禁用时会销毁点云订阅、停止
全部评估并重置稳定状态；进程保持常驻，因此重新启用无需承担进程启动延迟。

```bash
# 启用
ros2 service call /landing_evaluator/set_enabled std_srvs/srv/SetBool "{data: true}"

# 禁用
ros2 service call /landing_evaluator/set_enabled std_srvs/srv/SetBool "{data: false}"
```

设置 `startup_enabled: false` 可让节点上电后保持空闲，等飞控发出启用请求。每次重新
启用后，状态从 UNKNOWN 开始，并重新执行 LANDABLE 连续多帧确认。

上游节点也可以通过 `enable_topic`（默认 `/landing_evaluator/enable`，类型
`std_msgs/msg/Bool`）控制。该订阅在算法禁用期间仍保持活动：

```bash
ros2 topic pub --once /landing_evaluator/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /landing_evaluator/enable std_msgs/msg/Bool "{data: false}"
```

服务和话题共用同一个幂等切换入口，重复发送相同状态不会重复创建或销毁订阅。对于只在
状态变化时发布一次的上游节点，应确保评估节点已经启动；若需要跨节点重启保留最后状态，
建议上游在连接建立后重发当前飞行状态，或周期性低频发布使能状态。

参数集中在 `config/landing.yaml`。`extrinsics.translation` 和
`extrinsics.rotation_rpy` 是雷达到飞机重力对齐坐标系的固定外参；`landing_radius`
是含安全余量的所需半径。飞行时必须先保证输出坐标系已经按 IMU 重力对齐，固定外参不能替代动态 roll/pitch 补偿。

各硬件/仿真 YAML 均显式列出当前节点的算法参数。`ransac_seed` 用于复现实验采样，
`max_grid_cells` 在分配二维栅格前限制容量，超限时输出 `UNKNOWN/grid_too_large`；
`imu.gravity_norm_filter_alpha` 控制有效加速度模参考值的缓慢更新。相邻高差使用每格相对
拟合平面的平均残差，且只比较圆形 ROI 内的邻接格，避免合法坡度趋势和圆外边界格触发
台阶拒绝。

`status`: 0=UNKNOWN、1=LANDABLE、2=NOT_LANDABLE。飞控应将 UNKNOWN 和
NOT_LANDABLE 都视为禁止自动下降。`raw_status` 是当前帧结果；`status` 是连续
`confirmation_frames` 帧后的稳定结果。为了安全，`NOT_LANDABLE` 和 `UNKNOWN` 会在
当前帧立即生效，只有进入 `LANDABLE` 状态需要连续多帧确认。

节点对每一帧输入点云评估并发布一次消息。`status_changed` 表示稳定状态在本帧发生
变化，下游可以用它触发告警或飞控动作；即使状态未变化，坡度、覆盖率等指标仍会持续
刷新。`processing_time_ms` 用于监控处理延迟，`reason_mask`、`reason` 及其他指标用于诊断。

点云 watchdog 默认在 `watchdog.cloud_timeout_sec`（0.30 秒）内没有收到有效 PointCloud2
时立即清除稳定状态，并发布 `UNKNOWN/cloud_timeout`；断流期间按
`watchdog.publish_period_sec`（1.0 秒）重复发布，避免晚启动的下游继续沿用旧的
LANDABLE。恢复点云后自动回到逐帧评估。节点被显式禁用时 watchdog 不发布状态。

日志采用事件模式：进入 `NOT_LANDABLE` 时立即打印 WARN；状态连续保持
`LANDABLE` 且期间未出现 `NOT_LANDABLE` 或 `UNKNOWN` 达到
`landable_log_delay_sec`（默认 5 秒）后打印一次 INFO。危险或证据不足会立即重置
这段安全计时。一次危险事件只打印一条 WARN；短暂恢复不会重新开放 WARN，只有连续
安全达到上述时长并打印 LANDABLE 后，下一次危险才会再次打印，避免状态抖动刷屏。
