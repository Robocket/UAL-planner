# 开题报告第三章算法优化清单

## 已完成

- [x] 仅优化报告中已经描述并在现有代码中实现的降落面评估流程。
- [x] 点云先做雷达到机体的外参转换，再按 IMU 重力方向进行姿态对齐。
- [x] 保留单帧圆形 ROI、RANSAC 支撑平面、坡度、粗糙度、内点率、2.5D 栅格覆盖率、
  空域侵入、相邻高度台阶、三态判定、连续帧确认和 watchdog。
- [x] 相邻台阶改为相对拟合平面的残差比较，避免斜坡自身高度趋势被误报为台阶。
- [x] 栅格只统计圆形 ROI 内的单元，并加入 `max_grid_cells` 内存上限。
- [x] RANSAC 随机种子改为参数 `ransac_seed`，保证测试可复现。
- [x] 二次平面分类点数不足时安全返回 UNKNOWN，而不是继续输出不可靠结果。
- [x] Landing、实例分割、点云投影、实例图作为并列节点，由 `ual_planner_bringup` 管理。
- [x] `ins_seg/launch/seg.launch.py` 恢复为只启动本包三个节点，不再包含 Landing。
- [x] 算法公共参数集中到仓库根目录 `config/common.yaml`。
- [x] D1、Avia、HIL 的话题、时钟、坐标系、雷达/相机标定与设备覆盖分别集中到
  `config/d1.yaml`、`config/avia.yaml`、`config/hil_sim.yaml`。
- [x] 同一设备配置内，点云投影与 Landing 使用同一个雷达话题；D1 均使用 `/iv_points`。
- [x] 点云投影支持 `world`、`base`、`sensor` 三种输入坐标语义，并通过配置完成
  sensor → base → world 转换。
- [x] 点云投影与实例图统一以世界坐标为内部计算基准；原始传感器/机体点云先转换到
  里程计世界系，`/projected_cloud` 与 `/projected_instance_info` 使用相同世界 frame_id。
- [x] 提供 D1、Avia、HIL 三个 Jazzy launch；设备驱动不耦合到算法启动层。
- [x] RViz launch 与统一显示配置移到仓库根目录，并由 `ual_planner_bringup` 安装和启动。
- [x] 核对 Scene_Water 原始 bag 的实际话题、消息类型与 frame；原始输入严格使用
  `/left_camera/image/compressed`、`/livox/lidar`、`/livox/imu` 和
  `/fsdk/aircraft_state`。
- [x] Scene_Water 启动入口自动将 `CompressedImage` 转为 `/left_camera/image`，并由仓库
  内的适配节点将 `CustomMsg` 转为 `/livox/lidar/points`；投影与 Landing 共享该标准点云。
- [x] 提供 Jazzy 原生的 `livox_avia_driver/msg/CustomMsg` 兼容接口，Scene_Water 回放不再
  依赖含 Humble 路径或 Python 3.10 产物的外部工作空间。
- [x] Scene_Water RViz 的 Fixed Frame 使用 `avia_frame`，点云和两路算法图像分别对齐
  `/livox/lidar/points`、`/sam3/annotated_image` 和 `/projected_image`。
- [x] Scene_Water 标准点云输出使用可配置的 Reliable QoS，同时兼容 RViz Reliable 与
  算法节点的 Best Effort 订阅。
- [x] 实例分割后端替换为 Meta SAM 3，以 `large ship` 文本提示识别海上大型船只；
  删除本地训练权重和 Ultralytics 依赖，改从魔塔 `facebook/sam3` 自动下载 Transformers
  权重，并使用 TorchAO INT8 权重量化适配 4GB 显存。
- [x] 在 NVIDIA T600 4GB 和 Scene_Water 船只画面上完成端到端验证：INT8 权重在 CPU
  量化后移至 GPU，节点稳定发布 3–4 个实例，避免 FP16 在 Turing 上产生 NaN 输出。
- [x] SAM 3 默认改用动态激活 INT8 + INT8 权重量化，并缓存固定文本提示编码；T600
  单帧总耗时由约 6 秒降至约 3.5 秒，同时发布 `/sam3/performance` 分阶段耗时和显存诊断。
- [x] 增加并实测最小的动态激活 INT8 + INT4 权重量化选项；`group_size=128` 可正常识别，
  但 T600 上约 6.7 秒/帧且峰值显存约 1.59 GiB，性能差于默认 A8W8，故保留为可配置
  试验项而不替换性能默认值。
- [x] 投影同步改为点云/里程计时间缓存，由延迟到达的分割结果按原始时间戳查找最近数据；
  Scene_Water 分别保留约 8 秒点云和 10 秒里程计，并处理 rosbag loop 时间戳回跳。
- [x] Scene_Water 海康相机与 Avia 雷达暂按同轴安装处理，直接 LiDAR→Camera 外参设为
  单位旋转和零平移，内参 `fx/fy` 暂沿用原值。
- [x] 提供隔离的 Jazzy 构建脚本，避免 Humble/Jazzy 路径混入同一个构建缓存。
- [x] 修复 Python 节点可执行权限和 Jazzy 重复 SIGINT 时的退出清理。
- [x] Jazzy 全量构建成功，Landing C++/Python 测试全部通过，四套 launch 已完成解析与
  参数加载验证。

## 明确保留为后续工作

- [ ] D1/Avia 与相机的真实内外参实机标定；当前数值是从原有节点迁移的测试初值。
- [ ] ROS 1 Noetic 的 `roscpp/rospy` 薄包装和对应 `.launch`；共享算法核心及参数名保持不变。

## 本次未添加

- [ ] 跨模态轮廓匹配与完整特征关联图。
- [ ] 跨帧点云配准、实体级融合、非合作移动目标跟踪与轨迹预测。
- [ ] 分阶段自主降落轨迹规划与控制。

以上功能在报告当前章节尚未形成可实现算法，因此本次没有自行补写。
