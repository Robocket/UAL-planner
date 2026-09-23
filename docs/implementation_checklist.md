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
- [x] 提供 D1、Avia、HIL 三个 Jazzy launch；设备驱动不耦合到算法启动层。
- [x] RViz launch 与统一显示配置移到仓库根目录，并由 `ual_planner_bringup` 安装和启动。
- [x] 核对 Scene_Water bag 的实际话题与 frame，并提供以 `avia_frame` 为输入点云
  Fixed Frame 的 `scene_water.yaml`、launch 和分割投影 RViz 配置。
- [x] 实例分割后端替换为 Meta SAM 3，以 `large ship` 文本提示识别海上大型船只；
  删除本地权重路径和 Ultralytics 依赖，由官方模型代码自动下载 Hugging Face 权重。
- [x] 提供隔离的 Jazzy 构建脚本，避免 Humble/Jazzy 路径混入同一个构建缓存。
- [x] 修复 Python 节点可执行权限和 Jazzy 重复 SIGINT 时的退出清理。
- [x] Jazzy 全量构建成功，Landing C++/Python 测试全部通过，三套 launch 已完成解析与
  参数加载验证。

## 明确保留为后续工作

- [ ] D1/Avia 与相机的真实内外参实机标定；当前数值是从原有节点迁移的测试初值。
- [ ] ROS 1 Noetic 的 `roscpp/rospy` 薄包装和对应 `.launch`；共享算法核心及参数名保持不变。

## 本次未添加

- [ ] 跨模态轮廓匹配与完整特征关联图。
- [ ] 跨帧点云配准、实体级融合、非合作移动目标跟踪与轨迹预测。
- [ ] 分阶段自主降落轨迹规划与控制。

以上功能在报告当前章节尚未形成可实现算法，因此本次没有自行补写。
