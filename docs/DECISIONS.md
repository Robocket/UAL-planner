# master 架构决定

更新日期：2026-10-28。维护范围：master 本次集成与回放。
visual-only 的历史 D01–D17 保留在原分支，不被本文件改写。

## M01：ws_ual 三维栅格集成到 master，默认由实例掩码选点

- 状态：用户 2026-10-26 明确要求；代码已实现并验证。2026-10-28 后默认模式由 M05 改为检测框。
- 算法来源：本机 ws_ual 点云聚类/最大框区域三维体素；移植纯数值核心与 Jazzy 包装。
- 掩码来源：采用 visual-only 使用的原图实例 ID 图。核查 master 已含相同生成实现，
  直接复用，不合并纯视觉分支删除历史雷达包等无关改动。
- 默认空间筛选以真实像素掩码为准，零标签及孔洞排除；原最大 bbox 只作显式独立对照。
- 船体栅格按六个面的空面数分类：绿 `==4`，红 `>=5`，蓝为其余；保持此前颜色与透明度。
  这是占用启发式，不声称可靠识别真实船角或安全降落面。
- 掩码和点云必须按采集时间匹配；点云输出沿用原 frame，标定未经确认时明确阻止掩码选点。
  不伪造 frame/time、补世界坐标或新增跟踪/控制。

## M02：根目录维护 launch/config/rviz，移除 Bringup 包

- 状态：沿用用户此前“对应文件直接放仓库主目录对应文件夹”的要求，本轮执行。
- 算法包在 `src/` 并列。原 Bringup 启动文件迁入根目录 `launch/`，删除其包构建入口。
  `pointcloud_voxels` 在安装时收集根目录三类资源，支持源码路径和安装包两种启动入口。
- 默认 Scene_Water 入口启动掩码栅格链路，保留图像/Livox 消息适配。
  历史 D1/Avia/HIL 全链路入口保持可选，不因新默认流程删除原算法实现。
- 只面向 ROS 2 Jazzy，不新增 Noetic 包装。模型、bag、缓存、凭据及机器环境不纳入集成。

## M03：旧 bag 转为 Jazzy 标准消息并内置可显示图像

- 状态：2026-10-28 用户要求，已执行转换及原生回放/图像订阅验证。
- 可读恢复版输出 SQLite3/CDR，保留四个原始文件；损坏备份不重复转换或静默忽略。
- 标准点云统一输出 `/livox/lidar/points`，原 JPEG 和新增 `/left_camera/image` 同时保留，
  以增加磁盘占用换取直接回放显示、无需额外图像解码节点。原 header/frame/时间不重写。
- 根目录增加独立播放/RViz 入口，不依赖未构建的算法包，不启动模型或推断。
  Snap 环境的 GTK/GIO 干扰只在 RViz 子进程局部处理，不改系统库。
- 这只验证数据完整性与回放显示，不代表融合标定或算法精度已通过；证据见项目状态和 bag 报告。

## M04：用户授权近似内外参联调，保留正式标定状态

- 状态：2026-10-28 用户说明本机无相机内参，要求参考海康通用型号，允许相机/雷达近似共点、共同朝地面。
- 查到的是 MV-CA013-20GC 厂家传感器规格，不是其任意镜头的通用 K。
  临时采用4.8 μm像元、1280×1024、假设6 mm镜头的 K（fx/fy1250、cx640、cy512），畸变暂零。
- 共点同向仍须做坐标轴转换，暂采用光学坐标 `(-y_lidar,-z_lidar,x_lidar)`，并假设没有额外相对偏航。
  新增独立近似配置和 `geometry_config` 覆盖入口，严格配置及 `calibration_confirmed=false` 保留。
  只有显式 `allow_approximate_calibration` 才启用临时几何，不把联调输出称为标定通过。
- 当时已构建五包、CTest 52项零失败、原生消息/历史启动验证和真实单帧非空掩码选点/体素输出通过。
  单帧结果不能代表识别精度、全场景效果或安全降落面；没有新增训练或替换模型。

## M05：默认检测模型换成 visual-only 的 YOLO11n-OBB，保留实例掩码兼容

- 状态：2026-10-28 用户要求将 master 的实例分割模型换成 visual-only 使用的目标检测模型，
  算法需兼容两种模型，并以 bbox 内点云特征作为主要输入。已实现、构建并通过真实单帧验证。
- 模型：visual-only 已验收的 YOLO11n-OBB / DOTA `ship`，权重
  `b62898eb...aa9f3a32`（5,795,654 B），由显式脚本下载到
  `~/.cache/ual_planner/models/`，不入库、不自动下载。
- 输出：新增 `ins_seg/DetectionResult`，每个目标携带原图像素四点多边形、类别、置信度。
  OBB 保留旋转四边形；水平检测展开为轴对齐四边形；不生成/外接填充实例 ID 图。
- 算法：`selection_mode:=detection` 为默认，只选择图像边界内且落在多边形内的点；
  `mask` 继续使用真实 32SC1 实例像素；`cluster` 保留纯雷达 PCA 框诊断。
  三者共用相机投影、点云时间匹配、三维体素分类、`/ship/points` 和 RViz 输出。
- 依赖：Ultralytics/PyTorch 不进入 ROS 构建；launch 的 `detector_python` 指定独立解释器。
  Jazzy `cv_bridge` 需要 NumPy 1.x 和 OpenCV 4.x，安装说明已固定版本。
- 权重/阈值/类别在 `config/detector.yaml`；检测选择与体素在
  `config/pointcloud_voxels.yaml`；未新增训练、跟踪或安全面判定。
- 验证：五包构建、CTest 65项零失败；真实 Scene_Water 第300秒单帧得到1个 `ship`
  (0.284) → 925点 → 466体素（蓝283/绿94/红89），检测/点云/标注图时间与尺寸一致。
  单帧接口通过不代表识别精度、正式标定或实时性验收。

## M06：bbox 中心为目标点，采用自写 3D A* 而非直接接 EGO-Planner

- 状态：2026-10-28 用户要求在检测 bbox 中心做路径规划并可视化；优先评估 EGO-Planner，
  复杂则使用简单规划。已实现并在真实单帧 + RViz 中验证。
- EGO-Planner 评估：本机 `/home/crl/ego-planner` 是 ROS 1 catkin 工程，
  依赖 `roscpp`、`quadrotor_msgs`、catkin devel，不能直接链接 ROS 2 Jazzy；
  官方 ROS 2 版本在 ego-planner-swarm 的 `ros2_version` 分支，本工作区没有源码。
  完整接入还需要局部地图/里程计/FSM、世界坐标系、目标接口和独立构建验证。
  当前链路没有世界系里程计/TF，只有 LiDAR 系点云，因此不直接接 EGO-Planner。
- 目标：`/ship/points`（bbox 内点云）的三维 AABB 中心，支持 `target_offset`；
  起点默认 `[0,0,0]`（LiDAR 原点），支持 `start_point`，没有接入闭环里程计。
- 规划：障碍来自 `/livox/lidar/points`，目标 AABB 加边距后从障碍中排除；
  26 邻域 A*，默认 1 m 网格、0.5 m 膨胀，之后做视线可达折线简化。
  输出 `/ship/target`、`/ship/path`、`/ship/path_markers`、
  `/ship/path_status`，保持在原始点云 frame。
- RViz：`ship_voxels.rviz` 增加 `Planned path` 和 `Path target and bbox`；
  绿色起点、红色目标、黄色目标 AABB、青色路径线、状态文字。
- 失败语义：空目标、无匹配障碍点云、无路径或规划异常都发布空路径与状态，
  不伪造一条不可达/未校验的路径。
- 验证：五包构建、CTest 77项零失败；真实第300秒单帧得到目标
  `(49.92,7.94,1.93)`、2个路径点、50.59 m，规划约141 ms，
  RViz 确实订阅 `/ship/path`。该帧路径近似直线，不能证明复杂场景避障效果。
- 边界：这是传感器系局部路径演示，不是 EGO-Planner 的动力学优化、避障安全认证、
  闭环控制或真实世界坐标轨迹。
- 注：M07 将目标从选中点云 AABB 中心改为维护的点云 3D bbox 中心；本条 AABB 数字是历史记录。

## M07：维护的是点云 3D bbox，不是图像 bbox

- 状态：2026-10-28 用户先要求 bbox 持续化，后明确“不是图像中的而是点云中的”，
  并要求参考 `~/ws_crl` 的 bbox 生成逻辑。本机不存在该目录，改用可用的同类实现
  `/home/crl/ws_ual/src/pointcloud_cluster_bbox/scripts/pointcloud_cluster_bbox.py`。
- 生成：`pointcloud_voxels/bbox.py` 对选中 3D 点投影到 `xy/xz/yz`（默认 `yz`），
  体素化后按欧氏连通聚类，对主簇做 PCA 有向矩形，再沿法向取点云 min/max，形成 8 角点
  3D OBB、中心、长/宽/高和点数。无有效簇时可回退整体 AABB。
- 维护：`PointCloudBBoxTracker` 在 3D LiDAR 坐标中匹配；同类别且 3D AABB IoU 或中心
  距离命中时，只有新置信度严格更高才替换 bbox 几何/置信度，否则保留旧 3D bbox。
  连续 `bbox_timeout_sec` 未匹配才过期；稳定 `track_id`，最多 `max_bboxes` 个。
- 集成：`mask_voxels` 在检测选点后按实例生成 bbox，发布
  `/ship/bboxes`（8 角点 OBB + 尺寸/置信度/点数标签）和
  `/ship/bbox_target`（当前选中观测对应 bbox 的中心；无当前观测时按体积/置信度选择）。
  图像检测框只作为 ROI，
  图像端不再维护 bbox，`yolo_detector.py` 恢复每帧原始检测。
- 路径：`ship_path_planner` 优先使用与点云同时间戳的 `/ship/bbox_target`；
  没有时回退到选中点云 AABB 中心。因此路径目标是点云 3D bbox 中心。
- 参数：`bbox_projection_plane`、`bbox_voxel_size`、`bbox_cluster_tolerance`、
  `bbox_min_cluster_points`、`bbox_min_box_size`、`bbox_min_height`、
  `bbox_confidence_margin`、`bbox_match_iou`、`bbox_match_center_ratio`、
  `bbox_timeout_sec`、`max_bboxes`、`bbox_target_selection`，位于
  [config/pointcloud_voxels.yaml](../config/pointcloud_voxels.yaml) 的 `mask_voxels` 段。
- 界面：RViz 增加 `Point-cloud bbox (maintained)` 显示 `/ship/bboxes`；
  路径目标显示点云 bbox 中心，不再使用图像 bbox 中心。
- 验证：CTest 86项零失败；新增 7 项 bbox 生成/维护纯测试、原生检测 bbox 输出断言和
  bbox 目标优先路径测试。
  真实第300秒单帧：点云 3D bbox 目标 `(49.9235,8.1008,2.4482)`，
  路径 2 点、50.64 m；`/ship/bbox_target` 与点云同时戳，路径终点与 bbox 中心一致，
  `/ship/bboxes` 含 `ship_point_bbox`/`ship_point_bbox_labels`。
- 边界：3D bbox 来自当前可见点云的主簇，不是完整船体 CAD/真值；遮挡、稀疏、背景点和
  投影面选择会影响中心/尺寸。持续化没有运动补偿；相机/目标移动时只有更高置信度观测
  才能更新 bbox。不是跟踪、世界坐标、安全降落面或 EGO-Planner 等价实现。

## 验证边界

2026-10-28 用户进一步要求数据放仓库当前目录但不上传：实际数据统一置于根目录 `data/`，
整个目录加入 Git 忽略；只维护脚本与配置，不暂存数据。旧位置保留软链接兼容历史命令。

数值测试通过与代码架构完成不等于在线跑通、精度合格或安全面已验收。
本机已完成 Jazzy 构建、86项CTest、回放、RViz、检测/掩码选点、点云 3D bbox 维护及局部路径的实帧接口链路；
正式标定、识别精度、复杂场景避障和闭环控制验收仍待完成。
本次不自动 commit/push，原有其他分支历史保持不变。