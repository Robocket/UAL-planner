# master 集成状态

更新日期：2026-10-28（Asia/Shanghai）。本文件只记录本机 master 集成及数据回放，
visual-only 的历史试验和交接仍保存在该分支，不表示其权重/数据已同步到这台电脑。

## 已完成

- 实际目标为 `/home/crl/UAL-planner`，当前 `master` 基线 `bbe2261`。
  任务开始曾检出 `visual-only`，只读检查期间工作区从外部切到干净 master，未由本任务切分支。
  `/home/crl/ws_ual/UAL-planner` 的 `ros1` 检出及原 Noetic 点云算法均未修改。
- 从 `/home/crl/ws_ual/src/pointcloud_cluster_bbox/scripts/pointcloud_cluster_bbox.py`
  提取投影聚类/最大 PCA 框选点、三维占用栅格、六空面统计，集成为 ROS 2 `pointcloud_voxels`。
- 确认 master 与 `visual-only` 的 `seg.py`、`seg_q4_node.cpp`、两种二维分割消息、
  图像适配器实现相同。原图 `32SC1` 实例掩码保留为兼容路径。
- 2026-10-28 按用户要求，默认链路切换为 visual-only 已验收的 YOLO11n-OBB 检测框：
  检测多边形与采集时间匹配点云 → LiDAR 点投影 → 框内点云 → 每实例三维栅格 → RViz。
  `selection_mode:=mask` 仍按真实实例像素选点；最大 bbox 聚类仅作显式 `cluster` 对照。
- 默认分类沿用用户最后确认的规则：空面 `==4` 浅绿，`>=5` 红，其余半透明亮蓝。
  邻面在完整占用集合上统计；可视化数量截断在分类之后，避免假边缘。
- 算法节点并列放在 `src/`，launch/参数/RViz 源文件均在根目录对应文件夹。
  删除 `src/UAL_Bringup` 的包文件，将其启动入口迁到根目录；资源由算法包安装。
- 点云输出及 Marker 使用原始点云 header/frame；不伪造世界坐标、TF 或同步。
  Reliable 掩码输入、有界点云缓存、原采集时间匹配及点云回跳清理已实现。
  CameraInfo 和掩码尺寸/frame 检查、原图畸变投影、损坏消息拒绝处理已实现。
- 收窄崩溃转储的 Git 忽略模式，避免 `core.py` 被误忽略；节点入口补齐可执行权限，
  支持构建脚本默认的 symlink-install。

## 验证

- 2026-10-26 初次核查：本机 Python 3.12.3、NumPy 1.26.4、PyYAML 6.0.1；
  当时没有 `/opt/ros`、`rclpy`、`colcon` 或 OpenCV。2026-10-28 已确认 Jazzy 安装完成，见下节。
- 纯 NumPy 投影、畸变、掩码孔洞、最大实例选择、bbox 对照、六空面分类和点云/掩码
  二进制解码、延迟时间匹配、缓存回跳共 27 项 unittest 测试全部通过。
- 16 份 Python AST、包 XML、配置/RViz YAML、配置参数名/数组类型、启动资源路径、
  文档链接、构建脚本语法及 `git diff --check` 检查通过。
  这是离线检查，不是 ROS launch 解析通过或新模型精度评测。
- 已实际尝试 `./scripts/build_jazzy.sh`，入口因缺少 `/opt/ros/jazzy/setup.bash` 返回失败，
  未执行 colcon 构建；没有安装依赖绕过本机缺失环境。
- 2026-10-26 用户随后要求安装 ROS 2：核实本机 Ubuntu 24.04 amd64，根分区可用约 4.9 GB，
  `/home` 可用约 267 GB；`sudo -n true` 因需要密码失败，当前工具会话没有管理员权限。
  新增 `scripts/install_jazzy.sh`，按官方 ros2-apt-source 方式准备 Jazzy ros-base、RViz、
  ros-dev-tools 和项目系统依赖安装，`bash -n` 与无权限入口检查通过。
  尚未执行特权安装，需用户在本机终端运行 `sudo bash scripts/install_jazzy.sh` 输入密码。
  未向聊天索取密码、未修改 apt 配置或安装系统包；ROS 2 仍未安装。
- 上述 2026-10-26 集成任务没有安装依赖、下载模型/数据、训练、运行模型、人工看图、提交或推送。
  master 的工作区修改尚未 commit；没有更新远端或其他机器。

## 2026-10-28：Jazzy 数据转换与图像回放

- 已确认 `/opt/ros/jazzy/setup.bash`、rclpy、rosbag2_py、RViz、cv_bridge、系统 OpenCV 可用。
  本次不再安装系统 ROS 包；没有执行完整 UAL colcon 构建或启动 SAM3。
- 原目录 `/home/crl/ws_ual/data` 的两份 2.8 GB 文件均有可读索引、四话题计数一致。
  5.2 GB 的 `.orig.bag` 与 `_backup.bag` 在其 index_pos 处仍是损坏记录，未宣称修复或转换其全部内容。
- 使用 `_pointcloud2_noetic_recovered.bag` 只读转换到
  `/home/crl/ws_ual/data/Scene_Water_20260805_013651_ros2_jazzy`，原四个文件的大小/mtime 保持不变。
  输出 ROS 2 SQLite3/CDR、metadata v9，约 13 GiB、353.630558115 秒、99121 条消息。
- 消息完整计数：里程计 17682、IMU 72068、点云 3777、压缩图像 2797、新增标准 Image 2797。
  点云话题从 `/livox/lidar` 改为 `/livox/lidar/points`；全部源 header/frame/采集及记录时间保持不变。
  新增 `/left_camera/image` 为 `1280×1024 bgr8`，保留原压缩图像；不再要求用户另启图像适配器。
- 转换环境在 `/home/crl/.cache/ual-bag-convert-venv`，只新增 rosbags 0.11.9 及其依赖，
  复用系统 OpenCV/NumPy；没有修改系统 Python 或原项目环境。无 ensurepip 时使用无 pip venv，
  由系统 pip 的 `--python` 选项向目标环境安装，未请求管理员密码。
- 脚本：`scripts/convert_scene_water_bag.py`；已有输出会拒绝覆盖，图像解码失败即中止。
  四类真实 ROS 1 小样本 → ROS 2 原生解码、原 header 保留、拒绝覆盖检查均通过。
- 对完整输出重新用 Jazzy 原生消息解码检查五个话题的首尾消息，并核对 SQLite 每话题计数。
  全部 2797 张原始图像再次解码后，像素 SHA-256 与转换时一致：
  `e81bd0341029a3188c46f7c7825dccb934975dfff3ba92edb38679edc947f33b`。
- 新增根目录 `launch/play_scene_water.launch.py` 与 `rviz/scene_water_playback.rviz`。
  无需构建项目，source Jazzy 后可以直接启动；只播放和显示，不启动/下载分割模型。
  初次 GUI 测试因 VS Code Snap 的 GTK/GIO 插件混入 core20 libc 而失败，失败日志保留。
  已在 RViz 子进程局部清空 GTK_PATH/GIO_MODULE_DIR/GTK_MODULES，不修改系统库或全局环境。
- 修复后实际运行 6 秒回放：原生订阅收到 21 张图像、51 帧点云、1225 条 IMU、300 条里程计，
  校验无错误；ROS 图确认 RViz 已订阅图像，OpenGL 4.6 初始化正常、测试进程已干净退出。
  无人工图像质量验收，不能由可显示推断识别/标定正确。
- 证据在新 bag 目录中的 `conversion_report.json`、`playback_validation.json`、
  `playback_validation.log`（失败）及 `playback_validation_run02.log`（修复后）。
  `ros2 bag info`、新脚本 AST/RViz YAML、launch 参数解析、27 项回归和 `git diff --check` 通过。
- 本次未训练、下载模型、伪造时间/TF、清理原数据、提交或推送。
- 同日用户要求一键播放脚本：新增仓库 `scripts/play_scene_water.sh`，
  在数据目录提供可执行入口 `/home/crl/ws_ual/data/play.sh`。
  自动隔离旧 ROS 路径并加载 Jazzy，默认循环播放和启动 RViz，原样转发 launch 参数。
  Bash 语法、帮助/参数解析通过；从 `/tmp` 且继承模拟 Noetic/PYTHONPATH 的环境实际回放 0.5 秒，
  数据库只读打开、播放正常退出。支持 `loop:=false`、`rate:=0.5`、`start_offset:=45`、
  `start_rviz:=false`；用 Ctrl+C 停止 launch 及其子进程。未修改原始数据或全局 shell 配置。
- 同日按用户要求将整个数据目录从 `/home/crl/ws_ual/data` 移至本仓库 `/home/crl/UAL-planner/data`，
  包括四份原始文件、转换包及本地播放入口。同一文件系统内移动，目录与数据库 inode/大小保持不变。
  原路径保留指向新目录的软链接，兼容既有命令及报告中的历史路径；当前数据实际存放在仓库根目录。
  `.gitignore` 增加 `/data/`，整个目录均不提交、不上传；播放 launch 默认路径改为仓库内 `data/`。

## 2026-10-28：后续构建与融合接口验证

- 在本机 Jazzy 环境完成全部五包构建：`ins_seg`、`landing_evaluator`、`livox_avia_driver`、
  `auto_landing`、`pointcloud_voxels`。首次全量约 50 秒；复用本机已存在的 sam3.cpp 源码，
  核实其提交严格等于 CMake 固定的 `416186c501d060df7ca02989d49b38080f5f81f3`，未复用 Noetic 库。
  新二进制的依赖未发现 Noetic/Humble 路径或缺失库；三方源码存在格式警告但构建成功，未改三方源码。
- 修正 `ins_seg/package.xml` 的 curl rosdep 键为 `libcurl-dev`，解析仍为 `libcurl4-openssl-dev`。
  `rosdep check --from-paths src --ignore-src --rosdistro jazzy` 全部满足，无新增系统依赖安装。
- 为 `pointcloud_voxels` 接入 CTest：该阶段 27 个纯数值/传输测试和 7 个真实 ROS 2/DDS 测试，
  覆盖延迟掩码、掩码先于点云、掩码孔洞、空掩码清空、严格/近似标定开关、恰好四空面绿色、
  短片段回跳后重复采集时间可再次处理；包含原 Landing 测试的 colcon 汇总为 52 项。
- 9 个安装后根目录 launch 的 `--show-args` 解析通过；历史 `ual_planner/d1/avia/hil_sim`
  实际启动并发现对应的投影、实例图、Landing（HIL 另含重力适配器）节点。
  这些历史启动冒烟关闭了分割和 RViz，不代表历史投影几何或安全面精度已重验。
- 完整 bag 时间审计：2797 张图像全部能在 0.2 秒内找到点云，最近 header 时间差
  min/median/P95/max 为 `0/0/0/0.100248 s`；压缩/原图 header 完全一致。
  各传感器 header 相对记录时间均约 +4786 秒；这是共同时间基准偏移，未重写数据。
  点云/IMU 存在短时乱序，但图像与点云按 header 可同步，无需增大 sync_slop。
- 用户明确本机无内参，允许借代表海康相机参数、外参近似共点且镜头俯视。
  查阅海康厂家 MV-CA013-20GC 规格书（1280×1024、4.8 μm），没有把机身规格冒充实测 K：
  **假设 6 mm 镜头**推导 fx/fy=1250，主点取 640/512，畸变暂取零。
  外参按 Avia X 轴视线映射到光学 Z 轴，采用 `(-y,-z,x)`、零平移、零额外相对偏航。
  `config/scene_water_approximate.yaml` 独立保存这些假设；`calibration_confirmed` 保持 false，
  通过显式 `allow_approximate_calibration=true` 允许联调；默认严格标定门槛仍保留。
  来源及推导见 README；它不是本 bag 拍摄设备的正式标定。
- SAM3 缓存 SHA-256 为配置预期的 `5dafc790c8493319f542f575e718084f4e0a452fd7e483c64853d33ffe3f1889`。
  未重新下载模型。初次单帧验证发现大图像 Best Effort 传输未触发推理，随后为两种分割后端
  增加 `image_reliability` 参数；Scene_Water 的图像/点云订阅显式使用 Reliable，保留其他设备默认 Best Effort。
  单帧脚本还需识别算法具名订阅，不能把 RViz 已订阅点云误当算法已就绪；已加就绪等待和发送确认。
- 早期运行记录保留：第 45 秒图像曾输出 0 实例；第 180 秒图像输出 1 实例，但初期单帧点云就绪问题
  导致需诊断重发才能产生结果。`run03` 的 111.54 秒含诊断等待，不作网络推理耗时报告。
- 修复后 `run04` 独立运行，无人工补发：采集时间完全相同的真实图像/24096 点点云 →
  SAM3 1 个实例、124419 前景像素 → 1145 个选中点 → 645 体素（蓝337、绿154、红154）。
  原图/标注图均 1280×1024，原 header 和 avia_frame 保留；观测端到端结果延迟约 30.26 秒，
  C++ 自报该帧 total=30.159 秒（本机 i7-11800H，8 CPU 线程、Q4_0，
  单帧样本，不是总体性能统计或实时性结论）。
  RViz 的算法图像订阅正常，三节点 SIGINT 后干净退出。已处理 Python 清理阶段重复 SIGINT。
- 真实 bag bbox 入口也已输出非空点云/体素：第45秒起2秒采样收到8个非空结果，
  选中点数范围8160–24096，末次体素蓝806/绿95/红13；不是目标识别精度验收。
- 可复现实帧验证：source `install/jazzy/setup.bash` 后运行
  `python3 scripts/verify_scene_water_pipeline.py --report-dir data/<new-directory> --start-rviz --start-offset 180`。
  禁止自动下载模型，不覆盖已有证据。输出在 `data/validation_20261028.9IzGKn/`：
  `time_audit.json`、`bbox_runtime.json`、`legacy_launches.json`、`run04/single_frame.json` 及相应日志。
  全部数据/日志仍被 `/data/` 忽略；未提交、推送、训练或进行人工视觉质量验收。

## 2026-10-28：默认检测模型切换与双模型兼容

- 用户要求：把 master 算法的实例分割模型换成 visual-only 使用的目标检测模型，算法兼容两种模型，
  主要用 bbox 内点云特征。未整分支合并、未训练、未改 visual-only 历史。
- 新增 `ins_seg/msg/Detection.msg`、`DetectionResult.msg`；检测节点
  `src/Instance_Seg/scripts/yolo_detector.py` 使用 Ultralytics，支持 OBB 和水平检测两种头，
  输出原图四点多边形、类别、置信度，不生成掩码。
- 模型采用 visual-only 已验收的 YOLO11n-OBB / DOTA `ship`；权重 SHA-256
  `b62898ebf38940ca4df323863e45ee9d84a1a46d5d11ebdde529fb33aa9f3a32`（5,795,654 B），
  由 `scripts/fetch_yolo_detector_weights.py` 显式下载到
  `~/.cache/ual_planner/models/yolo11n-obb.pt`，不提交、不在节点内自动下载。
- `pointcloud_voxels` 新增 `selection_mode:=detection` 默认路径：
  `project_pixels` 复用原畸变/光学系变换，`points_in_polygon` 对 OBB 做点在多边形判断，
  只接受图像边界内的投影点；`largest` 按有有效点的最大多边形选择。`mask` 路径仍使用
  原 32SC1 精确像素；`cluster`/旧别名 `bbox` 保留原纯雷达 PCA 诊断。
- 为保证 Jazzy 兼容，检测节点在单独虚拟环境运行，launch 参数 `detector_python` 指定解释器；
  ROS 构建不依赖 PyTorch/Ultralytics。实测发现 OpenCV 5 与 Jazzy `cv_bridge` 不兼容
  （`KeyError: 16`），安装说明固定 NumPy 1.x、OpenCV 4.x；本机 `.venv-yolo` 复用系统 OpenCV 4.6。
- 配置：`config/detector.yaml`（权重/类别/阈值/imgsz/device/标注图），
  `config/pointcloud_voxels.yaml` 默认 `selection_mode: detection`，
  `config/scene_water.yaml` 为检测图像使用 Reliable，RViz 增加 `/ship/detections_image`。
- 验证：五包 Jazzy 构建通过；CTest **65 tests，0 errors，0 failures，0 skipped**，
  新增 4 项检测解码、5 项多边形/投影、3 项原生检测选点测试。
  真实转换包第 300 秒图像（1280×1024）与同采集时刻 24096 点点云：
  YOLO11n-OBB 输出 1 个 `ship`（置信度 0.284，OBB 四角）→ 925 个选中点 →
  466 个体素（蓝283/绿94/红89），`/ship/points` 时间戳与点云一致，检测标注图 1280×1024；
  CPU 预热后单帧推理约 0.04 秒，首次验证进程观测结果延迟约 1.0 秒。
  该帧检测框在图像边缘伸出，仍只选图像内的投影点；这是接口/数值验证，不是精度或实时性验收。
  证据在 `data/validation_detection_20261028/detection_frame.json`（本地忽略目录）。
- 未做：全 bag 识别率/漏检率、真值标定、跟踪、跨帧融合、安全面判定、训练或模型调参；
  检测框内的背景回波仍由点云本身承担，不能用 bbox 当实例轮廓真值。

## 2026-10-28：bbox 中心目标与局部 3D A* 路径可视化

- 用户要求以 bbox 中心为目标规划路径并在 RViz 展示；优先评估直接接 EGO-Planner，
  复杂则自写简单规划。已实现并完成真实单帧和 RViz 订阅验证。
- EGO-Planner 核对：本机 `/home/crl/ego-planner` 为 ROS 1 catkin（roscpp、
  quadrotor_msgs、catkin devel），不能直接链接 Jazzy；ROS 2 版本需 ego-planner-swarm 的
  `ros2_version` 分支，当前工作区没有该源码。完整接入还需局部地图、世界系里程计/TF、
  FSM、目标接口和独立构建，故本轮不直接接 EGO-Planner。
- 新增 `pointcloud_voxels/planning.py` 纯 NumPy 3D A*、`planner.py` ROS 2 节点和
  `scripts/path_planner_node`。目标 = `/ship/points` 的三维 AABB 中心
  （可选 `target_offset`）；起点默认 LiDAR 原点 `[0,0,0]`（可选 `start_point`），
  没有接入里程计闭环或世界坐标 TF。
- 障碍来自 `/livox/lidar/points`，目标 AABB 加边距后排除；26 邻域 A*，默认 1 m 网格、
  0.5 m 膨胀、5 m/3 m 规划边距，随后做视线可达折线简化。输出
  `/ship/target`、`/ship/path`、`/ship/path_markers`、
  `/ship/path_status`，frame 保持原始点云 frame。
- 失败时发布空 Path + 目标/bbox Marker 和明确状态（空目标、无障碍点云、无路径、异常），
  不伪造不可达路径。参数位于 [config/pointcloud_voxels.yaml](../config/pointcloud_voxels.yaml)
  的 `path_planner` 段；Scene_Water 的 Reliable QoS 覆盖在
  [config/scene_water.yaml](../config/scene_water.yaml)。
- RViz [ship_voxels.rviz](../rviz/ship_voxels.rviz) 增加 `Planned path`
  （`/ship/path`）和 `Path target and bbox`（`/ship/path_markers`）：
  绿起点、红目标、黄色目标 AABB、青色路径线、状态文字。
- 验证：五包构建、CTest **77 tests，0 errors，0 failures，0 skipped**；新增 6 项纯规划测试、
  4 项原生 ROS 规划测试。真实第300秒单帧：目标 `(49.92,7.94,1.93)`、
  2 个路径点、路径长 50.59 m、规划约 141 ms；`/ship/path` 与目标/点云时间戳一致，
  路径终点等于目标，RViz 报告 `rviz_path_subscription=true`。
  该帧路径近似直线，只能证明接口和可视化，不能证明复杂障碍下避障效果。
- 证据：`data/validation_path_20261028/detection_frame.json`（本地忽略目录）。
- 未训练、未改 EGO-Planner、未接飞控、未发布控制指令；路径是传感器系局部演示。

## 2026-10-28：点云 3D bbox 生成与持续维护

- 用户先要求 bbox 持续化，随后明确“不是图像中的而是点云中的”，并指向
  `~/ws_crl` 的 bbox 生成逻辑。本机不存在 `~/ws_crl`；采用可用的同类实现
  `/home/crl/ws_ual/src/pointcloud_cluster_bbox/scripts/pointcloud_cluster_bbox.py`。
- 新增 `pointcloud_voxels/bbox.py`：
  选中 3D 点投影到 `xy/xz/yz`（默认 `yz`）→ 体素化 + 欧氏连通聚类 →
  主簇 PCA 有向矩形 → 沿法向取点云 min/max → 8 角点 3D OBB、中心、长/宽/高、点数。
  无有效簇时可按 `bbox_fallback_to_aabb` 回退整体 AABB。
- 新增 `PointCloudBBoxTracker`：在 LiDAR 3D 坐标中按同类别 + 3D AABB IoU/中心距离
  匹配；只有新置信度严格更高才替换 bbox 几何/置信度，否则保留旧 3D bbox；连续
  `bbox_timeout_sec` 未匹配才过期。稳定 `track_id`，最多 `max_bboxes` 个。
- `mask_voxels` 在检测选点后按实例生成 bbox，发布
  `/ship/bboxes`（8 角点 OBB + 尺寸/置信度/点数标签）和
  `/ship/bbox_target`（当前选中观测对应 bbox 中心；无当前观测时按体积/置信度选择）。
  图像检测 bbox 只作为 ROI；
  `yolo_detector.py` 已恢复为每帧原始检测，不再在图像坐标维护 bbox。
- `ship_path_planner` 优先使用与点云同时间戳的 `/ship/bbox_target`，
  无匹配时回退选中点云 AABB 中心；因此路径目标 = 点云 3D bbox 中心。
- 配置在 [config/pointcloud_voxels.yaml](../config/pointcloud_voxels.yaml) 的
  `mask_voxels` 段：`bbox_projection_plane`、`bbox_voxel_size`、
  `bbox_cluster_tolerance`、`bbox_min_cluster_points`、`bbox_min_box_size`、
  `bbox_min_height`、`bbox_confidence_margin`、`bbox_match_iou`、
  `bbox_match_center_ratio`、`bbox_timeout_sec`、`max_bboxes`、
  `bbox_target_selection`。
- RViz [ship_voxels.rviz](../rviz/ship_voxels.rviz) 增加 `Point-cloud bbox (maintained)`
  显示 `/ship/bboxes`；路径目标显示点云 bbox 中心。
- 验证：CTest **86 tests，0 errors，0 failures，0 skipped**；新增 7 项 bbox 生成/维护
  纯测试和原生检测 bbox 输出断言。真实第300秒单帧：点云 3D bbox 目标
  `(49.9235,8.1008,2.4482)`，路径 2 点、50.64 m；
  `/ship/bbox_target` 与点云同时戳，路径终点与 bbox 中心一致，
  `/ship/bboxes` 含 `ship_point_bbox`/`ship_point_bbox_labels`。
  该目标与旧的点云 AABB 中心 `(49.92,7.94,1.93)` 不同，证明路径已改用维护的点云 bbox。
  证据：`data/validation_point_bbox_20261028/detection_frame.json`（本地忽略目录）。
- 边界：3D bbox 是当前可见点云主簇的拟合，不是完整船体真值；遮挡、背景点和投影面选择
  会影响中心/尺寸。持续化没有运动补偿，不是跟踪、世界坐标、安全降落面或 EGO-Planner。
- 未训练、未接飞控、未发布控制指令，未改掩码路径。


## 未验证与下一步

1. 构建、测试、安装启动和近似几何下的单帧检测/掩码/局部路径链路已完成；尚未给出全 bag
   识别率、漏检率、测距误差或路径安全性，没有真值不能作精度验收。YOLO11n-OBB 单帧 CPU
   预热后约 0.04 秒，但未测全 bag/端到端实时性；SAM3 Q4_0 的约 30 秒/帧仍不满足逐帧实时。
   当前 A* 单帧约 0.1 秒，未做高速动态障碍、长距离或复杂海况测试。
2. 正式几何仍需实际相机/镜头型号、内参畸变和相对偏航/平移标定。
   用户指定的共点俯视及代表规格可供近似联调，不能取代正式标定；不要直接把 calibration_confirmed 改为 true。
3. 已核查图像/点云 header 可同步；若后续接世界系 TF，仍需协调 bag clock 与 header 的公共时基。
   对小于30秒的循环片段，显式降低 timestamp_reset_threshold_sec；0.5秒门槛的回跳重处理已通过 ROS 测试。
4. 本机已有上述 ROS 2 转换包，算法使用它时关闭图像/Livox 适配器：
   `start_image_adapter:=false start_livox_adapter:=false`，避免重复发布或消息类型冲突。
   两份 5.2 GB 损坏备份仍保留，本次未尝试全文件重新修复，不宣称其中额外记录已恢复。
5. 本次按检测框或真实像素掩码选择雷达回波，不解决框内背景/遮挡深度歧义、跨帧关联、
   跟踪、融合、测距真值或安全降落判定。检测框不是实例轮廓真值，稀疏与斜面造成的空面仍需数据验收。
   路径规划以 LiDAR 原点和维护的点云 3D bbox 中心为输入，未接 EGO-Planner、里程计闭环、
   飞控或世界系 TF；路径终点位于目标物体 bbox 内，不能直接当作安全降落点。3D bbox 持续化
   没有运动补偿，目标/相机移动时只有更高置信度观测才能更新；超时参数不是永久记忆。
   默认 Marker 显示最近一次有效采集，不是实时跟踪的船体位置。

## 分支取用

本次只改 master 工作区。`visual-only` 和 `learning-methods` 的提交历史保留；
纯视觉资源/试验结论可用 `git show visual-only:docs/PROJECT_STATE.md` 只读查阅。
后续提交/推送需要用户明确要求，不能把普通集成完成视为已同步到远端。