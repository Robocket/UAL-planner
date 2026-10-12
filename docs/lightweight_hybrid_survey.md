# 船舶场景：轻量模型 + 传统视觉几何调研

更新：2026-10-12；分支：`visual-only`。已完成直接小模型及自动检测框补掩膜试验，未实现跟踪或几何算法。

## 结论

优先验证 **YOLO11n-seg + KLT 稀疏光流 + RANSAC 几何筛选**。
网络只承担船舶 bounding box / 实例掩膜；传统算法承担跨帧关联、局部运动与有条件的稀疏几何。
YOLO26n-seg 作更小的对照，RTMDet-Ins-tiny 作另一套实现的备选。
这是面向本项目的工程建议，不是已验证的海事实时测距方案或现成开源降落系统。

本机 T600 Laptop / 4 GB 不再优先加载大型稠密重建网络。
上一轮 MASt3R 在 Scene_Water 384/512 上约 3.24/5.41 秒每图像对，
含 GPU 推理及回传 CPU、未同时运行实例分割；仅保留为离线参考。
第一阶段先验证二维实例、跟踪和可观测的局部几何。
**稀疏点/平面不是稠密船体重建，整船框也不是已确认安全的降落面。**

## 1. 轻量实例分割候选

| 候选 | 官方参数规模 | 直接输出 | 场景限制与优先级 |
| --- | --- | --- | --- |
| YOLO11n-seg | 约 2.9M | COCO 类别框、实例掩膜、分数 | 第一基线；不需文本编码器；俯视船舶、舱面与遮挡需本地验证 |
| YOLO26n-seg | 官方融合推理统计约 2.7M | COCO 类别框、实例掩膜；端到端 NMS-free 推理 | 较新轻量对照；完整训练 checkpoint 参数数可能更高 |
| RTMDet-Ins-tiny | 约 5.6M | 框、实例掩膜 | 次选；MMCV/导出部署依赖单独核验，不默认兼容本机 Python 3.12 |

依据：[YOLO11 官方原始模型表](https://github.com/ultralytics/ultralytics/blob/main/docs/en/models/yolo11.md)、
[YOLO26 分割文档](https://docs.ultralytics.com/tasks/segment/)、
[RTMDet 官方模型表](https://github.com/open-mmlab/mmdetection/tree/main/configs/rtmdet)。
不使用作者在 T4、3090 等显卡上的时延推算 T600 帧率；权重小也不等于运行显存就是权重大小。
缩放、mask 解码、CPU/GPU 复制及可视化都应进入端到端测试。

COCO 类别含 `boat`（Ultralytics 零起始编号 8），也含车辆，但没有独立的 `ship_deck` 或
`landing_platform`。通用 `boat` 不等于已训练好大型货船俯视分割；先验证公开权重，不自行训练。
[官方 COCO 类别定义](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco.yaml)

Ultralytics 提供 AGPL-3.0 / Enterprise 许可选项，MMDetection 代码为 Apache-2.0。
权重和数据条款需分别核对，不把代码许可当作对所有数据的授权。
[YOLO 许可说明](https://docs.ultralytics.com/models/yolo11/)、
[MMDetection LICENSE](https://github.com/open-mmlab/mmdetection/blob/main/LICENSE)

不优先继续 MobileSAM / FastSAM：提示式或类别无关分割还需额外目标选择；
当前船舶实例任务更适合先验证闭集小模型。这是项目判断，不是通用精度排名。

## 2. 传统模块如何组合

| 模块 | 输入 → 输出 | 条件 / 不能解决什么 |
| --- | --- | --- |
| Shi–Tomasi / ORB + KLT | 船内纹理点、相邻图像 → 2D 点轨迹 | 避开波纹、反光和轮廓；光流不直接输出米制距离 |
| RANSAC 仿射 / 单应性 | 同一实例点对应 → 局部运动、内点 | 不能将不同高度船体共用一个甲板平面；单应性本身不是距离 |
| 多视图三角化 | 标定、相机位姿、同一物理点观测 → 3D 点 | 已知尺度、足够视差、目标近似静止；纯旋转/小基线需拒绝 |
| 平面拟合 / 滑窗优化 | 有效 3D 点 → 平面、稳定稀疏几何 | 无效深度不能靠拟合修复，不恢复不可见表面或微小障碍 |
| KF / EKF | 可观测状态与测量 → 平滑、预测、不确定性 | 不凭空补出尺度；错误运动模型可能使结果平滑但不准确 |

这是候选组合，尚未实现。[OpenCV KLT](https://docs.opencv.org/4.x/dc/d6b/group__video__track.html)、
[OpenCV 几何接口](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)

建议网络在关键帧更新真实掩膜，帧间用点轨迹维护实例位置；验证局部平面模型后才传播掩膜。
传播结果需标为预测，携带原分割时间和数据年龄，不能冒充该帧新推理结果。
遮挡、前后向光流不一致、内点不足或重投影误差过大时重新分割，不继续发布过期框。
执行频率、特征数、阈值、最大预测年龄都应放在根目录 `config/`，不预设固定值为实测最佳值。

## 3. 距离、尺度和运动船只

停泊船可作为第一阶段较可控的对象：短窗口近似世界静止，配合米制相机位姿、正式标定和足够视差，
尝试三角化。仍须检查摇摆、物理点关联及重投影误差，不能把“停泊”等同于完全静止。
纯单目 `findEssentialMat/recoverPose` 的平移尺度不定；没有尺度来源，就只报告局部几何或相对变化。
[OpenCV recoverPose 尺度说明](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)

自由运动的船舶不能直接套用静态三角化。相机平移和船舶运动共同产生相对运动；
仅知道无人机里程计、内参和实例尺寸变化，在未知真实尺寸/任意船舶运动下不足以保证绝对距离可观测。
需另行研究运动约束、已知几何或相对速度等信息，验证可观测性。
分割、RANSAC 和 EKF 都不能单独消除歧义；本轮不添加固定船长、甲板高度或恒速假设。

尺寸变化可作为相对接近趋势或 TTC 候选，但要求同一刚性目标的投影尺度主要由接近运动变化。
转向、视角变化、遮挡、波动和 mask 抖动都会破坏条件，不能把面积变化直接当准确米制测距。
海上视觉伺服已有用目标平面特征/光流避免显式估计未知距离的研究，但展示的是仿真控制验证，
不是可直接运行的非合作货船开源感知库，本轮不实现其降落控制。
[海上移动平台视觉控制论文](https://arxiv.org/abs/2404.11336)

PnP 需要已知 3D–2D 对应与目标几何，不能只给未知船舶 bbox 就估计 6D 位姿。
TNN-MO 的船基无人机实验可参考，但针对特定船舶，不作为任意船只的轻量通用方案。
[TNN-MO 作者代码](https://github.com/fdcl-gwu/TNN-MO)

## 4. 现成开源项目的取舍

| 项目 | 可借鉴部分 | 与当前场景的边界 |
| --- | --- | --- |
| ORB-SLAM3 | 传统单目跟踪、稀疏地图与优化，不需大型几何网络 | 不提供实例分割；纯单目尺度不可靠；动态船/海面不能都当静态地图；不是现成 Jazzy 包装 |
| DSO | 直接法稀疏里程计与滑窗优化 | 不是稠密船体重建；需考虑曝光、光度标定、反光及动态目标；不优先于 KLT 基线 |
| XFeat | 学习式局部特征/匹配，接 RANSAC 与传统几何 | 不提供实例/深度；作为 KLT/ORB 跟丢后的关键帧对照，首轮不同时加载 |
| AirSLAM | 学习式点线前端 + 传统 SLAM 后端 | 论文采用双目/双目惯性建图；单目重定位不等于当前单相机在线建图；官方依赖含 TensorRT/Noetic |
| VINS-Fusion | 传统优化式相机/IMU 自定位 | 单相机模式需要 IMU，不是 RGB-only；不默认启用 `/livox/imu`；ROS 1 基线需独立适配 |

官方实现：[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)、[DSO](https://github.com/JakobEngel/dso)、
[XFeat](https://github.com/verlab/accelerated_features)、[AirSLAM](https://github.com/sair-lab/AirSLAM)、
[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)。
AirSLAM 的输入区别见 [作者论文](https://arxiv.org/html/2408.03520v2)。
场景风险和迁移成本为本项目工程评估，不是作者声称已支持本场景。

已有 `/fsdk/aircraft_state` 时，先核验它，而不是为了测距再移入完整 SLAM。
若来源含雷达，应明确为“视觉目标感知 + 外部位姿辅助”，不能宣称严格纯视觉定位。
RGB-only 与外部位姿辅助实验应分别记录，不能混用尺度/精度统计。

## 5. 匹配数据与补充模型

- **Scene_Water**：首个本地验证对象。按停泊/运动、近远、小目标、截断及曝光分组，当前无船舶 mask/几何真值。
- **SeaDronesSee**：海上 UAV 检测和单/多目标跟踪参考；公开任务不能自动当甲板实例掩膜或米制深度真值。
  [作者仓库](https://github.com/Ben93kie/SeaDronesSee)
- **MTReD**：大型船舶航拍几何场景对照，无真实三维真值，不能替代距离误差验证。
  [作者仓库](https://github.com/RuiYiYong/MTReD)
- **Mobile WaSR-T**：有 MaSTr1325/1478 海事预训练权重，水/天/障碍语义分割可作水面过滤候选，
  但不是船舶实例分割。作者 Jetson Nano、256×192 约 13 FPS 不能套到本机或俯视场景，首轮不再叠加该网络。
  [作者 Mobile 配置](https://github.com/lojzezust/WaSR-T)

## 6. 验证顺序与当前进展

1. 只测 YOLO11n-seg 的 `boat` bbox/mask，320/480/640 作对照。不先训练、量化或改 ROS；
   **已完成**：本机推理中位数约 6.1/7.6/11.2 ms，但默认阈值下 43 张本地图像均无 `boat` 输出。
   降低阈值只将车辆误认成船；公开斜视货船对照可识别，当前权重不适合直接进入下一阶段。
   详见 [实测报告](yolo_scene_trial.md)。没有标注真值，不报告 mAP；没有有效船舶序列，不评估边界抖动。
   下一轮先筛选有公开海事/俯视训练权重的轻量实例模型，再重复相同场景测试；不自动启动本地训练。
   **2026-10-12 补充**：完成 [公开权重核查](maritime_weights_audit.md)，用户已同意自动检测框补掩膜。
   海事 YOLO11n + MobileSAM 实测中位数约 145 ms、GrabCut 约 681 ms，仍有局部船体框和车辆误检。
   [组合结果](ship_hybrid_trial.md) 尚不支持直接进入测距；先解决整船定位和语义误检。
2. 加 KLT/ORB 对照，检查轨迹连续性、跟丢重检和预测年龄。网络推理频率与跟踪频率分开计量。
   **尚未执行**；需先通过分割场景验证，不能在错误实例上叠加跟踪。
3. 核查标定、相机位姿来源/尺度/时间轴，通过后才对停泊船短窗口做稀疏几何。
   fx/fy=385.45 仍是历史占位值；上一轮抽样图像 frame_id 为空，header 与 bag 时间相差约 80 分钟。
   世界坐标必须用真实变换，禁止只改 frame_id。
4. 单独验证运动船距离可观测性；不满足条件只输出二维跟踪、相对变化及失效状态。
5. 效果通过后比较 ONNX Runtime / TensorRT FP16。INT8 要有本场景校准和精度回归，不预设低比特更快。
   记录端到端耗时、峰值显存、CPU 负载、结果年龄，而非只报网络前向。
6. 再接 Jazzy/RViz；核心不导入 ROS API，Noetic 独立包装/launch，不复制两套几何算法。

本轮没有变更 `src/`、后端、话题或启动默认值。切回分支恢复原 SAM3 代码，
**不意味着推荐 SAM3/Q4_0 继续作为实时方案**；试验方向确认后再单独替换。

## 7. 保留上一轮工作

此前 `learning-methods` 的未提交代码和报告保存在 stash：`8b3231ef50c961121701588f1512e5363b7361ef`。
2026-10-12 已将这组更新提交到 `learning-methods`：`8f577bd`（MASt3R 场景验证及调研报告）。
原 stash 保留为备份，不再需要重复应用到已更新的分支。未推送；权重、数据、输出和 `.venv-mast3r`
继续保留在本机，未纳入提交。

仅在需要恢复到其他干净的历史工作树时，检查目标版本并保存当前改动后，可使用：

```bash
git stash apply 8b3231ef50c961121701588f1512e5363b7361ef
```

不要将该 stash 应用到 `visual-only`，也不要直接应用到已经包含 `8f577bd` 的工作树；
`apply` 保留 stash，但重复应用可能冲突或遇到未跟踪文件已存在。
