# 相近论文

核查：2026-10-15；分支：`visual-only`。以下均为**文献候选，未复现、未接 ROS、未验证本机场景精度**。
代码公开不等于完整复现或具备明确开源许可；“未找到代码”仅指本轮检索结果。

## 严格相似的判据

已核对本机开题报告《融合视觉与点云信息的无人机非合作目标自主降落导航方法》第三章：
目标不仅是检测或跟随船只，还包括实例关联、运动与形状区分、远距整体到近距局部的模型维护、
目标上的安全降落区域选择及轨迹规划。控制器设计不作为本课题的主要研究内容。
本分支已改走纯视觉路线，不能将原报告的视觉＋点云方案当作本分支现有实现。

本轮重点筛选：移动船舶（车辆作为邻近场景）、无专设标记/主动信标/目标状态通信、
机载感知、未知表面上的安全降落区域。另核查实例掩膜与目标几何维护是否实际具备。
GNSS 不可用、无目标通信、无人工标记、无已知 CAD 是不同条件，不能相互替代。

结论：找到多项环节相近工作，尚未找到同时满足上述任务要求且提供完整可复现代码的系统；
不据此断言不存在相关工作，也不据此保证论文创新性。

## 优先阅读

### P01：轻量视觉路线最接近，暂未找到作者代码

**Vision-Based Marker-Less Landing of an Unmanned Aerial System on Moving Ground Vehicle**
— Blake Krpec、John Valasek、Stephen Nogar，Journal of Aerospace Information Systems，2024。
[DOI](https://doi.org/10.2514/1.I011282)；[作者实验室介绍](https://vscl.tamu.edu/2024/05/05/krpec-and-valasek-publish-vision-based-marker-less-landing-of-a-uas-on-moving-ground-vehicle-in-journal-of-aerospace-information-systems/)。

- 神经网络目标检测＋相机内参定位移动地面车辆，再生成降落轨迹；有仿真和实物直线/转弯降落实验，强调机载低重量与算力约束。
- 对当前“轻量模型＋传统几何”的路线最值得先读，但对象不是船舶，公开介绍未证实未知甲板安全面搜索或实例掩膜输出。
- 论文出版页访问受限，本轮依据作者实验室公开介绍，**尚未核实尺度来源、目标尺寸先验与完整定位公式**；不能宣称就是本项目拟用的面积变化测距。
- 本轮未找到论文对应作者源码或权重。下一步应先取得可访问全文并核查尺度假设，不直接按摘要实现。

### P02：无标记移动平台，作者有公开代码，但链路不完整

**UAV Landing on General Moving Platforms Without Markers** — Sagar Dalai 等；2022 年会议，2025 年论文集出版。
[论文记录](https://pure.ul.ie/en/publications/uav-landing-ongeneral-moving-platforms-without-markers/)；
[作者仓库](https://github.com/Manaro-Alpha/Drone-Landing-on-an-unamrked-moving-platform)。

- 论文描述 RGB 图像轮廓/光流检测跟踪、视觉伺服和 PID 下降；与不预设人工标记的移动车辆降落相近。
- 已只读检查公开仓库树、13 个 Python 文件及内存读取的 `src.zip` 文件清单：主要是控制/轨迹脚本，未定位到完整图像检测与光流处理链路。
- [`PID3.py`](https://github.com/Manaro-Alpha/Drone-Landing-on-an-unamrked-moving-platform/blob/main/landing/src/PID3.py)
  将 `/husky_velocity_controller/odom` 的位姿和速度作为目标状态；`PID2.1.py` 也订阅该话题。
  这只能证明这些公开脚本依赖目标里程计，不能反推论文所有实验都这样，也不能将仓库视作完整纯视觉非合作系统。
- 使用 ROS 1 `rospy`/catkin，不是本项目 Jazzy 即用包；未见仓库 LICENSE，`package.xml` 许可仍为 TODO。
  状态：**公开参考代码，复现链路及复用许可待核实**，不直接迁入。

### P03：海上平台场景接近，但不满足本项目严格无标记定义

**Vision-Based Agile Landing on Turbulent Waters** — Dimosthenis Angelis、Leonard Bauersfeld、Davide Scaramuzza、Evangelos Boukas，2026 预印本。
[论文](https://arxiv.org/abs/2605.23717)；[已核查 v3 全文](https://arxiv.org/html/2605.23717v3)。

- 平台区域检测＋A-KAZE/SURF 稀疏特征＋强化学习策略，输出姿态/推力；有 Stewart 平台和水池浮台实物实验。
- 实验含 AprilTag 分支；随机特征分支仍在降落垫附加随机图案，**不能视作已验证自然船舶甲板的无改造降落**。
- 实机机载 Jetson Orin Nano/PX4/ROS 2；自身状态估计用 IMU＋GNSS，不是整个系统严格 RGB-only，也不是 GNSS-denied 验证。
- 重点是感知到控制策略，不输出本项目所需整船实例掩膜/未知甲板三维安全面；水池验证不等于大型移动船舶海试。
- 本轮未找到论文对应公开作者仓库/策略权重。仅列海上运动及轻量特征接口参考，不启动训练或替换算法。

## 第三章相关环节参考（不是严格同题）

| 编号 / 论文 | 对应环节 | 关键差异与代码状态 |
| --- | --- | --- |
| P04 [Aircraft Carrier Pose Tracking Based on Adaptive Region in Visual Landing](https://www.mdpi.com/2504-446X/6/7/182)，Drones 2022 | 跨尺度轮廓、船体 6D 位姿连续跟踪 | **已知船舶 CAD＋给定初始位姿**；用合成与缩比物理图像序列验证，不是未知船体在线重建或垂直降落实机闭环；未找到对应作者代码 |
| P05 [SafeLand: Safe Autonomous Landing in Unknown Environments with Bayesian Semantic Mapping](https://arxiv.org/abs/2603.17430)，2026，ECCV DriveX Workshop | 未知环境语义地图、安全区域持续检查 | 相机＋轻量高度传感器，动态障碍不等于移动降落载体；[作者仓库](https://github.com/markus-42/SafeLand)明确**只公开分割模块，不公开完整降落链路** |
| P06 [Robust Autonomous Landing of UAV in Non-Cooperative Environments based on Dynamic Time Camera-LiDAR Fusion](https://arxiv.org/abs/2011.13761)，2020 预印本；同作者相关期刊工作 [comprehensive terrain understanding](https://doi.org/10.1007/s11432-021-3429-1)，2022 | 原报告的视觉/点云融合、深度补全、坡度/粗糙度/安全面积评价 | 双目＋LiDAR，主要是未知地面选点，不是运动船体上的实例建模；不能作为纯视觉方案，未找到对应完整作者代码 |

P05 的额外限制：公开模型为 SegFormer MiT-B3（约 47.3M 参数），输出**语义**而非实例分割；
仓库 `boat` 类 IoU 标为 n/a，不能据此声称船舶能力已验证；许可 CC BY-NC-SA 4.0。
上述信息来自[作者 README](https://github.com/markus-42/SafeLand#2-ros2-package)，本机未下载其权重、安装或测试 Jazzy。

## 对本项目的使用方式

- 当前视觉路线先精读 P01 的目标定位/尺度假设；P02 仅做公开代码对照，先补证据，不直接集成。
- 原第三章的远近尺度连续性可对照 P04，安全区域评价可对照 P05/P06，但其先验和静态地面假设不能直接照搬到运动船体。
- 目前没有证据表明上述工作已提供“整船实例掩膜→未知甲板安全面→移动目标降落”的完整开源方案。
  检测框、语义区域、整船表面和可降落面是不同输出。
- 本轮只检索/只读核查，不下载模型或整套数据，不训练、不修改 ROS 默认后端或控制/测距算法；未提交/推送。
