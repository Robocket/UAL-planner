# 项目状态与跨机交接

更新日期：2026-10-12（Asia/Shanghai）。维护分支：`visual-only`。
这是项目状态快照，不是完整聊天记录；开始工作先核查实际分支、提交和本机环境。

## 接手时先看

- 当前主线：轻量公开模型 + 传统算法的纯视觉船舶感知，保留实例掩膜。
- **组合链路已离线跑通，但整船识别未通过；跟踪、视觉测距和降落控制均未实现。**
- ROS 默认仍为 SAM3 Q4_0；不要因为 YOLO/MobileSAM 离线文件存在，就宣称线上已换模型。
- 下一个优先问题是整船定位、车辆误检和一船多框，不是继续叠加测距。
- 用户已允许自动海事检测框补掩膜的离线实验；没有授权本地训练、自动替换 ROS 后端或新增降落算法。

## 分支与版本

| 分支 | 用途 | 已有实现/状态 | 最近算法或试验提交 |
| --- | --- | --- | --- |
| `master` | 历史相机 + 雷达融合路线 | 保留原节点和量化模型相关实现；本轮未改、未重验 | `bbe2261` |
| `visual-only` | 当前轻量纯视觉路线 | 相机/SAM3 ROS 链路 + 独立 YOLO/GrabCut/MobileSAM 离线实验 | `02e9405` |
| `learning-methods` | 模型驱动重建实验 | MASt3R 两帧局部点图/点云，不是在线 SLAM；该分支当前没有 ROS 包 | `8f577bd` |

2026-10-12 核查时，三分支与本地 `origin/*` 跟踪引用一致，已有更新已显示在跟踪引用中；
未在本轮执行 fetch，这不是实时网络查询。本次交接文件新增和 README 更新尚未提交/推送。
以后确认同步状态应先核对 Git，不能照抄本段作为实时状态。

stash `8b3231ef50c961121701588f1512e5363b7361ef` 保留作备份，其内容已提交至 `learning-methods` 的 `8f577bd`。
**不要再次应用到已含该提交的分支，也不要应用到 `visual-only`。** stash 不随普通 push/pull 同步。

交接三文件当前只在 `visual-only` 新增，另外两个分支没有自动合并这些文件。
在其他分支可只读查看摘要：

```bash
git show visual-only:docs/PROJECT_STATE.md
git show visual-only:docs/DECISIONS.md
```

上面命令要等交接文件提交到 `visual-only` 后才可用；新电脑还需先同步该分支。
这些摘要仍标记为 `visual-only` 快照，不取代其他分支自己的 README/代码。

## 已完成的场景试验

| 试验 | 本机测量 | 判定与证据 |
| --- | --- | --- |
| COCO YOLO11n-seg，320/480/640 | 预测中位数 6.14/7.61/11.21 ms/帧 | 43 张本地图像默认阈值无 boat；低阈值出现车辆误判；公开货船对照可检出。见 [首轮报告](yolo_scene_trial.md) |
| SeaDronesSee YOLO11n detector，640 | 预测中位数 9.55 ms/帧 | 本地 40/43 张有 boat 框，但含误检和局部框，不是正确率；不能输出掩膜 |
| 海事 YOLO11n + MobileSAM | 处理中位数/P95 144.68/167.34 ms/帧 | 原图候选掩膜已导出；仍有车辆、局部船体和岸边误分割；保留为离线低频基线 |
| 海事 YOLO11n + GrabCut | 处理中位数/P95 680.69/2645.59 ms/帧 | 当前配置不推荐逐帧运行；不是已经优化至实时的传统方案 |
| MASt3R，Scene_Water 384/512 | 3.24/5.41 秒/图像对 | 仅离线几何参考，米制尺度和世界坐标未核验；不是实例分割或在线重建 |

轻量试验使用 43 张去重本地图像；组合/诊断另加 2 张 MTReD 对照。
所有时延均有明确报告口径，**不含解码、可视化导出或 ROS/RViz**；没有标注真值，不能给出 mAP/测距误差。
完整组合与资源取舍见 [组合报告](ship_hybrid_trial.md)、[公开权重核查](maritime_weights_audit.md)、
[混合方案调研](lightweight_hybrid_survey.md)。本轮未重跑性能实验。

MASt3R 文档在另一分支，只读查看：

```bash
git show learning-methods:docs/maritime_scene_trial.md
git show learning-methods:docs/learning_methods_survey.md
```

## 当前代码入口

- ROS：`src/Instance_Seg`（包名 `ins_seg`）、`src/UAL_Bringup`（包名 `ual_planner_bringup`）。
  默认 `seg_q4_node`，可显式选已有 `seg.py` 回退，不包含 YOLO ROS 后端。
- 离线提帧：[extract_yolo_frames.py](../scripts/extract_yolo_frames.py)，用 Jazzy 的系统 Python 读取 bag。
- 直接模型/定位诊断：[yolo_scene_trial.py](../scripts/yolo_scene_trial.py)，只有显式 `task: detect` 才做无掩膜检测。
- 自动框补掩膜：[ship_hybrid_trial.py](../scripts/ship_hybrid_trial.py)，不导入 ROS、不分配跨帧 ID。
- 参数：[YOLO 场景](../config/yolo_scene_trial.yaml)、[诊断](../config/yolo_scene_trial_diagnostic.yaml)、
  [海事定位](../config/seadronessee_localization_trial.yaml)、[组合分割](../config/ship_hybrid_trial.yaml)。
- ROS 输入预留与标定：[scene_water.yaml](../config/scene_water.yaml)；跨 ROS 边界见 [兼容约定](ros_compatibility.md)。

## 本机条件和不随 Git 同步的资源

已验证电脑为 `htfp-crl`，仓库 `/home/htfp/UAL-planner`；另一台电脑的硬件、路径和环境尚未核查。
本机 T600 Laptop / 标称 4 GB，PyTorch 可见容量约 3718 MiB；Python 3.12、Torch 2.5.1+cu124、
torchvision 0.20.1+cu124、Ultralytics 8.3.226，详细依赖见 [requirements](../scripts/requirements-yolo-trial.txt)。

- bag：`/home/htfp/data/Scene_Water_20260805_013651`，主图像 `/left_camera/image/compressed`，1280×1024。
- 离线输入：`datasets/scene_water_audit/manifest.json`、`datasets/scene_water_yolo_clips/manifest.json`、
  `datasets/mtred_ship/scene03_{0000,0061}.png`。前者是旧审计产物，短窗口提帧脚本不会生成它。
  没有旧审计集时只能另作样本统计，不能将 36 张窗口集冒称为本报告 43 张。
- YOLO/海事 YOLO/MobileSAM 权重：`checkpoints/`；URL 和 SHA-256 在配置中，推理可按配置下载并校验。
- MASt3R 源码在 `models/mast3r`，权重在 `checkpoints/`，不因切分支就重新下载。
- `.venv-yolo` 的 `.pth` 绝对路径只读引用 `.venv-mast3r` 的 site-packages，不是自包含环境。
  新电脑独立建环境的示例在首轮报告，未在另一台电脑验证；旧 `.venv-sam3` 未修改。
- 输出：`outputs/yolo11n_scene_water_fp32`、`outputs/yolo11n_scene_water_diagnostic`、
  `outputs/seadronessee_yolo11n_localization`、`outputs/ship_hybrid_trial_fp32`、
  `outputs/maritime_mast3r_fp16core`。失败产物也保留，不纳入 Git。
- 论文 `doc/` 和外部附件不随 Git 自动同步；涉及论文约束时需确认本机可访问文件。

## 未解决问题与下一步

1. 核心识别问题：整船框不完整、车辆误检、一船多框、局部/岸边误分割。先明确和验证目标区域，
   不把驾驶室自动当成新的降落目标，不用降低阈值替代场景适配。
2. 数据问题：没有 bbox/mask/距离真值。若需要建立人工评测集、训练或额外下载数据，应先取得用户同意。
3. 几何问题：fx/fy=385.45 是历史占位；压缩图像 frame_id 为空，header 与 bag 时间约差 80 分钟。
   里程计来源/尺度/坐标轴、相机标定和时间同步待核验；当前没有真实世界坐标测距结果。
4. 集成问题：Jazzy 有现有包装，Noetic 尚未适配。新核心与 ROS 包装分层；默认后端切换需另行要求。
5. 下一次优先验证定位与语义误检；通过后才讨论跟踪，时间/尺度通过后才实现几何。KLT/RANSAC 目前仅候选。

## 最近验证与维护记录

- 2026-10-12：轻量试验 10 项测试通过；此前 198 份二值掩膜尺寸/面积检查通过，非精度验证。
- `learning-methods` 提交前 3 项 MASt3R 辅助测试通过；不等于跨机或在线验证。
- 2026-10-12 本次交接：新增 `AGENTS.md`、本文件和 `DECISIONS.md`，README 增加入口。
  仅文档变更，未改算法、ROS、配置、默认模型或其他分支，未自动提交/推送。
  本轮重跑 10 项单元测试全部通过，交接/README 的本地链接与跨分支证据路径检查通过，格式检查通过。
- 后续每次实质工作补充：日期、实际分支/电脑、变更、证据/测试、失败/未验证项、下一步与同步状态。

## 换电脑时的最短流程

1. 离开前让当前任务更新本文件，检查 diff；按用户确认提交并推送当前分支。
2. 另一台先检查未提交改动，再同步并检出同一分支；不要覆盖该电脑独立改动，也不要将 stash 当远端备份。
3. 打开仓库根目录并开始新 Codex 会话，让它先读交接文件。
   官方指令发现通常在启动时完成；既有会话不能假定热加载了新 `AGENTS.md`。
   [OpenAI 官方说明](https://learn.chatgpt.com/docs/agent-configuration/agents-md)
4. 可直接发送：

   > 先阅读 AGENTS.md、docs/PROJECT_STATE.md、docs/DECISIONS.md，核查当前分支和本机环境。
   > 概括已完成和未验证事项，再处理我本次指定的任务；不要自动开始训练、测距或切换 ROS 后端。

5. Git 同步只带代码/配置/文档，不带聊天、权重、bag、忽略的输出或虚拟环境；缺什么就明确报缺项。
