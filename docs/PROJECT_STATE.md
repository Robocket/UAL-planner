# 项目状态与跨机交接

更新日期：2026-10-13（Asia/Shanghai）。维护分支：`visual-only`。
这是项目状态快照，不是完整聊天记录；开始工作先核查实际分支、提交和本机环境。

## 接手时先看

- 当前主线：轻量公开模型 + 传统算法的纯视觉船舶感知，保留实例掩膜。
- **历史离线链路曾跑通，但整船识别未通过；相关试验内容现已清理。跟踪、视觉测距和降落控制均未实现。**
- 当前保留 SAM3 Q4_0 与 Python 回退的 ROS 链路；默认模型和节点未替换。
- 下一个优先问题是整船定位、车辆误检和一船多框，不是继续叠加测距。
- 用户已允许自动海事检测框补掩膜的离线实验；没有授权本地训练、自动替换 ROS 后端或新增降落算法。
- 最新：2026-10-13 用户选择清理方案 1，已删除离线试验权重、MASt3R 第三方源码、脚本、配置、
  测试、报告及调研文档；先前全部输出也已清空。删除内容移入系统回收站，未清空前可恢复。
  下列数值仅为失败实验的历史摘要，不表示原始产物、相关代码或运行配置仍在工作区可用。

## 分支与版本

| 分支 | 用途 | 已有实现/状态 | 最近算法或试验提交 |
| --- | --- | --- | --- |
| `master` | 历史相机 + 雷达融合路线 | 保留原节点和量化模型相关实现；本轮未改、未重验 | `bbe2261` |
| `visual-only` | 当前轻量纯视觉路线 | 保留相机/SAM3 ROS 链路；离线实验内容已从工作区清理，尚未提交 | 历史试验 `02e9405` |
| `learning-methods` | 模型驱动重建实验 | MASt3R 两帧局部点图/点云，不是在线 SLAM；该分支当前没有 ROS 包 | `8f577bd` |

2026-10-13 核查时，三分支与本地 `origin/*` 跟踪引用一致；`visual-only` 已有交接提交 `1343483`。
未在本轮执行 fetch，这不是实时网络查询。本轮离线试验清理及文档更新尚未提交/推送。
`learning-methods` 的 Git 文件未变，但其共用本机 MASt3R 源码/权重已移入回收站，不能直接运行旧试验。
以后确认同步状态应先核对 Git，不能照抄本段作为实时状态。

stash `8b3231ef50c961121701588f1512e5363b7361ef` 保留作备份，其内容已提交至 `learning-methods` 的 `8f577bd`。
**不要再次应用到已含该提交的分支，也不要应用到 `visual-only`。** stash 不随普通 push/pull 同步。

交接三文件已在 `visual-only` 提交，另外两个分支没有自动合并这些文件。
在其他分支可只读查看摘要：

```bash
git show visual-only:docs/PROJECT_STATE.md
git show visual-only:docs/DECISIONS.md
```

本地上述命令已可用；新电脑还需先同步该分支，且只能读到已提交的版本。
这些摘要仍标记为 `visual-only` 快照，不取代其他分支自己的 README/代码。

## 已完成的场景试验

| 试验 | 本机测量 | 判定与证据 |
| --- | --- | --- |
| COCO YOLO11n-seg，320/480/640 | 预测中位数 6.14/7.61/11.21 ms/帧 | 43 张本地图像默认阈值无 boat；低阈值出现车辆误判；公开货船对照可检出 |
| SeaDronesSee YOLO11n detector，640 | 预测中位数 9.55 ms/帧 | 本地 40/43 张有 boat 框，但含误检和局部框，不是正确率；不能输出掩膜 |
| 海事 YOLO11n + MobileSAM | 处理中位数/P95 144.68/167.34 ms/帧 | 历史实验曾导出原图候选掩膜；仍有车辆、局部船体和岸边误分割；本机产物和代码已清理 |
| 海事 YOLO11n + GrabCut | 处理中位数/P95 680.69/2645.59 ms/帧 | 当前配置不推荐逐帧运行；不是已经优化至实时的传统方案 |
| MASt3R，Scene_Water 384/512 | 3.24/5.41 秒/图像对 | 仅离线几何参考，米制尺度和世界坐标未核验；不是实例分割或在线重建 |
| Airbus MobileNetV2-U-Net + 连通域 / 224 | 48.90 ms/帧 | 作者/公开对照可产生前景，本地近景整船漏检，仍有负例误分割 |
| 社区 iSAID SegFormer-B0 + 连通域 / 320、512 | 36.53/52.61 ms/帧 | 部分船头/驾驶室前景，车辆误判；非学习式实例头，训练来源未完全核验 |

轻量试验使用 43 张去重本地图像；组合/诊断另加 2 张 MTReD 对照。
所有时延均有明确报告口径，**不含解码、可视化导出或 ROS/RViz**；没有标注真值，不能给出 mAP/测距误差。
新数据集模型历史实测使用上述 45 张加作者 Airbus 样例 1，共 46 张；清理时未重跑任何模型。
离线报告和调研文档已删除，旧提交中的报告仍可只读查看，例如
`git show 02e9405:docs/ship_hybrid_trial.md`。新 `dataset_model_trials.md` 未提交，只在回收站保留，
不能用 Git 历史恢复。不要将历史测试数量或耗时当作清理后的重新验证。

MASt3R 文档在另一分支，只读查看：

```bash
git show learning-methods:docs/maritime_scene_trial.md
git show learning-methods:docs/learning_methods_survey.md
```

## 当前代码入口

- ROS：`src/Instance_Seg`（包名 `ins_seg`）、`src/UAL_Bringup`（包名 `ual_planner_bringup`）。
  默认 `seg_q4_node`，可显式选已有 `seg.py` 回退，不包含 YOLO ROS 后端。
- 构建：[build_jazzy.sh](../scripts/build_jazzy.sh)。`scripts/` 已无离线提帧/推理脚本或其依赖清单。
- 参数：[common.yaml](../config/common.yaml)、[camera.yaml](../config/camera.yaml)、
  [scene_water.yaml](../config/scene_water.yaml)；仅保留 ROS 共用/相机/Scene_Water 配置。
- RViz：`rviz/visual_only.rviz`、`rviz/scene_water.rviz`；跨 ROS 边界见 [兼容约定](ros_compatibility.md)。
- 离线试验测试文件及缓存已清理，`tests/` 当前为空，不能再宣称本分支现有 17 项测试。

## 本机条件和不随 Git 同步的资源

已验证电脑为 `htfp-crl`，仓库 `/home/htfp/UAL-planner`；另一台电脑的硬件、路径和环境尚未核查。
本机 T600 Laptop / 标称 4 GB，PyTorch 可见容量约 3718 MiB；Python 3.12、Torch 2.5.1+cu124、
torchvision 0.20.1+cu124、Ultralytics 8.3.226；这是历史实验环境记录，本轮未改依赖或进行 GPU 推理。

- bag：`/home/htfp/data/Scene_Water_20260805_013651`，主图像 `/left_camera/image/compressed`，1280×1024。
- 离线输入：`datasets/scene_water_audit/manifest.json`、`datasets/scene_water_yolo_clips/manifest.json`、
  `datasets/mtred_ship/scene03_{0000,0061}.png`。另外保留初期 `scene_water_trial` 和作者 `airbus_control` 样例。
  数据未删除，但原提帧脚本已清理；不能将 36 张窗口集冒称为历史 43 张去重评测集。
- 离线模型：`checkpoints/` 当前为空，8 个文件已移入回收站；`models/mast3r` 第三方源码已移入回收站，
  `models/` 当前为空。SAM3 配置与外部模型缓存未触碰，不自动重新下载已清理的实验模型。
- `.venv-yolo` 的 `.pth` 绝对路径只读引用 `.venv-mast3r` 的 site-packages，不是自包含环境。
  原部署报告已清理；这些虚拟环境未删除，但不能保证直接复制到另一台电脑可运行；旧 `.venv-sam3` 未修改。
- 新 `.venv-airbus` 同样只读复用基础 Torch，另装 Keras 3.11.3/h5py 3.14.0、Transformers 4.48.3；
  不安装 TensorFlow，不修改旧环境。Airbus H5、社区 SegFormer bin/config/processor 已清理，
  作者对照 `datasets/airbus_control/test_image_1.jpg` 保留，不随 Git 同步。
- 输出：本机 `outputs/` 当前为空。2026-10-13 按用户要求将全部 9 个实验目录移入系统回收站：
  `yolo11n_scene_water_fp32`、`yolo11n_scene_water_diagnostic`、`seadronessee_yolo11n_localization`、
  `ship_hybrid_trial_fp32`、`ship_hybrid_trial_fp32_incomplete_json`、`maritime_mast3r`、
  `maritime_mast3r_nocache`、`maritime_mast3r_fp16core`、`dataset_semantic_trial_20261013`。
  共 1876 个文件、约 338 MiB；包含失败记录，原路径不再存在，回收站未清空，可在被清空前恢复。
  本轮继续清理对应报告和配置，仅在交接文件保留简要失败结论；不因此重新运行模型。
- 论文 `doc/` 和外部附件不随 Git 自动同步；涉及论文约束时需确认本机可访问文件。

## 未解决问题与下一步

1. 核心识别问题：整船框不完整、车辆误检、一船多框、局部/岸边误分割。先明确和验证目标区域，
   不把驾驶室自动当成新的降落目标，不用降低阈值替代场景适配。
2. 数据问题：没有 bbox/mask/距离真值。若需要建立人工评测集、训练或额外下载数据，应先取得用户同意。
3. 几何问题：fx/fy=385.45 是历史占位；压缩图像 frame_id 为空，header 与 bag 时间约差 80 分钟。
   里程计来源/尺度/坐标轴、相机标定和时间同步待核验；当前没有真实世界坐标测距结果。
4. 集成问题：Jazzy 有现有包装，Noetic 尚未适配。新核心与 ROS 包装分层；默认后端切换需另行要求。
5. 下一次优先验证定位与语义误检；通过后才讨论跟踪，时间/尺度通过后才实现几何。KLT/RANSAC 目前仅候选。
6. 数据集候选进展：iSAID CAT Mask R-CNN 有约 219.7 MB 实例权重，尚未安装 MMDetection 环境或测试，
   后续可单独核查；ShipSG 官方仓库未找到 checkpoint、MariSat 的小模型列为检测、SISP 尚未发布可用权重，
   HRSID 是 SAR 不优先用于海康 RGB。这些仅为此前调研记录，报告已清理，不自动开始新试验。

## 最近验证与维护记录

- 2026-10-12：轻量试验 10 项测试通过；此前 198 份二值掩膜尺寸/面积检查通过，非精度验证。
- `learning-methods` 提交前 3 项 MASt3R 辅助测试通过；不等于跨机或在线验证。
- 2026-10-12 本次交接：新增 `AGENTS.md`、本文件和 `DECISIONS.md`，README 增加入口。
  仅文档变更，未改算法、ROS、配置、默认模型或其他分支，未自动提交/推送。
  本轮重跑 10 项单元测试全部通过，交接/README 的本地链接与跨分支证据路径检查通过，格式检查通过。
- 2026-10-13：交接文件已见于 `1343483`；按用户数据集列表完成两组新模型的离线试验，
  新增脚本/配置/兼容包装及模型可用性报告，17 项单元测试通过；新旧环境依赖检查无冲突。
  138 份原图前景及 158 份候选掩膜的尺寸/面积验证通过，语法、格式和文档链接检查通过。
  模型均未通过整船场景要求；未训练、未改 ROS 或其他分支，未自动提交/推送。
- 2026-10-13 输出清理：用户明确要求删除已有测试结果，已用系统回收站清理 `outputs/` 全部内容，
  保留空目录。数据集、原始 bag、模型权重、虚拟环境、算法代码、配置及历史 Markdown 报告未删除。
  已核查 `outputs/` 无剩余条目；仅更新本分支交接状态，未改算法、未重跑实验、未提交/推送。
- 2026-10-13 离线试验清理（用户确认方案 1）：进一步移入回收站 8 个模型文件（约 2.7 GiB）、
  `models/mast3r`（约 12 MiB）、20 个试验脚本/依赖清单/配置/测试/报告文件，以及
  `scripts/__pycache__`、`tests/__pycache__`。`checkpoints/`、`models/`、`outputs/`、`tests/` 保留为空目录。
  README 移除失效入口，决策追加 D10，历史失败摘要保留；SAM3/ROS、设备配置/RViz、数据、bag、虚拟环境、
  其他分支和 stash 未删除或改动。删除及交接改动未提交/推送。
  本轮验证为剩余 Python 语法、构建脚本语法、文档链接/删除范围及 `git diff --check`；
  离线测试已清理，unittest 发现 0 项，不算算法验证；未构建 ROS、未跑 GPU/在线链路。
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
