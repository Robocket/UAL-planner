# 项目状态与跨机交接

更新日期：2026-10-19（Asia/Shanghai）。维护分支：`visual-only`。
这是项目状态快照，不是完整聊天记录；开始工作先核查实际分支、提交和本机环境。

## 接手时先看

- 2026-10-19 集中提交：按用户要求，将 MSRS 诊断、CAT 实测、LiM 预检、MariSat 权重数值审计、
  候选筛选及其代码/配置/测试/交接纳入本次提交，并正常推送当前上游 `origin/visual-only`。
  提交前远端实时查询仍为 `db3bdec`，与本地 HEAD 一致；提交及推送的最终结果须以 Git 核验为准。
  下文各轮“未提交/推送”是历史执行快照，不表示这些变更仍需另行找回；忽略的模型、数据、输出和环境不纳入提交。
- 当前主线：轻量公开模型 + 传统算法的纯视觉船舶感知，保留实例掩膜。
- 2026-10-19 最新 MariSat 实测：用户下载的是 `models/MariSat/sam3_semantic_best/` 解压容器，
  已在不改源字节的情况下复原 `.pt`；**1118 个条目严格匹配并成功加载，但原始文本投影矩阵有 2 个 -inf**。
  普通 FP16/混合精度 GPU 预热均得到非有限掩膜，未完成有效 48 张测试；不是零检出或用户质量不合格。
  现脚本会先数值预检再拒绝异常权重，不清零/替换参数；需作者正常/可校验权重再测，详见 [统一报告](ship_detection_trial.md)。
- 2026-10-19 MariSat 前期来源核查：作者分享文件的预览元数据确认 `sam3_semantic_best.pt`，
  训练脚本基于完整 SAM3；未找到额外更大分割版本或另一 `sam3_maritime_best.pt` 的公开链接。
  当时仅列待下载/加载候选；随后用户提供文件的实测异常见上条，不替换现有 Q4_0，详见 [模型清单](待测试/models.md)。
- 2026-10-19 后续筛选（D17）：用户指定 **Airbus MaskFormer / Swin 淘汰**；
  **WaSR 无 IMU 版确认可自行训练，保留为异机语义分割训练候选**。
  已核对作者训练入口/损失/优化器、无 IMU 模型与可省略 IMU 的数据加载器；未运行训练、安装依赖或下载模型/训练集。
  默认三类语义掩膜不是单船实例，数据与环境仍待准备，详情见 [模型清单](待测试/models.md)。
- 2026-10-19 最新：CAT Mask R-CNN R50/iSAID 已在新 `.venv-cat` 完成 **48/48 张 GPU 推理**，
  生成 32 份原图实例 mask 和 96 张标注图；本地预测中位数/P95 为 **1055.92/1179.12 ms**，质量待用户验收。
  LiM-YOLO 已核实训练代码及 OBB 头，但本轮未找到公开训练权重，未进入图像推理，保留待权重/异机训练。
  用户已允许 CAT 独立环境安装，不允许本机训练；结果追加 [同一报告](ship_detection_trial.md)。
- 新门槛（D16）：有可获取训练实现，且原生输出 **OBB 或像素掩膜**，满足一种输出即可进入筛选。
  仅水平框模型与无法核实训练实现的资源已移出可执行候选，详情见 [模型清单](待测试/models.md)。
  淘汰是停止推进，不删除旧文件/历史，不把 OBB 或语义前景当实例掩膜；本轮未提交/推送。
- 2026-10-19：按用户要求测试 `models/MSRS/` 的两份 MSRS-DETR，并预检同目录两份 RT-DETR 基线。
  四份大小/SHA-256 均匹配作者 release；实际加载首先缺 `dill`，静态另确认缺作者自定义网络层。
  作者仓库当前仅 README，**未进入图像推理，不作质量不合格判断**。
  随后按 D16 的水平框排除规则淘汰，不再优先索取其配套代码，既有诊断保留。
  诊断已追加 [当前报告](ship_detection_trial.md)，本轮代码/配置/交接未提交、未推送，未改 ROS 默认模型。
- **历史离线链路曾跑通，但整船识别未通过；相关试验内容现已清理。跟踪、视觉测距和降落控制均未实现。**
- 2026-10-15 新增检测对照已完成：YOLO11n-OBB、YOLO-MOD 船舶 s/l，共 6 配置×48 张。
  **最新用户验收：YOLO11n-OBB 合格、YOLO-MOD s/l 不合格**；取代此前助手的统一未通过判断，见 D14。
  未测试补掩膜，未接 ROS；原始统计/历史观察保留，不重新看图或修改测量值。
  详见 [本轮报告](ship_detection_trial.md)，这些新输出/权重未被历史清理删除。
- 后续测试统一追加到 [当前报告](ship_detection_trial.md)，不另建逐轮报告，不主动人工视觉核查。
  仍执行代码、输出完整性/格式与性能检查，可保留可视化供用户查看。
- 交接提交 `fb7f4b4` 保存待测试资源/论文清单、Jazzy 单平台约定、检测脚本/配置/测试及验收记录，
  现已成功推送到 `origin/visual-only`。最新约定：**用户要求“提交”默认包含推送当前分支**（D15），
  明确要求仅本地时除外；普通维护仍不自动提交。模型、数据、输出和虚拟环境不随 Git 同步。
- 当前保留 SAM3 Q4_0 与 Python 回退的 ROS 链路；默认模型和节点未替换。
- 平台决定（2026-10-15）：后续开发、测试和部署只面向 ROS 2 Jazzy，取消 Noetic 迁移；见 D12。
- 当前阶段已接受 OBB，MOD 不推进；新增指定 CAT/LiM 离线测试，后续集成仍待明确，不自动复核 OBB 画面或叠加测距。
- 用户已允许自动海事检测框补掩膜的离线实验；没有授权本地训练、自动替换 ROS 后端或新增降落算法。
  最新补充：无合适公开模型时可考虑在其他机器训练，**本机不得训练**；不自动启动远程任务。
  2026-10-15 用户另行指定 n-OBB/MOD s/l 的下载和离线推理；2026-10-19 指定本地 MSRS-DETR、CAT/LiM 测试。
  CAT 仅新增独立 Python 3.10/Torch 2.0/MMCV 环境，不因此扩大为其他候选的下载/测试或任何训练。
- 清理记录：2026-10-13 用户选择清理方案 1，已删除离线试验权重、MASt3R 第三方源码、脚本、配置、
  测试、报告及调研文档；先前全部输出也已清空。删除内容移入系统回收站，未清空前可恢复。
  下列数值仅为失败实验的历史摘要，不表示原始产物、相关代码或运行配置仍在工作区可用。
- 上次交接核查（2026-10-15）：清理已包含在 `25336c1`，交接修订已提交为 `081fea9`。
  实验目录未恢复，默认 ROS 链路未变。
- 最新：俯视船舶资源已整理为 [待测试目录](待测试/README.md)（入口、模型、数据集及相近论文），见 D11。
  初轮仅核查公开页面、仓库树、文件元数据及 5 个下载入口的 HEAD；随后指定模型测试已单独完成，无训练。
- 非合作移动目标降落论文检索见 [相近论文](待测试/papers.md)：优先精读 Krpec 等 2024 无标记视觉移动车辆降落；
  Dalai 等作者仓库有代码但目标里程计依赖、视觉链路及许可待核实；SafeLand 仅公开语义分割模块。
  未找到同时覆盖未知移动船体/安全面且完整公开代码的方案，不据此宣称不存在或保证创新性。

## 分支与版本

| 分支 | 用途 | 已有实现/状态 | 最近算法或试验提交 |
| --- | --- | --- | --- |
| `master` | 历史相机 + 雷达融合路线 | 保留原节点和量化模型相关实现；本轮未改、未重验 | `bbe2261` |
| `visual-only` | 当前轻量纯视觉路线 | 保留相机/SAM3 ROS 链路；本次集中提交 MSRS 诊断、CAT 测试、LiM 预检、MariSat 审计与筛选 | 提交基线 `db3bdec`；新提交号以 `git log -1` 核验 |
| `learning-methods` | 模型驱动重建实验 | MASt3R 两帧局部点图/点云，不是在线 SLAM；该分支当前没有 ROS 包 | `8f577bd` |

2026-10-19 接手核查：`visual-only` HEAD 为 `db3bdec`，与本地 `origin/visual-only` 引用一致；
MSRS 任务接手时干净；CAT/LiM 接续时已有上述 MSRS 未提交变更，已保留。
本轮新增 CAT 脚本/配置/测试、清单/交接与 D16；未 fetch/push，不是实时远端查询，其他分支未改。

2026-10-19 用户要求集中提交时，实时查询 `origin` 的 `visual-only` 为
`db3bdec22dd75cd3fefd2eaac863e567cd11befd`，与提交前本地 HEAD 一致。
本次范围共 17 个代码/配置/测试/文档文件，包含后续 WaSR 筛选与 MariSat 审计；按 D15 提交并推送当前分支。
不推送其他分支/标签，不清理 stash，不强推；本节不预先承诺推送成功，最终核对远端引用与本地 HEAD。

2026-10-15 本轮调研接手核查时，三分支与本地 `origin/*` 跟踪引用一致；`visual-only` 为 `081fea9`，
清理位于 `25336c1`。未执行 fetch/push，这不是实时远端查询，不据此承诺另一台电脑已同步。
上述内容随后已集中提交为 `fb7f4b4`。用户补充“以后提交包含推送”后，已正常推送该提交，
远端返回 `081fea9..fb7f4b4 HEAD -> visual-only`；本次约定修订也按新规则提交并推送，最终状态以 Git 核验为准。
没有自动同步另一台电脑，需其主动拉取同一分支。下文各轮“未提交/推送”为当时快照，最新同步范围以本节为准。
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
| 2026-10-15 DOTA YOLO11n-OBB / 640、1024 | 预测调用中位数 14.34/22.28 ms/帧 | **用户验收合格**；43 张本地＋5 张对照，仅框、无掩膜、未接 ROS；历史观察见报告 |
| 2026-10-15 YOLO-MOD 船舶 s / 640、800 | 预测调用中位数 19.40/28.19 ms/帧 | **用户验收不合格**；两个 checkpoint 实际类别均为 ship/warship，保留失败记录 |
| 2026-10-15 YOLO-MOD 船舶 l / 640、800 | 预测调用中位数 62.77/99.43 ms/帧 | **用户验收不合格**；不因更多输出或历史局部改善继续优先推进 |
| 2026-10-19 CAT Mask R-CNN R50/iSAID | 预测调用中位数/P95 1055.92/1179.12 ms/帧 | 48 张完成，32 份候选实例 mask；未人工核查，质量待用户验收，不满足当前配置下逐帧实时需求 |
| 2026-10-19 LiM-YOLO | 无时延，未推理 | 训练实现及 OBB 头存在，缺公开训练 checkpoint；不是“无检出”或质量不合格 |

轻量试验使用 43 张去重本地图像；组合/诊断另加 2 张 MTReD 对照。
所有时延均有明确报告口径，**不含解码、可视化导出或 ROS/RViz**；没有标注真值，不能给出 mAP/测距误差。
新数据集模型历史实测使用上述 45 张加作者 Airbus 样例 1，共 46 张；清理时未重跑任何模型。
离线报告和调研文档已删除，旧提交中的报告仍可只读查看，例如
`git show 02e9405:docs/ship_hybrid_trial.md`。新 `dataset_model_trials.md` 未提交，只在回收站保留，
不能用 Git 历史恢复。不要将历史测试数量或耗时当作清理后的重新验证。
上表三项检测对照为 2026-10-15 独立新测试，CAT/LiM 是 2026-10-19 追加，证据在 [统一报告](ship_detection_trial.md)，
不是恢复旧实验；计时分为网络推理、预测调用、解码/绘制/两份 JPEG 写盘，无人工真值，不报告精度。

MASt3R 文档在另一分支，只读查看：

```bash
git show learning-methods:docs/maritime_scene_trial.md
git show learning-methods:docs/learning_methods_survey.md
```

## 当前代码入口

- ROS：`src/Instance_Seg`（包名 `ins_seg`）、`src/UAL_Bringup`（包名 `ual_planner_bringup`）。
  默认 `seg_q4_node`，可显式选已有 `seg.py` 回退，不包含 YOLO ROS 后端。
- 构建：[build_jazzy.sh](../scripts/build_jazzy.sh)。旧离线脚本仍已清理；本轮新增
  [指定权重下载](../scripts/fetch_ship_detection_weights.py)与[检测对照](../scripts/ship_detection_trial.py)，均与 ROS 独立。
- MSRS 加载诊断：[probe_msrs_weights.py](../scripts/probe_msrs_weights.py)及
  [msrs_weight_probe.yaml](../config/msrs_weight_probe.yaml)，只是加载预检；MSRS 当前已移出候选，不继续补推理后端。
- CAT 离线实例：[cat_mask_trial.py](../scripts/cat_mask_trial.py)、
  [cat_mask_trial.yaml](../config/cat_mask_trial.yaml)、[独立依赖](../config/cat_requirements.txt)，与 ROS 分离，拒绝覆盖输出。
- MariSat 离线：[marisat_sam3_trial.py](../scripts/marisat_sam3_trial.py)与
  [marisat_sam3_trial.yaml](../config/marisat_sam3_trial.yaml)，复原容器、严格参数校验及数值预检；当前权重异常，拒绝正式推理。
- 参数：[common.yaml](../config/common.yaml)、[camera.yaml](../config/camera.yaml)、
  [scene_water.yaml](../config/scene_water.yaml)；另有新离线 [ship_detection_trial.yaml](../config/ship_detection_trial.yaml)。
- RViz：`rviz/visual_only.rviz`、`rviz/scene_water.rviz`；平台约定见 [Jazzy 开发约定](jazzy_development.md)。
- 旧离线测试已清理；现有 [检测对照单元测试](../tests/test_ship_detection_trial.py)与
  [MSRS 预检测试](../tests/test_msrs_weight_probe.py)、[CAT 测试](../tests/test_cat_mask_trial.py)及
  [MariSat 测试](../tests/test_marisat_sam3_trial.py)合计 34 项在 `.venv-sam3` 与 `.venv-yolo` 通过；
  早先 27 项在 `.venv-yolo` 和 `.venv-cat` 通过，不是恢复旧 17 项测试。
  模型、数据、输出和环境仍不随 Git 同步；本机模型路径见下节，不能只同步代码就假定具备环境/权重。

## 本机条件和不随 Git 同步的资源

已验证电脑为 `htfp-crl`，仓库 `/home/htfp/UAL-planner`；另一台电脑的硬件、路径和环境尚未核查。
本机 T600 Laptop / 标称 4 GB，PyTorch 可见容量约 3718 MiB；Python 3.12、Torch 2.5.1+cu124、
Ultralytics 8.3.226；2026-10-15 完成 GPU 检测推理，2026-10-19 MSRS 仅作 CPU 加载、CAT 用新独立环境 GPU 推理；
原环境未改依赖，CAT 的 Python/Torch 版本另见下条。
i7-11800H，驱动 595.99.02。
torchvision 0.20.1+cu124 仅为历史记录，本轮未重新查询其版本。

- bag：`/home/htfp/data/Scene_Water_20260805_013651`，主图像 `/left_camera/image/compressed`，1280×1024。
- 离线输入：`datasets/scene_water_audit/manifest.json`、`datasets/scene_water_yolo_clips/manifest.json`、
  `datasets/mtred_ship/scene03_{0000,0061}.png`。另外保留初期 `scene_water_trial` 和作者 `airbus_control` 样例。
  数据未删除，但原提帧脚本已清理；不能将 36 张窗口集冒称为历史 43 张去重评测集。
- 离线模型：历史 8 个文件仍在回收站；本轮新下载到 `checkpoints/ship_detection_20261015/`：
  n-OBB 5.80 MB、MOD s 19.21 MB、MOD l 51.23 MB，保存来源、CRC/SHA-256 与说明文件。
  未下载完整 788 MB 包；`models/mast3r` 源码仍在回收站。
  2026-10-19 用户已放入 `models/MSRS/`：MSRS-DETR HRSC/ShipRS 各约 58 MB、RT-DETR 基线各约 81 MB；
  四份均匹配作者发布散列，但缺配套实现，未推理。本轮未下载任何额外权重。
  随后 CAT/LiM 任务另行取得 `models/CATNet/`（`64775d0`）、`models/LiM-YOLO/`（`8467bba`）源码，
  仅下载 CAT 指定 219.7 MB checkpoint 到 `checkpoints/cat_isaid_20261019/` 并核验作者 SHA-256；
  LiM 未找到训练权重，源码未安装到旧环境，不下载整套数据或用其他模型代替。
  用户随后提供 `models/MariSat/sam3_semantic_best/`（1124 文件/2,651,931,193 B）；
  复原容器 `checkpoints/marisat_sam3_20261019/sam3_semantic_best_repacked.pt`，2,652,082,661 B，
  源字节未变，逐文件 SHA 与 CRC 已记录，但没有作者原始压缩包/校验值，不确定异常产生环节。
  数值审计：1118 条，343 FP16/775 FP32；唯一异常是 language_backbone 的 text_projection 两个 -inf，
  原始 `data/433` 偏移 12/20 的字节 `00fc` 已独立核验，不是本次重封装/转半精度制造的。
  SAM3 配置/外部缓存未触碰，不自动重新下载旧实验权重。
- `.venv-yolo` 的 `.pth` 绝对路径只读引用 `.venv-mast3r` 的 site-packages，不是自包含环境。
  原部署报告已清理；这些虚拟环境未删除，但不能保证直接复制到另一台电脑可运行；旧 `.venv-sam3` 未改依赖。
- MariSat 复用 `.venv-sam3`：Python 3.12.3、Torch 2.10.0+cu128、torchvision 0.25.0+cu128、
  NumPy 1.26.4、系统 OpenCV 4.6.0；通过进程内路径引用 `models/ultralytics_marisat/` 官方 v8.4.8
  （`fbaaba3`）与 `models/CLIP_marisat/` tokenizer（`7ffa84b`），不覆盖已有 Ultralytics，不安装新包。
  未下载基础 SAM3/CLIP 权重，网络自动模型加载/安装关闭；未修改第三方源码。
- 新 `.venv-airbus` 同样只读复用基础 Torch，另装 Keras 3.11.3/h5py 3.14.0、Transformers 4.48.3；
  不安装 TensorFlow，不修改旧环境。Airbus H5、社区 SegFormer bin/config/processor 已清理，
  作者对照 `datasets/airbus_control/test_image_1.jpg` 保留，不随 Git 同步。
- 新 `.venv-cat` 是独立 Python 3.10.12，不复用旧环境：Torch 2.0.1+cu118、torchvision 0.15.2+cu118、
  MMCV 2.0.0、MMDetection 3.0.0、MMEngine 0.7.4、NNCore 0.3.6、NumPy 1.26.4；MMCV 使用官方编译 wheel。
  用户已允许本环境安装；NNCore 打包需 setuptools<81/关闭 build isolation，YAPF 固定 0.40.1。
  初次 NumPy 混合安装故障已通过隔离旧文件/干净安装解决，只影响新环境；`pip check` 通过。
  运行用 `env -u PYTHONPATH -u LD_LIBRARY_PATH` 避免继承 ROS/Python 路径；环境及完整依赖快照见 CAT 输出。
- 输出：新 `outputs/ship_detection_20261015/` 约 155 MiB，有 288 条预测、576 张原图大小标注图和 12 张联系表。
  `outputs/msrs_weight_probe_20261019/` 保存四份加载异常/缺失项、48 张待测样本清单及环境/配置；
  实际推理 0 张，无预测/耗时/精度结论，不能当作 48 张均未检出。既有输出未覆盖。
  CAT 首次环境导入失败保留在 `outputs/cat_mask_isaid_20261019/`；成功运行在
  `outputs/cat_mask_isaid_20261019_run02/`（约 25 MiB，48 条预测/32 mask/96 标注图，质量未验收）。
  `outputs/lim_yolo_probe_20261019/preflight.json` 保存 OBB/训练源码散列与缺权重证据，不是识别效果报告。
  MariSat 四次尝试分别保留在 `outputs/marisat_sam3_20261019{,_run02,_run03,_run04}/`：
  初次容器时间戳失败、FP16 预热异常、混合精度预热异常、最终存储数值预检拒绝。
  没有有效预测/标注图；run04 的 numeric audit 与 original storage evidence 记录了异常。
  下载/推理脚本拒绝默默覆盖，重跑改用新输出目录；详见报告。2026-10-13 曾将旧 9 个实验目录移入回收站：
  `yolo11n_scene_water_fp32`、`yolo11n_scene_water_diagnostic`、`seadronessee_yolo11n_localization`、
  `ship_hybrid_trial_fp32`、`ship_hybrid_trial_fp32_incomplete_json`、`maritime_mast3r`、
  `maritime_mast3r_nocache`、`maritime_mast3r_fp16core`、`dataset_semantic_trial_20261013`。
  共 1876 个文件、约 338 MiB；包含失败记录，原路径不再存在，回收站未清空，可在被清空前恢复。
  对应旧报告/配置已清理，仅保留失败摘要；本轮新对照不恢复这些旧输出。
- 论文 `doc/` 和外部附件不随 Git 自动同步；涉及论文约束时需确认本机可访问文件。

## 未解决问题与下一步

1. 当前选型：用户已验收 OBB 合格、MOD 不合格，按该结论接续；不自动人工看图复判。
   仍需实例掩膜但尚未实现本轮组合，不把检测框、整船区域或驾驶室当作已验证安全降落面。
2. 数据问题：没有 bbox/mask/距离真值。若需要建立人工评测集、训练或额外下载数据，应先取得用户同意。
3. 几何问题：fx/fy=385.45 是历史占位；压缩图像 frame_id 为空，header 与 bag 时间约差 80 分钟。
   里程计来源/尺度/坐标轴、相机标定和时间同步待核验；当前没有真实世界坐标测距结果。
4. 集成边界：只开发 Jazzy，Noetic 迁移已取消，不再是未完成项。核心与 ROS 接口按测试需要分离；
   默认后端切换需另行要求，不增加跨 ROS 版本兼容层。
5. 后续报告在 `ship_detection_trial.md` 末尾追加日期、配置、环境、统计与用户验收，不另建报告、
   不做主动人工视觉核查。跟踪、几何/尺度与时间问题仍待明确，KLT/RANSAC 目前仅候选，不自动实施。
6. 最新候选：CAT Mask R-CNN/iSAID 已跑通，约 1.06 秒/帧，质量待用户验收；n-OBB 用户合格，MOD s/l 用户不合格。
   不因验收自动指定新分辨率/阈值、实现跟踪/补掩膜或替换 ROS；不再优先推进 MOD。
   LiM-YOLO 可训练且原生 OBB，但未找到公开训练权重，需作者 checkpoint 和类别表；不随机初始化作效果评测。
   MSRS/RT、RF-DETR Nano、MOD、MariSat YOLO、Sentinel-2 YOLO 仅水平框，按 D16 淘汰；
   ScatYOLO 未核实训练实现，移出可执行候选。保留 YOLO OBB/seg，非优先像素参考另见清单，不自动测试。
   后续 D17 明确淘汰 Airbus MaskFormer；WaSR 无 IMU 可训练，保留但未训练/推理，不能代替实例船舶识别。
   MariSat 当前用户权重含 2 个原生 -inf，完整加载能通过但 GPU 预热数值无效；需正常权重再测，不擅自修补或混入基础模型参数。
7. 若走训练：先确认另一台电脑 GPU/显存与环境，再考虑 n/s 级实例模型及 iSAID 船类；MariSat 当前共享链接
   跳转登录且许可未明，UOW-Vessel 需申请。公共卫星俯视数据与本地近景仍有差异，自采标注需另行确认。
8. 文献下一步：取得 P01 可访问全文，核查尺度/尺寸先验及定位公式；P02 需查明视觉代码和目标状态来源。
   P03 实验用附加随机图案/AprilTag，不能当自然甲板非合作验证。均仅调研，未决定替换现有算法。

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
  其他分支和 stash 未删除或改动。当时删除及交接改动未提交；2026-10-15 接手已见于 `25336c1`。
  当时已检查删除范围、构建脚本语法及 `git diff --check`，unittest 发现 0 项，不算算法验证；
  剩余 Python 语法和文档链接在下述 2026-10-15 交接核查中补验，未构建 ROS、未跑 GPU/在线链路。
- 2026-10-15 交接核查：电脑 `htfp-crl`，`visual-only`，Python 3.12.3；确认清理提交 `25336c1`、
  四个实验目录为空、数据集及三个实验虚拟环境仍保留。比较 `1343483..25336c1`，`src/`、三个 ROS 配置、
  RViz、launch 和 Jazzy 构建脚本无改动；`master`、`learning-methods` 与备份 stash 引用未变。
  本次修正过期的“尚未提交”描述，更新日期及跨机提示；剩余 Python 语法、Markdown 本地链接、
  `bash -n scripts/build_jazzy.sh` 和 `git diff --check` 检查通过。
  `python3 -m unittest discover -s tests -v` 发现 0 项，不等于通过算法测试；未重建 ROS、未运行模型或下载依赖。
  仅将本次交接修订提交到当前分支，不推送、不修改其他分支。
- 2026-10-15 俯视资源重新调研：建立资源记录（现已整理至 `docs/待测试/`），决策 D11 明确仅考虑异机训练、
  本机不训练。5 个候选下载入口 HEAD 为 200；只读核查确认 CATNet/iSAID 真正实例头、YOLO OBB 仅框、
  MSRS-DETR 仓库缺实现，以及 MariSat 数据共享的登录要求。未下载权重/数据、未恢复旧实验、未安装依赖、
  未训练/推理/构建 ROS。仅变更调研与交接文档，格式和本地链接检查通过；未提交/推送。
- 2026-10-15 待测试目录整理：按用户要求将单篇调研替换为 `docs/待测试/{README,models,datasets}.md`，
  合并登记新候选、待核实资源、现有 SAM3、历史已测/已清理方法，以及公共数据与本机五组样本。
  不将历史失败条目或有访问障碍的资源误标为可直接测试；只改文档和入口，未下载/推理/训练、未提交/推送。
  本地链接、清单覆盖和 `git diff --check` 检查通过。
- 2026-10-15 Jazzy 单平台决定：用户取消 Noetic 迁移，已同步 `AGENTS.md`、README、D12、本状态文件和
  纯视觉迁移清单；原兼容文档改为 `jazzy_development.md`。当前源码仅有 ROS 2 接口，未删除运行节点，
  构建脚本仅调整环境隔离说明，保留保护逻辑。文档链接、脚本语法和 `git diff --check` 检查通过；
  未构建 ROS、未运行模型、未训练、未提交/推送，其他分支未改。
- 2026-10-15 相近论文检索：`htfp-crl`/`visual-only`，核对本机开题报告第三章，并区分原视觉＋点云课题
  与当前视觉路线。新增 `docs/待测试/papers.md`（6 组文献），核查作者页面、论文全文/摘要、公开仓库树、
  P02 的 13 个 Python 文件和内存读取的 zip 清单；未将仓库存在当作完整开源系统。
  P01 全文/尺度先验仍待核实；P03 按 v3 核查图案与自身 IMU/GNSS 条件；P05 确认仅分割模块公开。
  未下载模型/数据、未安装依赖、未训练/推理/构建、未改算法或其他分支、未提交/推送；
  3 份相关文档的 10 个本地链接和 `git diff --check` 检查通过；纯文档调研，未作算法验证。
- 2026-10-15 指定检测模型测试：按用户要求下载并加载 n-OBB/MOD s/l，仅取指定 PT，不安装 QGIS、
  不下载整包/新数据集。6 组输入配置全部完成 48 张测试，含 43 张本地去重和 5 张现有公开对照。
  新增下载/推理脚本、根目录配置、16 项单元测试和 [结果报告](ship_detection_trial.md)，待测试清单更新状态。
  当时助手根据视觉观察作统一未通过判断；该选型判断已被后续用户验收（D14）取代，历史观察留在报告。
  没有真值，不报告精度；未输出实例 mask。
  下载慢连接与元数据同名冲突已修复并校验续传；运行时来源快照中说明文件尚在补齐，三份权重已逐一散列验证。
  16 项单元测试、288 条预测/576 张原图标注/12 联系表结构检查通过；9 份 Python AST、48 个本地链接、
  新文件空白和 `git diff --check` 通过。实际重复执行被 FileExistsError 拒绝，未覆盖结果；最终下载 complete，
  11 个保留文件均复核散列，环境快照里的 incomplete 仅表示当时辅助说明文件仍在下载。
  未训练、未改依赖/SAM3/ROS/其他分支/stash，未提交/推送；权重与输出不随 Git 同步。
- 2026-10-15 用户验收与报告约定：用户明确 OBB 合格、MOD 不合格，已同步报告、模型清单、README 和 D14，
  在 AGENTS.md 固化“同一报告持续追加、不主动人工视觉核查”，保留既有统计、失败记录及可视化。
  仅改文档，未重跑模型或看图；代码/完整性/性能检查仍保留，不扩大为 ROS 集成或训练授权。
  7 份文档的 49 个本地链接、空白及 `git diff --check` 检查通过；纯文档变更，未重跑算法测试。
  未提交/推送，其他分支和 stash 未改。
- 2026-10-15 交接提交前核查：`htfp-crl`/`visual-only`，基于 `081fea9`，用户要求更新交接并提交。
  提交范围包含此前本轮已完成的资源/论文清单、Jazzy 文档调整、检测下载/推理脚本、参数、单元测试和验收记录，
  不只提交引用了未跟踪文件的摘要。其他分支、stash、现有 SAM3/ROS 源码和默认配置保持不变。
  重新运行 16 项单元测试全部通过，9 份 Python AST、11 份 Markdown 的 52 个本地链接、
  `bash -n scripts/build_jazzy.sh` 和 `git diff --check` 通过；这些是代码/文档检查，不是新推理或新精度评测。
  未重新运行模型或人工看图，未下载/训练/安装依赖/构建 ROS；只作本地提交，不执行推送。
  权重、数据、输出和环境仍被 Git 忽略，接手者需另外核查本机资源；提交后的实际状态以 Git 核验为准。
- 2026-10-15 提交/推送约定变更：用户明确“以后提交包含推送”，已写入 AGENTS.md、D15 和本交接。
  已成功推送上一提交 `fb7f4b4` 到 `origin/visual-only`，本次仅修订文档并按新规则提交/推送；
  不改算法，不重新推理或看图，不同步其他分支或忽略的模型/数据；推送成功与否以实际 Git 返回核验。
- 2026-10-19 MSRS-DETR 加载测试：`htfp-crl`/`visual-only`，基于 `db3bdec`。
  新增离线预检脚本、根目录配置及 5 项测试；四份本地文件大小/SHA-256 与作者发布 API 匹配。
  复用 `.venv-yolo`；四次实际 CPU 加载均因缺 dill 受阻，静态确认另缺作者网络层，仓库只有 README。
  48 张既有样本解析/去重完成，但实际推理 0 张，无精度、时延或用户质量验收；报告末尾已追加证据。
  21 项单元测试、11 份 Python AST、诊断结构/散列、文档本地链接与 `git diff --check` 通过，
  重复执行被 FileExistsError 拒绝，未覆盖诊断。未安装依赖、下载模型/数据、训练、人工看图或构建 ROS；
  默认 SAM3/ROS、其他分支和 stash 未改。未提交/推送；当时下一步为取得配套实现，随后被 D16 的水平框淘汰规则取代。
- 2026-10-19 CAT/LiM 指定测试与筛选：用户允许 CAT 独立依赖安装，已取得固定源码与作者散列匹配的 CAT 权重，
  不升级现有环境；新 `.venv-cat` 完成兼容环境安装与 GPU/算子检查，首个导入失败目录保留，成功另用 run02。
  CAT 48/48 张完成，32 份原图二值实例 mask、96 张标注图结构检查通过，输入像素散列与首轮一致；
  本地预测中位数/P95 1055.92/1179.12 ms，无真值，不报告精度、不人工看图，质量待用户验收。
  checkpoint 缺类别名，显式使用固定作者 iSAID 的 ship=0 并检查全部 state_dict 键/形状一致，未使用 COCO 回退。
  LiM 已确认 OBB/训练入口，但作者树/release/HF 检索未找到 checkpoint，仅记录预检，未随机推理/训练。
  新门槛和候选淘汰已写入 AGENTS/D16/模型清单；只停止推进，不删除已有模型/输出/历史或改 ROS。
  新旧两环境均通过 27 项单元测试，语法、本地链接、拒绝覆盖、输出与散列、pip check、diff 格式检查通过。
  未训练、标注、人工视觉核查、构建/改 ROS 或修改其他分支/stash，未提交/推送；下一步是用户验收 CAT、提供 LiM 权重。
- 2026-10-19 Airbus/WaSR 后续筛选：只修改当前 `visual-only` 文档，保留已有未提交的 MSRS/CAT/LiM 工作。
  Airbus MaskFormer 按用户要求淘汰；WaSR 作者版本 `fdb2b036ab9e43bc92a44816852b667737ee79cd`
  确认提供无 IMU 模型、训练入口及损失/优化器，满足用户保留条件；未实测训练运行或精度/性能。
  已更新模型清单/目录、D17 与报告补充；文档本地链接和 `git diff --check` 通过，未重跑模型或代码测试。
  无安装、权重/训练集下载、标注、训练、推理、ROS 修改或文件删除；未提交/推送。
  下一步仍需用户指定测试/训练方案；若训练 WaSR，先确定异机环境和有像素标签的数据，不直接使用未标注 bag。
- 2026-10-19 MariSat 大权重可用性问答：只读核对作者 README、公开分享 HTML 元数据、训练/修订 notebook 和 release；
  作者版本仍为 `a9a35dd`，确认 semantic 文件名及完整模型微调保存方式，maritime 文件只有本地路径，未找到额外公开版本。
  仅更新模型清单和本状态文件，不作新的选型决定或测试通过结论；本地链接和 diff 格式检查通过。
  未下载权重/训练集、安装依赖、训练/推理或改 ROS，未提交/推送；完整文件内容及 T600 4 GB 能否加载仍待另行测试。
- 2026-10-19 MariSat 用户本地权重测试：保留之前全部未提交变更，在 `htfp-crl`/`visual-only` 接续。
  新增根目录配置、离线恢复/推理脚本和 7 项测试；原始目录不动，复原 ZIP 并记录逐文件散列，所有键/形状严格匹配。
  GPU 两种精度预热掩膜非有限，进一步检出源存储 `data/433` 的两个 -inf；独立半浮点字节解码与重新封装内容相同。
  最终脚本在 CPU 数值预检时拒绝该输入，不自改权重，不把未完成样本记录成无检出；所有失败目录/22 B 未完成容器保留。
  已追加统一报告及清单，34 项单元测试、15 份 Python 语法、文档本地链接、源散列/原始字节证据、拒绝覆盖与格式检查通过。
  未训练、人工看图、安装包、下载其他模型权重/数据集或改 ROS/其他分支/stash；未提交/推送。
  下一步需要用户提供来源可校验、参数有限的作者权重；本轮无精度或有效速度统计，也不能据此判定 SAM3 架构不可用。
- 2026-10-19 集中提交前核查：`htfp-crl`/`visual-only`，保留并提交上述累计更新，共 17 个文本文件；
  用户的“提交”包含推送到已确认的 `origin/visual-only`。34 项单元测试分别在 `.venv-yolo`、`.venv-sam3`
  通过，项目 Python AST、Markdown 本地链接、构建脚本语法及 diff 格式检查通过。
  本次不重跑模型、不人工看图、不训练/安装/下载或构建 ROS；模型异常与质量待验收状态不因提交改变。
  模型、数据、输出、第三方源码和虚拟环境不暂存；其他分支和备份 stash 不改，提交/推送结果另以 Git 核验。
  下一步仍为提供正常可校验的 MariSat 权重、用户验收 CAT，或补充 LiM 作者权重；不自动启动这些任务。
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
