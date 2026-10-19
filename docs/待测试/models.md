# 模型清单

[返回](README.md) · 更新：2026-10-19。大小为文件大小，不是运行显存；状态按每项记录。
YOLO11n-OBB 与 YOLO-MOD s/l 已完成[首轮离线测试](../ship_detection_trial.md)：用户验收 OBB 合格、MOD 不合格；均未接 ROS。

## 筛选规则（2026-10-19）

仅推进 **有可获取训练实现，且能原生输出旋转框（OBB）或像素掩膜** 的模型。
两种输出满足其一即可进入筛选；OBB 仍不是实例掩膜，语义前景也不保证单船实例分离。
“不可训练”不等于“禁止在本机训练”：前者是模型/公开实现能力，后者是本项目的执行边界。
配套实现无法核实的资源移出当前可执行候选，不断言模型理论上不能训练。
淘汰指不再安排测试，不删除权重、已有报告、数据集或现有 SAM3/ROS 链路。

## 保留候选

| 模型 / 数据来源 | 输出 | 权重 / 代码 | 状态与限制 |
| --- | --- | --- | --- |
| CAT Mask R-CNN R50 / iSAID | 框＋实例掩膜 | [固定权重，219.7 MB](https://huggingface.co/yeliudev/CATNet/resolve/7d69b41903a8b47837c9759c042c63bbee5f75ca/checkpoints/cat_mask_rcnn_r50_3x_isaid-384df911.pth) · [训练代码/配置](https://github.com/yeliudev/CATNet) | 2026-10-19 独立环境完成 48 张 GPU 测试，32 份候选实例 mask；预测中位数 1055.92 ms，仅运行统计，用户质量未验收；SHA-256 匹配，GPL-3.0，未接 ROS；[结果](../ship_detection_trial.md#2026-10-19cat-mask-r-cnn-r50isaid-与-lim-yolo) |
| YOLO11n/s-OBB / DOTA | 旋转框，无掩膜 | [n，5.8 MB](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-obb.pt) · [s，19.9 MB](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s-obb.pt) · [说明](https://docs.ultralytics.com/tasks/obb) | n 已离线测试 640/1024，**用户验收合格**；s 未下载/未测；AGPL-3.0；无掩膜、未接 ROS；原统计/历史观察保留于报告 |
| YOLO11n/s-seg / COCO → 俯视船类 | 实例掩膜 | [官方模型](https://docs.ultralytics.com/tasks/segment) | 训练候选；不是现成的船舶专用权重；n 的通用 COCO 原样推理曾失败，不代表微调后效果；只考虑异机训练 |
| LiM-YOLO / 遥感船舶 | 旋转框，无掩膜 | [训练/推理代码](https://github.com/egshkim/LiM-YOLO) · [权重相关 issue](https://github.com/egshkim/LiM-YOLO/issues/1) | 本轮指定测试但缺训练权重；固定版本的 lim-yolo YAML 使用 OBB 头，有自定义 GN-CBLinear 和训练入口；仓库/release 未提供 PT，HF 检索未找到对应发布；保留为异机训练/待权重候选，不用随机网络作识别评测 |

其他像素输出模型：MariSat 已按用户另行指定测试，WaSR 只保留训练候选，不自动测试：

| 模型 | 保留依据与限制 |
| --- | --- |
| [MariSat SAM3](https://github.com/amirabbes/P2M-Maritime-Segmentation) | **2026-10-19 测试受阻：用户提供的解压权重内含 2 个 -inf**；1118 个条目严格匹配/加载通过，FP16 和混合精度 GPU 预热均有无效掩膜值，无有效 48 张结果，不是零检出/质量不合格。原始字节已证实异常，不修补参数；待作者正常权重，[诊断](../ship_detection_trial.md#2026-10-19用户下载的-marisat-sam3-权重测试数值异常)。许可及有效实例输出仍待核验；另一个 maritime 文件未找到公开链接 |
| [WaSR（无 IMU 变体）](https://github.com/lojzezust/WaSR) | **确认可自行训练，按用户条件保留为异机训练候选**；有完整训练入口、损失/优化器和无 IMU 模型。默认障碍物/水面/天空三类语义掩膜，不是单船实例；需要带像素标注的数据，未训练/推理，环境兼容性未实测 |

MariSat 权重核查（2026-10-19）：作者版本 `a9a35dd74edb3fc66ce2f12f8edbc2b8b5e65d7c`；
[微调 notebook](https://github.com/amirabbes/P2M-Maritime-Segmentation/blob/a9a35dd74edb3fc66ce2f12f8edbc2b8b5e65d7c/finetuningSAM3.ipynb)
基于 `sam3.pt`，用 `SAM3SemanticPredictor` 加载再覆盖 state_dict；不能把它当现有 GGML Q4_0 文件直接替换。
[修订 notebook](https://github.com/amirabbes/P2M-Maritime-Segmentation/blob/a9a35dd74edb3fc66ce2f12f8edbc2b8b5e65d7c/correctionfinetuningSAM3.ipynb)
中的 maritime 权重名不证明文件已发布；仓库 release 当前为空。
上述是公开来源核查；随后用户提供解压的完整参数（原目录 2.65 GB），已严格加载并执行 GPU 预热，
但文本投影矩阵有 2 个负无穷，无法有效评测；当前文件不可直接用于正式测试，不据此淘汰 SAM3 架构。
本机已复原容器并保留逐文件散列，但缺作者原压缩包/校验值，不能确定异常出自训练、导出、下载或解压。
既有量化/ROS 链路未修改，也未取得或测试所谓更大版本。

WaSR 核查版本：`fdb2b036ab9e43bc92a44816852b667737ee79cd`（2026-10-19）。
[训练入口](https://github.com/lojzezust/WaSR/blob/fdb2b036ab9e43bc92a44816852b667737ee79cd/train.py)
实际使用 `--model wasr_resnet50` 或 `--model wasr_resnet101`；默认是 IMU 版，不能原样照跑。
[模型实现](https://github.com/lojzezust/WaSR/blob/fdb2b036ab9e43bc92a44816852b667737ee79cd/wasr/models.py)
会为这两个名称选择 `NoIMUDecoder`；
[数据加载器](https://github.com/lojzezust/WaSR/blob/fdb2b036ab9e43bc92a44816852b667737ee79cd/datasets/mastr.py)
允许训练/验证配置不含 `imu_dir`，仅提供 RGB 和语义 mask。
可按作者 MaSTr1325 配方训练；若换为船体专用标签，须另外核对标签及损失定义，不能把现有三类模型说成船舶实例模型。
代码使用旧 Lightning API，需另配兼容环境；本次仅核查源码，不安装依赖、下载训练集或在本机开训。

## 淘汰 / 移出当前可执行候选

| 模型 / 方法 | 原因与保留记录 |
| --- | --- |
| [Airbus MaskFormer / Swin](https://huggingface.co/artemkramov/airbus-ship-challenge-segmentation) | **2026-10-19 用户明确淘汰**，不再测试或推进训练；未在本项目运行，不将选型淘汰描述为实测效果不合格或模型不能训练 |
| YOLO-MOD 船舶 s/l | 仅水平框，无 OBB/像素掩膜，且用户已验收不合格；[既有结果](../ship_detection_trial.md)及权重不删除 |
| RF-DETR Nano / SeaDronesSee | [当前发布](https://huggingface.co/dronefreak/seadronessee-rfdetr-nano)为水平检测，不是可输出实例掩膜的其他 RF-DETR 变体；不继续安排测试 |
| MSRS-DETR 与其 RT-DETR 基线 | 当前权重为水平框，无 OBB/掩膜，另缺作者实现；[已完成的加载诊断](../ship_detection_trial.md#2026-10-19msrs-detr-本地权重加载测试受阻未进入推理)保留，不再优先索取源码来测试不合门槛的输出 |
| MariSat YOLO11n/l | 作者 notebook 用普通 `yolo11n.pt` 并将多边形转水平 bbox，不能因源数据有多边形就说模型可输出掩膜；SAM3 单列保留 |
| mayrajeo marine-vessel-yolo / Sentinel-2 | [作者代码](https://github.com/mayrajeo/ship-detection)使用水平检测/框标注；模型卡的 image-segmentation 标签不足以证明该权重有掩膜头；按当前发布能力移出，不下载待赌任务头 |
| ScatYOLOv8+CBAM / ShipSG | 论文具备分割研究结果，但尚未确认可获取的配套训练实现和专用 checkpoint；移出可执行候选，不断言论文模型理论上不可训练；[研究资料](https://elib.dlr.de/211646/)仍可参考 |
| GrabCut 单独作为模型路线 | 无可训练模型权重；不再作为本轮筛选后的学习模型候选，历史组合失败记录保留 |
| MASt3R 重建路线 | 本任务要求的 OBB/船体像素分割并非其原生输出；历史实验留在另一分支，不恢复 |

## 现有与历史资源（不自动重测）

| 模型 / 方法 | 当前状态 |
| --- | --- |
| SAM3 Q4_0 / TorchAO 回退 | 现有 ROS 保留；配置在 `config/common.yaml`；不是本次新增候选，本轮未运行 |
| COCO YOLO11n-seg | 历史已测，整船识别未通过；本地权重/试验代码已清理；若微调，见上表训练候选 |
| SeaDronesSee YOLO11n | 历史已测，局部框/车辆误检；已清理；[原公开模型](https://huggingface.co/dronefreak/seadronessee-yolo11n) |
| MobileSAM / GrabCut＋海事检测框 | 历史已测，候选掩膜不等于整船；试验已清理；[MobileSAM](https://github.com/ChaoningZhang/MobileSAM)，GrabCut 无训练权重 |
| Airbus MobileNetV2-U-Net | 历史已测失败、已清理；语义前景＋连通域；[原项目](https://github.com/sovden/airbus_ship_segmentation) |
| iSAID SegFormer-B0 | 历史已测失败、已清理；语义前景＋连通域；[原模型](https://huggingface.co/giuseppemartino/segformer-finetuned-isaid)，训练来源未完全核验 |
| MASt3R | 历史两帧重建，非船舶实例模型；本机源码/权重已清理；Git 实验仍在 `learning-methods`，不恢复 |

复现记录：CAT 权重版本 `7d69b41903a8b47837c9759c042c63bbee5f75ca`；
RF-DETR Nano 版本 `5bc1c5e19f284faf48a350ef4cb309dd5b649673`；YOLO OBB 为官方 `v8.3.0` release。
初轮调研 HEAD 200 的 5 项为 CAT、YOLO n-OBB、YOLO-MOD 包、RF-DETR Nano、MSRS ShipRS 权重；当时未下载。
随后按用户要求下载并加载 n-OBB、YOLO-MOD s/l；2026-10-19 对用户提供的 MSRS/RT 权重做加载预检，受阻未推理。
本轮按用户要求继续 CAT/LiM 测试并筛选其余模型，状态分别见上表；未指定的保留项不自动测试，不恢复历史已清理模型。
先确认来源、许可与配套代码，再讨论运行；公开文件不等于精度、安全性或实时性已验证。
