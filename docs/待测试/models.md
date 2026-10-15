# 模型清单

[返回](README.md) · 更新：2026-10-15。大小为文件大小，不是运行显存；状态按每项记录。
YOLO11n-OBB 与 YOLO-MOD s/l 已完成[首轮离线测试](../ship_detection_trial.md)：用户验收 OBB 合格、MOD 不合格；均未接 ROS。

## 新候选

| 模型 / 数据来源 | 输出 | 权重 / 代码 | 状态与限制 |
| --- | --- | --- | --- |
| CAT Mask R-CNN R50 / iSAID | 框＋实例掩膜 | [固定权重，219.7 MB](https://huggingface.co/yeliudev/CATNet/resolve/7d69b41903a8b47837c9759c042c63bbee5f75ca/checkpoints/cat_mask_rcnn_r50_3x_isaid-384df911.pth) · [代码/配置](https://github.com/yeliudev/CATNet) | 待测；HEAD 200；GPL-3.0；MMDetection 3/MMCV 2 旧依赖；非轻量部署首选，区别于已失败的 SegFormer |
| YOLO11n/s-OBB / DOTA | 旋转框，无掩膜 | [n，5.8 MB](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-obb.pt) · [s，19.9 MB](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11s-obb.pt) · [说明](https://docs.ultralytics.com/tasks/obb) | n 已离线测试 640/1024，**用户验收合格**；s 未下载/未测；AGPL-3.0；无掩膜、未接 ROS；原统计/历史观察保留于报告 |
| YOLO-MOD YOLO11s/l / ShipRSImageNet | 检测框，无掩膜 | [多模型权重包，787.8 MB](https://zenodo.org/records/19534383) · [代码](https://github.com/mateusz29/yolo-mod) | s/l 已离线测试 640/800，**用户验收不合格**，未接 ROS；Range 提取 PT 为 19.2/51.2 MB，实际两类 ship/warship；包许可声明 CC BY 4.0，插件 GPL-3.0；[结果](../ship_detection_trial.md) |
| RF-DETR Nano / SeaDronesSee | 检测框，无掩膜 | [模型卡/文件](https://huggingface.co/dronefreak/seadronessee-rfdetr-nano) | 待测；`checkpoint_best_total.pth` 120.8 MB，HEAD 200；30.5M 参数；社区发布、Apache-2.0；偏人员/小艇 |
| YOLO11n/s-seg / COCO → 俯视船类 | 实例掩膜 | [官方模型](https://docs.ultralytics.com/tasks/segment) | 训练候选；不是现成的船舶专用权重；n 的通用 COCO 原样推理曾失败，不代表微调后效果；只考虑异机训练 |
| MSRS-DETR / ShipRSImageNet、HRSC2016 | 船舶检测 | [作者权重](https://github.com/loges00/MSRS-DETR/releases/tag/weights) · [仓库](https://github.com/loges00/MSRS-DETR) | 待核实；ShipRS 权重 58.0 MB、HEAD 200；当前仓库只有 README，缺实现/配置/许可，不列为完整开源即用方案 |
| MariSat YOLO11n/l、SAM3 | YOLO 为检测；SAM 实例能力待核实 | [项目](https://github.com/amirabbes/P2M-Maritime-Segmentation) · [共享权重](https://drive.google.com/file/d/1w3XW2gtmdcLUH3TAXct9jDZY0pvY4-Uw/view) | 待核实；共享文件名 `sam3_semantic_best.pt`，不能认作 YOLO nano 实例权重；许可证未明确 |
| mayrajeo marine-vessel-yolo / Sentinel-2 | 模型卡写实例分割，代码说明为检测 | [模型卡/文件](https://huggingface.co/mayrajeo/marine-vessel-yolo) · [代码](https://github.com/mayrajeo/ship-detection) | 待核实、低优先；`yolo11s_tci.pt` 19.2 MB；10 m/像素卫星域，任务头未加载确认；AGPL-3.0 |
| ScatYOLOv8+CBAM / ShipSG | 实例分割研究 | [作者研究](https://elib.dlr.de/211646/) · [展示](https://dlr-mi.github.io/ship3d-demo/) | 待核实；未确认可直接取得的专用权重和完整推理代码，不把展示当可运行模型 |
| LiM-YOLO / 遥感船舶 | 旋转框 | [代码](https://github.com/egshkim/LiM-YOLO) | 待核实、低优先；未确认 checkpoint，主要配置约 21M 参数 |
| Airbus MaskFormer / Swin | 分割头具体任务待核实 | [模型卡/文件](https://huggingface.co/artemkramov/airbus-ship-challenge-segmentation) | 历史调研候选，未实测；约 411 MB，非轻量；类别映射曾有疑点，未在本轮重新核查 |
| WaSR 等 USV 前视分割 | 语义前景，不保证船舶实例 | [MODS 入口](https://seadronessee.cs.uni-tuebingen.de/dataset) | 非优先；与 UAV 俯视不同，未核查具体 checkpoint |

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
随后按用户要求下载并加载 n-OBB、YOLO-MOD s/l，其他新候选仍未运行，不自动恢复历史已清理模型。
先确认来源、许可与配套代码，再讨论运行；公开文件不等于精度、安全性或实时性已验证。
