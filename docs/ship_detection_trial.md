# 船舶感知测试记录（持续追加）

后续模型/配置测试均追加到本文件，不另建逐轮报告；已完成轮次及原始统计保留。
各轮“未提交/推送”为当时快照，最新同步范围见 [交接记录](PROJECT_STATE.md)。
后续不主动进行人工视觉核查，可继续导出可视化供用户查看；质量验收按用户反馈单独记录。
当前验收：**YOLO11n-OBB 合格；YOLO-MOD s/l 不合格**（2026-10-15 用户确认，见文末补充）。

## 首轮：YOLO11n-OBB 与 YOLO-MOD 船舶检测

日期：2026-10-15；电脑：`htfp-crl`；分支：`visual-only`，基于 `081fea9` 的未提交工作区。
状态：**三个权重均已运行，六组配置完成；用户验收 OBB 合格、MOD 不合格。**
本轮只做检测，不输出实例掩膜，不训练，不接 ROS，不新增跟踪、测距或降落控制。

## 输入与测试方法

- 本地：`scene_water_audit` 11 张＋`scene_water_yolo_clips` 36 张，按解码像素去重后 43 张。
  覆盖车辆、反光水面、船头、多船和近距离甲板；不少船体被图像边界截断，不能要求模型恢复不可见船体。
- 对照：现有 MTReD 场景 01/03 各 2 张＋Airbus 作者样例 1 张，共 5 张。未下载新数据集。
  对照不是本项目的独立有标注测试集，不能用于报告 mAP。
- 总计 48 张；每组均处理全部样本，6 组共 288 次记录；没有手工框、裁剪放大或提示分割。
- 原图 BGR 输入，Ultralytics 方形 letterbox，`rect=false`、batch=1、FP32、GPU 0，预热 3 次。
  NMS IoU=0.7，max_det=300，类别相关 NMS，无 TTA。预测阈值统一为 0.10；0.25 结果是对同次 NMS 后
  检测列表的再次过滤，不是另一遍独立推理。不通过调低阈值宣称算法通过。
- OBB 保留原图像素四角及其外接水平框；水平检测保留原图 xyxy。OBB 角点可延伸到画面外，未伪造裁剪后的完整轮廓。
  原图尺寸、像素 SHA-256、来源 manifest、bag/header 时间和空 frame_id 均按来源记录；没有修补标定/同步。

## 权重来源与实际内容

| 权重 | 实际任务与类别 | 本地文件大小 | 本轮输入 |
| --- | --- | --- | --- |
| `yolo11n-obb.pt` | DOTA 旋转框，15 类；按名称选择 `ship`（本权重编号 1） | 5,795,654 B | 640、1024 |
| `ships_yolo11s.pt` | 水平检测；`ship`=0、`warship`=1，两者均计作船舶候选 | 19,213,466 B | 640、800 |
| `ships_yolo11l.pt` | 水平检测；同上 | 51,232,217 B | 640、800 |

来源：[Ultralytics 官方 OBB](https://docs.ultralytics.com/tasks/obb)、
[固定 v8.3.0 权重](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-obb.pt)、
[YOLO-MOD 作者仓库](https://github.com/mateusz29/yolo-mod)、[Zenodo 固定记录 19534383](https://zenodo.org/records/19534383)。
作者仓库核查版本为 `be071b0e79b6a1d145ded89355590ea8a76e4915`，
[数据配置](https://github.com/mateusz29/yolo-mod/blob/be071b0e79b6a1d145ded89355590ea8a76e4915/train/dataset/ShipRSImageNet.yaml)也是两类，
不能把 ShipRSImageNet 原始细分类数量当作本模型类别数量。

发布 metadata 写 `input_resolution=930x930`，两个实际 checkpoint 的 `train_args.imgsz` 都是 800，
故本轮使用 640/800，不宣称 930 就是 checkpoint 的真实训练输入；DOTA checkpoint 记录为 1024。
发布 metadata 的泛称 `ships` 也不替代加载后的类别表。

只通过 HTTP Range 提取 s/l 的 PT 和说明文件，未下载完整 787,774,856 B 压缩包、其他模型、ONNX、QGIS 或训练集。
三份权重保留合计约 76.2 MB。ZIP 成员通过 CRC32，下载后记录 SHA-256，推理前再次核对；
**没有下载完整包，因此未核验 Zenodo 整包 MD5**。CRC/本地散列不等于独立签名验证。
下载经历慢连接中止续传及元数据同名冲突，已加入校验续传和按 s/l 区分元数据文件名；未覆盖已有权重。
首次留下的 `metadata.json` 是 s 元数据副本，仍保留在来源记录中。

权重 SHA-256：

```text
yolo11n-obb.pt   b62898ebf38940ca4df323863e45ee9d84a1a46d5d11ebdde529fb33aa9f3a32
ships_yolo11s.pt eb440c5af87c58eda200eb381e8f11895184ee07e91227ebbd6874421fcbff14
ships_yolo11l.pt cc68deaf469c568b492da05b812c7cdff6287dd2e07a7c70f303d1cfcefe0def
```

Zenodo 记录及包内 LICENSE 声明 CC BY 4.0，作者插件代码 GPL-3.0；Ultralytics 框架另有其许可。
这里只登记来源声明，不据此消除框架/派生模型的许可义务，也不作商用授权承诺。

## 本机性能与输出数量

硬件：i7-11800H、NVIDIA T600 Laptop / 标称 4096 MiB；PyTorch 可见约 3718 MiB；驱动 595.99.02。
Python 3.12.3，Torch 2.5.1+cu124，Ultralytics 8.3.226，OpenCV 4.11.0，NumPy 1.26.4，PyYAML 6.0.2；CPU 线程 4。
复用已有 `.venv-yolo`，未安装/升级依赖。该环境仍以绝对路径引用基础虚拟环境，不保证可直接跨机复制。

下表计时仅使用 **43 张本地图像**，排除预热，CUDA 已同步：

| 配置 | 预测调用中位数 / P95（ms） | 网络推理中位数（ms） | 解码＋预测＋两份可视化写盘中位数（ms） | PyTorch 峰值 allocated / reserved（MiB） |
| --- | --- | --- | --- | --- |
| n-OBB / 640 | 14.34 / 14.64 | 11.59 | 41.15 | 66.2 / 88.0 |
| n-OBB / 1024 | 22.28 / 26.54 | 17.98 | 49.39 | 103.5 / 138.0 |
| MOD s / 640 | 19.40 / 19.92 | 17.07 | 46.18 | 110.2 / 132.0 |
| MOD s / 800 | 28.19 / 29.02 | 25.16 | 55.19 | 135.4 / 152.0 |
| MOD l / 640 | 62.77 / 64.97 | 60.09 | 89.86 | 223.6 / 256.0 |
| MOD l / 800 | 99.43 / 101.25 | 96.10 | 126.16 | 276.8 / 302.0 |

“预测调用”包含预处理、网络推理、NMS/包装，不含图片解码、画框、JPEG 写盘。
“两份可视化写盘”包含 0.25/0.10 两份原图大小 JPEG，不含 JSON 写盘、联系表、ROS、bag 解码或分割。
这不是在线端到端时延；PyTorch allocated/reserved 也不是整卡全部显存，不能用这些数值承诺机载 FPS。
峰值显存统计覆盖该配置全部 48 张，包括 5 张对照，不限于上表计时使用的 43 张。

| 配置 | 本地有船舶候选输出的帧数（0.25 / 0.10） | 本地候选框数（0.25 / 0.10） |
| --- | --- | --- |
| n-OBB / 640 | 18 / 29 | 22 / 46 |
| n-OBB / 1024 | 3 / 11 | 4 / 17 |
| MOD s / 640 | 22 / 38 | 23 / 60 |
| MOD s / 800 | 21 / 29 | 28 / 61 |
| MOD l / 640 | 33 / 39 | 46 / 120 |
| MOD l / 800 | 25 / 35 | 63 / 122 |

**这些是输出数量，不是正确率、召回率或漏检率。** 例如车辆误报也会增加“有输出帧数”。

## 历史视觉观察（已完成记录，不作后续必需流程）

人工只读查看六组 0.25 联系表及 s/l 640 的 0.10 联系表，并核对下列原图坐标/分数；不是新增人工真值集。
样本编号对应输出 `sample_manifest.json`，前 11 张依次为 audit 原图。

- `000`（60 s，岸边车辆）：n-OBB/640 将两辆车标成 ship（0.787、0.718）；MOD s/l 也有横跨车辆的框。
  MOD l/800 该帧没有框，但 `010` 仍有车辆误报，不能称已解决车辆混淆。
- `002`（180 s，船头与邻船）：n-OBB 未检出；MOD s/640 的 0.848 高分框只覆盖局部且混入邻船/水面，
  不能当成完整单船。高置信度不保证框正确。
- `004`（300 s，多船）：MOD l/640 能框出两条船的较大可见区域，比 s/640 的横向局部框更合适；仍漏其他船。
  MOD s/800 在左侧同一船体产生两个重叠框，说明增大输入并未稳定改善实例一致性。
- `007`（420 s，主要近景紫色甲板）：n-OBB 和 MOD s/l 640 均无船框；l/800 只有左边缘细小局部框，
  未定位主要甲板。不能将远距/多船表现等同于近距目标持续识别。
- `009`（540 s，多船）：MOD l/640、l/800 都能较好覆盖左侧货船的画面内部分，分数约 0.83/0.85，
  但邻船仍漏检；图像边界外船体不可恢复。
- `010`（600 s，岸边车辆）：MOD l/640 将车辆区域标为 warship（0.528）；l/800 也把该区域标为 ship。
- 所有配置在 Airbus 对照 `047` 都能定位画面内的截断船舶；MTReD 场景 03 的斜视货船 `045/046` 在 0.25 下均未检出。
  s 模型对场景 01 的远处地平线区域产生框，不能将对照样本有输出视作正确检出帆船。
- 从 0.25 降到 0.10 后候选明显增加，伴随水面/边界局部框、重复框和更多 warship 预测；并未解决一致整船定位。

此前助手判断：MOD l/640 可作后续离线定位质量对照，s/640 可作较快对照；两者未达到当时助手采用的定位标准，
YOLO11n-OBB 当时也未被助手优先推荐。**该选型判断已被文末用户验收结论取代：OBB 合格、MOD 不合格。**
上述观察仅保留为历史记录，不重新核查，不作为否定用户验收的依据，也不是有真值的精度评测。
只输出框，没有测试本轮“框＋掩膜”组合，也不能把检测框当实例分割结果或安全降落面。

## 结果入口与复现

- [完整输出汇总](../outputs/ship_detection_20261015/summary.json)、[样本来源](../outputs/ship_detection_20261015/sample_manifest.json)、
  [环境及运行时来源快照](../outputs/ship_detection_20261015/environment.json)。
- 联系表：[n-OBB/640](../outputs/ship_detection_20261015/dota_yolo11n_obb_640/contact_0.25.jpg)、
  [MOD s/640](../outputs/ship_detection_20261015/yomod_yolo11s_640/contact_0.25.jpg)、
  [MOD l/640](../outputs/ship_detection_20261015/yomod_yolo11l_640/contact_0.25.jpg)。
- 每组目录有 48 行 `predictions.jsonl`、0.25/0.10 各 48 张原图大小标注图、两张联系表和统计。
- [统一配置](../config/ship_detection_trial.yaml)、[下载脚本](../scripts/fetch_ship_detection_weights.py)、
  [推理脚本](../scripts/ship_detection_trial.py)、[单元测试](../tests/test_ship_detection_trial.py)。

```bash
# 首次下载到不存在的权重目录；若中断，显式加 --resume，只复用散列匹配文件。
.venv-yolo/bin/python scripts/fetch_ship_detection_weights.py --config config/ship_detection_trial.yaml
.venv-yolo/bin/python scripts/ship_detection_trial.py --config config/ship_detection_trial.yaml
.venv-yolo/bin/python -m unittest discover -s tests -v
```

当前默认输出已存在，推理脚本应拒绝覆盖；重跑需复制配置并改为新的 `output_dir`，不用删除旧结果。
权重、输入和输出均被 Git 忽略，另一台电脑不会随代码自动取得；公开结果链接只在本机产物保留时有效。
当前没有接入 ROS，后续集成仍只面向 Jazzy。

验证：6/6 配置完成、无推理失败；288 条预测和 576 张原图大小标注图、12 张联系表结构检查通过。
16 项单元测试、9 份 Python AST 语法、8 份文档的 48 个本地链接及 `git diff --check` 检查通过。
实际重复执行推理入口得到预期的 FileExistsError，确认拒绝覆盖，不是本轮推理失败；
最终下载状态 complete，11 项保留文件（含早期元数据副本）均复核 SHA-256。
运行时 `environment.json` 的来源快照抓取于说明文件尚在补齐时，其中 incomplete 指辅助下载进度，
不是三份推理权重缺失；运行前已经核验三份权重散列。最终完整来源以权重目录的 `download_provenance.json` 为准。
未标注、未训练、未修改现有 SAM3/ROS、其他分支或 stash，未提交/推送。

## 2026-10-15 补充：用户验收与后续报告约定

- 用户明确验收：本轮 **YOLO11n-OBB 合格，YOLO-MOD（s/l）不合格**；当前选型状态以此为准。
  不把未经测试的 YOLO11s-OBB 标为合格，也不自行指定新的输入分辨率或阈值。
- 本次仅记录反馈，没有重新推理或看图，未改动原始计时、数量、预测文件或可视化。
  用户验收不等于已经取得 mAP/距离误差等有真值指标，也不表示已接 ROS 或已输出实例掩膜。
- 后续每轮在本文件末尾追加：日期/目的、模型/配置、输入与环境、运行结果与耗时、输出路径、用户验收。
  保留既有轮次，不新建独立测试报告；其他文档只作索引和交接摘要。
- 不再主动人工视觉核查或新增视觉核查结论。仍做代码测试、输出结构/完整性、文件校验和性能统计；
  可继续生成标注图供用户自行查看，不自动删除已有图像、MOD 权重或失败记录。
- 未授权替换 ROS 默认后端、训练或实现跟踪/测距/降落算法；本次不执行这些操作，未提交/推送。

## 2026-10-19：MSRS-DETR 本地权重加载测试（受阻，未进入推理）

电脑：`htfp-crl`；分支：`visual-only`，基于 `db3bdec` 的未提交工作区。
用户已将权重放在 `models/MSRS/`，本轮未下载模型或数据、未安装依赖。
**结论：权重文件完整，但当前缺少作者配套实现，尚不能完成图像推理；不是检测效果不合格。**

### 来源与文件完整性

核对[作者 release](https://github.com/loges00/MSRS-DETR/releases/tag/weights)及其
[发布 API](https://api.github.com/repos/loges00/MSRS-DETR/releases/tags/weights)：
本地四份文件的字节数与 SHA-256 均与发布 API 的 size/digest 一致。
两份 RT-DETR 是同目录自带的基线，只做加载预检，不替代用户指定的 MSRS-DETR。

| 文件（均位于 `models/MSRS/`） | 字节数 | 完整性 | 实际加载 |
| --- | --- | --- | --- |
| `MSRS-DETR_HRSC2016_epochs500_best.pt` | 58,037,441 | 匹配作者发布散列 | 缺 `dill`，受阻 |
| `MSRS-DETR_ShipRSImageNet_epochs500_best.pt` | 58,009,665 | 匹配作者发布散列 | 缺 `dill`，受阻 |
| `RT-DETR_HRSC2016_epochs500_best.pt` | 80,715,448 | 匹配作者发布散列 | 缺 `dill`，受阻 |
| `RT-DETR_ShipRSImageNet_epochs500_best.pt` | 80,725,432 | 匹配作者发布散列 | 缺 `dill`，受阻 |

精确 SHA-256 和文件大小保存于 [预检配置](../config/msrs_weight_probe.yaml)。
仅匹配发布文件不能证明 pickle 安全；本轮先静态检查引用，再显式加载用户提供且散列匹配的文件。

### 加载阻塞与所缺实现

现有 `.venv-yolo`：Python 3.12.3、Torch 2.5.1+cu124、Ultralytics 8.3.226；
CUDA 可见，硬件仍为 T600 Laptop / 标称 4096 MiB。加载使用 CPU `map_location`，未运行 GPU 推理。
四份 checkpoint 的实际 `torch.load(..., weights_only=False)` 均首先报错：
`ModuleNotFoundError: No module named 'dill'`。

此外，对 ZIP 内 pickle 的 GLOBAL 引用做不反序列化的静态检查，并与当前可导入模块对照，确认：

- 两份 MSRS 还缺 `ultralytics.nn.extra_modules.block` 中的 `C2f_FAT`、`ETB`、
  `Attention_F`、`Attention_S`、`FeedForward`；`filc` 中的 `FAT_Block`、`Swin_FDWA`、
  `WindowAttention`、`WindowFrequencyModulation`；`tsdn` 中的两种 LayerNorm。
- 两份 RT-DETR 基线还缺其旧版/定制 `ultralytics.nn.modules.block` 中的
  `BasicBlock`、`Blocks`、`ConvNormLayer`。不能认为安装 `dill` 后就能直接使用当前官方 Ultralytics 加载。
- [作者仓库固定版本](https://github.com/loges00/MSRS-DETR/tree/1fdef43e7834397f2f296f41a23852d79eec14d1)
  当前仅有 README，本地 `MSRS-DETR-weights/` 也只有 README；未提供上述实现、配置或依赖清单。
  [论文](https://www.mdpi.com/1424-8220/25/18/5734)的数据可用性说明指向同一权重 release，
  不能把公开权重等同完整推理项目。

没有为了消除异常安装一串依赖、伪造同名层或把网络替换为官方 RT-DETR。
下一步需要作者配套 Ultralytics 源码（上述模块及匹配的 `rtdetr-C2f-FAT_p2_ETB.yaml`、依赖版本），
或作者导出的可用 ONNX/TorchScript 与预处理/类别说明，再继续实际图像测试。

### 输入、产物和验证

复用既有配置，成功解析并按解码像素去重得到 43 张本地＋5 张对照的 **48 张待测样本**。
由于加载受阻，**实际推理 0 张**：没有预测框、掩膜或推理时延，也没有精度/质量验收结论。
这里的 0 是“未进入推理”，不是“48 张均未检出”；状态为“加载受阻、待配套实现”。
未进行人工视觉核查，未修改 ROS 默认后端、既有试验输出或权重；未训练，未提交/推送。

- [四份加载诊断与静态缺失项](../outputs/msrs_weight_probe_20261019/checkpoint_probe.json)
- [待测样本清单](../outputs/msrs_weight_probe_20261019/sample_manifest.json)、
  [环境/代码散列](../outputs/msrs_weight_probe_20261019/environment.json)、
  [配置快照](../outputs/msrs_weight_probe_20261019/config_snapshot.json)
- [预检脚本](../scripts/probe_msrs_weights.py)、[单元测试](../tests/test_msrs_weight_probe.py)

```bash
.venv-yolo/bin/python scripts/probe_msrs_weights.py --config config/msrs_weight_probe.yaml --attempt-load
.venv-yolo/bin/python -m unittest discover -s tests -v
```

首次预检返回码 2 表示记录到加载/实现缺项，不表示完成推理。输出目录拒绝覆盖；
重跑应另存配置并使用新 `output_dir`，不删除旧诊断。
21 项单元测试通过；11 份 Python AST、诊断结构/散列、文档本地链接与 `git diff --check` 检查通过。
这些是代码/文件和加载检查，不是算法精度或性能验证。模型与诊断输出被 Git 忽略，不随提交跨机同步。

## 2026-10-19：CAT Mask R-CNN R50/iSAID 与 LiM-YOLO

电脑：`htfp-crl`；分支：`visual-only`，HEAD `db3bdec` 的未提交工作区；接续上节 MSRS 诊断，不覆盖旧输出。
用户指定本轮两个模型，并另行允许创建独立 CAT 环境、安装兼容依赖；不训练、不改 ROS 默认模型、不人工看图。

| 指定模型 | 本轮实际状态 | 输出与验收 |
| --- | --- | --- |
| CAT Mask R-CNN R50 / iSAID | **48/48 张 GPU 推理完成** | 32 份原图像素实例掩膜、96 张标注图；用户质量尚未验收 |
| LiM-YOLO | 代码/输出类型预检完成，**缺训练权重，未进入图像推理** | 0 张推理不是 0 个检出；保留待权重/异机训练候选，不作质量判定 |

### CAT 来源与独立环境

- [作者代码/训练说明](https://github.com/yeliudev/CATNet)固定版本
  `64775d0885995f99f0547161abef6032db1b65aa`，本地 `models/CATNet/`；未修改第三方源码。
- [指定权重](https://huggingface.co/yeliudev/CATNet/resolve/7d69b41903a8b47837c9759c042c63bbee5f75ca/checkpoints/cat_mask_rcnn_r50_3x_isaid-384df911.pth)
  为无增强版，219,657,199 B；来自作者 HF 固定版本 `7d69b41903a8b47837c9759c042c63bbee5f75ca`。
  完整 SHA-256 匹配该版本 LFS 元数据：`384df911651f825b3eb26a790b7b93d20ef28d0e7bcb4c28e720a3f6fb5d76ab`。
  本地 `checkpoints/cat_isaid_20261019/checkpoints/cat_mask_rcnn_r50_3x_isaid-384df911.pth`。
- 新建 `.venv-cat`，Python 3.10.12、Torch 2.0.1+cu118、torchvision 0.15.2+cu118、MMCV 2.0.0
  （官方编译算子 wheel）、MMDetection 3.0.0、MMEngine 0.7.4、NNCore 0.3.6、NumPy 1.26.4。
  GPU 为 T600 Laptop 4096 MiB，驱动 595.99.02，CPU i7-11800H，线程数 4；FP32、batch=1、GPU 0。
  未修改 `.venv-yolo`、`.venv-mast3r`、`.venv-airbus`、`.venv-sam3` 或系统 Python 包。
- 本机 Python 3.10 缺 ensurepip，最终用 pip 的 `--python` 管理新环境；不安装系统 apt 包。
  NNCore 旧打包需要 `pkg_resources`，用 setuptools 80.10.2 和 `--no-build-isolation`；YAPF 固定 0.40.1。
  初次环境安装出现 NumPy 混合文件/ABI 冲突（本次重叠安装造成），已将该新环境内旧 NumPy 安装隔离保留并干净重装；
  `.venv-cat/retired_numpy_install/` 及 disabled 元数据可用于追溯，不触碰用户原有环境。
  首次运行停在 SciPy 导入，记录保留在 `outputs/cat_mask_isaid_20261019/summary.json`，未推理，未覆盖。
  完整成功测试另存 `outputs/cat_mask_isaid_20261019_run02/`；`pip check` 通过。

配置、类别与输出约束：

- 复用此前 43 张像素去重本地图像＋5 张公开对照，48 张的路径/像素散列与首轮相同，未下载新数据集。
- 使用作者 `configs/isaid/cat_mask_rcnn_r50_3x_isaid.py`，保持长宽比 resize `(1400, 800)`，
  本地 1280×1024 输入缩放为 1000×800；不使用切片、TTA、手工框或另一个分割模型。
  NMS 为作者 soft_nms、IoU=0.5，max_per_img=1000；模型分数阈值 0.05，mask 二值阈值 0.5。
  导出最低置信度 0.10，0.25 统计是同次结果过滤，不是再次推理。
- checkpoint 的 meta 只有时间、无类别名；MMDetection 会警告并默认 COCO。脚本明确以固定作者
  `datasets/isaid.py` 的 15 类覆盖该回退，`ship=0`，不是 COCO boat=8 或此前 DOTA ship=1。
  两个类别头维度为 16（含背景）/15，全部模型 state_dict 键和张量形状与 checkpoint 严格一致。
- 仅为无标注图像推理移除 LoadAnnotations，关闭无关 ImageNet backbone 自动下载；不改变网络层或权重。
  保存模型产生的原图二值实例掩膜，不填框、不用连通域伪造实例，不导出未经定义的安全降落面。

### CAT 运行统计（不是精度）

预热 3 次不计入统计，CUDA 同步计时；以下时延统计限 **43 张本地图像**：

| 指标 | 中位数 / P95 |
| --- | --- |
| 预测调用（预处理＋网络＋检测/掩膜后处理） | **1055.92 / 1179.12 ms** |
| 图像解码＋预测调用＋掩膜和两份标注图写盘 | **1100.70 / 1212.50 ms** |

第二行不含 JSON 写盘、bag 解码、ROS/RViz，也不是在线端到端时延。
当前配置本机约 1.06 秒一次预测，不能当轻量实时方案；不把该测量泛化为其他设备/配置的性能上限。
CUDA 运行曾提示 CuDNN 已应用 workaround；本轮未因此安装额外 CUDA 系统包，性能按实际环境记录。
PyTorch 峰值 allocated/reserved 为 **1521.74 / 3366.00 MiB**，覆盖预热后全部 48 张，
不是整卡显存；运行中一次 nvidia-smi 快照为 3456 MiB、GPU 利用率 95%，只是瞬时采样，不是平均/峰值统计。
本轮不是独占设备基准压测，推理期间还执行过该状态采样及约 1 秒的单元测试。

| 置信度 | 本地有船舶候选的帧数 | 本地候选实例数 |
| --- | --- | --- |
| 0.25 | 18 / 43 | 21 |
| 0.10 | 21 / 43 | 30 |

全体 48 张共保存 32 份 ship 实例掩膜（最低 0.10），其中 30 份来自本地，2 份来自公开对照。
48 条预测、96 张原图大小标注图及 32 份原图二值掩膜结构检查通过，掩膜面积与记录一致，无空掩膜。
**非空、候选数量和结构正确不等于识别正确**；未进行人工视觉核查，没有真值，不报告 mAP/召回率，用户质量尚未验收。

入口与复现：

- [汇总](../outputs/cat_mask_isaid_20261019_run02/summary.json)、
  [逐帧预测](../outputs/cat_mask_isaid_20261019_run02/predictions.jsonl)、
  [环境/代码散列](../outputs/cat_mask_isaid_20261019_run02/environment.json)、
  [完整依赖快照](../outputs/cat_mask_isaid_20261019_run02/requirements.freeze.txt)。
- [掩膜目录](../outputs/cat_mask_isaid_20261019_run02/masks)、
  [0.25 标注图](../outputs/cat_mask_isaid_20261019_run02/conf_0.25)、
  [0.10 标注图](../outputs/cat_mask_isaid_20261019_run02/conf_0.10)，供用户自行查看。
- [测试配置](../config/cat_mask_trial.yaml)、[依赖清单](../config/cat_requirements.txt)、
  [离线脚本](../scripts/cat_mask_trial.py)、[单元测试](../tests/test_cat_mask_trial.py)。

```bash
# 已准备好的本机独立环境；不 source 旧 ROS/Python 路径进入试验。
env -u PYTHONPATH -u LD_LIBRARY_PATH .venv-cat/bin/python scripts/cat_mask_trial.py --config config/cat_mask_trial.yaml
env -u PYTHONPATH -u LD_LIBRARY_PATH .venv-cat/bin/python -m unittest discover -s tests -v
```

当前输出目录已经存在，重跑会拒绝覆盖；需另存配置并改新目录。新电脑不能直接复制虚拟环境：
先用 Python 3.10 建独立环境、准备 pip/setuptools 80.10.2/wheel，再安装该依赖清单，
Torch 使用 `https://download.pytorch.org/whl/cu118`，MMCV wheel 使用
`https://download.openmmlab.com/mmcv/dist/cu118/torch2.0/index.html`，NNCore 需 `--no-build-isolation`。
模型源码、权重、依赖环境和产物均不随 Git 同步，不自动重下整套 iSAID 或开启训练。

### LiM-YOLO：训练及 OBB 代码存在，但缺公开 checkpoint

[作者仓库](https://github.com/egshkim/LiM-YOLO)固定版本
`8467bbadaf0330def798f8f781ba4c1eb709dc3c`，已取到 `models/LiM-YOLO/`，未安装覆盖现有 Ultralytics。
`ultralytics/cfg/models/v9/lim-yolo.yaml` 最后一层确认为 `OBB(P2,P3,P4)`，
自定义 GN-CBLinear 和 `ultralytics/examples/train.py` 训练入口均存在；仅做静态核查，未验证训练运行。

当前固定仓库树没有 `.pt/.pth/.onnx/.safetensors`，release API 返回空列表；推理示例为 `path/to/best.pt` 占位路径。
HF 检索 LiM-YOLO/lim-yolo 未找到对应作者训练模型；[权重相关 issue #1](https://github.com/egshkim/LiM-YOLO/issues/1)
也未提供权重回复。这里只能说本轮未找到，不宣称全网绝不存在。
静态证据/源码散列见 [LiM 预检记录](../outputs/lim_yolo_probe_20261019/preflight.json)。

因此未装新 Ultralytics、未随机初始化做效果展示、未借用官方 YOLO 权重冒充 LiM、未训练。
保留为“可训练 OBB、待作者权重/后续异机训练”候选；需要取得匹配的训练 checkpoint 和类别表才能继续实际识别测试。

### 本轮筛选与验证

按用户新要求，只推进有可获取训练实现且原生输出 **OBB 或像素掩膜** 的候选（满足一种输出即可）。
仅水平框的 MOD、RF-DETR Nano、MSRS/RT、MariSat YOLO、Sentinel-2 YOLO 已淘汰；
ScatYOLO 未核实配套训练实现，移出可执行候选，GrabCut 不作为可训练模型路线。
有像素输出和训练能力的 MariSat SAM3、MaskFormer、WaSR 无 IMU 版保留为非优先参考，不擅自扩展测试。
详见 [精简模型清单](待测试/models.md)与 D16；淘汰不删除已有文件/历史失败，不改 ROS 默认链路。

新旧两环境均通过 27 项单元测试；13 份 Python AST、86 个文档本地链接、输出结构/二值像素格式、权重散列、
重复输出拒绝覆盖和 `git diff --check` 均通过。没有人工视觉评判、标注、训练、ROS 构建或集成，未提交/推送。

## 2026-10-19 后续选型补充：Airbus 淘汰，WaSR 保留

- 用户指定 Airbus MaskFormer / Swin 淘汰，取代上节的非优先保留状态；不新增测试，也不称其已实测不合格。
- WaSR 无 IMU 版已核查可自行训练，按用户条件保留。固定作者版本、源码证据与数据要求见
  [模型清单](待测试/models.md)及 D17；未执行训练，不能将源码可训练当成本机/异机环境已验证。
- 默认输出障碍物/水面/天空语义掩膜，不是船舶实例；要训练需 RGB 与像素标注，当前 bag 无相应真值。
  本次仅文档筛选，不安装/下载模型或数据、不推理/训练/人工看图、不删除产物或改 ROS，未提交/推送。

## 2026-10-19：用户下载的 MariSat SAM3 权重测试（数值异常）

分支：`visual-only`，`db3bdec` 的未提交工作区；电脑：`htfp-crl`。
**结论：1118 个 state_dict 条目均可与模型匹配并加载，但当前输入权重本身有两个 `-inf`，
GPU 预热得到非有限掩膜；没有完成有效的 48 张推理，不作识别质量验收或精度判断。**
不是“48 张均未检出”，也没有出现 CUDA OOM。没有擅自清零/替换异常参数或用通用 SAM3 冒充微调结果。

### 输入、容器和加载校验

- 用户实际放入 `models/MariSat/sam3_semantic_best/` 的是解压后的 PyTorch 容器：
  `data.pkl`、`data/` 和元数据共 1124 个文件、2,651,931,193 B；原目录保持不变。
- 仅重新封装 ZIP_STORED 到
  `checkpoints/marisat_sam3_20261019/sam3_semantic_best_repacked.pt`，没有改写 pickle、存储字节或参数。
  封装文件 2,652,082,661 B，SHA-256 为 `8201a3bbf191d421b30533af9566921c8deb0466b57b2fd186dfeec205a93945`。
  每个源文件均记录 SHA-256、ZIP CRC 检查通过；这个散列是本机复原容器的散列，**不是作者原下载文件的散列**。
  原始压缩包和作者发布校验值未提供，不能确认异常产生于训练、导出、下载还是解压环节。
- 静态 pickle 只引用 OrderedDict、Torch tensor/storage 重建；使用 `weights_only=True`、CPU map 和 mmap 读取。
  权重共 1118 条：343 条 FP16、775 条 FP32，张量有效载荷共 2,651,670,232 B。
- 作者保存的是无 `detector.` 前缀的完整 SAM3 state_dict；当前库默认加载器会筛选此前缀。
  测试只适配加载入口，严格比较全部键/形状并 `strict=True, assign=True` 载入，不允许漏载后随机初始化推理。
  两次 GPU 预热前，1118 个条目全部匹配，加载阶段已通过。

### 环境与实际执行

复用已有 `.venv-sam3`，未安装或升级任何环境依赖：Python 3.12.3、Torch 2.10.0+cu128、
torchvision 0.25.0+cu128、NumPy 1.26.4、OpenCV 4.6.0；T600 Laptop 标称 4096 MiB，
Torch 可见 3717.94 MiB，驱动 595.99.02，CPU 线程 4。
只取得推理源码及 tokenizer 资源：官方 Ultralytics **v8.4.8**（`fbaaba3b7a386d348376b6dd2578f145a1e55714`，
`models/ultralytics_marisat/`）与 CLIP tokenizer（`7ffa84b3bfa40c42ecc1c77147a855e69cb2dd40`，
`models/CLIP_marisat/`），通过进程内路径引用，不覆盖现有已安装 Ultralytics。
未下载基础 SAM3/CLIP 模型权重，不需要重新申请 HF 访问，自动安装/联网模型加载关闭，未修改第三方源码。

准备复用此前的 43 张本地＋5 张公开对照，已成功生成 48 张样本清单。
输入保持作者 1008×1008、patch=14、72×72 特征网格，使用其 `finetuningSAM3.ipynb`
（版本 `a9a35dd74edb3fc66ce2f12f8edbc2b8b5e65d7c`）的 8 个英文航拍船型提示。
逐类文本解码，共享同一帧图像特征，不使用跟踪/历史帧、不追加 NMS、几何过滤或语义连通域分割。
计划分数为类别 sigmoid×存在性 sigmoid，0.10 导出、0.25 二次过滤，掩膜概率阈值 0.5、最多 200 个/图。
这些是配置/实现定义，**不代表本轮已经生成了有效掩膜**。

| 输出目录（`outputs/` 下） | 实际过程 | 结果 |
| --- | --- | --- |
| `marisat_sam3_20261019/` | 复原容器 | 源文件的 1979 时间戳不符合 ZIP 时间范围，初次封装失败；留下 22 B 未完成容器，不覆盖/删除 |
| `marisat_sam3_20261019_run02/` | 修正仅封装容器时间元数据；普通 FP16 GPU 预热 | 完整载入，第一张本地样本预热时输出非有限值，终止；正式样本计数 0 |
| `marisat_sam3_20261019_run03/` | FP16＋autocast 混合精度 GPU 预热 | sailboat 提示的 `pred_masks` 非有限，终止；正式样本计数 0 |
| `marisat_sam3_20261019_run04/` | 新增逐张量数值预检 | 原始权重检出 2 个 `-inf`，在 GPU 推理前拒绝继续，状态 `invalid_checkpoint_nonfinite` |

第二/第三次确实执行过第一张图的 GPU 预热，不是“从未计算图像”；但没有完成有效样本测试，
因此不导出正式预测/标注图/掩膜，不报告推理时延、mAP 或“零检出率”。
运行中一次 nvidia-smi 瞬时值为 2944 MiB / 100% 利用率；不是峰值/平均性能报告，也不能据此承诺完整测试显存够用。

### 直接来自原始存储的异常证据

异常键为 `backbone.language_backbone.encoder.text_projection`，FP16、形状 `[1024, 512]`；
`[0, 6]` 与 `[0, 10]` 两处均为 `-inf`。其他条目没有检出非有限值。
pickle 指向 `data/433`，其偏移 12、20 字节均为 little-endian `00fc`，
以 Python `struct.unpack('<e')` 不经 Torch 转换也读为负无穷。
这个源文件与复原 ZIP 对应成员逐字节一致，故异常不是本次重新封装、转 GPU 或 `.half()` 才产生。

- [完整数值预检](../outputs/marisat_sam3_20261019_run04/checkpoint_numeric_audit.json)、
  [原始 storage 独立字节证据](../outputs/marisat_sam3_20261019_run04/original_storage_evidence.json)
- [最后一次状态/异常](../outputs/marisat_sam3_20261019_run04/summary.json)、
  [环境与代码散列](../outputs/marisat_sam3_20261019_run04/environment.json)、
  [逐文件来源与复原散列](../outputs/marisat_sam3_20261019_run04/checkpoint_provenance.json)
- [FP16 预热异常](../outputs/marisat_sam3_20261019_run02/summary.json)、
  [混合精度预热异常](../outputs/marisat_sam3_20261019_run03/summary.json)、
  [已准备的样本](../outputs/marisat_sam3_20261019_run03/sample_manifest.json)
- [配置](../config/marisat_sam3_trial.yaml)、[离线脚本](../scripts/marisat_sam3_trial.py)、
  [测试](../tests/test_marisat_sam3_trial.py)。重新执行须改新的 `output_dir`，拒绝覆盖历史诊断。

下一步：需要重新取得能校验来源且所有张量有限的作者权重，或作者修复后的导出文件。
如果另行决定替换/修复某层，必须单独标为修改模型的诊断实验，不能当原始 MariSat 权重的测试结果；本次未做。

验证：34 项单元测试通过，覆盖容器往返、1979 时间戳、输入不变、来源变化拒绝复用、
严格加载和非有限值拒绝；语法、本地链接、来源散列/字节证据、拒绝覆盖和 `git diff --check` 检查通过。
未训练、人工看图、改 ROS 默认后端、其他分支或 stash；原始目录和所有失败记录保留，未提交/推送。
