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
