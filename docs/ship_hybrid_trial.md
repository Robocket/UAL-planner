# 海事 YOLO11n + GrabCut/MobileSAM 离线试验

日期：2026-10-12；分支：`visual-only`。
用户已确认接受自动海事检测框 + 传统或小型提示分割的离线验证，仍需候选实例掩膜，不进行本地训练。

## 结论

**已跑通自动定位到原图实例掩膜的组合；MobileSAM 组合明显快于本次 GrabCut 配置，但整船识别仍未通过。**
海事检测器能够在 180 秒画面中识别船头，补掩膜可分出部分船头轮廓。
300 秒近距离图像则只定位驾驶室/船沿，480 秒有掩膜延伸至岸边；道路车辆也会误判成船。
所以“非空掩膜数量”不能当作识别正确率，也不能开始使用其面积变化估计整船距离。

当前均为候选二维实例，不是已确认整船、安全降落面或跨帧跟踪结果。
没有修改 ROS 后端或 launch，没有雷达/里程计输入，没有训练、KLT、RANSAC、测距或控制。
轻量权重资源核查见 [筛选报告](maritime_weights_audit.md)。

验证：10 项单元测试通过，脚本语法及 `git diff --check` 通过，环境依赖检查无冲突；
两个方法共 198 份导出掩膜已核对原图尺寸、二值编码及逐实例面积，统计与文件一致。
这些是实现/产物检查，不是识别精度验证。

## 模型与数据

检测器为 [DetectionBench 发布的 SeaDronesSee YOLO11n](https://huggingface.co/dronefreak/seadronessee-yolo11n)，
不是 Ultralytics COCO 权重，也不是 SeaDronesSee 数据集作者发布的官方检测基线。
下载固定版本 `8afc232ee3c50f82b3bd40458e2629a020597f26/best.pt`，5,498,266 字节，融合前参数 2,590,815。
实读类别为 swimmer/boat/jetski/life_saving_appliances/buoy，**boat 编号为 1，不是 COCO 编号 8**；
代码按类别名解析，不硬编码旧编号。

MobileSAM 来自 [Ultralytics v8.3.0 公共权重](https://github.com/ultralytics/assets/releases/download/v8.3.0/mobile_sam.pt)，
40,728,226 字节，实读参数 10,130,092。
它不是 SAM3，没有海事类别训练；检测框提供位置，MobileSAM 提供提示式掩膜，不对 boat 类别作二次确认。
通过现有 Ultralytics 8.3.226 运行，**没有新装第三套框架或替换 PyTorch**。
[MobileSAM 作者说明](https://github.com/ChaoningZhang/MobileSAM)、[框提示接口](https://docs.ultralytics.com/models/mobile-sam/)

两份 checkpoint SHA-256 已固定在根目录配置：

```text
SeaDronesSee YOLO11n: b6702b3af13103892abf42c2da08f417e3d58c791b57ab3b2ba5ebceb90f1b6a
MobileSAM:           6dbb90523a35330fedd7f1d3dfc66f995213d81b29a5ca8108dbcdd4e37d6c2f
```

输入与 [上一轮 YOLO11n-seg](yolo_scene_trial.md) 相同：43 张去重 Scene_Water 本地图像，
加 2 张 MTReD 航拍货船对照，总计 45 张；无人工框/掩膜真值。
全部使用自动框，不用手工框或点提示改善展示图。没有下载整个训练数据集。
设备 T600 Laptop / 4 GB，PyTorch 2.5.1+cu124，Python 3.12，FP32、CUDA 0。
YOLO 及 OpenCV CPU 线程各为 4；每组预热 3 次，每张重复测量 3 次。

## 1. 单独定位诊断

置信度 0.25，NMS IoU 0.7，只保留 boat。

| 检测输入边长 | 推理中位数 / P95 | 有 boat 输出的本地图像 | 公开对照 |
| --- | --- | --- | --- |
| 320 | 6.27 / 6.93 ms | 37 / 43 | 2 / 2 |
| 480 | 8.39 / 9.67 ms | 29 / 43 | 2 / 2 |
| 640 | 9.55 / 10.50 ms | 40 / 43 | 2 / 2 |

有输出不表示正确检出：60 秒车辆画面在 640 下产生多个 boat 框；300 秒框只覆盖局部船体。
MTReD 的同一艘货船也会产生多个局部框，不能用输出数量证明实例已正确分离。
尺寸变大不保证结果单调提升，这不是经过参数搜索得出的最佳分辨率。
单独诊断峰值 PyTorch allocated/reserved：640 下约 65.9/88 MiB，进程峰值 RSS 约 1113 MiB。

## 2. 自动框到掩膜的组合

共同检测输入为 640，阈值与上节一致，最多 20 框。
每次实测重新运行检测和掩膜生成，**不是相加两次独立报告的耗时**。

- GrabCut：每框外扩 10% 上下文，ROI 长边最多 384，3 次迭代；一像素边界确定为背景。
  若目标贴原图边缘，该规则可能截去真实前景，记录 `touches_image_border`。
  结果还原到原图坐标；空/失败掩膜不替换成矩形。
- MobileSAM：原生 1024 编码尺寸，整帧编码一次、批量框提示；默认单掩膜输出。
  每次测量清空图像特征缓存，包含重新编码，禁止将上一帧 embedding 复用到当前帧。
  quality 阈值为 0，仅便于观察候选掩膜；**SAM 的 quality 不是船舶类别置信度**。

| 组合 | 检测 + 掩膜中位数 | P95 | PyTorch 峰值 allocated / reserved | 进程峰值 RSS |
| --- | --- | --- | --- | --- |
| YOLO11n + GrabCut | 680.69 ms | 2645.59 ms | 65.9 / 88 MiB | 1118 MiB |
| YOLO11n + MobileSAM | 144.68 ms | 167.34 ms | 332.0 / 520 MiB | 1291 MiB |

GrabCut 掩膜阶段中位数约 666.84 ms；MobileSAM 约 133.43 ms。
两个方法都在 42/45 张图像中生成非空掩膜，共 99 个；其中本地 40/43 张、93 个候选。
三张无检测本地图像不执行分割；非空并不保证轮廓或类别正确。

耗时是每张三次同步 CPU/GPU 处理时间的中位数，再跨 45 张计算中位数与 P95。
计时从已解码图像开始，包含实时检测、掩膜生成/还原及回传 CPU。
**不包含**文件解码、标注绘制、PNG/JSON 导出、ROS/RViz。
PNG 解码中位数约 19 ms，标注和掩膜导出约 19–21 ms，应单列而不是宣称在线端到端帧率。
各阶段中位数不必严格相加等于整流水线中位数；进程 RSS 是该进程累计高水位。
PyTorch 内存统计不包含驱动等总显存，不能替代 `nvidia-smi`。

## 3. 目视检查与失效边界

- **180 秒**：自动框捕捉三艘相邻驳船的船头，GrabCut 和 MobileSAM 都能产生部分轮廓。
  船体延伸出提示框/画面，不把该面积当整船投影面积。
- **300 秒**：框仅覆盖左船驾驶室和中船狭长船沿；两种掩膜跟随局部框，均不是整船分割。
  驾驶室 MobileSAM quality 约 0.94，说明高 quality 也不等于选对了整船实例。
- **480 秒**：MobileSAM 某掩膜主要覆盖右侧岸边/船沿，而非当前船体。
- **60 秒负例**：检测框覆盖车辆/道路对象，补掩膜不具备海事语义拒绝能力。
- **MTReD 对照**：多个框分割同一货船的局部，不是全船掩膜；低阈值输出微小片段也不能视作可靠实例。

没有标注真值，不报告 mAP、召回率、IoU 或跟踪稳定性分数。
当前掩膜独立对应每帧每个提示，不做跨帧 ID 分配或实例去重；可能重叠或一船多实例。
任何输出均带 `semantic_class_confirmed_by_mask: false`、`safe_landing_surface: false`，
不能因文件存在或状态为 candidate 就送入降落控制。

## 文件与复现

- [定位配置](../config/seadronessee_localization_trial.yaml)：单独 detector 诊断。
- [组合配置](../config/ship_hybrid_trial.yaml)：阈值、ROI、迭代、线程、重复次数及导出路径。
- [组合脚本](../scripts/ship_hybrid_trial.py)：离线核心，不依赖 ROS。
- [共同预测脚本](../scripts/yolo_scene_trial.py)：默认仍测 segment；只有显式 `task: detect` 才做定位诊断，
  且 detect 模式禁止导出掩膜。

本机使用上一轮 `.venv-yolo`，其基础依赖只读复用 `.venv-mast3r`，环境边界见上一轮报告。
无 checkpoint 时按配置 URL 下载并校验哈希；网络失败或现存 `.part` 不会静默换模型。
重复测试先修改 YAML 的 `output_dir` 为尚未存在的路径，程序拒绝覆盖原报告：

```bash
.venv-yolo/bin/python scripts/yolo_scene_trial.py --config config/seadronessee_localization_trial.yaml
.venv-yolo/bin/python scripts/ship_hybrid_trial.py --config config/ship_hybrid_trial.yaml
.venv-yolo/bin/python -m unittest discover -s tests -v
```

结果保留于本机、被 git 忽略：

- `outputs/seadronessee_yolo11n_localization/summary.json` 和各尺寸逐帧 `report.json`。
- `outputs/ship_hybrid_trial_fp32/summary.json`、`provenance.json`。
- `outputs/ship_hybrid_trial_fp32/{grabcut,mobilesam}/report.json`。
- 每个方法的 `scene_water_audit_frame_002/annotated.jpg` 为船头样例，
  `scene_water_audit_frame_004/annotated.jpg` 为近景局部框失败样例；另有逐实例 `mask_*.png`。
- 调试期间首次 JSON 导出中断的目录已移至 `outputs/ship_hybrid_trial_fp32_incomplete_json` 保留；
  没有删除，**不属于正式结果**。

## 后续选择

本机优先保留 MobileSAM 组合作为低频候选掩膜基线，GrabCut 当前配置不推荐逐帧运行。
但还不能直接接在线测距：先解决大型/近景船体的整船定位、车辆误检和一船多框，再检验跨帧有效性。
不自动升级大模型，不训练本地权重，不将甲板或驾驶室默认当成新降落目标。
跟踪只能维护正确实例，不能通过平滑把局部错误掩膜变成整船。
