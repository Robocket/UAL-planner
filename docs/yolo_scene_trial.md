# YOLO11n-seg：Scene_Water 首轮离线试验

日期：2026-10-11；分支：`visual-only`。

## 结论

**小模型的速度足够开展后续实验，但当前 COCO 公开权重未通过本地俯视船舶识别验证。**
320/480/640 下默认阈值均未产生船舶实例；低阈值诊断只把两辆车误认成船。
同一模型能识别 MTReD 的斜视货船对照，因此不是下载失败、类别编号错误或 CUDA 无法运行。
这是基于本次有限样本的场景不匹配判断，不是对全部海事场景的精度结论。

本轮只新增离线提帧、分割、计时、导出及测试代码。
没有修改 `src/`、ROS 消息/话题、启动默认值；没有训练、量化、跟踪、测距或降落算法。
现有 SAM3 节点仍是 ROS 默认后端，不能将下表当作当前 ROS 节点帧率。

## 数据与实验设置

- 本地 bag：`/home/htfp/data/Scene_Water_20260805_013651`。
- 只读取 `/left_camera/image/compressed`；不读取雷达或里程计。
- 复用既有审计的 11 张 1280×1024 图像，偏移为 60、120、180、240、300、300.5、360、420、480、540、600 秒。
- 新抽取 180、300、420 秒起始的三个约 2 秒窗口，以 0.125 秒为请求间隔，获得 10/16/10 张唯一帧。
  时间戳间隙及 seek 命中同一帧会使数量少于理论 48 张；保留实际时间，不伪造连续帧。
- 两组按 bag 路径和记录时间去重，合计 **43 张**。这不是整段视频评测；部分图像无船。
- 无人工 bbox/mask 真值，不报告 mAP、召回率或“0% 精度”；这里只统计模型输出并人工检查典型错误。
- 诊断另加两张 MTReD 货船图像（1280×720），来自
  [作者 Dataset/03，固定版本](https://github.com/RuiYiYong/MTReD/tree/110f4057acd055a5855d3891a6a9bb731856474f/Dataset/03)，
  文件为 `0000.png`、`0061.png`，本地命名 `scene03_0000.png`、`scene03_0061.png`。

设备：NVIDIA T600 Laptop，标称 4 GB；PyTorch 可见容量约 3718 MiB。
Python 3.12、PyTorch 2.5.1+cu124、Ultralytics 8.3.226，FP32、CUDA 0、CPU 线程数 4。
每档预热 3 次，每张测量 3 次；保持纵横比 letterbox 到方形输入，输出掩膜还原至原图大小并校验。
默认只保留 `boat`（编号 8），置信度 0.25，NMS IoU 0.7，最大实例数 20。

使用 [Ultralytics 官方 YOLO11n-seg checkpoint](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-seg.pt)，
不是本地训练模型。下载大小 6,182,636 字节（约 5.90 MiB），融合前参数数 2,876,848。
SHA-256：`55ed65c56c91713d23e8402371c6c49a6fd84f257f7dce452e8d70e41dcbe152`。
首次下载记录哈希后，已在两份配置中固定该值，用于后续加载校验。

## 默认阈值结果

| 输入边长 | 推理中位数 | P95 | PyTorch 峰值 allocated / reserved | 有 boat 输出的本地帧 |
| --- | --- | --- | --- | --- |
| 320 | 6.14 ms | 6.94 ms | 50.8 / 72 MiB | 0 / 43 |
| 480 | 7.61 ms | 9.73 ms | 60.6 / 68 MiB | 0 / 43 |
| 640 | 11.21 ms | 11.49 ms | 75.3 / 90 MiB | 0 / 43 |

每帧先取三次同步预测耗时的中位数，再对 43 帧取中位数/P95。
计时包含预处理、网络、后处理及结果回传 CPU，CUDA 前后同步；**不包含**图像解码、标注渲染、文件导出或 ROS/RViz。
PNG 解码中位数约 19 ms，空结果标注/图像导出约 4.2 ms，进程峰值 RSS 约 1138 MiB（1.11 GiB）。
PyTorch allocated/reserved 不包含驱动等全部显存，也不等于 `nvidia-smi` 的进程总显存。

默认测试均无实例，后处理和原图掩膜负载很低；不能将倒数耗时宣称为稳定、有效船舶分割的在线 FPS。
短窗口尚未形成可用船舶掩膜序列，因此未报告粘连、边界抖动或跨帧身份稳定性。

## 低阈值与公开货船对照

640 输入，置信度改为 0.10、开放全部 COCO 类别，仅用于诊断，不改变主配置的目标输出约定。

- 43 张本地图像中，只有 60 秒图像出现两个 `boat`，置信度约 0.138/0.112。
  人工查看原图与标注，二者实际都是路边车辆，**不是有效船舶检出**。
- 在 300 秒俯视驳船图像中，船舱还被误标为 `parking meter`（约 0.14）。
  其他类别输出不能一概算假阳性，例如船上可能确实有人；这里仅记录已目视确认的典型错误。
- 两张 MTReD 对照均检出货船，置信度约 **0.773 / 0.666**，输出非空原图尺寸掩膜。
  证明推理链路有识别船舶的能力，但不证明本地俯视甲板场景可用。

这组 45 张图像的预测中位数/P95 为 13.73/16.41 ms；PyTorch 峰值 allocated/reserved
约 161.6/244 MiB，进程峰值 RSS 约 1.36 GiB。
标注与掩膜导出中位数约 34.7 ms，显示有实例时的 CPU 可视化/导出负载不可忽略。
全部类别与样本组成不同，这组不能当作与默认阈值同条件的精度或速度提升比较。

本机结果位置（输出被 git 忽略，不包含在提交中）：

- `outputs/yolo11n_scene_water_fp32/summary.json`：三档汇总。
- `outputs/yolo11n_scene_water_fp32/size_640/report.json`：逐帧耗时、实例信息及输入时间。
- `outputs/yolo11n_scene_water_diagnostic/size_640/report.json`：诊断和公开对照。
- `outputs/yolo11n_scene_water_diagnostic/size_640/scene_water_audit_frame_000/annotated.jpg`：车辆误判。
- `outputs/yolo11n_scene_water_diagnostic/size_640/control_scene03_0000/annotated.jpg`：公开货船正对照。
- 每张图像目录中的 `prediction.json`、`annotated.jpg` 与非空结果的 `mask_*.png`。
  `frame_instance_index` 仅是当前帧顺序号，不是跨帧跟踪 ID。

## 配置与复现

主配置：[yolo_scene_trial.yaml](../config/yolo_scene_trial.yaml)；
诊断配置：[yolo_scene_trial_diagnostic.yaml](../config/yolo_scene_trial_diagnostic.yaml)。
参数集中在根目录 `config/`，与 ROS 参数文件分开；图像尺寸、阈值、类别、线程、重复次数和路径均可修改。
此实验当前明确要求 CUDA 0，不提供静默 CPU 回退。配置应放在项目根目录的 `config/` 下。

本机已建立 `.venv-yolo`，YOLO 专属依赖独立安装，基础 CUDA PyTorch 等通过
`.venv-yolo/lib/python3.12/site-packages/ual_readonly_base.pth` 只读复用 `.venv-mast3r` 的 site-packages。
未修改 `.venv-sam3` 或 `.venv-mast3r`；但该本机环境**依赖基础环境路径存在**，不是可直接拷贝的独立环境。
无需加载或下载 MASt3R 模型。

```bash
# 本机环境；先在对应 YAML 中将 output_dir 改成未存在的目录
.venv-yolo/bin/python scripts/yolo_scene_trial.py
.venv-yolo/bin/python scripts/yolo_scene_trial.py --config config/yolo_scene_trial_diagnostic.yaml
.venv-yolo/bin/python -m unittest discover -s tests -v
```

重新提取短窗口使用 Jazzy 的系统 Python，而不是 YOLO 虚拟环境：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/extract_yolo_frames.py --config config/yolo_scene_trial.yaml
```

提帧与推理均拒绝覆盖已有输出；重复实验前修改相关 output_dir 和输入 manifest 路径。
已有的 `datasets/scene_water_audit/manifest.json` 为此前审计产物，不由本轮短窗口脚本生成。
新机器没有该审计集时，可先将 `dataset.manifests` 改成只包含新提取的短窗口 manifest；
此时测量样本为 36 张，不应与本报告 43 张混用。诊断还需准备上文两张公开对照图像。

需要完全独立的环境时，可在本机支持的 CUDA 条件下重新安装同版本基础依赖：

```bash
python3 -m venv .venv-yolo-independent
.venv-yolo-independent/bin/python -m pip install \
  torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu124
.venv-yolo-independent/bin/python -m pip install -r scripts/requirements-yolo-trial.txt
```

此独立环境示例未在本轮另装一遍；实际验证的是上述只读复用环境。
模型、虚拟环境、数据和输出保留本机且被忽略；脚本、配置、测试和本报告可纳入版本控制。

## 下一步边界

优先寻找有公开海事/俯视船舶训练权重的轻量实例模型，用同一输入集先验证识别效果。
不把单纯调低阈值作为解决方案，不自动训练本地模型或扩大数据下载。
有稳定、正确实例后，才实施 KLT/RANSAC 对照；标定、位姿尺度和时间轴通过检查后再实现稀疏几何。
图像 frame_id 为空、header 与 bag 时间差约 80 分钟的历史问题仍未修复，本次不涉及世界坐标或测距。
