# MASt3R 海事场景验证

日期：2026-10-11；分支：`learning-methods`。

## 选择依据与范围

用户已接受先用在海事航拍数据上评测过、但未针对船舶微调的模型做场景验证。
首轮选择 **MASt3R 官方预训练权重 + MTReD**。
MTReD 作者使用 MASt3R 评测海事航拍重建，公开图像与处理脚本；这不是海事专用训练模型。
[MTReD 作者仓库](https://github.com/RuiYiYong/MTReD)、[论文](https://arxiv.org/html/2503.00853v1)

本次直接调用上游两帧点图推理，输出彩色局部点云、预测 Z、置信度。
不是 MTReD 多视图全流程复现，不是 MASt3R-SLAM，不包含全局优化、跨窗口融合或动态船舶跟踪。
不恢复 SAM3、不接 ROS、不使用雷达或里程计、不添加降落算法。
bounding box / 实例分割仍为后续独立需求，MASt3R 本身不提供船舶实例标签。

代码采用 CC BY-NC-SA 4.0，权重还受训练数据额外条款约束；仅作研究验证，不据此认定可商用。
[MASt3R 代码与权重说明](https://github.com/naver/mast3r)、
[权重附加许可](https://github.com/naver/mast3r/blob/main/CHECKPOINTS_NOTICE)

## 固定模型与环境

- 上游：`naver/mast3r`，提交 `f5209afc300cec36239a7ac992263f36847bbba0`。
- DUSt3R 子模块：`3cc8c88c413bb9e34c41db0e0eef99c2ee010b12`；CroCo：`d7de0705845239092414480bd829228723bf20de`。
- 权重：`MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth`，2,754,910,614 字节。
- 本次官方下载 SHA256：`e28f91b488554653e2b46ddae9c78c1143e0bcb2e27d3e26cdb0b717f1568eb2`。
- 独立环境：`.venv-mast3r`；Python 3.12、Torch 2.5.1+cu124、torchvision 0.20.1+cu124。
- NVIDIA T600 Laptop，4096 MiB；当前配置为网络主体 FP16 权重、输出头 FP32 + FP16 autocast，**没有量化**。
  上游输出头会将 token 转回 FP32，因此不能直接将整个模型全转 FP16。
  Torch 实际报告的可用总显存约 3.63 GiB，小于 `nvidia-smi` 的标称容量。
- 未编译上游自定义 CUDA RoPE，使用其 PyTorch 回退实现，不能将本次耗时当作 CUDA 优化上限。
- 模型、环境、数据与产物已被 `.gitignore` 排除；不修改旧 `.venv-sam3` 或系统 Torch。

## 输入数据与核查

公开样例选用 MTReD `Dataset/03/0000.png`、`0061.png`，包含大型货船、海面及远岸。
数据版本：`110f4057acd055a5855d3891a6a9bb731856474f`。
MTReD 无真实三维真值，本次不报告米制深度误差或降落精度。

本地样例：`Scene_Water_20260805_013651` 的 `/left_camera/image/compressed`。
先在 5/30 秒抽样发现镜头前近距离物体，再抽样 60–600 秒，最终使用 **300/300.5 秒**船舶图像。
完整抽样记录：`datasets/scene_water_audit/manifest.json`。
抽样图像为 1280×1024，header `frame_id` 全为空；header 时间领先 bag 记录时间约 80 分钟。
不能据此填写 TF，或未经修正就与里程计同步。离线两帧试验按 bag 记录时间选帧。
原 bag 中偶有车辆画面，不等于已具备车辆训练/标注数据；本轮不评估车辆。

模型未输入历史内参或外参。两幅预测点图都表达在第一幅相机局部坐标系，第二幅无需重复外参变换。
即使 checkpoint 名含 `metric`，本场景尺度仍未核验；点云不得发布为世界坐标或直接作为安全测距。

## 测量口径

两帧一对、batch=1，预热 1 次、测量 3 次，每次 GPU 前后同步。
计时包含预处理后 CPU 图像到 GPU、模型推理、结果复制回 CPU；不包含文件读取、缩放、点云导出或绘图。
预处理另行记录，报告给出 PyTorch 峰值 allocated/reserved 显存；它们不是系统全部 GPU 显存。
单位为 **秒/图像对**及 **图像对/秒**，不冒充在线 SLAM FPS。

384/512 使用上游长边缩放及 patch 对齐中心裁剪。
224 使用短边缩放后中心方形裁剪，仅为资源诊断，可能截掉目标，不能当作 512 模型的等价输入。
仅过滤非有限值、低置信点和相机后方点，不实施海水剔除、实例聚类或甲板判定。
每项均保留完整点图及有效掩膜，以免过滤后的好看点云掩盖模型失败。

## 结果

原始基线使用 FP32 权重 + FP16 autocast，384/512/224 在两组图像上均显存不足。
随后禁用 autocast 权重缓存，仍然全部 OOM；不能将关闭缓存描述为已经解决问题。
失败报告分别在 `outputs/maritime_mast3r` 与 `outputs/maritime_mast3r_nocache`，未删除或覆盖。
最终转为 FP16 主体权重、保留 FP32 输出头并禁用缓存，结果单独保存在
`outputs/maritime_mast3r_fp16core/<dataset>_<size>/report.json`。

最终配置的实测结果如下。有效点比例是每幅完整图像通过阈值 3.0 的比例，不是船舶召回率或分割精度。

| 输入 | 实际处理尺寸 W×H | 中位秒/图像对 | 峰值 allocated / reserved MiB | 两视图有效点比例 |
| --- | --- | --- | --- | --- |
| MTReD 384 | 384×208 | 2.00 | 1882 / 1978 | 1.63% / 0.93% |
| MTReD 512 | 512×288 | 3.78 | 2071 / 2222 | 2.76% / 2.97% |
| MTReD 224（裁剪诊断） | 224×224 | 1.54 | 1797 / 1890 | 0% / 0% |
| Scene_Water 384 | 384×304 | 3.24 | 1989 / 2142 | 73.21% / 63.71% |
| Scene_Water 512 | 512×400 | 5.41 | 2232 / 2360 | 80.04% / 78.74% |
| Scene_Water 224（裁剪诊断） | 224×224 | 1.55 | 1797 / 1890 | 50.09% / 1.91% |

结论：

- 官方模型已在本机成功运行并产生局部点图/点云，但当前配置不能跟上约 8 Hz 的 bag 图像流。
  Scene_Water 384/512 只有约 0.31/0.18 图像对每秒，且未同时运行实例分割或 ROS 发布。
- MTReD 512 的高置信点主要集中在船附近，但仍可见膨胀/扭曲形态；海面与天空大多被置信过滤。
  这只是图像/点云观察，没有真值，不能报告几何误差或证明精细甲板可靠。
- Scene_Water 384/512 能显示船舱与岸边的相对表面变化。不同分辨率预测 Z 整体范围也不同，
  因而不能将色条数值解释为已验证的米制距离；两帧小基线的结果不代表持续运动下稳定重建。
- 224 虽快，公开货船样例没有保留点，本地两视图的保留比例也严重不均衡；不建议作为有效场景方案。
- 当前建议仅保留 MASt3R 为离线几何参考，尚未确定最终实时模型。实例分割、尺度校验及在线地图仍待实现。

本机预览：[MTReD 512](../outputs/maritime_mast3r_fp16core/mtred_512/preview.png)、
[Scene_Water 512](../outputs/maritime_mast3r_fp16core/scene_water_512/preview.png)、
[本地点云 PLY](../outputs/maritime_mast3r_fp16core/scene_water_512/pair_local.ply)。
这些产物仅保存在本机，未提交 Git。

每项产物：

- `preview.png`：预处理输入、置信过滤后的预测 Z、置信度。
- `cloud_preview.png`：局部点云示意。
- `pair_local.ply`：两帧点图导出的彩色点云，不代表融合后的全局地图。
- `pointmaps.npz`：两幅完整点图、置信度及过滤掩膜。
- `report.json`：配置、版本、校验和、时延、显存和有效点统计。

## 复现

从仓库根目录执行。已有环境无需重建；已有上游代码/数据/权重无需再次下载。
推理无需激活 SAM3 环境或旧 ROS 工作区。

```bash
python3 -m venv --without-pip .venv-mast3r
python3 -m pip --python .venv-mast3r/bin/python install pip
.venv-mast3r/bin/python -m pip install --timeout 15 --retries 2 \
  'torch==2.5.1+cu124' 'torchvision==0.20.1+cu124' \
  --index-url https://pypi.org/simple --extra-index-url https://download.pytorch.org/whl/cu124
.venv-mast3r/bin/python -m pip install -r scripts/requirements-mast3r-trial.txt

git clone --recursive https://github.com/naver/mast3r.git models/mast3r
git -C models/mast3r checkout f5209afc300cec36239a7ac992263f36847bbba0
git -C models/mast3r submodule update --init --recursive

mkdir -p checkpoints datasets/mtred_ship
curl --fail --location --retry 3 --connect-timeout 20 --max-time 1800 \
  --output checkpoints/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth \
  https://download.europe.naverlabs.com/ComputerVision/MASt3R/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth
curl --fail --location --output datasets/mtred_ship/scene03_0000.png \
  https://raw.githubusercontent.com/RuiYiYong/MTReD/110f4057acd055a5855d3891a6a9bb731856474f/Dataset/03/0000.png
curl --fail --location --output datasets/mtred_ship/scene03_0061.png \
  https://raw.githubusercontent.com/RuiYiYong/MTReD/110f4057acd055a5855d3891a6a9bb731856474f/Dataset/03/0061.png

# 图像抽取只需系统 Python + Jazzy 的 rosbag2_py，不需 Livox 消息绑定。
source /opt/ros/jazzy/setup.bash
python3 scripts/extract_reconstruction_frames.py
.venv-mast3r/bin/python scripts/mast3r_scene_trial.py --dataset all
```

参数全部在 [试验配置](../config/maritime_reconstruction_trial.yaml)。重复推理应改 `benchmark.output_dir`，
重复抽帧应改 `extraction.output_dir` 并对应修改输入图片路径；脚本拒绝覆盖既有产物。
设备可用性、上游版本及权重校验失败会明确报错，不静默切换 CPU 或另一个模型。

检查已通过：独立环境 `pip check`，3 项单元测试（含点云/图像导出），
两个脚本的 `py_compile` 与 `git diff --check`。
完整测试用 `.venv-mast3r/bin/python -m unittest discover -s tests -v`；
系统 Python 缺少 Torch 时只运行过滤测试，导出测试会跳过。

后续先补尺度/几何真值、跨帧一致性与目标掩膜验证，再决定在线模型及 ROS 接入；
这一轮结果不能证明运动船舶、精细甲板障碍或实时降落能力。
