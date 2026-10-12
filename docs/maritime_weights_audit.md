# 海事轻量权重筛选与组合路线

核查日期：2026-10-12；分支：`visual-only`。
目标是光学 RGB 航拍船只，保留实例掩膜需求，不训练本地模型。

## 筛选结果

本次核查范围内，尚未确认到可立即复现、轻量、针对大型船舶俯视训练的直接实例分割权重。
这不代表不存在；仅表示下列公开资源不能同时满足全部条件。
用户已同意先离线验证“海事检测框 + 传统/小型提示分割”路线，不意味着允许用框充当掩膜。

| 公开资源 | 核查事实 | 本项目处理 |
| --- | --- | --- |
| [DetectionBench 的 SeaDronesSee YOLO11n](https://huggingface.co/dronefreak/seadronessee-yolo11n) | RGB UAV 海事数据训练；公开 `best.pt` 约 5.5 MB；`args.yaml` 的 task 为 detect | 已下载并在同一图像集做定位诊断；不称为实例模型，也不是数据集作者发布的官方基线 |
| [MobileNetV2-U-Net Airbus](https://github.com/sovden/airbus_ship_segmentation/tree/b2255c9ba879b543cb96db8712897c9854b4ea79) | 公开 `mobilenetv2_unet.h5` 为 30,113,784 字节；224 输入、单通道 sigmoid、旧 Keras/TensorFlow 实现 | 轻量备选；本质是船舶前景分割，不直接区分接触船舶；未安装或迁移旧环境 |
| [Airbus MaskFormer](https://huggingface.co/artemkramov/airbus-ship-challenge-segmentation) | 权重约 411 MB，config 为 MaskFormer + Swin，且类别正反映射残留不一致 | 不符合本轮小模型优先目标；未加载，不宣称该权重一定有问题或一定不适配 |
| [P2M/MariSat](https://github.com/amirabbes/P2M-Maritime-Segmentation) | 港口光学俯视场景相关，但 README 明确将 YOLO11n/l 列为 Detection，实例部分为 SAM3；共享权重链接未明确对应哪一模型 | 不把“项目做分割”推断成“存在 YOLO nano 实例权重”；未下载大模型或整个数据集 |
| [ScatYOLOv8+CBAM](https://elib.dlr.de/211646/) / [Ship3D 展示](https://dlr-mi.github.io/ship3d-demo/) | 海事实例分割与嵌入式研究相关；本次未确认可直接取得的对应轻量 checkpoint 和完整推理代码 | 保留论文参考，不将论文性能或网页演示当作已可部署的模型 |
| [HRSID 上的 YOLO 分割项目](https://github.com/marieltv/yolov8-seg-modified) | 训练数据 HRSID 是 SAR；本次代码树核查未找到 checkpoint | 不优先；SAR 不是当前海康 RGB 场景。[HRSID 作者定义](https://github.com/chaozhong2010/HRSID) |

文件大小来自仓库文件元数据，不等于运行显存或网络参数数。
Airbus 模型使用卫星数据，也不等于已适配本机近距离 UAV 船舱画面。
SeaDronesSee 模型卡的验证集指标不能替代本地精度，亦不是官方竞赛测试集成绩。
这里只引用资源类型和适用边界，不转述作者指标为本机实测。

## 本轮选择与边界

先用公开海事 YOLO11n 做自动定位，再比较两种补掩膜方法：

- **GrabCut**：只依赖现有 OpenCV，在框周围加入少量背景上下文，限制 ROI 边长后分割。
  不增加网络，不做固定颜色/船长假设；框外背景、低对比或截断可导致错误/空掩膜。
  [OpenCV 官方接口](https://docs.opencv.org/4.x/d8/d83/tutorial_py_grabcut.html)
- **MobileSAM**：使用公开轻量提示分割权重，由自动船舶框提供提示。
  它不是海事训练分类器，也不是 SAM3；没有文本提示或船舶语义确认能力。
  采用现有 Ultralytics 8.3.226 实现，无需额外安装另一套 SAM 框架。
  [作者实现](https://github.com/ChaoningZhang/MobileSAM)、[Ultralytics 接口](https://docs.ultralytics.com/models/mobile-sam/)

网络定位错误、框只包含船头/舱口时，补掩膜不会保证恢复完整船体。
掩膜生成方法也不会自动拒绝模型误认的车辆；输出只能标为候选实例。
语义前景 + 连通域并不保证相邻船舶分离，故未将其包装成直接实例模型。
本轮不添加水面过滤网络、KLT、RANSAC、测距、里程计融合或降落控制。

组合实测见 [离线组合试验报告](ship_hybrid_trial.md)。ROS 后端与启动默认值仍不变。
