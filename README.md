# UAL Planner：学习方法试验分支

当前分支为 `learning-methods`，由 `visual-only` 创建，工作目录仍为
`/home/htfp/UAL-planner`。原算法保存在 `visual-only` 分支中。

本分支先研究单相机纯视觉、模型驱动的在线三维重建，不采用雷达融合检测。
目标是从连续图像获得深度/三维点及空间几何，不沿用原来的实例尺寸变化测距算法。
当前降落目标以船只为主。
车辆保留为后续扩展类别，但目前没有自采车辆数据，不纳入首轮训练或效果承诺。
实例分割需求继续保留；重建模型不默认提供船舶或甲板实例标签。
下游空间计算仍以世界坐标为目标；尺度/变换未核验前只输出明确标记的局部/重建坐标，不能直接改 frame_id。

已选择 MASt3R 做第一轮离线海事场景验证：使用公开预训练权重，不进行船舶微调。
独立环境和官方权重已安装，MTReD 货船及 Scene_Water 两帧试验已完成。
Scene_Water 384/512 配置约 3.24/5.41 秒每图像对，当前不能跟上约 8 Hz 图像流。
不等于在线重建节点或完整 SLAM，不包含降落控制算法，实例分割尚未接入。
原 SAM3/Q4_0/TorchAO 节点、分割消息、旧启动文件及 RViz 配置已从本分支移除。
保留 `.gitignore` 和 Jazzy 隔离构建脚本作为开发基础；当前 `src` 中没有 ROS 包。

## 调研与试验方向

完整调研见 [学习方法选型报告](docs/learning_methods_survey.md)，包括：

- 首个场景实验：MASt3R，在 MTReD 海事航拍数据上已有作者评测；先验证 MTReD 与 Scene_Water。
- 完整在线重建候选：MASt3R-SLAM、SLAM3R；其他流式模型作为对照。
- 资源受限的逐帧局部几何基线：Depth Anything V2 Metric Small，不等于完整 SLAM。
- 图像实例分割候选：YOLOE、RF-DETR、RTMDet-Ins。
- 单目尺度、世界坐标、运动船只及端到端延迟的验证条件。

本机为 T600 Laptop、4 GiB 显存；论文“实时”帧率不能直接套用。
这些是候选路线，尚未确定最终在线模型；第一轮试验见 [海事场景验证](docs/maritime_scene_trial.md)。
上一轮相机 + 雷达检测路线仅作为 [历史参考](docs/multimodal_detection_reference.md) 保留，不进入当前开发计划。

## 数据与配置

[config/scene_water.yaml](config/scene_water.yaml) 仅记录现有数据接口和相机标定占位值，
不属于可直接传给 ROS 节点的参数文件：

- 图像：`/left_camera/image/compressed`，`sensor_msgs/msg/CompressedImage`。
- 里程计：`/fsdk/aircraft_state`，`nav_msgs/msg/Odometry`，仅保留为可选尺度/世界坐标辅助，首轮默认关闭且来源待核验。
- 雷达及雷达 IMU 不参与在线模型输入；雷达只记录为默认关闭的离线评测参考。
- 相机：1280 × 1024；内参暂沿用历史配置，尚需正式标定。

相机 frame_id、机体/光学坐标约定和时间同步仍需核验。
本轮抽样发现压缩图像 frame_id 为空，header 与 bag 时间相差约 80 分钟；不能直接用于 TF/同步。
纯单目结果不能默认有可靠米制尺度；启用外部里程计辅助时须与严格 RGB-only 结果分开评测。

bag、训练数据、模型权重、虚拟环境和训练产物不提交到 Git。

## 后续开发约定

独立环境为 `.venv-mast3r`，试验参数集中在 [config/maritime_reconstruction_trial.yaml](config/maritime_reconstruction_trial.yaml)。
复现安装、图像抽取及推理命令见试验报告。先确认尺度、运动目标和资源限制，再决定在线接入方式。
公开预训练权重不等于已验证船舶甲板几何；第一轮不要求自行训练。
实验环境不复用旧的 SAM3 虚拟环境或安装产物。
现有未跟踪的 `build`、`install` 和虚拟环境目录未清理；其中旧节点产物不属于本分支的实现。

ROS 2 Jazzy 为开发基线；Noetic 通过独立包装和 launch 适配。
边界见 [ROS 兼容约定](docs/ros_compatibility.md)。

切回原纯视觉实现：

```bash
git switch visual-only
```

切回本分支：

```bash
git switch learning-methods
```
