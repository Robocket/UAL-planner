# UAL Planner：学习方法试验分支

当前分支为 `learning-methods`，由 `visual-only` 创建，工作目录仍为
`/home/htfp/UAL-planner`。原算法保存在 `visual-only` 分支中。

本分支从学习型视觉感知的选型开始：使用相机图像识别降落目标，输出二维 bounding box
和实例掩膜。船舶整体识别、甲板区域识别和区域是否满足降落条件是三个不同的任务。

本次只完成分支准备与开源项目调研，尚未安装模型、实现推理节点或开始训练。
原 SAM3/Q4_0/TorchAO 节点、分割消息、旧启动文件及 RViz 配置已从本分支移除。
保留 `.gitignore` 和 Jazzy 隔离构建脚本作为开发基础；当前 `src` 中没有 ROS 包。

## 调研与试验方向

完整调研见 [学习方法选型报告](docs/learning_methods_survey.md)，包括：

- 公开预训练模型验证：YOLOE 实例分割。
- 监督学习/迁移学习：RF-DETR 实例分割、RTMDet-Ins。
- 降落区域语义理解参考：SafeLand、OpenLander。
- 智驾系统中可复用的推理、检测和跟踪模块。

这些是候选路线，尚未确定最终模型。实测结果应以 Scene_Water 和当前硬件为准。

## 数据与配置

[config/scene_water.yaml](config/scene_water.yaml) 仅记录现有数据接口和相机标定占位值，
不属于可直接传给 ROS 节点的参数文件：

- 图像：`/left_camera/image/compressed`，`sensor_msgs/msg/CompressedImage`。
- 里程计：`/fsdk/aircraft_state`，`nav_msgs/msg/Odometry`，留待后续距离判断。
- 相机：1280 × 1024；内参暂沿用历史配置，尚需正式标定。

bag、训练数据、模型权重、虚拟环境和训练产物不提交到 Git。

## 后续开发约定

先在独立 Python 环境中验证公开模型的离线推理，再按需要实现训练和 ROS 包装。
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
