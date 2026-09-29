# ROS Noetic 分支检查表

- [x] 工作空间根目录可直接由 `catkin_make` 构建，无额外嵌套工作空间。
- [x] 五个功能包改用 catkin、ROS 1 消息生成与 ROS 1 launch XML。
- [x] Landing C++ 核心 `evaluator.cpp` 和测试算法保持不变。
- [x] SAM Q4 推理、掩码后处理与追踪流程保持不变，仅迁移通信封装。
- [x] Python 数值计算流程保持不变，通过共用 rospy 接口层迁移节点生命周期。
- [x] D1、Avia、HIL、Scene_Water 统一入口完成 ROS 1 参数与话题适配。
- [x] Livox `CustomMsg` 到 `PointCloud2` 的 ROS 1 转换节点已提供。
- [ ] 使用目标机实际相机内参和雷达/相机/机体外参替换临时配置。
- [ ] 使用完整 SAM Q4 模型与现场数据进行端到端性能验收。
- [ ] 飞行前验证 `UNKNOWN` 的失效安全处理、看门狗和启停服务。
