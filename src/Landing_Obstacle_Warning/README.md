# landing_evaluator（ROS Noetic）

节点订阅 `sensor_msgs/PointCloud2` 和可选 `sensor_msgs/Imu`，发布
`landing_evaluator/LandingStatus` 与 `landing_evaluator/HeightMap`。核心计算位于
`src/evaluator.cpp`，不依赖 ROS。

```bash
roslaunch landing_evaluator landing_evaluator.launch
roslaunch landing_evaluator d1_test.launch
roslaunch landing_evaluator avia_test.launch
roslaunch landing_evaluator hil_sim.launch
```

运行中可通过服务或话题控制评估：

```bash
rosservice call /landing_evaluator/set_enabled "data: true"
rostopic pub -1 /landing_evaluator/enable std_msgs/Bool "data: false"
```

所有设备外参、雷达范围和机体半径均需在实机测试前校准。任何 `UNKNOWN` 状态都应被
下游控制器视为不可降落。
