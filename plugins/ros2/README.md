# ROS2 插件预留位置

计划在此实现独立 `csim_ros2_bridge` 包；当前目录不包含可运行 ROS2 节点。
公共边界为 `csim_bridge.BridgePlugin`，插件安装后注册 `csim.bridges` 中的 `ros2` 工厂。

后续职责：

- 发布仿真时钟、本体与载荷真值、固定 link 状态以及传感器测量。
- 接收任务或明确约定的实际推力 / 力矩输入，使用有界队列交给仿真运行器。
- 配置 namespace、frame_id、坐标朝向、发布频率与 QoS。
- 重置时清除消息队列和时钟会话；节点关闭时回收 executor 和线程。
- 支持与 PX4 插件组合，分别承载任务 / 遥测与飞控物理闭环。

依赖如 `rclpy`、消息包和 ROS2 构建配置仅由此插件持有。
具体发行版、消息与 QoS 在实施时固定；不在核心 wheel 中内置 ROS2 运行环境。
CSim 的 W 坐标不能在未指定世界朝向时直接标记为 ENU。

PX4 的 ROS2 桥接承载 uORB 话题交换，仿真传感器输入与执行器输出通路应另外明确设计。
依据：[PX4 ROS2 用户指南](https://docs.px4.io/main/en/ros2/user_guide)、[坐标约定](https://docs.px4.io/main/en/ros2/user_guide#ros-2-px4-frame-conventions)。
