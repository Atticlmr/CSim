# PX4 插件预留位置

计划在此实现独立 `csim_px4_bridge` 包；当前目录尚未与 PX4 或飞控硬件通信。
公共边界为 `csim_bridge.BridgePlugin`，插件安装后注册 `csim.bridges` 中的 `px4` 工厂。

第一条候选路线为外部 Simulator MAVLink API：

```text
CSim 真值 -> 传感器模型 -> MAVLink -> PX4 估计器 / 控制器
CSim 物理 <- 实际推力 / 力矩 <- 执行器映射与响应 <- PX4 执行器输出
```

后续职责：

- 在固定固件版本上验证 `HIL_SENSOR`、`HIL_GPS` 和 `HIL_ACTUATOR_CONTROLS` 等所需消息。
- 配置 SITL / HITL 传输、系统和组件标识、通道映射以及解锁 / 停机处理。
- 将归一化执行器输出映射为单旋翼推力和反扭矩，再转换到 `PhysicalWrench`；归一化输出不能直接当作 N。
- 实现 NED / FRD 与 CSim W / FLU 之间的坐标转换，以及四元数方向和字段顺序转换。
- 独立实现传感器频率、噪声、延迟、时间戳和参考地理原点。
- 分别验证 SITL lockstep 与 HITL 实时节拍，覆盖丢包、断线、重复消息、重置和关闭。

MAVLink 依赖只由此插件声明；真实飞控控制链路中不额外启用 CSim 的 CTBR 控制器。
ROS2 任务 / 遥测可由相邻插件承担；具体组合运行器后续实现。
本次仅预留插件边界，不声称已具备 SITL/HITL 运行能力。

依据：[PX4 Simulator MAVLink API](https://docs.px4.io/main/en/simulation/#simulator-mavlink-api)。
