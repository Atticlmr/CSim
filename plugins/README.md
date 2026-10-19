# 在环仿真桥接插件

这里预留独立插件的源码位置：`ros2/` 和 `px4/`。
当前提供 `csim_bridge` 接口和本地假桥接示例，尚未实现 ROS2 节点、PX4 通信、传感器或实时调度。
这两个目录目前只有接入约定；真实插件在后续实现时独立打包和安装。

```text
ROS2 任务 / 遥测 <--> ROS2 插件
                         |
                    无窗口仿真运行器
                         |
PX4 <--> PX4 插件 <--> 实际推力 / 力矩 <--> CSim
          ^                                 |
          +------- 传感器模型 <--- 真值 ------+
```

`csim_bridge` 随 CSim 的 Python 包安装，只依赖 Python 标准库。
导入它不会导入 ROS2、MAVLink 或 CSim 原生模块，也不会启动线程、网络或物理时钟。
物理引擎仍只处理实际推力和机体系力矩；坐标转换、消息协议、传感器噪声、执行器映射和响应属于插件层。
飞控闭环使用飞控自身的控制器，不能再叠加本地 CTBR 内环。

## 公共接口

插件按 `csim_bridge.BridgePlugin` 实现以下方法，不必继承协议类：

| 方法 | 约定 |
| --- | --- |
| `start(model_config)` | 接收 `csim.get_config(model)` 的独立配置副本，打开连接；构造函数不打开连接。 |
| `publish_state(state)` | 接收 `csim.get_state(model, data)` 的独立真值快照，时间单位为秒。 |
| `receive_wrench(simulation_time)` | 非阻塞轮询当前物理时刻可用的输入，返回 `PhysicalWrench` 或 `None`。 |
| `reset()` | 对应物理数据重置到零时刻，建立新会话，清除旧消息、序号和传感器历史。 |
| `close()` | 关闭连接并回收线程，可以重复调用；启动失败也必须允许清理。 |

`PhysicalWrench` 的总推力单位为 N，沿本体 `+Z_B`，必须非负；`torque_B` 为关于质心的 FLU 力矩，单位 N m。
`wrench.as_control()` 可直接传给 `csim.step(model, data, **wrench.as_control())`，控制和物理步整体提交。
该公共边界适用于现有无人机、质点吊载和刚体吊载标量模型；批量桥接、多机路由将在具体插件中另行扩展。
CSim 世界系 W 是局部 Z-up，不自动等同于地理 ENU；`q_WB` 为 Hamilton wxyz，表示 B 到 W。
世界系朝向、地理原点及目标协议坐标必须在插件配置中明确指定。

只有无窗口运行器持有和推进物理 Data，通信回调不能直接调用 `step/reset/set_control`。
SITL lockstep 中收到 `None` 时不推进物理时间，由外部调度器继续等待或报告超时。
HITL 的墙钟节拍、失联行为、输入有效期和超时策略需要单独实现与验证；不能把 `None` 自动解释为零推力。
真值快照不是 IMU 测量：加速度需要重力、坐标和传感器安装位置等转换，之后才可编码到目标消息。
viewer 可订阅快照，不能参与物理时钟或阻塞桥接通路。

## 插件发现

后续独立插件的 `pyproject.toml` 可注册一个工厂：

```toml
[project.entry-points."csim.bridges"]
ros2 = "csim_ros2_bridge:Ros2Bridge"
```

PX4 插件同样注册 `px4 = "csim_px4_bridge:Px4Bridge"`。
这些名称是预留约定，当前没有注册占位插件。
独立插件声明自己的 ROS2 / MAVLink 依赖，核心包不会替用户安装这些依赖。

```python
from csim_bridge import available_bridges, load_bridge

print(available_bridges())
bridge = load_bridge("px4", **transport_options)
```

加载只创建对象，调用 `start` 才打开连接；同名插件冲突或接口不完整会明确报错。
本地无需任何外部协议的示例：

```bash
xmake run csim_python examples/python/bridge_stub.py
```

后续实施约定分别见 [ROS2](ros2/README.md) 和 [PX4](px4/README.md)。
