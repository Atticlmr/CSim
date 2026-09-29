# Python 控制与执行器响应

C++ 专注物理引擎：接收实际总推力 `thrust`（N，沿 +Z_B）和质心处机体系力矩 `torque_B`（N·m），推进刚体、绳索和吊载动力学。CTBR、旋翼分配、限幅、响应、指令延迟及控制调度全部在独立的纯 Python 包 `csim_control` 中。

## 直接调用物理引擎

```python
import csim

model = csim.DroneModel(timestep=0.002)
data = csim.make_data(model, position_W=[0, 0, 5])
for _ in range(1000):
    csim.set_control(model, data, thrust=model.mass*model.gravity, torque_B=[0, 0, 0])
    csim.step(model, data)
print(csim.get_state(model, data))
```

保留名称 `set_control` 以兼容实际力输入脚本，它不执行控制算法，不做硬件限幅或响应。输入立即反映在加速度观测中，并在每次 `csim.step()` 的整个时间步保持。负推力、非有限值、错误形状及不满足当前绷紧绳条件的输入会被拒绝。仅依赖实际力历史就能复现物理轨迹，不需要任何控制器。

## 可选 Python 控制链

```python
import csim
from csim_control import ControlConfig, ControlLoop

model = csim.DroneModel(timestep=0.002)
data = csim.make_data(model, position_W=[0, 0, 5])
loop = ControlLoop(model, data, ControlConfig(
    mode="ctbr", rate_gain=[16, 16, 8],
    controller_period=0.004, command_delay=0.004,
    time_constants=[0.02, 0.01, 0.01, 0.01],
    max_thrust=30, max_torque=[1, 1, 0.5], max_rate=[3, 3, 3],
))
loop.set_ctbr(thrust=model.mass*model.gravity, body_rate_B=[0, 0, 0.3])
for _ in range(1000):
    loop.step()  # Python 控制/执行器 → set_control → C++ 物理 step
print(loop.get_state()["actuation"])
```

同样适用于 `SuspendedPayloadModel` 及 `load_model()` 返回的模型。`ControlConfig` 是不可变的 Python 参数对象；数组会复制成元组，`to_dict()` 返回独立、可修改的参数副本。几何、限幅和数值在配置构造时校验，步长相关的周期/延迟在 `ControlLoop` 构造时校验。不同 Data 配不同 loop，可共用一个物理 Model。

| Python 模式 | 提交方法 | 执行器通道 |
| --- | --- | --- |
| wrench（默认） | `loop.set_control(thrust, torque_B=[0,0,0])` | T、τx、τy、τz |
| ctbr | `loop.set_ctbr(thrust, body_rate_B)` | 角速度内环产生的 T、τx、τy、τz |
| rotor_thrust | `loop.set_rotor_thrusts(thrusts)` | 四个单旋翼推力 f0…f3 |

模式与命令必须匹配，body rate 单位 rad/s，机体系 FLU。Python 的 `loop.set_*` 只入队，不改变物理状态或推进时间，包括无延迟理想输入；在下一次 `loop.step()` 生效。直接原生 `csim.set_control` 则立即替换实际物理输入，两者语义明确区分。

使用 loop 时由它独占该 Data 的步进、输入和重置。不要再额外调用 `csim.step()`，否则每轮会推进两次；外部改写输入或时间会在下一次 loop 操作时被拒绝。查看状态和 `Viewer.sync(data)` 可照常使用。若自行管理控制循环，也可以完全不使用 `ControlLoop`。

## CTBR 与旋翼分配

Python 角速度 P 内环支持完整非对角惯性：

```text
ω_cmd_limited = clip(ω_cmd, ±max_rate)
α_cmd = diag(rate_gain) (ω_cmd_limited - ω_B)
τ_target = clip(J_B α_cmd + ω_B × (J_B ω_B), ±max_torque)
T_target = clip(T_cmd, 0, max_thrust)
```

`rate_gain` 默认 [8,8,4]，单位 1/s，有限非负；`gyroscopic_compensation=False` 可关闭补偿项。没有积分、滤波、抗饱和或飞控固件复刻，不把角速度指令直接写入实际角速度状态。

四个旋翼位置 `rotor_positions_B` 关于整机质心，默认顺序为 (0.2,0.2,0)、(-0.2,0.2,0)、(-0.2,-0.2,0)、(0.2,-0.2,0) m。全部推力沿 +Z_B。`rotor_moment_ratios` 默认 [0.015,-0.015,0.015,-0.015] m，是机体有符号反扭矩/推力比。倾转或冗余旋翼尚未支持。

```text
T = Σ f_i
τ_B = Σ (r_i × [0,0,f_i] + [0,0,k_i f_i])
A 的第 i 列 = [1, y_i, -x_i, k_i]^T
```

聚合映射也只在 Python 中计算。`allocation_mode="rotor"` 将 wrench/CTBR 的目标分配到四个旋翼，响应通道改为单旋翼。`rotor_thrust` 模式直接对各旋翼限幅，不再分配；总推力/力矩上限用于 wrench/CTBR，不对聚合值重复裁剪。

分配器求解 `min ||diag(weights)(A f - w_requested)||²`，约束 `0≤f_i≤max_rotor_thrust_i`。`allocation_weights` 默认四个 1，有限正值；可设 [1,5,5,2]，按 1/N、1/(N·m) 理解尺度和相对权重。最多枚举 81 个边界组合，在 Python 中用 Givens 旋转求自由变量最小二乘，不用法方程或逐旋翼裁剪替代优化。要求有效、非奇异四旋翼几何，目前不支持零上限的故障旋翼。

## 响应、延迟与数值语义

四个 `time_constants` 单位 s，0 表示理想通道；正值使用一阶指数响应。对每步保持不变的目标，Python 计算准确的执行器末值，以及区间平均值：

```text
u_end = u_target + (u_start-u_target) exp(-dt/tau)
u_mean = u_target + (u_start-u_target) (tau/dt) (1-exp(-dt/tau))
```

将 `u_mean` 聚合成实际推力/力矩，交给 C++ 在整个物理步保持；`u_end` 只留在 Python，作为下步响应初值。底层 RK4 不回调控制器，也不在 RK 中间阶段读取 Python 执行器状态。该近似保留固定姿态下的推力冲量；位移和姿态变化仍存在随步长收敛的离散误差。响应与物理耦合不再保证 RK4 四阶精度，需要做步长收敛检查。

这与旧版本在各 RK 阶段计算执行器力的结果可能不同，不承诺旧控制实验逐位一致。时间常数是待辨识参数，默认值不代表真实硬件。原生引擎只关心输入的实际力，不知道上述响应公式。

`command_delay` 必须为 timestep 的整数倍，`controller_period=0` 表示每物理步更新，非零值也须为整数倍。二者最大 100000 步。控制时钟用 Python 整数 tick；在 tick=k 提交，步骤 k+D 开始到达，CTBR 还需等到下一次控制采样点（0、N、2N…）。同一 tick 多次提交保留最后一次。墙钟和渲染频率不参与调度。

## 初始化、重置和失败处理

物理无人机默认零实际推力/力矩；吊载初始化必须显式提供保持正张力的初始输入，例如竖直静止时总重力。创建 loop 时从当前物理状态初始化响应；旋翼分支解 `A f = wrench` 并检查非负、旋翼上限和相应总力/力矩限制。初始实际力不经过响应或延迟。

`loop.reset(**initial)` 先验证新的物理状态与执行器初值，再调用原生重置，清空队列和采样历史。非法初值不改变两层状态。直接 `csim.reset()` 只负责物理数据；自行调用后应重新创建 loop。

`loop.step()` 在 Python 副本中消费队列、计算目标与响应，然后提交一个实际 wrench 并调用原生步进。原生步进失败时恢复原来的实际输入，Python 副本不提交。重复失败不会消费指令或推进控制时钟。未来绳索事件子步应在同一实际输入下执行，不在内部推进 Python 控制时钟。

## 状态与记录

`csim.get_state()` 只返回物理状态，`control` 字段是最近提交的实际力/力矩，加速度由此重算，不含 `actuation`。

`loop.get_state()` 额外附加 Python `actuation` 诊断：mode、commanded、delayed、target、applied、endpoint、saturated、pending_commands、tick、controller_updates。`applied` 是上一成功步使用的平均力，也等于原生 `control`；`endpoint` 是响应末值，不用于当前原生加速度观测。

旋翼分支另有 rotor_thrusts（响应末值）、target_rotor_thrusts、requested_wrench、allocation_residual、allocation_saturated、rotor_at_limit。`allocation_residual = target - requested_wrench`，thrust 残差允许为负；响应滞后不算分配残差。未启用旋翼分配时不伪造旋翼数据。

配置文件顶层 `model` 只放物理参数，`control` 存放 Python `ControlConfig.to_dict()`；实验外环参数仍放 `experiment.controller`。记录格式升级为 `csim-experiment-v2`，包含控制包源码哈希；旧 v1 记录需使用旧版本运行，不能仅改 schema 字符串。

## 迁移与后续开发

| 旧接口 | 新接口 |
| --- | --- |
| `csim.ControlConfig(...)` | `from csim_control import ControlConfig` |
| `DroneModel(control=config)` / `load_model(control=config)` | 创建纯物理模型后 `ControlLoop(model, data, config)` |
| `csim.set_ctbr(model, data, ...)` | `loop.set_ctbr(...)` |
| `csim.set_rotor_thrusts(model, data, ...)` | `loop.set_rotor_thrusts(...)` |
| 自动运行内环的 `csim.step(model, data)` | `loop.step()`，或自行计算实际 wrench 后调用原生 step |
| 原生 `get_state()["actuation"]` | `loop.get_state()["actuation"]` |

`uv pip install .` 同时安装 `csim_control`，不需要 NumPy/SciPy。`hover.py`、`tracking.py`、`payload_swing.py`、`payload_tracking.py` 均通过 Python 控制链运行；默认外环 100 Hz、Python 内环 250 Hz、物理 500 Hz。

后续 C++ 优先开发松紧绳切换、收紧冲击、接触、物理模型及数值求解。PID、UDE、MPC、旋翼转速模型、系统辨识及飞控执行器适配继续放在 Python/外部应用层。实时窗口和模型读取保持独立，详见 [Python viewer](python-viewer.md)、[可重复实验](reproducible-experiments.md) 与 [飞控接入](flight-controller-integration.md)。
