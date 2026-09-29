# 用 Python 开展吊载轨迹跟踪

控制链现已全部迁移到 Python `ControlLoop`；C++ 仅接收每步实际推力/力矩。下文既有实验数值为迁移前记录，新版平均力耦合可能产生小幅差异，重新运行后应保存新的结果。


可直接运行的入口是 [examples/python/payload_tracking.py](../examples/python/payload_tracking.py)。模型创建、轨迹函数、控制器 `tracking_command()`、仿真循环和状态记录均写在该文件里；四元数与向量运算复用 [flight_control.py](../bindings/python/csim_experiments/flight_control.py)。默认参考峰值速度已从原示例约 **0.156 m/s 提高到 1.0 m/s**，并加入绳索倾斜和姿态角速度前馈。

## 运行

在项目根目录、已安装 `csim-drone` 的虚拟环境中执行：

```bash
.venv/bin/python examples/python/payload_tracking.py --speed 1.0
```

默认显示 20 秒实验：载荷在 4 m 高度跟踪平滑曲线，水平范围为 x ∈ [0, 1] m、y ∈ [0, 0.5] m。启动的前 4 秒平滑增加轨迹相位速度，随后参考线速度周期变化，峰值为 1.0 m/s。无人机随加速度需求倾斜绳索，其高度也随绳索方向调整。

```bash
# 降低峰值速度，便于观察
.venv/bin/python examples/python/payload_tracking.py --speed 0.5

# 无窗口快速计算，输出载荷跟踪误差与最终状态
.venv/bin/python examples/python/payload_tracking.py --headless --speed 1.0

# 记录每个物理步的参考轨迹、载荷位置/速度/加速度和张力
.venv/bin/python examples/python/payload_tracking.py --headless --speed 1.0 --csv payload_tracking.csv

# 修改时长和显示间隔；物理步长保持 0.002 s
.venv/bin/python examples/python/payload_tracking.py --speed 1.0 --ramp-time 6 --duration 30 --render-every 10
```

`--speed` 单位为 m/s，控制参考轨迹在启动结束后的峰值速度，实际载荷可能存在超调。`--ramp-time` 只调整启动过程，不改变之后的峰值速度。`--render-every` 只改变显示频率，物理步长和控制周期固定；`--headless` 去掉实时等待，以计算机允许的速度计算同一物理过程。

CSV 文件要求尚不存在，避免覆盖实验结果。除每步的参考位置、载荷位置/速度/加速度和张力，还记录参考速度、位置误差、摆角（rad）、已施加推力及执行器饱和标志。该示例的 CSV 不包含全部模型参数和控制指令；需要完整参数、指令记录及精确回放时，参考 [可重复实验框架](reproducible-experiments.md)。

## 吊载状态如何获取

```python
state = csim.get_state(model, data)
payload_position = state['payload_position_W']
payload_velocity = state['payload_velocity_W']
payload_acceleration = state['payload_acceleration_W']
direction = state['cable_direction_W']
tension = state['tension']
```

| 字段 | 含义与单位 |
| --- | --- |
| `position_W` / `velocity_W` | 无人机质心位置 m / 速度 m/s |
| `q_WB` | 无人机姿态四元数，顺序 wxyz |
| `payload_position_W` | 载荷质点位置，m |
| `payload_velocity_W` | 载荷速度，m/s |
| `payload_acceleration_W` | 当前状态和已施加输入下的载荷加速度，m/s² |
| `cable_direction_W` | 无人机指向载荷的单位方向 |
| `cable_angular_velocity_W` | 绳方向的角速度，rad/s，不是载荷刚体角速度 |
| `tension` | 绳张力大小，N |
| `cable_force_on_payload_W` | 绳索作用于载荷的力，N |
| `mode` | 当前模型为 `taut` |

`W` 为局部 Z-up 世界系。`get_state()` 返回独立快照，不推进时间；修改字典不会修改仿真状态。每次 `step()` 后重新读取，才能得到新状态。

当前载荷是质点，因此没有载荷姿态或刚体转动惯性状态。需要载荷姿态控制时，需扩展刚体吊载模型。

## 控制器的含义

提高同一路径的速度会同时提高加速度和绳索倾斜需求，因此不能继续仅使用竖直绳长偏置。`payload_reference(t, speed, ramp_time)` 返回五个三维向量：位置、速度、加速度、jerk（位置三阶导数）、snap（位置四阶导数）。启动阶段使用平滑多项式，使这些导数在启动衔接处连续。

设世界系重力向量为 `gW = [0, 0, -g]`，绳方向 `s` 从无人机指向载荷。在无空气力、绳索绷紧的理想参考运动中，载荷受力满足 `mL*aL = mL*gW - T*s`，因此：

```text
s_ref = -(aL_ref - gW) / |aL_ref - gW|
pQ_ref = pL_ref - length * s_ref
vQ_ref = vL_ref - length * s_ref_dot
aQ_ref = aL_ref - length * s_ref_ddot
```

`cable_reference()` 使用加速度、jerk 和 snap 解析计算 `s_ref` 及前两阶时间导数。两质量的理想总推力前馈为：

```text
F_ref = mQ * (aQ_ref - gW) + mL * (aL_ref - gW)
      = (mQ + mL) * (aL_ref - gW) - mQ * length * s_ref_ddot
```

反馈部分将载荷位置误差加入无人机位置目标：

```text
pQ_target = pQ_ref + 0.5 * (pL_ref - pL)
```

在质量归一化的推力前馈上叠加无人机位置 PD（位置增益 2.0、速度增益 2.5），以及水平绳方向变化率误差反馈 `0.25 * (s_dot - s_ref_dot)`。这样抑制的是相对于计划运动的摆动，而不是要求快速转弯时仍保持竖直绳。载荷位置修正项的导数没有再加入速度目标。

推力方向确定目标姿态；姿态误差反馈叠加相邻目标姿态四元数得到的角速度前馈，再转换到当前机体系，输出 CTBR 的总推力与期望机体角速度。姿态反馈增益为 10.0，机体角速度指令逐轴限制在 ±3 rad/s；质量归一化推力指令在加重力前逐轴限制为 ±[6, 6, 4] m/s²。Python 控制链另有 30 N 总推力和 [1, 1, 0.5] N·m 力矩上限。

这是含理想吊载运动前馈的串级控制示例，并非完整复现 RotorTM 几何控制，也没有给出任意工况下的稳定性保证。前馈假设质点载荷、固定绳长、质心悬挂且没有空气力；改变物理模型后需调整前馈和控制器。

默认物理频率 500 Hz、Python 外环 100 Hz、Python CTBR 内环 250 Hz，含 4 ms 指令延迟和直接推力/力矩通道的一阶响应（推力时间常数 20 ms，力矩逐轴 10 ms）。本例使用直接推力/力矩响应，没有启用单旋翼分配；SPAD、风场、空气阻力默认关闭。

已有 `tracking.py` 默认使用无吊载无人机；即使通过配置启用载荷，其现有位置误差指标仍针对无人机。这个独立示例的参考和 `payload_position_rmse_m` 明确针对载荷。

## 修改自己的实验

1. 修改 `SuspendedPayloadModel(...)` 中的质量、绳长等参数。
2. 修改 `payload_reference(t, speed, ramp_time)`，返回相互一致的位置、速度、加速度、jerk 和 snap，避免起点突跳；初态需与参考运动匹配。若支持力 `aL_ref - gW` 为零，则这里的绷紧参考方向不再有定义。
3. 在 `tracking_command()` 中替换控制逻辑；保持使用 `loop.set_ctbr()` 和 `loop.step()`。
4. 从 `payload_*` 字段计算载荷误差，另行记录无人机误差和张力。
5. 用 CSV 对比参考与实际轨迹，再逐步增加摆角、速度和扰动。

当前物理模型要求持续正张力；进入非正张力工况时，脚本停止，输出 `status: failed`、失败原因和最后成功步的状态与指标，并以退出码 1 结束。失败步不会写入 CSV。松绳和收紧冲击尚未实现，此示例也没有通用接触求解。参数校验允许输入更高速度用于实验，但不表示该速度在当前路径、控制器和执行器约束下可行。

## 本次运行验证

2026-09-14，在项目现有 `.venv` 中运行 20 秒无窗口实验，启动时间均为 4 秒：

| 参考峰值速度 | 实际峰值速度 | 载荷位置 RMSE | 最大位置误差 | 最大偏离竖直角度 | 最小张力 |
| --- | --- | --- | --- | --- | --- |
| 0.5 m/s | 0.502 m/s | 0.220 cm | 0.392 cm | 4.18° | 1.9613 N |
| 1.0 m/s | 1.072 m/s | 1.843 cm | 3.105 cm | 17.33° | 1.9613 N |

两组实验的执行器饱和比例和 Python 控制器限幅比例均为 0。RMSE 对整个仿真区间的位置误差平方进行梯形积分再开方，包含启动阶段。最大摆角包含快速运动所需的计划绳索倾斜，不能直接当作多余摆动大小。

在相同参数下设置 `--speed 1.5 --duration 8`，约 4.168 秒时触发非正张力检查而停止，之前已经出现限幅和明显误差。因此当前建议从 0.5–1.0 m/s 开始；更高速度需要联合调整路径曲率、控制器和执行器约束，单独提高速度参数不能保证跟踪效果。以上是该数值模型的结果，不代表实机控制性能；本次验证未启动图形窗口。

回归测试位于 [tests/python/test_payload_tracking.py](../tests/python/test_payload_tracking.py)，覆盖轨迹导数、绳方向导数与单位球约束、两档速度的闭环跟踪、CSV 指标复算及失败状态记录。
