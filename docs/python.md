# Python 仿真接口

Python 负责配置实验、设置控制、推进仿真并读取物理状态，C++ 负责动力学及所有底层数值运算。
接口采用类似 MuJoCo `mj_step(model, data)` 的仿真边界：一次调用完成一个物理时间步。
MuJoCo 将模型描述与运行数据分开，`mj_step` 计算动力学并推进状态和时间；本项目借鉴这一组织方式，不引入 MuJoCo 依赖或承诺兼容其 API。
参考 [MuJoCo 仿真循环说明](https://mujoco.readthedocs.io/en/stable/programming/simulation.html)。

## 当前已实现

模块 `csim` 已提供 [固定悬点平面单摆](pendulum.md)、[无吊载无人机](drone.md) 与 [绳索质点吊载耦合](suspended-payload.md)，吊载可选严格绷紧或绷紧/松弛混合模式。
物理引擎的数学、旋转运算和数值积分不作为 Python 接口导出；Python 控制器自行计算所需的反馈和分配。原生无人机接口只接收实际推力/力矩；CTBR、单旋翼输入、响应和延迟由纯 Python `csim_control` 实现；URDF/MJCF 固定模型加载已实现，吊载通过 `cable_mode="hybrid"` 支持松弛和重新绷紧。

无吊载示例可直接运行 `xmake run csim_python examples/python/drone.py`。
实时窗口见 [Python viewer](python-viewer.md)，文件加载见 [URDF/MJCF](model-import.md)，控制配置见 [ControlConfig](control-and-response-models.md)。

```python
import csim

model = csim.PendulumModel(length=1, mass=1, timestep=0.01)
data = csim.make_data(model, angle=0.7)
csim.step(model, data)
state = csim.get_state(model, data)
print(state['time'], state['position_W'], state['tension'])
csim.reset(model, data, angle=0.1, angular_velocity=0)
```

## 单摆接口

| 接口 | 行为 |
| --- | --- |
| `PendulumModel(length=1, mass=1, gravity=9.80665, timestep=0.001, pivot_W=(0,0,0))` | 构造并验证不可变模型，量纲依次为 m、kg、m/s²、s、m |
| `make_data(model, angle=0, angular_velocity=0)` | 创建独立的 `PendulumData`，初始时间 0，角度与角速度为 rad、rad/s |
| `step(model, data)` | C++ 按所选积分器推进一个 `model.timestep`，成功返回 `None` |
| `get_state(model, data)` | 返回当前状态的独立字典快照，不推进时间 |
| `reset(model, data, angle=0, angular_velocity=0)` | 验证后设置状态并将时间重置为 0，成功返回 `None` |

模型的 `length`、`mass`、`gravity`、`timestep`、`pivot_W` 为只读属性；悬点读出为列表副本。
运行数据只能经 `make_data` 创建，不直接开放内部状态修改。`data.model` 返回该运行数据持有的模型。
一个模型可创建多份独立数据；参数相同的另一个模型也不能与当前数据混用。
数据在 C++ 中持有模型共享所有权，删除原 Python 模型变量后仍可通过 `data.model` 操作。

一步的全部阶段都在 C++ 完成，不接受 Python 导数回调或积分器对象。
当前每次 Python 调用推进一个物理步，暂未加入 `nstep`、批量输出或 GIL 释放；同一份运行数据不能并发步进。
固定步长存在浮点时间累积误差，循环实验宜按整数步数推进，再读取实际时间。

## 单摆物理状态快照

| 字段 | 含义与单位 |
| --- | --- |
| `time` | 当前已接受状态时间，s |
| `angle` | 从竖直向下向世界 +X 的摆角，rad |
| `angular_velocity` | 摆角的时间导数，rad/s；对应空间角速度沿世界 -Y |
| `position_W` | 质点的世界坐标，三个数的列表，m |
| `velocity_W` | 质点的世界线速度，三个数的列表，m/s |
| `cable_direction_W` | 从悬点指向质点的单位方向列表，无量纲 |
| `tension` | 同一状态下的绳索张力，N |
| `energy` | 动能与相对最低点的势能之和，J |
| `mode` | 当前模型恒为 `"taut"`，不是松紧模式切换器 |

世界系 Z 向上，摆动限定在固定的 X–Z 平面；悬点可能非零，见 `model.pivot_W`。
位置、速度和派生量由同一已接受状态重算。快照在后续 step、reset 或对象删除后仍有效，修改字典或列表不会修改仿真数据。
读取状态不需要使用底层 `Vector3`、四元数、矩阵或求解器对象。

## 单摆范围及异常

单摆要求有限正的长度、质量、重力和步长，悬点有限且能分辨当前绳长。
首版限定 `abs(angle) < pi/2` 且 `E/(m*g*l) < 1`；精确无阻尼轨迹在此范围内始终正张力。
高能量摆动、完整旋转、球摆、零张力事件与松绳均不支持，不应将此模型视为完整吊载模型。
详细推导和数值范围见 [单摆文档](pendulum.md)。

| Python 异常 | 情况 |
| --- | --- |
| `TypeError` | 不匹配的参数类型或向量长度 |
| `ValueError` | 非有限/非正参数、非有限状态、空模型或模型/数据不匹配 |
| `csim.PendulumDomainError`（继承 `ValueError`） | 初值、reset、阶段或终点超出上述低能量范围 |
| `OverflowError` | 参数/观测量无法表示、世界坐标精度不足、时间无法推进或数值溢出 |

step 和 reset 失败时都保留之前的时间和状态，不返回半步结果。
阶段越界也可能由过大步长造成；缩小步长需构造新模型并使用有效初值，不修改只读模型参数。
当前不做事件根定位，不将负张力截为零后继续运算。

## 无吊载无人机接口

```python
model = csim.DroneModel(mass=1, timestep=0.005)
data = csim.make_data(model, position_W=[0,0,5], q_WB=[1,0,0,0])
csim.set_control(model, data, thrust=model.mass*model.gravity, torque_B=[0,0,0.02])
for _ in range(200):
    csim.step(model, data)
state = csim.get_state(model, data)
print(state['q_WB'], state['angular_velocity_B'])
csim.reset(model, data, position_W=[0,0,5])
```

| 接口 | 行为 |
| --- | --- |
| `DroneModel(mass=1, inertia_B=diag(0.02,0.02,0.04), gravity=9.80665, timestep=0.001)` | 不可变模型；惯性参数传入 3×3 嵌套序列，单位 kg·m² |
| `make_data(model, position_W=None, velocity_W=(0,0,0), q_WB=None, angular_velocity_B=(0,0,0))` | 创建 `DroneData`，时间及控制为零，归一化有限非零姿态 |
| `set_control(model, data, thrust, torque_B=(0,0,0))` | 替换总推力及机体系力矩，不推进时间；控制持续保持 |
| `step(model, data)` | 按当前实际推力/力矩推进一个物理时间步 |
| `get_state(model, data)` | 返回包含当前控制与同一时刻物理量的独立快照 |
| `reset(model, data, position_W=..., velocity_W=..., q_WB=..., angular_velocity_B=...)` | 初值默认值与 make_data 相同；归一化姿态，清零时间及控制 |

`diag(...)` 仅表示默认矩阵，不是 Python 导出的函数。
`mass / inertia_B / gravity / timestep` 是只读属性，`inertia_B` 返回嵌套列表副本。
`data.model`、模型身份匹配、独立数据、单步调用和所有权规则与单摆相同。
创建和 reset 后都默认零推力，必须显式设置悬停输入；省略 `torque_B` 会把力矩清零。

无人机快照字段：`time`、`position_W`、`velocity_W`、`q_WB`（wxyz）、`angular_velocity_B`、
`acceleration_W`、`angular_acceleration_B`、`thrust_W`、`angular_momentum_W`、`energy`、
`control={'thrust': ..., 'torque_B': [...]}`。角速度和角加速度在机体系表达，关于质心的角动量在世界系表达。
实际输入立即反映在加速度中。原生快照不包含执行器或控制器状态；Python `ControlLoop.get_state()` 可额外附加 `actuation` 诊断，见控制文档。
None 初始位置/姿态使用加载文件的整机质心位姿，直接构造的参数模型使用原点/单位姿态。
没有地面碰撞或电机转速模型；完整坐标、单位、方程、异常及误差记录见 [无人机文档](drone.md)。

`step / reset / set_control` 失败时状态、时间、控制均保持不变；成功返回 `None`。
不匹配的类型/形状报 `TypeError`，非法参数、姿态、惯性或模型身份报 `ValueError`，数值溢出报 `OverflowError`。
耦合吊载使用同样的仿真接口边界，增加两端及绳索物理状态；配置加载、批量运行和轨迹数组按需要扩展。

无人机快照中的 `acceleration_W`、`angular_acceleration_B` 已可读取，它们由同一状态和当前输入重算，不需要作为独立初值传入。详见 [独立状态与加速度](drone.md#独立状态导数与加速度)。
通过独立的 `csim_control.ControlLoop(model, data, ControlConfig(...))` 使用 [Python CTBR、单旋翼、执行器响应和延迟](control-and-response-models.md)。原生模型不接受 `control=` 参数，原生模块不导出 `ControlConfig`、`set_ctbr` 或 `set_rotor_thrusts`。
吊载的 [SPAD 阻尼选项](aerodynamics.md) 已实现并默认关闭，通过 DragConfig/WindField 配置启用，仍不开放底层积分器对象。

模型可配置 `integrator="euler"/"midpoint"/"heun"/"rk4"/"dopri5"`，单摆另支持 `symplectic_euler`、`velocity_verlet`。自适应参数为 `rtol`、`atol`、`max_substeps`；详见 [积分器使用](integrators.md)。每次 `step()` 仍推进一个 `timestep`，不导出积分器对象。

## 无人机吊载接口

```python
model = csim.SuspendedPayloadModel(drone_mass=2, payload_mass=0.5, length=1.2, timestep=0.002)
thrust = (model.drone_mass + model.payload_mass) * model.gravity
data = csim.make_data(model, thrust=thrust, position_W=[0,0,5])
csim.step(model, data)
state = csim.get_state(model, data)
print(state['payload_position_W'], state['payload_acceleration_W'], state['tension'])
csim.reset(model, data, thrust=thrust, position_W=[0,0,5])
```

`SuspendedPayloadModel` 具有只读的 `drone_mass / payload_mass / length / inertia_B / gravity / timestep`。
创建及重置必须显式给出 `thrust`；静止零推力会要求零张力，属于不支持的分支边界。
初值另外接受 `cable_direction_W=(0,0,-1)` 和 `cable_angular_velocity_W=(0,0,0)`，要求单位方向和切向角速度。
`torque_B` 默认零，其他无人机状态参数的默认值与 `DroneModel` 相同。
`set_control / step / get_state` 沿用原有调用方式；reset 清零时间，采用传入的初始控制。

快照中的 `position_W / velocity_W / acceleration_W` 指无人机，增加 `payload_position_W / payload_velocity_W / payload_acceleration_W`，以及绳方向、绳方向角速度/角加速度、两端绳力、`tension` 和 `mode`。混合模式另有 `cable_distance`、`cable_radial_velocity`、`cable_impulse_W`。
`energy` 为整个系统总机械能；`angular_momentum_W` 仍只表示无人机关于自身质心的角动量。
各字段完整列表、单位、参数签名和方程见 [耦合模型文档](suspended-payload.md)。

严格绷紧模式在初值、控制或 RK 阶段检测到非正张力时抛出 `csim.CableDomainError`（继承 `ValueError`）。使用 `cable_mode="hybrid"` 可启用松弛端点和步末收紧冲量；当前不做连续时间事件根定位。完整字段和初值规则见 [绷紧/松弛接口](slack-taut.md)。
失败时保留状态、时间及控制。模型所有权、身份检查、独立快照与底层接口不导出的约定同前。

## 可视化实验

```bash
xmake run csim_python examples/python/record_payload.py build/flight.csv
# viewer 需要另行开启 --viewer=y 并构建
xmake run viewer build/flight.csv
```

Python 脚本通过现有物理快照记录 CSV，同时输出参数和代码版本元数据 JSON；不依赖图形库，不新增数学或 OpenGL 绑定。
已有录制路径不会被覆盖，重跑时换一个文件名。实时观察可直接运行 `.venv/bin/python examples/python/tracking.py`，CSV 回放仍可独立使用。
操作、字段和步长/采样约定见 [可视化文档](visualization.md)；未来飞控闭环的独立时钟与接口见 [PX4/APM 设计](flight-controller-integration.md)。

## 构建与加载

推荐使用当前目录已经建立的 `.venv`：

```bash
uv pip install .
.venv/bin/python examples/python/drone.py
```

需要指定目标环境时使用 `uv pip install --python .venv/bin/python .`。
安装后直接 `import csim`，无需 PYTHONPATH；构建依赖由 uv 在隔离环境准备。
源码安装仍需 xmake、C++17 编译器和对应 Python 头文件。
发行包名 `csim-drone`，导入名 `csim`；本地安装、重装、editable 与 wheel 说明见 [uv 安装指南](python-installation.md)。
目前未发布 PyPI，不能用 `uv pip install csim` 代替本地路径安装。

以下为 C++ 开发者的直接 xmake 构建方式。Python 模块默认关闭；`--python=n` 时不查找 Python 或 pybind11。
当前构建支持本机 Linux/macOS，不支持交叉编译或 Windows；本地验证平台为 Linux。
本机使用 Python 3.10.12、pybind11 2.9.1，CI 使用 Ubuntu 24.04 系统包。

```bash
sudo apt install python3-dev python3-pybind11
xmake f -y -m debug --viewer=n --python=y --python_executable=/usr/bin/python3
xmake build
xmake test -v
xmake run csim_python examples/python/pendulum.py
xmake run csim_python examples/python/drone.py
xmake run csim_python examples/python/suspended_payload.py
```

`xmake test` 的 `csim_python/bindings` 运行 68 项 Python 测试（另含模型导入、控制响应与飞行示例），覆盖解析轨迹、控制、快照、重置、模型生命周期、能量和失败恢复。
构建脚本从同一解释器读取开发头文件、pybind11 和扩展 ABI 后缀。已有本地依赖时可离线构建，无需下载数值库。

如果需要直接用 xmake 编译到 build 目录，可选择已有虚拟环境：

```bash
uv pip install --python .venv/bin/python pybind11
xmake f -y --python=y --python_executable="$PWD/.venv/bin/python"
xmake build -r csim_python
xmake run csim_python -c 'import csim'
```

后续运行实验可用 `xmake run csim_python path/to/experiment.py`，它会自动添加模块搜索路径。
仅使用上述 xmake 构建时，直接启动 Python 或 Jupyter 需使用同一解释器，并将对应构建目录加入 `PYTHONPATH`。
使用 uv 安装时由 site-packages 加载，不需要这一步；两种开发流程不要混用模块路径。
更换解释器应重新配置构建；重新编译扩展后重启 Python 进程，避免继续使用已加载的旧模块。
当前已支持 uv 本地安装与构建 wheel；图形模块已随默认 wheel 打包，公开索引发布和多平台 wheel 仍待完成。

[开发总览](development-workflow.md) · [阶段 5](development/05-simulation-experiments.md)

## 阻力、风与可重复实验

`DragConfig(k1=..., k2=..., k0=..., sign_mode=..., epsilon_v=...)` 和 `WindField(...)` 配置模型空气力；`get_state` 的 aerodynamics 给出分项力和风作用诊断。`get_config(model)` 导出无人机/吊载的完整物理配置，Python ControlConfig 与原生 DragConfig/WindField 的 `to_dict()` 返回参数副本。

安装包同时提供 `csim_experiments`，用于完整记录、回放、批量扫描及阶跃辨识；参见 [空气力](aerodynamics.md)、[旋翼分配](control-and-response-models.md)、[实验指南](reproducible-experiments.md)。
