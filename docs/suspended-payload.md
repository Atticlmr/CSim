# 无人机与绷紧绳索质点吊载

当前已实现三维无阻尼耦合动力学、C++ 仿真层和 Python 物理接口，并提供可选的绷紧/松弛混合模式。
动力学位于 `include/csim/dynamics/suspended_payload.hpp`，仿真管理位于 `include/csim/simulation/suspended_payload.hpp`。
这是 [阶段 4](development/04-dynamics.md) 的耦合基准；默认关闭的 SPAD、风场和控制器已按独立扩展验收，配置见 [空气力](aerodynamics.md)。

## 物理范围与状态

遵循 [建模约定](modeling-conventions.md)：世界系 W 为右手系、Z 向上，机体系 B 为 FLU，Hamilton 四元数 `q_WB` 按 wxyz 存储并把 B 向量转到 W。
无人机是具有完整对称正定惯性矩阵的刚体；吊载为质点。无质量、不可伸长的绳索连接在无人机质心，当前仅支持严格正张力的绷紧分支。
无人机可自由运动；下面方程描述默认无阻尼、无风基准。可选空气力及旋翼控制/响应已接入，尚无弹性或碰撞。绳索在三维空间运动，不限于一个摆动平面。混合模式的独立端点状态和收紧冲量见 [绷紧/松弛接口](slack-taut.md)。

`SuspendedPayloadState` 保存：

| 成员 | 含义 |
| --- | --- |
| `drone.position_W / velocity_W` | 无人机质心的位置和线速度，m、m/s |
| `drone.q_WB / angular_velocity_B` | 姿态与机体系角速度，rad/s |
| `cable_direction_W` | 从无人机指向吊载的单位向量 `s` |
| `cable_angular_velocity_W` | 绳方向的切向角速度 `omega`，rad/s，满足 `omega·s=0` |

不独立积分吊载的位置和速度，也不独立积分张力和加速度。这些量由同一状态及当前输入计算：

```text
s_dot = omega × s
p_L = p_Q + l*s
v_L = v_Q + l*s_dot
```

`omega` 是描述绳方向变化的最小角速度，不表示绳索扭转或吊载自转。
`cable.hpp`、`payload.hpp` 继续为更复杂的独立部件保留位置，当前绳约束与质点受力直接在耦合模型中闭合。

## 受力与方程

输入复用 `DroneControl`：总推力 `f >= 0` 沿机体正 Z，控制力矩 `tau_B` 关于无人机质心。
令 `G=(0,0,-g)`、`F=R_WB*(0,0,f)`，则：

```text
m_Q*a_Q = m_Q*G + F + T*s
m_L*a_L = m_L*G     - T*s
```

绳长固定意味着 `s·s=1`、`s·s_dot=0`、`s·s_ddot=-|s_dot|²`。
将两端加速度相减，并投影到 `s`，直接解得张力和绳方向角加速度：

```text
mu = m_Q*m_L/(m_Q+m_L)
T = mu * (l*|s_dot|² - s·F/m_Q)
omega_dot = -(s × F)/(m_Q*l)

a_Q = G + (F + T*s)/m_Q
a_L = G - T*s/m_L
```

实现用 `m_Q*(m_L/(m_Q+m_L))` 计算约化质量，避免直接相乘引起不必要的溢出。
重力从相对运动方程中抵消，但仍影响系统的绝对运动。
无人机悬点是运动的，不能套用固定悬点摆的张力式。
更一般的外力和阻力推导见 [Zhu 等模型解析第 5.2 节](zhu-2025-payload-model.md#52-单位绳方向形式csim-推导)；本实现对应吊载外力为零、无人机外力仅为推力的情况。

悬点在质心，因此绳力不产生机体力矩，姿态方程复用已有 [无人机模型](drone.md)：

```text
q_dot = 0.5*q_WB ⊗ (0, Omega_B)
J_B*Omega_dot = tau_B - Omega_B × (J_B*Omega_B)
```

静止悬停时 `s=(0,0,-1)`、姿态为单位旋转、`f=(m_Q+m_L)*g`，结果为 `T=m_L*g`、`a_Q=a_L=0`。
竖直绳索且只有竖直推力时，两端都以 `f/(m_Q+m_L)-g` 加速。

总机械能以世界 `z=0` 为势能零点：

```text
E = 0.5*m_Q*|v_Q|² + 0.5*Omega_B·J_B*Omega_B + m_Q*g*z_Q
  + 0.5*m_L*|v_L|² + m_L*g*z_L
dE/dt = F·v_Q + tau_B·Omega_B
```

理想绳索两端的功率相互抵消。存在推力时，总能量未必守恒；恒定世界系推力、零机体力矩的算例可检查 `E-F·p_Q`。
推力为零时，质心自由落体，绳方向可保持匀速转动，向心张力仍可为正。零推力本身不是判错条件。

## 积分与有效范围

仿真层默认使用固定步长 RK4，每个对外时间步内保持实际总推力和机体系力矩；可选择 [其他积分器](integrators.md)，自适应子步不改变外部时钟。
四元数与绳方向的原始状态加法、标量乘法不做归一化；导数的零四元数也不能误写成单位姿态。
RK 阶段可能暂时偏离约束曲面，动力学通过归一化绳方向和去除角速度径向分量的光滑延拓计算受力；姿态旋转归一化，`q_dot` 使用原始阶段四元数。
候选终点再次投影，重算张力和物理量，全部成功后才提交状态和时间。
这不是 RATTLE 或严格保能算法；投影和积分仍产生截断误差，已用解析解验证四阶收敛。

外部初值和 reset 必须满足以下规则：

- 质量、绳长、重力和步长有限且为正；惯性矩阵要求与无吊载模型相同。
- 所有状态有限，姿态四元数非零；允许缩放和相反符号的四元数，内部归一化。
- `abs(|s|-1) <= 1e-9`，`abs(s·omega) <= 1e-9*max(1,|omega|)`，只修正该容差内的初值误差。不能传入任意长度方向或带明显径向分量的角速度。
- 初值、当前控制、所有 RK 阶段及候选终点的张力都必须严格为正。
- 输出世界坐标必须能分辨绳长；重建的 `(p_L-p_Q)/l` 与 `s` 的误差超过 `1e-9` 时报告数值范围错误。

严格 `taut` 模式检测到 `T<=0` 抛出 `CableDomainError`。`hybrid` 模式在步末切换到松弛，并在距离达到绳长时施加径向完全非弹性收紧冲量；事件尚未进行连续时间根定位，不把负张力截成零。详细接口和误差限制见 [绷紧/松弛接口](slack-taut.md)。
阶段检查不能保证找出采样点之间的所有边界穿越，过大步长也可能导致阶段越界；应缩小步长并检查结果收敛。
`step / setControl / reset` 的任何失败都保留此前的状态、时间和输入。

## C++ 与 Python 接口

`SuspendedPayloadModel` 不可变；`SuspendedPayloadData` 持有模型共享所有权，只能通过仿真接口修改。
模型身份匹配、快照独立性和生命周期规则与 [现有 Python 接口](python.md) 相同。

```cpp
#include <csim/simulation/suspended_payload.hpp>

using namespace csim::simulation;
auto model = std::make_shared<SuspendedPayloadModel>();
const auto& physics = model->physics();
const double thrust = (physics.drone().mass()+physics.payloadMass())*physics.drone().gravity();
csim::dynamics::SuspendedPayloadState initial;
initial.drone.position_W = {0,0,5};
auto data = makeData(model, {thrust,{}}, initial);
step(*model, data);
auto snapshot = getState(*model, data);
reset(*model, data, {thrust,{}}, initial);
```

创建和重置时必须显式给出初始控制，因为静止、零推力初值会要求 `T=0`，已经位于本模型的分支边界。
不自动补偿重力；当前 API 也不允许先创建非法绷紧状态，再靠 `set_control` 修复。
无吊载 `DroneModel` 原有的零控制初始化规则不受此约束。

```python
import csim

model = csim.SuspendedPayloadModel(drone_mass=2, payload_mass=0.5, length=1.2,
                                  gravity=9.81, timestep=0.002)
hover_thrust = (model.drone_mass + model.payload_mass) * model.gravity
data = csim.make_data(model, thrust=hover_thrust, position_W=[0,0,5])
csim.set_control(model, data, thrust=hover_thrust, torque_B=[0,0,0])
for _ in range(500):
    csim.step(model, data)
state = csim.get_state(model, data)
print(state['payload_position_W'], state['payload_acceleration_W'], state['tension'])
```

Python 模型参数为 `drone_mass=1, payload_mass=0.2, length=1, inertia_B=diag(0.02,0.02,0.04), gravity=9.80665, timestep=0.001`，均有同名只读属性。
`make_data(model, thrust, ...)` 和 `reset(model, data, thrust, ...)` 的其余参数相同：
`torque_B=(0,0,0), position_W=(0,0,0), velocity_W=(0,0,0), q_WB=(1,0,0,0), angular_velocity_B=(0,0,0), cable_direction_W=(0,0,-1), cable_angular_velocity_W=(0,0,0)`。
reset 将时间设为零，同时替换初值和控制；省略力矩表示零力矩。

快照中的无前缀 `position_W / velocity_W / acceleration_W / q_WB / angular_velocity_B / angular_acceleration_B / thrust_W` 指无人机。
`angular_momentum_W` 是无人机关于自身质心的角动量，不是整个系统的总角动量。
`energy` 则是整个耦合系统的总机械能。
另有 `time`、`control={'thrust':..., 'torque_B':[...]}` 及以下吊载字段：

| 字段 | 含义及单位 |
| --- | --- |
| `payload_position_W / payload_velocity_W / payload_acceleration_W` | 吊载世界系位置、速度和加速度，m、m/s、m/s² |
| `cable_direction_W` | 从无人机到吊载的单位方向 |
| `cable_angular_velocity_W / cable_angular_acceleration_W` | 绳方向角速度与其导数，rad/s、rad/s² |
| `cable_force_on_drone_W / cable_force_on_payload_W` | 两端轴向绳力，N |
| `tension` | 正张力标量，N |
| `mode` | 当前分支，`"taut"` 或混合模式下的 `"slack"` |

混合模式还返回 `cable_distance`、`cable_radial_velocity` 和 `cable_impulse_W`。

设置控制后立即读取即可得到该控制下的加速度和张力，不需要先推进一步。
Python 只收到普通字典和列表副本；不开放底层积分、向量或约束求解器。
参数形状错误为 `TypeError`，非法参数、约束初值和模型身份为 `ValueError`，绳索边界为继承 `ValueError` 的 `csim.CableDomainError`，数值范围错误为 `OverflowError`。

## 验证与运行

新增五组 C++ 动力学测试、五组仿真测试和五项 Python 接口测试。
验证悬停、共同竖直加速度、三维 Newton 受力与二阶绳约束、绳力零合功率、外部输入功率、完整惯性矩阵、质心连接无附加力矩、初值范围和非正张力。
仿真层还验证快照/所有权、重置、控制立即生效、阶段边界与数值溢出时的失败原子性。

自由落体中的匀速绳方向转动具有解析解：质心作抛体运动，`s` 绕固定切向轴按 Rodrigues 公式旋转，`omega` 恒定，`T=mu*l*|omega|²`。
测试取 `m_Q=2 kg, m_L=0.5 kg, l=1.2 m, |omega|=2 rad/s`，绕 `(1,2,3)/sqrt(14)` 旋转 2 s。
误差指标为绳方向误差，以及分别按 1 m、1 m/s 缩放的无人机位置/速度与吊载位置误差之最大值：

| 步长 / s | 最大缩放状态误差 | 最大总能量误差 / J |
| --- | --- | --- |
| 0.04 | 5.10929e-7 | 8.40239e-7 |
| 0.02 | 3.19832e-8 | 4.68578e-8 |
| 0.01 | 1.99974e-9 | 2.93039e-9 |

误差比约 16，符合四阶收敛。另一个 10 s 三维恒推力摆动算例，步长 0.002 s，`E-f*z_Q` 的最大误差约 `2.54e-12 J`；测试允许 `2e-8 J`，并独立检查动量、绳长和切向速度约束。这些是数值基准，未声称复现论文实物实验。

```bash
xmake build
xmake test -v suspended_payload_test/default suspended_payload_simulation_test/default
xmake run simulator payload
# 开启 Python 构建后：
xmake test -v csim_python/bindings
xmake run csim_python examples/python/suspended_payload.py
```

Python 算例先悬停 1 s，再重置为具有三维初始摆动的状态，恒定推力运行 10 s。
无人机随吊载反作用力移动；这不是位置保持或消摆控制演示。CI 在原有矩阵中加入 C++/Python 耦合算例。
已接入默认关闭的 SPAD/相对空气速度阻力和风场；Python 控制支线已实现 CTBR 内环、受约束旋翼分配、执行器响应及延迟，见 [控制配置](control-and-response-models.md)。
