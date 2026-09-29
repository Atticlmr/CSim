# 无吊载无人机刚体动力学

当前实现单个无人机的六自由度运动，以总推力和质心处力矩为输入。
实现位于 [动力学头文件](../include/csim/dynamics/drone.hpp) 与 [仿真头文件](../include/csim/simulation/drone.hpp)。
坐标定义遵循 [建模约定](modeling-conventions.md)；Python 只操作物理模型、运行数据和状态快照。

## 物理范围与坐标

- 质量 `mass` 与关于质心的机体系惯性矩阵 `inertia_B` 恒定。
- W 为右手世界系，Z 向上；B 为质心处的 FLU 系，X 向前、Y 向左、Z 向上。
- `q_WB` 为 Hamilton 四元数，顺序 `(w,x,y,z)`，把机体系分量转换到世界系。
- `angular_velocity_B` 为机体系角速度，控制 `torque_B` 也在机体系表达。
- 总推力 `thrust >= 0` 沿机体正 Z；世界系重力为 `(0,0,-g)`。
- 本节推导默认关闭空气力且不含吊载、绳索和接触；核心可配置相对空气速度阻力与风场，Python 层可配置 CTBR、受约束旋翼分配、执行器一阶响应及限幅，C++ 只接收实际力。`z=0` 仅为势能参考面，自由落体可以穿过该平面。

单位采用 SI：m、s、kg、N、N·m、kg·m²，角速度为 rad/s。
默认惯性 `diag(0.02,0.02,0.04)` 仅为示例参数，不对应具体实机。

## 独立状态、导数与加速度

“先计算加速度”描述数值推进的计算顺序，不表示加速度必须是第一个独立状态。
这里的状态是：给定它和后续输入，就足以确定系统未来运动的一组变量。
当前刚体采用 `x=(position_W, velocity_W, q_WB, angular_velocity_B)`，共 13 个存储分量，其中四元数有单位长度约束。

| 类别 | 当前模型中的量 | 如何得到 |
| --- | --- | --- |
| 独立积分状态 | 位置、线速度、姿态、角速度 | 保存初值，随步进更新 |
| 状态导数 | 线速度、线加速度、四元数导数、角加速度 | 每个积分阶段由状态和输入计算 |
| 派生物理量 | 线/角加速度、推力、角动量、能量 | 从当前状态、模型参数和当前实际输入重算 |

```text
a_W = G_W + R_WB*(0,0,f/m)
alpha_B = solve(J, tau_B-Omega_B × (J*Omega_B))
x_dot = (v_W, a_W, q_dot_WB, alpha_B)
```

每步确实先求力和加速度，然后积分得到速度与位置；RK4 在不同中间状态多次进行这项计算。
加速度作为 `v_dot` 参与推进，位置则以 `v` 为导数，两者由同一个积分步骤一致更新。
改变实际推力或姿态会改变加速度，因此不应再要求用户独立输入一个与受力无关的“初始加速度”。
当前 `get_state` 已包含 `acceleration_W` 与 `angular_acceleration_B`，它们是物理状态快照的一部分；`DroneState` 只存储积分所需的独立变量。
在现有 ideal 模式下，`set_control` 后无需 step 就能读取新输入对应的加速度。

在 Python 层加入执行器动态时，实际推力或电机转速会成为该层的独立状态，加速度仍由这些状态和刚体受力计算。
若另建以 jerk 为输入的运动学模型，才可取 `(p,v,a)` 为状态，并令 `a_dot=jerk`；那是另一套动力学假设。
传感器测得的加速度也是观测量，不能直接当作可自由指定的刚体真实状态。

## 方程

记位置为 `p`、世界线速度为 `v`、机体角速度为 `Omega`，`R=R_WB(q)`：

```text
p_dot = v
v_dot = (0,0,-g) + R * (0,0,f/m)
q_dot = 0.5 * q ⊗ (0,Omega)
J * Omega_dot = tau_B - Omega × (J * Omega)
```

转动方程由世界系角动量定理转换到随体坐标得到。`Omega × (J*Omega)` 是坐标系旋转引入的项，不能在一般运动中省略。
当 J 非球对称且角速度不沿主惯性轴时，零力矩不意味着机体系角速度恒定。
转动方程背景见 [MIT 三维刚体动力学讲义](https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/resources/mit16_07f09_lec28/)。

实现接受完整的对称正定 3×3 惯性矩阵，不要求机体系轴与主惯性轴对齐。
模型构造时调用已有 `PartialPivLU<3>` 分解一次；求角加速度时解线性方程，不显式构造逆矩阵。
其他分解方法的 TODO 保持预留状态。

从同一状态和当前控制重算以下物理量：

```text
F_thrust_W = R * (0,0,f)
H_W = R * (J * Omega)                         # 关于无人机质心
E = 0.5*m*(v·v) + 0.5*Omega·(J*Omega) + m*g*p.z
E_dot = F_thrust_W·v + tau_B·Omega             # 连续方程的功率关系
```

无推力、无力矩时，连续系统的总机械能与关于质心的世界系角动量守恒。
施加控制后能量通常不守恒；悬停测试是理想初值下的力平衡检查，不代表存在稳定控制器。

## C++ 与步进语义

```cpp
#include <csim/simulation/drone.hpp>

using namespace csim::simulation;
auto model = std::make_shared<DroneModel>();
auto data = makeData(model, {{0,0,5}, {}, {}, {}});
setControl(*model, data, {model->physics().mass()*model->physics().gravity(), {}});
step(*model, data);
auto snapshot = getState(*model, data);
```

`DroneState` 成员顺序为 `position_W, velocity_W, q_WB, angular_velocity_B`。
默认状态的位置和速度为零、姿态为单位四元数；导数通过显式赋值构造，静止时四元数导数为零。
`DroneControl` 成员为 `thrust, torque_B`。

`DroneModel(mass=1, inertia_B=defaultInertia(), gravity=9.80665, timestep=0.001)` 不可变。
`DroneData` 持有模型共享所有权、状态、控制与时间；不同 data 相互独立，模型必须与创建该 data 时的对象相同。

每次 `step` 在 C++ 中完成一个对外时间步，实际推力/力矩在全部阶段保持不变。默认固定步长 RK4，可配置 [其他积分器](integrators.md)，包括内部自适应的 Dormand–Prince。
阶段旋转矩阵使用归一化的四元数副本，四元数导数使用原始阶段值。
候选终点的四元数在仿真层归一化，再验证观测量，最后一次性提交状态和时间。
不修改通用积分器，不归一化导数，也不在中间阶段修改存储状态。

`makeData` 和 `reset` 接受有限非零四元数并归一化，包括不同长度或相反符号的等价表示；不强制 `w>=0`。
`setControl` 替换整个控制输入且不推进时间，持续生效直到下一次设置或 reset。
`reset` 将时间设为零、恢复指定状态，并将推力和力矩都清零；创建数据时同样默认零控制，必须显式设置悬停推力。

`step`、`reset`、`setControl` 任一失败时保留此前的全部状态、时间和控制。
快照包含当前控制下重算的加速度，因此设置控制后立即 `getState` 可观察新加速度，时间不变。
返回值是副本；同一份 data 不支持并发读写。

## Python 接口与示例

```python
import csim

model = csim.DroneModel(mass=1, timestep=0.005,
                       inertia_B=[[0.02,0,0], [0,0.02,0], [0,0,0.04]])
data = csim.make_data(model, position_W=[0,0,5], velocity_W=[0,0,0],
                      q_WB=[1,0,0,0], angular_velocity_B=[0,0,0])
csim.set_control(model, data, thrust=model.mass*model.gravity, torque_B=[0,0,0.02])
for _ in range(200):
    csim.step(model, data)
state = csim.get_state(model, data)
print(state['position_W'], state['q_WB'], state['angular_velocity_B'])
csim.reset(model, data, position_W=[0,0,5])  # 时间和控制清零
```

`DroneModel` 的 `mass, inertia_B, gravity, timestep` 均只读；惯性矩阵读出为嵌套列表副本。
`make_data` 和 `reset` 的四个状态参数均可省略，默认零位置/速度、单位姿态、零角速度。
`set_control(model, data, thrust, torque_B=(0,0,0))` 要求显式给出总推力，省略力矩表示清零力矩。
`step / reset / set_control` 成功返回 `None`。

`get_state` 返回独立字典，列表及嵌套控制字典都不共享底层存储：

| 字段 | 意义 |
| --- | --- |
| `time` | 时间，s |
| `position_W`、`velocity_W` | 质心世界位置、线速度，长度 3 的列表 |
| `q_WB` | B→W 单位四元数，长度 4，wxyz |
| `angular_velocity_B` | 机体角速度，rad/s |
| `acceleration_W`、`angular_acceleration_B` | 当前控制下的线加速度与角速度分量导数，m/s²、rad/s² |
| `thrust_W` | 世界系推力，N |
| `angular_momentum_W` | 关于质心的世界系角动量，kg·m²/s |
| `energy` | 总机械能，J，势能零点为世界 z=0 |
| `control` | `{'thrust': 数值, 'torque_B': [x,y,z]}` |

完整算例先悬停 1 秒，再施加 1 秒 `tau_z=0.02 N·m`：

```bash
xmake f -y -m debug --viewer=n --python=y --python_executable=/usr/bin/python3
xmake build
xmake run simulator drone
xmake run csim_python examples/python/drone.py
xmake test -v
```

同配置最终高度 5 m，偏航角速度 0.5 rad/s，偏航角 0.25 rad；Python 示例检查解析姿态误差。
绑定仅导出物理 API，不导出 `DroneState` 数学运算、动力学导数、LU 或积分器。

## 参数范围与异常

质量、重力和步长必须有限且严格为正；状态和控制必须有限，总推力不能为负。
惯性矩阵检查有限性、相对尺度 `1e-12` 内的对称性及正定性；仅对容差内的不对称项取平均，模型属性返回实际使用的矩阵。
正定性用缩放后的三阶顺序主子式检查，数值求解还受 LU 默认主元阈值限制，病态矩阵可能被拒绝。
此处只验证矩阵对称正定及可求解性，尚未依据质量分布验证主惯性矩的三角不等式；实机参数应来自有效惯性辨识或几何模型。`load_model` 的独立文件适配层另外检查主惯量三角不等式。

- `ValueError`：非法参数、零/非有限姿态、非正定或数值奇异惯性、非有限状态、模型与数据不匹配。
- `TypeError`：Python 参数类型、向量/矩阵形状或模型/数据类型不匹配。
- `OverflowError`：中间计算或观测量不可表示、时间无法前进。

模型按 double 精度工作，不承诺在任意极端量级下计算成功；所有阶段及终点必须可表示。
步进失败不会自动减小步长；过大的有限步长也可能产生有限但不准确的结果，四元数归一化不能替代收敛检查。
当前没有自适应步进、事件定位和接触处理。

## 验证记录

新增 `drone_test/default` 六组动力学检查、`drone_simulation_test/default` 五组仿真检查，以及六项 Python 接口测试。

- 自由落体与倾斜固定推力对照解析抛物线；10 秒理想悬停保持初始状态。
- 分别沿三个主惯性轴施加恒定力矩，对照 `Omega=Omega0+alpha*t` 和 `theta=Omega0*t+0.5*alpha*t²`；初始姿态非单位旋转，验证随体乘法顺序。
- 独立检查非对角惯性下的角加速度、陀螺项符号、旋转导数、机械能及输入功率。
- 无外力矩转动验证机体转动能与世界系角动量，以及步长减半后的误差下降。
- 验证状态快照、输入保持、重置、模型生命周期、身份匹配、非法输入和失败原子性；Python 直接对照物理解析结果。

无外力矩算例 `J=diag(2,3,4)`、`Omega0=(1,2,3)`，运行 10 秒，整个轨迹的最大相对误差：

| 步长 / s | 转动动能 | 关于质心的世界系角动量 |
| --- | --- | --- |
| 0.04 | 1.25402e-7 | 1.15431e-6 |
| 0.02 | 4.43983e-9 | 7.15343e-8 |
| 0.01 | 2.51231e-10 | 4.45197e-9 |

每次步长减半，两项误差均要求下降超过 10 倍；最细步长要求均小于 `1e-7`，已接受四元数的长度误差验收容差为 `1e-14`。
这些记录适用于上述算例，不是任意飞行条件下的全局误差保证。

[无人机、不可伸长绳索与质点吊载耦合](suspended-payload.md) 已实现无阻尼基准，保留本独立刚体作为对照。
耦合模型的可选 SPAD 阻尼和论文推导见 [Zhu 等模型解析](zhu-2025-payload-model.md)；CTBR、单旋翼推力和响应延迟见 [控制接口与响应模型](control-and-response-models.md)。SPAD 与两端空气力已实现，见 [阻力/风场](aerodynamics.md)；控制/分配/响应已迁移至独立 Python 层。
