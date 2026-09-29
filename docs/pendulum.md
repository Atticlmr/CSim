# 固定悬点平面单摆

已实现第一个物理模型：无质量、不可伸长绳索悬挂质点，在世界系 X–Z 平面无阻尼摆动。
悬点固定，不包含无人机、驱动力、碰撞或松绳。数值推进使用 C++ RK4，Python 通过物理仿真接口操作。

## 状态、坐标与方程

采用 [建模约定](modeling-conventions.md) 的右手世界系，Z 向上。
长度 `l`、质量 `m`、重力大小 `g` 为有限正数，默认分别为 1 m、1 kg、9.80665 m/s²。
固定悬点 `pivot_W` 默认 `(0,0,0)`。摆角 `theta` 从竖直向下向 +X 测量，单位 rad；`omega=theta_dot` 为标量角速度，单位 rad/s。
相应空间角速度沿世界 -Y，为 `(0,-omega,0)`，不能把这个标量当作世界 Z 轴角速度。

```text
s_W = (sin(theta), 0, -cos(theta))
tangent_W = (cos(theta), 0, sin(theta))
position_W = pivot_W + l * s_W
velocity_W = l * omega * tangent_W

theta_dot = omega
omega_dot = -(g/l) * sin(theta)
T = m * (g*cos(theta) + l*omega^2)
E = 0.5*m*l^2*omega^2 + m*g*l*(1-cos(theta))
```

动力学使用完整 `sin(theta)`，没有小角度近似；质量不影响摆角轨迹，但影响张力与能量。
`E` 以最低点为势能零点。实现用 `2*sin(theta/2)^2` 计算势能项，减少小振幅下的相消。
运动方程及小振幅周期参考 [OpenStax 单摆章节](https://openstax.org/books/university-physics-volume-1/pages/15-4-pendulums)。
张力公式由径向加速度 `-l*omega²*s_W` 与径向力平衡得到。

位置和速度由角度状态重建，不单独积分；因此绳长与速度切向约束只受浮点几何运算误差影响，不累积约束漂移。
物理观测量全部由同一次已接受状态计算，不混用新位置和上一步张力。

## 首版支持范围

首版限定 `abs(theta) < pi/2` 且无量纲能量 `e = E/(m*g*l) < 1`。
这是一个明确的低能量验证模型，不覆盖所有物理上绷紧的轨迹。
对无阻尼精确解，能量不变且 `1-cos(theta) <= e < 1`，可知 `cos(theta)>0`，所以张力始终为正。
完整旋转、高能量摆动、零张力事件与松紧绳切换留待后续模型。

初值、reset、每个 RK4 阶段和最终候选都检查这个范围。
阶段越界或数值失败抛出异常，整步不提交时间或状态；应检查模型范围及步长，不会静默截断张力或投影回可用区域。
这不是零张力的事件根定位器；当前模型通过限制初值能量排除需要模式切换的精确轨迹。

参数须使 `g/l`、`sqrt(g/l)`、`m*g`、`m*g*l` 能表示为有限正数。
世界位置相对悬点的重建方向误差须不大于 `1e-10`，否则报告坐标尺度无法分辨绳长；应使用局部世界坐标。
合法参数和有限结果不代表步长足够精确：应检查轨迹及能量的步长敏感性。

## C++ 分层

- [`dynamics/pendulum.hpp`](../include/csim/dynamics/pendulum.hpp)：`PendulumState`、模型参数、导数、范围校验及物理量计算。
- [`simulation/pendulum.hpp`](../include/csim/simulation/pendulum.hpp)：不可变 `PendulumModel`、运行数据、步进、重置及状态快照。
- [`bindings/python/module.cpp`](../bindings/python/module.cpp)：只导出仿真对象和物理数据。

```cpp
#include <csim/simulation/pendulum.hpp>
#include <memory>

using namespace csim::simulation;
auto model = std::make_shared<PendulumModel>(1.0, 1.0, 9.80665, 0.01);
auto data = makeData(model, 0.7, 0.0);
step(*model, data);
const PendulumSnapshot snapshot = getState(*model, data);
reset(*model, data); // time=0, angle=0, angular_velocity=0
```

模型保存参数和固定时间步长。每个 `PendulumData` 持有模型的共享所有权，并保存独立状态与时间；同一个模型可创建多个独立实验。
所有操作检查模型对象身份，参数相同但对象不同也不能交叉使用数据。
`getState` 返回完整值副本，不推进仿真；修改副本不影响内部状态。
`step` 和 `reset` 均先计算及校验，再用不会抛出的赋值提交；失败时数据保持原样。
首版不支持同一份运行数据并发访问。

## Python 接口

```python
import csim

model = csim.PendulumModel(length=1, mass=1, timestep=0.01)
data = csim.make_data(model, angle=0.7, angular_velocity=0)
csim.step(model, data)
state = csim.get_state(model, data)
print(state['time'], state['position_W'], state['tension'])
csim.reset(model, data, angle=0.1)
```

`step` 一次推进一个模型时间步，返回 `None`。内部积分子阶段在 C++ 执行，无 Python 动力学回调；默认 RK4，可通过 `integrator=` 选择 [其他方法](integrators.md)。
`get_state` 返回普通字典，数值向量为列表，均为独立副本；字段、单位和异常见 [Python 接口](python.md)。
`data.model` 可读取其持有的模型；模型的长度、质量、重力、步长和悬点只读。
本模型是被动单摆，不提供控制输入；无人机与吊载耦合模型另提供总推力和力矩输入。

```bash
xmake run simulator
xmake run csim_python examples/python/pendulum.py
```

两者运行同一配置：1 m、1 kg、`g=9.80665`、初始角 0.7 rad、初速度 0、步长 0.01 s、2000 步（20 s）。
命令行程序默认运行这个单摆示例；`xmake run simulator drone` 可运行独立无人机示例，目前不读取场景配置。

## 验收记录

动力学五组检查：几何与牛顿力平衡、镜像对称及质量缩放、小振幅周期、能量步长收敛、参数与模型范围。
仿真层四组检查：真实步进和时间、快照/reset/所有权、模型匹配、失败原子性。
Python 五项检查覆盖同一物理基准、独立数据、只读参数、快照及模型生命周期、能量与异常。
所有 C++ 检查不依赖 `assert`，Debug/Release 都执行。

小振幅 `theta0=0.01`、`l=1` 时，通过连续向下过零的时间差测得周期约 2.00642 s；
小角度公式 `2*pi*sqrt(l/g)` 给出约 2.00641 s，相对差小于 `1e-5`。非线性摆的周期略大，符合预期。

20 s、初始角 0.7 rad 的最大相对能量误差：

| 步长（s） | 最大相对能量误差 | 最大绳长残差（m） |
| --- | --- | --- |
| 0.04 | 2.24394e-5 | 2.22e-16 |
| 0.02 | 7.09503e-7 | 2.22e-16 |
| 0.01 | 2.26534e-8 | 2.22e-16 |

数据为本机双精度结果，末位可能随平台变化。这里只证明该算例下误差随步长下降；RK4 不具有严格能量守恒或辛性质。

[无吊载无人机](drone.md) 已完成独立验证，[吊载耦合](suspended-payload.md) 也已完成无阻尼基准，下一步接入可选阻力。

[阶段 4](development/04-dynamics.md) · [Python 接口](python.md)
