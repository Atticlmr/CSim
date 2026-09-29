# 数值积分算法调研与预留接口

调研日期：2026-09-10。本文讨论初值问题的时间积分。
当前 Euler、中点 RK2、Heun RK2、辛 Euler、Velocity Verlet、[RK4](rk4.md) 与 [Dormand–Prince 5(4)](dormand-prince.md) 已实现；模型选择、容差和边界说明见 [积分器使用](integrators.md)。复杂隐式、约束及李群方法仍保留 TODO。

固定步长 RK4 作为验证基线，已实现的 **Dormand–Prince 5(4)** 用于非刚性模型的自适应精度控制。
长期保守摆动、显式绳长约束、高刚度弹性绳和流形姿态分别需要评估不同方法，不能仅凭阶数选择积分器。
以下开发优先级是结合本项目[建模约定](modeling-conventions.md)作出的工程判断，数值算例的误差与计算次数见已实现算法文档，尚无完整吊载模型的性能实测。

## 候选算法

| 方法 | 特点 | 本项目用途与限制 | 预留位置 |
| --- | --- | --- | --- |
| 显式 Euler / 显式中点 RK2 | 一阶 / 二阶，简单、单步成本低 | 调试与收敛对照；Euler 不作为长期摆动的默认方法 | `explicit_runge_kutta.hpp` |
| 经典 RK4 | 四阶、固定步长，无内置误差估计 | 首个实现，便于控制周期对齐和复现实验 | `rk4.hpp` |
| Dormand–Prince 5(4) | 五阶推进，嵌入四阶误差估计 | 非刚性离线仿真的优先扩展；需要拒步和步长控制 | `dormand_prince.hpp` |
| DOP853 | 八阶显式 RK | 非刚性高精度参考轨迹，低容差时评估成本优势 | 同上 |
| 辛 Euler | 一阶、保持辛结构的机械积分 | 可分离保守系统的低成本对照；有稳定步长限制 | `symplectic.hpp` |
| Velocity Verlet | 二阶、对称、适当条件下为辛方法 | 简谐振子及无约束保守机械系统；当前接口要求加速度仅依赖位置 | 同上 |
| Radau IIA（3 阶段、5 阶） | 隐式 RK，适合刚性 ODE | 高刚度弹性绳等扩展；需要非线性迭代、Jacobian 和线性求解 | `implicit.hpp` |
| BDF | 隐式多步法 | 刚性问题的另一选择；需要历史、启动和重启逻辑 | 同上 |
| RATTLE | 二阶约束机械积分 | 显式保留绳长等完整约束时考虑，同时约束位置与速度 | `rattle.hpp` |
| 李群积分（RKMK / 无交换子方法等） | 在群或群作用下的流形上推进 | 姿态 `SO(3)`、绳索方向 `S²` 的后续方向；具体阶数和几何接口待选 | `lie_group.hpp` |

SciPy 官方文档区分了显式非刚性算法与 Radau/BDF 等刚性算法，并推荐 DOP853 用于较高精度需求。
其中 RK45 使用 Dormand–Prince 5(4)，不能把它与 Fehlberg RKF45 的系数表混用。
LSODA 会在 Adams/BDF 间自动切换，也可作为以后独立对照工具；本项目暂不预留自研自动刚性检测器。
这里引用 SciPy 作为算法资料，不引入 SciPy 运行依赖。[SciPy solve_ivp](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html)

辛方法的长期优势依赖哈密顿结构、步长和正则性等条件，不代表每一步都精确守恒能量，也不代表任意步长都稳定。
加入推力、阻尼或任意变步长后，不能直接沿用保守系统的结论。
显式中点 RK2 与隐式中点法不同；“二阶”本身并不意味着“辛”。
参考 [Hairer 的辛积分讲义](https://www.unige.ch/~hairer/poly_geoint/week2.pdf)
及 [Hairer、Lubich、Wanner 的 Verlet 综述](https://www.unige.ch/~hairer/preprints/gniverlet.html)。

RATTLE 针对完整约束机械系统，具有二阶、对称、辛等性质；它通常通过乘子求解同时满足位置和速度约束，不能用一次方向归一化替代。
位置与速度约束的区别见 [LAMMPS SHAKE/RATTLE 文档](https://docs.lammps.org/fix_shake.html)
和 [Console、Hairer、Lubich 的约束积分研究](https://www.unige.ch/~hairer/preprints/sym-constr.html)。

李群积分与本项目直接相关：Celledoni 等的 *Lie Group integrators for mechanical systems* 同时讨论球摆及双旋翼飞行器运输吊载，涉及 `SO(3)` 与 `TS²`。
这支持将其列为后续候选，但不证明它一定比 RK4 更快；保持流形也不自动等于保持能量或辛结构。
[论文原文与摘要](https://arxiv.org/abs/2102.12778)

## 根据吊载模型选择

第一版是无质量、不可伸长、始终绷紧的绳索，不应仅凭“有绳索”判定数值刚性。
若推导为状态导数显式可求的约化 ODE，可先使用 RK4，再以 Dormand–Prince 检查误差与计算成本。
单位四元数、绳索方向长度及切向速度仍需分别检查；局部截断误差控制不会自动保证这些约束。

若改用笛卡尔位置、动量和约束乘子，显式保留
`g(q) = |p_L - p_Q|² - l² = 0`，会形成约束机械系统。
常见位置层约束写法是高指数 DAE，不能把代数约束直接当作 `dx/dt` 交给一般 ODE 接口。
可考虑 RATTLE，或先处理指数、变量和一致初值，再使用合适的 DAE 求解器。
SUNDIALS IDA 采用残差 `F(t, y, ydot) = 0` 和 BDF，并要求一致初值；其自动初值求解针对特定问题类别，并不承诺直接处理任意高指数系统。
[SUNDIALS IDA 数学说明](https://sundials.readthedocs.io/en/latest/ida/Mathematics_link.html)

加入大刚度弹性绳、快速执行器或显著分离的时间尺度后，可能出现显式法受稳定性而非精度限制的情况。
这时比较 Radau/BDF 与显式方法的误差、拒步次数及总耗时；隐式法允许更稳定的推进不等于允许忽略快速运动的精度需求。

张力降到零是模型边界。第一版应定位边界并停止；以后才加入松绳和再次绷紧的切换。
不能靠增大阶数或截断负张力处理切换。事件定位、必要的冲量模型和求解器重启属于仿真层工作。
一般依赖步内符号变化的事件检测也可能漏过同一步中的多次过零，需限制步长并验证边界算例。
[SciPy 事件检测说明](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html)

## 预留接口的含义

所有头文件位于 [`include/csim/numerics/`](../include/csim/numerics/)。
以下占位约定仅适用于尚未实现的算法。Euler、中点/Heun RK2、辛 Euler/Verlet、RK4 与 Dormand–Prince 5(4) 已可调用；DOP853、Radau/BDF、RATTLE 和李群接口仍是设计草案。

- **单步函数使用 `= delete` 占位**：可正常包含头文件，但尝试调用会产生编译错误；实现时替换为真正函数体。
- **复杂求解器只前置声明**：可以声明指针或引用，不能构造实例。成员函数、配置及结果布局待对应算法开发时确定。
- 每个头文件都写明模型要求、必要的求解能力和验收 TODO，不提供返回原状态之类的伪实现。

已实现 RK4 的单步签名如下；Euler/中点/Heun RK2 已实现并沿用此参数顺序：

```cpp
template <typename State, typename Control, typename Dynamics>
State rk4Step(double t, const State& state, const Control& control,
              double dt, Dynamics&& dynamics);
```

`eulerStep`、`midpointStep`、`heunStep` 使用同样的参数顺序。
约定 `dynamics(t, state, control)` 返回状态导数；一个步内保持 `control` 不变，时间和子阶段状态照常变化。
状态需支持加法、标量乘法，输入状态不被修改。通用算法不能假设 `State{}` 为零，尤其 `Quaternion{}` 是单位四元数。
已实现的显式方法都要求自定义状态提供 `isFinite()`，检查时间、状态和导数并传播异常，详见 [RK4 数值约定](rk4.md) 与 [积分器使用](integrators.md)。物理有效性由模型验证，未实现的占位接口不提供运行保证。

`dormandPrince54Trial` 已返回 `EmbeddedStep<State>{state, error}`，包含五阶候选值与差值 `x5 - x4`。
一次试算不等于接受一步。已实现的 `DormandPrince54<State>` 驱动器负责接受/拒绝和下一步长，调用方提供误差度量，拒步不修改已接受时间与状态。详见 [接口与验收](dormand-prince.md)。
`Dop853<State>` 可复用控制框架，但具有自己的系数、误差估计和连续输出公式。

`symplecticEulerStep(position, velocity, dt, acceleration)` 与 `velocityVerletStep(...)`
返回 `std::pair<Vector, Vector>`，顺序为新位置、新速度。
这里限定 `position' = velocity`、`velocity' = acceleration(position)`；不暗含一般速度相关阻力、约束或旋转群运算。
辛性质还要求质量矩阵为常量正定矩阵，力由相应的保守势产生。完整无人机状态不满足这个简单接口时，不应强行代入。

| 前置声明 | 后续必须确定的接口职责 |
| --- | --- |
| `Dop853<State>` | `step/reset`、误差度量、容差、步长上下限、拒步上限、接受状态、统计和连续输出 |
| `RadauIIA5<State>` | `step/reset`、Jacobian、Newton/线性求解策略、误差控制与失败信息 |
| `Bdf<State>` | `initialize/step/reset`、历史和启动、阶数/步长管理、非线性求解；DAE 需另设残差接口 |
| `Rattle<MechanicalState>` | 质量/逆质量、力、`g(q)`、约束 Jacobian、乘子求解、位置和速度残差 |
| `LieGroupIntegrator<State, Geometry>` | 状态与切向量分离、群作用、指数/回缩映射、子阶段组合与欧氏状态耦合 |

不强行用一个 `step(t,x,u,dt)` 抹平所有方法的区别：BDF 需要历史，RATTLE 需要约束，李群算法需要几何结构。
`RadauIIA5` 与 `Bdf` 当前仅预留 ODE 路线；DAE 的残差、初值和指数处理仍是单独 TODO。

## 后续实施与验收

1. **RK4 已完成**：指数衰减、自由落体、恒定机体角速度；用步长 `h`、`h/2`、`h/4` 对照同一终止时刻，检查阶数与阶段时间。
2. **Dormand–Prince 已完成基础版本**：验证嵌入估计、容差变化、拒步不推进、步长下限失败及最大重试次数；分别记录 RHS 调用与耗时。
3. **按模型补几何积分**：用振子验证 Verlet，再用约束摆验证 RATTLE；李群方法验证恒定及变化角速度、流形残差与收敛阶。
4. **出现刚性需求后补隐式法**：先准备 Newton/Jacobian 和收敛失败诊断，验证刚性衰减、步长敏感性及事件后的历史重启。

误差控制要考虑 m、m/s、rad/s 与无量纲状态的不同尺度，并为四元数双覆盖 `q`/`-q` 设计适当误差度量。
姿态投影只在明确的接受步策略下执行，不能归一化导数；投影会影响误差估计和收敛，需要单独验证。
控制更新及事件边界限制步长，拒步不能重复推进控制器内部状态；状态、控制或模型变更后须使缓存导数失效，多步法还要处理历史重启。
内部步长与输出采样周期分开定义，需要连续输出或明确的边界截步规则。

Python 继续负责配置和批量实验，C++ 完成子阶段、拒步和时间循环。
只有实现且验收通过的方法才可供 C++ 仿真核心选择。Python 通过模型参数选择方法，仅访问仿真层的步进、实际输入和物理状态，不导出积分器、数学运算或数值算例函数。

[返回阶段 3](development/03-numerical-integration.md) · [返回开发总览](development-workflow.md)
