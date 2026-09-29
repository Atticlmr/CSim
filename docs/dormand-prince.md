# Dormand–Prince 5(4) 自适应积分

已实现 [`dormand_prince.hpp`](../include/csim/numerics/dormand_prince.hpp)：七阶段嵌入试算、独立误差度量、自适应接受/拒绝、时间边界及重置。
适用于非刚性显式 ODE，仅在 C++ 内部使用。Python 通过模型 `integrator="dopri5"` 选择，由物理仿真层提供复合状态误差度量及已接受子步的投影/检查，见 [积分器使用](integrators.md)。

采用 Dormand–Prince 的五阶推进与四阶嵌入估计，系数核对 [SciPy RK45 源码](https://github.com/scipy/scipy/blob/main/scipy/integrate/_ivp/rk.py)。
原始方法为 Dormand、Prince（1980）的 *A family of embedded Runge-Kutta formulae*，引用信息亦见该源码。
本项目的误差符号定义为 `x5-x4`，与 SciPy 源码中的 `E` 符号相反；这不影响取范数，但比较带符号误差时须区分。
实现不依赖 SciPy、Boost 或 Python，也不包含 DOP853。

## 单次试算

```cpp
#include <csim/numerics/dormand_prince.hpp>

const auto rhs = [](double /*time*/, double x, double rate) { return -rate * x; };
const auto trial = csim::numerics::dormandPrince54Trial(0.0, 1.0, 2.0, 0.1, rhs);
// trial.state: 五阶候选值；trial.error: x5-x4，尚未决定接受或推进时间。
```

`dynamics(t, const State&, const Control&) -> State` 在同一个试算步内使用固定控制，共调用七次。
阶段时间系数为 `0, 1/5, 3/10, 4/5, 8/9, 1, 1`；最后两次在同一时间计算不同状态。
有限性检查和状态算术要求与 [RK4](rk4.md)一致：支持加法、右侧标量乘法、浮点标量或 `isFinite()`，无需默认零构造。
误差直接由导数的差权重求和得到，避免直接相减两个接近的候选状态；仍有浮点舍入和相消误差。

试算不修改输入，不归一化，不投影，不执行控制器更新，也不自动接受结果。
时间/步长/初值必须有限，步长为正；不同阶段时间必须严格递增且可表示，重复端点除外。
非有限阶段、导数、候选或误差立即失败，模型异常原样传播。
负系数和较大的阶段系数可能产生中间溢出；不保证所有数学上可表示的问题都能在当前算术下计算。

## 自适应驱动器

```cpp
using namespace csim::numerics;
DormandPrince54<double> solver(0.0, 1.0, 0.1);
ScalarErrorNorm norm(1e-7, 1e-9); // rtol, atol
const auto rhs = [](double, double x, double rate) { return -rate * x; };

while (solver.time() < 2.0) {
    const auto report = solver.step(2.0, rhs, norm, 2.0);
    // 一次调用只接受一个内部积分步，可能先进行多次被拒绝的试算。
    // report.step_size / error_norm / attempts / rejected
}
const double final_value = solver.state();
```

构造参数为 `(time, initial_state, first_step, options={})`。`AdaptiveStepOptions` 包含：

| 字段 | 默认值 | 要求 |
| --- | --- | --- |
| `min_step` | `1e-12` | 有限正数 |
| `max_step` | `1.0` | 有限且不小于 `min_step` |
| `max_attempts` | `32` | 正整数，包含成功试算在内 |

初始建议步长必须位于上下限内，不自动修正无效输入。
`time()` 返回已接受时间，`state()` 返回内部已接受状态的只读引用，`nextStepSize()` 返回下一次建议步长。
如需保留历史状态，应复制 `state()` 的值；该引用不是独立快照。
`reset(time, state, first_step)` 验证后重置三者，沿用已有步长配置，失败时不提交新值。

`step(control, dynamics, error_norm, time_bound)` 最多接受一步，返回报告；要到达边界需循环调用。
`time_bound` 必须有限且大于当前时间，区间差值也必须可表示。
每次试算限制在边界内，使用实际可表示的时间增量；返回到达边界时 `time()` 等于传入的边界值。
已到达边界后再次对同一边界调用是参数错误，不是一次空操作。

误差不大于 1 时接受；否则缩短步长并从原状态重试。
步长比例使用 `0.9 * error_norm^(-1/5)`，限制为 `[0.2, 5]`，零误差允许最大增长。
一个调用中发生过拒步后，本次接受不立即建议增大步长；下一建议步长仍限制在配置上下限内。
最终边界的剩余间隔可小于 `min_step`，但如果这一短步被拒绝，则报告失败，不再细分。
这里的边界是已知的控制/采样时间，并不具备未知事件的根定位能力。

## 误差度量

误差回调为 `norm(const old_state, const candidate, const error) -> double`，返回无量纲非负值。
正无穷代表误差太大，可以拒步；NaN、负值或负无穷代表错误的度量结果，立即报错。

内置 `ScalarErrorNorm(rtol, atol)` 仅用于标量，定义为：

```text
abs(error) / (atol + rtol * max(abs(old_state), abs(candidate)))
```

要求 `rtol >= 0`、`atol > 0` 且都有限，内部以 `long double` 计算尺度。
向量、矩阵或物理组合状态需要调用方提供误差度量；不自动给不同单位的量套用同一绝对容差。
四元数等流形状态还需考虑双覆盖及几何意义。测试用原始四元数分量误差范数验证恒定角速度，不把它定义为完整吊载系统的通用误差策略。
局部误差控制不是终点误差的严格上界，也不保证绳长、单位方向、能量或张力条件。

## 失败与所有权

| 异常 | 情况 |
| --- | --- |
| `std::invalid_argument` | 无效输入、步长配置、容差、时间边界或误差度量 |
| `std::overflow_error` | 阶段时间坍缩/溢出、非有限状态或算术结果 |
| `IntegrationFailure`（继承 `std::runtime_error`） | 耗尽试算次数、最小步长无法满足容差、时间增量不能准确对应边界等 |
| 回调或状态算术异常 | 原样传播 |

拒步和失败不修改驱动器已接受的时间、状态或建议步长；一旦成功接受则提交候选状态及报告。
为保证提交阶段不抛出，驱动器要求 `State` 的移动赋值是 `noexcept`，并具有独立值语义。
这不回滚回调自行修改的外部对象，不支持回调重入修改当前驱动器，也不支持共享同一实例的并发推进。

首版不缓存 FSAL 导数，每次试算重新计算全部七个阶段；控制、模型或误差策略变化后不存在过期导数复用。
代价是多一次可复用的 RHS 计算，后续有实测需求再引入缓存及明确失效机制。
该通用数值类不提供连续输出、自动初始步长、姿态投影、松紧绳事件、刚性处理或 DOP853；物理层现在另行负责已接受子步的投影；相关扩展仍保留 TODO。

## 验证记录

```bash
xmake test -v dormand_prince_test/default
```

八组 C++ 检查验证：阶段与误差符号、五阶推进与估计器阶数、容差与时间边界、拒步/重置/控制变化、失败原子性、一般状态与姿态、非法参数、非有限阶段与时间分辨率。
测试不依赖 `assert`，Debug/Release 都执行；也验证不可默认构造状态和不可复制回调。

固定步长 `x'=x, x(0)=1, T=1` 的终点误差如下，步长减半后的误差比趋近五阶的 32：

| 步长 | 绝对误差 |
| --- | --- |
| 0.25 | 4.68428e-7 |
| 0.125 | 1.84911e-8 |
| 0.0625 | 6.46033e-10 |

自适应 `x'=-2x, x(0)=1, T=2`，初始步长 1、`atol=rtol*0.01`：

| rtol | 终点绝对误差 | RHS 次数 |
| --- | --- | --- |
| 1e-4 | 2.69264e-6 | 63 |
| 1e-7 | 1.91888e-9 | 203 |
| 1e-10 | 1.75449e-12 | 756 |

数据为本机双精度实测，末尾位可能随编译器有所差异。这些数值积分检查不替代物理模型验证。

[RK4](rk4.md) · [算法调研](numerical-integration.md) · [阶段 3](development/03-numerical-integration.md)
