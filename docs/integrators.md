# 积分器选择与物理步进

数值算法和所有子阶段均在 C++ 中运行。Python 通过模型参数选择积分方法，仍只调用 `csim.step(model, data)`；不导出积分器对象、导数回调或矩阵运算。默认保持 `rk4`。

## 使用

```python
import csim

model = csim.DroneModel(
    timestep=0.01,
    integrator="dopri5",
    rtol=1e-7,
    atol=1e-10,
    max_substeps=10000,
)
data = csim.make_data(model, position_W=[0, 0, 5])
csim.set_control(model, data, thrust=model.mass * model.gravity)
csim.step(model, data)
print(csim.get_state(model, data)["time"])  # 0.01
```

`PendulumModel`、`SuspendedPayloadModel` 和 `load_model()` 同样接受这四个新增参数。它们在模型上是只读属性；无人机/吊载的 `get_config(model)` 会保存这些参数，实验重建和回放沿用相同方法。`timestep` 仍是一次对外物理步的时间长度。

| `integrator` | 阶数 / 每步导数计算 | 可用模型 | 主要用途与限制 |
| --- | --- | --- | --- |
| `euler` | 一阶 / 1 | 单摆、无人机、吊载 | 低成本调试和收敛对照；振荡问题易积累能量误差 |
| `midpoint` | 二阶 / 2 | 同上 | 显式中点 RK2，不是隐式中点，也不保证辛结构 |
| `heun` | 二阶 / 2 | 同上 | 显式梯形 RK2，平均起点与预测终点斜率 |
| `rk4` | 四阶 / 4 | 同上 | 默认固定步长基准，保持原有物理算例结果 |
| `dopri5` | 五阶推进、四阶误差估计 / 每次试步 7 | 同上 | 已有 Dormand–Prince 5(4) 接入物理模型，内部可拒步及细分 |
| `symplectic_euler` | 一阶 / 1 次位置加速度 | 仅固定悬点平面单摆 | kick–drift，保守系统长期能量误差通常有界，仍受步长限制 |
| `velocity_verlet` | 二阶 / 2 次位置加速度 | 仅固定悬点平面单摆 | kick–drift–kick；在这里的保守、可分离系统中对称且为辛方法 |

一般无人机有姿态动力学、速度相关阻力，吊载还有方向和切向速度约束，不能直接套用 `q'=v, v'=a(q)` 的简单辛方法。给这些模型设置 `symplectic_euler` 或 `velocity_verlet` 会在构造时明确报错，不能偷偷退回 RK4。

## 固定步长公式

Euler：`x1 = x0 + h f(t0,x0)`。

显式中点：`k1=f(t0,x0)`，`k2=f(t0+h/2,x0+h*k1/2)`，`x1=x0+h*k2`。

Heun：`k1=f(t0,x0)`，`k2=f(t0+h,x0+h*k1)`，`x1=x0+h*(k1+k2)/2`。

辛 Euler：`v1=v0+h*a(q0)`，`q1=q0+h*v1`。

Velocity Verlet：`vhalf=v0+h*a(q0)/2`，`q1=q0+h*vhalf`，`v1=vhalf+h*a(q1)/2`。

通用数值函数只进行状态算术，不归一化中间状态或导数，不修改输入；支持标量、Vector3 和提供有限性检查及加法/数乘的状态类型。模型层在完成固定一步后执行自身的投影和有效性检查。辛方法的简单接口不适用于一般速度相关阻力、非完整约束或四元数状态。

## 自适应步进与容差

`dopri5` 每次从当前物理时刻推进到 `time + timestep`，初始内部试步取整个区间，误差过大则拒步缩短，之后可以再次放大。所有内部步都截断在该区间终点，时间不越过下一次 Python 更新边界。

局部误差采用已有嵌入差值 `x5-x4`，对每个原始状态分量计算：

```text
scale_i = atol + rtol * max(abs(x_old_i), abs(x5_i))
error_norm = max_i(abs(error_i) / scale_i)
接受条件：error_norm <= 1
```

位置按 m、速度按 m/s、角速度按 rad/s、单摆角按 rad，四元数及绳索方向分量无量纲。`atol` 是分别作用于这些分量的同一数值，默认 `1e-9`；它不是将不同单位相加所得的一个物理误差。`rtol` 默认 `1e-6`，无量纲。需要比较不同量级/单位时，应根据状态尺度选择容差并做收敛验证；首版尚未提供逐分量绝对容差数组。

四元数误差是原始分量局部误差，不是旋转角距离；同时将初值和轨迹替换为相反符号，归一化误差相同。误差在投影前测量，避免归一化掩盖径向误差。每个已接受子步随后归一化姿态，并在吊载模型中归一化绳方向、投影角速度至切平面，再验证物理状态。此投影不是 RATTLE，也不保证精确能量守恒。

`rtol` 必须有限且非负，`atol` 必须有限且正，所有方法都会校验参数；只有 `dopri5` 使用它们控制精度。`max_substeps` 是每个外部物理步最多接受的内部步数，范围 1–1000000。每个内部步最多尝试 32 次；内部步长下限取区间长度的 `1e-12` 与 `1e-12 s` 的较小值，并受浮点可表示范围限制。到达次数/步长/时间分辨率限制时抛出异常，整个外部步不提交。

没有跨外部步缓存、FSAL 复用或连续输出；每次调用从当前输入重新启动自适应求解。数值容差控制的是局部截断误差，不是总轨迹误差保证。高刚度问题仍需评估隐式方法；不连续风力、SPAD 精确符号换向及绳状态切换也不会因为自适应自动变得光滑。

## 与 Python 控制和模型边界的关系

C++ 在整个外部时间步保持实际推力/力矩输入。`csim_control.ControlLoop` 仍在 Python 计算限幅、分配、响应、平均实际力和延迟；其整数 tick 每次 `loop.step()` 只增加一次。自适应子步不会重新执行 CTBR 或消费 Python 指令。

因此，提高物理积分精度不等于提高 Python 响应模型的离散精度。需要更密集的输入更新时，缩小模型 `timestep`，并相应检查控制周期与延迟的整数步关系。显示频率与积分方法独立。

模型域异常、非有限值和张力失效立即传播，不伪装成普通误差拒步来跨越边界。严格绷紧模型不跨越松绳边界；吊载 `cable_mode="hybrid"` 会在外部步末处理释放和收紧冲量，但尚未做连续时间事件根定位。单摆仍受其原有低能量模型域约束。所有内部候选状态、投影与观测均在提交外部状态前完成；失败后时间、实际输入和物理状态保持不变。

## 实现位置与验证

- `include/csim/numerics/explicit_runge_kutta.hpp`：Euler、中点、Heun。
- `include/csim/numerics/symplectic.hpp`：辛 Euler、Velocity Verlet。
- `include/csim/numerics/dormand_prince.hpp`：已有 5(4) 试步和自适应驱动器。
- `include/csim/simulation/integration.hpp`：模型方法选择、复合状态误差度量、内部步边界与提交策略。

数值测试验证非自治解析解的一阶/二阶收敛，谐振子阶数、10 万步能量误差及 Verlet 反演，状态类型、时间分辨率和异常传播。模型测试验证自由落体、恒定角速度、容差收紧、绳约束、阵风子阶段时刻、失败原子性、加载/记录/回放以及 Python 控制时钟独立性。原 RK4 物理基准继续使用原始期望值。

DOP853、Radau、BDF、RATTLE 和李群积分仍保留 TODO；它们需要各自的误差估计、非线性/约束求解或几何更新，不能用其他算法冒充实现。
