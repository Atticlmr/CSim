# 数值方法与刚体吊载可视化

## 分层与选择

模型构造使用 `integrator="..."`，`get_config` 返回所选方法。默认 `rk4`。
物理引擎接收实际总推力 `thrust` 和机体系力矩 `torque_B`。
CTBR、姿态/角速度反馈、旋翼分配、执行器响应和指令延迟都保留在纯 Python 的
`csim_control`；`ControlLoop` 现支持 `RigidPayloadModel`。
内步细分不改变外部物理步长和 Python 控制更新时钟。

| 配置名 | 阶数/实现 | 模型范围 |
| --- | --- | --- |
| `euler` | 显式 Euler，1 阶 | 全部 |
| `midpoint`, `heun` | 显式 RK2，2 阶 | 全部 |
| `rk4` | 经典 RK4，4 阶 | 全部 |
| `dopri5` | Dormand–Prince 5(4)，自适应 | 全部 |
| `dop853` | DOP853 8(5,3)，自适应 | 全部 |
| `implicit_euler` | 隐式 Euler，1 阶，Newton | 全部 |
| `implicit_midpoint` | 隐式中点，2 阶，Newton | 全部 |
| `radau5` | 三阶段 Radau IIA，5 阶，Newton＋步长加倍估计 | 全部 |
| `bdf1`, `bdf2` | BDF1/2，Newton＋步长加倍估计 | 全部 |
| `lie_midpoint`, `lie_rk4` | 局部旋转图中点/RKMK4，2/4 阶 | 无人机、质点吊载、刚体吊载 |
| `symplectic_euler`, `velocity_verlet` | 辛 Euler/Verlet | 平面单摆 |
| `rattle` | Cartesian RATTLE，2 阶 | 平面单摆、无空气阻力的质点吊载 |

`DroneBatch` 和 `RigidPayloadBatch` 支持上述适用于各自模型的方法；批量和逐个模型步进数值一致。
`RigidPayloadBatch` 的状态数组为 `(N,27)`，列顺序是无人机 13 项、载荷 13 项和 `slack` 标志；事件数组记录一次批量调用（包括其 `substeps`）内的所有事件。

`csim.step(model, data, thrust=..., torque_B=...)` 原子地设置实际输入并推进一个物理步；失败时状态、绳模式、输入、时间和事件都保持原值。Python `ControlLoop` 使用这个入口，CTBR 仍完全属于 Python 控制层。

批量 `reset` 要求 `slack` 为 `0/1`，严格绷紧模型拒绝松绳状态。hybrid 模型会根据复位后的零输入释放失去张力的绳。严格模型的批量初始化和复位保留零输入，调用 `step` 时须提供维持张力的有效动作。

`csim_experiments.model_from_config(csim.get_config(model))` 支持刚体吊载，并可独立于原始 URDF/MJCF 文件恢复质量、惯量、吊点、初始姿态和数值设置。恢复的是聚合物理模型；源 link 树和视觉网格仍需原始描述文件。实验 JSON 记录和确定性回放保留完整刚体状态及事件。

`record_payload.py` 接受 `--drone`、`--cable`、`--payload` 三个文件参数，输出 viewer CSV v2，包含载荷四元数、世界坐标吊点、松绳标志和绳长；C++ viewer 同时支持旧 CSV v1。CSV 用于姿态展示，确定性动力学重演使用实验 JSON。
刚体吊载没有 RATTLE 适配：偏置吊点的转动约束需要专门的刚体离散求解器；
选择 `rattle` 会明确报错。质点吊载的 RATTLE 支持 hybrid 松紧绳事件，但拒绝空气阻力。
辛方法的长时间能量性质针对保守系统，不保证有推力、阻力或冲击时能量守恒。

## 隐式方法、自适应和旋转

Newton 使用有限差分 Jacobian、带部分主元的 LU 和回溯；非线性迭代最多 20 次。
Radau/BDF 通过一次粗步与两次细步估计误差，失败时细分，受 `max_substeps` 限制。
固定隐式 Euler/中点不自动细分，Newton 不收敛则整步失败。
所有方法失败时保留原物理状态和时间；Python 控制层的既有回滚机制同样适用。
`rtol`、`atol` 是原始 SI 状态分量的相对/绝对容差，四元数分量无量纲。
这是 ODE 加投影实现，不是通用 DAE 求解器。

低层 `numerics::Bdf<State>` 保存跨步历史并支持变步长 BDF2，步长增长比例限制为 2；
`reset` 清除历史。物理模型适配器在投影、控制和事件边界重新启动历史：
BDF2 的每个试算区间使用两个等长子步，先以隐式中点启动，再执行 BDF2。
这一选择保持二阶精度并避免使用失效的控制/约束历史，但不同于持续历史的生产级变阶 BDF。
Radau/BDF 的稠密有限差分 Newton 成本通常高于 RK4，刚性问题应结合精度和耗时评估。

DOP853 使用独立的 12 阶段八阶 tableau 和五阶/三阶组合误差估计，步长指数为 1/8；
没有 FSAL 缓存或连续输出。Radau 和 DOP853 都不是其他显式方法的别名。
Lie 方法在局部旋转向量图中推进，以指数映射更新姿态；RKMK4 使用截断逆 dexp。
质点绷紧绳方向也在旋转图中更新；松弛阶段使用自由 Cartesian 状态。

```python
import csim
from csim_control import ControlConfig, ControlLoop

model = csim.load_suspended_model(
    "examples/models/rigid_payload/drone.urdf",
    "examples/models/rigid_payload/cable.json",
    "examples/models/rigid_payload/payload.urdf",
    timestep=0.002, integrator="lie_rk4",
)
data = csim.make_data(model, thrust=15, position_W=[0, 0, 3])
loop = ControlLoop(model, data, ControlConfig(mode="ctbr", controller_period=0.004))
loop.set_ctbr(15, [0, 0, 0.1])
loop.step()
```

## 矩阵分解 C++ 接口

公共数学层独立于具体动力学和 Python 控制。均使用 `Matrix<Rows, Cols>`，支持多列右端项。
分解时缓存因子，后续 `solve` 不重复分解。

| 头文件 / 类型 | 因子关系和接口 | 适用条件 |
| --- | --- | --- |
| `cholesky.hpp` / `Cholesky<N>` | `A = L Lᵀ`；`lower`, `solve` | 对称正定，非正定/阈值下主元报错 |
| `qr.hpp` / `HouseholderQR<M,N>` | `A P = Q R`；`orthogonal`, `upper`, `permutationMatrix`, `rank`, `solve` | 可分解长/宽/秩亏矩阵；求解要求满列秩且 M≥N |
| `ldlt.hpp` / `PivotedLDLT<N>` | `P A Pᵀ = L D Lᵀ`；`lower`, `diagonal`, `blockSizes`, `permutationMatrix`, `solve` | 对称可逆，可不定；D 含 1×1/2×2 块 |
| `svd.hpp` / `JacobiSVD<M,N>` | `A = U Σ Vᵀ`；`left`, `right`, `singularValues`, `rank`, `solve`, `pseudoinverse` | 长/宽/零/秩亏矩阵；最小范数最小二乘 |

SVD 返回薄 U/V，奇异值降序；使用单边 Jacobi，宽矩阵通过转置分解。
默认最多 100 个 sweep，超限抛出 `DecompositionFailure`。
LDLT 使用对称完全主元搜索，根据最大对角/非对角值选择 1×1 或 2×2 主元，
不是未带主元的 LDLT，也不声称与 LAPACK 的 Bunch–Kaufman 主元序列相同。
对称检查容差为 `64*N*epsilon`（相对于最大元素）；输入不会被修改。

各类型构造函数第二参数是相对阈值（有限且位于 `[0,1)`），默认最大维度乘机器精度。
Cholesky/LDLT 的主元阈值相对于输入最大元素；QR 的秩阈值相对于最大 R 对角元；
SVD 的秩和伪逆截断阈值相对于最大奇异值。设为零仅按非零值判断，可能放大舍入误差。
所有接口拒绝非有限输入，算术溢出报错；QR 秩亏求解应改用 SVD。
矩阵分解是 C++ 公共 API，本次没有新增 Python NumPy 分解接口或 BLAS 后端。

```cpp
#include <csim/math/cholesky.hpp>
#include <csim/math/svd.hpp>

csim::math::Matrix<2, 2> matrix{4, 1, 1, 3};
csim::math::Matrix<2, 1> rhs{1, 2};
const auto solution = csim::math::Cholesky<2>(matrix).solve(rhs);
const auto inverse = csim::math::JacobiSVD<2, 2>(matrix).pseudoinverse();
```

## 可视化与验证

```bash
.venv/bin/python examples/python/rigid_payload.py --viewer --integrator lie_rk4
.venv/bin/python examples/python/rigid_payload.py --format mjcf --hidden --screenshot build/rigid-payload.ppm
xmake run csim_python examples/python/batch_benchmark.py --kind both --num-envs 1024 --threads 4
xmake test -v
xmake run csim_python tests/viewer/python_viewer_test.py
```

图形测试需要显示服务；CI 使用 Xvfb，隐藏窗口也需要显示服务。
Viewer 是被动快照接收器，不推进物理或控制时钟。
两刚体使用各自导入的几何和姿态，绳端为世界坐标实际吊点；没有可视几何时使用默认形状。
松弛绳为吊点间的虚线示意，未求解柔性绳形。刚体实验 CSV 回放仍未接入。

验证包括积分阶数、刚性解析解、RATTLE 长时间约束/能量与时间反演、
非交换旋转的 Lie 收敛阶、DOP853 拒绝与边界、模型约束和失败回滚、CPU 批量一致性，
以及矩阵重构、正交性、最小二乘正规条件、伪逆 Moore–Penrose 恒等式、
确定性随机矩阵、`1e-250`/`1e250` 尺度和非法输入。
图形测试覆盖被动同步、URDF/MJCF 同图、载荷姿态和松弛状态；批量测试覆盖刚体状态展平、事件汇总、选择性 reset、原子失败和并发调用。

Tableau 与约束公式参考：
[Hairer 的 DOP853 系数](https://www.unige.ch/~hairer/prog/nonstiff/dop853.f)、
[SciPy Radau IIA 系数](https://github.com/scipy/scipy/blob/main/scipy/integrate/_ivp/radau.py)、
[CCP5 RATTLE 公式](https://summer.ccp5.ac.uk/WORKSHOP/Day_6/Constraints/MD-Constraints.html)、
[Wuppertal Lie 群积分讲义](https://acm.uni-wuppertal.de/fileadmin/mathe/www-num/teaching/ode_1819/script_1819.pdf)。
