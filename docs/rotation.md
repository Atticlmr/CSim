# 旋转矩阵与四元数

头文件：`<csim/math/rotation.hpp>`、`<csim/math/quaternion.hpp>`，命名空间为 `csim::math`。
接口遵循 [建模约定](modeling-conventions.md)：右手系、角度为 rad、列向量变换，姿态从机体系 B 映射到世界系 W。

## 旋转矩阵

| 接口 | 含义 |
| --- | --- |
| `rotationX(angle)` | 绕正 X 轴按右手规则旋转 |
| `rotationY(angle)` | 绕正 Y 轴按右手规则旋转 |
| `rotationZ(angle)` | 绕正 Z 轴按右手规则旋转 |
| `rotationFromRollPitchYaw(roll, pitch, yaw)` | 返回 `Rz(yaw) * Ry(pitch) * Rx(roll)` |
| `skew(v)` | 返回叉乘矩阵，满足 `skew(v) * u = v.cross(u)` |
| `isRotationMatrix(R, tolerance)` | 检查有限值、正交性和行列式 +1，默认容差为 `1e-10` |

```cpp
#include <csim/math/quaternion.hpp>

using namespace csim::math;
constexpr double pi = 3.14159265358979323846;
const Matrix3 R_WB = rotationZ(pi / 2);
const Vector3 forward_W = R_WB * Vector3{1, 0, 0}; // 约为 (0, 1, 0)
const Vector3 forward_B = R_WB.transposed() * forward_W;
```

`Matrix3` 本身仍是通用矩阵，不强制旋转约束。`isRotationMatrix` 检查 `RᵀR - I` 的最大元素绝对值及 `|det(R) - 1|` 是否均不超过容差，
并检查元素是否处于 `[-1-tolerance, 1+tolerance]`。这也避免对明显异常的大数做溢出的矩阵乘法。
容差必须是 `[0, 1)` 内的有限值；非法容差抛异常，非法矩阵返回 `false`。容差为零要求浮点结果精确满足检查，通常仅适合精确单位矩阵等输入。
反射矩阵即使正交也不属于有效姿态，因为其行列式为 -1。

## Quaternion 数据与算术

`Quaternion` 是四个 `double` 分量的聚合类型，顺序为 `(w, x, y, z)`。
默认 `Quaternion{}` 为单位旋转 `(1, 0, 0, 0)`；**零四元数必须显式写为 `Quaternion{0, 0, 0, 0}`**，例如零状态导数。
`Quaternion::identity()` 也返回单位旋转。公开分量可直接修改，类型本身不强制单位长度。

| 接口 | 行为 |
| --- | --- |
| `+`、`-`、取负、标量 `*` / `/` | 普通分量运算，用于后续数值积分；提供对应原地运算 |
| `q1 * q2` | Hamilton 乘法，不自动归一化；先应用右侧旋转 |
| `dot(other)`、`squaredNorm()`、`norm()` | 四维内积、模平方和模 |
| `isFinite()` | 四个分量是否均有限 |
| `conjugated()` | 返回 `(w, -x, -y, -z)` |
| `normalized()` | 返回单位长度副本，拒绝零或非有限四元数 |
| `inverse()` | 返回一般四元数的逆 `conjugate(q) / |q|²`，不限于单位四元数 |

对 `q = (w, v)`，Hamilton 乘法为：

```text
(w1, v1) * (w2, v2) = (w1*w2 - v1·v2, w1*v2 + w2*v1 + v1×v2)
```

`q_WB = q_WA * q_AB` 表示 B → A → W，与 `R_WB = R_WA * R_AB` 一致，交换顺序通常会改变结果。
仅对单位四元数，共轭才等于逆。单位 `q` 与 `-q` 表示同一旋转，不能用分量相等判断姿态相等。
测试使用单位四元数点积的绝对值或比较旋转矩阵；本接口尚未提供专用姿态误差函数。

## 构造与转换

| 接口 | 行为 |
| --- | --- |
| `Quaternion::fromAxisAngle(axis, angle)` | 归一化旋转轴，返回单位四元数；即使角度为零也要求非零且有限的轴 |
| `Quaternion::fromRollPitchYaw(roll, pitch, yaw)` | 按 `qz * qy * qx` 组合，返回单位四元数 |
| `Quaternion::fromRotationMatrix(R, tolerance)` | 先验证旋转矩阵，再转换并归一化；默认容差为 `1e-10` |
| `q.toRotationMatrix()` | 归一化副本后计算旋转矩阵，不修改原四元数 |
| `q.rotate(v_B)` | 归一化副本后旋转向量，返回 `v_W`，不修改输入 |

```cpp
const auto q_WB = Quaternion::fromAxisAngle(Vector3{0, 0, 1}, pi / 2);
const Vector3 v_W = q_WB.rotate(Vector3{1, 0, 0});
const Vector3 v_B = q_WB.inverse().rotate(v_W);
const auto recovered = Quaternion::fromRotationMatrix(q_WB.toRotationMatrix());
```

`rotate` 与 `toRotationMatrix` 接受任意有限非零尺度的四元数：例如 `q`、`2*q` 与 `-q` 产生相同旋转。
这能处理轻微模长漂移，但不会把存储的姿态状态自动变成单位长度；后续积分层仍需制定归一化策略。

矩阵转四元数时，从 `4w²`、`4x²`、`4y²`、`4z²` 中选择最大分量作为起点，避免在接近 180° 时除以接近零的 `w`。
转换不保证固定符号或沿时间序列连续选符号；轨迹插值时需另行处理。
输入容差只用于接纳浮点误差，不会对任意坏矩阵执行正交化。通过容差的近似矩阵经转换后会得到单位四元数，其返回矩阵可与输入略有差别；这也不是最近旋转矩阵的优化算法。

Euler 角目前仅提供输入构造，包含 `pitch = ±π/2` 时的正常构造；尚未提供逆向 Euler 提取、球面插值或指数/对数映射。

## 机体系角速度导数

```cpp
const Vector3 Omega_B{0, 0, 1}; // rad/s
const Quaternion dq_WB = q_WB.derivativeBodyRate(Omega_B);
```

该接口实现 `dq_WB/dt = 0.5 * q_WB * (0, Omega_B)`，角速度分量必须在机体系 B 中。
世界系角速度对应不同的乘法顺序，不能直接传入本接口。
它检查有限输入和输出，但不归一化四元数或其导数；有限零四元数的导数也按代数公式得到零，姿态合法性由使用者检查。
对单位姿态应满足 `q · dq = 0`，并对应矩阵运动学 `dR_WB/dt = R_WB * skew(Omega_B)`。
这提供瞬时运动学导数；已实现的 [RK4](rk4.md) 可推进四元数，恒定机体角速度算例已验证归一化策略与四阶收敛。

## 数值与异常约定

`normalized()` 和 `inverse()` 先按最大分量缩放，避免先平方导致的额外溢出/下溢。
`rotate()` 在计算前缩放向量，在恢复尺度后检查结果。仍遵循 `double` 的舍入与表示范围，不保证任意极端输入都能保留相对精度。
`norm()` 使用 `hypot`；真实模超出范围时仍可返回 Inf。`squaredNorm()` 和原始算术不额外过滤 NaN/Inf，与向量、矩阵的原始算术一致。

| C++ 异常 | 条件 |
| --- | --- |
| `std::domain_error` | 零/非有限四元数的归一化、求逆或姿态查询；零/非有限轴；标量除零 |
| `std::invalid_argument` | 非有限角度、待旋转向量或角速度/导数输入；非法转换矩阵或容差 |
| `std::overflow_error` | 逆、旋转后向量或姿态导数出现非有限结果 |

基础四元数乘法不是带溢出保护的姿态组合函数，大尺度原始四元数宜先归一化再组合。
姿态查询返回副本；除零检查在原地修改前执行，失败不修改输入。

## 验证范围

`rotation_test/default` 包含八组检查：原始四元数算术、主轴与叉乘矩阵、旋转组合与 Euler 输入、归一化与逆、矩阵往返转换、向量旋转、机体角速度运动学、异常与有效性检查。
用独立的 Rodrigues 公式检查六种旋转轴、十种转角，覆盖零角、微小角、精确/接近 180° 和整周旋转。
另外检查正交性、行列式、长度保持、Hamilton 向量变换，并用中心差分验证 `Rdot = R * skew(Omega_B)`。
常规比较容差为 `1e-12 * max(1, |actual|, |expected|)`；导数差分步长 `1e-5`，比较容差 `2e-9`。
姿态运算在 C++ 测试，不直接导出到 Python；未来可通过物理状态快照读取姿态分量。

[Python 开发指南](python.md) · [返回阶段 2](development/02-math-library.md)
