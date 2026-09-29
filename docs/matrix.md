# 固定大小矩阵接口

头文件：`<csim/math/matrix.hpp>`；类型：`csim::math::Matrix<Rows, Cols>`。
行列数为编译期正整数，元素固定使用 `double`，通过内部 `std::array` 按行连续存储，无动态分配。
常用的 `Matrix3` 是 `Matrix<3, 3>` 的别名。实现随头文件提供，由 `csim_numerics` 导出。

## 构造与访问

```cpp
#include <csim/math/matrix.hpp>

using csim::math::Matrix;
using csim::math::Matrix3;

Matrix<2, 3> zero;                 // 默认全零，zero{} 也为全零
Matrix<2, 3> a{1, 2, 3, 4, 5, 6}; // 按行排列：第一行 1,2,3，第二行 4,5,6
a(1, 2) = 7;                     // 行列索引均从 0 开始
const auto identity = Matrix3::identity();
```

显式提供初始化列表时，元素数量必须等于 `Rows * Cols`；不执行隐式填充或截断，数量错误抛出 `std::invalid_argument`。
`Matrix<2, 2>{1}` 不表示全 1 或单位矩阵。空花括号 `Matrix<2, 2>{}` 调用默认构造生成零矩阵。

`operator()(row, col)` 提供可写和只读访问，均检查边界；越界抛出 `std::out_of_range`，Release 下也生效。
`rows` 和 `cols` 是类型的静态常量。复制矩阵会复制数值，修改副本不影响原对象。

## 已实现运算

| 接口 | 行为与维度要求 |
| --- | --- |
| `A + B`、`A - B`、`-A` | 分量加减与取负，加减双方尺寸须相同 |
| `A * k`、`k * A`、`A / k` | 标量乘除 |
| `+=`、`-=`、`*=`、`/=` | 原地更新；乘除的右操作数为标量 |
| `A * B` | `Matrix<R, K>` 乘 `Matrix<K, C>` 得到 `Matrix<R, C>` |
| `A.transposed()` | 返回 `Matrix<C, R>`，不修改原矩阵 |
| `Matrix<N, N>::identity()` | 返回单位矩阵；非方阵调用会触发编译错误 |
| `Matrix3 * Vector3` | 把三维向量视为列向量，返回 `Vector3` |
| `A.isFinite()` | 所有元素是否均为有限值 |

矩阵乘法遵循通常的线性代数定义，不是逐元素相乘；尺寸不匹配在编译时拒绝。
不提供 `Vector3 * Matrix3`，避免把列向量与行向量的意义混用。
需要原地计算矩阵乘积时可写 `A = A * B`，右侧先产生独立结果。
通用列向量暂可使用 `Matrix<N, 1>`；仅 `Matrix3` 提供与现有 `Vector3` 的直接乘法接口。

除以正零或负零抛出 `std::domain_error`，原地除法失败时不修改原矩阵。
标量除法直接逐元素相除，避免对极小标量先求倒数导致额外溢出。
其他运算遵循普通浮点规则，不自动检查溢出、饱和或过滤 NaN；矩阵乘法不保证极端数值或严重相消情况下的精度。
逆矩阵与行列式尚未实现；LU 分解和线性方程求解由独立的 [lu.hpp 接口](lu.md) 提供。

## 坐标变换示例

遵循 [建模约定](modeling-conventions.md)，变换写为 `v_W = R_WB * v_B`。
按行存储只是内存布局，不改变列向量乘法的含义。

```cpp
using csim::math::Vector3;

const Matrix3 R_WB{0, -1, 0,
                  1,  0, 0,
                  0,  0, 1}; // 绕世界正 Z 轴正转 90°
const Vector3 forward_B{1, 0, 0};
const Vector3 forward_W = R_WB * forward_B; // (0, 1, 0)
const Vector3 recovered_B = R_WB.transposed() * forward_W;
```

上例手动给出一个正交旋转矩阵；`Matrix3` 本身不会保证矩阵正交，也不负责生成姿态。
已有的主轴旋转构造、有效性检查与四元数转换见 [旋转接口](rotation.md)。
仅对正交矩阵，转置才等于逆。通用矩阵不能使用转置代替求逆或求解方程。
组合 `R_WB = R_WA * R_AB` 的作用顺序是先从 B 到 A，再从 A 到 W。
该类型不携带坐标系、单位或正定性信息；惯性矩阵和姿态接口需在后续阶段验证自己的物理约束。

## 测试与验收

```bash
xmake f -m debug --viewer=n
xmake test -v
xmake f -m release --viewer=n
xmake test -v

# 只运行矩阵测试
xmake test -v matrix_test/default
```

`matrix_test/default` 包含六组检查：构造与访问、算术、矩阵乘积、单位矩阵与转置、向量变换、异常与极端输入。
已知结果覆盖非方阵乘法、非交换性、正交变换和对角惯性矩阵乘角速度。
测试还验证越界和错误初始化、除零失败不修改数据、微小标量除法及非有限值检查。
编译期检查验证结果维度、`constexpr` 运算和乘法维度不匹配。

测试不依赖 `assert`，采用与向量测试相同的浮点比较容差，Release 下仍会报告失败。
本地额外检查零维矩阵和非方阵单位矩阵无法编译，并在临时文件中修改期望值验证 Release 失败退出码。
现有 CI 执行 `xmake test -v`，会自动运行向量、矩阵、LU、姿态与 RK4 测试，以及单摆测试及已启用的 Python 仿真接口检查。

[返回阶段 2](development/02-math-library.md)
