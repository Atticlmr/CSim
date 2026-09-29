# LU 分解与线性方程求解

头文件：`<csim/math/lu.hpp>`；类型：`csim::math::PartialPivLU<N>`。
适用于固定大小的实数方阵，使用 `double` 和按列选取最大绝对值主元的行交换。
算法参考 [LAPACK DGETF2 的部分主元 LU](https://www.netlib.org/lapack/explore-html/db/d4e/group__getf2_ga5f8f2998bad179dcee6a58d00599dba8.html)，实现由本项目提供，不依赖 LAPACK。

## 使用方式

```cpp
#include <csim/math/lu.hpp>

using csim::math::Matrix;
using csim::math::Matrix3;
using csim::math::PartialPivLU;
using csim::math::Vector3;

const Matrix3 A{0, 2, 1,
                1, 1, 0,
                2, 0, 1};
const PartialPivLU<3> lu(A);
const Vector3 x = lu.solve(Vector3{3, 3, 1}); // (1, 2, -1)

// 每列是一个右端项；同一分解可重复求解，无需重复分解 A。
const Matrix<3, 2> B{3, 4,
                     3, -1,
                     1, 2};
const auto X = lu.solve(B); // 按行：(1,-1), (2,0), (-1,4)
```

构造时复制并分解 `A`，不修改输入。
`solve(Matrix<N, K>)` 返回 `Matrix<N, K>`，仅 `N=3` 另支持 `solve(Vector3)`。
维度不匹配在编译时拒绝；内部没有动态分配。分解成本为 O(N³)，K 列求解为 O(N²K)。

## 置换约定

本接口采用 **`P * A = L * U`**，`L` 为单位下三角，`U` 为上三角。
`permutation()[i]` 表示新第 i 行来自原矩阵的哪一行，即 `P(i, permutation()[i]) = 1`。
上例返回 `[2, 0, 1]`，因此 `P * b = [b₂, b₀, b₁]ᵀ`。
求解顺序为 `L y = P b`，再求 `U x = y`。其他库可能采用不同置换定义，不能直接混用。

`lower()`、`upper()`、`permutationMatrix()` 返回矩阵副本。
C++ 的 `permutation()` 返回内部数组的只读引用，其生命周期跟随分解对象。
`matrixScale()` 返回原矩阵元素绝对值的最大值，`relativePivotTolerance()` 返回构造时使用的容差。

## 数值阈值与异常

构造函数第二个参数为相对主元阈值，默认 `N * std::numeric_limits<double>::epsilon()`，
也可调用 `PartialPivLU<N>::defaultTolerance()` 查询。
阈值必须是 `[0, 1)` 内的有限值。非零阈值下，若 `|pivot| / max|Aᵢⱼ| <= tolerance` 则拒绝分解；零主元始终拒绝。
使用相对比较使统一缩放后的判定保持一致，浮点舍入与下溢仍可能影响极端输入。

该判据是数值保护，并非严格的秩或条件数估计。例如 `diag(1, 1e-18)` 默认会被拒绝，虽然它在数学上可逆。
显式传入 `0.0` 只检查精确零主元，可用于已了解尺度差异的输入，但不会改善病态问题的求解精度。
当前没有行列平衡、条件数估计或迭代改进，也不承诺任意极端输入均能求解。

| 异常 | 含义 |
| --- | --- |
| `std::invalid_argument` | 系数或右端项包含 NaN/Inf，或阈值不合法 |
| `csim::math::SingularMatrixError` | 零矩阵、零主元或主元未通过阈值；继承 `std::domain_error` |
| `std::overflow_error` | 分解或三角求解产生非有限算术结果 |

分解和求解不会静默返回含 NaN/Inf 的结果，但常规舍入和下溢仍遵循 `double` 浮点运算。
`solve` 失败不修改分解对象或右端项，可修正输入后复用同一个分解。
本接口不提供逆矩阵或行列式；Cholesky、LDLT、QR、SVD 继续保留空头文件与 TODO。

## 验证

```bash
xmake test -v lu_test/default
```

七组测试覆盖：带多次行交换的 `P A = L U`；已知解与多个右端项；1/2/3/5/8 维系统；
统一缩放和次正规数；奇异、近奇异及显式阈值；非法输入；分解与前后代入溢出。
测试同时检查输入不变和分解复用，使用独立的 `long double` 累加计算归一化残差：

```text
||A x - b||∞ / (||A||∞ ||x||∞ + ||b||∞)
```

测试对正常已知解系统要求该值不大于 `1e-14`，并另行比较解。
小残差不能单独保证病态系统的解准确，参见 [LAPACK 的误差与条件数说明](https://www.netlib.org/lapack/lug/node78.html)。
测试不依赖 `assert`，debug/release 均执行检查。

[Python 仿真边界](python.md) · [返回阶段 2](development/02-math-library.md)
