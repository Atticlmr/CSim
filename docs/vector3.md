# 三维向量接口

头文件：`<csim/math/vector.hpp>`；类型：`csim::math::Vector3`。
第一版采用三个公开的 `double` 分量 `x`、`y`、`z`，默认初始化为零，支持 `Vector3{1.0, 2.0, 3.0}`。
实现为头文件内联函数，由 `csim_numerics` 提供，不需要外部数值库。

## 已实现接口

| 接口 | 行为 |
| --- | --- |
| `a + b`、`a - b`、`-a` | 分量加减及取负 |
| `a * k`、`k * a`、`a / k` | 标量乘除 |
| `+=`、`-=`、`*=`、`/=` | 原地更新，返回对象引用 |
| `a.dot(b)` | 点积 |
| `a.cross(b)` | 右手系叉积 |
| `a.squaredNorm()` | 点积 `a.dot(a)`，可能产生平方的溢出或下溢 |
| `a.norm()` | 使用三分量 `std::hypot` 计算范数，避免直接平方的中间溢出/下溢；真值超出 `double` 范围时仍可能溢出 |
| `a.normalized()` | 返回单位方向，不修改原对象；先缩放再求范数，支持原范数超出可表示范围的有限向量 |
| `a.isFinite()` | 三个分量是否均为有限值 |

零向量及含 NaN/无穷大的向量无法归一化，抛出 `std::domain_error`。
不会用固定 epsilon 把微小但非零的向量自动当作零；应用若有物理阈值，需在调用处明确判断。
除以正零或负零抛出 `std::domain_error`，原地除法在失败时保持对象不变。
其他运算遵循普通浮点运算规则，不自动饱和或检查溢出；点积和叉积也不保证极端输入下仍可表示结果。

该类型不携带坐标系或物理单位，不提供浮点相等运算符。上层应遵守 [建模约定](modeling-conventions.md)，并按实际误差预算比较向量。

## 使用示例

```cpp
#include <csim/math/vector.hpp>

using csim::math::Vector3;

const Vector3 displacement_W{3.0, 4.0, 0.0};
const double distance = displacement_W.norm(); // 5.0
const Vector3 direction_W = displacement_W.normalized(); // (0.6, 0.8, 0)
const Vector3 force_W = 10.0 * direction_W;
```

## 验证

```bash
xmake f -m debug --viewer=n
xmake test -v
xmake f -m release --viewer=n
xmake test -v
```

测试目标为 `vector_test`，注册用例为 `vector_test/default`。
测试包含五组检查：构造与算术、点积与叉积、范数与归一化、极端数值、无效输入。
检查不依赖 `assert`，在定义 `NDEBUG` 的 Release 构建中仍执行；失败打印原因并返回非零退出码。
数值比较采用相对容差 `1e-12` 与绝对容差 `1e-14`；极小范数的测试禁用绝对容差，避免把错误的零结果判为通过。
测试通过 [xmake 的 add_tests 机制](https://xmake.io/guide/basic-commands/run-tests.html) 接入 CI，无需图形环境。

[返回阶段 2](development/02-math-library.md)
