# CSim

无人机吊载仿真：自研 C++17 数学与数值计算库，通过 pybind11 提供 Python 接口。
C++ 负责动力学、积分与批量并行，Python 负责配置、控制与实验编排。

- **数学**：固定大小向量/矩阵、LU、Cholesky、列主元 QR、支持 1×1/2×2 主元的 LDLT、Jacobi SVD、旋转与四元数。
- **数值**：Euler、RK2/RK4、Dopri5、DOP853、隐式 Euler/中点、Radau IIA5、BDF1/2、Lie 中点/RKMK4、辛方法与 RATTLE。
- **物理**：固定悬点单摆、无人机、质点吊载、偏置吊点双刚体吊载，含松紧绳事件与收紧冲量。
- **实验**：`csim_control`（CTBR、单旋翼、执行器响应、指令延迟）、`csim_experiments`（记录、回放、扫描、辨识、随机化）、`csim_bridge`（在环桥接）。
- **可视化**：OpenGL/GLFW 实时窗口与轨迹回放。

待开发：弹性绳、接触、多绳联合求解、ROS2/PX4 实际通信。

> `docs/` 为本地说明目录，不纳入 Git 跟踪，因此本文不引用其中文件。
> 仓库内已跟踪的补充说明：[数值方法](examples/python/NUMERICAL_METHODS.md)、[三文件刚体吊载示例](examples/models/rigid_payload/README.md)、[插件说明](plugins/README.md)。

## 安装

已有虚拟环境时在根目录执行 `uv pip install .`，随后 `import csim`。
发行包名 `csim-drone`，导入名 `csim`，尚未发布到 PyPI。

源码安装需要 [xmake 3.1.1+](https://xmake.io/guide/quick-start.html) 或 CMake 3.20+、C++17 编译器、pkg-config 与 TinyXML2：

```bash
sudo apt install libtinyxml2-dev
uv pip install --python .venv/bin/python .
```

改 C++ 后用 `uv pip install --reinstall .` 并重启 Python 进程。

## Python 快速开始

```python
import csim

model = csim.DroneModel(timestep=.002)
data = csim.make_data(model)
csim.set_control(model, data, model.mass * model.gravity)   # 悬停推力
csim.step(model, data)
state = csim.get_state(model, data)                          # 独立快照
```

三个模型共用同一组入口：`PendulumModel`、`DroneModel`、`SuspendedPayloadModel`，
配合 `make_data` / `step` / `set_control` / `get_state` / `reset`。
`set_control(model, data, thrust, torque_B)` 设置总推力与机体系力矩，C++ 只接收实际推力/力矩。

绷紧吊载模型在创建和重置时必须显式传入 `thrust`，只接受正张力状态；悬停输入为
`(drone_mass + payload_mass) * gravity`。需要松紧事件时用 `SuspendedPayloadModel(cable_mode="hybrid")`：
张力归零转入松弛，距离到达绳长时按冲量收紧。积分器通过 `integrator=` 选择，默认 RK4；
自适应子步不改变对外步长与 Python 控制时钟。

```bash
xmake run csim_python examples/python/pendulum.py
xmake run csim_python examples/python/drone.py
xmake run csim_python examples/python/suspended_payload.py
xmake run csim_python examples/python/payload_tracking.py --speed 1.0
```

## CPU 批量仿真

`DroneBatch` / `RigidPayloadBatch` 用 C++ 常驻线程并行推进，计算期间释放 GIL；
每个环境状态独立，整批成功后提交。

```python
import csim
import numpy as np

model = csim.DroneModel(timestep=.002)
batch = csim.DroneBatch(model, num_envs=1024, threads=4)
actions = np.tile([model.mass * model.gravity, 0, 0, 0], (batch.num_envs, 1))
batch.step(actions, substeps=5)
snapshot = batch.get_state()
```

- 动作 `(N,4)`：实际总推力与机体力矩 xyz，`substeps` 个物理步内保持不变。
- 无人机快照：`state (N,13)`、`control (N,4)`、`time (N,)`；状态列为位置 xyz、速度 xyz、姿态 wxyz、机体角速度 xyz。
- 刚体快照：`state (N,27)`（无人机 13、载荷 13、`slack`），另有 `event_count` / `event_type` / `event_time` / `event_energy_loss`。
- `threads=1` 串行基线，`threads=0` 按硬件线程数自动选择并限制在环境数内；训练时建议显式限线程。
- `batch.reset(states, indices=[...])` 重置指定环境并清除时间与控制，省略索引则重置全部。
- 控制触发的 `release` 事件在设置控制后立即可见，保留到下一次成功物理步，之后不重复报告。

```bash
xmake run csim_python examples/python/batch_benchmark.py --kind both --num-envs 1024 --threads 4
```

当前未提供奖励、终止条件与 Gymnasium 适配。

**随机化**：`csim_experiments.BatchRandomizer` 为每个环境从
`SeedSequence(seed, spawn_key=(environment_id,))` 派生独立 PCG64 流，
结果不依赖线程数、批量大小或采样顺序；`seed` 接受 `[0, 2**128)`，实际值见 `random.metadata`。

```python
from csim_experiments import BatchRandomizer

randomizer = BatchRandomizer(batch, seed=42)
randomizer.reset(seed=42, position_low=[-1, -1, 1], position_high=[1, 1, 2])
noise = randomizer.random.normal(scale=.01, size=3, indices=[3, 1])
```

随机重置以创建时的快照为模板，位置/速度范围为世界系偏移（m、m/s）。
物理状态仍经底层校验，失败时状态、种子与随机流都不提交。接口不修改全局随机状态。

## 三文件刚体吊载

```python
model = csim.load_suspended_model(drone_path, cable_path, payload_path)
```

两个本体支持 URDF、MJCF 或 JSON 且可混用；固定吊点名统一为 `cable_attachment`
（URDF 同名 fixed link，MJCF 同名 site）。质量、质心、惯量与吊点偏置自动解析。
两刚体各有独立姿态与角速度，张力与收紧冲量同时计入两端力矩。

```bash
xmake run csim_python examples/python/rigid_payload.py
xmake run csim_python examples/python/rigid_payload.py --format json
xmake run csim_python examples/python/rigid_payload.py --format mjcf
```

## 模型加载与 link 状态

支持固定连接树、显式惯性、基本几何、OBJ/STL，以及固定部件的质量/质心/惯性合并。
用 `model.link_names`、`csim.get_link_state(model, data, name)`、`csim.get_link_states(model, data)`
查询固定 link 的位置、姿态、速度、加速度及自身质心状态：

```bash
xmake run csim_python examples/python/link_states.py
```

## 在环仿真插件

已预留 [ROS2](plugins/ros2/README.md)、[PX4](plugins/px4/README.md) 目录与 `csim_bridge` 公共接口。
生命周期为连接启动、真值发布、实际推力/力矩轮询、重置与关闭；插件通过 `csim.bridges` entry point 注册。
目前只有接口与本地假桥接示例，实际通信与 SITL/HITL 调度未实现。

```bash
xmake run csim_python examples/python/bridge_stub.py
```

## 目录

```text
include/csim/     math · numerics · model · io · dynamics · simulation
src/              非模板实现
bindings/python/  pybind11 模块 + csim_control / csim_bridge / csim_experiments
apps/             simulator（算例验证）· viewer（OpenGL/GLFW）
tests/            C++ 模块测试 + python/ 绑定测试
examples/python/  调用与实验脚本
plugins/          ROS2 / PX4 桥接预留
configs/          机体参数与场景配置
docs/             本地说明，不纳入版本控制
```

## 构建与运行

### xmake（主构建）

```bash
xmake f -m debug --viewer=n --python=n
xmake build
xmake run simulator
xmake run simulator drone
xmake run simulator payload
xmake test -v
```

`simulator` 默认跑 20 秒被动单摆算例，`drone` 跑悬停与偏航力矩算例，`payload` 跑 10 秒三维无阻尼耦合算例；
三者都校验能量与张力，失败时以非零码退出。`xmake test` 覆盖数学、积分、单摆、无人机、质点/刚体吊载、
松紧绳事件与 CPU 批量仿真，无需图形依赖；启用 Python 后还包括物理接口、模型导入、link 状态、
批量、控制与实验测试。切换 Release 用 `xmake f -m release` 后重新 `xmake build`。

### CMake（备用构建与安装导出）

```bash
cmake -S . -B build-cmake -DCSIM_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-cmake -j 2
ctest --test-dir build-cmake --output-on-failure
```

默认只构建核心 C++ 与测试。可选 `-DCSIM_BUILD_PYTHON=ON`（需 pybind11）、
`-DCSIM_BUILD_VIEWER=ON`（需 GLFW 3.3+、OpenGL、TinyXML2）、`-DCSIM_BUILD_PYTHON_TESTS=ON`。
安装后可用 `find_package(CSim CONFIG REQUIRED)` 引用 `CSim::csim_io` 等目标。
wheel 默认走 xmake；需要 CMake 时设置 `CSIM_BUILD_SYSTEM=cmake`。

## 可视化

默认不参与构建。启用需 pkg-config、GLFW 3.3+、OpenGL 开发库；运行需图形环境与 OpenGL 3.3 Core 驱动。

```bash
sudo apt install pkg-config libglfw3-dev libgl1-mesa-dev
xmake f -m debug --viewer=y
xmake build -j 2
xmake run viewer
```

三文件刚体吊载实时窗口（已装 Python 包）：

```bash
.venv/bin/python examples/python/rigid_payload.py --viewer --integrator lie_rk4
```

可切换 `--format mjcf` / `--format json`，`--screenshot <path>` 截图，`--hidden` 隐藏窗口。
窗口显示两本体的导入几何与姿态，绳索连接实际吊点；松弛绳以两吊点间虚线示意，不模拟柔性绳形。

交互：左拖旋转、右拖平移、滚轮缩放；Space 暂停、N 单步、R 重置、F 跟随、C 恢复相机、T 切换轨迹、Esc 退出。
物理步长与显示帧率分离。也可先录制再回放：

```bash
xmake run csim_python examples/python/record_payload.py build/flight.csv
xmake run viewer build/flight.csv
```

录制保存 JSON 参数与代码版本元数据，不覆盖已有记录。CSV v2 保留载荷姿态、偏置吊点与松绳状态，
viewer 兼容 v1/v2。着色器从可执行文件旁的 `shaders/` 加载，支持离屏帧缓冲截图。

## 持续集成

- [xmake CI](.github/workflows/ci.yml)：Ubuntu 24.04，debug/release × viewer/python 共四个任务。
  构建、跑三个 `simulator` 算例、`xmake test -v`；启用 viewer 时用 Xvfb/Mesa 验证实时与回放截图。
- 同一工作流的 `cmake` 任务：debug/release 两种构建类型下 `cmake --build` 并执行
  `ctest --output-on-failure`，防止 CMake 路径与 xmake 漂移。
- [Python 打包 CI](.github/workflows/python-package.yml)：用 uv 构建 sdist 与 wheel，
  在 Python 3.10–3.13 独立环境验证物理接口与 editable 安装；不自动发布 PyPI。

测试检查在 release 下仍有效，失败以非零退出码报告。

## 模块约定

- `include/csim/` 存放公共头文件，用 `#include <csim/math/vector.hpp>` 形式引用；`src/` 存放非模板实现。
- `math` + `numerics` 组成 `csim_numerics`，不依赖具体物理模型。
- `csim_dynamics` 依赖 `csim_numerics`；`csim_simulation` 依赖 `csim_dynamics`。
- `csim_model` 依赖 `csim_numerics`；`csim_io` 链接 TinyXML2，负责 XML/网格读取与固定刚体合并。
- `bindings/python/` 只导出物理与批量接口，不导出底层数学与积分类型。
- 可视化只读取仿真状态，仿真核心不依赖窗口或绘图库。
- 数学算法需验证数值误差；物理模型需记录坐标系、单位与建模假设。

## 许可证

本项目以 **Apache-2.0** 许可证发布，全文见 [LICENSE](LICENSE)，版权归属见 [NOTICE](NOTICE)。
再分发时须保留这两个文件；修改本项目的文件须标注变更。

### 第三方依赖

分发源码或 wheel 时需保留下列组件的版权与许可声明。

| 依赖 | 用途 | 许可证 | 引入方式 |
| --- | --- | --- | --- |
| [TinyXML2](https://github.com/leethomason/tinyxml2) | URDF/MJCF 解析 | zlib/libpng | 系统库，`csim_io` 链接 |
| [pybind11](https://github.com/pybind/pybind11) | Python 绑定 | BSD-3-Clause | 仅构建期 |
| [NumPy](https://numpy.org/) | 批量快照与数组 | BSD-3-Clause | 运行时依赖 |
| [GLFW](https://www.glfw.org/) | 可视化窗口 | zlib/libpng | 可选，开启 viewer 时 |
| OpenGL | 可视化渲染 | 系统库，遵循发行版许可 | 可选，开启 viewer 时 |
| Python 3 | 运行环境 | PSF-2.0 | 运行环境 |

本仓库不包含上述库的源码，它们通过系统包管理器、pip 或构建时探测引入。
若将来以 vendoring 方式内置第三方头文件（例如传感器模块的 BVH 库），必须在该文件旁保留原始版权声明与许可证全文，并更新本节。

