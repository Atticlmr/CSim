# CSim

面向强化学习并行环境的无人机吊载仿真器。C++17 负责动力学、数值积分和 CPU 批量并行，
Python 通过 pybind11 调用，负责控制与实验编排。

- 支持单摆、无人机、质点吊载和偏置吊点双刚体吊载，包含松紧绳切换与收紧冲量。
- 提供显式、隐式、Lie 群等积分方法，以及 OpenGL/GLFW 可视化。
- Python 扩展包含控制器、执行器模型、记录回放、参数辨识和可复现随机化。

当前不包含接触、弹性绳、多绳求解及 Gymnasium 适配；ROS2/PX4 仅预留桥接接口。

## 安装

需要 Python 3.10+、C++17 编译器、pkg-config、TinyXML2，以及 xmake 3.1.1+ 或 CMake 3.20+。
发行包名为 `csim-drone`，导入名为 `csim`。

```bash
sudo apt install pkg-config libtinyxml2-dev
uv venv .venv
CSIM_BUILD_VIEWER=0 uv pip install --python .venv/bin/python .
```

默认通过 xmake 构建；使用 CMake 时在安装命令前增加 `CSIM_BUILD_SYSTEM=cmake`。
修改 C++ 后需重新安装（加 `--reinstall`）并重启 Python 进程。

## 快速开始

```python
import csim

model = csim.DroneModel(timestep=.002)
data = csim.make_data(model)
csim.set_control(model, data, model.mass * model.gravity)
csim.step(model, data)
state = csim.get_state(model, data)
```

C++ 接收实际推力与机体系力矩；CTBR 等控制器位于 `csim_control`。
绷紧吊载创建和重置时须显式提供正张力对应的 `thrust`；需要松绳时使用 `cable_mode="hybrid"`。

三文件刚体吊载输入为无人机、绳子配置和吊载物：

```python
model = csim.load_suspended_model("drone.urdf", "cable.json", "payload.urdf")
```

本体支持 URDF、MJCF 或 JSON，自动合并固定部件的质量、质心和惯量。
吊点名称固定为 `cable_attachment`：URDF 使用同名 fixed link，MJCF 使用同名 site。
详见[模型与配置示例](examples/models/rigid_payload/README.md)。

## CPU 批量仿真

`DroneBatch` 和 `RigidPayloadBatch` 使用 C++ 常驻线程，计算时释放 GIL，整批成功后提交状态。

```python
import csim
import numpy as np

batch = csim.DroneBatch(csim.DroneModel(timestep=.002), num_envs=1024, threads=4)
actions = np.tile([batch.model.mass * batch.model.gravity, 0, 0, 0], (batch.num_envs, 1))
batch.step(actions, substeps=5)
snapshot = batch.get_state()
```

动作形状为 `(N,4)`（推力、力矩 xyz）；状态为无人机 `(N,13)` 或双刚体 `(N,27)`。
`batch.reset(states, indices=[...])` 可局部重置；`threads=1` 用于串行基线。
随机化示例见 [seeded_batch.py](examples/python/seeded_batch.py)。

```bash
.venv/bin/python examples/python/batch_benchmark.py --kind both --num-envs 1024 --threads 4
```

## 构建与测试

xmake 主构建：

```bash
xmake f -m release --viewer=n --python=n
xmake build
xmake test -v
```

CMake 构建：

```bash
cmake -S . -B build-cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build-cmake --parallel 2
ctest --test-dir build-cmake --output-on-failure
```

CMake 默认构建核心 C++ 与测试；`CSIM_BUILD_PYTHON=ON` 启用 Python（需 pybind11），
`CSIM_BUILD_VIEWER=ON` 启用可视化。安装导出的 `CSim::` 目标会传递 C++17 编译要求。
[CI](.github/workflows/ci.yml) 验证物理与数值方法；[打包 CI](.github/workflows/python-package.yml)
验证 wheel、Ninja/Ninja Multi-Config 构建、Python 文件同步和下游链接。

## 可视化与更多示例

可视化需要 GLFW 3.3+、OpenGL 开发库，以及支持 OpenGL 3.3 Core 的图形环境：

```bash
sudo apt install libglfw3-dev libgl1-mesa-dev
CSIM_BUILD_VIEWER=1 uv pip install --python .venv/bin/python --reinstall .
.venv/bin/python examples/python/rigid_payload.py --viewer
```

- [数值方法](examples/python/NUMERICAL_METHODS.md)
- [Python 算例与实验脚本](examples/python/)
- [ROS2/PX4 桥接接口](plugins/README.md)

## 许可证

[Apache-2.0](LICENSE)，版权与声明见 [NOTICE](NOTICE)。再分发时须保留两者及所用第三方依赖的许可声明。
主要依赖为 TinyXML2、pybind11、NumPy，以及可选的 GLFW/OpenGL。

