# CSim

用于无人机吊载仿真的项目，自研 C++ 数学与数值计算基础库，通过 pybind11 提供 Python 调用接口。
后续主要使用 Python 脚本配置和运行实验，C++ 负责动力学与数值计算。
已有虚拟环境时在根目录执行 `uv pip install .`，随后直接 `import csim`。
源码安装需要 xmake 和 C++17 编译器；匹配的 wheel 可免编译安装，见 [uv 安装与分发](docs/python-installation.md)。
当前已实现 OpenGL/GLFW 三维可视化与轨迹回放，实现三维向量、固定大小矩阵、LU 分解与线性求解、旋转矩阵、四元数、Euler、中点/Heun RK2、辛 Euler/Verlet、固定步长 RK4 和 Dormand–Prince 5(4) 自适应积分；固定悬点平面单摆、无吊载无人机、绷紧绳索质点吊载耦合及 Python 控制、步进与状态读取已实现；CTBR 内环、单旋翼输入、执行器响应和指令延迟由纯 Python 的 `csim_control` 提供，C++ 只接收实际推力/力矩；默认关闭的 SPAD、两端相对空气速度阻力、确定性风场、受约束旋翼分配、可重复 Python 实验及可选绷紧/松弛绳索已实现；弹性绳、接触和连续时间冲量根定位仍待开发。

开发计划已拆分为七份阶段文档，入口见 [开发流程总览](docs/development-workflow.md)；
构建、验证和提交步骤见 [日常开发流程](docs/development/daily-workflow.md)。
物理模型见 [单摆](docs/pendulum.md)、[无吊载无人机](docs/drone.md) 与 [无人机吊载耦合](docs/suspended-payload.md)。无人机吊载的坐标与状态定义见 [建模约定](docs/modeling-conventions.md)，已实现功能见 [三维向量接口](docs/vector3.md)、[固定大小矩阵接口](docs/matrix.md)、[LU 求解接口](docs/lu.md)、[旋转与四元数](docs/rotation.md) 和 [Python 开发指南](docs/python.md)。
松紧绳扩展的研究依据见 [文献调研](docs/slack-taut-impact-research.md) 与 [RotorTM 详细解析](docs/rotortm-analysis.md)，包含单绳冲量、多机联合求解及验收设计。
已实现的积分接口见 [RK4](docs/rk4.md) 与 [Dormand–Prince](docs/dormand-prince.md)，其他积分算法的选择与接口占位见 [数值积分调研](docs/numerical-integration.md)，包括自适应、辛、刚性及约束/李群方法。
论文建模与实现见 [Zhu 等（2025）建模与可选 SPAD 阻尼解析](docs/zhu-2025-payload-model.md)、[CTBR、单旋翼推力及响应延迟](docs/control-and-response-models.md)。实际配置见 [空气力与风场](docs/aerodynamics.md)，记录、回放、参数扫描和旋翼响应辨识见 [可重复实验](docs/reproducible-experiments.md)。
模型加载见 [URDF/MJCF 与 TinyXML2](docs/model-import.md)，支持固定连接树、显式惯性、基本几何、OBJ/STL 及固定部件质量/质心/惯性合并。通过 `model.link_names`、`csim.get_link_state(model, data, name)` 和 `csim.get_link_states(model, data)` 查询固定 link 的位置、姿态、速度、加速度及自身质心状态；运行示例为 `examples/python/link_states.py`。
[Python 实时窗口](docs/python-viewer.md) 已提供 Viewer、sync(data)、is_running() 和 close()，显示频率不改变物理结果。

积分器可通过模型 `integrator=` 选择，默认 RK4；自适应子步不改变对外步长和 Python 控制时钟，见 [积分器使用](docs/integrators.md)。

## 目录

```text
CSim/
├── xmake.lua
├── pyproject.toml          # uv / PEP 517 安装入口
├── setup.py                # setuptools 调用 xmake 的打包适配
├── MANIFEST.in             # 源码包文件清单
├── README.md
├── LICENSE
├── .gitignore
├── include/csim/
│   ├── math/              # 向量、矩阵、四元数与线性求解
│   ├── numerics/          # 数值积分等算法
│   ├── model/             # 固定机架描述、校验与合并
│   ├── io/                # URDF/MJCF 与网格读取接口
│   ├── dynamics/          # 无人机、绳索与吊载模型
│   └── simulation/        # 仿真循环、状态与结果接口
├── src/                   # 按上述模块划分的实现与内部头文件
├── bindings/python/       # pybind11 模块及构建配置
│   ├── csim_control/      # Python 控制、分配、响应及延迟
│   └── csim_experiments/  # Python 外环、实验与记录
├── apps/
│   ├── simulator/         # 命令行仿真入口
│   └── viewer/            # OpenGL/GLFW 可视化程序
│       ├── xmake.lua
│       ├── main.cpp
│       ├── camera.hpp
│       ├── camera.cpp
│       ├── renderer.hpp
│       ├── renderer.cpp
│       └── window.cpp    # Python 被动窗口；着色器在 bindings/python/csim_viewer/shaders/
├── tests/                 # C++ 模块测试及 python/ 绑定测试
├── examples/python/       # Python 调用与实验脚本
├── configs/               # 机体参数与场景配置
├── docs/                  # 方程推导、坐标系和单位约定
└── build/                 # 本地构建产物，不提交
```

空目录使用 `.gitkeep` 保留，添加实际文件后可移除对应占位文件。
`include/csim/math/` 中的 `vector.hpp`、`matrix.hpp`、`lu.hpp`、`rotation.hpp` 和 `quaternion.hpp` 已实现；其他数学公共头文件仍为占位文件。
`numerics/rk4.hpp`、`dormand_prince.hpp` 的 5(4) 试算和驱动器已实现；DOP853 仍是前置声明，其他积分函数使用 `= delete` 占位，复杂求解器只前置声明。

## 构建与运行

需要 [xmake 3.1.1 或更高版本](https://xmake.io/guide/quick-start.html)，
以及支持 C++17 的编译器、pkg-config 和 TinyXML2 开发库；CI 固定使用 xmake 3.1.1。
Ubuntu 安装 `libtinyxml2-dev`；Python 默认安装含 viewer，还需 `libglfw3-dev libgl1-mesa-dev`。

```bash
xmake f -m debug --viewer=n --python=n
xmake build
xmake run simulator
xmake run simulator drone
xmake run simulator payload
```

命令行默认运行 20 秒被动单摆示例，输出摆角、张力和能量误差；`simulator drone` 运行无人机悬停及偏航力矩算例，输出位置、角速度和姿态；`simulator payload` 运行 10 秒三维无阻尼耦合算例，输出吊载位置、最小张力和输入功校正后的能量误差。暂不读取场景配置。

运行单元测试：

```bash
xmake test -v
```

`xmake test` 会构建并运行 `vector_test`、`matrix_test`、`lu_test`、`rotation_test`、`rk4_test`、`dormand_prince_test`、`pendulum_test`、`pendulum_simulation_test`、`drone_test`、`drone_simulation_test`、`suspended_payload_test`、`suspended_payload_simulation_test` 和 `viewer_test`，以及 `explicit_methods_test`，共 81 组 C++ 检查，无需图形依赖。启用 Python 后还会执行 68 项 Python 物理、模型导入、link 状态、控制和实验测试，另有 4 项图形测试。

切换 Release 使用 `xmake f -m release`，然后重新执行 `xmake build`。
构建产物位于 `build/<平台>/<架构>/<模式>/`，通过 `xmake run` 无需手写路径。
项目配置保存在 `.xmake/`，构建输出与配置缓存均不提交。

## Python 开发入口

推荐直接使用已有 `.venv`：

```bash
uv pip install --python .venv/bin/python .
.venv/bin/python examples/python/hover.py
.venv/bin/python examples/python/tracking.py
.venv/bin/python examples/python/payload_swing.py
.venv/bin/python examples/python/hover.py --model examples/models/drone.xml
```

发行包名为 `csim-drone`，导入名为 `csim`，当前尚未发布到 PyPI。
修改 C++ 后使用 `uv pip install --reinstall .`，然后重启 Python 进程。
editable、wheel 和发布准备见 [安装指南](docs/python-installation.md)。以下 xmake 命令保留用于 C++ 核心开发。

Python 用于仿真层的配置、控制、步进与物理状态读取，底层运算不导出。当前支持被动平面单摆、无吊载无人机和绷紧绳索吊载耦合。Ubuntu 配置方式：

```bash
sudo apt install python3-dev python3-pybind11
xmake f -y -m debug --viewer=n --python=y --python_executable=/usr/bin/python3
xmake build
xmake test -v
xmake run csim_python examples/python/pendulum.py
xmake run csim_python examples/python/drone.py
xmake run csim_python examples/python/suspended_payload.py
```

使用 `PendulumModel`、`DroneModel` 或 `SuspendedPayloadModel` 和 `make_data` 创建实验，`step(model, data)` 推进一个物理步，`get_state(model, data)` 读取独立快照，`reset` 设置初始状态。无人机通过 `set_control(model, data, thrust, torque_B)` 设置总推力与机体系力矩。
严格绷紧吊载模型创建和重置必须显式传入 `thrust`，只接受正张力状态；悬停输入为 `(drone_mass+payload_mass)*gravity`。需要松弛/收紧事件时构造 `SuspendedPayloadModel(cable_mode="hybrid")`，接口见 [绷紧/松弛文档](docs/slack-taut.md)。

需要直接编写吊载跟踪循环时，运行 `examples/python/payload_tracking.py --speed 1.0`；模型、可调速度轨迹、绳索倾斜前馈、载荷位置反馈和状态读取均在该文件中。默认参考峰值速度为 1 m/s，使用 `--headless` 可无窗口计算。操作、速度限制与字段说明见 [Python 吊载跟踪](docs/python-payload-tracking.md)。
`xmake run csim_python your_script.py` 自动配置模块搜索路径。
单摆限定绳索始终绷紧的低能量平面摆动；无人机物理核心支持可选空气阻力；控制内环和执行器响应位于 Python 层，暂不含地面碰撞。虚拟环境和接口说明见 [Python 开发指南](docs/python.md)。

## 可视化入口

viewer 默认不参与构建。启用时需要 pkg-config、GLFW 3.3 或更高版本和 OpenGL 开发库，
运行时需要图形显示环境和支持 OpenGL 3.3 Core 的驱动。
Ubuntu 可安装开发依赖：

```bash
sudo apt install pkg-config libglfw3-dev libgl1-mesa-dev
```

```bash
xmake f -m debug --viewer=y
xmake build -j 2
xmake run viewer
```

viewer 默认实时显示无人机、绳索和吊载，带地面网格、坐标轴和轨迹，不绘制屏幕文字或装饰面板。
鼠标左拖旋转、右拖平移、滚轮缩放；Space 暂停、N 单步、R 重置、F 跟随、C 恢复相机、T 切换轨迹、Esc 退出。
物理步长与显示帧率分开。Python 可先录制再回放：

```bash
xmake run csim_python examples/python/record_payload.py build/flight.csv
xmake run viewer build/flight.csv
```

录制同时保存 JSON 参数与代码版本元数据，已有记录不会被覆盖。
着色器从可执行文件旁的 `shaders/` 加载，GL 函数通过 GLFW 加载，支持离屏帧缓冲截图。
完整操作、CSV 格式、时钟和显示限制见 [可视化文档](docs/visualization.md)。
后续 PX4/APM 软件/硬件在环的协议、传感器、执行器和时间同步要求见 [飞控接入设计](docs/flight-controller-integration.md)，目前尚未实现飞控桥接。
依赖通过 pkg-config 查找本机安装的库，不自动下载 GLFW。
构建脚本使用 `network.mode = private` 跳过远程仓库更新，已有依赖时可离线构建。
以后若改用 xmake 下载第三方包，需要移除这项策略。
关闭 viewer 使用 `xmake f --viewer=n`，此时无需图形开发依赖。

## 持续集成

新增 [Python 打包 CI](.github/workflows/python-package.yml)：使用 uv 构建源码包与 wheel，
在 Python 3.10–3.13 独立环境安装并验证物理接口和 editable 安装，保存原生 Linux 构建产物；不自动发布 PyPI。

[xmake CI](.github/workflows/ci.yml) 在推送和 Pull Request 时自动运行，
也可从 GitHub Actions 页面手动触发。工作流在 Ubuntu 24.04 上分别配置、
编译 debug 和 release：每个模式验证纯 C++ 构建，以及同时开启 viewer 与 Python 的构建，共四个任务。
运行 `simulator`、`simulator drone` 和 `simulator payload` 检查程序能否正常退出，并通过 `xmake test -v` 执行 C++ 数学、积分、单摆、无人机及吊载耦合测试，以及已启用的 Python 仿真接口测试；同时运行三个 Python 物理示例。viewer 组合还会在 Xvfb/Mesa 下验证实时和 Python 轨迹回放截图，无需物理显示器。

测试覆盖向量、矩阵、LU 分解与残差、旋转和姿态运动学、RK4/Dormand–Prince 阶段与收敛、自适应拒步及时间边界、极端数值和无效输入，单摆周期、能量及约束，无人机自由落体、悬停、力矩响应与转动守恒，吊载三维受力、绳约束、解析自由转动和输入功平衡，以及 Python 控制、物理状态、所有权和失败恢复；测试检查在 release 下仍有效。
工作流语法和源码检出配置参考 [GitHub Actions 文档](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax)
与 [xmake 安装 Action 文档](https://github.com/xmake-io/github-action-setup-xmake)。

## 模块约定

- `include/csim/` 存放公共头文件，使用 `#include <csim/math/vector.hpp>` 形式引用。
- `src/` 存放实现与内部头文件；模板实现通常放在公共头文件中。
- `math` 与 `numerics` 组成 `csim_numerics` 基础库，不依赖具体物理模型。
- `csim_dynamics` 依赖 `csim_numerics`；`csim_simulation` 依赖 `csim_dynamics`。
- `csim_model` 依赖 `csim_numerics`，提供描述数据；`csim_io` 链接 TinyXML2，实现 XML/网格读取与固定刚体合并。
- 控制器按职责归入动力学或仿真目标；需要独立复用时再建立单独目标。
- 可视化读取仿真状态，仿真核心不依赖窗口或绘图库。
- `bindings/python/` 中的原生绑定提供物理接口，`csim_experiments` 是随安装包分发的 Python 实验层；`examples/python/` 提供脚本入口。不导出底层数学与积分接口。
- 四个基础/描述/仿真库使用 `headeronly`；包含非模板实现的 `csim_io` 使用 `static`。
- 数学算法需验证数值误差；物理模型需记录坐标系、单位和建模假设。
