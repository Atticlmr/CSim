# CSim 开发流程

架构边界：C++ 仅实现物理模型、求解与状态接口，输入为实际推力/力矩；CTBR、控制分配、执行器响应、限幅、延迟和调度全部在 Python。新功能按此边界归属，详见 [控制迁移](control-and-response-models.md)。


本文约定自研数值基础库与无人机吊载仿真的开发顺序、交付内容和验收标准。
总体顺序为：明确模型 → 开发基础库 → 验证简单算例 → 实现耦合仿真 → Python 实验与可视化。
C++ 完成计算与仿真核心，pybind11 仅随仿真层接口扩展，后续主要通过 Python 脚本开展实验。

已接入多种积分方法和自适应内部步，配置与适用范围见 [积分器使用](integrators.md)。

## 当前状态

- 构建系统使用 xmake，语言标准为 C++17。
- [uv 安装与分发](python-installation.md) 提供本地路径安装、源码包/wheel 和 editable；C++ 继续由 xmake 构建，Python 包 CI 单独验证安装产物，尚未发布 PyPI。
- `csim_numerics`、`csim_dynamics`、`csim_simulation`、`csim_model` 为 `headeronly` 目标；`csim_io` 为链接 TinyXML2 的静态库。
- [第一版建模约定](modeling-conventions.md) 已记录局部 Z-up 惯性系、FLU 机体系、姿态与绳索方向的定义。
- [Vector3](vector3.md)、[Matrix](matrix.md)、[LU](lu.md) 和 [姿态接口](rotation.md) 已实现向量、固定大小矩阵、线性求解、旋转与四元数；[RK4](rk4.md) 已完成通用固定步长推进、解析解验证及 C++ 数值算例，[Dormand–Prince](dormand-prince.md) 已提供自适应推进、拒步及误差度量，[固定悬点单摆](pendulum.md)、[无吊载无人机](drone.md) 与 [绷紧绳索吊载耦合](suspended-payload.md) 已实现。
- [数值积分调研](numerical-integration.md) 已记录候选算法、适用条件及接口 TODO；预留声明不代表算法已可调用。
- [Python 仿真接口](python.md) 已提供单摆、无人机及吊载耦合模型/数据、步进、重置及物理状态快照，无人机增加总推力与机体系力矩输入，可通过 `--python=y` 启用；不导出底层运算。
- `simulator` 运行 20 秒被动单摆示例，`simulator drone` 运行悬停及偏航力矩算例；`simulator payload` 运行三维耦合算例；三个模型均有 Python 示例。
- [OpenGL/GLFW 可视化](visualization.md) 已实现三维吊载场景、相机交互、暂停/单步、实时演示及 Python CSV 录制/回放；截图进入 CI。
- [URDF/MJCF 导入](model-import.md) 已实现固定树、显式惯性、基本几何、OBJ/STL、独立物理适配与 Python load_model。
- [Python viewer](python-viewer.md) 已支持两种模型、文件几何、被动同步和窗口生命周期；改变显示频率不改变物理。
- [PX4/APM 在环扩展设计](flight-controller-integration.md) 已记录协议适配、传感器、执行器和时钟边界，目前尚无飞控连接实现。
- [Zhu 等（2025）吊载模型与 SPAD 阻尼解析](zhu-2025-payload-model.md) 已加入建模设计；[CTBR、单旋翼推力及响应延迟](control-and-response-models.md) 已在独立 Python 层实现，提供 Python 悬停、跟踪及抑摆对照示例；SPAD/风场和受约束旋翼分配已实现，提供完整记录、回放、扫描和响应辨识，见 [可重复实验](reproducible-experiments.md)。
- `vector_test`、`matrix_test`、`lu_test`、`rotation_test`、`rk4_test`、`dormand_prince_test`、`pendulum_test`、`pendulum_simulation_test`、`drone_test`、`drone_simulation_test`、`suspended_payload_test`、`suspended_payload_simulation_test`、`viewer_test` 与可选的 Python 仿真接口测试已接入 `xmake test`；CI 在 debug/release 下验证纯 C++ 和 viewer/Python 同时开启的四种组合。

阶段 1 的初版约定、阶段 2 的最小数学基础库和阶段 3 的 RK4 与 Dormand–Prince，以及阶段 4 的单摆、无吊载无人机及无阻尼耦合里程碑已落地；阶段 5 的显示轨迹录制、阶段 6 的单机最小可视化也已完成。

## 模块边界

| 模块 | 主要目录 | 职责 |
| --- | --- | --- |
| 数学基础 | `include/csim/math/`、`src/math/` | 向量、矩阵、线性求解和旋转表示 |
| 数值算法 | `include/csim/numerics/`、`src/numerics/` | 数值积分、误差估计等通用算法 |
| 模型描述与导入 | `include/csim/model/`、`include/csim/io/`、`src/io/` | TinyXML2 格式解析、网格资源、固定部件合并及独立物理适配 |
| 物理模型 | `include/csim/dynamics/`、`src/dynamics/` | 无人机、绳索、吊载及耦合动力学 |
| 控制器与执行器 | `bindings/python/csim_control/` | Python 内环、分配、响应与延迟；外环位于 csim_experiments/；C++ 只接收实际力 |
| 仿真管理 | `include/csim/simulation/`、`src/simulation/` | 时间推进、状态管理、数据记录 |
| 应用入口 | `apps/simulator/`、`apps/viewer/` | 命令行运行、交互与图形显示 |
| Python 绑定 | `bindings/python/` | 仿真参数、控制、步进和物理状态的跨语言接口 |
| Python 实验 | `bindings/python/csim_experiments/`、`examples/python/` | 场景配置、批量实验和后续结果分析 |

依赖方向为应用 → 仿真与动力学 → 数学和数值基础库。
Python 脚本 → pybind11 → 仿真核心，计算循环在 C++ 内执行。
基础库不依赖无人机或吊载概念，仿真核心不调用 Python 或 OpenGL/GLFW。
公共接口放在 `include/csim/`，非模板实现和内部头文件放在 `src/`；模板实现通常随头文件提供。

## 分阶段文档

各阶段分别维护目标、前置条件、实施内容和验收标准。完成验收后，再更新对应文档与总览中的开发状态。

| 阶段 | 文档 | 核心交付 |
| --- | --- | --- |
| 1 | [明确建模约定](development/01-modeling.md) | 模型约定和状态定义 |
| 2 | [开发最小数学基础库](development/02-math-library.md) | 数学接口、实现与独立测试 |
| 3 | [开发数值积分模块](development/03-numerical-integration.md) | RK4、解析解对照和收敛记录 |
| 4 | [逐个实现物理模型](development/04-dynamics.md) | 独立与耦合模型及验证算例 |
| 5 | [建立可重复的仿真实验](development/05-simulation-experiments.md) | C++ 仿真循环、Python 实验接口、轨迹与实验记录 |
| 6 | [接入 OpenGL 与 GLFW](development/06-visualization.md) | 状态显示、相机交互与播放控制 |
| 7 | [逐步增加复杂度](development/07-model-extensions.md) | 新增物理效应、适用范围与回归测试 |

共用的构建命令、测试接入和提交步骤见 [日常开发流程](development/daily-workflow.md)。

## 当前优先事项

阶段 2 的向量、矩阵、LU 求解、旋转矩阵和四元数已完成，均保留为 C++ 内部计算接口。
固定步长 RK4 与 Dormand–Prince 5(4) 自适应推进已完成。单摆、无吊载无人机、绷紧绳索质点吊载及其 Python 步进/控制/状态接口已通过验证。耦合基准已检查张力、作用力与反作用力、绳长及速度约束、解析轨迹四阶收敛和外力功率平衡。
本轮已完成默认关闭的 SPAD、相对空气速度阻力与风场；纯 Python CTBR→受约束分配→单旋翼响应链；完整实验记录、回放、四参数扫描及阶跃辨识工具。
绷紧/松弛混合模式和步末重新绷紧冲量已完成，下一步优先做连续时间事件根定位、地面接触和弹性绳，再用实测旋翼与吊载数据辨识参数；辨识脚本目前通过合成数据验收。

可视化已可用于观察后续物理扩展。飞控在环支线先补执行器、传感器、坐标适配和地面/松绳流程，再进入协议回环与 SITL；HITL 在选定硬件并完成软件闭环验收后推进。

用户提出的三个近期里程碑已交付：Python 实时窗口 → TinyXML2/URDF/MJCF 子集与网格、质量惯性适配 → CTBR/单旋翼/执行器/延迟和 Python 控制实验。
新增的 SPAD/风扰、完整旋翼链、可重复实验和单绳松紧切换已交付；通用关节、碰撞、多绳联合冲量及在环仍是独立里程碑。
本地安装已扩展为含 viewer/着色器的 wheel，公开发布及跨平台预编译包仍待验收。

后续绳索扩展的文献与建模路线见 [绷紧—松弛与收紧冲击调研](slack-taut-impact-research.md)，区分事件冲量、互补约束与有限刚度响应。
单机及多机动力学、冲击推导和实验边界见 [RotorTM 详细解析](rotortm-analysis.md)；该文档仍用于指导连续事件、多绳联合求解和实机验证。
