# 日常开发流程

[返回开发总览](../development-workflow.md)

本流程适用于各阶段的日常实现、验证和提交。

## 开发循环

Python 实验使用已有虚拟环境，先 `uv pip install .`，再运行 `.venv/bin/python examples/python/drone.py`。
只改实验脚本可直接重跑；改 C++ 后执行 `uv pip install --reinstall .` 并重启解释器。
wheel、editable 与源码依赖见 [uv 安装指南](../python-installation.md)；以下 xmake 流程用于核心开发和测试。

1. 确定一个小功能，写明输入、输出、边界情况和验收条件。
2. 编写接口和对应的验证算例，再实现功能。
3. 更新 `xmake.lua`，注册新增源文件、测试目标和依赖。
4. 在 debug 下构建并运行相关测试，必要时调试数值结果。
5. 在 release 下验证，确认测试检查没有因优化或断言关闭而失效。
6. 仅为仿真层的参数、控制、步进和物理状态接口补充 pybind11 绑定及 Python 用例；底层数学与积分只在 C++ 测试。更新接口说明、模型说明、算例参数和开发状态。
7. 检查变更后提交代码，通过 Pull Request 查看 CI 结果。

添加非模板 `.cpp` 时，将相应库目标从 `headeronly` 改为 `static`，并使用 `add_files` 注册源文件。
第一批算法应同步建立测试目标并接入 CI；构建成功和 `simulator` 正常退出不能替代算法测试。

当前可使用的命令：

```bash
# 日常开发：C++ 核心 + Python 脚本（需安装 Python 开发头文件与 pybind11）
xmake f -m debug --viewer=n --python=y --python_executable=/usr/bin/python3
xmake build
xmake test -v
xmake run csim_python examples/python/pendulum.py
xmake run csim_python examples/python/drone.py
xmake run csim_python examples/python/suspended_payload.py
xmake run simulator
xmake run simulator drone
xmake run simulator payload

# 检查可视化，需要图形开发依赖和显示环境
xmake f -m debug --viewer=y
xmake build
xmake run viewer

# 提交前验证 Release；viewer 和 python 选项延续上一次配置
xmake f -m release
xmake build
xmake test -v
xmake run simulator
```

`xmake test -v` 自动构建测试，当前有 `vector_test/default`、`matrix_test/default`、`lu_test/default`、`rotation_test/default`、`rk4_test/default`、`dormand_prince_test/default`，分别包含五、六、七、八、七、八组检查；另有 `pendulum_test/default` 五组动力学检查和 `pendulum_simulation_test/default` 四组仿真检查。
无人机另有 `drone_test/default` 六组动力学检查和 `drone_simulation_test/default` 五组仿真检查，吊载另有 `suspended_payload_test/default` 与 `suspended_payload_simulation_test/default` 各五组，另有 `viewer_test/default` 四组无窗口相机/轨迹/播放检查，C++ 合计 75 组。
启用 Python 后增加 `csim_python/bindings`，执行 `tests/python/` 中 48 项 Python 测试（另含模型导入、控制响应与飞行示例）。
可用 `xmake test -v rotation_test/default` 或 `xmake test -v csim_python/bindings` 单独运行。
检查纯 C++ 构建时配置 `xmake f --viewer=n --python=n`，随后构建并测试；无需 Python 或图形依赖。
虚拟环境及自定义脚本命令见 [Python 开发指南](../python.md)。
新增测试目标时，应在本文及 [CI 工作流](../../.github/workflows/ci.yml) 中同步维护实际命令。
CI 在 debug/release 下分别验证纯 C++、同时开启 viewer/Python 的四种组合；单元测试无需图形显示环境；viewer 图像检查使用 Xvfb 和 Mesa。

可视化检查：`xmake run viewer`；录制后运行 `xmake run viewer build/flight.csv`。完整按键与有限帧截图命令见 [可视化说明](../visualization.md)。
