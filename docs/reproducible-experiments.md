# 可重复的 Python 实验

`uv pip install .` 同时安装 `csim`、`csim_viewer` 与纯 Python 的 `csim_control`、`csim_experiments`。
实验层调用物理接口，控制和记录都在 Python；不向 Python 导出积分器或矩阵分解。
源码中的 CLI 位于 `examples/python/`，安装后的库也可直接在任意脚本中使用。

## 单次实验与配置

```python
from csim_experiments import default_config, run_flight, replay

config = default_config("tracking", rotors=True, payload=True)
config["experiment"]["duration"] = 2.0
config["model"]["wind"]["velocity_W"] = [2, 0, 0]
config["model"]["drone_drag"]["k1"] = 0.1
config["model"]["payload_drag"].update(
    k1=0.02, k0=0.001, sign_mode="tanh", epsilon_v=0.02)
result = run_flight(config, "runs/tracking-wind", headless=True)
print(result["metrics"])
print(replay("runs/tracking-wind")["verified_states"])
```

目录必须不存在。`headless=False` 打开被动 Viewer，`render_every` 只控制同步频率；
固定物理步数、控制时钟与阵风时间均不依赖显示或墙钟。
`default_config` 只是可编辑的教学参数；`csim.get_config(model)` 可导出已有无人机/吊载的物理参数。

```bash
.venv/bin/python examples/python/tracking.py --rotors --duration 2 --output-dir runs/tracking
.venv/bin/python examples/python/payload_swing.py --headless --rotors --duration 2 --output-dir runs/swing
.venv/bin/python examples/python/replay_experiment.py runs/swing
```

也可用 `--config configuration.json` 输入仅含 model、control、initial、experiment 四节的配置。
记录的 config.json 还包含元数据；重跑时提取上述四节，或使用 `replay`。
实验配置包含质量、完整惯性、重力、步长、吊索长度、风与阻力、旋翼几何及限幅、内环增益、延迟/响应常数，
以及位置/速度/姿态外环增益、加速度限幅、角速度限幅、最小推力比例、抑摆增益、外环周期与初值。
参考轨迹由 task 指定，生成公式保存在包内 flight_control.py 并记录源码 SHA256；当前只提供 hover、tracking、swing。

duration、outer_period、非零 controller_period 与 command_delay 必须与物理步长满足整数步关系；
扫描时不会偷偷四舍五入改变控制周期。物理参数保持不可变，改变物理参数应新建模型；控制参数位于独立顶层 control，改变控制配置应新建 ControlLoop。

## 记录格式

每个目录使用 `csim-experiment-v2` 格式：

| 文件 | 内容 |
| --- | --- |
| config.json | 完整四节配置、初始化后的完整快照、CSim 版本、原生模块 SHA256、Python/平台信息、实验包和控制包各源码 SHA256 |
| commands.jsonl | 实际接受的每条 CTBR 指令、物理 tick 与提交时刻 |
| states.jsonl | tick=0 及每个成功步的完整 ControlLoop.get_state 快照和期望位置/速度/加速度 |
| result.json | 完成/关窗/失败状态、失败类型及阶段、最终快照、指标 |

状态包含无人机及吊载位置/速度/加速度、姿态/角速度、张力、能量、绳状态、全部控制快照、各旋翼推力、
分配残差与饱和信息、两端的阻尼/风作用力与功率。当前 ControlLoop.get_state 不导出队列内部每条待到达指令，
但完整 commands.jsonl 可从初始条件重建队列；这不是可在任意时间点直接恢复的 checkpoint 格式。

文件使用严格 JSON：配置中的无限限幅编码为字符串 `"+inf"`，读取时恢复；不输出非标准 Infinity/NaN。
完整状态要求数值有限。数据按步流式写入，正常退出或可捕获异常时关闭并保存结果；
尚无断电事务恢复机制。参数在构造阶段被拒绝时，由批量运行器保存 rejected_config.json 和 error.json。

文件模型记录合并后的质量、质心处惯性及初始位姿，可不依赖原 XML 重现物理。
当前不打包视觉网格或恢复 Viewer 视角；需要同样外观时另行保留原模型资源。

## 回放与指标

`replay(directory)` 从初值、参数和指令重建仿真，比较每个成功步的**完整状态**，默认要求精确一致。
吊索失去张力导致的失败步也会重试并检查失败类型与原子性。图形异常不要求重现窗口环境。
跨编译器/平台不保证逐位一致；可显式设置绝对 `tolerance`，结果同时报告原生二进制是否相同。
当前回放验证物理状态与最终步数；参考轨迹和汇总指标不是独立的签名或防篡改格式。

物理时段上的 RMSE/RMS 用相邻记录端点的梯形积分：位置误差、相对竖直向下的摆角、分配残差分别报告。
另保存最小/最大张力，实际推力平方积分 N²·s、各轴力矩平方积分 (N·m)²·s，以及饱和步比例。
分配残差按推力 N、各轴力矩 N·m 分开，不把不同单位混为一个“控制误差”。
推力平方积分是控制努力指标，不等于电池能耗。
为了兼容原飞行示例，还保留末四分之一时段的离散位置 RMSE 和摆角 RMS。

## 批量扫描

```bash
.venv/bin/python examples/python/sweep.py --output-dir runs/sweep \
  --duration 2 --timesteps 0.001 0.002 --payload-masses 0.2 0.4 \
  --delays 0 0.004 --wind-speeds 0 2
```

这会执行 16 个组合。每组保存完整记录，顶层保存 sweep.json、summary.json 和便于比较的 summary.csv。
`default_sweep_config()` 使用吊载轨迹跟踪、旋翼分配及显式的示例阻力系数；这些系数没有实物辨识依据。
风速扫描改变 W 的 +X 方向均匀风，保留原配置的空间梯度及阵风。
改变载荷质量时，初始实际推力明确重设为总重力；初始几何与其他参数保持配置值。
失败组独立记录并继续下一组，适用于步长、控制周期不相容等情况。
库入口为 `run_sweep(config, directory, timesteps=..., payload_masses=..., delays=..., wind_speeds=...)`。

## 旋翼响应辨识

```bash
# 从已知模拟模型生成阶跃，只把测量序列交给拟合器
.venv/bin/python examples/python/identify_response.py --output-dir runs/identify --noise-std 0.01 --seed 42
# 换成实验台测得的数据
.venv/bin/python examples/python/identify_response.py --input measured.csv --output-dir runs/fit
```

CSV 列名按顺序为 `time,command_0,command_1,command_2,command_3,thrust_0,thrust_1,thrust_2,thrust_3`，
时间单位 s，命令与测量均为 N。需要均匀采样、足够的稳态前段、四个通道各一个同时发生的非零阶跃和足够的后段；
不是 PWM/RPM 标定器，也不拟合多段指令或 CTBR 闭环响应。

拟合模型为 `f_i(t)=f0_i+gain_i*delta_command_i*(1-exp(-(t-t_step-delay)/tau_i))`，
到达时刻之前保持 f0_i。共享延迟在采样整数倍上搜索，每个通道拟合正时间常数和增益；
阶跃后前 70% 样本用于拟合，后 30% 单独给出验证 RMSE。报告搜索范围、样本数及延迟是否位于边界。
增益用于检验命令标定是否一致；当前 Python 响应模型的 N→N 执行器稳态增益为 1，不能把非 1 拟合增益直接写入 ControlConfig。
拟合 tau 与 delay 可用于 Python ControlConfig；真实设备还应检查激励幅值、噪声、非线性、时间戳偏差及不同工况。

保存 response.csv、完整合成配置或实测来源/哈希、identification.json。合成真值单独保存，不参与优化。
默认无噪声实验恢复 6 ms 延迟及 [15,25,40,60] ms 时间常数；回归测试还验证固定随机种子的噪声数据与留出误差。
这验证了辨识流程，没有证明某台真实无人机具有这些参数。

## 控制分层验收（2026-09-14）

C++ 只接收实际推力/力矩，控制链位于 Python `csim_control`。75 组 C++ 检查、62 项 Python 测试与 4 项图形测试通过；默认 20 s 悬停、跟踪和抑摆对照仍通过现有验收阈值。
新增验证包括：原生 API 无控制配置与控制器诊断、只用实际力历史精确重现物理、响应指数末值与推力冲量、位移随步长二阶收敛、外部改写检测。风扰/SPAD 的原物理基准保持逐位一致，旧控制器诊断不再属于原生快照。

## 历史验收记录（2026-09-10，迁移前）

以下数值来自旧版 C++ 控制链；新版 Python 使用每步平均实际力，旧控制实验不保证逐位相同。v1 记录需用旧版运行，不能直接改 schema 作为 v2。

已用 uv 构建源码包/wheel，并将 Release wheel 安装到项目原有 Python 3.11 虚拟环境。
75 组 C++ 检查、48 项 Python 测试和 4 项图形测试通过。默认 16 组、每组 2 s 的扫描全部完成。
以下是其中 timestep=0.002 s、payload_mass=0.2 kg 的结果；仅说明教学模型与外环在该短时实验中的表现：

| 延迟 s | 风速 m/s | 位置 RMSE m | 摆角 RMS rad | 最小张力 N |
| --- | --- | --- | --- | --- |
| 0 | 0 | 0.014864 | 0.036167 | 1.960682 |
| 0 | 2 | 0.056982 | 0.038337 | 1.961330 |
| 0.004 | 0 | 0.014866 | 0.036283 | 1.960652 |
| 0.004 | 2 | 0.056992 | 0.038457 | 1.961330 |

0.01 N 噪声、seed=42 的合成辨识得到共享延迟 0.006 s，时间常数约 [0.014978, 0.025050, 0.039978, 0.059873] s，
四通道留出 RMSE 为 0.00993–0.01107 N。2 s 风扰记录的 1001 个状态可精确回放。
测试通过不表示已经执行 GitHub 远程 CI，也不代表已完成真实设备参数辨识。
