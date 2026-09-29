# 三维可视化与轨迹回放

当前 viewer 已显示无人机、绷紧绳索、质点吊载、地面网格、世界坐标轴和运动轨迹。
支持内嵌实时算例与 Python CSV 轨迹回放。窗口和绘制使用 GLFW/OpenGL，物理计算继续由已有 C++ 仿真接口完成。

## 构建与启动

Ubuntu 需要 `pkg-config libglfw3-dev libgl1-mesa-dev`，Python 录制另需 `python3-dev python3-pybind11`。
viewer 仍是默认关闭的可选目标；关闭后不查找 GLFW/OpenGL，也不影响命令行或 Python 仿真。

```bash
xmake f -y -m debug --viewer=y --python=y --python_executable=/usr/bin/python3
xmake build
xmake run viewer
```

默认运行与 `simulator payload` 相同的三维无阻尼吊载初值：无人机 2 kg、吊载 0.5 kg、绳长 1.2 m、重力 9.81 m/s²、步长 0.002 s、总推力 24.525 N。
初始无人机高度 5 m，绳方向为归一化的 `(0.3,0.2,-1)`，并具有切向初速度。
没有位置保持或消摆控制器，无人机会随吊载反作用力移动。

使用 `xmake run` 选择当前模式的程序。直接运行时，本机 Debug 路径为 `build/linux/x86_64/debug/viewer`；迁移构建系统后，旧 CMake 可执行文件不会自动更新。
程序从可执行文件旁的 `shaders/` 加载着色器，分发程序时需要保留该目录。Linux 可从其他工作目录启动程序。

## 操作

| 操作 | 功能 |
| --- | --- |
| 鼠标左键拖动 | 绕观察目标旋转 |
| 鼠标右键拖动 | 平移相机目标，退出跟随 |
| 滚轮 | 缩放观察距离 |
| Space | 暂停/继续 |
| N 或右方向键 | 暂停并推进一个物理步，回放时前进一条记录 |
| R | 重置算例或回到首条记录，清空轨迹；保留暂停状态和播放倍率 |
| `+ / -` | 播放倍率在 0.125× 到 4× 之间变化；主键盘 `=` 也可加速 |
| F | 开启/关闭跟随无人机与吊载的中点 |
| C | 恢复观察当前物体的默认相机，退出跟随 |
| T | 显示/隐藏轨迹 |
| Esc 或关闭窗口 | 退出当前 viewer |

窗口只绘制三维场景，不叠加状态面板、操作提示、图例或装饰条。物理时间和暂停状态显示在独立 viewer 的窗口标题中；Python 中通过状态接口读取物理量。
青色对应无人机，金色对应吊载与绳索；红色机头指向机体 +X。
世界坐标轴 X/Y/Z 分别为红/绿/蓝，地面网格间距为 1 m，显示系保持 Z 向上。
回放到末尾停留在最后一条记录，R 可重新开始。

机架、起落架和旋翼是示意几何，没有碰撞体或空气动力学含义；圆盘与桨叶标记不表示已实现电机转速仿真。
吊载球体半径 0.13 m 仅为显示大小，物理模型仍是质点。绳线连接物理质心与吊载中心，厚度只是便于观察。
地面网格不是碰撞面；穿过地面也不会自动施加接触力。

## Python 录制与回放

```bash
xmake run csim_python examples/python/record_payload.py build/flight.csv
xmake run viewer build/flight.csv
```

录制脚本默认仿真 10 s，每 5 个 0.002 s 物理步记录一次，即 100 Hz，共 1001 个样本。
还可传入 `--duration 20 --timestep 0.002 --sample-steps 10`。时长须为物理步长的整数倍；即使最后一步不是采样间隔的整数倍，也会保存终点。
输出路径不存在时自动创建父目录；已有 CSV 或同名元数据不会被覆盖，重新录制时选择新文件名。

旁边的 `flight.csv.json` 保存模型参数、初值、控制、积分方法、步长、采样间隔、坐标约定、代码提交/工作区是否有修改及完成状态。
`dirty=true` 时仍需另行保留对应源代码才能完整重现；无法读取 Git 时版本字段为 null。
若中途失败，脚本传播错误，可能保留有效的 CSV 前缀，但不会写成功元数据。
viewer 只读取 CSV，不依赖旁边的 JSON；完整实验归档应同时保留两者。

CSV v1 严格列名及顺序为：

```text
time,drone_x,drone_y,drone_z,q_w,q_x,q_y,q_z,payload_x,payload_y,payload_z,tension
```

位置为世界系米，时间为秒，张力为牛顿；姿态是 Hamilton wxyz、B→W，机体系 FLU。
这是显示轨迹格式，不是用于恢复积分状态的存档，未包含全部速度、控制器或执行器状态。
Python 仍只使用 `make_data / step / get_state` 等物理 API；无需导入 OpenGL，也没有新增底层数学或积分绑定。

读取时检查：

- 列数、有限数值、严格递增的非负时间、非零姿态四元数、正张力；姿态在显示副本中归一化。
- 每帧绳长相同，容差 `1e-7*max(1,l)`；不接受用不同绳长拼接的文件。
- 最多一百万条记录；显示位置距原点不超过 10 km，绳长范围 0.0001–1000 m。这是 viewer 的显示/资源范围，不改变动力学参数范围。
- 文件错误报告行号，并在创建窗口前终止。

回放按记录的相对时间推进，起始时间可以非零、采样间隔可以不均匀。
画面保持最近一条已到时的记录，不在两个位置间线性插值，从而不会凭空画出违反绳长约束的中间状态。
低采样率或慢速回放可看到离散步进，若需要更细画面，应增加记录频率。
单条记录也可显示。轨迹显示最多保留 1500 个历史点，长文件的记录本身仍可回放。

## 时间与错误处理

实时演示的每个物理步始终为 0.002 s。墙钟经过时间只决定本次绘制之前推进几步，播放倍率不改变物理步长。
为避免窗口卡死，一帧最多处理 0.25 s 的墙钟增量、最多推进 128 步；超出预算会丢弃积压的墙钟时间并提示播放减慢，不增大积分步长追赶。
实时轨迹按物理时间约 50 Hz 保存，与显示帧率无关。

暂停、单步和改变播放倍率会清理时间积压。窗口最小化时暂停内嵌演示推进，恢复时不追赶最小化期间的时间。
物理异常（例如非正张力）保留最后有效状态、暂停演示，终端输出具体原因；R 可重置。
OpenGL 初始化、着色器或文件错误则以非零退出码终止。

当前窗口中的暂停只控制本地演示或文件回放。未来 PX4/APM 后端必须独立运行，暂停显示、关闭窗口不能隐式暂停硬件或协议调度。
详细边界和前置条件见 [飞控在环扩展设计](flight-controller-integration.md)。

## 渲染实现与验证

`camera.cpp` 用本项目 `Matrix<4,4>` 计算视图及透视投影；GPU 上传时显式转换为列主序 float。
`gl_api.hpp` 在上下文建立后使用 `glfwGetProcAddress` 加载实际使用的 OpenGL 3.3 函数。
`renderer.cpp` 管理着色器、VAO/VBO、独立颜色/深度帧缓冲，构建机架、球体、绳索、网格及轨迹。
同一个离屏帧缓冲既用于显示，也用于截图，窗口尺寸变化时重建；GL 资源在销毁上下文之前释放。
参考 [GLFW 上下文与函数加载](https://www.glfw.org/docs/3.3/context_guide.html)、[窗口与帧缓冲尺寸](https://www.glfw.org/docs/3.3/window_guide.html)。

`viewer_test/default` 不链接图形库，包含四组检查：相机基与裁剪/矩阵排列、CSV 校验、回放操作、不同绘制节奏下的物理轨迹一致性。
其中以 100 Hz 和 40 Hz 显示节奏推进同一算例，验证其在相同物理时间与直接执行 1000 个物理步得到一致状态。
另有两项 Python 录制测试，检查物理约束、元数据、终点样本及输出保护。

可进行有限帧截图验证：

```bash
xmake run viewer --hidden --frames 361 --screenshot build/viewer-live.ppm
python3 tests/viewer/check_image.py build/viewer-live.ppm
xmake run viewer build/flight.csv --hidden --frames 361 --screenshot build/viewer-replay.ppm
```

`--frames N` 用确定性的 1/60 s 显示时间增量运行 N 帧，首帧在初值，361 帧对应 6 s（无暂停、倍率为 1 时）。
`--screenshot` 把最后一帧写成 P6 PPM；`--hidden` 和截图选项都要求 `--frames`，防止隐藏程序无限运行。
隐藏窗口仍需要可用的图形上下文；CI 使用 Xvfb 和 Mesa 软件渲染。图片检查采样中央场景区域，确认场景中实际存在青色无人机与金色吊载几何，不使用跨显卡脆弱的逐像素哈希。

本机 Linux/NVIDIA 已验证实时与 Python 回放截图、暂停/继续/单步/重置、窗口缩放和退出；鼠标相机计算另有无窗口测试。
Debug/Release 测试由 xmake 运行，GitHub CI 同步加入两种渲染路径的有限帧验证。macOS/Windows 图形运行尚未在本机验证。

[阶段 6](development/06-visualization.md) · [Python 接口](python.md) · [吊载模型](suspended-payload.md)

## Python 实时窗口与模型入口

CLI 继续提供原有吊载演示与 CSV 回放。共享 Renderer 已支持无吊载/吊载快照，
Python 通过 [Viewer](python-viewer.md) 控制窗口、同步和关闭，窗口不会调用 step。
[load_model](model-import.md) 已支持固定 URDF/MJCF 子集、基本几何及 OBJ/STL，导入机架可直接实时显示。
文件加载入口位于 Python，不增加 CLI 模型加载选项。
