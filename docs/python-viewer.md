# Python 实时可视化

已实现 `csim_viewer.Viewer`，支持无吊载和绷紧绳索吊载，以及 `load_model` 导入的机架几何。
`csim` 管理物理，`csim_viewer` 管理窗口；显示不会推进物理或修改控制。

## 最小循环

```python
import time
import csim
from csim_viewer import Viewer

model = csim.DroneModel(timestep=0.002)
data = csim.make_data(model, position_W=[0, 0, 5])
csim.set_control(model, data, thrust=model.mass * model.gravity)

with Viewer(model) as viewer:
    start = time.monotonic()
    viewer.sync(data)
    for index in range(5000):
        if not viewer.is_running():
            break
        csim.step(model, data)
        if (index + 1) % 10 == 0:
            viewer.sync(data)
            time.sleep(max(0, start + (index + 1)*model.timestep - time.monotonic()))
```

物理固定 500 Hz，显示 50 Hz。调整显示间隔不改变同一物理步数的结果；慢机器只会落后墙钟时间，不丢物理步。
`sync` 同步绘制并处理事件，不主动限速。实时节奏、暂停与实验终止由 Python 决定。
窗口关闭标志通过 `sync` 处理事件后更新，因此长计算应定期同步。

| 接口 | 行为 |
| --- | --- |
| `Viewer(model, width=1280, height=720, hidden=False)` | 创建窗口并持有模型；同一进程只允许一个窗口 |
| `sync(data)` | 校验模型所有权，读取快照，更新轨迹、处理事件和绘制 |
| `is_running()` | 窗口是否仍开启；不推进仿真 |
| `close()` | 幂等关闭；`with` 异常退出也释放资源 |
| `screenshot(path)` | 保存最近一次同步的画面为 P6 PPM，须先调用 sync |

创建、同步、查询和关闭均须在 Python 主线程。底层调用不释放 GIL，不允许并发写同一份 Data。
默认 GLFW/OpenGL 3.3 core；`hidden=True` 仍需要图形上下文，不是无 GPU 的离线渲染器。
无显示服务时可以 `import csim`、加载模型和执行物理仿真；只有创建 Viewer 才要求显示服务。

鼠标左键旋转、右键平移、滚轮缩放；F 跟随、C 恢复相机、T 轨迹、Esc 关闭。窗口只绘制三维场景，不叠加文字或装饰面板。
CLI 的暂停、重置和倍率键不用于 Python 窗口，以免隐式改变实验状态。
无人机模式省略绳索、吊载和张力；吊载模式读取真实吊载位置及张力。默认机架由程序生成，文件模型使用导入几何。

## 运行示例

```bash
uv pip install --python .venv/bin/python .
.venv/bin/python examples/python/live.py
.venv/bin/python examples/python/live.py --payload
.venv/bin/python examples/python/hover.py
.venv/bin/python examples/python/tracking.py --render-every 10
.venv/bin/python examples/python/payload_swing.py
.venv/bin/python examples/python/hover.py --model examples/models/drone.xml
```

三个控制示例支持 `--headless`，此时不创建窗口；`--hidden --screenshot frame.ppm` 可做自动截图。
可通过 `--output result.json` 比较显示频率不同但步数相同的最终物理状态。
安装依赖见 [安装指南](python-installation.md)，模型边界见 [模型导入](model-import.md)。

## 实现与验收

`Display` 只保存显示输入，`Renderer` 不再依赖内嵌吊载 `Scene`；CLI 保留自身演示/CSV 调度。
`Window` 管理 GLFW 生命周期，`_csim_viewer` 负责物理快照转换，Python `Viewer` 提供主线程检查和上下文管理。
着色器在 `bindings/python/csim_viewer/shaders/`，随 wheel 安装，CLI 构建时复制到程序旁。

`tests/viewer/python_viewer_test.py` 验证两种模型、关闭/异常/所有权、URDF 与 MJCF 画面一致性，
以及带 Python CTBR、延迟和执行器响应时，RK4 与自适应 Dormand–Prince 分别在不同显示频率下的最终快照逐字段完全一致。
运行：`.venv/bin/python tests/viewer/python_viewer_test.py`；Linux CI 使用 Xvfb。
当前不支持多窗口、场景编辑或异步显示线程。在环仿真仍需独立后端和快照通道，见 [飞控在环设计](flight-controller-integration.md)。
