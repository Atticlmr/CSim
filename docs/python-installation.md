# 使用 uv 安装与分发 CSim

## 当前目录的一键安装

已有 `.venv` 时，直接在项目根目录执行：

```bash
uv pip install .
.venv/bin/python examples/python/drone.py
```

uv 自动查找虚拟环境；如果当前 shell 激活了另一个环境，显式选择当前目录这一份：

```bash
uv pip install --python .venv/bin/python .
```

不需要先激活环境，也不需要手动设置 PYTHONPATH 或安装 pybind11 到运行环境。
uv 自身负责安装，不要求 `.venv` 内存在 pip。
安装的发行包名是 **`csim-drone`**，Python 导入名保持 **`csim`**。
这是本项目当前的包元数据名称，尚未在 PyPI 发布或确认该名称的注册归属。
不能把 `uv pip install csim` 当成本项目安装命令。

源码安装需要本机可用的 xmake ≥ 3.1.1、C++17 编译器和所选 Python 的开发头文件。
当前支持原生 Linux/macOS 构建，尚不支持 Windows、交叉编译或 universal2；本机实测 Linux。
Ubuntu 系统 Python 通常需要 `build-essential python3-dev`；uv 管理的 Python 应检查它自带的头文件。
按 [xmake 官方说明](https://xmake.io/guide/quick-start.html) 安装构建工具，或用
`CSIM_XMAKE=/绝对路径/xmake uv pip install .` 指定可执行文件。

安装过程中 uv 在隔离构建环境获取 setuptools、wheel 和 pybind11，
再调用 xmake 编译 Release 扩展并安装到 `.venv` 的 site-packages。
默认构建物理扩展和 Python viewer；源码编译需 TinyXML2、GLFW 和 OpenGL 开发库，无 MuJoCo/NumPy 运行依赖。
Ubuntu 可先执行 `sudo apt install build-essential pkg-config libtinyxml2-dev libglfw3-dev libgl1-mesa-dev`；
系统 Python 另需 python3-dev。macOS 源码构建需本机提供 TinyXML2/GLFW，尚未验证 macOS 安装产物。
只需无窗口模块时可使用 `CSIM_BUILD_VIEWER=0 uv pip install --reinstall .`，仍需 TinyXML2。
本机因包源 DNS 不可用，验证时使用了本地缓存的构建依赖；在线安装需要包源可访问。

## Python 实验流程

```python
import csim

model = csim.DroneModel(mass=1.0, timestep=0.002)
data = csim.make_data(model, position_W=[0, 0, 5])
csim.set_control(model, data, thrust=model.mass * model.gravity)

for _ in range(500):
    csim.step(model, data)

print(csim.get_state(model, data))
```

选择 `.venv/bin/python` 作为编辑器解释器，实验脚本可以放在其他目录，无需从 xmake 启动。
Jupyter 也必须使用这一环境的 kernel。
接口见 [Python 指南](python.md)、[实时窗口](python-viewer.md) 和 [模型加载](model-import.md)。
安装后可直接运行 `.venv/bin/python examples/python/tracking.py`。

## 修改代码之后

只改自己的 Python 实验脚本：保存后直接运行即可。
修改 C++ 核心、绑定或编译配置后，需要重新编译安装，并重启 Python/Jupyter kernel：

```bash
uv pip install --reinstall .
```

`pyproject.toml` 的 uv cache-keys 已包含 C++ 头文件、绑定源码和构建脚本，
避免只检查 Python 元数据而复用过期二进制。如果怀疑缓存，可使用 `uv pip install --reinstall --no-cache .`。

需要开发安装也可使用：

```bash
uv pip install -e .
```

当前主要代码是 C++，editable 不会在 import 时自动编译。
改 C++ 后仍需 `uv pip install --reinstall -e .`；生成的扩展放在 `bindings/python/`，不提交。
普通安装在 site-packages 中保留独立二进制；editable 环境依赖源码目录，不能移动或删除该目录。

## 生成可离线安装的 wheel

维护者在有构建工具的环境执行：

```bash
uv build --python .venv/bin/python
```

默认先生成源码包，再从源码包构建 wheel，输出到 `dist/`。
接收方拿到与 Python、系统和 CPU 架构匹配的 wheel 后执行：

```bash
uv pip install --no-index /路径/csim_drone-0.0.1-cp311-cp311-linux_x86_64.whl
```

文件名只是 Python 3.11/Linux x86_64 的例子，使用实际生成的文件名。
**安装匹配 wheel 不需要 xmake、C++ 编译器或 pybind11**；当前物理模块没有额外 Python 运行依赖。
原生 Linux wheel 仍依赖目标机器兼容的 glibc/libstdc++，不能把 `linux_x86_64` 标签误当作跨 Linux 发行版兼容承诺。

网络不可用时，源码构建也可从已准备好的本地依赖 wheel 目录取包：

```bash
uv pip install --offline --find-links /路径/build-deps .
```

该目录需要包含 pyproject.toml 声明的构建依赖及其传递依赖；它不是项目运行数据目录。

## 打包实现与 CI

| 文件 | 职责 |
| --- | --- |
| `pyproject.toml` | PEP 517 元数据、构建依赖、Python 最低版本和 uv 缓存输入 |
| `setup.py` | setuptools wheel/editable 适配，调用现有 xmake 目标 |
| `MANIFEST.in` | 源码包包含头文件、绑定、Lua 构建脚本、例子与测试 |
| `xmake.lua` | C++ 目标及唯一项目版本来源 |
| `.github/workflows/python-package.yml` | Python 3.10–3.13 的 Linux 源码包/wheel、独立安装、物理/导入/控制测试、Xvfb 图形及 editable 检查 |

打包在独立临时目录复制源码、隔离 xmake 配置并使用构建解释器，
不覆盖开发目录里的 `.xmake`，不读取已有 `build/` 二进制。
默认 wheel 安装 csim、_csim_viewer、csim_viewer 包及着色器。Linux wheel 同时携带实际链接的
TinyXML2/GLFW 共享库及许可证，运行时从包内 .libs 查找；仍使用系统 OpenGL 驱动和窗口系统库。
只 `import csim` 不加载 GLFW/OpenGL。macOS 暂使用系统安装的第三方动态库，未承诺可携带分发。
源码包额外包含开发源码、模型示例、说明和测试；控制示例位于源码 examples/，不作为 wheel 命令行工具安装。
Python 包 CI 独立于原有 C++/可视化 CI；新增工作流尚需推送后由 GitHub 执行，本机检查不代表远程矩阵已通过。

## 达到类似 MuJoCo 的公开安装体验还需要什么

1. 当前已打通本地 `uv pip install .` 和 wheel 分发。
2. 为目标 Python/平台构建经过验证的预编译 wheel；Linux 增加 manylinux/auditwheel 校验，macOS 做最低系统版本和架构验证，Windows 先补构建支持。
3. 确定公开发行名称、版本和发布渠道，配置 PyPI Trusted Publishing 或私有索引，再发布通过验证的 wheel。当前没有发布步骤或凭据。
4. 发布后用户才可以从任意目录用 `uv pip install <发行包名>` 安装，无需源码或本地编译工具。
5. 当前已将 Python viewer、模型加载、着色器和 Linux 第三方动态库纳入安装及图形测试；跨平台分发继续独立验收。

wheel 安装体验、Python API 和仿真功能分别验收；当前只提供单摆、无吊载无人机和绷紧绳索吊载模型。

参考：[uv 安装包](https://docs.astral.sh/uv/pip/packages/)、[uv 构建分发包](https://docs.astral.sh/uv/guides/package/)、
[uv 缓存](https://docs.astral.sh/uv/concepts/cache/)、[Python 二进制扩展分发](https://packaging.python.org/en/latest/guides/packaging-binary-extensions/)。

## 实验工具包

同一次 `uv pip install .` 会安装纯 Python 的 `csim_experiments`，不新增第三方运行时依赖。该包在 headless 构建中也可用，包含 run_flight、replay、run_sweep 和 identify；命令及记录格式见 [可重复实验](reproducible-experiments.md)。修改此包的 Python 源码后，普通安装需重新安装；editable 安装直接读取源码。修改 C++ 仍需重建并重启 Python。
