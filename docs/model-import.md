# URDF / MJCF 模型导入

已通过 TinyXML2 实现固定连接树、基本几何、显式惯性、无纹理 OBJ/STL 网格，以及 Python 物理模型加载。
导入后所有固定部件合并为一个自由刚体；当前不是任意多刚体、接触或通用 MuJoCo 模型执行器。
合并时保留每个源 link/body 的名称、父节点、坐标系和质心偏置，可查询各部件的运动状态。

## Python 入口

```python
import csim

model = csim.load_model("examples/models/drone.urdf", free_base=True, timestep=0.002)
data = csim.make_data(model, position_W=[0, 0, 5])
csim.set_control(model, data, model.mass * model.gravity)
csim.step(model, data)
print(model.mass, model.inertia_B, csim.get_state(model, data))
```

`load_model(path, *, format="auto", free_base=False, root_to_body=[1,0,0,0],
package_roots={}, gravity=9.80665, timestep=0.001, payload_mass=None, length=1.0)` 返回 `DroneModel`；提供 `payload_mass` 则返回 `SuspendedPayloadModel`。
吊载只是在整机质心挂接的质点/定长绳索，不能从文件中推断吊点或任意关节约束。
带吊载的 `make_data/reset` 仍须显式初始 `thrust`，验证正张力。

`format="auto"` 将 `.urdf` 作为 URDF，其他扩展作为 MJCF；XML 根元素必须匹配。
其他命名的 URDF 应显式 `format="urdf"`。参数路径为字符串，`Path` 可用 `str(path)` 转换。
URDF 不决定根的运动性，MJCF 没有 freejoint 的根固定；必须显式 `free_base=True` 才转换为无人机。
MJCF 根 `<freejoint/>` 可直接加载。

`make_data/reset` 的 `position_W` 与 `q_WB` 省略或为 None 时，使用文件初始位置/姿态转换后的整机质心状态；
显式传入则覆盖相应项。默认参数模型仍从原点、单位姿态开始。
文件模型可以直接传入同一个 [Viewer](python-viewer.md)。`examples/models/` 提供本项目原创的 URDF/MJCF 等价机架和 OBJ 网格。

## 固定 link 状态查询

URDF 的 `link` 和 MJCF 的 `body` 在以下接口中统一称为 link。查询不会推进仿真，也不会新增关节自由度。

```python
import csim

model = csim.load_model("tests/fixtures/model_import/fixed_frame.urdf", free_base=True)
data = csim.make_data(model, position_W=[0, 0, 5])
csim.set_control(model, data, model.mass * model.gravity)
csim.step(model, data)

print(model.link_names)  # ['base', 'arm']，返回独立的列表
arm = csim.get_link_state(model, data, "arm")
print(arm['position_W'], arm['q_WL'])
print(arm['velocity_W'], arm['acceleration_W'])
print(arm['com_position_W'])  # arm 自身质心，区别于 link 原点和整机质心

links = csim.get_link_states(model, data)  # {名称: 状态字典}，一次计算全部 link
```

两种状态接口返回独立快照；修改返回的列表/字典不会改变仿真。调用 `step()`、`reset()` 或修改已施加输入后，应重新查询。
批量查询复用一次整机物理观测，避免逐个 link 重复计算动力学。名称按源描述中的顺序保留，不保证父节点总在子节点前面。

| 字段 | 含义 |
| --- | --- |
| `name` / `parent` | 源 link 名称 / 父 link 名称；根的 parent 为 None |
| `time` | 当前仿真时刻，s |
| `position_W` | 源 link 坐标系原点的世界系位置，m |
| `q_WL` | 从 link 系 L 到世界系 W 的 Hamilton 四元数，wxyz |
| `velocity_W` / `acceleration_W` | link 原点的世界系线速度 m/s / 运动学加速度 m/s²（不是 IMU 比力） |
| `angular_velocity_W` / `angular_acceleration_W` | 世界系角速度 rad/s / 角加速度 rad/s² |
| `com_position_W` | 该 link 自身质心的世界系位置，m |
| `com_velocity_W` / `com_acceleration_W` | 该 link 质心的世界系线速度 m/s / 加速度 m/s² |

`q_WL` 对应 link 坐标轴，不是 visual 或 inertial 的坐标轴。`com_*` 取惯性原点的位置；惯性坐标轴的旋转不会改变质心位置。
现有 `get_state()` 的 `position_W` 仍指整机质心，`angular_velocity_B` 仍在整机机体系；与这里的世界系角速度不要混用。

可直接运行的入口为 [examples/python/link_states.py](../examples/python/link_states.py)：

```bash
.venv/bin/python examples/python/link_states.py
.venv/bin/python examples/python/link_states.py tests/fixtures/model_import/fixed_frame.xml
.venv/bin/python examples/python/link_states.py tests/fixtures/model_import/fixed_frame.urdf --payload-mass 0.4
```

默认的 `examples/models/drone.urdf` 只有一个 `frame` link，机臂和旋翼是该 link 下的 visual，因此 `link_names` 只有 `frame`。需要多个名称可查询的部件，应在模型中定义实际的 fixed link/body，并按当前导入要求提供显式惯性；visual 名称不是 link 名称。

直接通过 `DroneModel()` / `SuspendedPayloadModel()` 创建的参数模型没有导入的 link，`link_names` 返回 `[]`，批量查询返回 `{}`。
未知名称抛出 `KeyError`；同类型但不属于该 model 的 data 抛出 `ValueError`，不同类型的 model/data 组合不匹配绑定签名。
通过 `payload_mass` 添加的吊载质点仍使用 `get_state()['payload_*']` 查询，不自动插入 link 列表；它没有可定义的刚体姿态。
实验框架的 `get_config()` 保存物理参数，不保存源 link 树；从这些参数重建的模型没有 link，需要重新加载源文件才能恢复命名坐标系。

计算时令 B 为整机质心坐标系，L 为某个固定 link 坐标系。合并阶段保留 `T_BL`，并正确应用整体质心平移与 `root_to_body` 旋转。
运行时，对任一固定点取 `r_W = R_WB * r_B`：

```text
p_WL = p_WB + r_W
R_WL = R_WB * R_BL
v_WL = v_WB + omega_W × r_W
a_WL = a_WB + alpha_W × r_W + omega_W × (omega_W × r_W)
```

link 质心使用 `r_B = p_BL + R_BL * center_L` 代入同一公式。所有固定 link 的世界系角速度、角加速度相同，但不同原点的线速度与加速度一般不同。
吊载模型使用包含绳作用力的整机加速度；空气力与 Python 提交的实际推力/力矩也通过当前整机物理观测进入计算。
这些都是派生观测量，无需为每个 link 独立积分。可动关节、多刚体动力学和 link 约束反力查询仍未实现。

## 支持范围

| 格式 | 已支持 | 明确拒绝/尚未支持 |
| --- | --- | --- |
| URDF | 单根 link 树、fixed joint、origin xyz/rpy、显式 mass 和六项 inertia；visual/collision 的 box/sphere/cylinder/mesh；局部颜色和全局命名颜色引用 | 活动关节、transmission、Gazebo/xacro、自动惯性、纹理、未知元素/属性 |
| MJCF 子集 | compiler、单个 worldbody 根 body、固定子 body、根 freejoint；显式 inertial；基本 geom；asset mesh | defaults/class、include/attach、joint、tendon、actuator、sensor、option、接触、自动惯性、fromto、primitive mesh fitting |
| 网格 | 本地/显式 package 映射、正的非均匀 scale、无纹理 OBJ、ASCII/二进制 STL | DAE/glTF、MTL/纹理、透明材质、负 scale、退化三角形、非平面/凹 OBJ 多边形 |

MJCF 按 [MuJoCo 3.3.1 XML 参考](https://mujoco.readthedocs.io/en/3.3.1/XMLreference.html) 的明确子集解析：
必须写 `<compiler inertiafromgeom="false"/>`；每个 body 有显式惯性，geom 必须 `contype="0" conaffinity="0"`。
支持 `angle="degree|radian"`（默认 degree）和 `eulerseq`（默认 xyz，大小写分别为内禀/外禀旋转）。
姿态只接受 quat 或 euler 之一；惯性接受 diaginertia 或 fullinertia 之一。
fullinertia 顺序为 xx, yy, zz, xy, xz, yz，表达在 body 轴，不允许再指定惯性姿态。
box 的 size 为半边长；cylinder 第二个 size 为半长。中间描述统一转换成全尺寸，圆柱局部 Z 为轴。
mesh 不做惯性推断或再居中，源网格 scale 与 geom 位姿组合后直接显示；refpos/refquat 尚不支持。

URDF 的 collision 只校验几何；构建可运行模型时给出无接触求解器提示，不产生碰撞力。
透明颜色目前拒绝，正常颜色要求 rgba 四项有限、在 [0,1]，alpha=1。
OBJ 支持三角形和可扇形三角化的凸平面多边形、正/负索引；法线重算法向，UV/法线索引仍检查有效性。
所有单位采用 m、kg、s、rad，惯性为 kg·m²。URDF rpy 固定使用弧度及 Rz(yaw)Ry(pitch)Rx(roll)。

## 独立的物理适用性检查

XML 解析成功只得到 `ModelDescription`，不是可运行模型。
`buildRigidBody` 再验证根的运动性、每个部件的显式正质量及对称正定惯性，并检查主惯量三角不等式。
不根据视觉大小猜质量，也不以零、单位矩阵补缺失惯性。

设固定树源根为 R，各部件质心为 c_i，惯性从其惯性轴旋转到 R 后为 J_i^R：

```text
M = Σ m_i
c_R = Σ m_i c_i / M
J_C^R = Σ [J_i^R + m_i (||c_i-c_R||² I - (c_i-c_R)(c_i-c_R)^T)]
J_C^B = R_BR J_C^R R_BR^T
p_visual^B = R_BR (p_visual^R - c_R)
```

`root_to_body` 是 Hamilton wxyz 的 R→B 旋转，B 是整机质心处的 FLU 轴；默认假设源根方向已为 FLU。
文件源世界 A 被明确解释为右手 Z-up 的 W，不能自动猜 ENU/NED。
保留文件根位姿时：`p_WB = p_WR + R_WR c_R`，`q_WB = q_WR * inverse(q_BR)`。
显示几何随质心移动而反向偏移，因此移动机体系原点不会移动实际机架。

测试 `fixed_frame.urdf/xml` 的人工参考值：M=2.5 kg，c_R=(0.14,0,0.02) m，
`J_C^R=[[.036,0,-.008],[0,.041,0],[-.008,0,.059]]` kg·m²。
测试同时覆盖整机旋转、自由根、初始位姿、网格资源、无效物理参数及两种文件渲染一致性。

## 资源与错误

相对网格路径以模型文件目录为基准；MJCF 再应用 compiler meshdir。
`package://name/path` 必须给出 `package_roots={"name": "/local/package"}`；相对映射根也以模型目录为基准。
不启动 ROS/xacro、不下载资源；不存在或未映射的资源报错，不替换成占位方块。

当前限制：XML ≤4 MiB、深度 ≤128、节点 ≤10000、body ≤256、visual ≤2048；
每个网格 ≤32 MiB、总解码及显示网格 ≤200000 三角形，OBJ 面 ≤64 顶点。
位姿范围 10000 m、几何尺度范围 1000 m；这些是首版有限场景的输入约束。
拒绝 DTD、无穷/NaN、残缺数字、重复必要节点、环、多根和未知父引用。
C++ `ImportResult` 只有完整成功才包含 description；失败提供文件、行号（可用时）、元素与错误信息。
物理适配的整体错误可能没有单一 XML 行号。Python 映射为 ValueError。

## 文件位置与后续

`description.hpp` 定义中间数据；`src/io/model_import.cpp` 私有 TinyXML2 DOM；`src/io/mesh.cpp` 读取网格；
`src/model/rigid_body.cpp` 做校验和固定树合并；`bindings/python/model_loading.cpp` 是高层加载入口。
`include/csim/simulation/link.hpp` 计算固定 link 状态，`bindings/python/link_binding.cpp` 提供 Python 查询；
`tests/python/test_link_state.py` 验证两种格式、质心偏置、嵌套树、源根变换、旋转导数、吊载耦合和快照语义。
`csim_io` 为链接 TinyXML2 的静态库；公开头文件不暴露 XML DOM。

后续保留：更多资源格式/材质、MJCF defaults/include、通用关节、离心吊点、碰撞和松紧绳切换。
每项扩展需先建立可表达的状态和物理规则，再接受对应 XML；未知语义默认报错。
TinyXML2 只负责 XML，不代替 URDF/MJCF 的物理语义校验。
