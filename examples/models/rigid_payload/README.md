# 三文件刚体吊载输入

输入顺序固定为无人机本体、绳索配置、吊载物本体：

```python
model = csim.load_suspended_model("drone.urdf", "cable.json", "payload.urdf")
data = csim.make_data(model, thrust=15, position_W=[0, 0, 3])
csim.step(model, data)
state = csim.get_state(model, data)
```

两个本体可以分别选择 URDF、MJCF（`.xml` / `.mjcf`）或 JSON，也可以混用。
本目录的三个版本具有相同质量、惯量、质心和吊点位置。

## 固定吊点名

两个 URDF 文件各自包含一个名为 `cable_attachment` 的 link，通过 fixed joint
连接到本体。joint 的 `origin` 给出吊点的位置，可包含 `rpy`；吊点采用 link 原点。
两份文件的命名空间独立，因此可以使用相同 link 名。

```xml
<link name="cable_attachment"/>
<joint name="cable_mount" type="fixed">
  <parent link="base"/>
  <child link="cable_attachment"/>
  <origin xyz="0.15 0 -0.1"/>
</joint>
```

作为坐标标记的空 link 不需要质量、惯量或可视几何。实际物理 link 必须显式给出
正质量和有效惯量；带惯性的吊点 link 也会计入本体聚合。仅支持 fixed 连接树。
加载器将两棵树分别作为自由刚体，绝不会把吊载物聚合到无人机中。

MJCF 使用 `<site name="cable_attachment" pos="..."/>`，也可使用同名的固定子 body
原点（子 body 需要显式惯性）。JSON 的 `cable_attachment` 数组自动生成同名空 link。
绳索配置不用指定 link 名。缺少吊点会报错。

## 坐标与惯量

全部使用 SI 单位，世界坐标 Z 向上，四元数为 `[w, x, y, z]`。
URDF/MJCF 中惯性原点必须是对应 link/body 的质心。聚合器计算整体质量和质心，
旋转各惯量并利用平行轴定理合并，再自动将吊点换算到整体质心坐标。

本示例无人机根坐标的质心为 `[0.05, 0, 0]`，吊点为 `[0.15, 0, -0.1]`，
因此 `model.drone_attachment_B` 为 `[0.1, 0, -0.1]`。
吊载物的对应偏置 `model.payload_attachment_P` 为 `[0.1, 0, 0.1]`。
绳索张力和收紧冲量都通过这两个偏置产生转动力矩。

JSON 本体字段：`name`、`mass`、`inertia`（3×3、关于质心、根坐标轴表达）、
`cable_attachment`（根坐标）；可选 `center_of_mass`（默认零）、
`initial_pose`（`position_W` 和 `q_WR`，表示根坐标在世界的位姿）。
`drone_root_to_body` 和 `payload_root_to_body` 可独立调整两个本体的坐标轴。
配置拒绝未知字段、重复 JSON 键、非有限数和无效惯量。

## 绳索与初态

最小绳索配置为 `{"length": 1.0}`。默认 `mode` 为 `hybrid`，包含松弛、释放和
非弹性收紧事件；`taut` 模式遇到非正张力则报错。可选
`initial_direction_W`（无人机吊点指向载荷吊点的单位向量，默认向下）、
`event_max_step`、`event_tolerance`、`max_events`。事件使用分段采样和二分定位，
快速运动时应减小 `event_max_step`；该算法不保证识别同一采样段内的多次穿越。

默认初态使用无人机文件的根位姿和载荷文件的初始朝向，根据绳长和两端吊点装配
载荷质心位置；载荷文件的世界平移不参与装配。`make_data` / `reset` 可覆盖
`position_W`、`velocity_W`、`q_WB`、`angular_velocity_B`、`payload_position_W`、
`payload_velocity_W`、`payload_q_WP`、`payload_angular_velocity_P`、`mode`。
未指定载荷速度时自动匹配吊点速度；显式绷紧初态必须满足长度和径向速度约束。
`thrust` 和 `torque_B` 是作用在无人机质心的输入，载荷为被动刚体。

`get_state` 返回两刚体质心状态、载荷姿态/角速度、世界吊点状态、张力、能量和
该步事件。固定 link 查询使用 `drone/base`、`payload/base` 等前缀。
步进、重置或控制更新失败时不会提交部分状态。

## 运行示例

从项目根目录运行：

```bash
xmake build csim_python
xmake run csim_python examples/python/rigid_payload.py
xmake run csim_python examples/python/rigid_payload.py --format json
xmake run csim_python examples/python/rigid_payload.py --format mjcf
xmake run csim_python examples/python/rigid_payload.py --viewer --integrator lie_rk4
```

偏置吊点会使两个本体转动，恒定总重推力并不保证姿态悬停。
目前绳索是无质量、不可伸长、两端自由转动的约束，没有绳索弹性、绳索扭转或碰撞接触。
阻力作用在质心，支持线性/二次平移阻力；载荷的 SPAD `k0` 项不适用于此刚体模型。
默认无窗口；`--viewer` 实时显示无人机和载荷的独立姿态、导入几何及实际吊点。
`--hidden --screenshot build/rigid-payload.ppm` 可保存隐藏窗口截图，需要图形显示服务。
松弛绳的虚线仅表示连接关系。可用 `csim.RigidPayloadBatch` 并行推进多个刚体吊载环境；实验 CSV 回放仍未接入刚体吊载。
积分器选择和约束方法的支持范围见 [数值方法说明](../../python/NUMERICAL_METHODS.md)。
