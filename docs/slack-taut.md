# 绷紧/松弛绳索接口

`SuspendedPayloadModel` 默认仍使用严格绷紧模式 `cable_mode="taut"`。需要模拟端点分离时可选 `cable_mode="hybrid"`。

混合模式在松弛时保存吊载独立位置和速度，并分别积分两端：`mQ*aQ=mQ*g+F`，`mL*aL=mL*g+Fair`，绳力为零。当绷紧状态计算出的约束张力 `T<=0` 时，步进在该外部时间步结束处切换到松弛；当两端距离 `d>=l` 时，把吊载投影到 `d=l`，再用完全非弹性径向冲量消除相对径向速度：

```text
mu = mQ*mL/(mQ+mL)
J = mu * ((vL-vQ) dot s)
vQ+ = vQ + J*s/mQ
vL+ = vL - J*s/mL
```

这保留切向摆动速度，并在 `get_state` 中报告 `cable_impulse_W`。当前事件处理位于物理步末端，未对穿越时刻做连续时间根定位；减小 `timestep` 可降低事件位置误差。绳索弹性、恢复系数、地面接触和多根绳联合冲量仍未实现。

## Python 用法

```python
import csim

model = csim.SuspendedPayloadModel(cable_mode="hybrid", timestep=0.001)
data = csim.make_data(
    model, thrust=0.0, position_W=[0, 0, 5],
    payload_position_W=[0, 0, 4.5], payload_velocity_W=[0, 0, 0],
    cable_mode="slack",
)
for _ in range(1000):
    csim.step(model, data)
state = csim.get_state(model, data)
print(state["mode"], state["cable_distance"], state["cable_impulse_W"])
```

`make_data` 和 `reset` 的 `cable_mode` 表示初始分支，取值为 `"taut"` 或 `"slack"`；`"slack"` 只允许用于混合模型，且必须提供 `payload_position_W`。

| 字段 | 含义 |
| --- | --- |
| `mode` | 当前分支，`"taut"` 或 `"slack"` |
| `cable_distance` | 两端距离，m |
| `cable_radial_velocity` | `(vL-vQ)·s`，m/s |
| `cable_impulse_W` | 最近一次收紧事件施加在无人机上的冲量，N·s；无事件为零 |

严格 `taut` 模式遇到非正张力仍抛出 `CableDomainError`，旧代码的失败行为保持不变。混合模式允许读取非正的未约束张力并自动切换。可视化接口会显示松弛绳和吊载，零张力不会被当作非法显示状态。
