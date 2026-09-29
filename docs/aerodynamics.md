# SPAD、相对空气速度阻力与风场

无人机和绷紧吊载模型现已支持确定性风场及相对空气速度阻力；SPAD 仅用于质点吊载。
所有阻力系数默认是零，风场默认静止。默认模型与改动前的无人机/吊载轨迹基准逐项一致。
论文的来源、坐标转换、建模推导及系数适用范围见 [Zhu 等解析](zhu-2025-payload-model.md)。

## 使用

```python
import csim

model = csim.SuspendedPayloadModel(
    timestep=0.002,
    drone_drag=csim.DragConfig(k1=0.1, k2=0.02),
    payload_drag=csim.DragConfig(k1=0.0034, k0=0.001,
                                sign_mode="tanh", epsilon_v=0.02),
    wind=csim.WindField(velocity_W=[2, 0, 0],
                        gust_amplitude_W=[0, 0.3, 0], gust_frequency=0.5),
)
data = csim.make_data(model, position_W=[0, 0, 5],
                      thrust=(model.drone_mass + model.payload_mass)*model.gravity)
csim.step(model, data)
print(csim.get_state(model, data)["aerodynamics"])
print(csim.get_config(model))
```

这些数值仅演示配置，不代表当前机体的辨识结果。无人机使用 `DroneModel(drone_drag=..., wind=...)`；
文件模型使用 `load_model(..., drone_drag=..., payload_drag=..., wind=...)`，仅吊载模型接受非零 payload_drag。
DragConfig/WindField 复制到不可变模型，`to_dict()` 返回独立参数副本。

## 风与力的定义

全部矢量在固定 Z-up 世界系 W 下表达。每个物体使用自己的位置和绝对速度；吊载速度不是相对悬点速度：

```text
p_L = p_Q + l*s
v_L = v_Q + l*(omega_c × s)
w(p,t) = velocity_W + gradient_W*(p-reference_W)
         + gust_amplitude_W*sin(2*pi*gust_frequency*t + gust_phase)
v_air = v_body_W - w(p_body_W,t)
F_linear    = -k1*v_air
F_quadratic = -0.5*k2*norm(v_air)*v_air
F_spad      = -k0*sign(v_air)                  # 仅吊载
F_air       = F_linear + F_quadratic + F_spad
```

`gradient_W` 为 3×3 矩阵，行表示速度分量、列表示位置分量，单位 1/s；`reference_W` 单位 m；
风速与阵风幅值单位 m/s，频率 Hz，相位 rad。当前支持均匀风、仿射空间变化与正弦阵风，
没有随机湍流、流体网格或 Python 风场回调。物理使用仿真时间，在每个 RK 阶段重算两端的风与受力。

系数必须有限非负：k1 为 kg/s，k2 为 kg/m，k0 为 N。k2 保留论文公式中的 1/2。
`sign_mode="exact"` 逐 W 分量取符号，sign(0)=0，要求 epsilon_v=0；
`sign_mode="tanh"` 用 tanh(v_i/epsilon_v)，要求有限正 epsilon_v（m/s）。
逐分量 sign 不是 v/norm(v)，也不是各向同性模型。平滑模式不是论文原式；exact 换向不保证 RK4 四阶收敛。

SPAD 是施加在吊载上的经验外力。无人机通过张力受到其间接影响，不额外施加相反的 SPAD 力。
真实接头摩擦依赖相对转动；将 SPAD 按空速计算是本项目记录的扩展，风下参数需重新验证。
空气力施加于各自质心，当前没有空气阻力矩、旋翼下洗、风对绳索的分布力或载荷姿态。

## 张力与能量

令 F_Q 为无人机推力加空气力，F_L 为吊载空气力，均不含重力及张力：

```text
mu = 1 / (1/m_Q + 1/m_L)
T = mu * (l*|s_dot|^2 + s·(F_L/m_L - F_Q/m_Q))
a_Q = (0,0,-g) + (F_Q + T*s)/m_Q
a_L = (0,0,-g) + (F_L - T*s)/m_L
omega_c_dot = s × (F_L/m_L - F_Q/m_Q) / l
```

阻力的径向分量也进入张力，不只修改摆动。必须 T>0；任何 RK 阶段失败均不提交物理状态；使用 Python ControlLoop 时其候选队列和执行器状态也不提交。
没有松绳/再绷紧事件、接触或地面约束。

`get_state()["aerodynamics"]` 包含 drone，以及吊载模型的 payload：

| 字段 | 含义 |
| --- | --- |
| wind_velocity_W / air_velocity_W | 该物体位置处风速 / 相对空气速度 |
| linear_force_W / quadratic_force_W / spad_force_W | 分项实际力，N |
| total_force_W | 三项之和；动力学实际施加的空气力 |
| wind_induced_force_W | F(v-w)−F(v)，同一对地速度下相对无风的力变化，仅作诊断，不重复施加 |
| air_power | F_air·v_air，W，非负系数下非正 |
| mechanical_power | F_air·v_body_W，W，有风时可以为正 |

受控自由飞行不能只检查机械能单调下降。能量校验应包含推力功率、机体力矩功率与两端 mechanical_power。
关闭所有系数后仍可读取风速，但风不产生物理作用。

## 验证

`tests/python/test_aerodynamics.py` 检查改动前基准、两端力平衡与径向张力、绝对空速、零速与 sign/tanh、
耗散及输入功平衡、正弦风解析响应和步长收敛、非有限输入与失败原子性。
基准存于 `tests/fixtures/aerodynamics_baseline.json`。尚无原论文原始实验数据，未复现其辨识 RMSE。
