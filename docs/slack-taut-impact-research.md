# 绷紧—松弛与收紧冲击：论文模型调研

调研日期：2026-09-10。目标是为 CSim 选择可实现、可验证的模型，区分连续动力学、模式选择、瞬时冲量和有限时间张力响应。
本文只新增研究记录；当前 CSim 仍是严格绷紧绳模型，没有实现本文讨论的模式切换或冲量求解。

优先参考 **RotorTM 的显式冲击重置**建立单根不可伸长绳基准；需要绳索拉伸和冲击峰值时，再建立独立的单边弹性阻尼模型。
规划中的互补约束可以描述模式选择，但不能单独确定收紧后的速度或真实张力峰值。

2026-09-14 补充：[RotorTM 详细解析](rotortm-analysis.md) 给出统一坐标下的连续动力学、单绳冲量及多机六维求解推导，并核正本文实验章节定位。

## 1. 查阅范围与证据

| 文献 | 已核对内容与定位 | 模型用途及限制 |
| --- | --- | --- |
| Sreenath、Michael、Kumar，2013，*Trajectory Generation and Control of a Quadrotor with a Cable-Suspended Load—A Differentially-Flat Hybrid System* | 本地 ICRA 正文；§II-A/B，式 (6)–(16)，§III 式 (21)–(24) | 非零张力/零张力两套动力学与混合系统框架；该文不是显式冲量重置的主要依据 |
| Kotaru、Wu、Sreenath，2017，*Dynamics and Control of a Quadrotor with a Payload Suspended through an Elastic Cable* | 作者公开 PDF；§II 式 (3)、(8)–(14)、Remark 4，§III-A | 增加轴向伸长与阻尼；主控制推导未显式处理单边张力及完整混合过程 |
| Zeng、Kotaru、Mueller、Sreenath，2020，*Differential Flatness Based Path Planning with Direct Collocation on Hybrid Modes for a Quadrotor with a Cable-Suspended Payload* | 作者公开预印本；§II 式 (1)–(4)、§III 式 (5) | 互补约束＋平坦输出＋直接配点，用于规划；不能直接充当碰撞重置律 |
| Li、Liu、Loianno，2024，*RotorTM: A Flexible Simulator for Aerial Transportation and Manipulation* | arXiv v3，§IV–V；单机式 (27)–(34)，多机式 (39)–(57)，§VII-B | 无质量不可伸长绳、显式模式切换、完全非弹性收紧，含多机/刚体载荷的线速度与角速度重置 |
| Wang、Li、Zhou、Gao、Shen，2024，*Impact-Aware Planning and Control for Aerial Robots With Suspended Payloads* | 本地 T-RO 正文；§III-A 式 (1)–(3)，§IV-C 式 (20)，§V 式 (21)–(30)、Algorithm 2 | 互补约束规划＋含线性阻力的混合 NMPC；核对章节未给出 RotorTM 式的显式冲量重置 |
| Sarvaiya、Li、Loianno，HPA-MPC，2024 年预印本 | 作者 arXiv 正文 §III、§IV–V | 两模式动力学及模式估计/控制，关注真实状态估计；这里引用其模型，不作为新的材料冲击定律 |
| Kotaru、Wu、Sreenath，2018，*Differential-Flatness and Control of Quadrotor(s) with a Payload Suspended through Flexible Cable(s)* | 本地文件内容为 arXiv 2017 v1；对照作者 2018 会议版本入口 | 固定长度杆段＋球铰＋集中质量，描述绳的弯曲形状；不能把 flexible 自动理解为轴向弹性 |
| Cao 等，FLARE，2026 年修订预印本 | 本地正文仿真设置段；作者 arXiv v2 日期已核对 | 用 Genesis 和串联刚性链节训练策略；并未在该段提供独立的闭式收紧冲量律 |

原始来源：

- [Sreenath 2013 DOI](https://doi.org/10.1109/ICRA.2013.6631275)，[本地全文](</home/li/Zotero/storage/P3T8PLWN/Sreenath 等 - 2013 - Trajectory generation and control of a quadrotor with a cable-suspended load - a differentially-flat.pdf>)。
- [Kotaru 2017 作者 PDF](https://hybrid-robotics.berkeley.edu/publications/ACC2017_QuadLoad_ElasticCable.pdf)，[DOI](https://doi.org/10.23919/ACC.2017.7963553)。
- [Zeng 2020 作者 PDF](https://hybrid-robotics.berkeley.edu/publications/RAL2020_PPQL.pdf)，[DOI](https://doi.org/10.1109/LRA.2020.2972845)。
- [RotorTM v3 全文](https://arxiv.org/html/2205.05140v3)，[版本与期刊信息](https://arxiv.org/abs/2205.05140)，[作者代码](https://github.com/arplaboratory/RotorTM)。DOI 含 2023，期刊/作者最终版本标为 2024；本笔记的公式编号按 v3。
- [Impactor DOI](https://doi.org/10.1109/TRO.2024.3381555)，[本地全文](</home/li/Zotero/storage/LMB5NN4P/Wang 等 - 2024 - Impact-aware planning and control for aerial robots with suspended payloads.pdf>)，[作者代码](https://github.com/HKUST-Aerial-Robotics/IMPACTOR)。
- [HPA-MPC 作者全文](https://arxiv.org/html/2411.11982v1)。
- [Flexible Cable 作者会议 PDF](https://hybrid-robotics.berkeley.edu/publications/ICC2018_MultiQuadLoad_FlexCable.pdf)，[预印本](https://arxiv.org/abs/1711.04895)。
- [FLARE 作者版本记录](https://arxiv.org/abs/2508.09797)，[本地全文](</home/li/Zotero/storage/LSG6XBW7/Cao 等 - 2026 - FLARE agile flights for quadrotor cable-suspended payload system via reinforcement learning.pdf>)。首次预印本为 2025，v2 为 2026；不依据 Zotero 文件名推断期刊出版年份。

补充线索：Cruz、Fierro 的 *Cable-suspended load lifting by a quadrotor UAV: hybrid model, trajectory generation, and control*（Autonomous Robots，2017，DOI 10.1007/s10514-017-9632-2）明确区分 Setup、Pull、Raise，并包含载荷由地面进入空中的过程。本次只取得[出版社摘要和注释](https://link.springer.com/article/10.1007/s10514-017-9632-2)，没有取得全文，因此不臆写其地面反力或冲量公式。
另检索到 Cui 等 2026 年关于协同运输、地面接触及互补约束的论文 [DOI](https://doi.org/10.1016/j.conengprac.2026.107037)，出版社全文访问失败，本轮不以它作为方程依据。

## 2. 统一符号与连续动力学

以下是按 CSim 的 Z-up、质心悬挂、质点载荷假设重新整理的方程，不照抄不同论文的方向符号。

```text
r = p_L - p_Q
 d = |r|                  两端距离
 l0                       不可伸长绳长度；弹性模型中则表示自然长度
 s = r/d                  两端连线单位方向（d>0 时定义）
 u = s·(v_L-v_Q)          分离为正的径向相对速度
 g_W = (0,0,-g)
 mu = m_Q*m_L/(m_Q+m_L)   约化质量
```

不可伸长绳的约束为：

```text
phi = l0-d >= 0
T >= 0
T*phi = 0
```

这是单边约束。`phi=0,T=0` 是退化边界，三个条件本身并没有唯一规定边界处的模式或冲击后速度。
绳索实体长度不变，松弛时变化的是两端直线距离；论文把该距离也写作 l 时，不应误解为自动收放绳。

令 F_Q、F_L 为除重力和绳力之外的两端外力，包含已有推力和空气力：

```text
m_Q*a_Q = m_Q*g_W + F_Q + T*s
m_L*a_L = m_L*g_W + F_L - T*s
```

绷紧时 `d=l0`、`s·(v_L-v_Q)=0`，由固定距离二阶导数消去张力：

```text
T_required = mu * ( |v_L-v_Q|^2/l0 + s·(F_L/m_L-F_Q/m_Q) )
```

松弛时 `T=0`，两端分别运动。无其他外力时吊载是抛体；启用空气阻力后不再是纯重力抛体。

上述双分支与 Sreenath 2013 的基本建模路线一致；HPA-MPC 也明确写出两种连续动力学。
在仿真实现中，需要依据 T_required 的变化方向处理释放，并特别处理零径向速度、零张力边界；不能只依赖等号比较。

## 3. RotorTM：显式非弹性收紧模型

RotorTM §V-A 将释放映射取为恒等映射；收紧条件写为 `d=l0, d_dot>0`。
式 (31)–(34) 分解平行/垂直绳方向的速度：径向总动量守恒，收紧后两端径向速度相同，各自切向速度保持。
§V-B 进一步用冲量—动量和角冲量方程联合求解刚体吊载的速度变化；其结果包含载荷角速度，不能逐绳独立修正后直接相加。
§VII-B 给出单机收紧仿真及三机刚体载荷的收紧仿真/实机对比，其中三机场景使用实测初态进行仿真比较；单机案例不应表述为同样完成了实机冲击验证。[原文](https://arxiv.org/html/2205.05140v3)

将单机关系化为适合 CSim 的标量冲量表示：

```text
P = mu*u_minus                         P 的单位是 N*s
v_Q_plus = v_Q_minus + (P/m_Q)*s
v_L_plus = v_L_minus - (P/m_L)*s
```

位置、姿态不跳变；当前质心悬挂下无人机角速度不跳变。可直接推导：

```text
u_plus = 0
m_Q*v_Q_plus + m_L*v_L_plus = m_Q*v_Q_minus + m_L*v_L_minus
E_plus - E_minus = -0.5*mu*u_minus^2
```

这里的“collision”指绳索突然约束相对运动，不要求无人机和载荷实体碰撞。
它提供收紧前后状态及损失能量，但不提供有限冲击时长或有限张力峰值。

CSim 的实现补充：现有绷紧参数化严格保持 d=l0，不能机械照搬 `d<l0` 作为唯一释放检测条件，否则约束积分永远进不了该集合。应分离张力计算与合法性检查，定位张力失效，再切换到自由运动。
对于多绳，除绷紧根数外还必须保留具体绳索索引集合；相同根数并不意味着相同动力学。

## 4. 恢复系数、有效质量与多约束冲量

本节是对冲量模型的通用扩展和 CSim 推导，不声称以上论文都使用了可调恢复系数。

若用 Newton 径向恢复系数 e，规定 `u_plus=-e*u_minus`，则：

```text
0 <= e <= 1
P = (1+e)*mu*u_minus
Delta_E = -0.5*mu*(1-e^2)*u_minus^2
```

`e=0` 对应前述完全非弹性收紧。`e>0` 后两端径向运动转为接近，应允许再次松弛，不能把速度改成负径向后又立即投影成零。
真实绳索的等效 e 应通过相应工况辨识，不能把它与已有 SPAD 系数混为一谈。

若悬点偏心，径向速度必须使用两端挂点速度，而不是只用质心速度：

```text
v_attach = v_COM + omega_W × r_attach_W
```

两端均为刚体且只有一根绳时，逆有效质量增加转动贡献：

```text
1/m_eff = 1/m_Q + 1/m_L
        + (r_Q×s)^T * I_Q_W^{-1} * (r_Q×s)
        + (r_L×s)^T * I_L_W^{-1} * (r_L×s)
```

点载荷删除其转动项，质心悬挂删除相应偏置项。冲量还会通过挂点产生角动量变化。
多绳、地面或其他接触同时发生时，各约束会通过质量和惯性矩阵耦合，需联合求解。

## 5. 互补约束：区分规划与冲击求解

Zeng 2020 在配点问题中使用非负张力、距离上限及二者互补，使优化器选择松/紧模式。
Impactor 2024 采用同类模式约束进行多项式轨迹优化，另外使用含线性阻力的混合 NMPC；Algorithm 2 按预测距离设置模式。
在核对的这些模型/算法段落中，没有与 RotorTM 等价的显式冲量—速度重置公式。
因此不能仅凭标题 impact-aware 就认为它给出了可复用的冲击力积分器。[Zeng](https://hybrid-robotics.berkeley.edu/publications/RAL2020_PPQL.pdf) · [Impactor](https://doi.org/10.1109/TRO.2024.3381555)

要由互补约束真正计算冲击，还要加上动量更新和冲击闭合条件。以下以无摩擦、完全非弹性、事件发生在间隙零点为例：

```text
M*(v_plus-v_minus) = G^T*P
0 <= P  ⟂  G*v_plus >= 0
```

其中 `G` 是非负间隙函数对广义坐标的雅可比；单绳的平移部分是 `[s^T,-s^T]`，所以 `G*v=-u`。
符号 `⟂` 表示每一对非负分量的乘积为零。消去 v_plus：

```text
0 <= P  ⟂  G*v_minus + G*M^{-1}*G^T*P >= 0
```

这才形成可求解的冲量互补问题，并在单绳质心案例退化为 P=mu*u_minus。
绳索外的地面接触也可以增加自己的间隙和雅可比，但摩擦、冗余约束及多个恢复条件需要进一步处理。

Stewart–Trinkle 的时间步进路线将一个时间步内的约束作用放进动量/冲量更新，可以联合处理同时碰撞，避免逐个精确定位所有冲击；它仍需步长收敛检验。
这里已核对作者公开的 ICRA 2000 论文 [An Implicit Time-Stepping Scheme for Rigid Body Dynamics with Coulomb Friction](https://www.cse.lehigh.edu/~trink/Papers/STicra00.pdf)。上式是无摩擦瞬时事件的简化说明，不是该论文完整离散算法，也不是 Zeng/Impactor 原式。

## 6. 弹性阻尼：有限时间的收紧响应

Kotaru 2017 增加绳索长度 l 和 l_dot，自由度从固定绳长模型增加一个；弹性势能为 `0.5*k*(l-L)^2`，阻尼与轴向速度成正比。
§II 式 (14) 给出绳长加速度，§III-A 分析大刚度/阻尼下的快慢时间尺度。
**Remark 4 明确说明主动力学未考虑张力非负约束**，并提出混合扩展、释放恒等映射与收紧完全非弹性映射；后续控制分析不显式考虑这些混合事件。
因此，它是弹性动力学的重要依据，不能直接称为已经完整验证的单边弹性冲击求解器。[作者原文](https://hybrid-robotics.berkeley.edu/publications/ACC2017_QuadLoad_ElasticCable.pdf)

CSim 可另行评估的工程候选是单边弹簧—阻尼形式：

```text
delta = d-l0
T = 0                                     d < l0
T = max(0, k*delta + c*d_dot)               d >= l0
```

这不是原论文逐字给出的完整接触定律。需明确加载、卸载、力释放条件，并验证耗散/储能记账；简单截断可能引入卸载误差。
有限刚度模型允许 d>l0。通过积分伸长和速度，获得 T(t)、最大伸长、衰减和反弹，而非预先强制径向速度跳变。
若同时使用瞬时冲量，应有明确的额外物理来源和校准，不能默认叠加两种耗能规则。

用于理解尺度的局部推导：忽略外部驱动、绳方向变化和阻尼，轴向方程近似为

```text
mu*delta_ddot + k*delta = 0
omega_n = sqrt(k/mu)
delta(0)=0, delta_dot(0)=u_minus
T_peak = u_minus*sqrt(k*mu)
t_peak = (pi/2)*sqrt(mu/k)
```

这只描述第一次伸长峰值的理想近似，不是带阻尼完整吊载系统的峰值公式。
它说明：刚度增大时峰值上升、响应时间缩短；刚性极限保留冲量，不能给出有限峰值。
带阻尼时的轴向阻尼比近似 `zeta=c/(2*sqrt(k*mu))`。步长必须解析该快过程，必要时采用自适应/隐式积分。
已有 SPAD 是基于载荷速度的等效外力；轴向阻尼是基于两端距离变化率的内部力，两者不能相互替代。

## 7. 柔性链节模型能解决什么

Kotaru 的 flexible-cable 工作将绳离散为固定长度杆段和球铰，质量集中在节点，配置增加各段方向。
它允许整条绳弯曲，两个端点间距离自然可以小于各段长度之和；但固定长度段本身不等于可伸长、仅受拉的弹性单元。
FLARE 的仿真设置也使用串联刚性链节；其物理引擎负责约束求解，论文该段并未给出可直接照搬的独立冲量公式。
因此，这一路线适合后续绳体形状、质量、障碍物缠绕和接触，而不是当前单根无质量绳的最小实现。
[Flexible Cable](https://hybrid-robotics.berkeley.edu/publications/ICC2018_MultiQuadLoad_FlexCable.pdf) · [FLARE](https://arxiv.org/abs/2508.09797)

## 8. 对 CSim 的建议及验收

| 实验目标 | 选择 | 需要输出 |
| --- | --- | --- |
| 松绳飞行、重新收紧及控制恢复 | 显式事件＋不可伸长绳＋e=0 冲量 | 模式、事件时刻、两端速度、冲量、损失能量 |
| 偏心挂点、刚体载荷或多绳 | 包含转动惯性的联合冲量求解 | 各绳冲量、载荷线/角速度、活跃约束集合 |
| 地面接触与多约束反复作用 | 后续评估互补时间步进 | 接触/绳索间隙、冲量、摩擦与求解残差 |
| 真实张力峰值、伸长与冲击振荡 | 单边有限刚度/阻尼模型，实测标定 | T(t)、delta(t)、峰值、持续时间与能量 |
| 绳自身形状、质量和接触 | 多段离散绳或连续体 | 节点/段状态、内力、形状与接触 |

实施顺序：先保留现有 strict_taut 基准，新增混合模型状态、原始 T_required 计算和事件定位，再实现单绳冲量与事件记录。
Python 仍使用 step/get_state；模式和冲量作为物理诊断输出，不公开底层积分或互补求解器。
地面接触应单独实现并与绳约束联合检查，不能把载荷落地等同于自由飞行冲量问题。

验收项目：

1. 释放时位置和速度连续；零张力退化状态不产生无意义的模式抖动。
2. 松绳时 T=0；收紧事件位于 d=l0 且存在向外径向速度，或需激活约束的零速边界。
3. e=0 收紧后径向相对速度为零，线动量守恒，能量损失符合上式；偏心模型进一步核对角动量。
4. 收紧后的张力合法性重新判断；不强迫不可维持的约束持续活跃。
5. 步长变化下事件时间、冲量、轨迹收敛；事件子步保持该物理步实际输入，不调用 Python 的 CTBR 采样、执行器响应或指令延迟时钟。
6. 冲量单位 N*s 与持续张力 N 分开；P/dt 只能是人为时间窗上的等效平均量，不是预测的物理峰值。
7. 保留全部事件前后状态及模式，扩展现有记录/回放；已有无切换基准继续通过。

该笔记完成文献与方程整理，不代表已实现/验证混合绳索仿真，也不代表复现了上述论文的实物实验。

[当前耦合模型](suspended-payload.md) · [SPAD 与风场](aerodynamics.md) · [开发计划](development-workflow.md)
