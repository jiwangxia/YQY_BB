# MPC 约束理论与实现

本文按线性约束、静力非线性消元、连续动力、时间离散、刚性中心间隔棒和验证边界组织 MPC 理论。它合并原有六份说明，以完整静力推导为公共主干，避免在动力章节重复推导同一组 $c,G,H,T,p$。

当前 YQY 的约束对象在静力与动力中共用约束值、Jacobian 和 Hessian。Newmark 与自适应 TSSBN 已接入隐式 Newton 迭代，并检查位置、速度和加速度约束；RK4 当前明确拒绝非线性 MPC。严格能量—动量格式和 Betsch 离散零空间算法仍属于研究方向。

符号约定：第 2 节保留 Nastran 线性文献的 $n$ 独立、$m$ 从属记号；第 3 节起改用 $m$ 主、$s$ 从，不能跨节直接套用下标。静力的 $\boldsymbol V$ 与动力的 $\mathbf q$ 均表示当前广义构型；论文残差统一为内力减外力，程序求解右端使用其相反数。

## 1. 来源、公式职责与实现边界

静力推导保留 Boungard–Wackerfuß 论文式 (1)–(6)、(20)–(39)、(59)–(64) 的对应关系。动力部分是结合主从运动学与当前积分器的离散实现，不能标成该静力论文的原式。

### 1.1 Boungard 与 Wackerfuß（2024）

[Master–slave elimination scheme for arbitrary smooth nonlinear multi-point constraints](https://link.springer.com/article/10.1007/s00466-024-02463-7)

完整出处：J. Boungard, J. Wackerfuß, *Computational Mechanics* 74 (2024), 955–992，DOI: 10.1007/s00466-024-02463-7。

论文明确研究的是静力有限元中的、与时间无关的光滑非线性 MPC。它给出了任意约束的统一数据：

$$
\mathbf c(\mathbf q),\qquad
\mathbf G=\frac{\partial\mathbf c}{\partial\mathbf q},\qquad
\mathbf H_\alpha=\frac{\partial^2c_\alpha}{\partial\mathbf q^2},
$$

以及主从消元后残量、切线和约束乘子的统一公式。本文静力部分以它为依据；[本地论文](../参考文献/Boungard_Wackerfuss_2024_Master_slave_elimination.pdf)保留原公式编号。

需要特别说明：Boungard 2024 没有给出 Newmark 动力公式，不能把本文后面的动力离散公式说成“Boungard 论文原式”。

### 1.2 Jelenić 与 Crisfield（1996、2001）

1996 年论文讨论大转动三维梁关节的非线性主从运动学；2001 年论文 [Dynamic analysis of 3D beams with joints in presence of large rotations](https://www.sciencedirect.com/science/article/abs/pii/S0045782500003443) 把这种关节主从关系扩展到动力问题，并关注无外载 Hamilton 系统中的能量和动量性质。

它们的重要作用是说明：梁关节的 transformation 依赖当前构型，不能把大转动约束当成常系数线性 MPC。

### 1.3 Ibrahimbegović 与 Mamouri（2000）

[On rigid components and joint constraints in nonlinear dynamics of flexible multibody systems employing 3D geometrically exact beam model](https://www.sciencedirect.com/science/article/abs/pii/S0045782599003631) 讨论几何精确梁、刚性部件和关节的非线性动力学，包含乘子法以及直接消除相对运动自由度的思路。

它提供了动力刚性偏置必须满足的位置、速度、加速度关系，尤其指出动力中不能遗漏向心加速度项。

### 1.4 Crisfield、Galvanetto 与 Jelenić（1997）

本项目 [Crisfield–Galvanetto–Jelenić 1997 原文](../参考文献/Crisfield_Galvanetto_Jelenic_1997_Dynamics_3D_corotational_beams.pdf) 研究无 MPC 的三维共旋梁动力学。论文给出了端点 Newmark 类格式的惯性力、一致线性化，以及中点近似能量守恒格式。

它解决的是“梁的动力残量和动力切线如何构造”，不是“通用 MPC 如何消元”。本程序必须先有正确的无约束动力系统，MPC 才有可投影的对象。

### 1.5 Le、Battini 与 Hjiaj（2012）

[Dynamics of 3D beam elements in a corotational context](https://www.sciencedirect.com/science/article/abs/pii/S0168874X12001217) 对共旋三维梁的 Newmark 时间积分、有限转动更新及多种动力公式进行比较。当前程序节点的有限转动 Newmark 更新采用了这一类空间旋量与材料增量转角转换思想。

### 1.6 Betsch（2005）及后续论文

[The discrete null space method, Part I: Holonomic constraints](https://www.sciencedirect.com/science/article/pii/S0045782505000137) 的关键原则是：先把受约束动力 DAE 离散，再在离散系统上构造零空间并消除乘子。Part II 将其扩展到多体动力学。

这为本程序“先形成 Newmark 离散动力残量和有效切线，再做主从消元”的顺序提供理论依据。不过，当前实现还不是 Betsch 的能量一致离散零空间算法，只遵循了它的离散顺序。

---

### 1.7 补充文献与原阅读路线的保留范围

| 文献 | 保留的理论职责 | 当前状态 |
|---|---|---|
| Jelenić & Crisfield, 1996, *Non-linear ‘master-slave’ relationships for joints in 3-D beams with large rotations* | 构型相关主从运动学、球铰/转动副/移动副/圆柱副与释放 | 为约束建模提供来源，不表示 YQY 已有全部关节 |
| Ibrahimbegović & Al Mikdad, 1998, *Finite rotations in dynamics of beams and implicit time-stepping schemes* | $SO(3)$ 上有限转动、角速度和角加速度的隐式更新 | 有限转动不能直接相加 |
| Betsch & Leyendecker, 2006, *The discrete null space method … Part II: Multibody dynamics* | 刚体、关节与闭环多体约束 | 专门离散零空间版本未实现 |
| Leyendecker, Betsch & Steinmann, 2008, *The discrete null space method … Part III: Flexible multibody dynamics* | 刚体、梁、壳与关节的统一柔性多体框架 | 研究参考 |
| Leyendecker, Betsch & Steinmann, 2006, *Objective energy–momentum conserving integration for the constrained dynamics of geometrically exact beams* | 能量、线动量、角动量和长期约束漂移 | 专门能量—动量版本未实现 |

原阅读路线中的“先实现最简单动力 MPC，再接 Newmark”等步骤已由现有代码完成，不再作为当前待办。仍有价值的顺序是：先核实无 MPC 梁动力本体，再验证约束动力学，最后研究复杂关节及守恒格式。研究中的离散零空间满足

$$
\mathbf G\mathbf P=\mathbf0,\qquad \Delta\mathbf q=\mathbf P\Delta\mathbf z.
$$

它与当前构型切空间投影有共同数学背景，但不能据此认定当前程序已实现论文的能量一致离散算法。

## 2. 线性 MPC：变换消元的基础

MPC 给出多个自由度之间的关系，SPC 直接规定一个自由度的数值。以下线性理论可解释等位移、固定比例和线性插值；大转动刚性连接必须使用后文的构型相关约束。

### 2.1 一般形式

有限元离散后的平衡方程为：

$$
K u = F
$$

其中：

- $K$ 为整体刚度矩阵；
- $u$ 为整体位移向量；
- $F$ 为整体载荷向量。

一组线性多点约束可以写成：

$$
R u = d
$$

其中 $R$ 为约束系数矩阵，$d$ 为约束右端项。Nastran `MPC` 卡片通常描述齐次约束：

$$
R u = 0
$$

单条约束可以表示为：

$$
a_1u_1+a_2u_2+\cdots+a_pu_p=0
$$

将其中一个自由度选为从自由度（dependent DOF 或 m-set），其余自由度作为主自由度（independent DOF），就可以利用约束方程消去从自由度。

### 2.2 主从自由度划分

将自由度重新排列为：

$$
u=
\begin{bmatrix}
u_n\\
u_m
\end{bmatrix}
$$

其中：

- $u_n$ 是保留并参与求解的主自由度；
- $u_m$ 是通过 MPC 消去并在求解后恢复的从自由度。

约束方程相应地写成：

$$
R_{mn}u_n+R_{mm}u_m=d
$$

如果 $R_{mm}$ 可逆，则：

$$
u_m=-R_{mm}^{-1}R_{mn}u_n+R_{mm}^{-1}d
$$

定义：

$$
G=-R_{mm}^{-1}R_{mn},\qquad g=R_{mm}^{-1}d
$$

于是：

$$
u_m=Gu_n+g
$$

齐次线性 MPC 取 $d=0$，因此：

$$
u_m=Gu_n
$$

### 2.3 变换矩阵

齐次 MPC 下，完整位移可以写成：

$$
u=
\begin{bmatrix}
u_n\\
u_m
\end{bmatrix}
=
\begin{bmatrix}
I\\
G
\end{bmatrix}u_n
=Tu_n
$$

其中：

$$
T=
\begin{bmatrix}
I\\
G
\end{bmatrix}
$$

是从缩减自由度空间映射到完整自由度空间的变换矩阵。

这一表示保证任何由 $u=Tu_n$ 得到的位移都自动满足 MPC 方程，因为：

$$
RT=0
$$

### 2.4 缩减方程的理论依据

MPC 消元可以由虚功原理直接推导。齐次约束下：

$$
u=Tu_n,\qquad \delta u=T\delta u_n
$$

完整系统的虚功方程为：

$$
\delta u^T(Ku-F)=0
$$

代入位移变换可得：

$$
\delta u_n^T T^T(KTu_n-F)=0
$$

由于任意允许的 $\delta u_n$ 都应满足该方程，因此缩减系统为：

$$
\bar K u_n=\bar F
$$

其中：

$$
\bar K=T^TKT
$$

$$
\bar F=T^TF
$$

因此，这种处理不是经验修正，而是把平衡方程投影到满足约束的运动空间中。

### 2.5 分块矩阵展开

将整体刚度矩阵和载荷向量划分为：

$$
K=
\begin{bmatrix}
K_{nn} & K_{nm}\\
K_{mn} & K_{mm}
\end{bmatrix},\qquad
F=
\begin{bmatrix}
F_n\\
F_m
\end{bmatrix}
$$

代入 $T=[I\;G]^T$ 后：

$$
\bar K=
K_{nn}+K_{nm}G+G^TK_{mn}+G^TK_{mm}G
$$

$$
\bar F=F_n+G^TF_m
$$

这说明不能只删除从自由度对应的矩阵行和列。来自 $K_{nm}$、$K_{mn}$、$K_{mm}$ 和 $F_m$ 的贡献必须通过 $G$ 转移到保留自由度上，否则模型的刚度、载荷和能量都将不正确。

如果原始 $K$ 对称，则 $T^TKT$ 仍然对称。如果原始 $K$ 正定，并且 $T$ 列满秩，则缩减后的刚度矩阵保持正定。

### 2.6 动力学方程

对于线性动力学方程：

$$
M\ddot u+C\dot u+Ku=F
$$

当约束关系与时间无关且 $u=Tu_n$ 时：

$$
\bar M=T^TMT
$$

$$
\bar C=T^TCT
$$

$$
\bar K=T^TKT
$$

$$
\bar F=T^TF
$$

因此刚度、质量、阻尼和载荷必须使用同一个变换矩阵进行处理。只凝聚刚度矩阵而不凝聚质量矩阵，会改变系统的固有频率和振型。

若 $T$ 或非齐次项 $g$ 随时间变化，还会出现额外的惯性项和阻尼项，不能直接使用上述时间无关公式。

### 2.7 非齐次约束

对于：

$$
u_m=Gu_n+g
$$

可以写成：

$$
u=Tu_n+u_0,\qquad
u_0=
\begin{bmatrix}
0\\
g
\end{bmatrix}
$$

静力缩减方程变为：

$$
\bar K=T^TKT
$$

$$
\bar F=T^T(F-Ku_0)
$$

这是一般线性约束的非齐次形式。历史 SGFem `MPCData` 未启用非齐次项；该数据结构不属于当前 YQY。当前 YQY 按具体几何约束对象计算 $c,G,H$，是否支持某种非齐次关系须以约束类和输入契约为准。

### 2.8 结果恢复

缩减系统求解得到 $u_n$ 后，从自由度由下式恢复：

$$
u_m=Gu_n
$$

然后按照全局自由度编号将 $u_n$、SPC 自由度和 $u_m$ 重新组合为完整结果向量。

变换消元法不会直接产生 MPC 约束力。如果需要输出约束力，可以在恢复完整位移后计算残差：

$$
r=Ku-F
$$

并根据约束矩阵求取与 $R^T\lambda$ 对应的约束反力。约束反力的分解可能受约束冗余和符号约定影响。

### 2.9 从自由度选择条件

约束消元能否成立，关键不只是某一条 MPC 的首项系数是否非零，而是全局矩阵 $R_{mm}$ 是否可逆。

合理的 MPC 系统应满足：

1. 约束方程数量与选定的从自由度数量一致；
2. 约束方程线性独立；
3. 每个从自由度具有唯一、可确定的从属关系；
4. $R_{mm}$ 满秩且数值条件良好；
5. 从自由度不能再被 SPC 直接指定；
6. 约束不能与其他 MPC、刚性单元或连接关系互相矛盾。

对于一条简单 MPC：

$$
a_mu_m+\sum_i a_i u_i=0
$$

只要 $a_m\ne0$，就可以得到：

$$
u_m=-\sum_i\frac{a_i}{a_m}u_i
$$

但多条 MPC 联立时，即使每条方程的首项系数都非零，整体 $R_{mm}$ 仍可能由于重复约束、循环依赖或线性相关而奇异。因此最终判据必须是全局 $R_{mm}$ 的秩和条件数。

### 2.10 数值稳定性

理论上，将某条约束方程整体乘以任意非零常数不会改变约束空间。但在浮点计算中，系数尺度差异过大会影响 $R_{mm}$ 求解的稳定性。

建议：

- 避免在同一约束中混用相差过大的系数；
- 对约束方程进行适当尺度归一化；
- 求解 $R_{mm}X=-R_{mn}$，而不是显式构造 $R_{mm}^{-1}$；
- 对 $R_{mm}$ 做秩检查或条件数估计；
- 对近奇异约束给出明确诊断，而不是继续产生 NaN；
- 对重复从自由度、SPC/MPC 冲突和闭合约束环提前检查。

### 2.11 与其他约束方法的比较

#### 2.11.1 变换消元法

优点：

- 精确满足线性约束；
- 不引入罚参数；
- 不增加求解方程数量；
- 对称系统凝聚后仍可保持对称性。

局限：

- 需要合理选择从自由度；
- 需要保证 $R_{mm}$ 可逆；
- 约束关系复杂时，变换矩阵可能增加矩阵填充；
- 位移主求解不以乘子为额外未知量；仍可由未消元的平衡方程恢复约束乘子和反力。

#### 2.11.2 罚函数法

通过在刚度矩阵中加入大刚度近似满足约束。实现简单，但结果依赖罚参数；罚参数过小会产生约束误差，过大则会恶化矩阵条件数。

#### 2.11.3 拉格朗日乘子法

通过增加约束变量精确满足约束，并能直接得到约束力，但会扩大方程规模，并形成鞍点型矩阵。

当前 YQY 使用下文的非线性主从消元；线性关系是其常 Jacobian、零 Hessian 特例。SPC 直接规定自由度值，处理方式与 MPC 不同。

## 3. 静力非线性 MPC：统一推导与实现

每种约束只向公共消元器提供 $c,G,H$ 和从自由度选择。定长、刚性偏置、平面释放等关系共用同一套残量和切线推导，不需要各写一个求解器。

### 3.1 符号和维数

设离散模型共有 \(n_{\mathrm{dof}}\) 个参与当前方程的自由度，有 \(n_c\) 个标量约束。

| 符号 | 维数 | 含义 |
|---|---:|---|
| \(\boldsymbol V\) | \(n_{\mathrm{dof}}\times 1\) | 当前广义自由度，可包含平动和转动 |
| \(W(\boldsymbol V)\) | 标量 | 总势能或与静力平衡等价的势函数 |
| \(\boldsymbol R\) | \(n_{\mathrm{dof}}\times 1\) | 论文的结构残差，\(\boldsymbol R=\partial W/\partial\boldsymbol V\) |
| \(\boldsymbol K_T\) | \(n_{\mathrm{dof}}\times n_{\mathrm{dof}}\) | 无约束问题的一致切线矩阵，\(\partial\boldsymbol R/\partial\boldsymbol V\) |
| \(\boldsymbol c\) | \(n_c\times1\) | 标量约束组成的向量，目标是 \(\boldsymbol c=\boldsymbol0\) |
| \(\boldsymbol G\) | \(n_c\times n_{\mathrm{dof}}\) | 约束 Jacobian，\(\partial\boldsymbol c/\partial\boldsymbol V\) |
| \(\boldsymbol H_i\) | \(n_{\mathrm{dof}}\times n_{\mathrm{dof}}\) | 第 \(i\) 个约束的 Hessian，\(\partial^2c_i/\partial\boldsymbol V^2\) |
| \(\boldsymbol V_m\) | \((n_{\mathrm{dof}}-n_c)\times1\) | 保留并求解的主自由度 |
| \(\boldsymbol V_s\) | \(n_c\times1\) | 通过约束恢复的从自由度 |
| \(\boldsymbol G_m,\boldsymbol G_s\) | — | \(\boldsymbol G\) 按主、从自由度列分块后的矩阵 |

“主节点”和“从节点”是建模层面的称呼；论文消元真正操作的是“主自由度”和“从自由度”。一个节点可以有 3、4 或 6 个自由度，也可以只把其中若干自由度选为从自由度。

最重要的计数关系是：

\[
\boxed{\text{每个独立标量约束必须对应一个从自由度}}
\]

因此，若一个 MPC 条目含有 3 个标量方程，就必须选 3 个互不重复的从自由度。`SlaveDirection=012` 的含义应当是选择三个平动从自由度，而不是“只有一条约束”。

---

### 3.2 从无约束静力平衡开始

#### 3.2.1 势能驻值和残差

论文式 (1)–(3)从无约束最小化问题出发：

\[
\min_{\boldsymbol V} W(\boldsymbol V).
\]

平衡状态满足势能的一阶导数为零：

\[
\boldsymbol R(\boldsymbol V)
:=\frac{\partial W}{\partial\boldsymbol V}
=\boldsymbol0.
\]

在有限元静力问题中，可把它理解成

\[
\boldsymbol R=\boldsymbol F_{\mathrm{int}}-\boldsymbol F_{\mathrm{ext}}.
\]

在第 \(k\) 次 Newton 迭代的当前状态 \(\boldsymbol V^{(k)}\) 附近作一阶展开：

\[
\boldsymbol R(\boldsymbol V^{(k)}+\Delta\boldsymbol V)
\approx
\boldsymbol R^{(k)}
+\boldsymbol K_T^{(k)}\Delta\boldsymbol V,
\]

其中

\[
\boldsymbol K_T
:=\frac{\partial\boldsymbol R}{\partial\boldsymbol V}
=\frac{\partial^2W}{\partial\boldsymbol V^2}.
\]

令线性化后的残差为零，得到普通非线性静力 Newton 方程：

\[
\boxed{\boldsymbol K_T\Delta\boldsymbol V=-\boldsymbol R.}
\]

#### 3.2.2 加入非线性约束

现在要求结构不仅满足平衡，还满足论文式 (4)：

\[
\boxed{\boldsymbol c(\boldsymbol V)=\boldsymbol0.}
\]

例如：

- 定长：\(\|\boldsymbol x_a-\boldsymbol x_b\|-L_0=0\)；
- 刚性偏置：\(\boldsymbol x_s-\boldsymbol x_m-\boldsymbol R_m\boldsymbol r_0=\boldsymbol0\)；
- 平动绑定：\(\boldsymbol u_s-\boldsymbol u_m=\boldsymbol0\)；
- 转角相等：\(\varphi_s-\varphi_m=0\)。

它们的物理含义不同，但都可统一写成 \(\boldsymbol c(\boldsymbol V)=\boldsymbol0\)。

#### 3.2.3 论文要求的三个前提

论文第 2 节给出了方法成立所需的假设：

1. **相容性**：存在同时满足全部 \(c_i=0\) 的状态，约束之间不能互相矛盾。
2. **光滑性**：每个 \(c_i\) 至少二次连续可微，使 \(\boldsymbol G\) 和 \(\boldsymbol H_i\) 存在，能够进行一致 Newton 线性化。
3. **独立性**：\(\boldsymbol G\) 满行秩，即

   \[
   \operatorname{rank}(\boldsymbol G)=n_c.
   \]

第三条意味着约束中不能存在重复方程。例如既设置 \(u_s-u_m=0\)，又以另一 MPC 重复设置完全相同的关系，会使约束 Jacobian 行相关。

---

### 3.3 主从自由度划分

论文式 (21)将未知量重新排列为

\[
\boldsymbol V=
\begin{bmatrix}
\boldsymbol V_m\\
\boldsymbol V_s
\end{bmatrix},
\qquad
\boldsymbol V_m\in\mathbb R^{n_{\mathrm{dof}}-n_c},
\quad
\boldsymbol V_s\in\mathbb R^{n_c}.
\]

对应地，结构残差和约束 Jacobian 分块为

\[
\boldsymbol R=
\begin{bmatrix}\boldsymbol R_m\\\boldsymbol R_s\end{bmatrix},
\qquad
\boldsymbol G=
\frac{\partial\boldsymbol c}{\partial\boldsymbol V}
=
\begin{bmatrix}\boldsymbol G_m&\boldsymbol G_s\end{bmatrix},
\]

其中

\[
\boldsymbol G_m=\frac{\partial\boldsymbol c}{\partial\boldsymbol V_m},
\qquad
\boldsymbol G_s=\frac{\partial\boldsymbol c}{\partial\boldsymbol V_s}.
\]

主从消元要求论文式 (23)：

\[
\boxed{\det(\boldsymbol G_s)\ne0.}
\]

这并不要求整个 \(\boldsymbol G\) 是方阵，而是要求从 \(\boldsymbol G\) 中挑出的 \(n_c\) 个从自由度列构成可逆的 \(n_c\times n_c\) 子矩阵。

这个条件解释了为什么“指定了从节点”仍可能无法消元：若选择的从方向在当前构形中对约束没有一阶影响，对应的 \(\boldsymbol G_s\) 就可能为零或奇异。比如二维定长约束在连线恰好水平时，若选择竖向位移作为唯一从自由度，则该点的长度对竖向位移的一阶导数为零，当前迭代点不能用这个方向作隐函数消元。

---

### 3.4 为什么从自由度能够由主自由度确定

若 \(\boldsymbol c\) 足够光滑且 \(\boldsymbol G_s\) 可逆，根据隐函数定理，论文式 (24)说明在当前状态附近存在函数

\[
\boldsymbol V_s=\boldsymbol c_{\mathrm{mod}}(\boldsymbol V_m),
\]

使得

\[
\boldsymbol c(\boldsymbol V_m,\boldsymbol V_s)=\boldsymbol0.
\]

这就是“主节点计算以后，从节点可以得到位移”的严格数学依据。不过应当更准确地说：

> 求解器计算主自由度增量，再通过线性化约束恢复从自由度增量；随后更新全部节点，并在下一次 Newton 迭代重新计算非线性约束。

对恒等式

\[
\boldsymbol c\bigl(\boldsymbol V_m,
\boldsymbol c_{\mathrm{mod}}(\boldsymbol V_m)\bigr)
=\boldsymbol0
\]

关于 \(\boldsymbol V_m\) 求导：

\[
\boldsymbol G_m
+\boldsymbol G_s
\frac{\partial\boldsymbol V_s}{\partial\boldsymbol V_m}
=\boldsymbol0.
\]

左乘 \(\boldsymbol G_s^{-1}\)，得到论文式 (27)：

\[
\boxed{
\frac{\partial\boldsymbol V_s}{\partial\boldsymbol V_m}
=-\boldsymbol G_s^{-1}\boldsymbol G_m.}
\]

定义

\[
\boldsymbol A:=-\boldsymbol G_s^{-1}\boldsymbol G_m,
\]

则 \(\boldsymbol A\) 表示从自由度对主自由度的局部变化率。

---

### 3.5 修改后的平衡残差

把 \(\boldsymbol V_s(\boldsymbol V_m)\) 代入势能，得到只依赖主自由度的势能

\[
W_{\mathrm{mod}}(\boldsymbol V_m)
=W\bigl(\boldsymbol V_m,\boldsymbol V_s(\boldsymbol V_m)\bigr).
\]

对主自由度求导，并使用链式法则：

\[
\frac{\partial W_{\mathrm{mod}}}{\partial\boldsymbol V_m}
=\frac{\partial W}{\partial\boldsymbol V_m}
+\left(\frac{\partial\boldsymbol V_s}{\partial\boldsymbol V_m}\right)^T
\frac{\partial W}{\partial\boldsymbol V_s}.
\]

代入 \(\boldsymbol R_m\)、\(\boldsymbol R_s\) 和式 (27)：

\[
\boxed{
\boldsymbol R_{\mathrm{mod},m}
=\boldsymbol R_m
-\boldsymbol G_m^T\boldsymbol G_s^{-T}\boldsymbol R_s.}
\]

这就是论文式 (28)。它不是简单地删除从自由度残差。第二项把从自由度上的不平衡通过约束关系传递给主自由度。

为简化后续表达，论文定义

\[
\boxed{\boldsymbol Y:=\boldsymbol G_s^{-T}\boldsymbol R_s.}
\]

于是

\[
\boldsymbol R_{\mathrm{mod},m}
=\boldsymbol R_m-\boldsymbol G_m^T\boldsymbol Y.
\]

完整的修改方程采用论文式 (29)：

\[
\boxed{
\begin{bmatrix}
\boldsymbol R_{\mathrm{mod},m}\\
\boldsymbol R_{\mathrm{mod},s}
\end{bmatrix}
=
\begin{bmatrix}
\boldsymbol R_m-\boldsymbol G_m^T\boldsymbol G_s^{-T}\boldsymbol R_s\\
\boldsymbol c
\end{bmatrix}
=\boldsymbol0.}
\]

注意下方方程已不再是原来的 \(\boldsymbol R_s=0\)，而是约束方程 \(\boldsymbol c=0\)。这一步正是用几何约束替换从自由度平衡方程。

论文式 (30)还给出约束力

\[
\boldsymbol C
=\boldsymbol G^T\boldsymbol G_s^{-T}\boldsymbol R_s
=\boldsymbol G^T\boldsymbol Y.
\]

因此 \(\boldsymbol Y\)在数学角色上等价于约束乘子；主从消元并未把约束力“丢掉”，只是没有把乘子作为额外未知量加入线性方程。

---

### 3.6 对修改残差作一致线性化

这是非线性 MPC 最关键、也最容易漏项的部分。

#### 3.6.1 分块记号

论文式 (31)定义

\[
\boldsymbol K_{\alpha\beta}
:=\frac{\partial\boldsymbol R_\alpha}
{\partial\boldsymbol V_\beta},
\qquad
\boldsymbol H_{\alpha\beta,i}
:=\frac{\partial^2c_i}
{\partial\boldsymbol V_\alpha\partial\boldsymbol V_\beta},
\]

其中 \(\alpha,\beta\in\{m,s\}\)。每个 \(\boldsymbol H_i\) 都可以按主从自由度分成四块。

#### 3.6.2 为什么会出现约束 Hessian

修改后的主残差为

\[
\boldsymbol R_{\mathrm{mod},m}
=\boldsymbol R_m-\boldsymbol G_m^T\boldsymbol Y.
\]

对任一分块变量 \(\boldsymbol V_\beta\) 求导：

\[
\frac{\partial\boldsymbol R_{\mathrm{mod},m}}
{\partial\boldsymbol V_\beta}
=\boldsymbol K_{m\beta}
-\frac{\partial\boldsymbol G_m^T}{\partial\boldsymbol V_\beta}
\boldsymbol Y
-\boldsymbol G_m^T
\frac{\partial\boldsymbol Y}{\partial\boldsymbol V_\beta}.
\]

其中 \(\partial\boldsymbol G/\partial\boldsymbol V\) 正是约束的二阶导数，因此非线性约束必然产生 Hessian 项。若省略它，得到的不是论文的一致切线，通常只剩割线式迭代，二次收敛性会受到破坏。

又因为

\[
\boldsymbol Y=\boldsymbol G_s^{-T}\boldsymbol R_s,
\]

需要使用逆矩阵微分公式

\[
d(\boldsymbol G_s^{-T})
=-\boldsymbol G_s^{-T}(d\boldsymbol G_s^T)
\boldsymbol G_s^{-T}.
\]

逐项代入并按每个标量约束展开，得到论文式 (33)：

\[
\boxed{
\begin{aligned}
\boldsymbol K_{\mathrm{mod},m\beta}
={}&\boldsymbol K_{m\beta}
-\sum_{i=1}^{n_c}\boldsymbol H_{m\beta,i}Y_i\\
&+\boldsymbol G_m^T\boldsymbol G_s^{-T}
\sum_{i=1}^{n_c}\boldsymbol H_{s\beta,i}Y_i\\
&-\boldsymbol G_m^T\boldsymbol G_s^{-T}
\boldsymbol K_{s\beta},
\qquad \beta\in\{m,s\}.
\end{aligned}}
\]

下方约束方程的导数直接为

\[
\boldsymbol K_{\mathrm{mod},s\beta}
=\frac{\partial\boldsymbol c}{\partial\boldsymbol V_\beta}
=\boldsymbol G_\beta.
\]

因此完整 Newton 线性化方程是论文式 (34)：

\[
\boxed{
\begin{bmatrix}
\boldsymbol K_{\mathrm{mod},mm}&
\boldsymbol K_{\mathrm{mod},ms}\\
\boldsymbol G_m&\boldsymbol G_s
\end{bmatrix}
\begin{bmatrix}
\Delta\boldsymbol V_m\\
\Delta\boldsymbol V_s
\end{bmatrix}
=
\begin{bmatrix}
-\boldsymbol R_{\mathrm{mod},m}\\
-\boldsymbol c
\end{bmatrix}.}
\]

---

### 3.7 消去从自由度增量

#### 3.7.1 先线性化约束方程

约束在当前状态附近的一阶展开是

\[
\boldsymbol c
+\boldsymbol G_m\Delta\boldsymbol V_m
+\boldsymbol G_s\Delta\boldsymbol V_s
=\boldsymbol0.
\]

整理得到论文式 (35)：

\[
\boxed{
\Delta\boldsymbol V_s
=-\boldsymbol G_s^{-1}\boldsymbol c
-\boldsymbol G_s^{-1}\boldsymbol G_m
\Delta\boldsymbol V_m.}
\]

这个公式有两部分：

1. \(-\boldsymbol G_s^{-1}\boldsymbol c\)：当前状态若已违反约束，用它把状态拉回约束面；
2. \(-\boldsymbol G_s^{-1}\boldsymbol G_m\Delta\boldsymbol V_m\)：主自由度变化以后，从自由度必须随之变化。

所以不能只写 \(\Delta\boldsymbol V_s=\boldsymbol A\Delta\boldsymbol V_m\)。在 Newton 迭代尚未满足 \(\boldsymbol c=0\) 时，必须保留第一项。

#### 3.7.2 代回主方程

把上式代入式 (34) 第一行：

\[
\boldsymbol K_{\mathrm{mod},mm}\Delta\boldsymbol V_m
+\boldsymbol K_{\mathrm{mod},ms}
\left(
-\boldsymbol G_s^{-1}\boldsymbol c
-\boldsymbol G_s^{-1}\boldsymbol G_m\Delta\boldsymbol V_m
\right)
=-\boldsymbol R_{\mathrm{mod},m}.
\]

把含 \(\Delta\boldsymbol V_m\) 的项放在左边，其余项放在右边：

\[
\left(
\boldsymbol K_{\mathrm{mod},mm}
-\boldsymbol K_{\mathrm{mod},ms}
\boldsymbol G_s^{-1}\boldsymbol G_m
\right)\Delta\boldsymbol V_m
=-\boldsymbol R_{\mathrm{mod},m}
+\boldsymbol K_{\mathrm{mod},ms}
\boldsymbol G_s^{-1}\boldsymbol c.
\]

定义论文式 (36)–(37)中的缩减切线和缩减残差：

\[
\boxed{
\boldsymbol K_{\mathrm{red}}
=\boldsymbol K_{\mathrm{mod},mm}
-\boldsymbol K_{\mathrm{mod},ms}
\boldsymbol G_s^{-1}\boldsymbol G_m,}
\]

\[
\boxed{
\boldsymbol R_{\mathrm{red}}
=\boldsymbol R_{\mathrm{mod},m}
-\boldsymbol K_{\mathrm{mod},ms}
\boldsymbol G_s^{-1}\boldsymbol c.}
\]

最终只求解主自由度：

\[
\boxed{
\boldsymbol K_{\mathrm{red}}\Delta\boldsymbol V_m
=-\boldsymbol R_{\mathrm{red}}.}
\]

求出 \(\Delta\boldsymbol V_m\) 后，再用式 (35)恢复 \(\Delta\boldsymbol V_s\)。方程规模从 \(n_{\mathrm{dof}}\) 降为 \(n_{\mathrm{dof}}-n_c\)，没有像 Lagrange 乘子法那样增加未知量。

---

### 3.8 程序使用的等价投影形式

当前 `NonlinearMPC::Reduce` 没有逐块显式构造式 (33)中的四个块，而是采用与论文式 (35)–(37)完全等价、且更适合稀疏矩阵实现的形式。

定义从自由度的特解修正

\[
\boldsymbol b=-\boldsymbol G_s^{-1}\boldsymbol c,
\]

以及主自由度到完整自由度增量的变换矩阵

\[
\boldsymbol T=
\begin{bmatrix}
\boldsymbol I\\
-\boldsymbol G_s^{-1}\boldsymbol G_m
\end{bmatrix},
\qquad
\boldsymbol p=
\begin{bmatrix}
\boldsymbol0\\
\boldsymbol b
\end{bmatrix}.
\]

于是式 (35)可整体写为

\[
\boxed{
\Delta\boldsymbol V
=\boldsymbol p+\boldsymbol T\Delta\boldsymbol V_m.}
\]

定义包含约束曲率的一致切线

\[
\boxed{
\boldsymbol K_L
=\boldsymbol K_T
-\sum_{i=1}^{n_c}Y_i\boldsymbol H_i,}
\qquad
\boldsymbol Y=\boldsymbol G_s^{-T}\boldsymbol R_s.
\]

注意

\[
\boldsymbol T^T\boldsymbol R
=\boldsymbol R_m
-\boldsymbol G_m^T\boldsymbol G_s^{-T}\boldsymbol R_s
=\boldsymbol R_{\mathrm{mod},m},
\]

而论文式 (33)的第一块行可以紧凑地写成

\[
\begin{bmatrix}
\boldsymbol K_{\mathrm{mod},mm}&
\boldsymbol K_{\mathrm{mod},ms}
\end{bmatrix}
=\boldsymbol T^T\boldsymbol K_L.
\]

因此论文的缩减量等价于

\[
\boxed{
\boldsymbol K_{\mathrm{red}}
=\boldsymbol T^T\boldsymbol K_L\boldsymbol T,}
\]

\[
\boxed{
\boldsymbol R_{\mathrm{red}}
=\boldsymbol T^T\boldsymbol R
+\boldsymbol T^T\boldsymbol K_L\boldsymbol p.}
\]

于是求解

\[
\boldsymbol K_{\mathrm{red}}\Delta\boldsymbol V_m
=-\boldsymbol R_{\mathrm{red}},
\]

并通过 \(\Delta\boldsymbol V=\boldsymbol p+\boldsymbol T\Delta\boldsymbol V_m\) 恢复完整增量。这正是当前代码的矩阵运算顺序。

若无约束结构切线 \(\boldsymbol K_T\) 对称，且约束 Hessian 对称，则 \(\boldsymbol K_L\) 和 \(\boldsymbol T^T\boldsymbol K_L\boldsymbol T\) 也对称。这对应论文对缩减切线对称性的结论。

---

### 3.9 论文符号与当前程序符号的转换

论文使用

\[
\boldsymbol R=\boldsymbol F_{\mathrm{int}}-\boldsymbol F_{\mathrm{ext}},
\qquad
\boldsymbol K_T\Delta\boldsymbol V=-\boldsymbol R.
\]

当前静力求解器中的 `ComputeResidual` 返回

\[
\texttt{rhs}
=\boldsymbol F_{\mathrm{ext}}-\boldsymbol F_{\mathrm{int}}
=-\boldsymbol R.
\]

因此代码进入 `NonlinearMPC::Reduce` 后先执行概念上的转换

\[
\texttt{structuralResidual}=-\texttt{rhs}=\boldsymbol R.
\]

再完全按照论文定义计算 \(\boldsymbol Y\)、\(\boldsymbol K_L\) 和 \(\boldsymbol R_{\mathrm{red}}\)，最后把求解右端写成

\[
\texttt{reduction.rhs}=-\boldsymbol R_{\mathrm{red}}.
\]

因此代码中的负号并不是更改论文理论，而是适配现有求解器 `K*dq = Fext-Fint` 的历史约定。

---

### 3.10 每个载荷增量内的完整 Newton 流程

对第 \(n\) 个载荷增量，静力非线性 MPC 的计算顺序如下。

1. 施加当前增量的外载荷和给定位移。
2. 在当前构形组装无约束结构切线 \(\boldsymbol K_T\)。
3. 计算结构残差 \(\boldsymbol R=\boldsymbol F_{\mathrm{int}}-\boldsymbol F_{\mathrm{ext}}\)。
4. 对每一个当前分析步激活的 MPC，在当前构形计算：

   \[
   c_i,\qquad
   \boldsymbol G_i=\frac{\partial c_i}{\partial\boldsymbol V},
   \qquad
   \boldsymbol H_i=\frac{\partial^2c_i}{\partial\boldsymbol V^2}.
   \]

5. 把所有约束行装配为全局 \(\boldsymbol c\)、\(\boldsymbol G\) 和 \(\{\boldsymbol H_i\}\)。
6. 按指定的从自由度列构造 \(\boldsymbol G_s\)，其余列组成 \(\boldsymbol G_m\)。
7. 检查 \(\boldsymbol G_s\) 是否满秩；若奇异，当前主从选择不可用，不能继续消元。
8. 解三个小型约束系统：

   \[
   \boldsymbol G_s\boldsymbol A=-\boldsymbol G_m,
   \qquad
   \boldsymbol G_s\boldsymbol b=-\boldsymbol c,
   \qquad
   \boldsymbol G_s^T\boldsymbol Y=\boldsymbol R_s.
   \]

   实现中应使用矩阵分解和 `solve`，不应显式计算逆矩阵。

9. 构造 \(\boldsymbol T\)、\(\boldsymbol p\) 和

   \[
   \boldsymbol K_L=\boldsymbol K_T-\sum_iY_i\boldsymbol H_i.
   \]

10. 构造并求解缩减方程：

    \[
    \boldsymbol T^T\boldsymbol K_L\boldsymbol T
    \Delta\boldsymbol V_m
    =-
    \left(
    \boldsymbol T^T\boldsymbol R
    +\boldsymbol T^T\boldsymbol K_L\boldsymbol p
    \right).
    \]

11. 恢复完整增量：

    \[
    \Delta\boldsymbol V=\boldsymbol p+\boldsymbol T\Delta\boldsymbol V_m.
    \]

12. 更新平动和转动状态。若采用有限转动，旋转更新必须与约束 Jacobian/Hessian 所采用的转动增量定义一致。
13. 在更新后的构形重新计算结构量和约束量，不能沿用上一次迭代的 \(\boldsymbol G\) 或 \(\boldsymbol H_i\)。
14. 同时检查三类收敛条件：平衡残差、位移修正和约束残差 \(\|\boldsymbol c\|\)。三者均满足才可提交该载荷增量。

该流程说明 MPC 必须位于每次 Newton 迭代内部，而不是只在分析开始时处理一次。

---

### 3.11 多个约束的装配

论文第 5.2 节把约束看成类似有限元单元的“约束库”。第 \(i\) 个约束独立计算局部约束量：

\[
c_i,\qquad \boldsymbol G_i,\qquad \boldsymbol H_i.
\]

随后按论文式 (38)组装：

\[
\boldsymbol c=
\begin{bmatrix}c_1&\cdots&c_{n_c}\end{bmatrix}^T,
\qquad
\boldsymbol G=
\begin{bmatrix}
\boldsymbol G_1\\
\vdots\\
\boldsymbol G_{n_c}
\end{bmatrix}.
\]

Hessian 不能简单组装成一个固定矩阵，因为每个约束在一致切线中有不同权重。论文式 (39)使用有效 Hessian

\[
\widehat{\boldsymbol H}
=\sum_{i=1}^{n_c}Y_i\boldsymbol H_i.
\]

当前程序保留每一个稀疏 \(\boldsymbol H_i\)，得到 \(\boldsymbol Y\) 后再执行加权求和。

若不同 MPC 共享节点或自由度，它们会在全局 \(\boldsymbol G\) 中形成耦合块。此时不能只检查每条约束各自的导数，还必须检查组装后的全局 \(\boldsymbol G_s\) 是否可逆。论文第 5.3 节建议利用约束簇结构对 \(\boldsymbol G_s\) 分块求解，以降低计算成本；当前实现先保证统一的全局消元正确性。

---

### 3.12 定长约束：从约束值推到 Hessian

这是论文附录 D.1 的约束，也是首先需要复现的算例所使用的基本非线性关系。

#### 3.12.1 几何定义

设两点当前坐标为

\[
\boldsymbol x_a=\boldsymbol X_a+\boldsymbol u_a,
\qquad
\boldsymbol x_b=\boldsymbol X_b+\boldsymbol u_b,
\]

定义

\[
\boldsymbol d=\boldsymbol x_a-\boldsymbol x_b,
\qquad
\ell=\|\boldsymbol d\|,
\qquad
\boldsymbol n=\frac{\boldsymbol d}{\ell}.
\]

论文式 (59)采用

\[
c_{\mathrm{paper}}=L_0-\ell=0,
\qquad
L_0=\|\boldsymbol X_a-\boldsymbol X_b\|.
\]

当前程序采用等价的相反符号

\[
\boxed{c=\ell-L_0=0.}
\]

将一个约束方程整体乘以 \(-1\) 不改变约束面，但 \(c\)、\(\boldsymbol G\) 和 \(\boldsymbol H\) 必须同时反号，不能只改其中一个。

固定长度表示两点的欧氏距离不变，并不表示两点之间必须沿直线运动。如果点在圆弧上运动，只要两点弦长始终为 \(L_0\)，定长约束仍然成立；但仅凭一条定长约束不能唯一规定圆弧轨迹，还需要圆心、半径、平面或角度等其他约束。

#### 3.12.2 一阶导数

先求长度对 \(\boldsymbol d\) 的导数：

\[
d\ell
=d\sqrt{\boldsymbol d^T\boldsymbol d}
=\frac{\boldsymbol d^T}{\ell}\,d\boldsymbol d
=\boldsymbol n^T d\boldsymbol d.
\]

又因为

\[
d\boldsymbol d=d\boldsymbol u_a-d\boldsymbol u_b,
\]

所以程序符号下的 Jacobian 为

\[
\boxed{
\boldsymbol G
=\begin{bmatrix}\boldsymbol n^T&-\boldsymbol n^T\end{bmatrix}.}
\]

论文式 (60)对应的是上述矩阵的相反数，因为论文采用 \(L_0-\ell\)。

#### 3.12.3 二阶导数

由 \(\boldsymbol n=\boldsymbol d/\ell\)，有

\[
d\boldsymbol n
=\frac{1}{\ell}d\boldsymbol d
-\frac{\boldsymbol d}{\ell^2}d\ell.
\]

代入 \(d\ell=\boldsymbol n^Td\boldsymbol d\) 和 \(\boldsymbol d=\ell\boldsymbol n\)：

\[
d\boldsymbol n
=\frac{1}{\ell}
\left(\boldsymbol I-\boldsymbol n\boldsymbol n^T\right)
d\boldsymbol d.
\]

定义曲率矩阵

\[
\boldsymbol Q
:=\frac{1}{\ell}
\left(\boldsymbol I-\boldsymbol n\boldsymbol n^T\right).
\]

则程序符号下的 Hessian 为

\[
\boxed{
\boldsymbol H=
\begin{bmatrix}
\boldsymbol Q&-\boldsymbol Q\\
-\boldsymbol Q&\boldsymbol Q
\end{bmatrix}.}
\]

把 \(\boldsymbol Q\) 展开，可以得到论文式 (61)的相反数；两者的差别仍只来自约束定义的整体符号。

当 \(\ell\to0\) 时，\(\boldsymbol n\) 和 \(\boldsymbol Q\) 都无定义，所以重合点不能直接使用这种定长表达式。程序必须对此作退化检查。

#### 3.12.4 从自由度的选择

定长关系只有一个标量方程，所以只消去一个从自由度。假设选 \(u_{b,x}\) 为从自由度，则

\[
\boldsymbol G_s=\begin{bmatrix}-n_x\end{bmatrix}.
\]

只有 \(|n_x|>0\) 时该选择才可逆。若运动过程中 \(n_x\) 接近零，应选择导数绝对值更大的方向，或采用自动主元选择。这个问题与“节点是主还是从”无关，本质是所选从自由度列能否局部参数化约束面。

---

### 3.13 论文二维非线性力释放约束

论文附录 D.2 的式 (62)为

\[
c=
\boldsymbol n_i(\varphi_i)^T
\begin{bmatrix}u_i\\w_i\end{bmatrix}
-
\boldsymbol n_k(\varphi_k)^T
\begin{bmatrix}u_k\\w_k\end{bmatrix}
=0,
\]

其中

\[
\boldsymbol n(\varphi)=
\begin{bmatrix}\cos\varphi\\\sin\varphi\end{bmatrix}.
\]

写开后：

\[
c=
\cos\varphi_i\,u_i+
\sin\varphi_i\,w_i-
\cos\varphi_k\,u_k-
\sin\varphi_k\,w_k.
\]

若自由度顺序为

\[
\begin{bmatrix}
u_i&w_i&\varphi_i&u_k&w_k&\varphi_k
\end{bmatrix}^T,
\]

逐项求一阶导数，得到论文式 (63)：

\[
\boldsymbol G=
\begin{bmatrix}
\cos\varphi_i&
\sin\varphi_i&
-\sin\varphi_i u_i+\cos\varphi_i w_i&
-\cos\varphi_k&
-\sin\varphi_k&
\sin\varphi_k u_k-\cos\varphi_k w_k
\end{bmatrix}.
\]

再对上述各分量求导，可得到论文式 (64)的对称 Hessian。非零项包括平动—转动耦合项以及两个转动—转动项。例如

\[
\frac{\partial^2c}{\partial u_i\partial\varphi_i}
=-\sin\varphi_i,
\qquad
\frac{\partial^2c}{\partial w_i\partial\varphi_i}
=\cos\varphi_i,
\]

\[
\frac{\partial^2c}{\partial\varphi_i^2}
=-\cos\varphi_i u_i-\sin\varphi_i w_i,
\]

\[
\frac{\partial^2c}{\partial u_k\partial\varphi_k}
=\sin\varphi_k,
\qquad
\frac{\partial^2c}{\partial w_k\partial\varphi_k}
=-\cos\varphi_k,
\]

\[
\frac{\partial^2c}{\partial\varphi_k^2}
=\cos\varphi_k u_k+\sin\varphi_k w_k.
\]

当前 `PlanarShearReleaseMPCConstraint` 将这个非线性方程与转角相等方程组合为两个标量约束。前者有非零 Hessian，后者在线性化所用的相对转角坐标下 Hessian 为零。为避免梁转过 \(90^\circ\) 或 \(270^\circ\) 时固定选择的平动列失去主元，程序在从节点 \(x/y\) 两个平动方向中选择当前 Jacobian 绝对值较大的方向。这是保证论文条件 \(\det\boldsymbol G_s\ne0\) 的数值实现措施。

---

### 3.14 刚性偏置关系如何进入同一框架

三维大转动刚性偏置可写成

\[
\boxed{
\boldsymbol c
=\boldsymbol x_s-\boldsymbol x_m
-\boldsymbol R_m\boldsymbol r_0
=\boldsymbol0,}
\]

其中 \(\boldsymbol r_0\) 是主节点局部参考系中的初始偏置，\(\boldsymbol R_m\) 是主节点当前旋转矩阵。该式含 3 个标量约束，因此通常消去从节点的 3 个平动自由度。

它与定长约束使用完全相同的消元器；差别只在该约束类提供的 \(\boldsymbol c\)、\(\boldsymbol G\) 和 \(\boldsymbol H_i\)。大转动情况下，\(\boldsymbol R_m\boldsymbol r_0\) 对转动增量是非线性的，因此必须提供与程序旋转更新方式一致的转动 Jacobian 和 Hessian。

这个关系也说明：若间隔棒只负责保持导线连接点相对刚性位置，可用梁节点作为主节点、导线节点作为从节点，用刚性偏置 MPC 传递主节点平移和转动对从节点位置的影响。导线节点额外的扭转自由度是否绑定，应由物理连接形式另行定义，不能因为节点具有该自由度就自动绑定。

---

### 3.15 边界条件与 MPC 的先后顺序

论文第 5.1 节特别强调，边界条件的处理顺序会影响正确性。

#### 3.15.1 Neumann 边界条件

力边界条件先进入完整结构残差。它可以施加在参与约束的任意主或从自由度上，因为 \(\boldsymbol R_s\) 会通过

\[
-\boldsymbol G_m^T\boldsymbol G_s^{-T}\boldsymbol R_s
\]

传递到缩减主方程。

#### 3.15.2 Dirichlet 边界条件

若采用常规的行列消除法处理给定位移，则给定位移自由度不应再被选成 MPC 从自由度，因为 MPC 消元以后从自由度不在缩减系统中。

论文给出的顺序为：

1. 建立无约束问题的残差和切线；
2. 加入 Neumann 边界条件；
3. 选择主从自由度，受 Dirichlet 条件控制的自由度不得选为从自由度；
4. 根据论文式 (36)形成 MPC 缩减系统；
5. 在缩减系统的主自由度上处理 Dirichlet 边界条件；
6. 求解缩减系统。

当前程序先从全局编号中移除固定自由度，然后约束对象只对剩余自由度组装 \(\boldsymbol c\)、\(\boldsymbol G\) 和 \(\boldsymbol H_i\)。因此指定为从自由度的方向必须在当前分析步是自由自由度，否则 `slaveDof` 无效，约束装配会失败。建模层应在求解前报告这类冲突。

---

### 3.16 计算区域与 MPC 完整性

MPC 是模型拓扑关系，不是某个计算区域临时生成的单元。若区域内包含一个 MPC 的主节点，但没有包含它所依赖的从节点，直接丢弃从节点会使约束关系残缺，无法计算该约束的 \(\boldsymbol c\)、\(\boldsymbol G\) 和 \(\boldsymbol H_i\)。

因此区域生成应执行约束闭包：

1. 先按用户选择得到初始节点和单元集合；
2. 查找与集合内任一节点相关的激活 MPC；
3. 把该 MPC 所需的全部主、从节点加入区域；
4. 新加入节点可能又关联其他 MPC，重复检查直至集合不再扩大；
5. 复制区域模型时同时复制 MPC 对象，并把节点引用重新绑定到区域内节点；
6. MPC 只有在分析步作用域不激活时才不参与本步装配，不能因区域裁剪而被静默删除。

这保证了同一个模型无论是否划分计算区域，原有约束拓扑始终存在。

---

### 3.17 当前代码与论文公式的对应关系

| 理论量/步骤 | 当前代码位置 | 说明 |
|---|---|---|
| 单条约束的 \(c_i,G_i,H_i\) | `DataStructure/Constraint/NonlinearMPCConstraint.cpp` | 各具体约束类的 `Evaluate` |
| 全局式 (38)约束装配 | `DataStructure/AnalysisStep/AnalysisStep.cpp` | `AssembleNonlinearMPC` 按行合并约束 |
| \(G_m,G_s\) | `Solver/Constraint/NonlinearMPC.cpp` | 按 `slaveDofs` 取列，其余列为主自由度 |
| 式 (35) | `NonlinearMPC::Reduce` | `slaveParticular` 和 `slaveFromMaster` |
| \(Y=G_s^{-T}R_s\) | `NonlinearMPC::Reduce` | `multipliers` |
| 式 (39)有效 Hessian | `NonlinearMPC::Reduce` | `lagrangianTangent -= multiplier[i] * hessians[i]` |
| 式 (36)–(37)缩减系统 | `NonlinearMPC::Reduce` | `T.transpose() * K_L * T` 及缩减右端 |
| 恢复完整增量 | `NonlinearMPCReduction::RecoverFullIncrement` | \(p+T\Delta V_m\) |
| 每次 Newton 迭代重新装配 | `Solver/Static/SolverStatic.cpp` | 结构矩阵、残差和 MPC 均在迭代循环内更新 |
| 约束收敛检查 | `Solver/Static/SolverStatic.cpp` | 使用 \(\|c\|\) 与 `tol_C` 比较 |

程序数据结构的对应关系为：

| 程序成员 | 论文符号 |
|---|---|
| `NonlinearMPCData::value` | \(\boldsymbol c\) |
| `NonlinearMPCData::jacobian` | \(\boldsymbol G\) |
| `NonlinearMPCData::hessians[i]` | \(\boldsymbol H_i\) |
| `NonlinearMPCData::slaveDofs` | 组成 \(\boldsymbol V_s\) 的全局自由度编号 |
| `masterTransformation` | \(\boldsymbol T\) |
| `particularCorrection` | \(\boldsymbol p\) |
| `multipliers` | \(\boldsymbol Y\) |
| `reduction.tangent` | \(\boldsymbol K_{\mathrm{red}}\) |
| `reduction.rhs` | \(-\boldsymbol R_{\mathrm{red}}\) |

---

### 3.18 新增一种非线性 MPC 时的推导模板

以后新增约束时，应按以下顺序推导，而不是直接猜 Jacobian。

#### 第一步：写当前构形中的物理关系

明确约束使用参考坐标、当前坐标、位移、旋转矩阵还是相对转角。例如

\[
c_i(\boldsymbol V)=0.
\]

先确认该方程确实表达所需物理关系，并检查量纲。

#### 第二步：确定标量方程数

一个向量等式要拆成若干标量方程。方程数就是要消去的从自由度数。

#### 第三步：解析求 Jacobian

\[
\boldsymbol G_i=\frac{\partial c_i}{\partial\boldsymbol V}.
\]

只对约束实际涉及的自由度填非零项，再装配到当前自由度编号。

#### 第四步：解析求 Hessian

\[
\boldsymbol H_i=\frac{\partial^2c_i}{\partial\boldsymbol V^2}.
\]

对于光滑标量约束，解析 Hessian 应对称。若明显不对称，通常意味着漏项、符号错误，或转动参数化与增量定义不一致。

#### 第五步：选择从自由度

从 \(\boldsymbol G\) 中选择一组使 \(\boldsymbol G_s\) 满秩的列。应避开：

- 已施加 Dirichlet 条件的自由度；
- 当前导数为零或很小的方向；
- 已被其他标量约束选作从自由度的方向；
- 物理上不应由该关系支配的自由度。

#### 第六步：做数值导数校验

对随机但合理的状态 \(\boldsymbol V\)，采用中心差分检查

\[
\boldsymbol G_{:,j}^{\mathrm{FD}}
\approx
\frac{\boldsymbol c(\boldsymbol V+h\boldsymbol e_j)
-\boldsymbol c(\boldsymbol V-h\boldsymbol e_j)}{2h},
\]

并用 Jacobian 的差分检查 Hessian：

\[
\boldsymbol H_{i,:,j}^{\mathrm{FD}}
\approx
\frac{\boldsymbol G_i(\boldsymbol V+h\boldsymbol e_j)
-\boldsymbol G_i(\boldsymbol V-h\boldsymbol e_j)}{2h}.
\]

有限转动问题不能随意直接加减转角分量；差分扰动必须调用与求解器相同的旋转更新操作。

#### 第七步：分别验证约束和结构求解

推荐依次检查：

1. 单条约束的 \(c/G/H\) 数值导数；
2. \(G_s\) 的秩和条件数；
3. 恢复增量是否满足

   \[
   \boldsymbol c+\boldsymbol G\Delta\boldsymbol V\approx\boldsymbol0;
   \]

4. 缩减切线是否对称；
5. Newton 迭代是否呈现预期收敛；
6. 最终约束残差、位移、内力和反力是否与论文结果一致。

---

### 3.19 复现论文时必须避免的错误

1. **把非线性约束当成固定线性变换。** \(\boldsymbol G\)、\(\boldsymbol H_i\)、\(\boldsymbol T\) 都应随当前构形更新。
2. **遗漏 \(\boldsymbol H_i\)。** 这会偏离论文式 (33)/(37)的一致切线。
3. **只恢复耦合增量，遗漏 \(-G_s^{-1}c\)。** 当前约束误差将无法被主动修正。
4. **直接求逆。** 论文写逆矩阵用于推导，程序应通过分解求解线性方程。
5. **只检查 \(\boldsymbol G\) 满秩，不检查所选 \(\boldsymbol G_s\)。** 主从消元真正要求的是 \(\boldsymbol G_s\) 可逆。
6. **一个从自由度对应多条约束。** 这会使从自由度集合重复，无法形成正确的方阵 \(\boldsymbol G_s\)。
7. **约束与边界条件矛盾。** 数学上不存在满足全部条件的解。
8. **混用论文残差和程序右端符号。** 论文 \(R=F_{int}-F_{ext}\)，当前程序 `rhs=Fext-Fint`。
9. **只比较位移，不检查约束残差。** 位移看似合理并不能证明 MPC 被准确满足。
10. **在大转动中使用小转角偏置公式。** 这会使刚性偏置在转角增大后产生系统误差。

---

### 3.20 公式索引

| 本文内容 | 论文公式 |
|---|---:|
| 无约束势能、残差、Newton 方程 | (1)–(3) |
| 非线性约束与导数 | (4)–(6) |
| 主从自由度划分 | (21)–(22) |
| \(G_s\) 可逆条件 | (23) |
| 隐函数关系 | (24) |
| 从自由度对主自由度导数 | (27) |
| 修改后的主残差 | (28) |
| 完整修改残差与约束力 | (29)–(30) |
| 一致线性化切线 | (31)–(34) |
| 从自由度增量恢复 | (35) |
| 缩减方程 | (36)–(37) |
| 多约束装配和有效 Hessian | (38)–(39) |
| 定长约束 | (59)–(61) |
| 二维非线性力释放 | (62)–(64) |

---

静力推导的计算闭环为：

\[
\boxed{
\boldsymbol c
\longrightarrow
\boldsymbol G,\boldsymbol H_i
\longrightarrow
\boldsymbol G_s^{-1}
\longrightarrow
\boldsymbol T,\boldsymbol p,\boldsymbol Y
\longrightarrow
\boldsymbol K_{\mathrm{red}},\boldsymbol R_{\mathrm{red}}
\longrightarrow
\Delta\boldsymbol V_m
\longrightarrow
\Delta\boldsymbol V_s.}
\]

## 4. 从静力约束到连续动力

### 4.1 连续动力平衡

无约束半离散动力方程统一写成

$$
\mathbf R_d(\mathbf q,\dot{\mathbf q},\ddot{\mathbf q},t)
=\mathbf f_{\mathrm{int}}+\mathbf f_{\mathrm{ine}}
+\mathbf f_{\mathrm{damp}}-\mathbf f_{\mathrm{ext}}=\mathbf0.
$$

大转动梁中，$\mathbf f_{\mathrm{ine}}$ 不一定只是常质量矩阵乘加速度，还可含陀螺项和与当前构型有关的惯性项。

加入理想完整约束：

$$
\boxed{
\mathbf R_d-\mathbf G^T\boldsymbol\lambda=\mathbf0,
\qquad \mathbf c(\mathbf q)=\mathbf0.
}
$$

这是一组指标较高的微分—代数方程，而不是普通常微分方程。

### 4.2 位置、速度、加速度约束

对不显含时间的完整约束求一次时间导数：

$$
\boxed{\mathbf g_v=\mathbf G\dot{\mathbf q}=\mathbf0.}
$$

再求一次：

$$
\boxed{
\mathbf g_a=\mathbf G\ddot{\mathbf q}+\mathbf h=\mathbf0,
}
$$

其中第 $\alpha$ 个分量为

$$
\boxed{
h_\alpha=\dot{\mathbf q}^{T}\mathbf H_\alpha\dot{\mathbf q}.
}
$$

所以 $\mathbf H_\alpha$ 在动力中有双重作用：一是平衡方程一致切线中的约束几何刚度，二是加速度约束中的二次速度项。

### 4.3 主从速度和加速度

速度关系为

$$
\boxed{
\dot{\mathbf q}_s=-\mathbf G_s^{-1}\mathbf G_m
\dot{\mathbf q}_m.
}
$$

加速度关系为

$$
\boxed{
\ddot{\mathbf q}_s=-\mathbf G_s^{-1}
\left(\mathbf G_m\ddot{\mathbf q}_m+\mathbf h\right).
}
$$

注意加速度关系不是简单地把速度 transformation 再乘一次；$\mathbf h$ 正是构型相关 transformation 的时间变化。

### 4.4 刚性偏置的直观检查

设

$$
\mathbf x_s=\mathbf x_m+\mathbf R_m\mathbf r^0,
\qquad \boldsymbol\rho=\mathbf R_m\mathbf r^0.
$$

则

$$
\boxed{
\mathbf v_s=\mathbf v_m+\boldsymbol\omega_m\times\boldsymbol\rho
}
$$

和

$$
\boxed{
\mathbf a_s=\mathbf a_m
+\boldsymbol\alpha_m\times\boldsymbol\rho
+\boldsymbol\omega_m\times
(\boldsymbol\omega_m\times\boldsymbol\rho).
}
$$

最后一项就是 $\mathbf h$ 在刚性偏置约束中的物理表现。任何动力刚性 MPC 若不能恢复该项，都是不完整的。

若还要求外围梁节点的完整姿态保持刚性关系，运动学需另满足

$$
\mathbf R_i=\mathbf R_m\mathbf A_i^0,
$$

其中 $\mathbf A_i^0$ 为参考相对姿态。它不是三条平动偏置约束自动包含的关系；索的标量扭转应由专门的轴向扭转约束处理。有限转动更新使用

$$
\mathbf R^{n+1}=\exp([\Delta\boldsymbol\theta]_\times)\mathbf R^n,
$$

而非直接相加绝对转角；角速度和角加速度仍须与时间离散的旋转增量定义一致。

---

## 5. 时间离散、三层运动学与当前实现

### 5.1 为什么要“先离散、后消元”

不能只把连续方程中的 $\mathbf K$ 换成 $\mathbf K+\mathbf M$。正确顺序是：

1. 在 $t_{n+1}$ 建立 Newmark 预测状态；
2. 把速度和加速度表示成当前 Newton 构型修正的函数；
3. 形成完整的时间离散动力残量及其一致有效切线；
4. 在这个离散系统上评价 $\mathbf c,\mathbf G,\mathbf H$ 并消元。

这一顺序与离散零空间方法强调的先离散后消元具有联系，但当前构形 Jacobian 主从变换不等于 Betsch 的离散零空间矩阵，不能据此继承其专用离散格式的能量守恒结论。

### 5.2 平动 Newmark 关系

$$
\mathbf q_{n+1}^{\mathrm{pred}}
=\mathbf q_n+\Delta t\dot{\mathbf q}_n
+\Delta t^2\left(\frac12-\beta\right)\ddot{\mathbf q}_n,
$$

$$
\dot{\mathbf q}_{n+1}^{\mathrm{pred}}
=\dot{\mathbf q}_n
+\Delta t(1-\gamma)\ddot{\mathbf q}_n.
$$

对 Newton 构型修正 $\Delta\mathbf q$：

$$
\delta\ddot{\mathbf q}=a_0\Delta\mathbf q,
\qquad a_0=\frac1{\beta\Delta t^2},
$$

$$
\delta\dot{\mathbf q}=a_1\Delta\mathbf q,
\qquad a_1=\frac\gamma{\beta\Delta t}.
$$

这些关系用于生成 Newton 试探状态。存在非线性 MPC 时，主自由度仍遵循 Newmark；从自由度的
试探速度和加速度在动力残量评估前，按当前 $\mathbf G$ 与 $\mathbf H$ 修正为满足
$\mathbf G\dot{\mathbf q}=0$、$\mathbf G\ddot{\mathbf q}+\mathbf h=0$ 的值。

六自由度节点的转动不能直接相加。程序使用指数映射更新 $\mathbf R$，并把空间旋转修正经 $\mathbf T_s^{-1}$ 转为步初材料增量转角，再更新角速度和角加速度。

### 5.3 动力有效切线

当前程序组装：

$$
\boxed{
\mathbf K_{\mathrm{eff}}
=\mathbf K_t+a_0\mathbf M+a_1\mathbf C_g+\mathbf K_c.
}
$$

这里 $\mathbf C_g$ 表示速度/陀螺切线，$\mathbf K_c$ 表示构型相关惯性切线。它们由各动力单元返回。求解器还扣除外载切线；结构阻尼可通过矩阵或低秩因子单独加入。因此对常质量、线性黏性阻尼的普通平动模型，实际等价切线为 $K_{\mathrm{int}}-K_{\mathrm{ext}}+a_0M+a_1C$，不能遗漏阻尼或随动力导数。

### 5.4 对完整离散动力系统做 Boungard 型消元

在每次动力 Newton 迭代，把静力推导中的

$$
\mathbf R\longrightarrow\mathbf R_d,
\qquad
\mathbf K\longrightarrow\mathbf K_{\mathrm{eff}}.
$$

于是

$$
\boldsymbol\lambda=\mathbf G_s^{-T}(\mathbf R_d)_s,
$$

$$
\mathbf K_{L,d}
=\mathbf K_{\mathrm{eff}}
-\sum_\alpha\lambda_\alpha\mathbf H_\alpha,
$$

$$
\boxed{
\mathbf K_{d,\mathrm{red}}
=\mathbf T^T\mathbf K_{L,d}\mathbf T,
}
$$

$$
\boxed{
\mathbf b_{d,\mathrm{red}}
=-\mathbf T^T\mathbf R_d
-\mathbf T^T\mathbf K_{L,d}\mathbf p.
}
$$

以上为带仿射约束修正 $\mathbf p$ 的一般消元关系，仍由共享 `NonlinearMPC::Reduce` 提供。
2026-10-26 修复后的 Newmark 采用如下专门路径：先固定本步从自由度，迭代解
$\mathbf G_s\delta\mathbf q_s=-\mathbf c$，把试探位置回缩至约束曲面；再投影从速度、从加速度，
最后在这个完整状态上组装残量。主变量修正使用

$$
\mathbf K_{d,\mathrm{red}}\Delta\mathbf q_m=-\mathbf T^T\mathbf R_d,
\qquad \Delta\mathbf q=\mathbf T\Delta\mathbf q_m.
$$

线搜索的动力指标是 $\|\mathbf T^T\mathbf R_d\|$。每个试探重新进行位置回缩和两层运动学投影，
不再把已单独处理的几何修正 $\mathbf p$ 再经未投影的有效切线加入动力右端项。
代码中 `m_R=-R_d`，因此实际右端为 `T.transpose()*m_R`。
共享消元、静力及 TSSBN 阶段公式不因这一 Newmark 专用处理而改变。

由于投影使速度、加速度对位移的导数不同于原始 $a_0,a_1$，这里保留的降阶切线仍是拟 Newton 切线；
每次迭代及线搜索都重新计算真实动力残量，不能将它称为投影系统的精确一致导数。

---

### 5.5 当前求解流程与代码映射

```text
开始时间步 n -> n+1
    保存 q_n, v_n, a_n
    Newmark predictor；有 MPC 时保留主自由度的时间离散运动学
    for Newton iteration:
        评价 MPC：c、G、H、slaveDofs
        有 MPC 时，固定本步从方向，先迭代回缩从位置，再投影 v、a
        组装结构切线 K_t
        组装 M、陀螺切线 C_g、构型惯性切线 K_c
        K_eff = K_t + a0*M + a1*C_g + K_c
        组装动力残量 R_d
        若无 MPC：直接求 K_eff*dq = -R_d
        若有 MPC：
            检查 G_s 满秩
            求 lambda、T、p 和 K_L,d
            求降阶系统 K_red*dq_m = -T^T*R_d
            构造切向修正 dq = T*dq_m
        ApplyDynamicCorrection(dq, a0, a1)；线搜索试探重做位置回缩、v/a 投影和残量评估
        同时检查 ||T^T*R_d|| 与 ||c||，提交前检查三层约束
    保存约束乘子
    提交时间步
```

代码对应关系：

| 数学对象 | 程序位置 |
|---|---|
| $\mathbf c,\mathbf G,\mathbf H$ | `NonlinearMPCConstraint::Evaluate` 各派生约束 |
| 多个约束全局组装 | `AnalysisStep::AssembleNonlinearMPC` |
| $\mathbf T,\mathbf p,\boldsymbol\lambda,\mathbf K_L$ | `NonlinearMPC::Reduce` |
| 静力调用 | `SolverStatic::Solve` |
| 动力 Newmark 调用 | `SolverNewmark::SolveSingleStep` |
| Newmark 试探位置回缩及运动学投影 | `SolverNewmark::PrepareMpcTrialState` |
| 动力 TSSBN 调用 | `SolverAdaptiveTSSBN::SolveImplicitStage` |
| 位移/转动/速度/加速度同步修正 | `AnalysisStep::ApplyDynamicCorrection` 与 `Node::ApplyNewmarkCorrection` |

---

### 5.6 三层运动学投影与时间误差

只让 Newmark 端点满足 $\mathbf c(\mathbf q_{n+1})=0$，无法保证从位置递推得到的速度与加速度满足隐藏约束。旧路径的非零初角速刚性偏置振子即使缩小步长，$\mathbf G\ddot{\mathbf q}+\mathbf h$ 仍约为 $0.4$；这是位置级 index-3 离散的寄生加速度误差。当前程序在每轮 Newmark 动力残量评估和线搜索试探前，先回缩从位置，再按当前构型修正从自由度的速度、加速度，依次求解

$$
\mathbf G_s\delta\mathbf v_s=-\mathbf G\mathbf v,
\qquad
\mathbf G_s\delta\mathbf a_s=-(\mathbf G\mathbf a+\mathbf h).
$$

主自由度仍按 Newmark 更新，动力残量在投影后的状态上计算，因此不会把收敛后的状态另行修改。刚性偏置振子的固定步二分恢复二阶收敛，三层约束达到数值精度。投影后的有效切线属于拟 Newton 近似；每次迭代和线搜索必须重新评价真实动力残量。

自适应 TSSBN 的 C1、C2 隐式阶段也在动力残量评估前投影速度和加速度，使阶段平衡对应约束流形。两个阶段分别满足位置约束，但加权形成的接受端点未必仍在流形上。提交前，程序仅修正从自由度，迭代解 $\mathbf G_s\delta\mathbf q_s=-\mathbf c$；然后用最终构型的 Jacobian 和 Hessian 依次解

$$
\mathbf G_s\delta\mathbf v_s=-\mathbf G\mathbf v,\qquad
\mathbf G_s\delta\mathbf a_s=-(\mathbf G\mathbf a+\mathbf h),\qquad
h_\alpha=\mathbf v^\mathsf T\mathbf H_\alpha\mathbf v.
$$

投影后的基础解位移、速度与嵌入解比较 LTE，并与投影前 LTE 取较大值。原嵌入速度与基础速度在当前系数下代数上恒等，因此 MPC 路径另用步初、C1、C2 三点加速度的二次求积建立独立速度误差探针，纳入自适应步长决策。端点顺序投影参考[约束动力系统的投影方法](https://www.fs.isy.liu.se/Edu/Courses/Simulation/Papers/HairerWanner_VII_1-2_clean.pdf)；阶段动力投影的二阶性质由本程序固定步二分算例验证，不是该文端点投影定理的直接结论。

上述位置与运动学投影本身不保证动力平衡；当前正式默认 TSSBN 在误差门通过后还调用 `RecoverAcceptedMpcAcceleration`，以最终位置和速度重新组装端点质量及外载，联合恢复一致加速度和乘子。记 $R^g=F_{\mathrm{ext}}-F_{\mathrm{int}}-F_{\mathrm{damp}}-Ma^g$，辅助系统为

$$
\begin{bmatrix}M&G^T\\G&0\end{bmatrix}
\begin{bmatrix}\delta a\\\mu\end{bmatrix}
=\begin{bmatrix}R^g\\-Ga^g-h(v)\end{bmatrix},
\qquad a=a^g+\delta a,\quad\lambda=-\mu.
$$

由此同时检查端点动力平衡与加速度约束；最终接受端点乘子不再简单沿用 C2 阶段值。这里固定端点位置和速度，只恢复加速度及乘子，并不是重解整个离散步，也不自动得到严格能量—动量性质。初始动力状态使用同类质量—约束恢复。位移主流程仍采用主从消元，不能因此把整步称为位移—乘子的全局增广求解。

---

### 5.7 一致初始条件

动力计算开始前，理想状态应满足：

$$
\|\mathbf c(\mathbf q_0)\|\le\varepsilon_c,
$$

$$
\|\mathbf G(\mathbf q_0)\dot{\mathbf q}_0\|
\le\varepsilon_v,
$$

$$
\|\mathbf G(\mathbf q_0)\ddot{\mathbf q}_0
+\mathbf h(\mathbf q_0,\dot{\mathbf q}_0)\|
\le\varepsilon_a.
$$

`AnalysisStep` 在动力求解前由质量与约束方程生成一致的初始加速度。直接调用 Newmark 或 TSSBN 的 `IAnalysisModel` 入口时，程序在第一步前统一检查上述三层条件；不一致时报告对应残差，缺少初始加速度时提示调用方先求解质量与约束方程。不能把第一步的位置修正当成初始速度、加速度的一致化。

---

### 5.8 数据检查与使用限制

`NonlinearMPCData::IsValid` 检查约束值、Jacobian 和稀疏 Hessian 的有限性；`NonlinearMPC::Reduce` 拒绝非有限切线和右端项。约束必须独立，选定的 $G_s$ 必须非奇异；程序不会自动删除冗余约束，也没有混合长度/转角约束的自动量纲缩放。

文本输入通过 `SlaveDirection` 选择已有类型，并不是任意函数解释器。当前契约为 `0`/`1`/`2` 定长、`05` 平面释放、`012` 三向刚性偏置；`05` 的两个字符及前导零必须保留。完整字段见[文本模型格式](../02_模型与格式/文本模型格式.md)。

旧核查记录称“弹簧已删除”，与当前已有 `ElementSpring1/2/A` 不符；本手册不沿用该状态判断。是否严格复现 Boungard 的刚杆 snap-through、剪切释放等算例，应以论文参数、单元、增量和可重复结果逐项核对，不能由单元支持情况直接下结论。

## 6. 刚性中心 MPC 间隔棒

拓扑和编号见[导线建模与编号](../02_模型与格式/导线建模与编号.md)。中心模型只在刚体近似下等效质量与惯性，不保留梁式间隔棒的局部柔度。

### 6.1 目标与模型含义

对于四分裂导线，一个间隔棒位置原来用外框四边形和四根中心撑表示，共 8 个梁/桁架单元。若只关心间隔棒“保持四根子导线相对位置”的作用，可以用多边形中心的一个虚拟主节点表示刚体运动，并把四个导线节点作为从节点。

这一表示是**带等效质量和转动惯量的刚性间隔棒**。它与梁式间隔棒具有相同的总质量、
质心和刚体运动下的转动惯量，但不表示梁杆件的弹性变形、应力和局部模态。

### 6.2 自由度与主从关系

- 中心主节点：6 自由度，分别为三向平动和三向有限转动。
- 每个导线从节点：保留导线单元原有自由度。当前共旋索节点为三向平动加一个扭转自由度。
- MPC 只消去从节点的 `0,1,2` 三个平动自由度。
- 若导线是带轴向扭转自由度的共旋索，另用 `AxialTwistTieMPCConstraint` 将该标量扭转与中心转动关联；它不会被误当成梁节点的三向转角。

设中心主节点参考位置为 \(\mathbf X_m\)，第 \(i\) 个导线节点参考位置为 \(\mathbf X_i\)，初始偏置为

\[
\mathbf r_i=\mathbf X_i-\mathbf X_m .
\]

中心当前平移为 \(\mathbf u_m\)，当前有限转动矩阵为 \(\mathbf R_m\)。从节点当前平移为 \(\mathbf u_i\)。每根子导线建立一个三方程刚性偏置约束：

\[
\mathbf c_i=
(\mathbf X_i+\mathbf u_i)
-(\mathbf X_m+\mathbf u_m)
-\mathbf R_m\mathbf r_i
=\mathbf 0 .
\]

因此中心平动时四个点一起平动，中心发生任意大转动时四个偏置向量随之转动。任意两根子导线在该截面的距离恒定：

\[
\|\mathbf x_i-\mathbf x_j\|
=\|\mathbf R_m(\mathbf r_i-\mathbf r_j)\|
=\|\mathbf r_i-\mathbf r_j\| .
\]

这正是用一个中心点保持分裂多边形形状的依据。

### 6.3 程序实现

界面的“相内间隔棒形式”新增“刚性中心 MPC（高效）”。对每一个间隔棒位置，建模器执行：

1. 在各子导线对应节点的几何平均位置创建一个 6 自由度中心节点。
2. 保存每个导线节点相对中心的参考偏置。
3. 为每个导线节点创建一个 `RigidOffsetMPCConstraint`，中心为主、导线为从，从方向为 `012`。
4. 不创建外框、辐条或其他间隔棒单元；按原八段杆件的线质量分布，在中心节点添加一个 `RigidBodyInertia`。原形式为 CR3D 梁时再计入梁截面自转惯量；原形式为 T3D2 桁架时按其无截面自转惯量的质量模型等效。
5. 将生成的 MPC 和惯性对象记录在间隔棒模型中，以支持失败回滚、多档建模和模型保存。

HDF5 导出/导入保存 MPC 和刚体惯性。计算区域仍使用已有的 MPC 闭包规则：主节点或从节点任一方进入计算区域时，另一方会随约束一起纳入，MPC 不会因区域裁剪而删除。

### 6.4 四分裂模型规模

单个四分裂间隔棒的局部规模比较如下。

| 形式 | 新增中心节点 | 间隔棒单元 | 非线性 MPC | 约束标量方程 | 集中惯性 |
|---|---:|---:|---:|---:|---:|
| 外框 + 中心撑 | 1 | 8 | 0 | 0 | 0 |
| 刚性中心 MPC | 1 | 0 | 8（含 4 个轴向扭转约束） | 16 | 1 |

MPC 形式减少了单元组装和导线节点被梁单元扩展为 6 自由度的问题，但每次牛顿迭代需要重新计算约束、约束雅可比、转动 Hessian 和消元变换。因此是否更快必须以实际模型计时为准，不能只按单元数判断。

### 6.5 适用范围

适合：

- 只研究分裂导线整体运动、舞动和间隔棒对相对位置的限制；
- 将间隔棒近似为刚体，并保留其总质量与刚体转动惯量；
- 不需要间隔棒杆件内力、局部变形和连接柔度。

不适合：

- 需要间隔棒应力、疲劳或局部振型；
- 需要间隔棒的弹性振型或截面内非刚体运动；
- 间隔棒夹头存在明显滑移、转动释放或弹性；
- 真实间隔棒不是刚性多边形。

这些情况应保留梁单元，并采用独立梁节点作为主节点、导线节点作为从节点的平动连接；这与“刚性中心 MPC”是两种不同的物理模型。

## 7. 验证条件、数值记录与适用边界

### 7.1 验证层级及解析单摆

验证分为独立解析约束动力学、正式生产求解器和文本导入到结果提交的端到端链路。仅“完成计算”不能证明约束或动力响应正确。

Le 2012 无约束梁动力基线还用于检查梁本体的论文响应包络；它与 MPC 验证互补，不能用约束残差合格替代梁动力公式验证。

定长单摆取 $\mathbf q=[x,y]^T$：

$$
c(\mathbf q)=\tfrac12(x^2+y^2-L^2),\quad \mathbf G=[x\quad y],\quad \mathbf H=\mathbf I.
$$

初角 $\theta_0=0.8$ rad 的精确周期为

$$
T=4\sqrt{L/g}\,K\!\left(\sin(\theta_0/2)\right),
$$

其中 $K$ 为第一类完全椭圆积分。它检查定长、Hessian、乘子及向心项；刚性偏置振子进一步检查多条构型相关约束和惯性投影。

### 7.2 三层投影后的解析振子与公开单摆

为同时检验非线性 transformation、约束 Hessian 和向心项，加入了刚性偏置振子：

$$
q=[\theta,x_s,y_s]^T,
$$

$$
c_1=x_s-L\cos\theta=0,
\qquad
c_2=y_s-L\sin\theta=0.
$$

主自由度为 $\theta$，从自由度为 $x_s,y_s$。质量为点质量 $m$ 和中心转动惯量 $J$，中心有转动弹性势 $\tfrac12k\theta^2$。消元后的解析方程是

$$
\boxed{(J+mL^2)\ddot\theta+k\theta=0.}
$$

因此解析圆频率为

$$
\omega=\sqrt{\frac{k}{J+mL^2}}.
$$

使用一周期 400 步的平均加速度 Newmark，当前验证结果为：

| 指标 | 结果 |
|---|---:|
| 最大位置约束范数 | $2.48\times10^{-16}$ |
| 一周期角位移相对误差 | $8.34\times10^{-9}$ |
| 一周期归一化角速度误差 | $1.29\times10^{-4}$ |
| 最大速度约束范数 | 数值零 |
| 最大加速度约束范数 | $6.55\times10^{-17}$ |

运行命令：

```powershell
x64\ReleaseVerification\YQYVerification.exe --verify-dynamic-mpc
x64\ReleaseVerification\YQYVerification.exe --verify-dynamic-mpc-convergence
x64\ReleaseVerification\YQYVerification.exe --verify-dynamic-mpc-longtime
```

固定步从 24、48、96 步二分时，Newmark 与 TSSBN 主变量相位误差均约按四倍下降；两者端点三层约束保持在数值容差内。20 周期刚性偏置振子的最大相对能量误差分别为 Newmark $1.51\times10^{-10}$、TSSBN $5.74\times10^{-7}$；该结果仅对应当前无物理阻尼的线性主模态。

公开参数对照另见 [IFToMM 平面单摆基准](../../YQY/TestModels/Published/IFToMM_PlanarPendulum/README.md)：两种积分器均完成 10 s 时程；步长 $0.0005$ s 时，Newmark/TSSBN 最大能量漂移分别约为 $1.64\times10^{-5}$ J 和 $2.73\times10^{-6}$ J，均低于题目 $5\times10^{-5}$ J 限值。两组结果来自程序生产求解器。

---

### 7.3 文本导入的 CR3D 刚性偏置梁

模型文件：[DynamicMPC_RigidOffsetBeam.inp](../../YQY/TestModels/Constraints/MPC/DynamicMPC_RigidOffsetBeam.inp)。

模型包含：

- 4 个六自由度节点；
- 2 个带密度的 CR3D 梁单元；
- 节点 2 为 master、节点 3 为 slave；
- `*MPC 012` 三方向刚性偏置；
- 0.1 s Newmark 动力分析；
- 节点 4 的竖向时变荷载。

原端到端验证记录（工况和版本以原报告为准）：

| 指标 | 结果 |
|---|---:|
| 刚性偏置位置约束间隙 | $2.17\times10^{-19}$ |
| 末端运动量 | $4.21\times10^{-2}$ |
| 节点/单元/MPC | 4 / 2 / 1 |

该测试证明了 MPC 能经过实际文本输入和 `AnalysisStep` 进入动力计算区域，并与 CR3D 梁的质量、惯性力和有效切线共同工作。

### 7.4 投影前的历史结果

下表来自原《动力非线性 MPC 算例筛选与验证报告》，是位置级旧实现的记录，保留以解释三层投影的必要性。它不能与上节当前三层投影结果混作同一版本的验收值。

| 指标 | 定长单摆，800 步/周期 | 刚性偏置振子，400 步/周期 |
|---|---:|---:|
| 最大位置约束误差 | $4.44\times10^{-16}$ | $2.48\times10^{-16}$ |
| 一周期位置/角位移误差 | $1.57\times10^{-9}$ | $1.11\times10^{-8}$ |
| 一周期归一化速度误差 | $4.74\times10^{-5}$ | $1.49\times10^{-4}$ |
| 最大速度约束误差 | $2.14\times10^{-5}$ | $7.32\times10^{-6}$ |
| 最大加速度约束误差 | $3.74\times10^{-3}$ | $1.37\times10^{-4}$ |
| 最大相对能量误差 | $1.52\times10^{-5}$ | 原报告未给出 |

### 7.5 刚性中心间隔棒验证

验证入口：

运行命令：

```text
--verify-conductor-spacer-mpc
```

验证同时建立原中心撑模型和刚性中心 MPC 模型，检查：

- 间隔棒单元数和 MPC 数量符合拓扑预期；
- 两个模型均能完成非线性静力求解；
- 求解后所有刚性偏置几何残差不超过 \(10^{-7}\)；
- 梁式间隔棒总质量与 MPC 集中质量相等，中心刚体惯性进入组装后的质量矩阵；
- HDF5 往返后 MPC 和刚体惯性数量保持不变；
- 输出总自由度、约化自由度和求解时间。

原报告记录的 ReleaseVerification 验证结果（四分裂、每根 50 段、4 个间隔棒）为：

| 指标 | 外框 + 中心撑 | 刚性中心 MPC |
|---|---:|---:|
| 间隔棒单元 | 32 | 0 |
| MPC 对象 | 0 | 32 |
| 刚体惯性对象 | 0 | 4 |
| 间隔棒总质量 | 12.3775 kg | 12.3775 kg |
| 总自由度 | 896 | 864 |
| 约化自由度 | 896 | 800 |
| 集中惯性与组装质量矩阵差 | — | 数值零 |
| 最大刚性偏置残差 | — | \(2.40\times10^{-14}\) m |
| 最大轴向扭转约束残差 | — | \(1.70\times10^{-19}\) |

质量等效仍不能消除两种模型的刚度差：梁式间隔棒允许杆件弯曲，MPC 间隔棒保持刚性多边形。性能也取决于约束装配、降阶投影和结构求解，需在同一算例和机器负载下计时比较。

### 7.6 四分裂导线配对验证与研究边界

当前已用 `RigidOffsetMPCConstraint + AnalysisStep + ElementBeam_CR` 的端到端模型文件验证动力求解；
也已用同一修复版可执行文件完成四分裂导线中等质量 MPC 间隔棒与梁式间隔棒的 60 s 舞动对照。
解析刚性偏置振子和单摆验证了 Newmark 与自适应 TSSBN 的位移、速度和加速度约束，
其中 TSSBN 包括接受端点的三层投影。

已完成初始三层一致性诊断、刚性偏置振子的固定步二分和 20 周期约束/能量漂移对照，
并用 IFToMM 公开平面单摆 10 s 时程作外部基准。
最终四分裂导线 60 s 配对中，两组均完成 12000 步；40–60 s 跨中四线平均竖向峰峰值为
梁式 $2.301926$ m、补齐截面转动惯量后的 MPC $2.31947125$ m，MPC 高约 $0.762\%$，
终态 MPC 间隙为 $1.90563\times10^{-14}$ m。三副间隔棒总质量均为 $10.443541091$ kg；
独立梁积分与 MPC 刚体转动惯量的六分量最大差为 $5.55\times10^{-17}$ kg·m²。
完整配置、时程和统计见
[间隔棒质量等效与配对复验报告](../../YQY/Export/ExportFile/验证归档/galloping_tests/mpc_spacer_comparison_20260928/report_mass_equivalence.md)；
旧版时程只用于复核修复的影响。

后续专门算法研究包括：

1. 对照 Jelenić–Crisfield 或 Ibrahimbegović–Mamouri 的公开参数建立三维关节梁动力算例；
2. 对具有非零总动量的无阻尼系统开展线动量、角动量长期漂移对照；
3. 实现 Betsch 离散零空间和能量—动量版本，比较更长时间下的守恒性质。

### 7.7 经典多体算例的复现边界

[Ibrahimbegović 等的柔性多体动力论文](https://www.researchgate.net/publication/227090234_Finite_Element_Method_in_Dynamics_of_Flexible_Multibody_Systems_Modeling_of_Holonomic_Constraints_and_Energy_Conserving_Integration_Schemes)中的 three-bar swing 包含两个刚性连杆、柔性梁、转动副、梁中点集中质量和能量守恒格式。[Jelenić–Crisfield 2001](https://www.sciencedirect.com/science/article/abs/pii/S0045782500003443)还包含随体关节释放和能量—动量版本。

当前 YQY 已有 `RigidBodyInertia`，可把等效质量和转动惯量附着在节点上，因此原报告“一概缺少集中惯性”的说法已过时。但它不能替代通用独立刚体构件、revolute/prismatic joint 和专门能量—动量积分器。三杆摆、刚柔机械臂及空间滑块曲柄尚不能据此声称逐参数复现。极大刚度梁、节点质量或简化释放会改变质量分布和高频响应，只能作为另行标注的近似模型。

验证入口按需运行（仓库根目录）：

运行命令：

```powershell
x64\ReleaseVerification\YQYVerification.exe --verify-dynamic-mpc
x64\ReleaseVerification\YQYVerification.exe --verify-dynamic-mpc-convergence
x64\ReleaseVerification\YQYVerification.exe --verify-dynamic-mpc-initial-conditions
x64\ReleaseVerification\YQYVerification.exe --verify-dynamic-mpc-longtime
x64\ReleaseVerification\YQYVerification.exe --verify-dynamic-mpc-model YQY\TestModels\Constraints\MPC\DynamicMPC_RigidOffsetBeam.inp
x64\ReleaseVerification\YQYVerification.exe --verify-conductor-spacer-mpc
x64\ReleaseVerification\YQYVerification.exe --verify-le2012-example1
```

数值表是原报告的工况记录；本次文档合并没有重新运行这些验证。比较版本或积分器时必须保留初始条件、步长/容差、阻尼、模型参数和约束检查层级。

### 7.8 Betsch 2005 双球面摆的生产约束动力复现

2026-10-20 新增论文第 4.2.5 节双球面摆的显式回归，参数、独立参考及运行说明见
[Betsch 算例目录](../../YQY/TestModels/Published/Betsch2005_ConstraintDynamics/README.md)。
它使用真实 `AnalysisStep + Node + RigidBodyInertia + DistanceMPCConstraint + Force_Node`，
不以单独手写的约束模型替代生产装配。两根无质量杆长均为 1，质量分别为 10、5，重力为
$(0,0,-10)$。论文采用相对杆向量，换成绝对节点后初位置为 $(0,1,0)$、$(0,2,0)$，
初速度为 $(1,0,0)$、$(2,0,0)$。

固定 Y 从方向不能全局覆盖本算例的球面运动：首次生产复现中 Newmark 在约 0.633 s、
TSSBN 在 0.600 s 失败，未完成图 6–8 的 10 s 区间。约束值接近零不能替代时程完成和轨迹精度验收。
距离对象的 `m_SlaveDirection=-1` 现支持在从节点的自由平动分量中按当前杆方向选择最大分量；
已有非负指定方向的含义保持不变。Newmark 在一次隐式步及其线搜索内固定步初选定的从自由度，
避免迭代中换轴把旧从变量的运动学投影修正带入新主变量。

最细步长还暴露了纯平动 MPC 不具备原有限转动精度停止资格的问题：在 5.285 s，
正常残差门为 $1.11803\times10^{-8}$，残差停在 $1.15132\times10^{-8}$，完整坐标修正仅
$6.07442\times10^{-16}$。现将非空 MPC 纳入既有精度受限候选，保留实际修正停滞、
约束、完整修正和当前切线行尺度检查，并用约束 Hessian 与 $|G_s^{-1}|$
补充从变量投影速度变化的一阶绝对上界；正常 Newton 容差不变。

验证入口为 `YQYTests.exe --verify-betsch-double-pendulum <新输出目录>`，
使用 `--betsch-solver newmark|tssbn`、`--betsch-step <步长>` 和 `--betsch-auto-slave`。
参考脚本独立求解连续约束方程，并独立重算论文的中点格式；后者不表示生产已实现 Betsch 算法。
四档步长为 0.01、0.005、0.0025、0.00125 s，验收检查完整终时、原始状态重算的三层约束、
六维位置和速度误差及二阶收敛。能量、竖直角动量和乘子差另行报告，不能把 Newmark/TSSBN
的结果标为论文专用算法的机器精度守恒。

原四档步长、自动选轴的最终八组均完成 10 s 和预定时间网格。最细步长 0.00125 s 时，Newmark 的全时程
六维位置/速度最大误差为 $3.91625\times10^{-4}$ m、$2.68905\times10^{-3}$ m/s；
TSSBN 为 $1.20222\times10^{-5}$ m、$7.02724\times10^{-5}$ m/s。
四档细化通过预先冻结的轨迹、三层约束及收敛门；失败尝试与全部时程保存在
[正式复现归档](../../YQY/Export/ExportFile/验证归档/mpc_dynamics_betsch_20261020/report.html)。

历史补充减半试算：原可执行文件的 Newmark 在 $h=0.000625$ s、最后接受时间 $9.79125$ s 后失败，
未完成目标 $10$ s；失败记录仍保留在[补充比较](../../../output/mpc-newmark-halfstep-20261020/comparison.json)。
两种步长均在相同物理时刻发生第二条约束 X 到 Z 的从方向切换，且从块条件数约为 2.418；
换轴本身不能解释失败。局部方程复核定位到：速度/加速度投影已移除纯从位置修正的直接惯性效应，
但旧动力右端及线搜索指标仍含未投影的 $\mathbf T^T\mathbf K_{L,d}\mathbf p$，导致方向和接受指标不一致。
该项的主导尺度为 $a_0M\mathbf p$，其中 $a_0=O(h^{-2})$、预测几何缺口通常为 $O(h^2)$，
所以单纯缩步不保证消除这项不一致。

2026-10-26 已按第 5.4 节修复实际生产 Newmark。使用原模型、原参考方程和原验收门重新运行：

| 名义步长 / s | 完成时间 / s | 最大六维位置误差 / mm | 最大六维速度误差 / mm/s |
|---:|---:|---:|---:|
| 0.01 | 10 | 25.741726 | 177.499444 |
| 0.005 | 10 | 6.336641 | 43.546309 |
| 0.0025 | 10 | 1.572653 | 10.797365 |
| 0.00125 | 10 | 0.391625 | 2.689051 |
| 0.000625 | 10 | 0.097749 | 0.671146 |
| 0.0003125 | 10 | 0.024422 | 0.167679 |

六组均完成预定网格，无非名义接受步；位置观测阶为 2.001～2.022。
原失败状态的半步长及 $10^{-5}$ s 局部测试均在禁止回退条件下连续推进四步，另有大参考坐标几何舍入回归。
位置回缩最多修正十次，仅在原位置容差内允许机器精度停滞；动力收敛容差、约束容差和迭代上限未放宽。
六组最大位置、速度、加速度约束误差分别不超过 $5.33\times10^{-15}$ m、
$1.34\times10^{-15}$ m/s、$3.56\times10^{-14}$ m/s²。
完整原始数据、参考加密检查及哈希见[修复后比较](../../../output/mpc-newmark-fix-20261026/final-comparison/comparison.json)。
最终 ReleaseVerification 编译为零警告、零错误；十项登记测试和四个动力 MPC 专项入口均通过。
旧轻量刚性偏置振子及单摆测试模型另补齐了检查点接口，以适配已有 TSSBN 整步/两半步路径；
测试快照包含运动量、步初备份、乘子及诊断状态，没有改变生产 TSSBN 或降低验收门槛。
这验证了已测算例的修复与二阶收敛，不保证任意 MPC 拓扑、构型或步长都能收敛。

本次当前 TSSBN 的 $h=0.00125$ s 单组复核也完成 10 s，位置误差为 0.002932 mm。
当前 TSSBN 已使用整步/两半步并接受细解，见[时间积分与阻尼](时间积分与阻尼.md)；
它与上面的 2026-10-20 历史 TSSBN 结果属于不同版本，不能将差异归功于此次仅修改 Newmark 的修复。

此项仅验证空间链式距离 MPC 与集中质量动力；自动方向的文本导入、界面选择和 HDF5 往返，
以及刚性偏置、柔性梁关节与冗余闭环均不在此项验收范围内。

#### 2026-10-26：更大固定步长的效果

进一步使用同一已修复的生产求解器，对两种方法各运行六档外层步长，目标均为 10 s。
新增测试入口 `--betsch-fixed-step` 在 Newmark 尝试缩步时、建立新试探状态之前停止；
TSSBN 维持 `minimumTimeStep=maximumTimeStep=h`。所有已接受步均符合指定外层网格，
没有通过失败后的缩步完成以下结果。生产算法、动力残量容差和迭代控制参数均未修改。

| 外层 h / s | Newmark 状态 | Newmark 完整 10 s 位置误差 / mm | TSSBN 状态 | TSSBN 完整 10 s 位置误差 / mm |
|---:|---|---:|---|---:|
| 0.005 | 完成 | 6.336641 | 完成 | 0.053739 |
| 0.01 | 完成 | 25.741726 | 完成 | 0.283432 |
| 0.02 | 完成 | 104.041276 | 完成 | 1.283714 |
| 0.05 | 最后接受到 4.90 s | — | 完成 | 12.766017 |
| 0.1 | 最后接受到 0.40 s | — | 最后接受到 2.20 s | — |
| 0.2 | 最后接受到 0.20 s | — | 最后接受到 0.20 s | — |

位置误差是在实际接受端点与原独立 DOP853 参考比较的两质点六维联合误差最大值。
失败组只在共同已完成区间分析，不能用短区间误差替代完整 10 s 误差；上表以横线标示。
已完成组和失败组已接受状态的三层约束均通过原独立物理门，但这不意味着轨迹精确。
例如 Newmark 的 0.02 s 虽能完成，位置误差已达约 104 mm，最大能量漂移约 1.246 J；
TSSBN 的 0.05 s 完成时，位置误差约 12.766 mm，最大能量漂移约 0.497 J。

当前 TSSBN 每个外层 h 仍计算粗步及两个 h/2，并接受两半步细解；相同外层 h 不能被表述为
相同内部积分步或相同计算量。其谱半径为 1，局部误差三项容差保持原基准的 1。
0.1、0.2 s 的 TSSBN 日志明确记录 C1+C2 修正总数超过现有上限 32，日志中的末阶段残量分别为
$2.85\times10^{-9}$ N、$3.77\times10^{-9}$ N，均已小于 $1.118\times10^{-8}$ N 的残量门。
因此这两项属于当前迭代次数控制触发，不能将它们简单描述为动力残量发散或方法理论不稳定。
Newmark 大步长则是在原名义步未收敛后请求回退，由固定步测试守卫停止；本轮不调整控制参数掩盖失败。

以 1 mm 为说明用精度参考，本轮较大步长中 TSSBN 的外层 0.005、0.01 s 达标，0.02 s 已超出；
Newmark 的 0.005 s 已超出，此前验证的 0.00125 s 约为 0.392 mm。
这不是求出的最大允许步长边界，仅是已测离散步长的结果。
原始成功/失败数据和科学绘图见
[大固定步长比较](../../../output/mpc-large-fixedstep-20261026/comparison/comparison.json)、
[可导入 Origin 的数值表](../../../output/mpc-large-fixedstep-20261026/comparison/metrics.csv)。

## 8. 外部历史实现与输入格式的区别

本节保留原线性 MPC 资料的可追溯信息。`MPCData`、`AssemblyEigen`、`DOFTYPE_MPC` 和 `TestData/SGFem` 属于另一套历史实现；当前 YQY 使用 `NonlinearMPCConstraint`、`AnalysisStep::AssembleNonlinearMPC` 和 `NonlinearMPC::Reduce`。下面 Nastran 卡片不能直接当作 YQY `*MPC` 四字段输入。

### 8.1 Nastran 线性卡片参考

典型卡片为：

```text
MPC     SID     G1      C1      A1      G2      C2      A2
```

其数学含义为：

$$
A_1u_{G_1,C_1}+A_2u_{G_2,C_2}+\cdots=0
$$

例如：

```text
MPC     2       3       5       1.0     2       5      -1.0
```

表示：

$$
u_{3,5}-u_{2,5}=0
$$

即两个自由度相等。

在旧 SGFem 数据模型中，第一项被保存为 `m_slaveItem`，后续项保存为 `m_masterItems`。因此第一项系数必须非零，并且第一项对应自由度应当能够被其余自由度唯一确定。

### 8.2 SGFem 外部历史实现映射

下表仅说明旧 SGFem/AssemblyEigen 资料中的实现，不是当前 YQY API：

| 理论对象 | 程序对象 |
| --- | --- |
| MPC 单条约束 | `MPCData` |
| 从自由度 | `MPCData::m_slaveItem`、`DOFTYPE_MPC` |
| 主自由度 | `MPCData::m_masterItems` |
| $R_{mn}$ | `AssemblyEigen::m_RnTri` 组装得到的稀疏矩阵 |
| $R_{mm}$ | `AssemblyEigen::m_RmTri` 组装得到的稀疏矩阵 |
| $G=-R_{mm}^{-1}R_{mn}$ | `AssemblyEigen::m_Gm` |
| $T^TKT$ | `AssemblyEigen::Constraint()` 中的 MPC 凝聚 |
| $T^TF$ | `AssemblyEigen::AdjustMPCLoad()` |
| $u_m=Gu_n$ | `AssemblyEigen::RecoverResultVec()` |

原资料中的外部仓库路径（保留名称用于追溯，不作为本仓库链接）：

- `../../../DataStructure/Input/Constraint/MPCData.h`
- `../DOFHandler/DOFHandler.cpp`
- `../DOFHandler/GlobalDOFAllocte.cpp`
- `../Assembly/Assembly.cpp`
- `../../Task/Kernel/MModel.cpp`
- `../../../TestData/SGFem/Task/Input/MPC_cbar_spc.bdf`

## 9. 合并来源与维护约定

| 原文 | 本文归属 |
|---|---|
| `docs/MPC多点约束理论与实现-已实现理论.md` | 第 2 节线性推导、第 8 节外部历史映射 |
| `docs/静力非线性MPC到动力非线性MPC.md` | 第 1 节文献职责、有限转动及离散零空间研究边界 |
| `YQY/DataStructure/Constraint/StaticNonlinearMPCTheory.md` | 第 3 节完整静力推导、公式索引和新增约束校验模板 |
| `YQY/DataStructure/Constraint/StaticToDynamicNonlinearMPCTheory.md` | 第 4–5 节动力实现、第 7 节当前验证与配对结果 |
| `YQY/DataStructure/Constraint/RigidCenterMPCSpacer.md` | 第 6 节中心模型、第 7.5 节质量与惯性验证 |
| `YQY/DataStructure/Constraint/DynamicMPCBenchmarkVerification.md` | 第 7 节验证层级、解析单摆、端到端模型和历史结果 |

后续修改直接维护对应理论、实现边界或验证章节；一次修改的过程说明不再新增独立手册。当前行为需与代码核对，历史数值必须保留版本/工况限制，论文原式与程序扩展始终分别说明。
