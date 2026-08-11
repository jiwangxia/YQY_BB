# 从静力非线性 MPC 到动力非线性 MPC：理论、离散、程序实现与核查

> 本文是对项目根目录 `静力非线性MPC到动力非线性MPC.md` 的展开和校核。目标不是把几篇论文简单拼接，而是说明：哪些公式直接来自论文，哪些是连续约束的数学推论，哪些是本程序第一版采用的 Newmark 离散实现。

## 1. 结论先行

当前程序采用的第一版动力 MPC 是：

$$
\boxed{
\text{光滑完整约束 }\mathbf c(\mathbf q)=\mathbf 0
+\text{ Newmark 隐式时间积分}
+\text{时间离散后主从消元}
}
$$

实现范围如下：

1. 静力和动力共用同一个约束对象，并共用 $\mathbf c$、$\mathbf G$、$\mathbf H_\alpha$；
2. 动力 MPC 已接入 `SolverNewmark` 的每次 Newton 迭代；
3. 约束作用于包含质量、陀螺和构型惯性切线的完整动力有效系统；
4. 每次迭代先求独立自由度修正，再恢复从自由度修正；
5. 收敛同时检查降阶动力残量和位置级约束残量；
6. 当前尚未把 MPC 接入 TSSBN、Adaptive TSSBN，也尚未实现 Betsch 离散零空间或能量—动量格式。

因此，“动力 MPC 已实现”的准确含义是：**Newmark 端点格式下的位置级、非线性、完整约束主从消元已经实现**。它不是最终的能量—动量一致约束积分器。

---

## 2. 论文分别解决什么问题

### 2.1 Boungard 与 Wackerfuß（2024）

[Master–slave elimination scheme for arbitrary smooth nonlinear multi-point constraints](https://link.springer.com/article/10.1007/s00466-024-02463-7)

论文明确研究的是静力有限元中的、与时间无关的光滑非线性 MPC。它给出了任意约束的统一数据：

$$
\mathbf c(\mathbf q),\qquad
\mathbf G=\frac{\partial\mathbf c}{\partial\mathbf q},\qquad
\mathbf H_\alpha=\frac{\partial^2c_\alpha}{\partial\mathbf q^2},
$$

以及主从消元后残量、切线和约束乘子的统一公式。本文静力部分严格以它为依据。

需要特别说明：Boungard 2024 没有给出 Newmark 动力公式，不能把本文后面的动力离散公式说成“Boungard 论文原式”。

### 2.2 Jelenić 与 Crisfield（1996、2001）

1996 年论文讨论大转动三维梁关节的非线性主从运动学；2001 年论文 [Dynamic analysis of 3D beams with joints in presence of large rotations](https://www.sciencedirect.com/science/article/abs/pii/S0045782500003443) 把这种关节主从关系扩展到动力问题，并关注无外载 Hamilton 系统中的能量和动量性质。

它们的重要作用是说明：梁关节的 transformation 依赖当前构型，不能把大转动约束当成常系数线性 MPC。

### 2.3 Ibrahimbegović 与 Mamouri（2000）

[On rigid components and joint constraints in nonlinear dynamics of flexible multibody systems employing 3D geometrically exact beam model](https://www.sciencedirect.com/science/article/abs/pii/S0045782599003631) 讨论几何精确梁、刚性部件和关节的非线性动力学，包含乘子法以及直接消除相对运动自由度的思路。

它提供了动力刚性偏置必须满足的位置、速度、加速度关系，尤其指出动力中不能遗漏向心加速度项。

### 2.4 Crisfield、Galvanetto 与 Jelenić（1997）

本项目根目录中的 `Crisfield_Galvanetto_Jelenic_1997_Dynamics_3D_corotational_beams.pdf` 研究无 MPC 的三维共旋梁动力学。论文给出了端点 Newmark 类格式的惯性力、一致线性化，以及中点近似能量守恒格式。

它解决的是“梁的动力残量和动力切线如何构造”，不是“通用 MPC 如何消元”。本程序必须先有正确的无约束动力系统，MPC 才有可投影的对象。

### 2.5 Le、Battini 与 Hjiaj（2012）

[Dynamics of 3D beam elements in a corotational context](https://www.sciencedirect.com/science/article/abs/pii/S0168874X12001217) 对共旋三维梁的 Newmark 时间积分、有限转动更新及多种动力公式进行比较。当前程序节点的有限转动 Newmark 更新采用了这一类空间旋量与材料增量转角转换思想。

### 2.6 Betsch（2005）及后续论文

[The discrete null space method, Part I: Holonomic constraints](https://www.sciencedirect.com/science/article/pii/S0045782505000137) 的关键原则是：先把受约束动力 DAE 离散，再在离散系统上构造零空间并消除乘子。Part II 将其扩展到多体动力学。

这为本程序“先形成 Newmark 离散动力残量和有效切线，再做主从消元”的顺序提供理论依据。不过，当前实现还不是 Betsch 的能量一致离散零空间算法，只遵循了它的离散顺序。

---

## 3. 静力非线性 MPC 从头推导

### 3.1 未约束平衡方程

令程序内部结构残量为

$$
\mathbf R(\mathbf q)=\mathbf f_{\mathrm{int}}(\mathbf q)
-\mathbf f_{\mathrm{ext}}=\mathbf 0.
$$

程序线性求解器使用的右端项是

$$
\mathbf b=-\mathbf R
=\mathbf f_{\mathrm{ext}}-\mathbf f_{\mathrm{int}}.
$$

这一区别十分重要，否则约束乘子和 Hessian 项很容易出现符号错误。

### 3.2 约束与主从划分

有 $n_c$ 条光滑约束：

$$
\mathbf c(\mathbf q)=\mathbf 0,
\qquad \mathbf c\in\mathbb R^{n_c}.
$$

把自由自由度重新理解为独立自由度 $\mathbf q_m$ 与待消去自由度 $\mathbf q_s$：

$$
\mathbf q=
\begin{bmatrix}\mathbf q_m\\\mathbf q_s\end{bmatrix},
\qquad
\mathbf G=
\frac{\partial\mathbf c}{\partial\mathbf q}
=\begin{bmatrix}\mathbf G_m&\mathbf G_s\end{bmatrix}.
$$

必须满足：

$$
\operatorname{rank}(\mathbf G_s)=n_c.
$$

即每条标量约束必须选择一个独立的从自由度，使 $\mathbf G_s$ 可逆。这里的“主”和“从”是代数消元角色，不表示物理上谁更重要。

### 3.3 约束的 Newton 线性化

在当前迭代点：

$$
\mathbf c(\mathbf q+\Delta\mathbf q)
\approx\mathbf c+\mathbf G\Delta\mathbf q=\mathbf0.
$$

分块后：

$$
\mathbf G_m\Delta\mathbf q_m
+\mathbf G_s\Delta\mathbf q_s=-\mathbf c.
$$

所以

$$
\boxed{
\Delta\mathbf q_s=
-\mathbf G_s^{-1}\mathbf G_m\Delta\mathbf q_m
-\mathbf G_s^{-1}\mathbf c
}
$$

定义

$$
\mathbf T=
\begin{bmatrix}
\mathbf I\\-\mathbf G_s^{-1}\mathbf G_m
\end{bmatrix},
\qquad
\mathbf p=
\begin{bmatrix}
\mathbf0\\-\mathbf G_s^{-1}\mathbf c
\end{bmatrix},
$$

得到完整修正：

$$
\boxed{\Delta\mathbf q=\mathbf p+\mathbf T\Delta\mathbf q_m.}
$$

$\mathbf p$ 是把当前点拉回约束流形的 particular correction；$\mathbf T$ 的列张成当前约束切空间。

### 3.4 乘子与一致约束切线

采用本程序的符号约定，受约束平衡写成

$$
\mathbf R-\mathbf G^T\boldsymbol\lambda=\mathbf0.
$$

从自由度行给出当前乘子：

$$
\boxed{
\boldsymbol\lambda=\mathbf G_s^{-T}\mathbf R_s.
}
$$

对 $\mathbf G^T\boldsymbol\lambda$ 线性化时，$\mathbf G$ 随 $\mathbf q$ 改变，因此必须出现约束 Hessian：

$$
\boxed{
\mathbf K_L=\mathbf K-\sum_{\alpha=1}^{n_c}
\lambda_\alpha\mathbf H_\alpha.
}
$$

漏掉该项一般会失去二次收敛，在大转动和大位移约束下尤其明显。

### 3.5 降阶方程

最终独立自由度方程为

$$
\boxed{
\mathbf K_{\mathrm{red}}
=\mathbf T^T\mathbf K_L\mathbf T
}
$$

以及

$$
\boxed{
\mathbf b_{\mathrm{red}}
=-\mathbf T^T\mathbf R
-\mathbf T^T\mathbf K_L\mathbf p.
}
$$

求得 $\Delta\mathbf q_m$ 后，用 $\Delta\mathbf q=\mathbf p+\mathbf T\Delta\mathbf q_m$ 恢复完整修正。

---

## 4. 从静力到连续动力

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
\mathbf v_s=\mathbf v_m+oldsymbol\omega_m\times\boldsymbol\rho
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

---

## 5. Newmark 时间离散后的动力 MPC

### 5.1 为什么要“先离散、后消元”

不能只把连续方程中的 $\mathbf K$ 换成 $\mathbf K+\mathbf M$。正确顺序是：

1. 在 $t_{n+1}$ 建立 Newmark 预测状态；
2. 把速度和加速度表示成当前 Newton 构型修正的函数；
3. 形成完整的时间离散动力残量及其一致有效切线；
4. 在这个离散系统上评价 $\mathbf c,\mathbf G,\mathbf H$ 并消元。

这与离散零空间方法所强调的“discretize first”原则一致。

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

六自由度节点的转动不能直接相加。程序使用指数映射更新 $\mathbf R$，并把空间旋转修正经 $\mathbf T_s^{-1}$ 转为步初材料增量转角，再更新角速度和角加速度。

### 5.3 动力有效切线

当前程序组装：

$$
\boxed{
\mathbf K_{\mathrm{eff}}
=\mathbf K_t+a_0\mathbf M+a_1\mathbf C_g+\mathbf K_c.
}
$$

这里 $\mathbf C_g$ 表示速度/陀螺切线，$\mathbf K_c$ 表示构型相关惯性切线。它们由各动力单元返回。

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

求解后恢复完整 $\Delta\mathbf q$，再由 Newmark 关系同步修正位移、转动、速度和加速度。

该推导不是“把静力刚度投影一下”的经验做法。它来自对**已经时间离散的受约束平衡方程**进行一致 Newton 线性化。

---

## 6. 程序中的实际流程

```text
开始时间步 n -> n+1
    保存 q_n, v_n, a_n
    Newmark predictor
    for Newton iteration:
        组装结构切线 K_t
        组装 M、陀螺切线 C_g、构型惯性切线 K_c
        K_eff = K_t + a0*M + a1*C_g + K_c
        组装动力残量 R_d
        评价所有活动 MPC：c、G、H、slaveDofs
        若无 MPC：直接求 K_eff*dq = -R_d
        若有 MPC：
            检查 G_s 满秩
            求 lambda、T、p 和 K_L,d
            求降阶系统 K_red*dq_m = b_red
            恢复 dq = p + T*dq_m
        ApplyDynamicCorrection(dq, a0, a1)
        同时检查 ||b_red|| 与 ||c||
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
| 动力 Newmark 调用 | `SolverNewmark::Solve` |
| 位移/转动/速度/加速度同步修正 | `AnalysisStep::ApplyDynamicCorrection` 与 `Node::ApplyNewmarkCorrection` |

---

## 7. 静力 MPC 代码核查结论

### 7.1 已确认正确的部分

逐式对照 Boungard 2024 后，当前 `NonlinearMPC::Reduce` 的核心关系一致：

- `slaveFromMaster = -Gs^{-1} Gm`；
- `particularCorrection_s = -Gs^{-1} c`；
- `multipliers = Gs^{-T} R_s`；
- `lagrangianTangent = K - sum(lambda_i H_i)`；
- `tangent = T^T K_L T`；
- `rhs = -T^T R - T^T K_L p`。

已有的定长、刚性偏置、平面剪切释放约束，其 Jacobian 和 Hessian 此前也经过中心差分核对，误差处在约 $10^{-8}$ 到 $10^{-10}$ 的量级。

### 7.2 本次补强

原先 `NonlinearMPCData::IsValid` 只检查 $\mathbf c$ 和 $\mathbf G$ 是否为有限数，没有检查稀疏 Hessian 的系数；`Reduce` 也没有显式拒绝含 NaN/Inf 的切线和右端项。本次已补充这些检查，避免非法数进入稀疏分解后才以难定位的方式失败。

### 7.3 仍然存在的限制

1. 约束必须独立，程序不会自动识别或删除冗余约束；
2. 用户指定的从自由度必须使 $\mathbf G_s$ 非奇异；
3. 当前输入格式通过 `SlaveDirection` 选择已有约束类型，并不是允许用户直接输入任意函数的通用表达式解释器；
4. 长度、转角、位移混合约束还没有自动量纲缩放；
5. 论文第一个刚杆 snap-through 算例含弹簧，而项目已按要求删除弹簧，因此目前不能声称该算例被原样复现；
6. 当前剪切释放算例使用 CR3D 等效模型和不同增量设置，是实现验证，不是论文参数逐项一致的严格复现。

结论是：**静力消元算法本身没有发现公式错误，但“已经严格复现 Boungard 论文全部算例”这一说法目前不成立。**

---

## 8. 动力验证算例

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
| 一周期角位移相对误差 | $1.11\times10^{-8}$ |
| 一周期归一化角速度误差 | $1.49\times10^{-4}$ |
| 最大速度约束范数 | $7.32\times10^{-6}$ |
| 最大加速度约束范数 | $1.37\times10^{-4}$ |

运行命令：

```powershell
x64\Release\YQY.exe --verify-dynamic-mpc
```

该算例证明 MPC 确实进入了生产 `SolverNewmark`，不是在验证程序外单独手算。

---

## 9. 为什么位置约束是机器精度，而速度/加速度不是

当前算法在每个 $t_{n+1}$ 直接求解 $\mathbf c(\mathbf q_{n+1})=0$，所以位置约束可达到 Newton 容差乃至机器精度。

但是普通 Newmark 使用端点位置增量更新速度和加速度。对于非线性约束，两个相邻端点都在曲面上，并不意味着由有限差分生成的端点速度严格位于连续切空间，也不保证加速度严格满足曲率项。因此一般有

$$
\mathbf G\dot{\mathbf q}=O(\Delta t^p),
\qquad
\mathbf G\ddot{\mathbf q}+\mathbf h=O(\Delta t^r),
$$

误差会随时间步缩小，但不一定达到机器精度。

不应在收敛后简单投影速度和加速度，因为那会改变惯性力，使刚刚收敛的动力平衡失效。若要求位置、速度、加速度关系和长期能量性质同时严格，应升级为：

1. 离散零空间或等价的受约束速度参数化；
2. 在离散层面共同构造构型与速度更新；
3. 采用能量—动量一致的约束积分格式；
4. 再验证能量、线动量、角动量和闭环约束长期漂移。

---

## 10. 初始条件必须满足什么

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

若初始位移不满足位置约束，第一时间步 Newton 会尝试通过 $\mathbf p$ 拉回约束流形，但这不等价于生成物理一致的初始速度和加速度。工程模型最好在求解前显式完成三层一致性检查。

---

## 11. 对四分裂导线目标的意义

最终间隔棒/导线模型可以继续复用同一框架。梁节点作为 master，导线连接点作为 slave。若只约束外围点位置：

$$
\mathbf x_i=\mathbf x_c+\mathbf R_c\mathbf r_i^0,
$$

每个外围点产生 3 条平动约束；若还要绑定截面姿态或索的扭转自由度，则需增加相应转动/扭转约束函数，而不是把 4 自由度索节点假装成 6 自由度梁节点。

动力计算中，外围导线质量仍保留在原自由度上。主从消元会自动把它们的惯性贡献投影到独立自由度，因此间隔棒中心会感受到外围质量带来的转动惯量和向心效应。这正是动力 MPC 相比只复制位移的关键价值。

---

## 12. 下一阶段的严格升级顺序

1. 为所有动力分析入口增加初始 $c/Gv/(Ga+h)$ 一致性诊断；
2. 做时间步二分试验，确认位移、速度约束、加速度约束的收敛阶；
3. 用真实 `RigidOffsetMPCConstraint + AnalysisStep + ElementBeam_CR` 建立端到端模型文件验证；
4. 对照 Jelenić–Crisfield 或 Ibrahimbegović–Mamouri 的公开参数建立关节梁动力算例；
5. 将相同离散约束接口接入 TSSBN/Adaptive TSSBN，但不能直接复制 Newmark 系数；
6. 实现 Betsch 离散零空间和能量—动量版本，用长时间无阻尼算例比较能量与动量漂移；
7. 最后进入四分裂导线、间隔棒和带扭转自由度索单元的完整系统验证。

## 13. 最终判断

静力到动力的核心不是新增一种“动力约束类型”，而是让同一个几何约束在不同层次发挥作用：

$$
\boxed{
\mathbf c
\xrightarrow{\partial/\partial\mathbf q}
\mathbf G
\xrightarrow{\partial/\partial\mathbf q}
\mathbf H
}
$$

- 静力：$\mathbf c$ 修正构型，$\mathbf G$ 给出切空间，$\mathbf H$ 给出一致约束切线；
- 连续动力：同一组量还决定 $\mathbf G\dot{\mathbf q}=0$ 和 $\mathbf G\ddot{\mathbf q}+\mathbf h=0$；
- 离散动力：先构造完整时间离散残量和有效切线，再用 $\mathbf T,\mathbf p,\mathbf H$ 消元。

当前第一版已经完成了这条路线中的“Newmark 位置级动力主从消元”。长期能量—动量一致和速度/加速度机器精度约束属于下一阶段，而不是当前实现已经具备的能力。
