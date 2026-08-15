# 从静力非线性 MPC 到动力非线性 MPC：论文阅读与实现路线规划

> 适用目标：三维空间梁（共旋梁 / Simo 几何精确梁）+ 大位移大转动 + 非线性 MPC + Newmark / HHT 动力求解。  
> 本文按“理论依赖关系”而不是论文发表年份排序。

---

## 1. 总体路线

建议将后续学习和实现划分为 6 个阶段：

$$
\boxed{
\text{静力非线性 MPC}
\rightarrow
\text{三维梁非线性动力学}
\rightarrow
\text{动力 Master-Slave}
\rightarrow
\text{一般非线性动力 MPC}
\rightarrow
\text{离散零空间约束动力学}
\rightarrow
\text{能量-动量一致方法}
}
$$

核心原则：

1. 先固定静力非线性 MPC 的统一约束接口；
2. 再确保“无 MPC”的三维梁动力学完全正确；
3. 然后加入动力主从关系；
4. 再把 Boungard 2024 的一般 $c,G,H$ 框架推广到动力；
5. 最后研究离散零空间、能量-动量一致和长期稳定性。

---

# 2. 第一阶段：固定静力非线性 MPC 理论

## 2.1 Boungard & Wackerfuß, 2024

**Master-slave elimination scheme for arbitrary smooth nonlinear multi-point constraints**

这是当前静力 MPC 模块的核心论文，也是后续动力扩展的起点。

### 需要彻底掌握

一般非线性约束：

$$
\boxed{\mathbf c(\mathbf q)=\mathbf 0}
$$

约束 Jacobian：

$$
\boxed{
\mathbf G
=
\frac{\partial \mathbf c}{\partial \mathbf q}
}
$$

第 $\alpha$ 条约束的 Hessian：

$$
\boxed{
\mathbf H_\alpha
=
\frac{\partial^2 c_\alpha}{\partial \mathbf q^2}
}
$$

主、从自由度划分：

$$
\mathbf q=
\begin{bmatrix}
\mathbf q_m\\
\mathbf q_s
\end{bmatrix},
\qquad
\mathbf G=
\begin{bmatrix}
\mathbf G_m & \mathbf G_s
\end{bmatrix}
$$

主从增量关系：

$$
\boxed{
\Delta\mathbf q_s
=
-\mathbf G_s^{-1}\mathbf c
-
\mathbf G_s^{-1}\mathbf G_m\Delta\mathbf q_m
}
$$

以及：

- 修改残差 $\mathbf R_{\mathrm{mod}}$；
- 修改切线 $\mathbf K_{\mathrm{mod}}$；
- 约束 Hessian 的一致线性化；
- 约束力恢复；
- 主从自由度消元。

### 程序目标

把约束从静力求解器中独立出来：

```text
NonlinearConstraint
    │
    ├── EvaluateConstraint()       -> c
    ├── EvaluateJacobian()         -> G
    ├── EvaluateHessian()          -> H / weighted H
    ├── GetMasterDofs()
    └── GetSlaveDofs()
```

最终思想：

$$
\boxed{
\mathbf c,\ \mathbf G,\ \mathbf H
}
$$

属于“约束对象”，以后静力 Newton、动力 Newmark、动力 HHT 都复用同一套约束定义。

---

# 3. 第二阶段：三维梁大转动 Master-Slave 的运动学来源

## 3.1 Jelenić & Crisfield, 1996

**Non-linear ‘master-slave’ relationships for joints in 3-D beams with large rotations**

这篇用于理解一般 $c,G,H$ 约束在三维空间梁大转动和关节问题中的实际物理意义。

### 重点

- master node / slave node；
- 三维梁大转动；
- 球铰；
- 转动副；
- 移动副；
- 圆柱副；
- 关节释放；
- 主从 transformation；
- transformation 随当前构型变化。

### 与刚性连接模型对应

$$
\boxed{
\mathbf x_i
=
\mathbf x_c+\mathbf R_c\mathbf r_i^0
}
$$

外围节点姿态：

$$
\boxed{
\mathbf R_i
=
\mathbf R_c\mathbf A_i^0
}
$$

其中中心节点 $c$ 为 master，外围节点 $i$ 为 slave。

### 这一阶段的认识目标

$$
\boxed{
\text{Jelenić 1996}
=
\text{三维梁特定约束的非线性运动学}
}
$$

而：

$$
\boxed{
\text{Boungard 2024}
=
\text{将这些约束抽象为统一的 }\mathbf c,\mathbf G,\mathbf H
}
$$

---

# 4. 第三阶段：先保证“无 MPC”的三维梁非线性动力学正确

加入动力 MPC 前，必须先保证：

$$
\boxed{
\text{无 MPC 时，共旋梁或 Simo 梁动力学本身完全正确}
}
$$

否则 MPC 和动力学同时出错时，很难定位问题。

---

## 4.1 共旋梁路线

### 4.1.1 Crisfield, Galvanetto & Jelenić, 1997

**Dynamics of 3-D co-rotational beams**

重点看：

- co-rotational dynamic formulation；
- 刚体运动与局部变形分离；
- 惯性力；
- 大转动；
- endpoint scheme；
- midpoint scheme；
- 能量表现；
- 数值稳定性。

主要回答：

$$
\boxed{
\text{共旋梁本身的非线性动力方程应该怎样建立？}
}
$$

### 4.1.2 Le, Battini & Hjiaj, 2012

**Dynamics of 3D beam elements in a corotational context**

重点精读：

$$
\mathbf f_{\mathrm{int}},
\qquad
\mathbf f_{\mathrm{ine}},
\qquad
\mathbf K_T,
\qquad
\mathbf K_{\mathrm{ine}}
$$

有限转动更新：

$$
\boxed{
\mathbf R^{n+1}
=
\exp\left(
[\Delta\boldsymbol\theta]_\times
\right)
\mathbf R^n
}
$$

以及这些量如何进入 Newmark 时间积分。

无约束动力残差目标：

$$
\boxed{
\mathbf R_d
=
\mathbf F_{\mathrm{ext}}
-
\mathbf F_{\mathrm{int}}
-
\mathbf F_{\mathrm{ine}}
-
\mathbf F_{\mathrm{damp}}
}
$$

---

## 4.2 Simo / 几何精确梁路线

### 4.2.1 Ibrahimbegović & Al Mikdad, 1998

**Finite rotations in dynamics of beams and implicit time-stepping schemes**

重点：

- $SO(3)$ 有限转动；
- rotation increment；
- angular velocity；
- angular acceleration；
- implicit time stepping；
- Newmark-type scheme；
- consistent linearization。

核心问题是：平动 Newmark 可以直接处理 $\mathbf u,\dot{\mathbf u},\ddot{\mathbf u}$，但有限转动不能简单使用

$$
\boldsymbol\theta_{n+1}
=
\boldsymbol\theta_n
+
\Delta\boldsymbol\theta
$$

因此必须理解 $\mathbf R,\boldsymbol\omega,\boldsymbol\alpha$ 在时间积分中的正确更新。

---

# 5. 第四阶段：正式进入动力 Master-Slave

这是从“静力非线性 MPC”进入“动力非线性 MPC”的核心阶段。

## 5.1 Ibrahimbegović & Mamouri, 2000

**On rigid components and joint constraints in nonlinear dynamics of flexible multibody systems employing 3D geometrically exact beam model**

这是 Simo / 几何精确梁方向最重要的动力约束论文之一。

### 位置关系

$$
\boxed{
\mathbf x_s
=
\mathbf x_m
+
\mathbf R_m\mathbf r^0
}
$$

令：

$$
\boldsymbol\rho
=
\mathbf R_m\mathbf r^0
$$

### 速度关系

$$
\boxed{
\mathbf v_s
=
\mathbf v_m
+
\boldsymbol\omega_m
\times
\boldsymbol\rho
}
$$

### 加速度关系

$$
\boxed{
\mathbf a_s
=
\mathbf a_m
+
\boldsymbol\alpha_m
\times
\boldsymbol\rho
+
\boldsymbol\omega_m
\times
\left(
\boldsymbol\omega_m
\times
\boldsymbol\rho
\right)
}
$$

其中：

$$
\boxed{
\boldsymbol\omega_m
\times
\left(
\boldsymbol\omega_m
\times
\boldsymbol\rho
\right)
}
$$

是静力问题中不存在的向心加速度项。

这篇主要解决：

$$
\boxed{
\text{几何精确梁}
+
\text{刚体/关节}
+
\text{非线性动力}
+
\text{主从自由度消元}
}
$$

---

## 5.2 Jelenić & Crisfield, 2001

**Dynamic analysis of 3D beams with joints in presence of large rotations**

这是共旋梁 / 一般六自由度空间梁方向最重要的动力 Master-Slave 论文之一。

静力中：

$$
\delta\mathbf q
=
\mathbf T
\delta\mathbf q_m
$$

动力中还必须处理：

$$
\dot{\mathbf q},
\qquad
\ddot{\mathbf q}
$$

典型动力降阶思想：

$$
\boxed{
\mathbf R_{\mathrm{dyn,red}}
=
\mathbf T^T
\mathbf R_{\mathrm{dyn}}
}
$$

但一致动力切线不能简单认为只是：

$$
\mathbf T^T
\mathbf K_{\mathrm{eff}}
\mathbf T
$$

因为：

$$
\boxed{
\mathbf T=\mathbf T(\mathbf q)
}
$$

会随构型变化，需要考虑 transformation 本身的变化。

核心问题：

$$
\boxed{
\text{静力 master-slave 如何真正进入非线性动力时间积分？}
}
$$

---

# 6. 第五阶段：自行推导 Boungard 2024 的动力扩展

前面的论文看完以后，开始从 Boungard 的统一约束形式向动力推广。

## 6.1 位置级约束

$$
\boxed{
\mathbf c(\mathbf q)=\mathbf 0
}
$$

## 6.2 速度级约束

$$
\boxed{
\mathbf G(\mathbf q)\dot{\mathbf q}
=
\mathbf 0
}
$$

其中：

$$
\mathbf G
=
\frac{\partial\mathbf c}{\partial\mathbf q}
$$

## 6.3 加速度级约束

$$
\boxed{
\mathbf G(\mathbf q)\ddot{\mathbf q}
+
\dot{\mathbf G}(\mathbf q,\dot{\mathbf q})
\dot{\mathbf q}
=
\mathbf 0
}
$$

其中：

$$
\boxed{
\dot{\mathbf G}\dot{\mathbf q}
}
$$

是静力中完全不存在的动力附加项。

## 6.4 引入 Newmark

需要将：

$$
\mathbf q_{n+1},
\qquad
\dot{\mathbf q}_{n+1},
\qquad
\ddot{\mathbf q}_{n+1}
$$

统一表示成当前 Newton 未知量。

之后重新建立：

$$
\boxed{
\mathbf R_{\mathrm{mod,dyn}}
}
$$

和：

$$
\boxed{
\mathbf K_{\mathrm{mod,dyn}}
}
$$

核心研究问题：

> Boungard 2024 静力中的 $\mathbf c,\mathbf G,\mathbf H$ 主从消元，如何在 Newmark 时间离散后的完整动力残差上进行一致线性化？

---

# 7. 第六阶段：通用约束动力学与离散零空间

第一版 Newmark + Dynamic MPC 做通以后，再进入这一阶段。

## 7.1 Betsch, 2005

**The discrete null space method for the energy consistent integration of constrained mechanical systems: Part I: Holonomic constraints**

研究：

$$
\mathbf M\ddot{\mathbf q}
+
\mathbf f
+
\mathbf G^T\boldsymbol\lambda
=
\mathbf F
$$

以及：

$$
\mathbf c(\mathbf q)=0
$$

重点：

- DAE；
- holonomic constraint；
- discrete null space；
- 约束力消元；
- 约束漂移；
- 时间离散后的约束处理。

构造零空间：

$$
\boxed{
\mathbf G\mathbf P
=
\mathbf 0
}
$$

使用：

$$
\Delta\mathbf q
=
\mathbf P\Delta\mathbf z
$$

从而消除 $\boldsymbol\lambda$。

## 7.2 Betsch & Leyendecker, 2006

**The discrete null space method ... Part II: Multibody dynamics**

重点：

- rigid body；
- revolute joint；
- prismatic joint；
- cylindrical joint；
- closed-loop constraint；
- multibody dynamics。

用于检查第一版 Dynamic MPC 在复杂关节和多体系统中的适用性。

## 7.3 Leyendecker, Betsch & Steinmann, 2008

**The discrete null space method ... Part III: Flexible multibody dynamics**

进入：

$$
\boxed{
\text{Rigid Body}
+
\text{Beam}
+
\text{Shell}
+
\text{Joint}
}
$$

统一柔性多体动力学框架。

---

# 8. 第七阶段：能量-动量一致约束动力学

## 8.1 Leyendecker, Betsch & Steinmann, 2006

**Objective energy–momentum conserving integration for the constrained dynamics of geometrically exact beams**

第一版 Newmark 动力 MPC 完成后再重点看。

研究：

- energy conservation；
- linear momentum；
- angular momentum；
- constraint satisfaction；
- long-time integration stability。

无阻尼系统特别关注：

$$
\boxed{
E_{n+1}
\approx
E_n
}
$$

以及：

- 动量漂移；
- 约束漂移；
- 长时间数值稳定性。

---

# 9. 最终推荐阅读顺序

## 第一组：静力 MPC

1. **Boungard & Wackerfuß, 2024**  
   目标：$\mathbf c,\mathbf G,\mathbf H,\mathbf G_m,\mathbf G_s,\mathbf R_{\mathrm{mod}},\mathbf K_{\mathrm{mod}}$

2. **Jelenić & Crisfield, 1996**  
   目标：把一般 $\mathbf c,\mathbf G,\mathbf H$ 与大转动梁主从运动学联系起来。

## 第二组：无约束非线性梁动力学

### 共旋梁方向

3. **Crisfield, Galvanetto & Jelenić, 1997**  
4. **Le, Battini & Hjiaj, 2012**

### Simo 梁方向

并行阅读：

**Ibrahimbegović & Al Mikdad, 1998**

目标：

$$
\boxed{
\text{先保证无 MPC 动力系统完全正确}
}
$$

## 第三组：动力 Master-Slave

5. **Ibrahimbegović & Mamouri, 2000**  
6. **Jelenić & Crisfield, 2001**

目标：

$$
\boxed{
\text{Position-level MPC}
\rightarrow
\text{Velocity-level MPC}
\rightarrow
\text{Acceleration-level MPC}
}
$$

并最终接入：

$$
\boxed{
\text{Newmark / HHT}
}
$$

## 第四组：自行推导 Boungard 动力扩展

$$
\boxed{
\mathbf c
\rightarrow
\mathbf G
\rightarrow
\dot{\mathbf G}
\rightarrow
\mathbf R_{\mathrm{dyn}}
\rightarrow
\mathbf R_{\mathrm{mod,dyn}}
\rightarrow
\mathbf K_{\mathrm{mod,dyn}}
}
$$

并使用 2000 / 2001 年动力论文作为特殊情况验证。

## 第五组：通用约束动力学

7. **Betsch, 2005 — Part I**  
8. **Betsch & Leyendecker, 2006 — Part II**  
9. **Leyendecker, Betsch & Steinmann, 2008 — Part III**

## 第六组：高级能量-动量算法

10. **Leyendecker, Betsch & Steinmann, 2006**

---

# 10. 近期只需要重点看 6 篇

| 顺序 | 论文 | 主要任务 |
|---|---|---|
| 1 | Boungard & Wackerfuß, 2024 | 一般静力非线性 MPC：$c,G,H$ |
| 2 | Jelenić & Crisfield, 1996 | 三维梁大转动主从运动学 |
| 3 | Le et al., 2012 | 共旋梁 + Newmark 非线性动力学 |
| 4 | Ibrahimbegović & Al Mikdad, 1998 | Simo 梁有限转动隐式时间积分 |
| 5 | Ibrahimbegović & Mamouri, 2000 | Simo 梁 + 刚体/关节 + 动力约束 |
| 6 | Jelenić & Crisfield, 2001 | 大转动三维梁动态 Master-Slave |

这 6 篇读透以后，理论上已经具备推导：

$$
\boxed{
\text{Boungard 2024 非线性 MPC}
+
\text{Newmark}
+
\text{CR / Simo 梁非线性动力学}
}
$$

所需的主要基础。

---

# 11. 推荐的程序实现顺序

## Step 1：保持当前静力 MPC 不动

验证：

$$
\|\mathbf c\|
$$

以及静力 benchmark。

## Step 2：独立验证无 MPC 动力梁

验证：

- 位移；
- 速度；
- 加速度；
- 能量；
- 惯性力；
- 转动更新；
- 时间步敏感性。

## Step 3：实现最简单的动力刚性 MPC

先做：

$$
\mathbf x_s
=
\mathbf x_m+\mathbf R_m\mathbf r^0
$$

只使用：

- 一个 master；
- 一个 slave；
- 刚性连接。

验证：

$$
\mathbf v_s
=
\mathbf v_m
+
\boldsymbol\omega_m\times\boldsymbol\rho
$$

以及：

$$
\mathbf a_s
=
\mathbf a_m
+
\boldsymbol\alpha_m\times\boldsymbol\rho
+
\boldsymbol\omega_m\times
(\boldsymbol\omega_m\times\boldsymbol\rho)
$$

## Step 4：扩展到五节点刚性整体

中心节点：

$$
\mathbf q_c\in\mathbb R^6
$$

四个外围节点：

$$
\mathbf q_i\in\mathbb R^6
$$

总自由度：

$$
30
$$

约束：

$$
24
$$

最终独立自由度：

$$
6
$$

## Step 5：把 Dynamic MPC 接到 Newmark Newton 迭代

```text
Time step n -> n+1
        │
        ├── Newmark predictor
        │
        ├── Newton iteration
        │       │
        │       ├── Assemble structural dynamic residual
        │       ├── Assemble structural effective tangent
        │       ├── Evaluate c, G, H
        │       ├── Dynamic MPC reduction
        │       ├── Solve master increment
        │       ├── Recover slave increment
        │       └── Update R, U, V, A
        │
        └── Commit converged state
```

---

# 12. 最终研究主线

$$
\boxed{
\begin{array}{c}
\text{Boungard 2024}\\
\text{一般静力非线性 MPC}\\
\downarrow\\
\text{Jelenić 1996}\\
\text{三维梁大转动主从运动学}\\
\downarrow\\
\text{CR / Simo 梁无约束非线性动力学}\\
\downarrow\\
\text{Ibrahimbegović 2000 + Jelenić 2001}\\
\text{动力 Master-Slave}\\
\downarrow\\
\text{自行推导 Boungard 动力扩展}\\
\downarrow\\
\text{Newmark / HHT + 一般非线性动力 MPC}\\
\downarrow\\
\text{Betsch / Leyendecker}\\
\text{离散零空间 + 能量动量一致约束动力学}
\end{array}
}
$$

---

# 13. 当前最推荐的工作重点

第一版建议保持：

$$
\boxed{
\text{6-DOF 节点}
+
\text{指数映射旋转更新}
+
\text{Newmark}
+
\text{Dynamic Master-Slave}
}
$$

在这一版本完全验证以后，再考虑：

- HHT-$\alpha$；
- generalized-$\alpha$；
- discrete null space；
- energy-momentum；
- 复杂关节；
- 冗余非线性 MPC；
- 大规模柔性多体系统。
