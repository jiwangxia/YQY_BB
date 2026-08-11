# 动力非线性 MPC 算例筛选与验证报告

## 1. 验证层级

动力 MPC 不能只看“计算没有报错”。本报告按三个层级判断：

1. **解析约束动力学验证**：有独立解析解，可判断动力消元公式是否正确；
2. **生产求解器验证**：必须经过正式 `SolverNewmark`，不能在验证代码中自行消元；
3. **端到端有限元验证**：必须经过文本导入、约束对象、计算区域、自由度编号、梁单元动力组装和结果提交。

## 2. 当前程序能够可靠验证的算例

| 算例 | 约束特点 | 参考量 | 状态 |
|---|---|---|---|
| 非线性定长单摆 | 单条定长约束，非零 Hessian | 椭圆积分精确周期 | 已通过 |
| 大转动刚性偏置振子 | 两条构型相关约束、向心效应 | 消元后单自由度解析方程 | 已通过 |
| CR3D 双梁刚性偏置瞬态 | 输入文件 `*MPC 012`、梁质量和大转动 | 约束间隙及非零动力响应 | 已通过 |
| Le 2012 无约束梁动力基线 | 无 MPC，检查梁动力本体 | 论文响应包络 | 已通过 |

### 2.1 非线性定长单摆

使用笛卡尔坐标 $\mathbf q=[x,y]^T$：

$$
c(\mathbf q)=\frac12(x^2+y^2-L^2)=0,
$$

$$
\mathbf G=[x\quad y],
\qquad
\mathbf H=\mathbf I.
$$

初始角度为 $\theta_0=0.8$ rad。其精确周期为

$$
T=4\sqrt{\frac Lg}
K\!\left(\sin\frac{\theta_0}{2}\right),
$$

其中 $K$ 是第一类完全椭圆积分。

采用一周期 800 个 Newmark 时间步，结果为：

| 指标 | 结果 |
|---|---:|
| 最大位置约束误差 | $4.44\times10^{-16}$ |
| 一周期位置误差 | $1.57\times10^{-9}$ |
| 一周期归一化速度误差 | $4.74\times10^{-5}$ |
| 最大相对能量误差 | $1.52\times10^{-5}$ |
| 最大速度约束误差 | $2.14\times10^{-5}$ |
| 最大加速度约束误差 | $3.74\times10^{-3}$ |

该算例验证定长约束、约束 Hessian、约束乘子和曲线运动的向心加速度效应。

### 2.2 大转动刚性偏置振子

$$
c_1=x_s-L\cos\theta=0,
\qquad
c_2=y_s-L\sin\theta=0.
$$

消元后的解析方程：

$$
(J+mL^2)\ddot\theta+k\theta=0.
$$

一周期 400 个时间步的结果为：

| 指标 | 结果 |
|---|---:|
| 最大位置约束误差 | $2.48\times10^{-16}$ |
| 一周期角位移相对误差 | $1.11\times10^{-8}$ |
| 一周期归一化角速度误差 | $1.49\times10^{-4}$ |
| 最大速度约束误差 | $7.32\times10^{-6}$ |
| 最大加速度约束误差 | $1.37\times10^{-4}$ |

该算例验证构型相关 transformation、多条约束、非零 Hessian、偏置质量折算和向心项。

### 2.3 文本导入的 CR3D 刚性偏置梁

模型文件：`YQY/TestModels/DynamicMPC_RigidOffsetBeam.inp`。

模型包含：

- 4 个六自由度节点；
- 2 个带密度的 CR3D 梁单元；
- 节点 2 为 master、节点 3 为 slave；
- `*MPC 012` 三方向刚性偏置；
- 0.1 s Newmark 动力分析；
- 节点 4 的竖向时变荷载。

最终结果：

| 指标 | 结果 |
|---|---:|
| 刚性偏置位置约束间隙 | $2.17\times10^{-19}$ |
| 末端运动量 | $4.21\times10^{-2}$ |
| 节点/单元/MPC | 4 / 2 / 1 |

该测试证明了 MPC 能经过实际文本输入和 `AnalysisStep` 进入动力计算区域，并与 CR3D 梁的质量、惯性力和有效切线共同工作。

## 3. 论文经典算例的可行性判断

### 3.1 三杆摆：当前不能严格复现

[Ibrahimbegović 等人的柔性多体动力论文](https://www.researchgate.net/publication/227090234_Finite_Element_Method_in_Dynamics_of_Flexible_Multibody_Systems_Modeling_of_Holonomic_Constraints_and_Energy_Conserving_Integration_Schemes)中的 three-bar swing 由两个刚性连杆、一个柔性梁、端部转动副以及梁中点集中质量组成，并以能量守恒时间格式评价结果。

当前程序缺少：

- 集中质量单元；
- 独立刚体构件；
- 通用转动副；
- 论文所用能量守恒积分器。

用“极大刚度梁”代替刚体、用节点质量近似集中质量会改变质量分布和高频响应，因此不能把这种近似称为论文复现。

### 3.2 刚柔机械臂：当前不能严格复现

论文模型使用内部刚性部件和 revolute joint。当前 CR3D 梁可以表达柔性臂，但没有相同的刚体/释放自由度组件。

### 3.3 空间滑块曲柄：当前不能严格复现

该算例需要：

- 三维转动副；
- 移动副；
- 闭环约束；
- 刚性连杆；
- 柔性连接杆。

当前 `*MPC` 只提供定长、平动刚性偏置和特定平面剪切释放，不能完整描述空间 revolute/prismatic joints。

### 3.4 Jelenić–Crisfield 2001 的两个梁关节算例

[Jelenić–Crisfield 2001](https://www.sciencedirect.com/science/article/abs/pii/S0045782500003443) 的 conventional dynamic master–slave 原理与当前 Newmark 实现方向一致：把静力 transformation 作用到完整动力残量。但论文还包含随体关节释放以及能量—动量保持版本。当前约束库和时间积分器不足以逐参数复现这两个算例。

## 4. 本次实际执行的命令

```powershell
x64\Release\YQY.exe --verify-dynamic-mpc

x64\Release\YQY.exe `
  --verify-dynamic-mpc-model `
  YQY\TestModels\DynamicMPC_RigidOffsetBeam.inp

x64\Release\YQY.exe --verify-le2012-example1
```

## 5. 当前结论

已验证的范围不是只有一个自定义数值例子，而是：

$$
\boxed{
\text{定长非线性约束}
+\text{刚性偏置约束}
+\text{解析动力响应}
+\text{实际 CR3D 有限元导入链路}
}
$$

这些结果足以确认当前 **Newmark 位置级动力 MPC** 的基本实现有效。

但三杆摆、空间滑块曲柄和论文能量—动量关节梁仍不能标记为“已复现”。要完成它们，应先增加集中质量、刚体、通用 revolute/prismatic joint，再实现能量—动量或离散零空间时间积分。
