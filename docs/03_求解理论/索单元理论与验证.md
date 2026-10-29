# 二节点共旋索单元理论、TSSBN 对照与验证

## 1. 范围与结论

本程序的 `ElementCable` 对应 `D:\VS\TSSBN\Wind_method\Element_Cable_CR`：二节点、三维、大位移、小应变的共旋索单元。每个节点具有 4 个自由度

\[
\mathbf q_i=[u_i,v_i,w_i,\theta_i]^T,
\]

其中前三项为全局平移，`theta` 为绕当前索轴的扭转角。该单元不是把整跨悬链线解析解写进一个单元的“精确悬链线单元”；重力弧垂由多个直线共旋索单元离散并通过几何非线性平衡得到。

本程序实现与 TSSBN 的 `Calculate_ke_TSSBN`、`Calculate_me` 在以下部分一致：

- 当前弦向量和共旋轴；
- 初始构形上的工程轴向应变；
- 轴向和扭转材料刚度；
- 由当前轴力产生的几何刚度；
- 三维平动与轴向扭转的一致质量矩阵；
- 试探位移、速度、加速度由求解器统一写入节点，单元在每次 Newton 迭代重新计算。

本程序有一项有意的物理扩展：索不承受压力。当试算轴力不大于零时，轴向内力、轴向材料刚度和几何刚度同时归零；扭转通道仍保留。原 TSSBN 参考代码没有做拉压截断。

## 2. 构形与运动学

初始节点坐标为 `X1`、`X2`，初始弦向量和长度为

\[
\mathbf L_0=\mathbf X_2-\mathbf X_1,
\qquad L_0=\|\mathbf L_0\|.
\]

当前节点位置、弦向量、长度和单位轴向为

\[
\mathbf x_i=\mathbf X_i+\mathbf u_i,
\qquad \mathbf l=\mathbf x_2-\mathbf x_1,
\qquad l=\|\mathbf l\|,
\qquad \mathbf n=\frac{\mathbf l}{l}.
\]

轴向伸长量和相对扭转角为

\[
e=l-L_0,
\qquad \varphi=\theta_2-\theta_1.
\]

刚体平移不改变 `l`，刚体转动只改变 `n` 而不产生轴向应变，因此该构造满足大位移下的客观性。

## 3. 截面与材料参数

定义

\[
EA=E A,
\qquad G=\frac{E}{2(1+\nu)},
\qquad GJ=GJ_p,
\]

其中 `Jp` 是截面对索轴的极惯性矩。圆截面半径为 `r` 时

\[
A=\pi r^2,
\qquad J_p=\frac{\pi r^4}{2}=\frac{A r^2}{2}.
\]

轴向和扭转切线系数采用初始材料长度：

\[
k_a=\frac{EA}{L_0},
\qquad k_t=\frac{GJ_p}{L_0}.
\]

这与 TSSBN 的 `Calculate_ke_TSSBN` 一致。旧 YQY 实现曾使用 `EA/l` 和 `GJ/l`，但仍把它们直接作为切线；对应内力 `EA(l-L0)/l` 的真实导数并不是 `EA/l`，会造成 Newton 切线不一致，并改变动力频率。

## 4. 广义内力

设输入的初始应力为 `sigma0`，初始轴力为

\[
N_0=\sigma_0 A.
\]

试算轴力和扭矩为

\[
N_{trial}=N_0+k_a(l-L_0),
\qquad T=k_t(\theta_2-\theta_1).
\]

索的单边本构为

\[
N=\max(0,N_{trial}).
\]

定义广义应变—位移矩阵

\[
\mathbf B=
\begin{bmatrix}
-\mathbf n^T&0&\mathbf n^T&0\\
0&-1&0&1
\end{bmatrix}.
\]

单元内力向量为

\[
\mathbf f_{int}=\mathbf B^T
\begin{bmatrix}N\\T\end{bmatrix}.
\]

它自动满足两节点轴向力和扭矩的作用—反作用平衡。

## 5. 一致切线刚度

张紧状态下材料切线为

\[
\mathbf K_m=\mathbf B^T
\begin{bmatrix}k_a&0\\0&k_t\end{bmatrix}
\mathbf B.
\]

当前轴向方向对节点位移的导数产生几何刚度。定义

\[
\mathbf P=\mathbf I-\mathbf n\mathbf n^T,
\qquad \mathbf G=\frac{N}{l}\mathbf P,
\]

则

\[
\mathbf K_g=
\begin{bmatrix}
\mathbf G&0&-\mathbf G&0\\
0&0&0&0\\
-\mathbf G&0&\mathbf G&0\\
0&0&0&0
\end{bmatrix}.
\]

最终切线为

\[
\mathbf K_e=\mathbf K_m+\mathbf K_g.
\]

当 `Ntrial <= 0` 时，轴向材料项和 `Kg` 同时为零，避免松弛索在压缩状态下产生虚假横向刚度。扭转项 `kt` 不受该截断影响。

## 6. 一致与集中质量矩阵

对称截面且质心、剪心重合时，单位长度惯性矩阵为

\[
\boldsymbol\mu=
\operatorname{diag}(\rho A,\rho A,\rho A,\rho J_p).
\]

一致质量矩阵为

\[
\mathbf M_e=\frac{L_0}{6}
\begin{bmatrix}
2\boldsymbol\mu&\boldsymbol\mu\\
\boldsymbol\mu&2\boldsymbol\mu
\end{bmatrix}.
\]

集中质量矩阵把总平动质量 `rho*A*L0` 和总扭转惯量 `rho*Jp*L0` 各分一半到两个节点。

TSSBN 参考实现含一个固定的 `Sy=0.459e-3` 平移—扭转偏心耦合值，它属于特定截面数据，不能作为通用索单元常数移植。本程序当前采用对称截面的 `Sy=Sz=0`；以后若截面数据结构增加质心/剪心偏置，应从截面属性读取，而不是硬编码。

## 7. 阻尼与动力装配

索单元阻尼按平移和扭转两个通道分别采用 Rayleigh 形式：

\[
\mathbf C_{xyz}=a_m\mathbf M_{xyz}+a_k\mathbf K_{xyz},
\qquad
\mathbf C_{theta}=b_m\mathbf M_{theta}+b_k\mathbf K_{theta}.
\]

本程序的四个阻尼输入依次为平移质量比例、平移刚度比例、扭转质量比例、扭转刚度比例。

动力平衡为

\[
\mathbf R=\mathbf F_{ext}-\mathbf F_{int}
-\mathbf M\ddot{\mathbf q}-\mathbf C\dot{\mathbf q}.
\]

旧 TSSBN 工程为每种积分器保存 `Ln_TSSBN_c1`、`Ln_TSSBN_c2` 等阶段专用变量。本程序采用统一的 `IAnalysisModel` 试探状态接口：自适应 TSSBN 在每个隐式阶段写入试探位移、速度和加速度，`ElementCable::Get_ke` 从节点当前试探状态重算内力和切线；拒绝时间步时由分析模型恢复备份状态。因此不需要在索单元内部复制求解器阶段变量，物理结果等价且职责更清晰。

## 8. 静力与动力基准

### 8.1 轴向静力

水平索仅保留端部轴向自由度，施加总轴力 `P>N0`，解析位移为

\[
u=\frac{(P-N_0)L_0}{EA}.
\]

Newton 残差应收敛到 `P-N=0`，并与上式一致。

### 8.2 初拉力索的横向自振

长度为 `L`、初拉力为 `N0`、线密度为 `mu=rho*A` 的固定—固定索，一阶连续体圆频率为

\[
\omega_1=\frac{\pi}{L}\sqrt{\frac{N_0}{\mu}}.
\]

用多个 `ElementCable` 装配横向几何刚度和一致质量矩阵，有限元一阶频率应随网格细化收敛到该解析值。

### 8.3 TSSBN 时间积分

将上述有限元一阶模态化为

\[
\ddot q+\omega_{1,h}^2q=0,
\qquad q(0)=1,\quad\dot q(0)=0,
\]

使用程序的自适应 TSSBN 积分五个周期，终点应回到 `q=1`、`qdot=0`。这同时检查索单元产生的 `K/M` 和程序 TSSBN 动力路径。

## 9. 自动验证入口

Release 构建后运行：

```powershell
YQY.exe --verify-cable-reference
YQY.exe --verify-cable-torsion
YQY.exe --verify-conductor-static
```

`--verify-cable-reference` 包含：

1. 与 `Element_Cable_CR::Calculate_ke_TSSBN` 的内力和刚度逐矩阵对比；
2. 与 `Element_Cable_CR::Calculate_me` 的一致质量逐矩阵对比；
3. 内力数值微分与解析切线对比；
4. 轴向静力 Newton 解与解析解对比；
5. 初拉力索横向频率与连续体解析解对比；
6. 索模态经本程序自适应 TSSBN 积分五周期后的位移和速度误差。

本次 Release 验证结果：

| 检查项 | 相对误差/结果 |
|---|---:|
| TSSBN 参考刚度矩阵 | 0 |
| TSSBN 参考内力向量 | 0 |
| TSSBN 参考一致质量矩阵 | 0 |
| 内力有限差分切线 | `2.24479e-9` |
| 轴向静力位移 | 0 |
| 轴向静力平衡残差 | `1.80747e-14` |
| 均布横向荷载跨中挠度 | `0.00299994 m` |
| 经典抛物线公式跨中挠度 | `0.00300000 m` |
| 横向静力挠度相对误差 | `2.10286e-5` |
| 横向静力 Newton 平衡残差 | `3.56128e-13`（3 次迭代） |
| 32 单元固定—固定索一阶圆频率 | `omega_h=23.2188 rad/s` |
| 连续体参考一阶圆频率 | `omega_ref=23.2095 rad/s` |
| 一阶频率相对误差 | `4.01643e-4` |
| 自适应 TSSBN 五周期位移误差 | `8.2695e-7` |
| 自适应 TSSBN 五周期归一化速度误差 | `3.64142e-4` |
| 扭转一阶频率相对误差 | `1.00402e-4` |
| 完整导线静力回归 | Cable、Truss、Beam 三种配置均成功 |

## 10. 适用限制

- 当前为直线二节点离散索，不是单单元精确悬链线公式；重力找形精度依赖网格。
- 当前轴向材料为线弹性工程应变，未包含塑性、蠕变和温度相关本构。
- 压缩—张紧切换点不可微，接近零张力时可能需要更小荷载步或平滑化本构。
- 扭转自由度描述绕索轴的标量扭转；复杂截面有限转动和翘曲不在本单元范围内。
- 气动负阻尼、覆冰系数和分裂导线气动映射属于外荷载模型，不能用“索单元有位移”替代舞动稳定性判定。
