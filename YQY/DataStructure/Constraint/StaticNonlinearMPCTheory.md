# 静力非线性 MPC 主从消元理论与程序实现

## 1. 文档目的与理论来源

本文用于说明本程序静力非线性分析中的多点约束（Multi-Point Constraint，MPC）如何建立、线性化、消元并进入 Newton–Raphson 迭代。目标是让第一次接触非线性 MPC 的读者能够从原始平衡方程开始，逐步推导出程序中实际求解的缩减方程。

本文的主线严格依据以下论文，而不是另行构造一种约束算法：

> J. Boungard, J. Wackerfuß, *Master–slave elimination scheme for arbitrary smooth nonlinear multi-point constraints*, Computational Mechanics 74 (2024), 955–992. DOI: 10.1007/s00466-024-02463-7.

对应的本地论文文件为仓库根目录下的 `Boungard_Wackerfuss_2024_Master_slave_elimination.pdf`。本文保留论文的主要符号，并在涉及程序时明确说明程序变量和符号差异。

本文重点对应论文：

- 第 2 节：无约束非线性问题和约束的基本假设，式 (1)–(6)；
- 第 4 节：非线性主从消元的完整推导，式 (20)–(37)；
- 第 5.1 节：边界条件与主从消元的处理顺序；
- 第 5.2 节：约束库及约束量的全局装配，式 (38)–(39)；
- 附录 D.1：定长约束的约束值、Jacobian 和 Hessian，式 (59)–(61)；
- 附录 D.2：二维非线性力释放约束的导数，式 (62)–(64)。

需要先明确一个核心观点：

> “MPC”是求解约束方程的一种统一框架；“定长”“刚性偏置”“转动相等”等只是不同的约束关系。求解器不应为每一种约束重新编写消元算法。每种约束只需要向统一求解器提供约束值 \(\boldsymbol c\)、Jacobian \(\boldsymbol G\) 和 Hessian \(\boldsymbol H_i\)。

---

## 2. 符号和维数

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

## 3. 从无约束静力平衡开始

### 3.1 势能驻值和残差

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

### 3.2 加入非线性约束

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

### 3.3 论文要求的三个前提

论文第 2 节给出了方法成立所需的假设：

1. **相容性**：存在同时满足全部 \(c_i=0\) 的状态，约束之间不能互相矛盾。
2. **光滑性**：每个 \(c_i\) 至少二次连续可微，使 \(\boldsymbol G\) 和 \(\boldsymbol H_i\) 存在，能够进行一致 Newton 线性化。
3. **独立性**：\(\boldsymbol G\) 满行秩，即

   \[
   \operatorname{rank}(\boldsymbol G)=n_c.
   \]

第三条意味着约束中不能存在重复方程。例如既设置 \(u_s-u_m=0\)，又以另一 MPC 重复设置完全相同的关系，会使约束 Jacobian 行相关。

---

## 4. 主从自由度划分

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

## 5. 为什么从自由度能够由主自由度确定

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

## 6. 修改后的平衡残差

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

## 7. 对修改残差作一致线性化

这是非线性 MPC 最关键、也最容易漏项的部分。

### 7.1 分块记号

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

### 7.2 为什么会出现约束 Hessian

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

## 8. 消去从自由度增量

### 8.1 先线性化约束方程

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

### 8.2 代回主方程

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

## 9. 程序使用的等价投影形式

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

## 10. 论文符号与当前程序符号的转换

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

## 11. 每个载荷增量内的完整 Newton 流程

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

## 12. 多个约束的装配

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

## 13. 定长约束：从约束值推到 Hessian

这是论文附录 D.1 的约束，也是首先需要复现的算例所使用的基本非线性关系。

### 13.1 几何定义

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

### 13.2 一阶导数

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

### 13.3 二阶导数

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

### 13.4 从自由度的选择

定长关系只有一个标量方程，所以只消去一个从自由度。假设选 \(u_{b,x}\) 为从自由度，则

\[
\boldsymbol G_s=\begin{bmatrix}-n_x\end{bmatrix}.
\]

只有 \(|n_x|>0\) 时该选择才可逆。若运动过程中 \(n_x\) 接近零，应选择导数绝对值更大的方向，或采用自动主元选择。这个问题与“节点是主还是从”无关，本质是所选从自由度列能否局部参数化约束面。

---

## 14. 论文二维非线性力释放约束

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

## 15. 刚性偏置关系如何进入同一框架

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

## 16. 边界条件与 MPC 的先后顺序

论文第 5.1 节特别强调，边界条件的处理顺序会影响正确性。

### 16.1 Neumann 边界条件

力边界条件先进入完整结构残差。它可以施加在参与约束的任意主或从自由度上，因为 \(\boldsymbol R_s\) 会通过

\[
-\boldsymbol G_m^T\boldsymbol G_s^{-T}\boldsymbol R_s
\]

传递到缩减主方程。

### 16.2 Dirichlet 边界条件

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

## 17. 计算区域与 MPC 完整性

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

## 18. 当前代码与论文公式的对应关系

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

## 19. 新增一种非线性 MPC 时的推导模板

以后新增约束时，应按以下顺序推导，而不是直接猜 Jacobian。

### 第一步：写当前构形中的物理关系

明确约束使用参考坐标、当前坐标、位移、旋转矩阵还是相对转角。例如

\[
c_i(\boldsymbol V)=0.
\]

先确认该方程确实表达所需物理关系，并检查量纲。

### 第二步：确定标量方程数

一个向量等式要拆成若干标量方程。方程数就是要消去的从自由度数。

### 第三步：解析求 Jacobian

\[
\boldsymbol G_i=\frac{\partial c_i}{\partial\boldsymbol V}.
\]

只对约束实际涉及的自由度填非零项，再装配到当前自由度编号。

### 第四步：解析求 Hessian

\[
\boldsymbol H_i=\frac{\partial^2c_i}{\partial\boldsymbol V^2}.
\]

对于光滑标量约束，解析 Hessian 应对称。若明显不对称，通常意味着漏项、符号错误，或转动参数化与增量定义不一致。

### 第五步：选择从自由度

从 \(\boldsymbol G\) 中选择一组使 \(\boldsymbol G_s\) 满秩的列。应避开：

- 已施加 Dirichlet 条件的自由度；
- 当前导数为零或很小的方向；
- 已被其他标量约束选作从自由度的方向；
- 物理上不应由该关系支配的自由度。

### 第六步：做数值导数校验

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

### 第七步：分别验证约束和结构求解

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

## 20. 复现论文时必须避免的错误

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

## 21. 公式索引

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

## 22. 最终结论

论文方法可归纳为以下闭环，但每一步都不可缺少：

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

也就是说，主从节点信息只负责告诉程序“哪些自由度由约束恢复”；真正定义相对位移、定长、刚性偏置或转动关系的是 \(\boldsymbol c(\boldsymbol V)=0\)。只要约束光滑、相容、独立，并能选择出可逆的 \(\boldsymbol G_s\)，同一套静力非线性 MPC 消元器就能够处理平动自由度、转动自由度及其非线性耦合。
