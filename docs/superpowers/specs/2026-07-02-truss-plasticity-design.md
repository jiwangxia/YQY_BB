# 杆单元一维弹塑性设计

## 1. 目标与范围

本次只为 `ElementTruss` 增加一维双线性弹塑性，并保留现有初始应力能力。梁单元、梁的共旋 `7×7` 基本刚度和其他单元不修改。

必须满足：

- 原有弹性材料与旧输入文件结果保持不变。
- 初始应力继续参与内力、几何刚度和屈服判断。
- 支持理想塑性（`H = 0`）和线性各向同性硬化（`H > 0`）。
- Newton 迭代中只计算试算状态，增量步收敛后才提交历史状态。
- 共享材料对象只保存参数，不保存任何单元的应力或塑性应变。

## 2. 职责划分

### 2.1 `Material`

`Material` 保存共享参数并提供一维本构入口：

```cpp
Material1DResult Update1D(
    double strain,
    const Material1DState& oldState) const;
```

`Update1D` 表示“根据当前总应变和上一个已收敛状态，计算新的试算结果”。名称使用 `1D` 明确该函数只服务于一维材料响应。

`Material` 新增参数：

```cpp
enum class MaterialModel
{
    Elastic,
    Plastic1D
};

MaterialModel m_Model = MaterialModel::Elastic;
double m_YieldStress = 0.0;  // 初始屈服应力
double m_Hardening = 0.0;    // 线性硬化模量 H
```

默认模型为 `Elastic`，保证旧模型不会因为已有的 `m_MaxStress` 数值而自动进入塑性。`m_MaxStress` 不直接复用为屈服应力，避免改变旧字段语义。

### 2.2 `ElementTruss`

杆单元负责：

1. 根据当前长度计算总对数应变。
2. 保存本单元独有的已收敛状态和试算状态。
3. 调用 `Material::Update1D`。
4. 使用返回应力计算轴力和几何刚度。
5. 使用返回的一维材料刚度计算材料刚度。

杆单元不实现屈服判断和塑性修正公式。

### 2.3 求解器与分析模型

求解器在一个增量步的 Newton 迭代中可以多次调用 `Get_ke()`。这些调用只能覆盖试算状态，不能累计提交塑性应变。

增量步收敛后，求解器调用分析模型的 `CommitState()`；分析模型再遍历单元并提交状态。梁等非塑性单元使用空实现。

## 3. 数据结构与命名

为降低理解成本，使用下面两个简单结构：

```cpp
/**
 * @brief 杆单元材料点在一个已收敛增量步上的状态
 */
struct Material1DState
{
    double strain = 0.0;             ///< 总应变
    double stress = 0.0;             ///< 总应力，包含初始应力
    double plasticStrain = 0.0;      ///< 有符号塑性应变
    double accumulatedPlastic = 0.0; ///< 非负累积塑性应变
};

/**
 * @brief 一维材料本构计算返回值
 */
struct Material1DResult
{
    double stress = 0.0;       ///< 当前试算应力
    double stiffness = 0.0;    ///< 一维切线刚度 D，即 1×1 矩阵的唯一分量
    Material1DState state;     ///< 当前试算状态
};
```

`ElementTruss` 增加：

```cpp
Material1DState m_OldState;   ///< 上一个已收敛增量步的状态
Material1DState m_TrialState; ///< 当前 Newton 迭代的试算状态
bool m_StateInitialized = false;

void InitializeState();
void CommitState() override;
```

`InitializeState()` 内部使用 `m_StateInitialized` 保证只初始化一次，避免每次组装刚度时重置历史状态。

代码注释重点解释物理意义和状态生命周期，不逐行翻译显而易见的 C++ 语句。

## 4. 初始应力

初始化材料点状态时：

```cpp
m_OldState.strain = 0.0;
m_OldState.stress = m_InitStress;
m_TrialState = m_OldState;
```

因此第一次计算的弹性预测为：

```text
Δε = ε当前 - ε旧
σtrial = σ旧 + E Δε
```

在初始构形中 `ε旧 = 0`、`σ旧 = m_InitStress`，所以：

```text
σtrial = m_InitStress + E ε当前
```

这与现有弹性公式一致。塑性材料要求：

```text
|m_InitStress| <= σy0
```

如果不满足，说明仅凭初始应力不足以建立一致的塑性历史。程序应报告清晰错误，不得静默截断或修正初始应力。

## 5. 一维本构算法

给定当前总应变 `ε` 和旧状态：

```text
Δε = ε - εn
σtrial = σn + E Δε
σy = σy0 + H αn
ftrial = |σtrial| - σy
```

其中 `α` 是非负累积塑性应变。

### 5.1 弹性步骤

当材料类型为弹性，或塑性材料满足 `ftrial <= tolerance`：

```text
σn+1 = σtrial
D = E
εp,n+1 = εp,n
αn+1 = αn
```

无论处于弹性还是塑性阶段，返回状态都必须令：

```text
state.strain = 当前总应变
state.stress = 当前试算应力
```

### 5.2 塑性步骤

当 `ftrial > tolerance`：

```text
Δγ = ftrial / (E + H)
direction = sign(σtrial)
σn+1 = σtrial - E Δγ direction
εp,n+1 = εp,n + Δγ direction
αn+1 = αn + Δγ
D = EH / (E + H)
```

理想塑性使用 `H = 0`，此时塑性段 `D = 0`。总单元切线仍可能包含由当前轴力产生的几何刚度。

## 6. 杆单元组装

`ElementTruss::Get_ke()` 保留当前几何和面积更新方式，只替换应力与材料切线的来源：

```cpp
const double strain = std::log(lengthCurrent / L0);

InitializeState();
const Material1DResult result =
    material->Update1D(strain, m_OldState);

m_TrialState = result.state;
m_Stress = result.stress;

const double D = result.stiffness;
const double materialStiffness = D * currentArea / L0;
```

后续规则不变：

- 材料刚度使用 `D`。
- 轴力使用 `m_Stress * currentArea`。
- 几何刚度使用当前总应力 `m_Stress`。
- `Get_ke()` 不修改 `m_OldState`。

## 7. 状态提交

在 `ElementBase` 增加默认空实现：

```cpp
virtual void CommitState() {}
```

`ElementTruss` 覆盖为：

```cpp
void ElementTruss::CommitState()
{
    m_OldState = m_TrialState;
}
```

在 `IAnalysisModel` 增加明确的 `CommitState()` 接口，由 `AnalysisStep` 遍历全部单元调用。各求解器仅在确认当前增量步成功后调用它，并且在保存该步结果之前完成提交。

如果当前增量步失败，旧状态保持不变。后续若增加自动减小步长，可直接从旧状态重新试算。

## 8. 输入格式

现有 `MATERIAL` 六字段格式保持不变，默认创建弹性材料。

新增独立塑性段，避免破坏旧文件：

```text
PLASTIC, 数量
材料ID, 初始屈服应力, 硬化模量
```

例如：

```text
PLASTIC, 1
1, 235000000, 1000000000
```

`PLASTIC` 段必须位于对应的 `MATERIAL` 段之后。读取后将对应材料设置为 `Plastic1D`。输入检查：

- 材料 ID 必须存在。
- `E > 0`。
- `σy0 > 0`。
- `H >= 0`。

## 9. 错误处理

以下情况输出包含材料或单元 ID 的中文错误，并终止当前求解：

- 材料、属性、截面或节点指针失效。
- 杆初始长度或当前长度接近零。
- 塑性参数不合法。
- 初始应力位于初始屈服面之外。
- 本构计算产生非有限数值。

输入阶段可以发现的问题由输入函数返回 `false`。本构计算阶段发现的非有限数值等内部错误抛出 `std::runtime_error`，由求解器入口捕获、记录中文错误并返回求解失败。本构接口不得通过返回全零刚度来掩盖错误。

## 10. 测试与验收

至少覆盖：

1. 旧弹性杆：无初始应力，结果与修改前一致。
2. 预应力弹性杆：验证 `σ = σ0 + Eε`。
3. 屈服前加载：`D = E`，塑性应变为零。
4. 理想塑性：屈服后应力保持在屈服应力，`D = 0`。
5. 线性硬化：屈服后 `D = EH/(E+H)`。
6. 卸载：卸载斜率为 `E`，保留残余应变。
7. 反向加载：使用应力绝对值进行双向屈服判断。
8. Newton 重复调用：同一增量内多次 `Get_ke()` 不重复累计塑性应变。
9. 状态提交：仅收敛后更新旧状态。
10. 非法初应力：`|σ0| > σy0` 时给出明确错误。

实现代码中的公开接口、状态字段和塑性公式必须配有简明中文注释。
