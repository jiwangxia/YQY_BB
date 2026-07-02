# 位移约束 TABULAR 时程设计

## 1. 目标与范围

在一个静力分析步中，通过约束自身保存的分段线性时程完成杆单元加载、卸载和再加载，并保留每个收敛增量的塑性历史。

本次只实现：

- `Constraint` 自己保存 `TABULAR` 位移时程；
- 静力求解器按当前伪时间施加位移；
- 一个三阶段一维弹塑性杆验证模型；
- 使用现有节点反力输出进行验证。

本次明确不实现：

- 独立的 `Amplitude` 类、幅值 ID 或全局幅值表；
- 修改 `LoadBase` 及现有力荷载时程；
- LE、PE、PEEQ 等新增结果输出；
- 梁或实体单元塑性。

荷载时程和约束时程保持两套独立实现。

## 2. Constraint 数据结构

在 `Constraint.h` 内定义约束专用时程点：

```cpp
struct ConstraintTimePoint
{
    double time = 0.0;
    double value = 0.0;
};
```

`Constraint` 增加：

```cpp
std::vector<ConstraintTimePoint> m_TimePoints;

/**
 * @brief 获取当前时间的约束位移
 * @param currentTime 当前分析步伪时间
 * @param factor 旧格式使用的线性增量因子
 */
double GetValue(double currentTime, double factor) const;
```

计算规则：

```cpp
double Constraint::GetValue(double currentTime, double factor) const
{
    if (m_TimePoints.empty())
        return m_Value * factor;

    return m_Value * Interpolate(currentTime);
}
```

其中 `m_Value` 是基础位移，表格值是无量纲缩放系数。

时程规则：

- 数据点至少两个；
- 时间严格递增；
- 时间和值必须为有限数；
- 区间内部使用线性插值；
- 当前时间小于首个时间时返回首值；
- 当前时间大于末个时间时返回末值。

## 3. 输入格式

现有四字段约束保持不变：

```text
约束ID, 节点ID, 方向, 位移值
```

带时程的约束使用：

```text
约束ID, 节点ID, 方向, 基础位移, TABULAR, 数据点数量
时间1, 缩放系数1
时间2, 缩放系数2
...
```

例如右端 x 向约束：

```text
4, 2, 0, 5.0, TABULAR, 4
0.0, 0.0
1.0, 1.0
2.0, 0.713181223
3.0, 0.8
```

输入器读取约束主行后，如果发现 `TABULAR`，立即继续读取指定数量的数据点。固定约束和旧模型仍使用四字段格式。

以下情况导致输入失败：

- 主行既不是 4 字段也不是 6 字段；
- 扩展类型不是 `TABULAR`；
- 数据点少于两个；
- 数据行不是两个数值；
- 时间不严格递增；
- 出现非有限数值；
- 时程数据不足或提前遇到下一个关键字。

## 4. 求解流程

接口由：

```cpp
Assemble_Constraint(x1, factor);
```

改为：

```cpp
Assemble_Constraint(x1, currentTime, factor);
```

静力求解器每个增量已有：

```cpp
factor = inc / numIncrements;
currentTime = duration * factor;
```

组装约束时：

```cpp
const double value =
    constraint->GetValue(currentTime, factor);

x1[dof] = value;
node->m_Displacement[direction] = value;
```

没有时程的约束仍使用 `m_Value * factor`，行为不变。

每个 Newton 收敛增量继续调用现有 `CommitState()`，因此位移反向变化时塑性历史仍然保留。

## 5. 验证模型

模型参数：

```text
L = 1000 mm
A = 100 mm²
E = 200000 MPa
初始屈服应力 = 250 MPa
H = 10000 MPa
```

右端基础位移为 `5 mm`，约束内时程为：

```text
t=0: 系数=0，位移=0
t=1: 系数=1，位移=5
t=2: 系数=0.713181223，位移=3.565906115
t=3: 系数=0.8，位移=4
```

第二阶段位移按程序采用的对数应变计算，使卸载终点接近零应力。

单一静力分析步：

```text
1, STATIC, 3.0, 0.01, 1e-8, 100
```

按当前对数应变和当前面积计算，预期节点反力大小约为：

```text
t=1: 28417.476 N
t=2: 0 N
t=3: 8614.700 N
```

反力符号取决于程序约定。

## 6. 文件范围

预计修改：

- `YQY/DataStructure/Constraint/Constraint.h`
- `YQY/DataStructure/Constraint/Constraint.cpp`
- `YQY/Import/Input_Model.cpp`
- `YQY/Solver/Interface/IAnalysisModel.h`
- `YQY/DataStructure/AnalysisStep/AnalysisStep.h`
- `YQY/DataStructure/AnalysisStep/AnalysisStep.cpp`
- `YQY/Solver/Static/SolverStatic.cpp`
- Visual Studio 工程文件（仅在新增测试文件需要登记时）

预计新增：

- 约束时程插值测试；
- 单一三阶段杆验证模型。

不会修改：

- `LoadBase` 及各类力荷载；
- HDF5 输出结构；
- 杆单元材料算法；
- 梁单元。

## 7. 测试

至少验证：

1. 表格首端、末端和区间中点插值；
2. 表格外时间取端点值；
3. 非递增时间被拒绝；
4. 四字段约束仍使用 `value × factor`；
5. 六字段约束使用 `value × TABULAR(time)`；
6. 完整模型能够读取；
7. Debug x64 完整构建成功；
8. 三个检查时刻的反力趋势分别为塑性加载、零应力卸载和弹性再加载。
