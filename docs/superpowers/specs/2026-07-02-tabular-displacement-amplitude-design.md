# 位移约束 TABULAR 时程设计

## 1. 目标与范围

在一个静力分析步中，通过分段线性位移时程完成杆单元的加载、卸载和再加载，并保留每个收敛增量的塑性历史。

本次只实现：

- `TABULAR` 幅值数据及线性插值；
- 位移约束引用幅值；
- 静力求解器按当前伪时间施加位移；
- 一个三阶段一维弹塑性杆验证模型；
- 使用现有节点反力输出进行第一阶段验证。

本次不实现：

- LE、PE、PEEQ 等新增结果输出；
- 修改现有 `LoadBase` 的力荷载时程；
- 其他解析幅值函数；
- 梁或实体单元塑性。

## 2. 数据结构

新增独立的 `TabularAmplitude`，避免把时程逻辑塞进 `Constraint`：

```cpp
struct AmplitudePoint
{
    double time = 0.0;
    double value = 0.0;
};

class TabularAmplitude : public Base
{
public:
    std::vector<AmplitudePoint> m_Points;

    /**
     * @brief 按时间对幅值进行分段线性插值
     */
    double GetValue(double time) const;
};
```

规则：

- 数据点至少两个；
- 时间严格递增；
- 时间和值必须为有限数；
- 区间内部线性插值；
- 小于首个时间时返回首值；
- 大于末个时间时返回末值。

`StructureData` 新增按 ID 保存的幅值表。

## 3. 输入格式

新增关键字：

```text
*AMPLITUDE, 幅值数量
幅值ID, TABULAR, 数据点数量
时间1, 幅值1
时间2, 幅值2
...
```

验证模型使用：

```text
*AMPLITUDE, 1
1, TABULAR, 4
0.0, 0.0
1.0, 1.0
2.0, 0.713181223
3.0, 0.8
```

幅值必须在引用它的约束之前定义。未知类型、重复 ID、无效点数、非递增时间均导致输入失败。

## 4. 约束格式

保留现有四字段格式：

```text
约束ID, 节点ID, 方向, 位移值
```

增加可选的第 5 个字段：

```text
约束ID, 节点ID, 方向, 基础位移, 幅值ID
```

`Constraint` 增加：

```cpp
std::weak_ptr<TabularAmplitude> m_pAmplitude;
```

约束值计算：

```cpp
double Constraint::GetValue(double currentTime, double factor) const
{
    if (auto amplitude = m_pAmplitude.lock())
        return m_Value * amplitude->GetValue(currentTime);

    return m_Value * factor;
}
```

因此旧模型行为不变；只有五字段约束使用自定义时程。

## 5. 求解流程

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
currentTime = duration * factor;
```

它将当前时间传给约束。约束计算绝对目标位移，并直接写入约束自由度。

每个 Newton 收敛增量继续调用现有 `CommitState()`，因此非单调位移时程不会丢失塑性历史。

## 6. 验证模型

模型参数：

```text
L = 1000 mm
A = 100 mm²
E = 200000 MPa
初始屈服应力 = 250 MPa
H = 10000 MPa
```

材料使用七字段格式，最后一个字段 `H` 使其成为线性硬化塑性材料。

右端基础位移为 `5 mm`，幅值控制：

```text
t=0: u=0
t=1: u=5
t=2: u=3.565906115
t=3: u=4
```

第二阶段位移按对数应变计算，使卸载终点接近零应力。

单一分析步：

```text
1, STATIC, 3.0, 0.01, 1e-8, 100
```

预期节点反力大小约为：

```text
t=1: 28417.476 N
t=2: 0 N
t=3: 8614.700 N
```

反力符号取决于程序约定。

## 7. 文件与兼容性

预计新增：

- `YQY/DataStructure/Amplitude/TabularAmplitude.h`
- `YQY/DataStructure/Amplitude/TabularAmplitude.cpp`
- 幅值插值单元测试；
- 三阶段杆验证模型。

预计修改：

- `Constraint`：可选幅值引用和当前值计算；
- `StructureData`：幅值容器及查找；
- `EnumKeyword`、`Input_Model`：新关键字和读取逻辑；
- `IAnalysisModel`、`AnalysisStep`、`SolverStatic`：传递当前时间；
- Visual Studio 工程文件。

旧约束文件、旧力荷载时程和现有输出格式保持兼容。

## 8. 测试

至少验证：

1. 表格首端、末端和区间中点插值；
2. 表格外时间取端点值；
3. 非递增时间输入失败；
4. 四字段约束仍使用 `value × factor`；
5. 五字段约束使用 `value × amplitude(time)`；
6. 完整模型能够读取；
7. Debug x64 完整构建成功；
8. 三个检查时刻的反力趋势分别为塑性加载、零应力卸载和弹性再加载。
