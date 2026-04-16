# TSSBN 和 自适应TSSBN 求解器使用说明

## 概述

本项目新增了两种动力学求解器：
1. **SolverTSSBN** - TSSBN (Two-Stage Single-Step Block Newmark) 方法
2. **SolverAdaptiveTSSBN** - 自适应TSSBN方法（带误差估计和自适应步长控制）

这两个求解器与现有的 `SolverNewmark` 接口完全一致，都实现了 `ISolver` 接口。

## 文件结构

```
YQY/Solver/
├── Interface/
│   ├── ISolver.h                    (已修改：添加了 TSSBN 和 AdaptiveTSSBN 类型)
│   └── IAnalysisModel.h
├── Static/
│   ├── SolverStatic.h
│   └── SolverStatic.cpp
├── Dynamic/
│   ├── SolverNewmark.h              (已有)
│   ├── SolverNewmark.cpp
│   ├── SolverTSSBN.h                (新增)
│   ├── SolverTSSBN.cpp              (新增)
│   ├── SolverAdaptiveTSSBN.h        (新增)
│   └── SolverAdaptiveTSSBN.cpp      (新增)
└── SolverFactory.h                  (新增：统一创建求解器)
```

## 使用方法

### 方法1：使用工厂类创建（推荐）

```cpp
#include "Solver/SolverFactory.h"

// 创建 TSSBN 求解器
auto solver = SolverNS::SolverFactory::CreateTSSBNSolver(
    0.01,    // dt: 时间步长
    0.9,     // rho_inf: 高频耗散参数 (0 <= rho_inf < 1)
    10,      // maxIter: 最大迭代次数
    1e-6     // tol: 收敛容差
);

// 创建自适应 TSSBN 求解器
auto adaptiveSolver = SolverNS::SolverFactory::CreateAdaptiveTSSBNSolver(
    0.01,    // dt: 初始时间步长
    0.9,     // rho_inf: 高频耗散参数
    1e-3,    // eps_LTE: 局部截断误差容限
    1e-6,    // dt_min: 最小步长
    0.1,     // dt_max: 最大步长
    10,      // maxIter: 最大迭代次数
    1e-6     // tol: 收敛容差
);

// 使用求解器
solver->Solve(model, duration);
```

## 参数说明

### TSSBN 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| dt | double | 0.01 | 时间步长 |
| rho_inf | double | 0.9 | 高频耗散参数 (0 ≤ rho_inf < 1) |
| maxIter | int | 10 | 每个子步的最大迭代次数 |
| tol | double | 1e-6 | 收敛容差 |

### 自适应TSSBN 参数

除了包含所有TSSBN参数外，还有：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| eps_LTE | double | 1e-3 | 局部截断误差容限 |
| dt_min | double | 1e-6 | 最小时间步长 |
| dt_max | double | 0.1 | 最大时间步长 |

## 算法特点

### TSSBN 方法

1. **两阶段结构**：C1 子步 + C2 子步
2. **高频耗散控制**：通过 rho_inf 参数控制
3. **无条件稳定**：对于线性问题

### 自适应TSSBN 方法

1. **误差估计**：使用嵌入式公式（2阶+3阶）
2. **自适应步长控制**：基于PI控制器
3. **统计信息**：总步数、拒绝步数、平均步长

## 与 Newmark 的对比

| 特性 | Newmark | TSSBN | 自适应TSSBN |
|------|---------|-------|-------------|
| 时间积分方式 | 单步 | 两阶段单步 | 两阶段单步+误差估计 |
| 高频耗散 | 无 | 可控 | 可控 |
| 步长控制 | 固定 | 固定 | 自适应 |
| 计算成本 | 低 | 中 | 高 |

## 使用建议

1. **选择 Newmark**：一般动力问题，对效率要求高
2. **选择 TSSBN**：存在高频振荡，刚性问题
3. **选择自适应TSSBN**：复杂非线性，对精度要求高

## 未来扩展

如果需要添加新的动力学方法，只需：
1. 创建新的求解器类继承 `ISolver`
2. 在 `ISolver.h` 中添加新的 `SolverType`
3. 在 `SolverFactory.h` 中添加创建逻辑

接口完全统一，易于扩展！
