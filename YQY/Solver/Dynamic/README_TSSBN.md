# TSSBN Dynamic Solver

TSSBN is selected through an `AnalysisStep`. Do not create solver objects
directly from feature-specific factory functions.

```cpp
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "Solver/SolverFactory.h"

AnalysisStep step;
step.m_Type = EnumKeyword::StepType::DYNAMIC;
step.m_DynamicSolverType = SolverNameSpace::SolverType::AdaptiveTSSBN;
step.m_StepSize = 0.01;
step.m_MaxIterations = 10;
step.m_Tolerance = 1e-6;

auto solver = SolverNameSpace::SolverFactory::Create_StepForSlover(step);
```

## 气动力数值切线模式

Newmark 和自适应 TSSBN 共用三种气动力切线更新策略：

```cpp
step.m_EnableGalloping = true;
step.m_GallopingAerodynamicTangentMode =
    SolverNameSpace::AerodynamicTangentMode::EveryNewtonIteration;
```

- `Disabled`：每轮 Newton 仍更新气动力，但不组装气动力导数，属于准 Newton；
- `OncePerTimeStepOrStage`：Newmark 每个时间步组装一次；TSSBN 的 C1、C2 每个隐式阶段组装一次；
- `EveryNewtonIteration`：每轮 Newton 都重新组装，为新建舞动工况的默认值。

该选项属于舞动工况，只在启用舞动气动力时生效，但不与积分器类型绑定。长时程舞动效率计算可先使用 `Disabled`，再用
`OncePerTimeStepOrStage` 和 `EveryNewtonIteration` 对关键工况做响应与收敛性复核。

The normal application path is:

```text
AnalysisRunner -> AnalysisStep::Solve -> SolverFactory::CreateForStep -> ISolver::Solve
```
