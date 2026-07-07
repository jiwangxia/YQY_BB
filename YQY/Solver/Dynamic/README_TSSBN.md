# TSSBN Dynamic Solver

TSSBN is selected through an `AnalysisStep`. Do not create solver objects
directly from feature-specific factory functions.

```cpp
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "Solver/SolverFactory.h"

AnalysisStep step;
step.m_Type = EnumKeyword::StepType::DYNAMIC;
step.m_DynamicSolverType = SolverNameSpace::SolverType::TSSBN;
step.m_StepSize = 0.01;
step.m_MaxIterations = 10;
step.m_Tolerance = 1e-6;

auto solver = SolverNameSpace::SolverFactory::CreateForStep(step);
```

The normal application path is:

```text
AnalysisRunner -> AnalysisStep::Solve -> SolverFactory::CreateForStep -> ISolver::Solve
```
