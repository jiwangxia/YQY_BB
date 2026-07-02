# Constraint Time History Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a prescribed displacement constraint own a piecewise-linear TABULAR history so one static analysis step can load, unload, and reload a plastic truss.

**Architecture:** `Constraint` owns its time points and evaluates its current absolute displacement. `SolverStatic` passes pseudo-time through `IAnalysisModel` to `AnalysisStep`; existing `LoadBase` time functions and result output remain untouched.

**Tech Stack:** C++17, Qt text input, Eigen, MSBuild, assertion-based C++ test.

---

### Task 1: Add tested constraint interpolation

**Files:**
- Create: `YQY/Tests/ConstraintTimeHistoryTest.cpp`
- Modify: `YQY/DataStructure/Constraint/Constraint.h`
- Modify: `YQY/DataStructure/Constraint/Constraint.cpp`

- [ ] Write a test that verifies old linear-factor behavior, endpoint clamping, midpoint interpolation, and invalid non-increasing times.
- [ ] Run the test first and confirm compilation fails because `ConstraintTimePoint`, `SetTimePoints()`, and `GetValue()` do not exist.
- [ ] Add `ConstraintTimePoint`, `m_TimePoints`, `SetTimePoints()`, and `GetValue()` to `Constraint`.
- [ ] Implement validation and piecewise-linear interpolation in `Constraint.cpp`; do not use `inline`.
- [ ] Compile and run the standalone test with the project Qt/Eigen include paths; expect `Constraint time-history tests passed`.

### Task 2: Extend constraint input

**Files:**
- Modify: `YQY/Import/Input_Model.cpp`

- [ ] Preserve the existing four-field constraint format.
- [ ] Accept a six-field main line: `ID, NodeID, Direction, BaseValue, TABULAR, PointCount`.
- [ ] Read exactly `PointCount` following two-field lines and validate them through `Constraint::SetTimePoints()`.
- [ ] Reject unknown types, insufficient points, early keywords, non-finite data, and non-increasing time with clear Chinese errors.
- [ ] Re-run the interpolation test and full build.

### Task 3: Pass pseudo-time to constraints

**Files:**
- Modify: `YQY/Solver/Interface/IAnalysisModel.h`
- Modify: `YQY/DataStructure/AnalysisStep/AnalysisStep.h`
- Modify: `YQY/DataStructure/AnalysisStep/AnalysisStep.cpp`
- Modify: `YQY/Solver/Static/SolverStatic.cpp`

- [ ] Change `Assemble_Constraint(x1, factor)` to `Assemble_Constraint(x1, currentTime, factor)`.
- [ ] In `AnalysisStep`, call `Constraint::GetValue(currentTime, factor)` and write that absolute displacement to the node.
- [ ] In `SolverStatic`, pass its existing `currentTime`.
- [ ] Build Debug x64 and expect zero compilation errors.

### Task 4: Create and validate the benchmark model

**Files:**
- Create: `YQY/Import/ImportFile/Truss_Plastic_TimeHistory.bdf`

- [ ] Define one 1000 mm truss, area 100 mm², `E=200000`, yield stress 250, and `H=10000`.
- [ ] Fix the left node; prescribe right-node x displacement with base value 5 and TABULAR points `(0,0)`, `(1,1)`, `(2,0.713181223)`, `(3,0.8)`; fix right-node y/z.
- [ ] Define one static step with duration 3.0 and step size 0.01.
- [ ] Run the application/model reader path available in the project, or minimally verify the parser and complete solution build if no noninteractive runner exists.
- [ ] Confirm no LE/PE/PEEQ output schema was added and no `LoadBase` file changed.

### Task 5: Final verification

- [ ] Run the constraint interpolation executable.
- [ ] Run the existing `Material1D` tests.
- [ ] Build `Debug|x64` with MSBuild.
- [ ] Run `git diff --check`.
- [ ] Review the changed-file list and leave all unrelated user changes untouched and unstaged.
