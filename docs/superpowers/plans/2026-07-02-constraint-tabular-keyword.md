# Constraint TABULAR Keyword Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the inline six-field constraint history syntax with `*CONSTRAINT_TABULAR, ConstraintID, PointCount`.

**Architecture:** Keep interpolation and solver time propagation inside the existing `Constraint` path. Add one input keyword that resolves an already-created constraint by ID, validates that it has no history yet, reads its points, and calls `SetTimePoints()`.

**Tech Stack:** C++17, Qt text input, MSBuild, HDF5 acceptance result.

---

### Task 1: Make the benchmark express the new syntax

**Files:**
- Modify: `YQY/Import/ImportFile/Truss_Plastic_TimeHistory.bdf`

- [ ] Replace the inline line:

```text
4, 2, 0, 5.0, TABULAR, 4
```

with:

```text
4, 2, 0, 5.0
```

and place this keyword after all six constraint lines:

```text
*CONSTRAINT_TABULAR, 4, 4
0.0, 0.0
1.0, 1.0
2.0, 0.713181223
3.0, 0.8
```

- [ ] Run the current executable and verify the acceptance reactions do not match the expected three-stage values because the keyword is not implemented yet.

### Task 2: Add keyword recognition and strict parser behavior

**Files:**
- Modify: `YQY/Utility/EnumKeyword.h`
- Modify: `YQY/Utility/EnumKeyword.cpp`
- Modify: `YQY/Import/Input_Model.h`
- Modify: `YQY/Import/Input_Model.cpp`
- Modify: `YQY/DataStructure/Constraint/Constraint.h`

- [ ] Add `CONSTRAINT_TABULAR` to `EnumKeyword::KeyData` and `MapKeyData`.
- [ ] Add `bool Constraint::HasTimePoints() const` returning `!m_TimePoints.empty()` so duplicate bindings can be rejected without exposing the vector.
- [ ] Restore `InputConstraint()` to exactly four fields and remove all inline TABULAR parsing.
- [ ] Declare and implement:

```cpp
bool InputConstraintTabular(
    QTextStream& flow,
    const QStringList& list_str);
```

Require exactly three keyword fields, parse a positive constraint ID and at least two points, find the constraint in `m_Structure->m_Constraint`, reject an unknown ID or `HasTimePoints() == true`, read exactly two finite numbers per point, then call `SetTimePoints()`.
- [ ] Dispatch `KeyData::CONSTRAINT_TABULAR` from `InputData()` after the normal constraint case.
- [ ] Build `Debug|x64`; expect zero compiler errors.

### Task 3: Verify parser errors and benchmark reactions

**Files:**
- Modify: `YQY/Tests/ConstraintTimeHistoryTest.cpp`

- [ ] Extend the standalone constraint test to verify `HasTimePoints()` is false initially and true after `SetTimePoints()`.
- [ ] Run the constraint test and expect `Constraint time-history tests passed`.
- [ ] Run `Material1DTest` and expect `Material1D tests passed`.
- [ ] Run the benchmark through the application and inspect HDF5 node-2 x reactions:

```text
t=1.00: 28417.475964 N
t=2.00: approximately 0 N
t=3.00: 8614.699715 N
```

- [ ] Run `git diff --check` and confirm `LoadBase` files and plastic-result schemas are unchanged.
