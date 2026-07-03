# Two-Bar Arch Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate and verify a native model equivalent to the supplied Abaqus two-bar elastoplastic arch.

**Architecture:** Express the Abaqus geometry, material, section, displacement boundary, and static step using the program's existing BDF keywords. Verify through the existing noninteractive application path and HDF5 results without modifying source code.

**Tech Stack:** BDF-style text model, C++ solver executable, HDF5 result inspection.

---

### Task 1: Create the native model

**Files:**
- Create: `YQY/Import/ImportFile/Two_Bar_Arch_Elastoplastic_H50.bdf`

- [ ] Add three nodes and two `T3D2` elements with the Abaqus coordinates and connectivity.
- [ ] Add material line `1, 200000, 0.3, 0, 250, 0, 10000`.
- [ ] Add a `100 mm²` section and assign it to both elements.
- [ ] Add nine constraints: both supports fixed, crown X/Z fixed, and crown Y base displacement `-200 mm`.
- [ ] Bind constraint 9 using:

```text
*CONSTRAINT_TABULAR, 9, 2
0.0, 0.0
1.0, 1.0
```

- [ ] Add static step `1, STATIC, 1.0, 0.001, 1e-8, 100` and enable HDF5.

### Task 2: Verify the model

- [ ] Copy the model into an isolated temporary run directory using the filename expected by the current executable.
- [ ] Run the application and require all `1000/1000` increments to complete.
- [ ] Read `YQY/RESULT/NODAL/DISPLACEMENT` and confirm node 2 final `U2 = -200 mm`.
- [ ] Read `YQY/RESULT/NODAL/REACTION_FORCE` and confirm node 2 `R2` contains nonzero values over the path.
- [ ] Run `git diff --check` and confirm no source files were changed for this model-generation task.
