# Truss Plasticity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add tested one-dimensional bilinear elastoplastic behavior with initial stress and converged-step state commits to `ElementTruss`.

**Architecture:** `Material::Update1D()` is the element-facing constitutive interface. `Material1D.h` contains only the state/result declarations, while the Qt-independent return-mapping algorithm lives in `Material1D.cpp`; each truss owns old/trial states, while shared `Material` objects own only parameters.

**Tech Stack:** C++17, Eigen, Qt/MSBuild, standalone assertion-based C++ tests.

---

## File structure

- Create `YQY/DataStructure/Material/Material1D.h`: one-dimensional state/result types and function declaration.
- Create `YQY/DataStructure/Material/Material1D.cpp`: pure constitutive update implementation.
- Create `YQY/Tests/Material1DTest.cpp`: standalone tests for elastic, initial-stress, yielding, hardening, unloading, and repeat-trial behavior.
- Modify `YQY/DataStructure/Material/Material.h/.cpp`: expose `Update1D()` and rename the yield field.
- Modify `YQY/DataStructure/Element/ElementBase.h`: default no-op state commit hook.
- Modify `YQY/DataStructure/Element/ElementTruss.h/.cpp`: own old/trial material states and use returned stress/stiffness.
- Modify `YQY/Solver/Interface/IAnalysisModel.h`, `YQY/DataStructure/AnalysisStep/AnalysisStep.h/.cpp`: expose and implement converged-state commit.
- Modify static and dynamic solver `.cpp` files: commit only accepted/converged steps.
- Modify import/export files: read optional hardening modulus and preserve serialized `MAX_STRESS` compatibility.
- Modify `YQY/Export/Hdf5FileFormat.md`: document that legacy `MAX_STRESS` stores initial yield stress.

### Task 1: Test and implement the pure 1D material update

**Files:**
- Create: `YQY/Tests/Material1DTest.cpp`
- Create: `YQY/DataStructure/Material/Material1D.h`
- Create: `YQY/DataStructure/Material/Material1D.cpp`

- [ ] **Step 1: Write the failing standalone test**

Create tests that call `CalculateMaterial1D()` with `E=200000`, `yield=200`, and verify:

```cpp
// Initial stress remains part of the total stress.
Material1DState oldState;
oldState.stress = 50.0;
const auto elastic = CalculateMaterial1D(
    MaterialModel::Elastic, 200000.0, 0.0, 0.0, 0.001, oldState);
CheckNear(elastic.stress, 250.0);
CheckNear(elastic.stiffness, 200000.0);

// Perfect plasticity returns to the yield surface.
const auto perfect = CalculateMaterial1D(
    MaterialModel::IdealPlastic1D, 200000.0, 200.0, 0.0, 0.002, {});
CheckNear(perfect.stress, 200.0);
CheckNear(perfect.stiffness, 0.0);

// Calling twice from the same old state must not accumulate history twice.
const auto repeat = CalculateMaterial1D(
    MaterialModel::IdealPlastic1D, 200000.0, 200.0, 0.0, 0.002, {});
CheckNear(repeat.state.plasticStrain, perfect.state.plasticStrain);
```

Also cover linear hardening, unloading with slope `E`, reverse yielding, and rejection of invalid parameters/initial stress.

- [ ] **Step 2: Compile the test and verify it fails**

Run from a Visual Studio developer command environment:

```powershell
cmd /d /s /c """D:\Visual Studio2022\Community\VC\Auxiliary\Build\vcvars64.bat"" && cl /nologo /std:c++17 /EHsc /IYQY YQY\Tests\Material1DTest.cpp /Fe:%TEMP%\Material1DTest.exe"
```

Expected: compilation fails because `DataStructure/Material/Material1D.h` does not exist.

- [ ] **Step 3: Implement `Material1D.h` and `Material1D.cpp`**

Define:

```cpp
enum class MaterialModel
{
    Elastic,
    IdealPlastic1D,
    HardeningPlastic1D
};

struct Material1DState
{
    double strain = 0.0;
    double stress = 0.0;
    double plasticStrain = 0.0;
    double accumulatedPlastic = 0.0;
};

struct Material1DResult
{
    double stress = 0.0;
    double stiffness = 0.0;
    Material1DState state;
};

Material1DResult CalculateMaterial1D(
    MaterialModel model,
    double young,
    double yieldStress,
    double hardening,
    double strain,
    const Material1DState& oldState);
```

The `.cpp` implementation validates finite values, `young > 0`, plastic `yieldStress > 0`, `hardening >= 0`, and `abs(oldState.stress) <= yieldStress + tolerance` only for a pristine initial plastic state. It performs the elastic predictor and one-dimensional return mapping from the design spec, assigns current strain/stress into the returned state, and throws `std::runtime_error` with clear English core messages for the standalone layer.

- [ ] **Step 4: Compile and run the standalone tests**

Run:

```powershell
cmd /d /s /c """D:\Visual Studio2022\Community\VC\Auxiliary\Build\vcvars64.bat"" && cl /nologo /std:c++17 /utf-8 /EHsc /IYQY YQY\Tests\Material1DTest.cpp YQY\DataStructure\Material\Material1D.cpp /Fe:%TEMP%\Material1DTest.exe && %TEMP%\Material1DTest.exe"
```

Expected: `Material1D tests passed`.

### Task 2: Add the material-facing interface and update material input/output

**Files:**
- Modify: `YQY/DataStructure/Material/Material.h`
- Modify: `YQY/DataStructure/Material/Material.cpp`
- Modify: `YQY/Import/Input_Model.cpp`
- Modify: `YQY/DataStructure/Structure/StructureData.cpp`
- Modify: `YQY/Export/Outputter.cpp`
- Modify: `YQY/Export/Hdf5ResultIO.cpp`
- Modify: `YQY/Export/Hdf5FileFormat.md`

- [ ] **Step 1: Add `Material::Update1D()`**

Include `Material1D.h`, rename `m_MaxStress` to:

```cpp
double m_YieldStress = 0.0; ///< 初始屈服应力
```

Add:

```cpp
MaterialModel m_Model = MaterialModel::Elastic;
double m_Hardening = 0.0; ///< 线性硬化模量，0 表示理想塑性

/**
 * @brief 根据杆单元当前总应变计算一维材料试算响应
 * @param strain 当前总应变
 * @param oldState 上一个已收敛增量步的材料状态
 */
Material1DResult Update1D(
    double strain,
    const Material1DState& oldState) const;
```

Implement it as a clear delegation to `CalculateMaterial1D()` using the material parameters.

- [ ] **Step 2: Extend material input without breaking six-field files**

Change `InputMaterial()` to accept exactly 6 or 7 fields. Use descriptive locals:

```cpp
const double young = fields[1].toDouble();
const double poisson = fields[2].toDouble();
const double density = fields[3].toDouble();
const double yieldStress = fields[4].toDouble();
const double expansion = fields[5].toDouble();
```

For 6 fields set `m_Model = Elastic`. For 7 fields parse `hardening`, require `yieldStress > 0` and `hardening >= 0`, then select `IdealPlastic1D` for `H = 0` or `HardeningPlastic1D` for `H > 0`. Keep `yieldStress` stored for both formats.

- [ ] **Step 3: Rename internal field references**

Replace all internal `m_MaxStress` references with `m_YieldStress`. Keep the HDF5 member name `MAX_STRESS` and `MaterialRecord::maxStress` unchanged for file compatibility, but source their values from `m_YieldStress`. Change nearby text comments from `MaxStress` to `YieldStress`.

- [ ] **Step 4: Document compatibility**

In `Hdf5FileFormat.md`, state that the legacy `MAX_STRESS` member contains the initial yield stress. Do not rename the serialized member.

- [ ] **Step 5: Re-run standalone material tests**

Expected: `Material1D tests passed`.

### Task 3: Connect `ElementTruss` to the material interface

**Files:**
- Modify: `YQY/DataStructure/Element/ElementBase.h`
- Modify: `YQY/DataStructure/Element/ElementTruss.h`
- Modify: `YQY/DataStructure/Element/ElementTruss.cpp`

- [ ] **Step 1: Add the state lifecycle hook**

Add to `ElementBase`:

```cpp
/**
 * @brief 提交当前增量步已经收敛的单元内部状态
 */
virtual void CommitState() {}
```

- [ ] **Step 2: Add truss-owned material states**

Add private members and simple Chinese comments:

```cpp
Material1DState m_OldState;   ///< 上一个已收敛增量步状态
Material1DState m_TrialState; ///< 当前 Newton 迭代试算状态
bool m_StateInitialized = false;

void InitializeState();
```

Add public `void CommitState() override;`.

- [ ] **Step 3: Initialize and commit without losing prestress**

Implement:

```cpp
void ElementTruss::InitializeState()
{
    if (m_StateInitialized) return;
    m_OldState.stress = m_InitStress;
    m_TrialState = m_OldState;
    m_StateInitialized = true;
}

void ElementTruss::CommitState()
{
    if (m_StateInitialized)
        m_OldState = m_TrialState;
}
```

- [ ] **Step 4: Replace direct elastic stress calculation**

In both engineering-strain and logarithmic-strain paths:

```cpp
InitializeState();
const auto result = pMaterial->Update1D(strain, m_OldState);
m_TrialState = result.state;
m_Stress = result.stress;
const double D = result.stiffness;
```

Use `D` in place of `E` only in material stiffness. Continue using current total stress in axial force and geometric stiffness. Add Chinese comments explaining that repeated Newton calls always start from `m_OldState`.

- [ ] **Step 5: Build the solution**

Run:

```powershell
& 'D:\Visual Studio2022\Community\MSBuild\Current\Bin\MSBuild.exe' 'YQY.sln' /t:Build /p:Configuration=Debug /p:Platform=x64 /m
```

Expected: build succeeds with zero compiler errors.

### Task 4: Commit material history only after accepted steps

**Files:**
- Modify: `YQY/Solver/Interface/IAnalysisModel.h`
- Modify: `YQY/DataStructure/AnalysisStep/AnalysisStep.h`
- Modify: `YQY/DataStructure/AnalysisStep/AnalysisStep.cpp`
- Modify: `YQY/Solver/Static/SolverStatic.cpp`
- Modify: `YQY/Solver/Dynamic/SolverNewmark.cpp`
- Modify: `YQY/Solver/Dynamic/SolverTSSBN.cpp`
- Modify: `YQY/Solver/Dynamic/SolverAdaptiveTSSBN.cpp`

- [ ] **Step 1: Add the model commit interface**

Add:

```cpp
/**
 * @brief 提交当前已经收敛或接受的增量步内部状态
 */
virtual void CommitState() = 0;
```

Declare the corresponding override in `AnalysisStep`.

- [ ] **Step 2: Commit all element states**

Implement:

```cpp
void AnalysisStep::CommitState()
{
    for (auto& elementPair : m_pData->m_Elements)
        elementPair.second->CommitState();
}
```

- [ ] **Step 3: Call commit at accepted-step boundaries**

Insert `model.CommitState();` immediately before `model.OnStepCompleted(...)` in:

- the static solver after Newton convergence;
- Newmark after the step converges;
- TSSBN after both substeps and final state update succeed;
- adaptive TSSBN only inside the `accepted` branch.

Do not call it in rejected or failed branches.

- [ ] **Step 4: Build and run material tests**

Expected: full solution build succeeds and `Material1D tests passed`.

### Task 5: Final verification

**Files:**
- Verify all files above.

- [ ] **Step 1: Check changed code for accidental user-change inclusion**

Run:

```powershell
git diff --check
git status --short
```

Review overlapping dirty files carefully. Do not revert or stage unrelated user changes.

- [ ] **Step 2: Run the standalone constitutive tests**

Expected: `Material1D tests passed`.

- [ ] **Step 3: Build the full Debug x64 solution**

Expected: MSBuild reports `0 Error(s)`.

- [ ] **Step 4: Inspect compatibility-sensitive output**

Confirm source-code references use `m_YieldStress`, while serialized HDF5 member `MAX_STRESS` remains unchanged and documented.

- [ ] **Step 5: Report verification evidence**

Report the test executable result, build result, files changed, and any pre-existing workspace changes that were deliberately left untouched. Do not commit implementation files because several target files already contain unrelated user edits; this avoids bundling those edits into an implementation commit.
