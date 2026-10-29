#include "Solver/Factory/SolverFactory.h"

#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "Solver/Dynamic/Integrators/SolverAdaptiveTSSBN.h"
#include "Solver/Dynamic/Integrators/SolverNewmark.h"
#include "Solver/Dynamic/Integrators/SolverRungeKutta4.h"
#include "Solver/Static/SolverStatic.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace SolverNameSpace
{
const std::array<DynamicAlgorithm, 6>& SolverFactory::DynamicAlgorithms()
{
    static const std::array<DynamicAlgorithm, 6> algorithms{
        {{SolverType::Newmark, "Newmark-β", {true}},
         {SolverType::AdaptiveNewmark, "Newmark-β（自适应）", {true}},
         {SolverType::RungeKutta4,
          "Runge-Kutta 4（显式）",
          {true, InitialStaticMode::Configured, false, false, true, false}},
         {SolverType::AdaptiveRungeKutta4,
          "Runge-Kutta 4（显式，自适应）",
          {true, InitialStaticMode::Configured, false, false, true, false}},
         {SolverType::AdaptiveTSSBN, "自适应 TSSBN", {true}},
         {SolverType::ReferenceTSSBN,
          "参考 TSSBN 舞动",
          {true, InitialStaticMode::ReferenceWindless, true, true, false}}}};
    return algorithms;
}

DynamicExecutionPolicy SolverFactory::GetDynamicExecutionPolicy(SolverType type)
{
    for (const DynamicAlgorithm& algorithm : DynamicAlgorithms())
    {
        if (algorithm.type == type)
            return algorithm.policy;
    }
    return {};
}

std::unique_ptr<ISolver> SolverFactory::Create(const AnalysisStep& step)
{
    switch (step.m_Type)
    {
        case EnumKeyword::StepType::STATIC:
            return CreateStaticSolver(step);
        case EnumKeyword::StepType::DYNAMIC:
            return CreateDynamicSolver(step);
        default:
            return nullptr;
    }
}

std::unique_ptr<ISolver> SolverFactory::Create_StepForSlover(const AnalysisStep& step)
{
    return Create(step);
}

std::unique_ptr<ISolver> SolverFactory::CreateStaticSolver(const AnalysisStep& step)
{
    SolverStatic::Params parameters;
    const double incrementCount = step.m_Time / step.m_StepSize;
    if (!std::isfinite(incrementCount) || incrementCount < 0.0 ||
        incrementCount > static_cast<double>(std::numeric_limits<int>::max()))
        throw std::invalid_argument("静力增量数量超出支持范围");
    parameters.numIncrements = static_cast<int>(incrementCount);
    if (parameters.numIncrements < 1)
        parameters.numIncrements = 1;
    parameters.maxIter = step.m_MaxIterations;
    parameters.tol_R = step.m_Tolerance;
    parameters.tol_dx = std::max(1.0e-10, step.m_Tolerance);
    parameters.tol_C = std::max(1.0e-12, step.m_Tolerance);
    return std::make_unique<SolverStatic>(parameters);
}

std::unique_ptr<ISolver> SolverFactory::CreateDynamicSolver(const AnalysisStep& step)
{
    if (!GetDynamicExecutionPolicy(step.m_DynamicSolverType).implemented)
        return nullptr;
    switch (step.m_DynamicSolverType)
    {
        case SolverType::AdaptiveNewmark:
            return CreateNewmarkFromStep(step, true);
        case SolverType::RungeKutta4:
            return CreateRungeKutta4FromStep(step);
        case SolverType::AdaptiveRungeKutta4:
            return CreateRungeKutta4FromStep(step, true);
        case SolverType::AdaptiveTSSBN:
            return CreateAdaptiveTssbnFromStep(step);
        case SolverType::ReferenceTSSBN:
            return CreateAdaptiveTssbnFromStep(step, true);
        case SolverType::Newmark:
            return CreateNewmarkFromStep(step);
        case SolverType::CentralDifference:
        case SolverType::HHT:
        default:
            return nullptr;
    }
}

AerodynamicTangentMode SolverFactory::ResolveAerodynamicTangentMode(const AnalysisStep& step)
{
    return step.m_EnableGalloping ? step.m_GallopingAerodynamicTangentMode : AerodynamicTangentMode::Disabled;
}

std::unique_ptr<ISolver> SolverFactory::CreateNewmarkFromStep(const AnalysisStep& step, bool adaptive)
{
    SolverNewmark::Params parameters;
    parameters.dt = step.m_StepSize;
    parameters.maxIter = step.m_MaxIterations;
    parameters.tol = step.m_Tolerance;
    parameters.aerodynamicTangentMode = ResolveAerodynamicTangentMode(step);
    parameters.timeStepMode = adaptive ? TimeStepMode::Adaptive : TimeStepMode::Fixed;
    if (adaptive)
        parameters.adaptiveTimeStep = step.m_AdaptiveTimeStep;
    return std::make_unique<SolverNewmark>(parameters);
}

std::unique_ptr<ISolver> SolverFactory::CreateRungeKutta4FromStep(const AnalysisStep& step, bool adaptive)
{
    SolverRungeKutta4::Params parameters;
    parameters.timeStep = step.m_StepSize;
    parameters.timeStepMode = adaptive ? TimeStepMode::Adaptive : TimeStepMode::Fixed;
    if (adaptive)
        parameters.adaptiveTimeStep = step.m_AdaptiveTimeStep;
    return std::make_unique<SolverRungeKutta4>(parameters);
}

std::unique_ptr<ISolver> SolverFactory::CreateAdaptiveTssbnFromStep(const AnalysisStep& step, bool referenceGalloping)
{
    SolverAdaptiveTSSBN::Params parameters(step.m_StepSize, step.m_Tolerance, step.m_MaxIterations,
                                           step.m_AdaptiveTssbn);
    parameters.aerodynamicTangentMode = ResolveAerodynamicTangentMode(step);
    parameters.diagnosticCacheExternalForcePerStage = step.m_DiagnosticTssbnCacheExternalForcePerStage;
    parameters.diagnosticErrorTrace = step.m_DiagnosticTssbnErrorTrace;
    if (referenceGalloping)
    {
        // 参考路径单独解析数值策略，不改写分析步或共享荷载的原有设置。
        parameters.referenceGalloping = true;
        parameters.errorNormMode = parameters.referenceIndependentErrorControl ? TssbnErrorNormMode::ComponentwiseMax
                                                                               : TssbnErrorNormMode::LegacyGlobal;
        parameters.useDuan2026Controller = true;
        parameters.useReferenceExtrapolatedAcceleration = !parameters.referenceIndependentErrorControl;
        if (!parameters.referenceIndependentErrorControl)
            parameters.velocityAbsoluteTolerance = parameters.absoluteTolerance;
        // 参考 Newton 只用结构和惯性切线，气动力仍随每次试探状态重新计算。
        parameters.aerodynamicTangentMode = AerodynamicTangentMode::Disabled;
        parameters.diagnosticCacheExternalForcePerStage = false;
    }
    return std::make_unique<SolverAdaptiveTSSBN>(parameters);
}
}
