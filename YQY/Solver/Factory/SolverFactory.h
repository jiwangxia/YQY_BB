#pragma once
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "../Interface/ISolver.h"
#include "../Static/SolverStatic.h"
#include "../Dynamic/Integrators/SolverNewmark.h"
#include "../Dynamic/Integrators/SolverRungeKutta4.h"
#include "../Dynamic/Integrators/SolverAdaptiveTSSBN.h"
#include <memory>
#include <algorithm>

namespace SolverNameSpace
{
// 根据分析步配置创建对应求解器。
class SolverFactory
{
public:
    // 创建一个分析步的求解器实例。
    static std::unique_ptr<ISolver> Create_StepForSlover(const AnalysisStep& step)
    {
        switch (step.m_Type)
        {
            case EnumKeyword::StepType::STATIC:
            {
                SolverStatic::Params p;
                p.numIncrements = static_cast<int>(step.m_Time / step.m_StepSize);
                if (p.numIncrements < 1)
                    p.numIncrements = 1;
                p.maxIter = step.m_MaxIterations;
                p.tol_R = step.m_Tolerance;
                p.tol_dx = std::max(1.0e-10, step.m_Tolerance);
                p.tol_C = std::max(1.0e-12, step.m_Tolerance);
                return std::make_unique<SolverStatic>(p);
            }
            case EnumKeyword::StepType::DYNAMIC:
                return CreateDynamicSolver(step);
            default:
                return nullptr;
        }
    }

private:
    // 按动力算法类型选择动力求解器。
    static std::unique_ptr<ISolver> CreateDynamicSolver(const AnalysisStep& step)
    {
        switch (step.m_DynamicSolverType)
        {
            case SolverType::Newmark:
                return CreateNewmarkFromStep(step);
            case SolverType::AdaptiveNewmark:
                return CreateNewmarkFromStep(step, true);
            case SolverType::RungeKutta4:
                return CreateRungeKutta4FromStep(step);
            case SolverType::AdaptiveRungeKutta4:
                return CreateRungeKutta4FromStep(step, true);
            case SolverType::CentralDifference:
                return CreateNewmarkFromStep(step);
            case SolverType::HHT:
                return CreateNewmarkFromStep(step);
            case SolverType::AdaptiveTSSBN:
                return CreateAdaptiveTssbnFromStep(step);
            default:
                return CreateNewmarkFromStep(step);
        }
    }

    // 将分析步参数转换为 Newmark 求解器参数。
    static std::unique_ptr<SolverNewmark> CreateNewmarkFromStep(const AnalysisStep& step, bool adaptive = false)
    {
        SolverNewmark::Params p;
        p.dt = step.m_StepSize;
        p.maxIter = step.m_MaxIterations;
        p.tol = step.m_Tolerance;
        p.aerodynamicTangentMode = step.m_EnableGalloping ? step.m_GallopingAerodynamicTangentMode
                                                         : AerodynamicTangentMode::Disabled;
        p.timeStepMode = adaptive ? TimeStepMode::Adaptive : TimeStepMode::Fixed;
        if (adaptive)
            p.adaptiveTimeStep.maximumTimeStep = std::max(step.m_StepSize, p.adaptiveTimeStep.maximumTimeStep);
        return std::make_unique<SolverNewmark>(p);
    }

    static std::unique_ptr<SolverRungeKutta4> CreateRungeKutta4FromStep(const AnalysisStep& step,
                                                                          bool adaptive = false)
    {
        SolverRungeKutta4::Params parameters;
        parameters.timeStep = step.m_StepSize;
        parameters.timeStepMode = adaptive ? TimeStepMode::Adaptive : TimeStepMode::Fixed;
        if (adaptive)
            parameters.adaptiveTimeStep.maximumTimeStep =
                std::max(step.m_StepSize, parameters.adaptiveTimeStep.maximumTimeStep);
        return std::make_unique<SolverRungeKutta4>(parameters);
    }

    // 将分析步参数转换为自适应 TSSBN 求解器参数。
    static std::unique_ptr<SolverAdaptiveTSSBN> CreateAdaptiveTssbnFromStep(const AnalysisStep& step)
    {
        SolverAdaptiveTSSBN::Params p(step.m_StepSize, step.m_Tolerance, step.m_MaxIterations, step.m_AdaptiveTssbn);
        p.aerodynamicTangentMode = step.m_EnableGalloping ? step.m_GallopingAerodynamicTangentMode
                                                         : AerodynamicTangentMode::Disabled;
        return std::make_unique<SolverAdaptiveTSSBN>(p);
    }
};
}
