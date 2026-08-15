#pragma once
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "Interface/ISolver.h"
#include "Static/SolverStatic.h"
#include "Dynamic/SolverNewmark.h"
#include "Dynamic/SolverAdaptiveTSSBN.h"
#include <memory>
#include <algorithm>

namespace SolverNameSpace
{
class SolverFactory
{
public:
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
    static std::unique_ptr<ISolver> CreateDynamicSolver(const AnalysisStep& step)
    {
        switch (step.m_DynamicSolverType)
        {
            case SolverType::Newmark:
                return CreateNewmarkFromStep(step);
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

    static std::unique_ptr<SolverNewmark> CreateNewmarkFromStep(const AnalysisStep& step)
    {
        SolverNewmark::Params p;
        p.dt = step.m_StepSize;
        p.maxIter = step.m_MaxIterations;
        p.tol = step.m_Tolerance;
        p.aerodynamicTangentMode = step.m_EnableGalloping ? step.m_GallopingAerodynamicTangentMode
                                                         : AerodynamicTangentMode::Disabled;
        return std::make_unique<SolverNewmark>(p);
    }

    static std::unique_ptr<SolverAdaptiveTSSBN> CreateAdaptiveTssbnFromStep(const AnalysisStep& step)
    {
        SolverAdaptiveTSSBN::Params p(step.m_StepSize, step.m_Tolerance, step.m_MaxIterations, step.m_AdaptiveTssbn);
        p.aerodynamicTangentMode = step.m_EnableGalloping ? step.m_GallopingAerodynamicTangentMode
                                                         : AerodynamicTangentMode::Disabled;
        return std::make_unique<SolverAdaptiveTSSBN>(p);
    }
};
}
