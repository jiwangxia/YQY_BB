#pragma once
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "Interface/ISolver.h"
#include "Static/SolverStatic.h"
#include "Dynamic/SolverNewmark.h"
#include "Dynamic/SolverTSSBN.h"
#include "Dynamic/SolverAdaptiveTSSBN.h"
#include <memory>

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
                if (p.numIncrements < 1) p.numIncrements = 1;
                p.maxIter = step.m_MaxIterations;
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
            return std::make_unique<SolverNewmark>(p);
        }
    };
}
