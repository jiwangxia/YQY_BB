/**
 * @file SolverFactory.h
 * @brief 求解器工厂 - 根据类型创建对应求解器
 */
#pragma once
#include "Interface/ISolver.h"
#include "Static/SolverStatic.h"
#include "Dynamic/SolverNewmark.h"

namespace Solver
{
    /**
     * @brief 创建求解器
     * 
     * 工厂函数：根据求解器类型创建对应的求解器实例
     * 
     * @param[in] type 求解器类型
     * @param[in] stepSize 时间/荷载步长
     * @param[in] tolerance 收敛容差
     * @param[in] maxIterations 最大迭代次数
     * @return 求解器唯一指针
     * 
     * @code
     * // 使用示例
     * auto solver = Solver::CreateSolver(Solver::SolverType::Static, 0.1, 1e-5, 32);
     * solver->Solve(model, 1.0);
     * @endcode
     */
    inline std::unique_ptr<ISolver> CreateSolver(SolverType type, double stepSize,
                                                  double tolerance, int maxIterations)
    {
        switch (type)
        {
        case SolverType::Static:
        {
            SolverStatic::Params p;
            p.numIncrements = static_cast<int>(1.0 / stepSize);  // stepSize 作为因子
            if (p.numIncrements < 1) p.numIncrements = 1;
            p.tol = tolerance;
            p.maxIter = maxIterations;
            return std::make_unique<SolverStatic>(p);
        }

        case SolverType::Newmark:
        {
            SolverNewmark::Params p;
            p.dt = stepSize;
            p.tol = tolerance;
            p.maxIter = maxIterations;
            return std::make_unique<SolverNewmark>(p);
        }

        case SolverType::CentralDifference:
            // TODO: 未来扩展
            throw std::runtime_error("CentralDifference solver not implemented yet");

        case SolverType::HHT:
            // TODO: 未来扩展
            throw std::runtime_error("HHT solver not implemented yet");

        default:
            throw std::runtime_error("Unknown solver type");
        }
    }

    /**
     * @brief 根据分析步类型创建求解器
     * 
     * 便捷函数：自动根据 StepType 选择对应求解器
     */
    inline std::unique_ptr<ISolver> CreateSolverForStepType(int stepType, double stepSize,
                                                             double tolerance, int maxIterations)
    {
        // stepType: 0=STATIC, 1=DYNAMIC (与 EnumKeyword::StepType 对应)
        if (stepType == 0)  // STATIC
        {
            return CreateSolver(SolverType::Static, stepSize, tolerance, maxIterations);
        }
        else if (stepType == 1)  // DYNAMIC
        {
            return CreateSolver(SolverType::Newmark, stepSize, tolerance, maxIterations);
        }
        return nullptr;
    }
}
