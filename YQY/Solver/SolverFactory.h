/**
 * @file SolverFactory.h
 * @brief 求解器工厂 - 统一创建各种求解器
 */
#pragma once
#include "Interface/ISolver.h"
#include "Static/SolverStatic.h"
#include "Dynamic/SolverNewmark.h"
#include "Dynamic/SolverTSSBN.h"
#include "Dynamic/SolverAdaptiveTSSBN.h"
#include <memory>

namespace SolverNameSpace
{
    /**
     * @brief 求解器工厂类
     *
     * 提供统一的接口创建各种类型的求解器
     */
    class SolverFactory
    {
    public:
        /**
         * @brief 创建求解器
         * @param[in] type 求解器类型
         * @param[in] stepSize 时间步长或增量步长
         * @param[in] tolerance 收敛容差
         * @param[in] maxIterations 最大迭代次数
         * @return 求解器智能指针
         */
        static std::unique_ptr<ISolver> CreateSolver(
            SolverType type,
            double stepSize = 0.01,
            double tolerance = 1e-5,
            int maxIterations = 32)
        {
            switch (type)
            {
                case SolverType::Static:
                {
                    SolverStatic::Params p;
                    p.numIncrements = 10;
                    p.maxIter = maxIterations;
                    p.tol = tolerance;
                    p.tol_R = 1e-3;
                    p.tol_dx = tolerance;
                    return std::make_unique<SolverStatic>(p);
                }

                case SolverType::Newmark:
                {
                    SolverNewmark::Params p;
                    p.dt = stepSize;
                    p.beta = 0.25;
                    p.gamma = 0.5;
                    p.maxIter = maxIterations;
                    p.tol = tolerance;
                    return std::make_unique<SolverNewmark>(p);
                }

                case SolverType::TSSBN:
                {
                    SolverTSSBN::Params p;
                    p.dt = stepSize;
                    p.rho_inf = 0.9;  // 默认高频耗散参数
                    p.maxIter = maxIterations;
                    p.tol = tolerance;
                    return std::make_unique<SolverTSSBN>(p);
                }

                case SolverType::AdaptiveTSSBN:
                {
                    SolverAdaptiveTSSBN::Params p;
                    p.dt = stepSize;
                    p.rho_inf = 0.9;
                    p.maxIter = maxIterations;
                    p.tol = tolerance;
                    p.eps_LTE = 1e-3;  // 局部截断误差容限
                    p.dt_min = stepSize * 0.01;
                    p.dt_max = stepSize * 10.0;
                    return std::make_unique<SolverAdaptiveTSSBN>(p);
                }

                case SolverType::CentralDifference:
                case SolverType::HHT:
                default:
                    return nullptr;
            }
        }

        /**
         * @brief 创建静力求解器
         * @param[in] numIncrements 荷载增量步数
         * @param[in] maxIter 最大迭代次数
         * @param[in] tol 收敛容差
         * @return 静力求解器智能指针
         */
        static std::unique_ptr<SolverStatic> CreateStaticSolver(
            int numIncrements = 10,
            int maxIter = 32,
            double tol = 1e-5)
        {
            SolverStatic::Params p;
            p.numIncrements = numIncrements;
            p.maxIter = maxIter;
            p.tol = tol;
            p.tol_R = 1e-3;
            p.tol_dx = tol;
            return std::make_unique<SolverStatic>(p);
        }

        /**
         * @brief 创建Newmark动力求解器
         * @param[in] dt 时间步长
         * @param[in] beta Newmark β参数
         * @param[in] gamma Newmark γ参数
         * @param[in] maxIter 最大迭代次数
         * @param[in] tol 收敛容差
         * @return Newmark求解器智能指针
         */
        static std::unique_ptr<SolverNewmark> CreateNewmarkSolver(
            double dt = 0.01,
            double beta = 0.25,
            double gamma = 0.5,
            int maxIter = 10,
            double tol = 1e-6)
        {
            SolverNewmark::Params p;
            p.dt = dt;
            p.beta = beta;
            p.gamma = gamma;
            p.maxIter = maxIter;
            p.tol = tol;
            return std::make_unique<SolverNewmark>(p);
        }

        /**
         * @brief 创建TSSBN动力求解器
         * @param[in] dt 时间步长
         * @param[in] rho_inf 高频耗散参数 (0 <= rho_inf < 1)
         * @param[in] maxIter 最大迭代次数
         * @param[in] tol 收敛容差
         * @return TSSBN求解器智能指针
         */
        static std::unique_ptr<SolverTSSBN> CreateTSSBNSolver(
            double dt = 0.01,
            double rho_inf = 0.9,
            int maxIter = 10,
            double tol = 1e-6)
        {
            SolverTSSBN::Params p;
            p.dt = dt;
            p.rho_inf = rho_inf;
            p.maxIter = maxIter;
            p.tol = tol;
            return std::make_unique<SolverTSSBN>(p);
        }

        /**
         * @brief 创建自适应TSSBN动力求解器
         * @param[in] dt 初始时间步长
         * @param[in] rho_inf 高频耗散参数
         * @param[in] eps_LTE 局部截断误差容限
         * @param[in] dt_min 最小步长
         * @param[in] dt_max 最大步长
         * @param[in] maxIter 最大迭代次数
         * @param[in] tol 收敛容差
         * @return 自适应TSSBN求解器智能指针
         */
        static std::unique_ptr<SolverAdaptiveTSSBN> CreateAdaptiveTSSBNSolver(
            double dt = 0.01,
            double rho_inf = 0.9,
            double eps_LTE = 1e-3,
            double dt_min = 1e-6,
            double dt_max = 0.1,
            int maxIter = 10,
            double tol = 1e-6)
        {
            SolverAdaptiveTSSBN::Params p;
            p.dt = dt;
            p.rho_inf = rho_inf;
            p.eps_LTE = eps_LTE;
            p.dt_min = dt_min;
            p.dt_max = dt_max;
            p.maxIter = maxIter;
            p.tol = tol;
            return std::make_unique<SolverAdaptiveTSSBN>(p);
        }
    };

    // 兼容旧接口的全局函数
    inline std::unique_ptr<ISolver> CreateSolver(
        SolverType type,
        double stepSize = 0.01,
        double tolerance = 1e-5,
        int maxIterations = 32)
    {
        return SolverFactory::CreateSolver(type, stepSize, tolerance, maxIterations);
    }
}
