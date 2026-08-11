/**
 * @file SolverStatic.h
 * @brief 静力求解器 - Newton-Raphson 增量迭代法
 */
#pragma once
#include "../Interface/ISolver.h"
#include "Solver/LinearSystemSolver.h"

namespace SolverNameSpace
{
    /**
     * @brief 静力求解器
     * 
     * 实现增量加载的 Newton-Raphson 迭代法。
     * 通过 IAnalysisModel 接口访问数据，不依赖具体结构类型。
     */
    class SolverStatic : public ISolver
    {
    public:
        /**
         * @brief 求解器参数
         */
        struct Params
        {
            int numIncrements = 10;       //  荷载增量步数
            int maxIter       = 32;       //  每个增量步的最大 N-R 迭代次数
            double tol_R      = 1e-4;     //  相对残差容差
            double tol_dx     = 1e-6;     //  位移增量容差
            double tol_C      = 1e-10;    //  非线性约束容差
        };

        /**
         * @brief 构造函数
         * @param[in] p 求解器参数
         */
	explicit SolverStatic(Params p) : m_param(p) {}

        // ============ ISolver 接口实现 ============
        bool Solve(IAnalysisModel& model, double duration) override;
	const char* GetName() const override { return "Newton-Raphson Static"; }
	SolverType GetType() const override { return SolverType::Static; }
	void SetStepCallback(StepCallback callback) override { m_callback = std::move(callback); }

        

        // ============ 参数访问 ============
	Params& GetParams() { return m_param; }
	const Params& GetParams() const { return m_param; }

    private:
        Params m_param;
        StepCallback m_callback;

        // 线性求解器缓存
        LinearSystemSolver m_linearSolver;

        // 工作区（避免重复分配内存）
        SpMat m_K;
        Vec m_R, m_dx;

        /**
         * @brief 求解线性方程组 K * x = b
         * @return 是否成功
         */
        bool SolveLinear(const SpMat& K, const Vec& b, _OUT Vec& x);
    };
}
