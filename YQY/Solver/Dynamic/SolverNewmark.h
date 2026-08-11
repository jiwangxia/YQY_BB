/**
 * @file SolverNewmark.h
 * @brief 动力求解器 - Newmark-β 隐式时间积分法
 */
#pragma once
#include "../Interface/ISolver.h"
#include "Solver/LinearSystemSolver.h"

namespace SolverNameSpace
{
    /**
     * @brief Newmark-β 动力求解器
     * 
     * 实现 Newmark-β 隐式时间积分法，配合 Newton-Raphson 迭代处理非线性。
     * 通过 IAnalysisModel 接口访问数据，不依赖具体结构类型。
     */
    class SolverNewmark : public ISolver
    {
    public:
        /**
         * @brief 求解器参数
         */
        struct Params
        {
            double dt = 0.01;         ///< 时间步长
            double beta = 0.25;       ///< Newmark β 参数 (默认: 平均加速度法)
            double gamma = 0.5;       ///< Newmark γ 参数
            int maxIter = 10;         ///< 每个时间步的最大 N-R 迭代次数
            double tol = 1e-6;        ///< 收敛容差
            double constraintTolerance = 1e-8; ///< position-level MPC tolerance
            // A failed nonlinear trial is rolled back and retried with a
            // smaller physical time increment.  This is a convergence aid,
            // not numerical damping; accepted increments still use the
            // average-acceleration Newmark scheme above.
            double minimumTimeStep = 1.0e-5;
            double cutbackFactor = 0.5;
            double recoveryFactor = 1.25;
            int maximumCutbacks = 12;
        };

        /**
         * @brief 构造函数
         * @param[in] p 求解器参数
         */
        explicit SolverNewmark(Params p);

        // ============ ISolver 接口实现 ============
        bool Solve(IAnalysisModel& model, double duration) override;
	const char* GetName() const override { return "Newmark-beta"; }
	SolverType GetType() const override { return SolverType::Newmark; }
	void SetStepCallback(StepCallback callback) override { m_callback = std::move(callback); }

        // ============ 参数访问 ============
	Params& GetParams() { return m_param; }
	const Params& GetParams() const { return m_param; }

    private:
        Params m_param;
        StepCallback m_callback;

        /**
         * @brief Newmark 积分系数
         */
        struct Coeffs
        {
            double a0, a1;
        };
        Coeffs m_c;

        /**
         * @brief 根据时间步长计算 Newmark 系数
         */
        void ComputeCoeffs(double dt);

        // 线性求解器缓存
        LinearSystemSolver m_linearSolver;

        // 工作区（避免重复分配内存）
        SpMat m_K, m_M, m_C, m_Kc, m_Keff;
        Vec m_R, m_dx;
        Vec m_Un, m_Vn, m_An;

        /**
         * @brief 求解线性方程组 K * x = b
         * @return 是否成功
         */
        bool SolveLinear(const SpMat& K, const Vec& b, _OUT Vec& x);
    };
}
