/**
 * @file SolverNewmark.h
 * @brief 动力求解器 - Newmark-β 隐式时间积分法
 */
#pragma once
#include "../Interface/ISolver.h"
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

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
        };

        /**
         * @brief 构造函数
         * @param[in] p 求解器参数
         */
        explicit SolverNewmark(Params p = {});

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
            double a0, a1, a2, a3, a4, a5, a6, a7;
        };
        Coeffs m_c;

        /**
         * @brief 根据时间步长计算 Newmark 系数
         */
        void ComputeCoeffs(double dt);

        // 线性求解器缓存
        struct LinearCache
        {
            Eigen::SimplicialLDLT<SpMat> ldlt;  // 首选（快）
            Eigen::SparseLU<SpMat> lu;           // 备选（稳）
            bool useLdlt = true;
            bool patternAnalyzed = false;

            void reset()
            {
                useLdlt = true;
                patternAnalyzed = false;
            }
        };
        mutable LinearCache m_cache;

        // 工作区（避免重复分配内存）
        SpMat m_K, m_M, m_C, m_Keff;
        Vec m_R, m_dx;
        Vec m_Un, m_Vn, m_An;           // 上一时刻状态
        Vec m_totalDx;                   // 当前步累计位移增量
        Vec m_Acurr, m_Vcurr;           // 当前试探的加速度和速度

        /**
         * @brief 求解线性方程组 K * x = b
         * @return 是否成功
         */
        bool SolveLinear(const SpMat& K, const Vec& b, Vec& x);
    };
}
