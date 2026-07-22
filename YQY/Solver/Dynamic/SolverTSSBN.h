/**
 * @file SolverTSSBN.h
 * @brief 动力求解器 - TSSBN (Two-Stage Single-Step Block Newmark) 隐式时间积分法
 */
#pragma once
#include "../Interface/ISolver.h"
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

namespace SolverNameSpace
{
    /**
     * @brief TSSBN 动力求解器
     *
     * 实现 TSSBN 隐式时间积分法，配合 Newton-Raphson 迭代处理非线性。
     * TSSBN 是一种两阶段单步方法，具有可控的高频耗散特性。
     * 通过 IAnalysisModel 接口访问数据，不依赖具体结构类型。
     */
    class SolverTSSBN : public ISolver
    {
    public:
        /**
         * @brief 求解器参数
         */
        struct Params
        {
            double dt = 0.01;         ///< 时间步长
            double rho_inf = 0.9;     ///< 高频耗散参数 (0 <= rho_inf < 1, rho_inf=1 表示无耗散)
            int maxIter = 10;         ///< 每个子步的最大 N-R 迭代次数
            double tol = 1e-6;        ///< 收敛容差

            // 以下参数由 rho_inf 自动计算
            double c1 = 0.0;          ///< C1 子步时间因子
            double c2 = 0.0;          ///< C2 子步时间因子
            double alpha = 0.0;       ///< C2 子步隐式参数
            double b1 = 0.0;          ///< 最终状态组合权重
        };

        /**
         * @brief 构造函数
         * @param[in] p 求解器参数
         */
        explicit SolverTSSBN(Params p);

        // ============ ISolver 接口实现 ============
        bool Solve(IAnalysisModel& model, double duration) override;
	const char* GetName() const override { return "TSSBN"; }
	SolverType GetType() const override { return SolverType::TSSBN; }
	void SetStepCallback(StepCallback callback) override { m_callback = std::move(callback); }

        // ============ 参数访问 ============
	Params& GetParams() { return m_param; }
	const Params& GetParams() const { return m_param; }

    private:
        Params m_param;
        StepCallback m_callback;

        /**
         * @brief 根据 rho_inf 计算 TSSBN 系数
         */
        void ComputeCoeffs();

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

        // C1 和 C2 子步的中间状态
        Vec m_Uc1, m_Vc1, m_Ac1;
        Vec m_Uc2, m_Vc2, m_Ac2;
        Vec m_totalDx_c1, m_totalDx_c2; // 子步累计位移增量

        /**
         * @brief 求解线性方程组 K * x = b
         * @return 是否成功
         */
        bool SolveLinear(const SpMat& K, const Vec& b, Vec& x);

        /**
         * @brief 求解 C1 子步
         * @param[in] model 分析模型
         * @param[in] currentTime 当前时间
         * @param[in] dt 时间步长
         * @return 是否成功
         */
        bool SolveSubstepC1(IAnalysisModel& model, double currentTime, double dt);

        /**
         * @brief 求解 C2 子步
         * @param[in] model 分析模型
         * @param[in] currentTime 当前时间
         * @param[in] dt 时间步长
         * @return 是否成功
         */
        bool SolveSubstepC2(IAnalysisModel& model, double currentTime, double dt);
    };
}
