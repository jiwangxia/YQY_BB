/**
 * @file SolverAdaptiveTSSBN.h
 * @brief 动力求解器 - 自适应TSSBN隐式时间积分法
 */
#pragma once
#include "../Interface/ISolver.h"
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

namespace SolverNameSpace
{
    /**
     * @brief 自适应TSSBN动力求解器
     *
     * 实现带误差估计和自适应步长控制的TSSBN方法。
     * 使用嵌入式公式（2阶+3阶）进行局部截断误差估计。
     * 通过 IAnalysisModel 接口访问数据，不依赖具体结构类型。
     */
    class SolverAdaptiveTSSBN : public ISolver
    {
    public:
        /**
         * @brief 求解器参数
         */
        struct Params
        {
            // TSSBN 基本参数
            double dt = 0.01;           ///< 初始时间步长
            double rho_inf = 0.9;       ///< 高频耗散参数 (0 <= rho_inf < 1)
            double c1 = 0.0;            ///< C1 子步时间因子（自动计算）
            double c2 = 0.0;            ///< C2 子步时间因子（自动计算）
            double alpha = 0.0;         ///< C2 子步隐式参数（自动计算）
            double b1 = 0.0;            ///< 最终状态组合权重（自动计算）

            // 嵌入式公式参数（3阶）
            double c3_hat = 1.0;        ///< 第三个子步时间因子
            double b1_hat = 0.0;        ///< 3阶公式权重1（自动计算）
            double b2_hat = 0.0;        ///< 3阶公式权重2（自动计算）
            double b3_hat = 0.0;        ///< 3阶公式权重3（自动计算）
            double a31_hat = 0.0;       ///< ELD子步系数1（自动计算）
            double a32_hat = 0.0;       ///< ELD子步系数2（自动计算）

            // 自适应步长控制参数
            double dt_min = 1e-6;       ///< 最小步长
            double dt_max = 0.1;        ///< 最大步长
            double eps_LTE = 1e-3;      ///< 局部截断误差容限
            double S = 0.9;             ///< 安全因子
            double p_order = 2.0;       ///< 方法阶数
            double gamma_grow = 1.5;    ///< 步长增长因子
            double gamma_shrink = 0.5;  ///< 步长缩小因子
            int N_target = 5;           ///< 目标迭代次数
            int N_max = 20;             ///< 最大迭代次数
            double k_d = 0.2;           ///< 数字滤波系数
            double FacD_min = 0.5;      ///< 微分因子最小值
            double FacD_max = 2.0;      ///< 微分因子最大值

            // 迭代参数
            int maxIter = 10;           ///< 每个子步的最大迭代次数
            double tol = 1e-6;          ///< 收敛容差
        };

        /**
         * @brief 构造函数
         * @param[in] p 求解器参数
         */
        explicit SolverAdaptiveTSSBN(Params p);

        // ============ ISolver 接口实现 ============
        bool Solve(IAnalysisModel& model, double duration) override;
        const char* GetName() const override { return "Adaptive-TSSBN"; }
        SolverType GetType() const override { return SolverType::AdaptiveTSSBN; }
        void SetStepCallback(StepCallback callback) override { m_callback = std::move(callback); }

        // ============ 参数访问 ============
        Params& GetParams() { return m_param; }
        const Params& GetParams() const { return m_param; }

        // ============ 统计信息 ============
        int GetTotalSteps() const { return m_totalSteps; }
        int GetRejectedSteps() const { return m_rejectedSteps; }
        double GetAverageStepSize() const { return m_avgDt; }

    private:
        Params m_param;
        StepCallback m_callback;

        // 统计信息
        int m_totalSteps = 0;
        int m_rejectedSteps = 0;
        double m_avgDt = 0.0;
        double m_LTE_prev = 1e-6;

        /**
         * @brief 根据 rho_inf 计算 TSSBN 基本系数
         */
        void ComputeCoeffs();

        /**
         * @brief 计算嵌入式公式系数
         */
        void ComputeEmbeddedCoeffs();

        // 线性求解器缓存
        struct LinearCache
        {
            Eigen::SimplicialLDLT<SpMat> ldlt;
            Eigen::SparseLU<SpMat> lu;
            bool useLdlt = true;
            bool patternAnalyzed = false;

            void reset()
            {
                useLdlt = true;
                patternAnalyzed = false;
            }
        };
        mutable LinearCache m_cache;

        // 工作区
        SpMat m_K, m_M, m_C, m_Keff;
        Vec m_R, m_dx;
        Vec m_Un, m_Vn, m_An;

        // 三个子步的中间状态
        Vec m_Uc1, m_Vc1, m_Ac1;
        Vec m_Uc2, m_Vc2, m_Ac2;
        Vec m_Ueld, m_Veld, m_Aeld;  // ELD子步（用于误差估计）
        Vec m_totalDx_c1, m_totalDx_c2, m_totalDx_eld;

        /**
         * @brief 求解线性方程组
         */
        bool SolveLinear(const SpMat& K, const Vec& b, Vec& x);

        /**
         * @brief 求解 C1 子步
         */
        bool SolveSubstepC1(IAnalysisModel& model, double currentTime, double dt);

        /**
         * @brief 求解 C2 子步
         */
        bool SolveSubstepC2(IAnalysisModel& model, double currentTime, double dt);

        /**
         * @brief 求解 ELD 子步（用于误差估计）
         */
        bool SolveSubstepELD(IAnalysisModel& model, double currentTime, double dt);

        /**
         * @brief 估计局部截断误差
         * @param[in] u2 2阶公式的位移
         * @param[in] v2 2阶公式的速度
         * @param[in] u3 3阶公式的位移
         * @param[in] v3 3阶公式的速度
         * @param[in] u_base 基准位移
         * @param[in] v_base 基准速度
         * @return 归一化的局部截断误差
         */
        double EstimateError(const Vec& u2, const Vec& v2,
                            const Vec& u3, const Vec& v3,
                            const Vec& u_base, const Vec& v_base);

        /**
         * @brief 计算新的时间步长
         * @param[in] dt_current 当前步长
         * @param[in] LTE 局部截断误差
         * @param[in] n_iter 迭代次数
         * @return 新的时间步长
         */
        double ComputeNewStepSize(double dt_current, double LTE, int n_iter);
    };
}
