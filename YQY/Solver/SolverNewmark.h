#pragma once
#include "ModelBase.h"
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

namespace Dynamics
{
    class SolverNewmark
    {
    public:
        struct Parameters
        {
            bool bAdaptive = true;
            double dt = 0.01;
            double min_dt = 1e-6;
            double max_dt = 1.0;
            double tol_adaptive = 1e-4;

            double beta = 0.25;
            double gamma = 0.5;

            int max_iter = 10;
            double tol = 1e-8;

            // 如果已知系统是非对称的（如摩擦、非保守力），设置为 true
            // 将直接使用 LU 分解，跳过 LDLT 尝试
            bool force_lu = false;
        } param;

    private:
        struct Coeffs { double a0, a1, a2, a3, a4, a5, a6, a7; };

        // --- 增强型求解器缓存 (支持 LDLT/LU 自动切换) ---
        struct LinearSolverCache
        {
            Eigen::SimplicialLDLT<SpMat> ldlt; // 首选 (快)
            Eigen::SparseLU<SpMat> lu;         // 备选 (通用)

            double cached_dt = -1.0;
            bool pattern_analyzed = false;
            bool use_ldlt = true; // 默认为 true，失败后置 false

            void reset()
            {
                cached_dt = -1.0;
                pattern_analyzed = false;
                use_ldlt = true;
            }
        };

        // --- 内部辅助函数 ---
        Coeffs calc_coeffs_for_dt(double dt) const;
        void update_kinematics(State& s, const State& s_prev, const Coeffs& c) const;

        // --- 统一的核心积分步 (Kernel) ---
        // 关键优化：传入指定的 Cache 指针，实现粗细步长的独立缓存
        bool step_integrate(const ModelBase& model, const State& curr, State& next,
            double dt, const Coeffs& c, LinearSolverCache* cache);

        void solve_fixed(const ModelBase& model, State& state, double duration, Observer observer);
        void solve_adaptive(const ModelBase& model, State& state, double duration, Observer observer);

        void reset_caches() const
        {
            m_cache_slot_A.reset();
            m_cache_slot_B.reset();
            m_cache_slot_C.reset();
        }

    public:
        SolverNewmark(Parameters p = Parameters()) : param(p) {}
        void solve(const ModelBase& model, State& state, double duration,
            Observer observer = nullptr);

    private:
        // --- 矩阵构建缓存 ---
        mutable SpMat m_KBuf, m_MBuf, m_CBuf;

        // --- 内部工作区 (避免循环内分配) ---
        mutable SpMat m_K_eff_workspace;
        mutable Vec m_R_workspace;
        mutable Vec m_dx_workspace;

        mutable LinearSolverCache m_cache_slot_A;
        mutable LinearSolverCache m_cache_slot_B;
        mutable LinearSolverCache m_cache_slot_C; // 新增：用于保护 Fine 1 不被 Fine 2 覆盖
    };
}
