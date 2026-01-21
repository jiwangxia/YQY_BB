#include "SolverNewmark.h"
#include <iostream>
#include <cmath>

namespace Dynamics
{
    // ==========================================
    // 辅助函数实现
    // ==========================================
    SolverNewmark::Coeffs SolverNewmark::calc_coeffs_for_dt(double dt) const
    {
        Coeffs c;
        if (dt <= 0) throw std::runtime_error("dt must be > 0");
        c.a0 = 1.0 / (param.beta * dt * dt);
        c.a1 = param.gamma / (param.beta * dt);
        c.a2 = 1.0 / (param.beta * dt);
        c.a3 = 1.0 / (2.0 * param.beta) - 1.0;
        c.a4 = param.gamma / param.beta - 1.0;
        c.a5 = dt * (param.gamma / (2.0 * param.beta) - 1.0);
        c.a6 = dt * (1.0 - param.gamma);
        c.a7 = param.gamma * dt;
        return c;
    }

    void SolverNewmark::update_kinematics(State& s, const State& s_prev, const Coeffs& c) const
    {
        s.a = c.a0 * (s.x - s_prev.x) - c.a2 * s_prev.v - c.a3 * s_prev.a;
        s.v = s_prev.v + c.a6 * s_prev.a + c.a7 * s.a;
    }

    // ==========================================
    // 统一的核心积分步 (速度优化版)
    // ==========================================
    bool SolverNewmark::step_integrate(const ModelBase& model,
        const State& curr, State& next,
        double dt, const Coeffs& c, LinearSolverCache* cache)
    {
        // 1. 预测
        next.t = curr.t + dt;
        next.x = curr.x + dt * curr.v + 0.5 * dt * dt * curr.a;
        update_kinematics(next, curr, c);

        bool is_linear = model.IsLinear();
        int max_iters = is_linear ? 1 : param.max_iter;
        bool matrix_needs_update = true;

        // 2. 缓存命中检查
        // 注意：这里的 cache 指针可能是指向 Slot_A 或 Slot_B 的，
        // 由 solve_adaptive 中的 swap 逻辑控制。
        if (is_linear && cache && cache->pattern_analyzed &&
            std::abs(dt - cache->cached_dt) < 1e-12)
        {
            matrix_needs_update = false;
        }

        // 3. 迭代求解
        for (int iter = 0; iter < max_iters; ++iter)
        {
            // A. 计算残差
            model.ComputeResidual(next, m_R_workspace);

            // 收敛判断
            if (!is_linear && m_R_workspace.norm() < param.tol) return true;
            if (is_linear && iter > 0) return true;

            // B. 矩阵更新与分解
            if (matrix_needs_update)
            {
                model.ComputeKeff(next, 1.0, c.a1, c.a0,
                    m_K_eff_workspace, m_KBuf, m_CBuf, m_MBuf);

                // === 修改点：默认回退到 Slot_A ===
                LinearSolverCache* pCache = cache ? cache : &m_cache_slot_A;

                // --- 模式分析 (Pattern Analysis) ---
                if (!pCache->pattern_analyzed)
                {
                    if (param.force_lu)
                    {
                        // 策略 (1): 强制 LU
                        pCache->lu.analyzePattern(m_K_eff_workspace);
                        pCache->use_ldlt = false;
                    }
                    else
                    {
                        // 策略 (2): 缺省尝试 LDLT
                        pCache->ldlt.analyzePattern(m_K_eff_workspace);
                        pCache->use_ldlt = true;
                    }
                    pCache->pattern_analyzed = true;
                }

                // --- 数值分解 (Factorize) ---
                if (pCache->use_ldlt)
                {
                    pCache->ldlt.factorize(m_K_eff_workspace);

                    // 策略 (2) 后半部分: LDLT 失败则切 LU
                    if (pCache->ldlt.info() != Eigen::Success)
                    {
                        pCache->use_ldlt = false; // 翻转标志位
                        pCache->lu.analyzePattern(m_K_eff_workspace); // 重新分析
                        pCache->lu.factorize(m_K_eff_workspace);
                    }
                }
                else
                {
                    pCache->lu.factorize(m_K_eff_workspace);
                }

                // --- 最终检查 ---
                bool success = (pCache->use_ldlt && pCache->ldlt.info() == Eigen::Success) ||
                    (!pCache->use_ldlt && pCache->lu.info() == Eigen::Success);

                if (!success)
                {
                    // 容错重试: 应对非线性过程中 Pattern 突变
                    if (pCache->use_ldlt) pCache->ldlt.analyzePattern(m_K_eff_workspace);
                    else                  pCache->lu.analyzePattern(m_K_eff_workspace);

                    if (pCache->use_ldlt) pCache->ldlt.factorize(m_K_eff_workspace);
                    else                  pCache->lu.factorize(m_K_eff_workspace);

                    if ((pCache->use_ldlt && pCache->ldlt.info() != Eigen::Success) ||
                        (!pCache->use_ldlt && pCache->lu.info() != Eigen::Success))
                    {
                        return false;
                    }
                }

                pCache->cached_dt = dt;
                if (is_linear) matrix_needs_update = false;
            }

            // C. 求解增量
            // === 修改点：同样使用 Slot_A 作为回退 ===
            LinearSolverCache* pCache = cache ? cache : &m_cache_slot_A;

            if (pCache->use_ldlt) m_dx_workspace = pCache->ldlt.solve(-m_R_workspace);
            else                  m_dx_workspace = pCache->lu.solve(-m_R_workspace);

            // D. 更新状态
            next.x += m_dx_workspace;
            update_kinematics(next, curr, c);
        }

        return true;
    }

    // ==========================================
    // 对外接口实现
    // ==========================================
    void SolverNewmark::solve(const ModelBase& model, State& state,
        double duration, Observer observer)
    {
        if (duration <= 0) throw std::runtime_error("Duration must be > 0");

        // 工作区预分配
        size_t dofs = model.GetDofs();
        if (m_R_workspace.size() != dofs)
        {
            m_R_workspace.resize(dofs);
            m_dx_workspace.resize(dofs);
            m_K_eff_workspace.resize(dofs, dofs);
        }

        reset_caches();

        model.SolveAcceleration(state);
        if (observer) observer(state);

        if (param.bAdaptive)
            solve_adaptive(model, state, duration, observer);
        else
            solve_fixed(model, state, duration, observer);
    }

    void SolverNewmark::solve_fixed(const ModelBase& model, State& state, double duration, Observer observer)
    {
        auto c = calc_coeffs_for_dt(param.dt);
        int n_steps = (int)std::ceil(duration / param.dt);
        State next = state;

        for (int step = 0; step < n_steps; ++step)
        {
            // 固定步长只使用 slot_A 即可
            if (!step_integrate(model, state, next, param.dt, c, &m_cache_slot_A))
            {
                throw std::runtime_error("Fixed step solver failed.");
            }
            state = next;
            if (observer) observer(state);
        }
    }

    // SolverNewmark.cpp

    void SolverNewmark::solve_adaptive(const ModelBase& model, State& state, double duration, Observer observer)
    {
        double t_current = 0.0;
        double dt = param.dt;

        State s_coarse = state;
        State s_mid = state;
        State s_fine = state;
        int retry_count = 0;

        // --- 定义逻辑指针 ---
        // p_coarse: 用于粗糙步 (dt)
        // p_fine_1: 用于精细步 1 (dt/2, t时刻) -> 这是我们回退时最想复用的！
        // p_fine_2: 用于精细步 2 (dt/2, t+dt/2时刻)
        LinearSolverCache* p_coarse = &m_cache_slot_A;
        LinearSolverCache* p_fine_1 = &m_cache_slot_B;
        LinearSolverCache* p_fine_2 = &m_cache_slot_C;

        while (t_current < duration)
        {
            if (t_current + dt > duration) dt = duration - t_current;
            bool step_accepted = false;
            retry_count = 0;

            while (!step_accepted && retry_count < 10)
            {
                auto c_full = calc_coeffs_for_dt(dt);
                auto c_half = calc_coeffs_for_dt(dt / 2.0);

                try
                {
                    // 1. 粗糙步 (dt) -> 使用 p_coarse
                    s_coarse = state;
                    if (!step_integrate(model, state, s_coarse, dt, c_full, p_coarse))
                        throw std::runtime_error("Coarse step failed");

                    // 2. 精细步 1 (dt/2) -> 使用 p_fine_1 (保护这个缓存！)
                    s_mid = state;
                    if (!step_integrate(model, state, s_mid, dt / 2.0, c_half, p_fine_1))
                        throw std::runtime_error("Fine step 1 failed");

                    // 3. 精细步 2 (dt/2) -> 使用 p_fine_2 (独立槽位，不覆盖 p_fine_1)
                    s_fine = s_mid;
                    if (!step_integrate(model, s_mid, s_fine, dt / 2.0, c_half, p_fine_2))
                        throw std::runtime_error("Fine step 2 failed");

                    // 4. 误差评估
                    double diff = (s_fine.x - s_coarse.x).norm();
                    double ref = s_fine.x.norm();
                    double error = diff / (ref + 1e-10);

                    if (error < param.tol_adaptive)
                    {
                        step_accepted = true;
                        state = s_fine;
                        t_current += dt;
                        if (observer) observer(state);

                        if (error < param.tol_adaptive * 0.1 && dt < param.max_dt)
                        {
                            dt *= 1.5;
                            // 增大步长，缓存失效，无需操作指针
                        }
                    }
                    else
                    {
                        // === 步长减半逻辑 ===
                        dt *= 0.5;

                        if (dt < param.min_dt)
                        {
                            // 强制接受...
                        }
                        else
                        {
                            // === 完美优化：指针交换 ===
                            // 下一步是新的粗糙步 (dt_new = dt_old / 2)
                            // 它的物理含义完全等同于刚才的 "精细步 1"

                            // 将 p_fine_1 (含有效分解) 换给 p_coarse (用于下一步)
                            // 将 p_coarse (含无效分解) 换给 p_fine_1 (作为备用槽)
                            std::swap(p_coarse, p_fine_1);

                            // p_fine_2 不需要动，它指向 Slot C，下次会被覆盖
                        }
                    }
                }
                catch (...)
                {
                    dt *= 0.5;
                    if (dt < param.min_dt) throw std::runtime_error("Diverged.");
                    // 异常情况下不建议复用，保持原样重新计算
                }
                retry_count++;
            }
        }
    }
}
