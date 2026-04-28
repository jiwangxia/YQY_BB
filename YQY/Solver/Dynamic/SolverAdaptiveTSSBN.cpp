/**
 * @file SolverAdaptiveTSSBN.cpp
 * @brief 自适应TSSBN动力求解器实现
 */
#include "SolverAdaptiveTSSBN.h"
#include <QDebug>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>  // 添加 Eigen Dense 模块，包含 QR 分解

namespace SolverNameSpace
{
    SolverAdaptiveTSSBN::SolverAdaptiveTSSBN(Params p) : m_param(p)
    {
        ComputeCoeffs();
        ComputeEmbeddedCoeffs();
    }

    void SolverAdaptiveTSSBN::ComputeCoeffs()
    {
        double rho = m_param.rho_inf;
        const double EPSILON = 1e-9;

        if (rho >= 0.0 && rho < 1.0 - EPSILON)
        {
            m_param.c1 = (std::sqrt(2.0) * std::sqrt(rho + 1.0) - 2.0) / (2.0 * (rho - 1.0));
            m_param.c2 = (2.0 * rho * m_param.c1 + 1.0) / (2.0 * rho * m_param.c1 - 2.0 * m_param.c1 + 2.0);
            m_param.alpha = -(2.0 * m_param.c1 - 1.0) / (2.0 * m_param.c2 - 2.0 * m_param.c1 * m_param.c2 + 2.0 * rho * m_param.c1 * m_param.c2);
            m_param.b1 = -(2.0 * m_param.c2 - 1.0) / (2.0 * m_param.c1 - 2.0 * m_param.c2);
        }
        else if (std::abs(rho - 1.0) < EPSILON)
        {
            m_param.c1 = 0.25;
            m_param.c2 = 0.75;
            m_param.alpha = 1.0 / 3.0;
            m_param.b1 = 0.5;
        }
        else
        {
            throw std::invalid_argument("AdaptiveTSSBN: rho_inf must be in [0, 1)");
        }

        qDebug().noquote() << QStringLiteral("自适应TSSBN基本参数: c1=%1, c2=%2, alpha=%3, b1=%4")
            .arg(m_param.c1, 0, 'f', 6)
            .arg(m_param.c2, 0, 'f', 6)
            .arg(m_param.alpha, 0, 'f', 6)
            .arg(m_param.b1, 0, 'f', 6);
    }

    void SolverAdaptiveTSSBN::ComputeEmbeddedCoeffs()
    {
        // 求解嵌入式公式的 b_hat 系数（3阶）
        // 使用 Eigen 求解线性方程组
        Eigen::Matrix3d M_b;
        M_b << 1.0, 1.0, 1.0,
               m_param.c1, m_param.c2, m_param.c3_hat,
               m_param.c1 * m_param.c1, m_param.c2 * m_param.c2, m_param.c3_hat * m_param.c3_hat;

        Eigen::Vector3d r_b;
        r_b << 1.0, 0.5, 1.0 / 3.0;

        Eigen::Vector3d b_hat_sol = M_b.colPivHouseholderQr().solve(r_b);

        m_param.b1_hat = b_hat_sol(0);
        m_param.b2_hat = b_hat_sol(1);
        m_param.b3_hat = b_hat_sol(2);

        // 求解 a_hat 系数
        Eigen::Matrix2d M_a;
        M_a << 1.0, 1.0,
               m_param.b3_hat * m_param.c1, m_param.b3_hat * m_param.c2;

        Eigen::Vector2d r_a;
        r_a << m_param.c3_hat,
               (1.0 / 6.0) - m_param.b1_hat * m_param.c1 * m_param.c1 -
               m_param.b2_hat * (m_param.c1 * m_param.c2 * (1.0 - m_param.alpha) +
                                m_param.c2 * m_param.c2 * m_param.alpha);

        Eigen::Vector2d a_hat_sol = M_a.colPivHouseholderQr().solve(r_a);

        m_param.a31_hat = a_hat_sol(0);
        m_param.a32_hat = a_hat_sol(1);

        qDebug().noquote() << QStringLiteral("嵌入式公式参数: b1_hat=%1, b2_hat=%2, b3_hat=%3")
            .arg(m_param.b1_hat, 0, 'f', 6)
            .arg(m_param.b2_hat, 0, 'f', 6)
            .arg(m_param.b3_hat, 0, 'f', 6);
        qDebug().noquote() << QStringLiteral("                a31_hat=%1, a32_hat=%2")
            .arg(m_param.a31_hat, 0, 'f', 6)
            .arg(m_param.a32_hat, 0, 'f', 6);
    }

    bool SolverAdaptiveTSSBN::Solve(IAnalysisModel& model, double duration)
    {
        qDebug().noquote() << QStringLiteral("开始自适应TSSBN动力非线性求解...");
        qDebug().noquote() << QStringLiteral("rho_inf=%1, dt_init=%2, eps_LTE=%3")
            .arg(m_param.rho_inf).arg(m_param.dt).arg(m_param.eps_LTE);

        const int nDofs = model.GetFreeDofs();
        if (nDofs <= 0)
        {
            qDebug().noquote() << QStringLiteral("错误: 自由度数量无效");
            return false;
        }

        if (m_param.dt <= 0.0)
        {
            qDebug().noquote() << QStringLiteral("错误: 时间步长必须为正");
            return false;
        }

        // 初始化工作区
        m_dx.resize(nDofs);
        m_R.resize(nDofs);
        m_totalDx_c1.resize(nDofs);
        m_totalDx_c2.resize(nDofs);
        m_totalDx_eld.resize(nDofs);
        m_Uc1.resize(nDofs);
        m_Vc1.resize(nDofs);
        m_Ac1.resize(nDofs);
        m_Uc2.resize(nDofs);
        m_Vc2.resize(nDofs);
        m_Ac2.resize(nDofs);
        m_Ueld.resize(nDofs);
        m_Veld.resize(nDofs);
        m_Aeld.resize(nDofs);
        m_Un.resize(nDofs);
        m_Vn.resize(nDofs);
        m_An.resize(nDofs);

        m_cache.reset();

        // 获取初始状态
        model.GetState(m_Un, m_Vn, m_An);

        double dt_try = m_param.dt;
        m_totalSteps = 0;
        m_rejectedSteps = 0;
        double dt_sum = 0.0;
        double currentTime = 0.0;

        const double time_tolerance = 1e-10;

        // 自适应时间步循环
        while (currentTime < duration - time_tolerance)
        {
            // 限制步长范围
            dt_try = std::max(m_param.dt_min, std::min(m_param.dt_max, dt_try));

            // 确保不超过终止时间
            if (currentTime + dt_try > duration)
            {
                dt_try = duration - currentTime;
            }

            // 备份当前状态（用于步长拒绝时恢复）
            Vec Un_backup = m_Un;
            Vec Vn_backup = m_Vn;
            Vec An_backup = m_An;

            bool accepted = false;
            int n_iter = 0;

            while (!accepted)
            {
                // ========== C1 子步 ==========
                if (!SolveSubstepC1(model, currentTime, dt_try))
                {
                    qDebug().noquote() << QStringLiteral("错误: C1 子步求解失败");
                    return false;
                }

                // ========== C2 子步 ==========
                if (!SolveSubstepC2(model, currentTime, dt_try))
                {
                    qDebug().noquote() << QStringLiteral("错误: C2 子步求解失败");
                    return false;
                }

                // ========== ELD 子步（用于误差估计）==========
                if (!SolveSubstepELD(model, currentTime, dt_try))
                {
                    qDebug().noquote() << QStringLiteral("错误: ELD 子步求解失败");
                    return false;
                }

                // ========== 计算两种阶数的解 ==========
                // 2阶解（TSSBN原始公式）
                Vec u_order2 = m_Un + dt_try * (m_param.b1 * m_Vc1 + (1.0 - m_param.b1) * m_Vc2);
                Vec v_order2 = m_Vn + dt_try * (m_param.b1 * m_Ac1 + (1.0 - m_param.b1) * m_Ac2);

                // 3阶解（嵌入式公式）
                Vec u_order3 = m_Un + dt_try * (m_param.b1_hat * m_Vc1 + m_param.b2_hat * m_Vc2 + m_param.b3_hat * m_Veld);
                Vec v_order3 = m_Vn + dt_try * (m_param.b1_hat * m_Ac1 + m_param.b2_hat * m_Ac2 + m_param.b3_hat * m_Aeld);

                // ========== 误差估计 ==========
                double LTE = EstimateError(u_order2, v_order2, u_order3, v_order3, m_Un, m_Vn);

                // ========== 步长控制 ==========
                if (LTE > 1.0)
                {
                    // 误差过大，拒绝当前步
                    if (dt_try <= m_param.dt_min * 1.001)
                    {
                        qDebug().noquote() << QStringLiteral("警告: LTE=%1 > 1.0 但已达最小步长，强制接受").arg(LTE);
                        accepted = true;
                    }
                    else
                    {
                        // 缩小步长重试
                        dt_try = ComputeNewStepSize(dt_try, LTE, n_iter);
                        m_rejectedSteps++;

                        // 恢复状态
                        m_Un = Un_backup;
                        m_Vn = Vn_backup;
                        m_An = An_backup;

                        continue;
                    }
                }
                else
                {
                    // 误差可接受
                    accepted = true;
                }

                if (accepted)
                {
                    // 使用2阶解更新状态
                    Vec totalDx = u_order2 - m_Un;

                    // 更新模型状态
                    model.ApplyIncrement(totalDx);
                    model.SetTrialKinematics(v_order2, m_An);

                    // 计算最终加速度
                    model.AssembleMatrices(m_K, &m_M, &m_C);
                    model.ComputeResidual(currentTime + dt_try, 1.0, m_R);

                    Vec An_new;
                    if (!SolveLinear(m_M, m_R, An_new))
                    {
                        qDebug().noquote() << QStringLiteral("错误: 最终加速度求解失败");
                        return false;
                    }

                    model.SetTrialKinematics(v_order2, An_new);

                    // 提交状态
                    Vec zero = Vec::Zero(nDofs);
                    model.ApplyIncrement(zero);

                    // 更新状态
                    m_Un = u_order2;
                    m_Vn = v_order2;
                    m_An = An_new;
                    currentTime += dt_try;

                    // 计算下一步的步长
                    dt_try = ComputeNewStepSize(dt_try, LTE, n_iter);

                    m_totalSteps++;
                    dt_sum += dt_try;

                    // 步回调
                    if (m_callback)
                    {
                        Vec u, v, a;
                        model.GetState(u, v, a);
                        m_callback(m_totalSteps, currentTime, u);
                    }

                    // 步结束回调
                    model.OnStepCompleted(currentTime);
                }
            }
        }

        m_avgDt = (m_totalSteps > 0) ? (dt_sum / m_totalSteps) : m_param.dt;

        qDebug().noquote() << QStringLiteral("自适应TSSBN求解完成:");
        qDebug().noquote() << QStringLiteral("  总步数=%1, 拒绝步数=%2, 平均步长=%3")
            .arg(m_totalSteps).arg(m_rejectedSteps).arg(m_avgDt, 0, 'e', 3);

        return true;
    }

    bool SolverAdaptiveTSSBN::SolveSubstepC1(IAnalysisModel& model, double currentTime, double dt)
    {
        const int nDofs = model.GetFreeDofs();
        double c1_dt = m_param.c1 * dt;
        double c1_dt2 = c1_dt * c1_dt;

        m_Uc1 = m_Un;
        m_totalDx_c1.setZero();

        for (int iter = 0; iter < m_param.maxIter; ++iter)
        {
            m_Vc1 = (m_Uc1 - m_Un) / c1_dt;
            m_Ac1 = (m_Vc1 - m_Vn) / c1_dt;

            model.SetTrialKinematics(m_Vc1, m_Ac1);
            model.AssembleMatrices(m_K, &m_M, &m_C);

            m_Keff = m_K + (1.0 / c1_dt2) * m_M + (1.0 / c1_dt) * m_C;
            model.ComputeResidual(currentTime + c1_dt, 1.0, m_R);

            double error = m_R.norm();
            if (error < m_param.tol && iter > 0)
            {
                return true;
            }

            if (iter == m_param.maxIter - 1)
            {
                qDebug().noquote() << QStringLiteral("警告: C1 子步未收敛 (残差=%1)").arg(error);
                return false;
            }

            if (!SolveLinear(m_Keff, m_R, m_dx))
            {
                return false;
            }

            m_totalDx_c1 += m_dx;
            m_Uc1 += m_dx;
            model.ApplyIncrement(m_dx);
        }

        return true;
    }

    bool SolverAdaptiveTSSBN::SolveSubstepC2(IAnalysisModel& model, double currentTime, double dt)
    {
        const int nDofs = model.GetFreeDofs();
        double c2_dt = m_param.c2 * dt;
        double alpha_c2_dt = m_param.alpha * c2_dt;
        double alpha_c2_dt2 = alpha_c2_dt * alpha_c2_dt;

        m_Uc2 = m_Uc1;
        m_totalDx_c2.setZero();

        Vec u_pred = m_Un + c2_dt * (1.0 - m_param.alpha) * m_Vc1;
        Vec v_pred = m_Vn + c2_dt * (1.0 - m_param.alpha) * m_Ac1;

        for (int iter = 0; iter < m_param.maxIter; ++iter)
        {
            m_Vc2 = (m_Uc2 - u_pred) / alpha_c2_dt;
            m_Ac2 = (m_Vc2 - v_pred) / alpha_c2_dt;

            model.SetTrialKinematics(m_Vc2, m_Ac2);
            model.AssembleMatrices(m_K, &m_M, &m_C);

            m_Keff = m_K + (1.0 / alpha_c2_dt2) * m_M + (1.0 / alpha_c2_dt) * m_C;
            model.ComputeResidual(currentTime + c2_dt, 1.0, m_R);

            double error = m_R.norm();
            if (error < m_param.tol && iter > 0)
            {
                return true;
            }

            if (iter == m_param.maxIter - 1)
            {
                qDebug().noquote() << QStringLiteral("警告: C2 子步未收敛 (残差=%1)").arg(error);
                return false;
            }

            if (!SolveLinear(m_Keff, m_R, m_dx))
            {
                return false;
            }

            m_totalDx_c2 += m_dx;
            m_Uc2 += m_dx;
            model.ApplyIncrement(m_dx);
        }

        return true;
    }

    bool SolverAdaptiveTSSBN::SolveSubstepELD(IAnalysisModel& model, double currentTime, double dt)
    {
        const int nDofs = model.GetFreeDofs();
        double c3_dt = m_param.c3_hat * dt;
        double c3_dt2 = c3_dt * c3_dt;

        // ELD 子步的预测（基于 C1 和 C2 的结果）
        Vec u_pred = m_Un + c3_dt * (m_param.a31_hat * m_Vc1 + m_param.a32_hat * m_Vc2);
        Vec v_pred = m_Vn + c3_dt * (m_param.a31_hat * m_Ac1 + m_param.a32_hat * m_Ac2);

        m_Ueld = u_pred;
        m_totalDx_eld.setZero();

        // 使用隐式参数（这里简化为与C2相同的alpha）
        double alpha_c3_dt = m_param.alpha * c3_dt;
        double alpha_c3_dt2 = alpha_c3_dt * alpha_c3_dt;

        for (int iter = 0; iter < m_param.maxIter; ++iter)
        {
            m_Veld = (m_Ueld - u_pred) / alpha_c3_dt;
            m_Aeld = (m_Veld - v_pred) / alpha_c3_dt;

            model.SetTrialKinematics(m_Veld, m_Aeld);
            model.AssembleMatrices(m_K, &m_M, &m_C);

            m_Keff = m_K + (1.0 / alpha_c3_dt2) * m_M + (1.0 / alpha_c3_dt) * m_C;
            model.ComputeResidual(currentTime + c3_dt, 1.0, m_R);

            double error = m_R.norm();
            if (error < m_param.tol && iter > 0)
            {
                return true;
            }

            if (iter == m_param.maxIter - 1)
            {
                qDebug().noquote() << QStringLiteral("警告: ELD 子步未收敛 (残差=%1)").arg(error);
                return false;
            }

            if (!SolveLinear(m_Keff, m_R, m_dx))
            {
                return false;
            }

            m_totalDx_eld += m_dx;
            m_Ueld += m_dx;
            model.ApplyIncrement(m_dx);
        }

        return true;
    }

    double SolverAdaptiveTSSBN::EstimateError(const Vec& u2, const Vec& v2,
                                               const Vec& u3, const Vec& v3,
                                               const Vec& u_base, const Vec& v_base)
    {
        const double tolerance_abs = 1e-6;
        const double epsilon_safe = 1e-30;

        Vec err_u = u2 - u3;
        Vec err_v = v2 - v3;

        double ref_u_norm = std::max(u_base.norm(), u2.norm());
        double ref_v_norm = std::max(v_base.norm(), v2.norm());

        double err_u_scaled = err_u.norm() / (tolerance_abs + m_param.eps_LTE * ref_u_norm + epsilon_safe);
        double err_v_scaled = err_v.norm() / (tolerance_abs + m_param.eps_LTE * ref_v_norm + epsilon_safe);

        double LTE = std::sqrt(0.5 * (err_u_scaled * err_u_scaled + err_v_scaled * err_v_scaled));

        return LTE;
    }

    double SolverAdaptiveTSSBN::ComputeNewStepSize(double dt_current, double LTE, int n_iter)
    {
        const double epsilon_safe = 1e-30;
        const double inv_p_plus_1 = 1.0 / (m_param.p_order + 1.0);

        // PI控制器
        double factor_P = m_param.S * std::pow(1.0 / (LTE + epsilon_safe), inv_p_plus_1);

        double err_ratio = LTE / (m_LTE_prev + epsilon_safe);
        double factor_D = std::pow(err_ratio, -m_param.k_d);
        factor_D = std::max(m_param.FacD_min, std::min(m_param.FacD_max, factor_D));

        // 性能因子
        double factor_perf = std::sqrt(double(m_param.N_target) / std::max(1, n_iter));

        // 组合因子
        double raw_factor = std::max(m_param.gamma_shrink, std::min(m_param.gamma_grow, factor_P * factor_D));
        double final_factor = std::min(raw_factor, factor_perf);

        double dt_new = dt_current * final_factor;
        dt_new = std::max(m_param.dt_min, std::min(m_param.dt_max, dt_new));

        m_LTE_prev = LTE;

        return dt_new;
    }

    bool SolverAdaptiveTSSBN::SolveLinear(const SpMat& K, const Vec& b, Vec& x)
    {
        bool solved = false;

        if (m_cache.useLdlt)
        {
            if (!m_cache.patternAnalyzed)
            {
                m_cache.ldlt.analyzePattern(K);
            }
            m_cache.ldlt.factorize(K);

            if (m_cache.ldlt.info() == Eigen::Success)
            {
                x = m_cache.ldlt.solve(b);
                if (m_cache.ldlt.info() == Eigen::Success)
                {
                    solved = true;
                    m_cache.patternAnalyzed = true;
                }
            }

            if (!solved)
            {
                qDebug() << "LDLT failed, switching to LU...";
                m_cache.useLdlt = false;
                m_cache.patternAnalyzed = false;
            }
        }

        if (!solved)
        {
            if (!m_cache.patternAnalyzed)
            {
                m_cache.lu.analyzePattern(K);
                m_cache.patternAnalyzed = true;
            }
            m_cache.lu.factorize(K);

            if (m_cache.lu.info() == Eigen::Success)
            {
                x = m_cache.lu.solve(b);
                solved = true;
            }
            else
            {
                qDebug() << "LU factorization failed!";
                m_cache.patternAnalyzed = false;
            }
        }

        return solved;
    }
}
