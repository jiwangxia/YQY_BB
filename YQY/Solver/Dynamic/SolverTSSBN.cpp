/**
 * @file SolverTSSBN.cpp
 * @brief TSSBN 动力求解器实现
 */
#include "SolverTSSBN.h"
#include <QDebug>
#include <stdexcept>
#include <cmath>

namespace SolverNameSpace
{
    SolverTSSBN::SolverTSSBN(Params p) : m_param(p)
    {
        ComputeCoeffs();
    }

    void SolverTSSBN::ComputeCoeffs()
    {
        double rho = m_param.rho_inf;
        const double EPSILON = 1e-9;

        if (rho >= 0.0 && rho < 1.0 - EPSILON)
        {
            // 根据 rho_inf 计算 TSSBN 参数
            m_param.c1 = (std::sqrt(2.0) * std::sqrt(rho + 1.0) - 2.0) / (2.0 * (rho - 1.0));
            m_param.c2 = (2.0 * rho * m_param.c1 + 1.0) / (2.0 * rho * m_param.c1 - 2.0 * m_param.c1 + 2.0);
            m_param.alpha = -(2.0 * m_param.c1 - 1.0) / (2.0 * m_param.c2 - 2.0 * m_param.c1 * m_param.c2 + 2.0 * rho * m_param.c1 * m_param.c2);
            m_param.b1 = -(2.0 * m_param.c2 - 1.0) / (2.0 * m_param.c1 - 2.0 * m_param.c2);
        }
        else if (std::abs(rho - 1.0) < EPSILON)
        {
            // rho_inf = 1.0 时的特殊情况（无耗散）
            m_param.c1 = 0.25;
            m_param.c2 = 0.75;
            m_param.alpha = 1.0 / 3.0;
            m_param.b1 = 0.5;
        }
        else
        {
            throw std::invalid_argument("TSSBN: rho_inf must be in [0, 1)");
        }

        qDebug().noquote() << QStringLiteral("TSSBN 参数: c1=%1, c2=%2, alpha=%3, b1=%4")
            .arg(m_param.c1, 0, 'f', 6)
            .arg(m_param.c2, 0, 'f', 6)
            .arg(m_param.alpha, 0, 'f', 6)
            .arg(m_param.b1, 0, 'f', 6);
    }

    bool SolverTSSBN::Solve(IAnalysisModel& model, double duration)
    {
        qDebug().noquote() << QStringLiteral("开始 TSSBN 动力非线性求解...");
        qDebug().noquote() << QStringLiteral("rho_inf=%1, dt=%2").arg(m_param.rho_inf).arg(m_param.dt);

        const int nDofs = model.GetFreeDofs();
        if (nDofs <= 0)
        {
            qDebug().noquote() << QStringLiteral("错误: 自由度数量无效");
            return false;
        }

        double dt = m_param.dt;
        if (dt <= 0.0)
        {
            qDebug().noquote() << QStringLiteral("错误: 时间步长必须为正");
            return false;
        }

        // 初始化工作区
        m_dx.resize(nDofs);
        m_R.resize(nDofs);
        m_totalDx_c1.resize(nDofs);
        m_totalDx_c2.resize(nDofs);
        m_Uc1.resize(nDofs);
        m_Vc1.resize(nDofs);
        m_Ac1.resize(nDofs);
        m_Uc2.resize(nDofs);
        m_Vc2.resize(nDofs);
        m_Ac2.resize(nDofs);
        m_Un.resize(nDofs);
        m_Vn.resize(nDofs);
        m_An.resize(nDofs);

        m_cache.reset();

        // 获取初始状态
        model.GetState(m_Un, m_Vn, m_An);

        int numSteps = static_cast<int>(duration / dt);

        // 时间步循环
        for (int step = 1; step <= numSteps; ++step)
        {
            double currentTime = step * dt;

            // ========== C1 子步 ==========
            if (!SolveSubstepC1(model, currentTime, dt))
            {
                qDebug().noquote() << QStringLiteral("错误: C1 子步求解失败");
                return false;
            }

            // ========== C2 子步 ==========
            if (!SolveSubstepC2(model, currentTime, dt))
            {
                qDebug().noquote() << QStringLiteral("错误: C2 子步求解失败");
                return false;
            }

            // ========== 组合最终状态 ==========
            // U_{n+1} = U_n + dt * (b1 * V_c1 + (1-b1) * V_c2)
            // V_{n+1} = V_n + dt * (b1 * A_c1 + (1-b1) * A_c2)
            Vec Un_new = m_Un + dt * (m_param.b1 * m_Vc1 + (1.0 - m_param.b1) * m_Vc2);
            Vec Vn_new = m_Vn + dt * (m_param.b1 * m_Ac1 + (1.0 - m_param.b1) * m_Ac2);

            // 计算最终位移增量
            Vec totalDx = Un_new - m_Un;

            // 更新模型状态
            model.ApplyIncrement(totalDx);
            model.SetTrialKinematics(Vn_new, m_An);  // 先用旧的加速度

            // 计算最终加速度（通过求解 M*a = F_ext - F_int - C*v）
            //model.AssembleMatrices(m_K, &m_M, &m_C);
            //model.ComputeResidual(currentTime, 1.0, m_R);

            // 求解加速度: M * a = R (这里 R 应该是 F_ext - F_int - C*v)
            // 简化处理：假设 ComputeResidual 返回的是不含惯性项的残差
            Vec An_new;
            if (!SolveLinear(m_M, m_R, An_new))
            {
                qDebug().noquote() << QStringLiteral("错误: 最终加速度求解失败");
                return false;
            }

            // 更新最终加速度
            model.SetTrialKinematics(Vn_new, An_new);

            // 提交状态
            Vec zero = Vec::Zero(nDofs);
            model.ApplyIncrement(zero);

            // 更新上一时刻状态为下一步做准备
            m_Un = Un_new;
            m_Vn = Vn_new;
            m_An = An_new;

            // 步回调
            if (m_callback)
            {
                Vec u, v, a;
                model.GetState(u, v, a);
                m_callback(step, currentTime, u);
            }

            // 步结束回调
            model.CommitState();
            model.OnStepCompleted(currentTime);
        }

        qDebug().noquote() << QStringLiteral("TSSBN 动力求解完成");
        return true;
    }

    bool SolverTSSBN::SolveSubstepC1(IAnalysisModel& model, double currentTime, double dt)
    {
        const int nDofs = model.GetFreeDofs();
        double c1_dt = m_param.c1 * dt;
        double c1_dt2 = c1_dt * c1_dt;

        // 初始猜测：C1 位移 = 上一时刻位移
        m_Uc1 = m_Un;
        m_totalDx_c1.setZero();

        // Newton-Raphson 迭代
        for (int iter = 0; iter < m_param.maxIter; ++iter)
        {
            // 1. 根据当前 C1 位移计算速度和加速度
            // V_c1 = (U_c1 - U_n) / (c1*dt)
            // A_c1 = (V_c1 - V_n) / (c1*dt)
            m_Vc1 = (m_Uc1 - m_Un) / c1_dt;
            m_Ac1 = (m_Vc1 - m_Vn) / c1_dt;

            // 2. 设置试探运动学状态
            model.SetTrialKinematics(m_Vc1, m_Ac1);

            // 3. 组装矩阵
            //model.AssembleMatrices(m_K, &m_M, &m_C);

            // 4. 有效刚度矩阵: K_eff = K + M/(c1*dt)^2 + C/(c1*dt)
            m_Keff = m_K + (1.0 / c1_dt2) * m_M + (1.0 / c1_dt) * m_C;

            // 5. 计算残差
            //model.ComputeResidual(currentTime + c1_dt, 1.0, m_R);

            // 6. 收敛检查
            double error = m_R.norm();
            if (error < m_param.tol && iter > 0)
            {
                return true;
            }

            // 7. 检查是否达到最大迭代次数
            if (iter == m_param.maxIter - 1)
            {
                qDebug().noquote() << QStringLiteral("警告: C1 子步未收敛 (残差=%1)").arg(error);
                return false;
            }

            // 8. 求解线性方程组: K_eff * dx = R
            if (!SolveLinear(m_Keff, m_R, m_dx))
            {
                qDebug().noquote() << QStringLiteral("错误: C1 子步线性求解失败");
                return false;
            }

            // 9. 累加位移增量
            m_totalDx_c1 += m_dx;
            m_Uc1 += m_dx;

            // 10. 更新模型的位移状态 (试探)
            model.ApplyIncrement(m_dx);
        }

        return true;
    }

    bool SolverTSSBN::SolveSubstepC2(IAnalysisModel& model, double currentTime, double dt)
    {
        const int nDofs = model.GetFreeDofs();
        double c2_dt = m_param.c2 * dt;
        double alpha_c2_dt = m_param.alpha * c2_dt;
        double alpha_c2_dt2 = alpha_c2_dt * alpha_c2_dt;

        // 初始猜测：C2 位移 = C1 位移
        m_Uc2 = m_Uc1;
        m_totalDx_c2.setZero();

        // 预测值（使用 C1 的结果和 alpha 参数）
        Vec u_pred = m_Un + c2_dt * (1.0 - m_param.alpha) * m_Vc1;
        Vec v_pred = m_Vn + c2_dt * (1.0 - m_param.alpha) * m_Ac1;

        // Newton-Raphson 迭代
        for (int iter = 0; iter < m_param.maxIter; ++iter)
        {
            // 1. 根据当前 C2 位移计算速度和加速度
            // V_c2 = (U_c2 - u_pred) / (alpha*c2*dt)
            // A_c2 = (V_c2 - v_pred) / (alpha*c2*dt)
            m_Vc2 = (m_Uc2 - u_pred) / alpha_c2_dt;
            m_Ac2 = (m_Vc2 - v_pred) / alpha_c2_dt;

            // 2. 设置试探运动学状态
            model.SetTrialKinematics(m_Vc2, m_Ac2);

            // 3. 组装矩阵
            //model.AssembleMatrices(m_K, &m_M, &m_C);

            // 4. 有效刚度矩阵: K_eff = K + M/(alpha*c2*dt)^2 + C/(alpha*c2*dt)
            m_Keff = m_K + (1.0 / alpha_c2_dt2) * m_M + (1.0 / alpha_c2_dt) * m_C;

            // 5. 计算残差
            //model.ComputeResidual(currentTime + c2_dt, 1.0, m_R);

            // 6. 收敛检查
            double error = m_R.norm();
            if (error < m_param.tol && iter > 0)
            {
                return true;
            }

            // 7. 检查是否达到最大迭代次数
            if (iter == m_param.maxIter - 1)
            {
                qDebug().noquote() << QStringLiteral("警告: C2 子步未收敛 (残差=%1)").arg(error);
                return false;
            }

            // 8. 求解线性方程组: K_eff * dx = R
            if (!SolveLinear(m_Keff, m_R, m_dx))
            {
                qDebug().noquote() << QStringLiteral("错误: C2 子步线性求解失败");
                return false;
            }

            // 9. 累加位移增量
            m_totalDx_c2 += m_dx;
            m_Uc2 += m_dx;

            // 10. 更新模型的位移状态 (试探)
            model.ApplyIncrement(m_dx);
        }

        return true;
    }

    bool SolverTSSBN::SolveLinear(const SpMat& K, const Vec& b, Vec& x)
    {
        bool solved = false;

        // 尝试 LDLT（更快）
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

        // 如果 LDLT 失败，尝试 LU
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
