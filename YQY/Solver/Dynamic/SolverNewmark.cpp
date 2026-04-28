/**
 * @file SolverNewmark.cpp
 * @brief Newmark-β 动力求解器实现
 */
#include "SolverNewmark.h"
#include <QDebug>
#include <stdexcept>

namespace SolverNameSpace
{
    SolverNewmark::SolverNewmark(Params p) : m_param(p)
    {
        ComputeCoeffs(p.dt);
    }

    void SolverNewmark::ComputeCoeffs(double dt)
    {
        if (dt <= 0.0)
        {
            throw std::invalid_argument("Time step must be positive");
        }

        double beta = m_param.beta;
        double gamma = m_param.gamma;

        m_c.a0 = 1.0 / (beta * dt * dt);
        m_c.a1 = gamma / (beta * dt);
        m_c.a2 = 1.0 / (beta * dt);
        m_c.a3 = 1.0 / (2.0 * beta) - 1.0;
        m_c.a4 = gamma / beta - 1.0;
        m_c.a5 = dt * 0.5 * (gamma / beta - 2.0);
        m_c.a6 = dt * (1.0 - gamma);
        m_c.a7 = gamma * dt;
    }

    bool SolverNewmark::Solve(IAnalysisModel& model, double duration)
    {
        qDebug().noquote() << QStringLiteral("开始 Newmark 动力非线性求解...");

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
        m_totalDx.resize(nDofs);
        m_Acurr.resize(nDofs);
        m_Vcurr.resize(nDofs);
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

            // 备份上一时刻状态 (已在循环末尾更新)
            // 第一步时 m_Un, m_Vn, m_An 已由 GetState 初始化

            // 当前步累计位移增量清零
            m_totalDx.setZero();

            // Newton-Raphson 迭代
            for (int iter = 0; iter < m_param.maxIter; ++iter)
            {
                // 1. 根据 Newmark 公式计算试探的加速度和速度
                // A_{n+1} = a0 * ΔU - a2 * V_n - a3 * A_n
                // V_{n+1} = V_n + a6 * A_n + a7 * A_{n+1}
                m_Acurr = m_c.a0 * m_totalDx - m_c.a2 * m_Vn - m_c.a3 * m_An;
                m_Vcurr = m_Vn + m_c.a6 * m_An + m_c.a7 * m_Acurr;

                // 2. 将试探的 V 和 A 设置到模型中
                model.SetTrialKinematics(m_Vcurr, m_Acurr);

                // 3. 组装 K, M, C 矩阵
                model.AssembleMatrices(m_K, &m_M, &m_C);

                // 4. 计算有效刚度矩阵: K_eff = K + a0*M + a1*C
                m_Keff = m_K + m_c.a0 * m_M + m_c.a1 * m_C;

                // 5. 计算残差: R = F_ext - F_int - M*a - C*v
                // 模型的 ComputeResidual 应该已经包含了惯性力和阻尼力
                model.ComputeResidual(currentTime, 1.0, m_R);

                // 6. 收敛检查
                double error = m_R.norm();
                if (error < m_param.tol && iter > 0)
                {
                    //qDebug().noquote() << QStringLiteral("  时间步 %1: 迭代 %2 次收敛").arg(step).arg(iter);
                    break;
                }

                // 7. 检查是否达到最大迭代次数
                if (iter == m_param.maxIter - 1)
                {
                    qDebug().noquote() << QStringLiteral("警告: 时间步 %1 未收敛 (残差=%2)")
                                              .arg(step).arg(error);
                    return false;
                }

                // 8. 求解线性方程组: K_eff * dx = R
                if (!SolveLinear(m_Keff, m_R, m_dx))
                {
                    qDebug().noquote() << QStringLiteral("错误: 线性求解失败");
                    return false;
                }

                // 9. 累加位移增量
                m_totalDx += m_dx;

                // 10. 更新模型的位移状态 (试探)
                model.ApplyIncrement(m_dx);
            }

            // 步末计算最终的 V 和 A
            Vec Afinal = m_c.a0 * m_totalDx - m_c.a2 * m_Vn - m_c.a3 * m_An;
            Vec Vfinal = m_Vn + m_c.a6 * m_An + m_c.a7 * Afinal;

            // 更新模型的最终速度和加速度
            model.SetTrialKinematics(Vfinal, Afinal);

            // 提交状态
            Vec zero = Vec::Zero(nDofs);
            model.ApplyIncrement(zero);

            // 更新上一时刻状态为下一步做准备
            m_Un += m_totalDx;  // 注意：实际位移已在模型中更新
            m_Vn = Vfinal;
            m_An = Afinal;

            // 步回调
            if (m_callback)
            {
                Vec u, v, a;
                model.GetState(u, v, a);
                m_callback(step, currentTime, u);
            }

            // 步结束回调（保存结果等）
            model.OnStepCompleted(currentTime);
        }

        qDebug().noquote() << QStringLiteral("动力求解完成");
        return true;
    }

    bool SolverNewmark::SolveLinear(const SpMat& K, const Vec& b, Vec& x)
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
