/**
 * @file SolverStatic.cpp
 * @brief 静力求解器实现
 */
#include "SolverStatic.h"
#include <QDebug>
#include <iostream>
namespace SolverNameSpace
{
    bool SolverStatic::Solve(IAnalysisModel& model, double duration)
    {
        qDebug().noquote() << QStringLiteral("开始静力求解 (Newton-Raphson)...");
        qDebug().noquote() << QStringLiteral("总增量步数: %1, 总时间: %2 s").arg(m_param.numIncrements).arg(duration);

        const int nDofs = model.GetFreeDofs();
        if (nDofs <= 0)
        {
            qDebug().noquote() << QStringLiteral("错误: 自由度数量无效");
            return false;
        }

        // 初始化工作区
        m_dx.resize(nDofs);
        m_R.resize(nDofs);
        m_cache.reset();

        // 进度条参数
        const int barWidth = 50;  // 进度条宽度

        // 增量步循环
        for (int inc = 1; inc <= m_param.numIncrements; ++inc)
        {
            double factor = static_cast<double>(inc) / m_param.numIncrements;
            double currentTime = duration * factor;

            // 施加约束（包括位移控制）
            Eigen::VectorXd x1;
            model.Assemble_Constraint(x1, factor);
            // 组装外荷载（增量步开始时做一次）
            Vec F1, F2;
            model.ComputeExternalForce(currentTime, factor, F1, F2);

            // 显示进度条（使用 printf 支持 \r 动态刷新）
            int pos = static_cast<int>(barWidth * factor);
            printf("\rProgress: [");
            for (int i = 0; i < barWidth; ++i)
            {
                if (i < pos) printf("=");
                else if (i == pos) printf(">");
                else printf(" ");
            }
            printf("] %d%% (%d/%d) t=%.4fs", int(factor * 100.0), inc, m_param.numIncrements, currentTime);
            fflush(stdout);  // 强制刷新输出

            // 当前增量步的累计位移（用于相对收敛判据）
            Eigen::VectorXd m_x2 = Eigen::VectorXd::Zero(nDofs);

            // 初始残差范数（用于相对判据）
            double norm_R0 = 0.0;

            // Newton-Raphson 迭代
            for (int iter = 0; iter < m_param.maxIter; ++iter)
            {
                model.Assemble_Matrix(m_K, false);
                // 使用已组装的外荷载计算残差
                model.ComputeResidual(F2, m_R);

                double norm_R = m_R.norm();

                // 第一次迭代：记录初始残差
                if (iter == 0)
                {
                    norm_R0 = norm_R;

                    // 如果初始残差已经很小，直接收敛
                    if (norm_R0 < 1e-12)
                    {
                        break;
                    }
                }

                // 收敛判据检查（第一次迭代跳过）
                if (iter > 0)
                {
                    double norm_dx = m_dx.norm();
                    double norm_x2 = m_x2.norm();
                    double norm_R = m_R.norm();

                    // 计算相对位移：dx / x2
                    double relative_dx = (norm_x2 > 1e-12) ? (norm_dx / norm_x2) : norm_dx;

                    // 多重收敛判据
                    bool disp_converged = (relative_dx < m_param.tol_dx);
                    bool force_converged = (norm_R < m_param.tol_R * norm_R0) || (norm_R < 1e-6);

                    // 位移和力同时满足
                    bool converged = disp_converged && force_converged;


                    // 调试输出：如果不收敛，打印详细信息
                    if (!converged && iter > m_param.maxIter - 3)
                    {
                        printf("  [t=%.4f, iter=%2d] norm_dx=%.3e, norm_x2=%.3e, relative_dx=%.3e\n",
                            currentTime, iter, norm_dx, norm_x2, relative_dx);
                    }

                    if (converged)
                    {
                        break;
                    }
                }

                // 检查是否达到最大迭代次数
                if (iter == m_param.maxIter - 1)
                {
                    qDebug().noquote() << QStringLiteral("失败: 达最大迭代次数");
                    return false;
                }

                // 求解线性方程组: K * dx = R
                if (!SolveLinear(m_K, m_R, m_dx))
                {
                    qDebug().noquote() << QStringLiteral("错误: 线性求解失败");
                    return false;
                }

                // 累加到当前增量步的总位移
                m_x2 += m_dx;

                // 更新试探状态
                model.ApplyIncrement(m_dx);
            }

            model.CalculateReactions(F1);
            //// 输出当前增量步的位移信息
            Vec u, v, a;
            model.GetState(u, v, a);

            model.OnStepCompleted(currentTime);

            // 步回调
            if (m_callback)
            {
                m_callback(inc, currentTime, u);
            }
        }

        // 进度条完成后换行
        printf("\n");

        // 求解完成回调
        //model.OnStepCompleted(duration);

        qDebug().noquote() << QStringLiteral("静力求解完成");
        return true;
    }

    bool SolverStatic::SolveLinear(const SpMat& K, const Vec& b, Vec& x)
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
