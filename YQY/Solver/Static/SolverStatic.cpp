/**
 * @file SolverStatic.cpp
 * @brief 静力求解器实现
 */
#include "SolverStatic.h"
#include <QDebug>
#include <iostream>
namespace SolverNS
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

            // 显示进度条（使用 printf 支持 \r 动态刷新）
            int pos = static_cast<int>(barWidth * factor);
            printf("\rProgress: [");
            for (int i = 0; i < barWidth; ++i)
            {
                if (i < pos) printf("=");
                else if (i == pos) printf(">");
                else printf(" ");
            }
            printf("] %d%% (%d/%d) t=%.4fs",
                int(factor * 100.0), inc, m_param.numIncrements, currentTime);
            fflush(stdout);  // 强制刷新输出

            // 初始残差范数（用于相对判据）
            double norm_R0 = 0.0;

            // Newton-Raphson 迭代
            for (int iter = 0; iter < m_param.maxIter; ++iter)
            {
                // 1. 组装刚度矩阵 (只需要 K)
                model.AssembleMatrices(m_K, nullptr, nullptr);
                //std::cout << "\nK22:\n" << Eigen::MatrixXd(m_K);
                // 2. 计算残差: R = F_ext - F_int
                model.ComputeResidual(currentTime, factor, m_R);
                //std::cout << "\nR:\n" << Eigen::VectorXd(m_R).transpose() << std::endl;
                // 3. 收敛检查
                double norm_R = m_R.norm();

                // 第一次迭代：记录初始残差
                if (iter == 0)
                {
                    norm_R0 = norm_R;

                    // 如果初始残差已经很小，直接收敛
                    if (norm_R0 < 1e-12)
                    {
                        //qDebug().noquote() << QStringLiteral("  增量步 %1: 初始残差已满足要求 (|R0|=%2)")
                        //    .arg(inc).arg(norm_R0, 0, 'e', 2);
                        break;
                    }
                }

                // 计算相对残差
                double relative_R = (m_param.use_relative && norm_R0 > 1e-12) ? (norm_R / norm_R0) : norm_R;

                // 位移增量范数（第一次迭代还没有 dx，设为大值）
                double norm_dx = (iter > 0) ? m_dx.norm() : 1e10;

                // 收敛判据：力平衡 + 位移增量
                bool force_converged = (relative_R < m_param.tol_R);
                bool disp_converged = (norm_dx < m_param.tol_dx);

                if (force_converged && disp_converged && iter > 0)
                {
                    //qDebug().noquote() << QStringLiteral("  增量步 %1: 迭代 %2 次收敛 (|R|/|R0|=%3, |dx|=%4)")
                    //    .arg(inc).arg(iter)
                    //    .arg(relative_R, 0, 'e', 2)
                    //    .arg(norm_dx, 0, 'e', 2);
                    break;
                }

                // 4. 检查是否达到最大迭代次数
                if (iter == m_param.maxIter - 1)
                {
                    qDebug().noquote() << QStringLiteral("警告: 增量步 %1 未收敛 (|R|/|R0|=%2, |dx|=%3)")
                        .arg(inc)
                        .arg(relative_R, 0, 'e', 2)
                        .arg(norm_dx, 0, 'e', 2);
                    return false;
                }

                // 5. 求解线性方程组: K * dx = R
                if (!SolveLinear(m_K, m_R, m_dx))
                {
                    qDebug().noquote() << QStringLiteral("错误: 线性求解失败");
                    return false;
                }
                //std::cout << "\nx2 " << m_dx.transpose() << std::endl;
                // 6. 更新试探状态
                model.ApplyIncrement(m_dx, IAnalysisModel::Phase::Trial);

                // 可选：输出详细迭代信息（调试用）
                if (m_param.verbose && (iter % 5 == 0 || iter < 3))
                {
                    qDebug().noquote() << QStringLiteral("    Iter %1: |R|/|R0|=%2, |dx|=%3")
                        .arg(iter)
                        .arg(relative_R, 0, 'e', 2)
                        .arg(m_dx.norm(), 0, 'e', 2);
                }
            }

            // 增量步结束后提交状态
            Vec zero = Vec::Zero(nDofs);
            model.ApplyIncrement(zero, IAnalysisModel::Phase::Commit);

            //// 输出当前增量步的位移信息
            Vec u, v, a;
            model.GetState(u, v, a);

            //// 输出位移范数和最大位移
            //double u_norm = u.norm();
            //double u_max = u.cwiseAbs().maxCoeff();
            //qDebug().noquote() << QStringLiteral("    增量步 %1 完成: |u|=%2, max|u|=%3")
            //    .arg(inc)
            //    .arg(u_norm, 0, 'e', 2)
            //    .arg(u_max, 0, 'e', 2);

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

    //bool SolverStatic::Solve(IAnalysisModel& model, double duration)
    //{
    //    qDebug().noquote() << QStringLiteral("开始线性静力求解 (Newton-Raphson)...");

    //    const int nDofs = model.GetFreeDofs();
    //    if (nDofs <= 0)
    //    {
    //        qDebug().noquote() << QStringLiteral("错误: 自由度数量无效");
    //        return false;
    //    }

    //    // 初始化工作区
    //    m_dx.resize(nDofs);
    //    m_cache.reset();
    //    Eigen::VectorXd F1, F2;

    //    // 增量步循环
    //    for (int inc = 1; inc <= m_param.numIncrements; ++inc)
    //    {
    //        double factor = static_cast<double>(inc) / m_param.numIncrements;
    //        double currentTime = duration * factor;


    //        // 1. 组装刚度矩阵 (只需要 K)
    //        model.AssembleMatrices(m_K, nullptr, nullptr);
    //        model.Assemble_AllLoads(F1, F2, factor);
    //        std::cout << "F2: " << F2.transpose() << std::endl;
    //        // 5. 求解线性方程组: K * dx = R
    //        if (!SolveLinear(m_K, F2, m_dx))
    //        {
    //            qDebug().noquote() << QStringLiteral("错误: 线性求解失败");
    //            return false;
    //        }

    //        // 6. 更新试探状态
    //        model.ApplyIncrement(m_dx, IAnalysisModel::Phase::Trial);


    //        // 增量步结束后提交状态
    //        Vec zero = Vec::Zero(nDofs);
    //        model.ApplyIncrement(zero, IAnalysisModel::Phase::Commit);

    //        // 步回调
    //        if (m_callback)
    //        {
    //            Vec u, v, a;
    //            model.GetState(u, v, a);
    //            m_callback(inc, currentTime, u);
    //        }
    //    }

    //    // 求解完成回调
    //    model.OnStepCompleted(duration);

    //    qDebug().noquote() << QStringLiteral("静力线性求解完成");
    //    return true;
    //}

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
