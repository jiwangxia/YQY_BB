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

        // 增量步循环
        for (int inc = 1; inc <= m_param.numIncrements; ++inc)
        {
            double factor = static_cast<double>(inc) / m_param.numIncrements;
            double currentTime = duration * factor;

            // Newton-Raphson 迭代
            for (int iter = 0; iter < m_param.maxIter; ++iter)
            {
                // 1. 组装刚度矩阵 (只需要 K)
                model.AssembleMatrices(m_K, nullptr, nullptr);

                // 2. 计算残差: R = F_ext - F_int
                model.ComputeResidual(currentTime, factor, m_R);

                // 3. 收敛检查
                double error = m_R.norm();
                if (error < m_param.tol && iter > 0)
                {
                    //qDebug().noquote() << QStringLiteral("  增量步 %1: 迭代 %2 次收敛").arg(inc).arg(iter);
                    break;
                }

                // 4. 检查是否达到最大迭代次数
                if (iter == m_param.maxIter - 1)
                {
                    qDebug().noquote() << QStringLiteral("警告: 增量步 %1 未收敛 (残差=%2)")
                                              .arg(inc).arg(error);
                    return false;
                }

                // 5. 求解线性方程组: K * dx = R
                if (!SolveLinear(m_K, m_R, m_dx))
                {
                    qDebug().noquote() << QStringLiteral("错误: 线性求解失败");
                    return false;
                }

                // 6. 更新试探状态
                model.ApplyIncrement(m_dx, IAnalysisModel::Phase::Trial);
            }

            // 增量步结束后提交状态
            Vec zero = Vec::Zero(nDofs);
            model.ApplyIncrement(zero, IAnalysisModel::Phase::Commit);

            // 步回调
            if (m_callback)
            {
                Vec u, v, a;
                model.GetState(u, v, a);
                m_callback(inc, currentTime, u);
            }
        }

        // 求解完成回调
        model.OnStepCompleted(duration);

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
