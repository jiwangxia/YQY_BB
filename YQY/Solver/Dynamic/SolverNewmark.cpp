/**
 * @file SolverNewmark.cpp
 * @brief Newmark-beta nonlinear dynamic solver
 */
#include "SolverNewmark.h"

#include <QDebug>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace SolverNameSpace
{
    SolverNewmark::SolverNewmark(Params p) : m_param(p)
    {
        ComputeCoeffs(p.dt);
    }

    void SolverNewmark::ComputeCoeffs(double dt)
    {
        if (dt <= 0.0 || m_param.beta <= 0.0)
        {
            throw std::invalid_argument("Newmark dt and beta must be positive");
        }

        m_c.a0 = 1.0 / (m_param.beta * dt * dt);
        m_c.a1 = m_param.gamma / (m_param.beta * dt);
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
        if (m_param.dt <= 0.0 || m_param.beta <= 0.0 || duration <= 0.0)
        {
            qDebug().noquote() << QStringLiteral("错误: 时间步长、beta 和总时间必须为正");
            return false;
        }

        m_dx.resize(nDofs);
        m_R.resize(nDofs);
        m_Un.resize(nDofs);
        m_Vn.resize(nDofs);
        m_An.resize(nDofs);
        m_cache.reset();
        // The gyroscopic tangent is generally non-symmetric, therefore the
        // dynamic effective matrix must use the general sparse LU path.
        m_cache.useLdlt = false;
        model.GetState(m_Un, m_Vn, m_An);

        m_M.resize(nDofs, nDofs);
        m_C.resize(nDofs, nDofs);
        m_Kc.resize(nDofs, nDofs);

        const int numSteps = static_cast<int>(std::ceil(duration / m_param.dt));
        qDebug().noquote() << QStringLiteral("总时间步数: %1, 标准时间步长: %2 s, 总时间: %3 s")
            .arg(numSteps).arg(m_param.dt).arg(duration);

        double time = 0.0;
        for (int step = 1; step <= numSteps; ++step)
        {
            if (model.IsCancellationRequested())
                return false;
            const double dt = std::min(m_param.dt, duration - time);
            if (dt <= 0.0) break;

            ComputeCoeffs(dt);
            const double currentTime = time + dt;

            Vec F1, F2;
            model.ComputeExternalForce(currentTime, 1.0, F1, F2);
            model.BeginDynamicStep(dt, m_param.beta, m_param.gamma);

            bool converged = false;
            double error = std::numeric_limits<double>::infinity();
            int iterationCount = 0;
            for (int iter = 0; iter < m_param.maxIter; ++iter)
            {
                iterationCount = iter + 1;
                model.Assemble_Matrix(m_K, true);
                model.AssembleDynamicSystem(m_M, m_C, m_Kc);
                m_Keff = m_K
                    + m_c.a0 * m_M + m_c.a1 * m_C + m_Kc;
                model.ComputeResidual(F2, m_R);

                error = m_R.norm();
                if (!std::isfinite(error))
                {
                    qDebug().noquote()
                        << QStringLiteral("错误: 时间步 %1 的残差不是有限数").arg(step);
                    break;
                }
                if (error < m_param.tol)
                {
                    converged = true;
                    break;
                }

                if (!SolveLinear(m_Keff, m_R, m_dx))
                {
                    qDebug().noquote()
                        << QStringLiteral("错误: 时间步 %1 线性求解失败").arg(step);
                    break;
                }

                model.ApplyDynamicCorrection(m_dx, m_c.a0, m_c.a1);
            }

            if (!converged)
            {
                model.RollbackDynamicStep();
                QTextStream(stderr)
                    << "Newmark failed at step=" << step
                    << " time=" << currentTime
                    << " residual=" << error << Qt::endl;
                qDebug().noquote()
                    << QStringLiteral("警告: 时间步 %1 未收敛 (残差=%2)")
                        .arg(step).arg(error);
                return false;
            }

            model.CommitState();
            model.GetState(m_Un, m_Vn, m_An);
            model.RecordStepIterations(currentTime, iterationCount);
            if (m_callback)
            {
                m_callback(step, currentTime, m_Un);
            }
            model.OnStepCompleted(currentTime);
            time = currentTime;
            model.ReportProgress(std::min(1.0, currentTime / duration),
                QStringLiteral("动力时间步 %1/%2").arg(step).arg(numSteps));
        }

        qDebug().noquote() << QStringLiteral("动力求解完成");
        return true;
    }

    bool SolverNewmark::SolveLinear(const SpMat& K, const Vec& b, Vec& x)
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
                if (m_cache.ldlt.info() == Eigen::Success && x.allFinite())
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
                solved = m_cache.lu.info() == Eigen::Success && x.allFinite();
            }
            if (!solved)
            {
                qDebug() << "LU factorization or solve failed!";
                m_cache.patternAnalyzed = false;
            }
        }

        return solved;
    }
}
