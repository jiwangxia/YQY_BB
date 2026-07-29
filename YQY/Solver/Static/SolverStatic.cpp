#include "SolverStatic.h"

#include "Solver/Constraint/NonlinearMPC.h"

#include <QDebug>

#include <algorithm>

namespace SolverNameSpace
{
    bool SolverStatic::Solve(IAnalysisModel& model, double duration)
    {
        qDebug().noquote()
            << QStringLiteral("开始静力非线性求解 (Newton-Raphson)...");

        const int nDofs = model.GetFreeDofs();
        if (nDofs < 0)
            return false;

        m_dx = Vec::Zero(nDofs);
        m_R = Vec::Zero(nDofs);
        m_cache.reset();

        for (int increment = 1;
             increment <= m_param.numIncrements; ++increment)
        {
            if (model.IsCancellationRequested())
                return false;

            const double factor =
                static_cast<double>(increment) / m_param.numIncrements;
            const double currentTime = duration * factor;

            Vec prescribedDisplacement;
            model.Assemble_Constraint(
                prescribedDisplacement, currentTime, factor);
            Vec fixedExternalForce;
            Vec freeExternalForce;
            model.ComputeExternalForce(
                currentTime, factor,
                fixedExternalForce, freeExternalForce);

            Vec accumulatedIncrement = Vec::Zero(nDofs);
            Vec convergedMultipliers;
            double initialResidualNorm = 1.0;
            bool converged = false;
            int iterationCount = 0;

            for (int iteration = 0;
                 iteration < m_param.maxIter; ++iteration)
            {
                iterationCount = iteration + 1;
                model.Assemble_Matrix(m_K, false);
                model.ComputeResidual(freeExternalForce, m_R);

                NonlinearMPCData constraints;
                if (!model.AssembleNonlinearMPC(constraints))
                    return false;

                SpMat solveTangent;
                Vec solveRhs;
                NonlinearMPCReduction reduction;
                double constraintNorm = 0.0;
                if (constraints.Empty())
                {
                    solveTangent = m_K;
                    solveRhs = m_R;
                }
                else
                {
                    if (!NonlinearMPC::Reduce(
                            m_K, m_R, constraints, reduction))
                    {
                        model.ReportProgress(
                            factor,
                            QStringLiteral(
                                "静力增量 %1：MPC 消元矩阵奇异")
                                .arg(increment));
                        qDebug().noquote()
                            << QStringLiteral(
                                "Error: 非线性MPC消元失败，"
                                "从自由度Jacobian可能奇异");
                        return false;
                    }
                    solveTangent = reduction.tangent;
                    solveRhs = reduction.rhs;
                    constraintNorm = reduction.constraintNorm;
                    convergedMultipliers = reduction.multipliers;
                }

                const double residualNorm = solveRhs.norm();
                if (iteration == 0)
                {
                    initialResidualNorm =
                        std::max(residualNorm, 1.0e-16);
                    if (residualNorm < m_param.tol_R
                        && constraintNorm < m_param.tol_C)
                    {
                        converged = true;
                        break;
                    }
                }
                else
                {
                    const double incrementNorm = m_dx.norm();
                    const double stateNorm =
                        accumulatedIncrement.norm();
                    const double relativeIncrement =
                        stateNorm > 1.0e-12
                        ? incrementNorm / stateNorm
                        : incrementNorm;
                    const bool displacementConverged =
                        relativeIncrement < m_param.tol_dx;
                    const bool forceConverged =
                        residualNorm
                        < m_param.tol_R
                            * std::max(1.0, initialResidualNorm);
                    const bool constraintConverged =
                        constraintNorm < m_param.tol_C;
                    if (displacementConverged && forceConverged
                        && constraintConverged)
                    {
                        converged = true;
                        break;
                    }
                }

                if (iteration == m_param.maxIter - 1)
                    break;

                Vec independentIncrement;
                if (!SolveLinear(
                        solveTangent, solveRhs, independentIncrement))
                {
                    model.ReportProgress(
                        factor,
                        QStringLiteral(
                            "静力增量 %1、迭代 %2：线性方程求解失败")
                            .arg(increment).arg(iteration + 1));
                    qDebug().noquote()
                        << QStringLiteral("Error: 线性方程求解失败");
                    return false;
                }

                m_dx = constraints.Empty()
                    ? independentIncrement
                    : reduction.RecoverFullIncrement(
                        independentIncrement);
                if (m_dx.size() != nDofs || !m_dx.allFinite())
                    return false;

                accumulatedIncrement += m_dx;
                model.ApplyIncrement(m_dx);
            }

            if (!converged)
            {
                model.ReportProgress(
                    factor,
                    QStringLiteral(
                        "静力增量 %1：达到 %2 次迭代仍未收敛")
                        .arg(increment).arg(m_param.maxIter));
                qDebug().noquote()
                    << QStringLiteral(
                        "静力增量%1达到最大迭代次数仍未收敛")
                           .arg(increment);
                return false;
            }

            model.SetNonlinearMPCMultipliers(convergedMultipliers);
            model.CalculateReactions(fixedExternalForce);
            Vec displacement;
            Vec velocity;
            Vec acceleration;
            model.GetState(displacement, velocity, acceleration);
            model.CommitState();
            model.RecordStepIterations(currentTime, iterationCount);
            model.OnStepCompleted(currentTime);
            model.ReportProgress(
                factor,
                QStringLiteral("静力增量 %1/%2")
                    .arg(increment).arg(m_param.numIncrements));

            if (m_callback)
                m_callback(increment, currentTime, displacement);
        }

        qDebug().noquote() << QStringLiteral("静力非线性求解完成");
        return true;
    }

    bool SolverStatic::SolveLinear(
        const SpMat& tangent,
        const Vec& rightHandSide,
        Vec& solution)
    {
        bool solved = false;

        if (m_cache.useLdlt)
        {
            if (!m_cache.patternAnalyzed)
                m_cache.ldlt.analyzePattern(tangent);
            m_cache.ldlt.factorize(tangent);

            if (m_cache.ldlt.info() == Eigen::Success)
            {
                solution = m_cache.ldlt.solve(rightHandSide);
                if (m_cache.ldlt.info() == Eigen::Success)
                {
                    solved = true;
                    m_cache.patternAnalyzed = true;
                }
            }

            if (!solved)
            {
                m_cache.useLdlt = false;
                m_cache.patternAnalyzed = false;
            }
        }

        if (!solved)
        {
            if (!m_cache.patternAnalyzed)
            {
                m_cache.lu.analyzePattern(tangent);
                m_cache.patternAnalyzed = true;
            }
            m_cache.lu.factorize(tangent);

            if (m_cache.lu.info() == Eigen::Success)
            {
                solution = m_cache.lu.solve(rightHandSide);
                solved = m_cache.lu.info() == Eigen::Success;
            }
            else
            {
                m_cache.patternAnalyzed = false;
            }
        }

        return solved && solution.allFinite();
    }
}
