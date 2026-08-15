#include "SolverStatic.h"

#include "Solver/Constraint/NonlinearMPC.h"

#include <QDebug>
#include <QElapsedTimer>
#include <QTextStream>

#include <algorithm>

namespace SolverNameSpace
{
bool SolverStatic::Solve(IAnalysisModel& model, double duration)
{
    qDebug().noquote() << QStringLiteral("开始静力非线性求解 (Newton-Raphson)...");

    const int nDofs = model.GetFreeDofs();
    if (nDofs < 0)
    {
        model.ReportProgress(0.0, QStringLiteral("Static solve failed: invalid free DOF count"));
        return false;
    }

    m_dx = Vec::Zero(nDofs);
    m_R = Vec::Zero(nDofs);
    m_linearSolver.Reset();

    const bool profile = qEnvironmentVariableIntValue("YQY_PROFILE_STATIC") != 0;
    qint64 matrixAndResidualNanoseconds = 0;
    qint64 mpcAssemblyNanoseconds = 0;
    qint64 mpcReductionNanoseconds = 0;
    qint64 linearSolveNanoseconds = 0;
    int totalNewtonIterations = 0;

    for (int increment = 1; increment <= m_param.numIncrements; ++increment)
    {
        if (model.IsCancellationRequested())
            return false;

        const double factor = static_cast<double>(increment) / m_param.numIncrements;
        const double currentTime = duration * factor;

        Vec prescribedDisplacement;
        model.Assemble_Constraint(prescribedDisplacement, currentTime, factor);
        Vec fixedExternalForce;
        Vec freeExternalForce;
        model.ComputeExternalForce(currentTime, factor, fixedExternalForce, freeExternalForce);

        Vec accumulatedIncrement = Vec::Zero(nDofs);
        Vec convergedMultipliers;
        double initialResidualNorm = 1.0;
        bool converged = false;
        int iterationCount = 0;

        for (int iteration = 0; iteration < m_param.maxIter; ++iteration)
        {
            iterationCount = iteration + 1;
            QElapsedTimer profileTimer;
            if (profile)
                profileTimer.start();
            model.Assemble_Matrix(m_K, false);
            model.ComputeResidual(freeExternalForce, m_R);
            if (profile)
                matrixAndResidualNanoseconds += profileTimer.nsecsElapsed();

            NonlinearMPCData constraints;
            if (profile)
                profileTimer.restart();
            if (!model.AssembleNonlinearMPC(constraints))
            {
                model.ReportProgress(factor, QStringLiteral("Static increment %1 iteration %2: MPC assembly failed")
                                                 .arg(increment)
                                                 .arg(iteration + 1));
                return false;
            }
            if (profile)
                mpcAssemblyNanoseconds += profileTimer.nsecsElapsed();

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
                if (profile)
                    profileTimer.restart();
                if (!NonlinearMPC::Reduce(m_K, m_R, constraints, reduction))
                {
                    model.ReportProgress(factor, QStringLiteral("静力增量 %1：MPC 消元矩阵奇异").arg(increment));
                    qDebug().noquote() << QStringLiteral("Error: 非线性MPC消元失败，"
                                                         "从自由度Jacobian可能奇异");
                    return false;
                }
                if (profile)
                    mpcReductionNanoseconds += profileTimer.nsecsElapsed();
                solveTangent = reduction.tangent;
                solveRhs = reduction.rhs;
                constraintNorm = reduction.constraintNorm;
                convergedMultipliers = reduction.multipliers;
            }

            const double residualNorm = solveRhs.norm();
            if (iteration == 0)
            {
                initialResidualNorm = std::max(residualNorm, 1.0e-16);
                if (residualNorm < m_param.tol_R && constraintNorm < m_param.tol_C)
                {
                    converged = true;
                    break;
                }
            }
            else
            {
                const double incrementNorm = m_dx.norm();
                const double stateNorm = accumulatedIncrement.norm();
                const double relativeIncrement = stateNorm > 1.0e-12 ? incrementNorm / stateNorm : incrementNorm;
                const bool displacementConverged = relativeIncrement < m_param.tol_dx;
                const bool forceConverged = residualNorm < m_param.tol_R * std::max(1.0, initialResidualNorm);
                const bool constraintConverged = constraintNorm < m_param.tol_C;
                if (displacementConverged && forceConverged && constraintConverged)
                {
                    converged = true;
                    break;
                }
            }

            if (iteration == m_param.maxIter - 1)
                break;

            Vec independentIncrement;
            if (profile)
                profileTimer.restart();
            if (!SolveLinear(solveTangent, solveRhs, independentIncrement))
            {
                model.ReportProgress(
                    factor, QStringLiteral("静力增量 %1、迭代 %2：线性方程求解失败").arg(increment).arg(iteration + 1));
                qDebug().noquote() << QStringLiteral("Error: 线性方程求解失败");
                return false;
            }
            if (profile)
                linearSolveNanoseconds += profileTimer.nsecsElapsed();

            m_dx = constraints.Empty() ? independentIncrement : reduction.RecoverFullIncrement(independentIncrement);
            if (m_dx.size() != nDofs || !m_dx.allFinite())
            {
                model.ReportProgress(
                    factor, QStringLiteral("Static increment %1 iteration %2: non-finite displacement correction")
                                .arg(increment)
                                .arg(iteration + 1));
                return false;
            }

            accumulatedIncrement += m_dx;
            model.ApplyIncrement(m_dx);
        }

        if (!converged)
        {
            model.ReportProgress(
                factor, QStringLiteral("静力增量 %1：达到 %2 次迭代仍未收敛").arg(increment).arg(m_param.maxIter));
            qDebug().noquote() << QStringLiteral("静力增量%1达到最大迭代次数仍未收敛").arg(increment);
            return false;
        }

        totalNewtonIterations += iterationCount;

        model.SetNonlinearMPCMultipliers(convergedMultipliers);
        model.CalculateReactions(fixedExternalForce);
        Vec displacement;
        Vec velocity;
        Vec acceleration;
        model.GetState(displacement, velocity, acceleration);
        model.CommitState();
        model.RecordStepIterations(currentTime, iterationCount);
        model.OnStepCompleted(currentTime);
        model.ReportProgress(factor, QStringLiteral("静力增量 %1/%2").arg(increment).arg(m_param.numIncrements));

        if (m_callback)
            m_callback(increment, currentTime, displacement);
    }

    qDebug().noquote() << QStringLiteral("静力非线性求解完成");
    if (profile)
    {
        QTextStream(stdout) << QStringLiteral("static_profile newton=%1 matrix_residual_ms=%2 "
                                              "mpc_assembly_ms=%3 mpc_reduction_ms=%4 linear_ms=%5")
                                   .arg(totalNewtonIterations)
                                   .arg(matrixAndResidualNanoseconds / 1.0e6, 0, 'f', 3)
                                   .arg(mpcAssemblyNanoseconds / 1.0e6, 0, 'f', 3)
                                   .arg(mpcReductionNanoseconds / 1.0e6, 0, 'f', 3)
                                   .arg(linearSolveNanoseconds / 1.0e6, 0, 'f', 3)
                            << Qt::endl;
    }
    return true;
}

bool SolverStatic::SolveLinear(const SpMat& tangent, const Vec& rightHandSide, Vec& solution)
{
    return m_linearSolver.Solve(tangent, rightHandSide, solution);
}
}
