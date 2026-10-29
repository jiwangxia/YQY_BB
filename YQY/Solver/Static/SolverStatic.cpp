#include "SolverStatic.h"

#include "Solver/Constraint/NonlinearMPC.h"
#include "Solver/Dynamic/TimeStepping/StateCheckpointGuard.h"

#include <QDebug>
#include <QElapsedTimer>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>

namespace SolverNameSpace
{
bool SolverStatic::Solve(IAnalysisModel& model, double duration)
{
    if (m_param.referenceEquilibrium)
        return SolveReferenceEquilibrium(model, duration);

    qDebug().noquote() << QStringLiteral("开始静力非线性求解 (Newton-Raphson)...");

    const int nDofs = model.GetFreeDofs();
    if (nDofs < 0 || !std::isfinite(duration) || duration <= 0.0 || m_param.numIncrements < 1 ||
        m_param.maxIter < 1 || !std::isfinite(m_param.tol_R) || m_param.tol_R <= 0.0 ||
        !std::isfinite(m_param.tol_dx) || m_param.tol_dx <= 0.0 || !std::isfinite(m_param.tol_C) ||
        m_param.tol_C <= 0.0)
    {
        model.ReportProgress(0.0, QStringLiteral("静力求解参数或自由度数量无效"));
        return false;
    }

    m_dx = Vec::Zero(nDofs);
    m_R = Vec::Zero(nDofs);
    m_linearSolver.Reset();
    m_linearSolver.SetAllowGpuAutomatic(model.ShouldPreferGpuLinearSolver());

    const bool profile = qEnvironmentVariableIntValue("YQY_PROFILE_STATIC") != 0;
    qint64 matrixAndResidualNanoseconds = 0;
    qint64 mpcAssemblyNanoseconds = 0;
    qint64 mpcReductionNanoseconds = 0;
    qint64 linearSolveNanoseconds = 0;
    int totalNewtonIterations = 0;
    // 静力步的收敛尺度应由整个荷载路径的初始残差决定，而不是每个增量
    // 各自重新缩小。后者会在前一增量已经收敛后，把残余舍入误差当成新的
    // 基准，导致下一增量提出不现实的更高精度要求并错误报不收敛。
    double solveResidualScale = 1.0;

    for (std::int64_t increment = 1; increment <= m_param.numIncrements; ++increment)
    {
        if (model.IsCancellationRequested())
            return false;

        StateCheckpointGuard checkpoint(model, true);
        if (!checkpoint.IsActive())
        {
            model.ReportProgress(0.0, QStringLiteral("静力增量状态检查点创建失败"));
            return false;
        }
        const double factor = static_cast<double>(increment) / m_param.numIncrements;
        const double currentTime = duration * factor;

        Vec prescribedDisplacement;
        model.Assemble_Constraint(prescribedDisplacement, currentTime, factor);
        Vec fixedExternalForce;
        Vec freeExternalForce;

        Vec accumulatedIncrement = Vec::Zero(nDofs);
        Vec convergedMultipliers;
        bool converged = false;
        int iterationCount = 0;
        double lastResidualNorm = 0.0;
        double lastRelativeIncrement = 0.0;
        double lastConstraintNorm = 0.0;

        for (int iteration = 0; iteration < m_param.maxIter; ++iteration)
        {
            iterationCount = iteration + 1;
            QElapsedTimer profileTimer;
            if (profile)
                profileTimer.start();
            model.Assemble_Matrix(m_K, false);
            model.ComputeExternalForce(currentTime, factor, fixedExternalForce, freeExternalForce);
            model.ComputeResidual(freeExternalForce, m_R);
            SpMat externalTangent;
            model.AssembleExternalLoadTangent(currentTime, factor, 0.0, externalTangent);
            m_K -= externalTangent;
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
            lastResidualNorm = residualNorm;
            lastConstraintNorm = constraintNorm;
            if (iteration == 0)
            {
                solveResidualScale = std::max(solveResidualScale, residualNorm);
                if (residualNorm < m_param.tol_R * solveResidualScale && constraintNorm < m_param.tol_C)
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
                lastRelativeIncrement = relativeIncrement;
                const bool displacementConverged = relativeIncrement < m_param.tol_dx;
                const bool forceConverged = residualNorm < m_param.tol_R * solveResidualScale;
                const bool forceStronglyConverged = residualNorm < 0.05 * m_param.tol_R * solveResidualScale;
                const bool constraintConverged = constraintNorm < m_param.tol_C;
                // 残差低于常规阈值的 5% 时，不再用当前荷载增量内的修正比值否定收敛。
                if (((displacementConverged && forceConverged) || forceStronglyConverged) && constraintConverged)
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
            const std::uint64_t patternVersion = constraints.Empty() ? model.GetSparsePatternVersion() : 0;
            if (!SolveLinear(solveTangent, solveRhs, independentIncrement, patternVersion))
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
            if (profile)
            {
                QTextStream(stdout) << "static_nonconverged increment=" << increment
                                    << " residual=" << lastResidualNorm
                                    << " residual_limit=" << m_param.tol_R * solveResidualScale
                                    << " relative_increment=" << lastRelativeIncrement
                                    << " increment_limit=" << m_param.tol_dx
                                    << " constraint=" << lastConstraintNorm
                                    << " constraint_limit=" << m_param.tol_C << Qt::endl;
            }
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
        checkpoint.Release();
        model.RecordStepIterations(currentTime, iterationCount);
        model.OnStepCompleted(currentTime);
        model.ReportProgress(factor, QStringLiteral("静力增量 %1/%2").arg(increment).arg(m_param.numIncrements));

        if (m_callback)
            m_callback(static_cast<int>(increment), currentTime, displacement);
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

bool SolverStatic::SolveReferenceEquilibrium(IAnalysisModel& model, double duration)
{
    const int freeDofs = model.GetFreeDofs();
    if (freeDofs < 0 || !std::isfinite(duration) || duration <= 0.0 ||
        !std::isfinite(m_param.referenceTimeStep) || m_param.referenceTimeStep <= 0.0 || m_param.maxIter < 1 ||
        !std::isfinite(m_param.tol_R) || m_param.tol_R <= 0.0 || !std::isfinite(m_param.tol_dx) ||
        m_param.tol_dx <= 0.0 || !std::isfinite(m_param.tol_C) || m_param.tol_C <= 0.0)
        return false;

    m_linearSolver.Reset();
    m_linearSolver.SetPreferLdlt(false);
    m_linearSolver.SetModeOverride(LinearSolverMode::Lu);
    m_linearSolver.SetAllowGpuAutomatic(false);
    std::vector<int> referencePermutation;
    if (model.GetReferenceLinearSolvePermutation(referencePermutation) &&
        !m_linearSolver.SetSignedPermutation(referencePermutation))
        return false;

    m_dx = Vec::Zero(freeDofs);
    const bool trace = qEnvironmentVariableIntValue("YQY_TRACE_REFERENCE_STATIC") != 0;
    double nextTime = m_param.referenceTimeStep;
    int increment = 0;
    while (true)
    {
        if (model.IsCancellationRequested())
            return false;
        StateCheckpointGuard checkpoint(model, true);
        if (!checkpoint.IsActive())
        {
            model.ReportProgress(0.0, QStringLiteral("静力增量状态检查点创建失败"));
            return false;
        }
        const double currentTime = std::min(nextTime, duration);
        const double factor = currentTime / duration;
        Vec prescribedDisplacement;
        model.Assemble_Constraint(prescribedDisplacement, currentTime, factor);
        Vec fixedExternalForce;
        Vec freeExternalForce;
        // 参考静力步在增量开始时计算并冻结外荷载，Newton 内只更新结构内力。
        model.ComputeExternalForce(currentTime, factor, fixedExternalForce, freeExternalForce);
        Vec previousConvergenceResidual;
        Vec convergedMultipliers;
        bool converged = false;
        int iterationCount = 0;
        double rhsChange = 0.0;
        double correctionNorm = 0.0;
        for (int iteration = 0; iteration < m_param.maxIter; ++iteration)
        {
            model.Assemble_Matrix(m_K, false);
            model.ComputeResidual(freeExternalForce, m_R);
            NonlinearMPCData constraints;
            if (!model.AssembleNonlinearMPC(constraints))
                return false;

            SpMat solveTangent;
            Vec solveRhs;
            NonlinearMPCReduction reduction;
            if (constraints.Empty())
            {
                solveTangent = m_K;
                solveRhs = m_R;
            }
            else
            {
                if (!NonlinearMPC::Reduce(m_K, m_R, constraints, reduction))
                    return false;
                solveTangent = reduction.tangent;
                solveRhs = reduction.rhs;
                convergedMultipliers = reduction.multipliers;
            }
            Vec independentIncrement;
            const std::uint64_t patternVersion = constraints.Empty() ? model.GetSparsePatternVersion() : 0;
            if (!SolveLinear(solveTangent, solveRhs, independentIncrement, patternVersion))
            {
                if (trace)
                {
                    const bool hasSolution = independentIncrement.size() == solveRhs.size() &&
                                             independentIncrement.allFinite();
                    const double linearResidual = hasSolution
                        ? (solveTangent * independentIncrement - solveRhs).norm()
                        : std::numeric_limits<double>::quiet_NaN();
                    QTextStream(stdout) << "YQY_REFERENCE_STATIC_LINEAR_FAILURE time=" << currentTime
                                        << " iteration=" << iteration + 1 << " rhs_norm=" << solveRhs.norm()
                                        << " solution_available=" << hasSolution
                                        << " linear_residual=" << linearResidual << Qt::endl;
                }
                return false;
            }
            m_dx = constraints.Empty() ? independentIncrement : reduction.RecoverFullIncrement(independentIncrement);
            if (m_dx.size() != freeDofs || !m_dx.allFinite())
                return false;

            // 参考实现在更新状态后才检查本次修正，不能在修正前提前接受。
            model.ApplyIncrement(m_dx);
            model.UpdateInternalForce();
            model.ComputeResidual(freeExternalForce, m_R);
            Vec convergenceResidual;
            model.GetReferenceConvergenceResidual(m_R, fixedExternalForce, convergenceResidual);
            if (previousConvergenceResidual.size() != convergenceResidual.size())
                previousConvergenceResidual = Vec::Zero(convergenceResidual.size());
            rhsChange = model.GetReferenceVectorNorm(convergenceResidual - previousConvergenceResidual);
            previousConvergenceResidual = convergenceResidual;
            correctionNorm = std::hypot(model.GetReferenceVectorNorm(m_dx), prescribedDisplacement.norm());
            ++iterationCount;
            NonlinearMPCData updatedConstraints;
            if (!model.AssembleNonlinearMPC(updatedConstraints))
                return false;
            const bool constraintsConverged =
                updatedConstraints.Empty() || updatedConstraints.value.norm() < m_param.tol_C;
            if ((rhsChange < m_param.tol_R || correctionNorm < m_param.tol_dx) && constraintsConverged)
            {
                converged = true;
                break;
            }
        }
        if (!converged)
        {
            model.ReportProgress(factor, QStringLiteral("参考静力增量 %1：达到 %2 次迭代仍未收敛")
                                             .arg(increment + 1)
                                             .arg(m_param.maxIter));
            return false;
        }
        ++increment;
        model.SetNonlinearMPCMultipliers(convergedMultipliers);
        model.CalculateReactions(fixedExternalForce);
        Vec displacement;
        Vec velocity;
        Vec acceleration;
        model.GetState(displacement, velocity, acceleration);
        model.CommitState();
        checkpoint.Release();
        model.RecordStepIterations(currentTime, iterationCount);
        model.OnStepCompleted(currentTime);
        model.ReportProgress(factor, QStringLiteral("参考静力增量 %1").arg(increment));
        if (trace)
            QTextStream(stdout) << "YQY_REFERENCE_STATIC_INCREMENT time=" << currentTime
                                << " iterations=" << iterationCount << " rhs_change=" << rhsChange
                                << " correction=" << correctionNorm << " displacement_norm=" << displacement.norm()
                                << Qt::endl;
        if (m_callback)
            m_callback(increment, currentTime, displacement);
        if (currentTime >= duration)
            return true;
        nextTime += m_param.referenceTimeStep;
    }
}

bool SolverStatic::SolveLinear(const SpMat& tangent, const Vec& rightHandSide, Vec& solution,
                               std::uint64_t patternVersion)
{
    if (m_param.referenceEquilibrium)
        return m_linearSolver.SolveRefined(tangent, rightHandSide, solution, patternVersion);
    return m_linearSolver.Solve(tangent, rightHandSide, solution, patternVersion);
}
}
