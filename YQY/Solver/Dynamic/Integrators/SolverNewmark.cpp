/**
 * @file SolverNewmark.cpp
 * @brief Newmark-beta 非线性动力求解器。
 */
#include "SolverNewmark.h"
#include "../TimeStepping/TimeStepError.h"
#include "Solver/Constraint/NonlinearMPC.h"

#include <QDebug>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

namespace SolverNameSpace
{
SolverNewmark::SolverNewmark(Params parameters)
    : m_param(std::move(parameters))
{
    ComputeCoeffs(m_param.dt);
}

void SolverNewmark::ComputeCoeffs(double timeStep)
{
    if (!std::isfinite(timeStep) || timeStep <= 0.0 || !std::isfinite(m_param.beta) || m_param.beta <= 0.0)
        throw std::invalid_argument("Newmark 时间步长和 beta 必须为正");

    m_c.a0 = 1.0 / (m_param.beta * timeStep * timeStep);
    m_c.a1 = m_param.gamma / (m_param.beta * timeStep);
}

bool SolverNewmark::Solve(IAnalysisModel& model, double duration)
{
    qDebug().noquote() << QStringLiteral("开始 Newmark 动力非线性求解...");
    const int dofCount = model.GetFreeDofs();
    if (dofCount <= 0 || !std::isfinite(duration) || duration <= 0.0 || !std::isfinite(m_param.dt) ||
        m_param.dt <= 0.0 || !std::isfinite(m_param.beta) || m_param.beta <= 0.0 ||
        !std::isfinite(m_param.gamma) || m_param.gamma < 0.0 || m_param.maxIter < 1 || !std::isfinite(m_param.tol) ||
        m_param.tol <= 0.0 || !std::isfinite(m_param.constraintTolerance) || m_param.constraintTolerance <= 0.0)
    {
        qDebug().noquote() << QStringLiteral("Newmark 参数无效");
        return false;
    }
    if (m_param.timeStepMode != TimeStepMode::Fixed && m_param.timeStepMode != TimeStepMode::Adaptive)
    {
        qDebug().noquote() << QStringLiteral("Newmark 时间步模式无效");
        return false;
    }
    if (m_param.timeStepMode == TimeStepMode::Fixed &&
        (!std::isfinite(m_param.minimumTimeStep) || m_param.minimumTimeStep <= 0.0 ||
         m_param.minimumTimeStep > m_param.dt || !std::isfinite(m_param.cutbackFactor) ||
         m_param.cutbackFactor <= 0.0 || m_param.cutbackFactor >= 1.0 || !std::isfinite(m_param.recoveryFactor) ||
         m_param.recoveryFactor < 1.0 || m_param.maximumCutbacks < 0))
    {
        qDebug().noquote() << QStringLiteral("Newmark 固定步长回退参数无效");
        return false;
    }
    if (m_param.aerodynamicTangentMode < AerodynamicTangentMode::Disabled ||
        m_param.aerodynamicTangentMode > AerodynamicTangentMode::EveryNewtonIteration)
    {
        qDebug().noquote() << QStringLiteral("Newmark 气动切线模式无效");
        return false;
    }

    Eigen::MatrixXd dampingFactor;
    model.GetStructuralDampingFactor(dampingFactor);
    if (dampingFactor.rows() != dofCount)
        return false;

    std::unique_ptr<TimeStepController> adaptiveController;
    if (m_param.timeStepMode == TimeStepMode::Adaptive)
    {
        try
        {
            adaptiveController = std::make_unique<TimeStepController>(m_param.adaptiveTimeStep);
        }
        catch (const std::exception& error)
        {
            qDebug().noquote() << QStringLiteral("Newmark 自适应时间步参数无效：%1").arg(error.what());
            return false;
        }
        if (!model.PushStateCheckpoint())
        {
            qDebug().noquote() << QStringLiteral("模型不支持自适应时间步状态检查点");
            return false;
        }
        model.DiscardStateCheckpoint();
    }

    m_dx.resize(dofCount);
    m_R.resize(dofCount);
    m_Un.resize(dofCount);
    m_Vn.resize(dofCount);
    m_An.resize(dofCount);
    m_linearSolver.Reset();
    m_linearSolver.SetPreferLdlt(false);

    const int estimatedSteps = static_cast<int>(std::ceil(duration / m_param.dt));
    double time = 0.0;
    double nextTimeStep = m_param.dt;
    if (m_param.timeStepMode == TimeStepMode::Adaptive)
    {
        nextTimeStep = std::clamp(nextTimeStep, m_param.adaptiveTimeStep.minimumTimeStep,
                                  m_param.adaptiveTimeStep.maximumTimeStep);
    }
    int rejectedAttempts = 0;
    int acceptedSteps = 0;

    while (time < duration - 1.0e-12)
    {
        if (model.IsCancellationRequested())
            return false;

        const double timeStep = std::min(nextTimeStep, duration - time);
        if (timeStep <= 0.0)
            break;

        if (m_param.timeStepMode == TimeStepMode::Fixed)
        {
            model.BackupStepState();
            StepResult result;
            if (!SolveSingleStep(model, time, timeStep, dampingFactor, result))
            {
                model.RollbackDynamicStep();
                const bool minimumReached = timeStep <= m_param.minimumTimeStep * (1.0 + 1.0e-12);
                if (minimumReached || rejectedAttempts >= m_param.maximumCutbacks)
                {
                    QTextStream(stderr) << "Newmark failed at time=" << time << " dt=" << timeStep
                                        << " residual=" << result.residual << " limit=" << result.residualLimit
                                        << Qt::endl;
                    return false;
                }
                nextTimeStep = std::max(m_param.minimumTimeStep, timeStep * m_param.cutbackFactor);
                ++rejectedAttempts;
                continue;
            }

            model.SetNonlinearMPCMultipliers(result.nonlinearMpcMultipliers);
            model.CommitState();
            ++acceptedSteps;
            time += timeStep;
            nextTimeStep = std::min(m_param.dt, timeStep * m_param.recoveryFactor);
            rejectedAttempts = 0;
            model.GetState(m_Un, m_Vn, m_An);
            model.RecordStepIterations(time, result.iterations);
            if (m_callback)
                m_callback(acceptedSteps, time, m_Un);
            model.OnStepCompleted(time);
            model.ReportProgress(std::min(1.0, time / duration),
                                 QStringLiteral("Newmark 第 %1/%2 步").arg(acceptedSteps).arg(estimatedSteps));
            continue;
        }

        if (!model.PushStateCheckpoint())
            return false;

        StepResult coarse;
        bool converged = SolveSingleStep(model, time, timeStep, dampingFactor, coarse);
        Vec coarseDisplacement;
        Vec coarseVelocity;
        Vec ignoredAcceleration;
        if (converged)
            model.GetState(coarseDisplacement, coarseVelocity, ignoredAcceleration);
        if (!converged || !model.RestoreStateCheckpoint())
        {
            model.RestoreStateCheckpoint();
            model.DiscardStateCheckpoint();
            const TimeStepDecision decision = adaptiveController->DecideRejected(timeStep, 1.0, 2);
            if (decision.nextTimeStep >= timeStep ||
                ++rejectedAttempts > m_param.adaptiveTimeStep.maximumRejectedAttempts)
                return false;
            nextTimeStep = decision.nextTimeStep;
            continue;
        }

        StepResult firstHalf;
        converged = SolveSingleStep(model, time, 0.5 * timeStep, dampingFactor, firstHalf);
        if (converged)
        {
            // 半步为后续半步提供材料历史状态；外层检查点可撤销它。
            model.SetNonlinearMPCMultipliers(firstHalf.nonlinearMpcMultipliers);
            model.CommitState();
        }

        StepResult fine;
        if (converged)
            converged = SolveSingleStep(model, time + 0.5 * timeStep, 0.5 * timeStep, dampingFactor, fine);

        Vec fineDisplacement;
        Vec fineVelocity;
        if (converged)
            model.GetState(fineDisplacement, fineVelocity, ignoredAcceleration);

        const double normalizedError =
            converged ? EstimateStepDoublingError(coarseDisplacement, coarseVelocity, fineDisplacement, fineVelocity,
                                                   m_param.adaptiveTimeStep, 2)
                      : std::numeric_limits<double>::infinity();
        const TimeStepDecision decision = converged
                                              ? adaptiveController->DecideAccepted(timeStep, normalizedError, 2)
                                              : adaptiveController->DecideRejected(timeStep, normalizedError, 2);
        if (!decision.accepted)
        {
            model.RestoreStateCheckpoint();
            model.DiscardStateCheckpoint();
            if (decision.nextTimeStep >= timeStep ||
                ++rejectedAttempts > m_param.adaptiveTimeStep.maximumRejectedAttempts)
            {
                QTextStream(stderr) << "Newmark adaptive step rejected too many times: t=" << time
                                    << " dt=" << timeStep << " normalized_error=" << normalizedError
                                    << " rejected_attempts=" << rejectedAttempts << Qt::endl;
                qDebug().noquote() << QStringLiteral("Newmark 自适应时间步在 t=%1 失败，dt=%2，误差=%3")
                                          .arg(time, 0, 'g', 10)
                                          .arg(timeStep, 0, 'g', 10)
                                          .arg(normalizedError, 0, 'g', 8);
                return false;
            }
            nextTimeStep = decision.nextTimeStep;
            continue;
        }

        model.SetNonlinearMPCMultipliers(fine.nonlinearMpcMultipliers);
        model.CommitState();
        model.DiscardStateCheckpoint();
        ++acceptedSteps;
        time += timeStep;
        nextTimeStep = decision.nextTimeStep;
        rejectedAttempts = 0;
        model.GetState(m_Un, m_Vn, m_An);
        model.RecordStepIterations(time, firstHalf.iterations + fine.iterations);
        if (m_callback)
            m_callback(acceptedSteps, time, m_Un);
        model.OnStepCompleted(time);
        model.ReportProgress(std::min(1.0, time / duration),
                             QStringLiteral("自适应 Newmark 第 %1 步，dt=%2，误差=%3")
                                 .arg(acceptedSteps)
                                 .arg(timeStep, 0, 'g', 6)
                                 .arg(normalizedError, 0, 'g', 4));
    }

    qDebug().noquote() << QStringLiteral("Newmark 动力求解完成");
    return true;
}

bool SolverNewmark::SolveSingleStep(IAnalysisModel& model, double currentTime, double timeStep,
                                    const Eigen::MatrixXd& dampingFactor, StepResult& result)
{
    ComputeCoeffs(timeStep);
    const int dofCount = model.GetFreeDofs();
    model.BeginDynamicStep(timeStep, m_param.beta, m_param.gamma);

    bool externalTangentAssembled = false;
    if (m_param.aerodynamicTangentMode == AerodynamicTangentMode::Disabled)
    {
        m_externalTangent.resize(dofCount, dofCount);
        m_externalTangent.setZero();
        externalTangentAssembled = true;
    }

    for (int iteration = 1; iteration <= m_param.maxIter; ++iteration)
    {
        result.iterations = iteration;
        Vec constrainedForce;
        Vec externalForce;
        model.ComputeExternalForce(currentTime + timeStep, 1.0, constrainedForce, externalForce);
        model.AssembleEffectiveTangent(m_c.a0, m_c.a1, m_Keff);
        if (!externalTangentAssembled ||
            m_param.aerodynamicTangentMode == AerodynamicTangentMode::EveryNewtonIteration)
        {
            model.AssembleExternalLoadTangent(currentTime + timeStep, 1.0, m_c.a1, m_externalTangent);
            externalTangentAssembled = true;
        }
        m_Keff -= m_externalTangent;
        model.ComputeResidual(externalForce, m_R);

        NonlinearMPCData constraints;
        if (!model.AssembleNonlinearMPC(constraints))
            return false;

        SpMat solveTangent;
        Vec solveRhs;
        Eigen::MatrixXd solveDampingFactor;
        NonlinearMPCReduction reduction;
        double constraintError = 0.0;
        if (constraints.Empty())
        {
            solveTangent = m_Keff;
            solveRhs = m_R;
            solveDampingFactor = dampingFactor;
            result.nonlinearMpcMultipliers.resize(0);
        }
        else
        {
            if (!NonlinearMPC::Reduce(m_Keff, m_R, constraints, reduction))
                return false;
            solveTangent = reduction.tangent;
            solveRhs = reduction.rhs;
            solveDampingFactor = reduction.masterTransformation.transpose() * dampingFactor;
            if (dampingFactor.cols() > 0)
                solveRhs -= m_c.a1 * solveDampingFactor *
                            (dampingFactor.transpose() * reduction.particularCorrection);
            constraintError = reduction.constraintNorm;
            result.nonlinearMpcMultipliers = reduction.multipliers;
        }

        result.residual = solveRhs.norm();
        result.residualLimit = m_param.tol * std::max(1.0, externalForce.norm());
        if (!solveRhs.allFinite() || !std::isfinite(constraintError))
            return false;
        if (result.residual <= result.residualLimit && constraintError <= m_param.constraintTolerance)
        {
            result.converged = true;
            return true;
        }

        Vec independentCorrection;
        if (!SolveLinear(solveTangent, solveDampingFactor, m_c.a1, solveRhs, independentCorrection))
            return false;
        m_dx = constraints.Empty() ? independentCorrection : reduction.RecoverFullIncrement(independentCorrection);
        if (m_dx.size() != dofCount || !m_dx.allFinite())
            return false;

        const bool canRestoreTrial = model.SaveTrialState();
        if (!canRestoreTrial)
        {
            model.ApplyDynamicCorrection(m_dx, m_c.a0, m_c.a1);
            continue;
        }

        bool correctionAccepted = false;
        double correctionScale = 1.0;
        constexpr int maximumLineSearchTrials = 8;
        constexpr double armijoSlope = 1.0e-4;
        for (int trial = 0; trial < maximumLineSearchTrials; ++trial)
        {
            if (!model.RestoreTrialState())
                return false;
            model.ApplyDynamicCorrection(correctionScale * m_dx, m_c.a0, m_c.a1);

            Vec trialConstrainedForce;
            Vec trialExternalForce;
            Vec trialResidual;
            SpMat trialEffectiveTangent;
            model.ComputeExternalForce(currentTime + timeStep, 1.0, trialConstrainedForce, trialExternalForce);
            model.AssembleEffectiveTangent(m_c.a0, m_c.a1, trialEffectiveTangent);
            if (m_param.aerodynamicTangentMode == AerodynamicTangentMode::EveryNewtonIteration)
            {
                SpMat trialExternalTangent;
                model.AssembleExternalLoadTangent(currentTime + timeStep, 1.0, m_c.a1, trialExternalTangent);
                trialEffectiveTangent -= trialExternalTangent;
            }
            else
            {
                trialEffectiveTangent -= m_externalTangent;
            }
            model.ComputeResidual(trialExternalForce, trialResidual);

            NonlinearMPCData trialConstraints;
            if (!model.AssembleNonlinearMPC(trialConstraints))
            {
                correctionScale *= 0.5;
                continue;
            }
            double trialResidualNorm = trialResidual.norm();
            double trialConstraintError = 0.0;
            if (!trialConstraints.Empty())
            {
                NonlinearMPCReduction trialReduction;
                if (!NonlinearMPC::Reduce(trialEffectiveTangent, trialResidual, trialConstraints, trialReduction))
                {
                    correctionScale *= 0.5;
                    continue;
                }
                trialResidualNorm = trialReduction.rhs.norm();
                if (dampingFactor.cols() > 0)
                {
                    const Eigen::MatrixXd trialDamping =
                        trialReduction.masterTransformation.transpose() * dampingFactor;
                    trialResidualNorm =
                        (trialReduction.rhs - m_c.a1 * trialDamping *
                                                  (dampingFactor.transpose() * trialReduction.particularCorrection))
                            .norm();
                }
                trialConstraintError = trialReduction.constraintNorm;
            }
            const double allowedConstraint = 1.1 * std::max(constraintError, m_param.constraintTolerance);
            if (std::isfinite(trialResidualNorm) && std::isfinite(trialConstraintError) &&
                trialResidualNorm <= (1.0 - armijoSlope * correctionScale) * result.residual &&
                trialConstraintError <= allowedConstraint)
            {
                correctionAccepted = true;
                break;
            }
            correctionScale *= 0.5;
        }
        if (!correctionAccepted)
        {
            model.RestoreTrialState();
            return false;
        }
    }
    return false;
}

bool SolverNewmark::SolveLinear(const SpMat& stiffness, const Eigen::MatrixXd& dampingFactor, double dampingScale,
                                const Vec& rhs, Vec& solution)
{
    return m_linearSolver.SolveLowRank(stiffness, dampingFactor, dampingScale, rhs, solution);
}
}
