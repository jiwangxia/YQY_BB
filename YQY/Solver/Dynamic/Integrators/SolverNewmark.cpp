/**
 * @file SolverNewmark.cpp
 * @brief Newmark-beta 非线性动力求解器。
 */
#include "SolverNewmark.h"
#include "../TimeStepping/StateCheckpointGuard.h"
#include "../TimeStepping/TimeStepError.h"
#include "../TimeStepping/NewtonPrecisionScale.h"
#include "Solver/Constraint/NonlinearMPC.h"

#include <QDebug>
#include <QTextStream>
#include <Eigen/LU>
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

bool SolverNewmark::PrepareMpcTrialState(IAnalysisModel& model, const std::vector<int>& stepSlaveDofs,
                                       _OUT NonlinearMPCData& constraints) const
{
    constexpr int maximumProjectionIterations = 10;
    constexpr double correctionTolerance = 8.0 * std::numeric_limits<double>::epsilon();
    const int dofCount = model.GetFreeDofs();
    double previousConstraintError = std::numeric_limits<double>::infinity();
    for (int iteration = 0; iteration <= maximumProjectionIterations; ++iteration)
    {
        if (!model.AssembleNonlinearMPC(constraints) || constraints.slaveDofs.size() != stepSlaveDofs.size())
            return false;
        if (constraints.Empty())
            return true;

        // 每次回缩只改变同一组从位置，保留主变量的 Newmark 关系；步内不能重新选轴。
        constraints.slaveDofs = stepSlaveDofs;
        if (!constraints.IsValid(dofCount))
            return false;
        const int constraintCount = static_cast<int>(constraints.value.size());
        Eigen::MatrixXd slaveJacobian(constraintCount, constraintCount);
        for (int column = 0; column < constraintCount; ++column)
            slaveJacobian.col(column) = constraints.jacobian.col(stepSlaveDofs[column]);
        Eigen::FullPivLU<Eigen::MatrixXd> factor(slaveJacobian);
        factor.setThreshold(1.0e-12);
        if (factor.rank() != constraintCount)
            return false;
        const Vec slaveCorrection = factor.solve(-constraints.value);
        if (!slaveCorrection.allFinite())
            return false;

        Vec displacement;
        Vec velocity;
        Vec acceleration;
        model.GetState(displacement, velocity, acceleration);
        if (displacement.size() != dofCount || !displacement.allFinite())
            return false;
        double scaledCorrection = 0.0;
        for (int column = 0; column < constraintCount; ++column)
            scaledCorrection =
                std::max(scaledCorrection, std::abs(slaveCorrection[column]) /
                                              std::max(1.0, std::abs(displacement[stepSlaveDofs[column]])));
        const double constraintError = constraints.value.norm();
        // 大参考坐标下 X+u 的舍入可能使几何误差停滞，不能额外强求 u 的机器级修正。
        // 只有真实位置误差已通过原门时才允许停止；动力平衡仍由外层原残量门检查。
        const bool projectionStalled = iteration > 0 && constraintError >= previousConstraintError;
        if (constraintError <= m_param.constraintTolerance &&
            (scaledCorrection <= correctionTolerance || projectionStalled))
            return ProjectMpcKinematics(model, constraints);
        if (iteration == maximumProjectionIterations)
            return false;
        previousConstraintError = constraintError;

        Vec correction = Vec::Zero(dofCount);
        for (int column = 0; column < constraintCount; ++column)
            correction[stepSlaveDofs[column]] = slaveCorrection[column];
        model.ApplyDynamicCorrection(correction, m_c.a0, m_c.a1);
    }
    return false;
}

// 位移修正后的从自由度速度、加速度仅满足 Newmark 离散公式；曲面约束要求重新满足两层导数约束。
// 主自由度保留 Newmark 运动学，投影后的完整状态用于惯性力和约束残差计算。
bool SolverNewmark::ProjectMpcKinematics(IAnalysisModel& model, const NonlinearMPCData& constraints) const
{
    if (constraints.Empty())
        return true;

    const int dofCount = model.GetFreeDofs();
    const int constraintCount = static_cast<int>(constraints.value.size());
    if (!constraints.IsValid(dofCount))
        return false;

    Vec displacement;
    Vec velocity;
    Vec acceleration;
    model.GetState(displacement, velocity, acceleration);
    if (displacement.size() != dofCount || velocity.size() != dofCount || acceleration.size() != dofCount ||
        !displacement.allFinite() || !velocity.allFinite() || !acceleration.allFinite())
        return false;

    Eigen::MatrixXd slaveJacobian(constraintCount, constraintCount);
    for (int column = 0; column < constraintCount; ++column)
        slaveJacobian.col(column) = constraints.jacobian.col(constraints.slaveDofs[column]);

    Eigen::FullPivLU<Eigen::MatrixXd> factor(slaveJacobian);
    factor.setThreshold(1.0e-12);
    if (factor.rank() != constraintCount)
        return false;

    const Vec velocityCorrection = factor.solve(-constraints.jacobian * velocity);
    if (!velocityCorrection.allFinite())
        return false;
    for (int column = 0; column < constraintCount; ++column)
        velocity[constraints.slaveDofs[column]] += velocityCorrection[column];

    Vec curvature = Vec::Zero(constraintCount);
    const bool hasSparseHessians = static_cast<int>(constraints.hessians.size()) == constraintCount;
    const bool hasLocalHessianEntries = static_cast<int>(constraints.hessianEntries.size()) == constraintCount;
    for (int row = 0; row < constraintCount; ++row)
    {
        if (hasSparseHessians && constraints.hessians[row].rows() == dofCount)
            curvature[row] += velocity.dot(constraints.hessians[row] * velocity);
        if (hasLocalHessianEntries)
            for (const auto& entry : constraints.hessianEntries[row])
                curvature[row] += entry.value() * velocity[entry.row()] * velocity[entry.col()];
    }
    if (!curvature.allFinite())
        return false;

    const Vec accelerationCorrection = factor.solve(-constraints.jacobian * acceleration - curvature);
    if (!accelerationCorrection.allFinite())
        return false;
    for (int column = 0; column < constraintCount; ++column)
        acceleration[constraints.slaveDofs[column]] += accelerationCorrection[column];

    const double jacobianScale = std::max(1.0, constraints.jacobian.norm());
    const double velocityLimit = m_param.constraintTolerance * jacobianScale * std::max(1.0, velocity.norm());
    const double accelerationLimit =
        m_param.constraintTolerance * std::max({1.0, jacobianScale * acceleration.norm(), curvature.norm()});
    if (!velocity.allFinite() || !acceleration.allFinite() ||
        (constraints.jacobian * velocity).norm() > velocityLimit ||
        (constraints.jacobian * acceleration + curvature).norm() > accelerationLimit)
        return false;

    model.SetTrialKinematics(velocity, acceleration);
    return true;
}

bool SolverNewmark::Solve(IAnalysisModel& model, double duration)
{
    const bool traceInitialError = qEnvironmentVariableIntValue("YQY_TRACE_NEWMARK_INITIAL_ERROR") != 0;
    m_timeStepStatistics = {};
    m_precisionLimitedSolveCount = 0;
    m_hasActiveFiniteRotations = false;
    std::vector<FiniteRotationState> rotationStates;
    model.GetFiniteRotationStates(rotationStates);
    for (const auto& state : rotationStates)
        for (int dof : state.freeDofs)
            m_hasActiveFiniteRotations = m_hasActiveFiniteRotations || dof >= 0;
    qDebug().noquote() << QStringLiteral("开始 Newmark 动力非线性求解...");
    const int dofCount = model.GetFreeDofs();
    if (dofCount <= 0 || !std::isfinite(duration) || duration <= 0.0 || !std::isfinite(m_param.dt) ||
        m_param.dt <= 0.0 || !std::isfinite(m_param.beta) || m_param.beta <= 0.0 || !std::isfinite(m_param.gamma) ||
        m_param.gamma < 0.0 || m_param.maxIter < 1 || !std::isfinite(m_param.tol) || m_param.tol <= 0.0 ||
        !std::isfinite(m_param.constraintTolerance) || m_param.constraintTolerance <= 0.0)
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

    NonlinearMPCData initialConstraints;
    if (!model.AssembleNonlinearMPC(initialConstraints))
    {
        const QString message = QStringLiteral("Newmark 初始 MPC 约束组装失败");
        model.ReportProgress(0.0, message);
        qDebug().noquote() << message;
        return false;
    }
    if (!initialConstraints.Empty())
    {
        Vec initialDisplacement;
        Vec initialVelocity;
        Vec initialAcceleration;
        model.GetState(initialDisplacement, initialVelocity, initialAcceleration);
        QString errorMessage;
        if (!NonlinearMPC::ValidateInitialKinematics(initialConstraints, initialDisplacement, initialVelocity,
                                                     initialAcceleration, errorMessage, m_param.constraintTolerance))
        {
            const QString message = QStringLiteral("Newmark %1").arg(errorMessage);
            model.ReportProgress(0.0, message);
            qDebug().noquote() << message;
            return false;
        }
    }

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
    }

    m_dx.resize(dofCount);
    m_R.resize(dofCount);
    m_Un.resize(dofCount);
    m_Vn.resize(dofCount);
    m_An.resize(dofCount);
    m_linearSolver.Reset();
    m_linearSolver.SetPreferLdlt(false);
    m_linearSolver.SetAllowGpuAutomatic(model.ShouldPreferGpuLinearSolver());

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
    const int methodOrder = m_param.gamma == 0.5 ? 2 : 1;

    while (time < duration - 1.0e-12)
    {
        if (model.IsCancellationRequested())
            return false;

        const double timeStep = SelectTimeStep(time, duration, nextTimeStep);
        if (timeStep <= 0.0)
            break;

        if (m_param.timeStepMode == TimeStepMode::Fixed)
        {
            model.BackupStepState();
            StepResult result;
            if (!SolveSingleStep(model, time, timeStep, result))
            {
                model.RollbackDynamicStep();
                const bool minimumReached = timeStep <= m_param.minimumTimeStep * (1.0 + 1.0e-12);
                if (minimumReached || rejectedAttempts >= m_param.maximumCutbacks)
                {
                    QTextStream(stderr) << "Newmark failed at time=" << time << " dt=" << timeStep
                                        << " residual=" << result.residual << " limit=" << result.residualLimit
                                        << Qt::endl;
                    const QString message =
                        QStringLiteral("Newmark 固定步求解在 t=%1 失败，dt=%2，残差=%3，残差限=%4")
                            .arg(time, 0, 'g', 17)
                            .arg(timeStep, 0, 'g', 17)
                            .arg(result.residual, 0, 'g', 17)
                            .arg(result.residualLimit, 0, 'g', 17);
                    model.ReportProgress(std::min(1.0, time / duration), message);
                    return false;
                }
                nextTimeStep = std::max(m_param.minimumTimeStep, timeStep * m_param.cutbackFactor);
                ++rejectedAttempts;
                m_timeStepStatistics.RecordRejected();
                continue;
            }

            model.SetNonlinearMPCMultipliers(result.nonlinearMpcMultipliers);
            model.CommitState();
            ++acceptedSteps;
            m_timeStepStatistics.RecordAccepted(timeStep);
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

        StateCheckpointGuard checkpoint(model);
        if (!checkpoint.IsActive())
            return false;

        StepResult coarse;
        bool converged = SolveSingleStep(model, time, timeStep, coarse);
        Vec coarseDisplacement;
        Vec coarseVelocity;
        Vec ignoredAcceleration;
        if (converged)
            model.GetState(coarseDisplacement, coarseVelocity, ignoredAcceleration);
        if (!converged)
        {
            if (!checkpoint.Restore())
                return false;
            const TimeStepDecision decision = adaptiveController->DecideSolveFailure(timeStep);
            if (decision.nextTimeStep >= timeStep ||
                ++rejectedAttempts > m_param.adaptiveTimeStep.maximumRejectedAttempts)
            {
                const QString message =
                    QStringLiteral("Newmark 自适应整步求解在 t=%1 失败，dt=%2，残差=%3，残差限=%4")
                        .arg(time, 0, 'g', 17)
                        .arg(timeStep, 0, 'g', 17)
                        .arg(coarse.residual, 0, 'g', 17)
                        .arg(coarse.residualLimit, 0, 'g', 17);
                model.ReportProgress(std::min(1.0, time / duration), message);
                qDebug().noquote() << message;
                return false;
            }
            m_timeStepStatistics.RecordRejected();
            nextTimeStep = decision.nextTimeStep;
            continue;
        }
        if (!checkpoint.Restore())
            return false;

        StepResult firstHalf;
        converged = SolveSingleStep(model, time, 0.5 * timeStep, firstHalf);
        if (converged)
        {
            // 半步为后续半步提供材料历史状态；外层检查点可撤销它。
            model.SetNonlinearMPCMultipliers(firstHalf.nonlinearMpcMultipliers);
            model.CommitState();
        }

        StepResult fine;
        if (converged)
            converged = SolveSingleStep(model, time + 0.5 * timeStep, 0.5 * timeStep, fine);

        Vec fineDisplacement;
        Vec fineVelocity;
        if (converged)
            model.GetState(fineDisplacement, fineVelocity, ignoredAcceleration);

        const double normalizedError =
            converged ? EstimateStepDoublingError(coarseDisplacement, coarseVelocity, fineDisplacement, fineVelocity,
                                                  m_param.adaptiveTimeStep, methodOrder)
                      : std::numeric_limits<double>::infinity();
        // 可选记录启动拒步的主导分量，区分步长下限、状态误差和非线性不收敛；不改变误差判据。
        if (traceInitialError && time == 0.0 && converged)
        {
            const auto& settings = m_param.adaptiveTimeStep;
            const double denominator = std::pow(2.0, methodOrder) - 1.0;
            int displacementDof = -1;
            int velocityDof = -1;
            double displacementError = -1.0;
            double velocityError = -1.0;
            for (Eigen::Index index = 0; index < coarseDisplacement.size(); ++index)
            {
                const double displacementScale = settings.absoluteTolerance + settings.relativeTolerance *
                    std::max(std::abs(coarseDisplacement[index]), std::abs(fineDisplacement[index]));
                const double velocityScale = settings.velocityAbsoluteTolerance + settings.velocityRelativeTolerance *
                    std::max(std::abs(coarseVelocity[index]), std::abs(fineVelocity[index]));
                const double qError = std::abs(fineDisplacement[index] - coarseDisplacement[index]) /
                                      (denominator * displacementScale);
                const double vError = std::abs(fineVelocity[index] - coarseVelocity[index]) /
                                      (denominator * velocityScale);
                if (qError > displacementError)
                {
                    displacementError = qError;
                    displacementDof = static_cast<int>(index);
                }
                if (vError > velocityError)
                {
                    velocityError = vError;
                    velocityDof = static_cast<int>(index);
                }
            }
            QTextStream trace(stdout);
            trace.setRealNumberPrecision(17);
            trace << "newmark_initial_error dt=" << timeStep << " error=" << normalizedError
                  << " q_error=" << displacementError << " q_dof=" << displacementDof
                  << " v_error=" << velocityError << " v_dof=" << velocityDof;
            if (displacementDof >= 0 && velocityDof >= 0)
                trace << " q_coarse=" << coarseDisplacement[displacementDof]
                      << " q_fine=" << fineDisplacement[displacementDof]
                      << " v_coarse=" << coarseVelocity[velocityDof]
                      << " v_fine=" << fineVelocity[velocityDof];
            trace << Qt::endl;
        }
        const TimeStepDecision decision = converged ? adaptiveController->Decide(timeStep, normalizedError, methodOrder)
                                                    : adaptiveController->DecideSolveFailure(timeStep);
        if (!decision.accepted)
        {
            if (!checkpoint.Restore())
                return false;
            if (decision.nextTimeStep >= timeStep ||
                ++rejectedAttempts > m_param.adaptiveTimeStep.maximumRejectedAttempts)
            {
                QTextStream(stderr) << "Newmark adaptive step rejected too many times: t=" << time << " dt=" << timeStep
                                    << " normalized_error=" << normalizedError
                                    << " rejected_attempts=" << rejectedAttempts << Qt::endl;
                QString message;
                if (converged)
                {
                    message = QStringLiteral("Newmark 自适应时间步在 t=%1 失败，dt=%2，归一化误差=%3")
                                  .arg(time, 0, 'g', 17)
                                  .arg(timeStep, 0, 'g', 17)
                                  .arg(normalizedError, 0, 'g', 17);
                }
                else
                {
                    const StepResult& failedStep = firstHalf.converged ? fine : firstHalf;
                    message = QStringLiteral("Newmark 自适应半步求解在 t=%1 失败，宏步dt=%2，残差=%3，残差限=%4")
                                  .arg(time, 0, 'g', 17)
                                  .arg(timeStep, 0, 'g', 17)
                                  .arg(failedStep.residual, 0, 'g', 17)
                                  .arg(failedStep.residualLimit, 0, 'g', 17);
                }
                model.ReportProgress(std::min(1.0, time / duration), message);
                qDebug().noquote() << message;
                return false;
            }
            m_timeStepStatistics.RecordRejected();
            nextTimeStep = decision.nextTimeStep;
            continue;
        }

        model.SetNonlinearMPCMultipliers(fine.nonlinearMpcMultipliers);
        model.CommitState();
        checkpoint.Release();
        ++acceptedSteps;
        m_timeStepStatistics.RecordAccepted(timeStep);
        time += timeStep;
        nextTimeStep = decision.nextTimeStep;
        rejectedAttempts = 0;
        model.GetState(m_Un, m_Vn, m_An);
        model.RecordStepIterations(time, firstHalf.iterations + fine.iterations);
        if (m_callback)
            m_callback(acceptedSteps, time, m_Un);
        model.OnStepCompleted(time);
        model.ReportProgress(std::min(1.0, time / duration), QStringLiteral("自适应 Newmark 第 %1 步，dt=%2，误差=%3")
                                                                 .arg(acceptedSteps)
                                                                 .arg(timeStep, 0, 'g', 6)
                                                                 .arg(normalizedError, 0, 'g', 4));
    }

    qDebug().noquote() << QStringLiteral("Newmark 动力求解完成");
    return true;
}

bool SolverNewmark::SolveSingleStep(IAnalysisModel& model, double currentTime, double timeStep,
                                    StepResult& result)
{
    ComputeCoeffs(timeStep);
    const int dofCount = model.GetFreeDofs();
    Vec initialDisplacement;
    Vec previousVelocity;
    Vec initialAcceleration;
    model.GetState(initialDisplacement, previousVelocity, initialAcceleration);
    NonlinearMPCData initialConstraints;
    if (!model.AssembleNonlinearMPC(initialConstraints))
        return false;
    const std::vector<int> stepSlaveDofs = initialConstraints.slaveDofs;
    model.BeginDynamicStep(timeStep, m_param.beta, m_param.gamma);
    const bool trialRequiresEffectiveTangent = model.HasActiveNonlinearMPC();

    bool externalTangentAssembled = false;
    bool previousFullCorrectionApplied = false;
    Vec previousIterationResidual;
    double previousCorrectionNorm = std::numeric_limits<double>::infinity();
    bool precisionCorrectionAttempted = false;
    double precisionCorrectionResidual = std::numeric_limits<double>::infinity();
    constexpr double armijoSlope = 1.0e-4;
    if (m_param.aerodynamicTangentMode == AerodynamicTangentMode::Disabled)
    {
        m_externalTangent.resize(dofCount, dofCount);
        m_externalTangent.setZero();
        externalTangentAssembled = true;
    }

    for (int iteration = 1; iteration <= m_param.maxIter; ++iteration)
    {
        result.iterations = iteration;
        NonlinearMPCData constraints;
        if (!PrepareMpcTrialState(model, stepSlaveDofs, constraints))
            return false;

        const bool requiresExternalTangent =
            !externalTangentAssembled || m_param.aerodynamicTangentMode == AerodynamicTangentMode::EveryNewtonIteration;
        SystemEvaluationResult evaluation;
        model.EvaluateSystem(
            SystemEvaluationRequest::EffectiveTangent(currentTime + timeStep, m_c.a0, m_c.a1, requiresExternalTangent),
            evaluation);
        const double* gpuCscValues = evaluation.gpuCscValues;
        const int gpuCscValueCount = evaluation.gpuCscValueCount;
        Vec externalForce = std::move(evaluation.externalForce);
        m_Keff = std::move(evaluation.effectiveTangent);
        if (requiresExternalTangent)
        {
            m_externalTangent = std::move(evaluation.externalTangent);
            externalTangentAssembled = true;
        }
        m_Keff -= m_externalTangent;
        model.ComputeResidual(externalForce, m_R);

        SpMat solveTangent;
        Vec solveRhs;
        NonlinearMPCReduction reduction;
        double constraintError = 0.0;
        if (constraints.Empty())
        {
            solveTangent = m_Keff;
            solveRhs = m_R;
            result.nonlinearMpcMultipliers.resize(0);
        }
        else
        {
            if (!NonlinearMPC::Reduce(m_Keff, m_R, constraints, reduction))
                return false;
            // 当前位置已回缩。方向只含切向增量，残量取实际动力平衡，避免把已被
            // 从加速度投影消除的法向特解惯性 K*p 再带入方程和线搜索指标。
            solveTangent = reduction.tangent;
            solveRhs = reduction.masterTransformation.transpose() * m_R;
            constraintError = reduction.constraintNorm;
            result.nonlinearMpcMultipliers = reduction.multipliers;
        }

        result.residual = solveRhs.norm();
        result.residualLimit = m_param.tol * std::max(1.0, externalForce.norm());
        if (!solveRhs.allFinite() || !std::isfinite(constraintError))
            return false;
        const double residualChange =
            previousFullCorrectionApplied && previousIterationResidual.size() == solveRhs.size()
                ? (solveRhs - previousIterationResidual).norm()
                : std::numeric_limits<double>::infinity();
        // 修正量使用完整 Newton 方向，不能以线搜索缩短后的步长冒充收敛。
        if ((result.residual == 0.0 || residualChange <= result.residualLimit ||
             previousCorrectionNorm <= m_param.tol) &&
            constraintError <= m_param.constraintTolerance)
        {
            result.converged = true;
            return true;
        }

        previousIterationResidual = solveRhs;
        Vec independentCorrection;
        const bool canUseDeviceCsc = constraints.Empty() &&
                                     m_externalTangent.nonZeros() == 0 && gpuCscValues &&
                                     gpuCscValueCount == solveTangent.nonZeros();
        const std::uint64_t patternVersion =
            constraints.Empty() && m_externalTangent.nonZeros() == 0 ? evaluation.sparsePatternVersion : 0;
        const bool solved = canUseDeviceCsc
                                ? m_linearSolver.SolveDeviceCscValues(solveTangent, gpuCscValues, gpuCscValueCount,
                                                                      solveRhs, independentCorrection, patternVersion)
                                : SolveLinear(solveTangent, solveRhs, independentCorrection,
                                              patternVersion);
        if (!solved)
            return false;
        if (constraints.Empty())
            m_dx = independentCorrection;
        else
            m_dx = reduction.masterTransformation * independentCorrection;
        if (m_dx.size() != dofCount || !m_dx.allFinite())
            return false;

        Vec stepIncrement;
        model.GetStepIncrement(stepIncrement);
        if (stepIncrement.size() != dofCount || !stepIncrement.allFinite())
            return false;
        previousCorrectionNorm = m_dx.norm() / std::max(1.0e-12, stepIncrement.norm());

        // 必须使用本轮重新装配的切线和完整 Newton 修正，不能把线搜索缩小量当作收敛证据。
        double scaledResidual = std::numeric_limits<double>::infinity();
        double scaledCorrection = std::numeric_limits<double>::infinity();
        const bool precisionCandidate =
            (m_hasActiveFiniteRotations || !constraints.Empty()) && iteration > 1 &&
            m_param.aerodynamicTangentMode != AerodynamicTangentMode::OncePerTimeStepOrStage &&
            constraintError <= m_param.constraintTolerance &&
            IsPrecisionLimitedConvergence(model, solveRhs, externalForce, solveTangent,
                                          constraints.Empty() ? nullptr : &reduction,
                                          constraints.Empty() ? nullptr : &constraints, previousVelocity,
                                          scaledResidual, scaledCorrection);

        const bool canRestoreTrial = model.SaveTrialState();
        if (!canRestoreTrial)
        {
            // 步内小增量仍可改变速度和加速度；先实际修正，再以重新装配的残差确认是否停滞。
            const bool precisionStalled = precisionCandidate && precisionCorrectionAttempted &&
                                          result.residual >= (1.0 - armijoSlope) * precisionCorrectionResidual;
            if (!precisionStalled)
            {
                precisionCorrectionAttempted = precisionCandidate;
                precisionCorrectionResidual = result.residual;
                model.ApplyDynamicCorrection(m_dx, m_c.a0, m_c.a1);
                previousFullCorrectionApplied = true;
                continue;
            }
        }
        else
        {
            bool correctionAccepted = false;
            double correctionScale = 1.0;
            constexpr int maximumLineSearchTrials = 8;
            for (int trial = 0; trial < maximumLineSearchTrials; ++trial)
            {
                if (!model.RestoreTrialState())
                    return false;
                model.ApplyDynamicCorrection(correctionScale * m_dx, m_c.a0, m_c.a1);

                NonlinearMPCData trialConstraints;
                if (!PrepareMpcTrialState(model, stepSlaveDofs, trialConstraints))
                {
                    correctionScale *= 0.5;
                    continue;
                }

                Vec trialExternalForce;
                Vec trialResidual;
                SpMat trialEffectiveTangent;
                SystemEvaluationResult trialEvaluation;
                const SystemEvaluationRequest trialRequest =
                    trialRequiresEffectiveTangent
                        ? SystemEvaluationRequest::EffectiveTangent(currentTime + timeStep, m_c.a0, m_c.a1, false)
                        : SystemEvaluationRequest::TrialResidual(currentTime + timeStep);
                model.EvaluateSystem(trialRequest, trialEvaluation);
                trialExternalForce = std::move(trialEvaluation.externalForce);
                trialEffectiveTangent = std::move(trialEvaluation.effectiveTangent);
                model.ComputeResidual(trialExternalForce, trialResidual);

                double trialResidualNorm = trialResidual.norm();
                double trialConstraintError = 0.0;
                if (!trialConstraints.Empty())
                {
                    if (!trialRequiresEffectiveTangent)
                    {
                        SystemEvaluationResult tangentEvaluation;
                        model.EvaluateSystem(SystemEvaluationRequest::EffectiveTangentOnly(m_c.a0, m_c.a1),
                                             tangentEvaluation);
                        trialEffectiveTangent = std::move(tangentEvaluation.effectiveTangent);
                    }
                    if (m_param.aerodynamicTangentMode == AerodynamicTangentMode::EveryNewtonIteration)
                    {
                        SystemEvaluationResult tangentEvaluation;
                        model.EvaluateSystem(SystemEvaluationRequest::ExternalTangent(currentTime + timeStep, m_c.a1),
                                             tangentEvaluation);
                        trialEffectiveTangent -= tangentEvaluation.externalTangent;
                    }
                    else
                    {
                        trialEffectiveTangent -= m_externalTangent;
                    }
                    NonlinearMPCReduction trialReduction;
                    if (!NonlinearMPC::Reduce(trialEffectiveTangent, trialResidual, trialConstraints, trialReduction))
                    {
                        correctionScale *= 0.5;
                        continue;
                    }
                    trialResidualNorm = (trialReduction.masterTransformation.transpose() * trialResidual).norm();
                    trialConstraintError = trialReduction.constraintNorm;
                }
                const double allowedConstraint = 1.1 * std::max(constraintError, m_param.constraintTolerance);
                const double trialResidualLimit = m_param.tol * std::max(1.0, trialExternalForce.norm());
                const double trialResidualChange = constraints.Empty() && trialConstraints.Empty()
                                                       ? (trialResidual - m_R).norm()
                                                       : std::numeric_limits<double>::infinity();
                const bool trialConverged = (trialResidualNorm == 0.0 || previousCorrectionNorm <= m_param.tol ||
                                             (correctionScale == 1.0 && trialResidualChange <= trialResidualLimit)) &&
                                            trialConstraintError <= m_param.constraintTolerance;
                if (std::isfinite(trialResidualNorm) && std::isfinite(trialConstraintError) &&
                    (trialConverged || trialResidualNorm <= (1.0 - armijoSlope * correctionScale) * result.residual) &&
                    trialConstraintError <= allowedConstraint)
                {
                    if (trialConverged && constraints.Empty() && trialConstraints.Empty() &&
                        !trialRequiresEffectiveTangent)
                    {
                        // 无 MPC 试探态已通过同一残差变化或完整修正门，无需重算切线。
                        m_R = std::move(trialResidual);
                        result.residual = trialResidualNorm;
                        result.residualLimit = trialResidualLimit;
                        result.nonlinearMpcMultipliers.resize(0);
                        result.converged = true;
                        return true;
                    }
                    previousFullCorrectionApplied = correctionScale == 1.0;
                    correctionAccepted = true;
                    break;
                }
                correctionScale *= 0.5;
            }
            if (correctionAccepted)
            {
                precisionCorrectionAttempted = false;
                continue;
            }
            if (!model.RestoreTrialState() || !precisionCandidate)
                return false;
            // 恢复节点后同步内力、惯性及阻尼缓存，不能把最后一次失败试算的力带入接受态。
            SystemEvaluationResult restoredEvaluation;
            model.EvaluateSystem(SystemEvaluationRequest::TrialResidual(currentTime + timeStep), restoredEvaluation);
        }

        // 只有实际修正未能满足正常门且残差已停滞，才使用原有精度预算兜底。
        ++m_precisionLimitedSolveCount;
        if (m_precisionLimitedSolveCount <= 3)
            QTextStream(stdout) << "newmark_precision_limited time=" << currentTime + timeStep << " dt=" << timeStep
                                << " residual=" << result.residual << " scaled=" << scaledResidual
                                << " correction=" << scaledCorrection << Qt::endl;
        result.converged = true;
        return true;
    }
    return false;
}

bool SolverNewmark::IsPrecisionLimitedConvergence(const IAnalysisModel& model, const Vec& rhs, const Vec& externalForce,
                                                  const SpMat& tangent, const NonlinearMPCReduction* reduction,
                                                  const NonlinearMPCData* constraints, const Vec& previousVelocity,
                                                  _OUT double& scaledResidual, _OUT double& scaledCorrection) const
{
    constexpr double precisionLimit = 64.0 * std::numeric_limits<double>::epsilon();
    const auto& settings = m_param.adaptiveTimeStep;
    if (!std::isfinite(settings.absoluteTolerance) || settings.absoluteTolerance <= 0.0 ||
        !std::isfinite(settings.velocityAbsoluteTolerance) || settings.velocityAbsoluteTolerance <= 0.0 ||
        !std::isfinite(settings.relativeTolerance) || settings.relativeTolerance < 0.0 ||
        !std::isfinite(settings.velocityRelativeTolerance) || settings.velocityRelativeTolerance < 0.0)
        return false;

    Vec increment;
    Vec displacement;
    Vec velocity;
    Vec acceleration;
    model.GetStepIncrement(increment);
    model.GetState(displacement, velocity, acceleration);
    const int dofCount = model.GetFreeDofs();
    if (increment.size() != dofCount || velocity.size() != dofCount || previousVelocity.size() != dofCount ||
        m_dx.size() != dofCount || !increment.allFinite() || !velocity.allFinite() || !previousVelocity.allFinite() ||
        !m_dx.allFinite())
        return false;

    scaledCorrection = 0.0;
    Vec velocityCorrection = (m_c.a1 * m_dx).cwiseAbs();
    for (int index = 0; index < dofCount; ++index)
    {
        scaledCorrection =
            std::max(scaledCorrection, std::abs(m_dx[index]) / std::max(1.0, std::abs(increment[index])));
        const double displacementScale =
            settings.absoluteTolerance + settings.relativeTolerance * std::abs(increment[index]);
        if (std::abs(m_dx[index]) > 1.0e-3 * displacementScale)
            return false;
    }
    if (scaledCorrection > precisionLimit)
        return false;

    // 姿态修正还会旋转当前空间角速度；a1*delta 本身不足以覆盖此项，尤其 gamma=0 时。
    std::vector<FiniteRotationState> rotationStates;
    model.GetFiniteRotationStates(rotationStates);
    for (const auto& state : rotationStates)
    {
        const int activeComponents = static_cast<int>(std::count_if(state.freeDofs.begin(), state.freeDofs.end(),
                                                                    [](int dof)
                                                                    {
                                                                        return dof >= 0;
                                                                    }));
        if (activeComponents == 0)
            continue;
        // 接口不含固定转轴的角速度，不能把未知分量取零后用于精度停止的上界。
        if (activeComponents != 3)
            return false;
        Eigen::Vector3d angularIncrement = Eigen::Vector3d::Zero();
        Eigen::Vector3d angularCorrection = Eigen::Vector3d::Zero();
        Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
        for (int component = 0; component < 3; ++component)
        {
            const int dof = state.freeDofs[component];
            if (dof < 0)
                continue;
            if (dof >= dofCount)
                return false;
            angularIncrement[component] = increment[dof];
            angularCorrection[component] = m_dx[dof];
            angularVelocity[component] = velocity[dof];
        }
        if (angularIncrement.norm() >= std::acos(-1.0) - 1.0e-6)
            return false;
        // 主转角小于 pi 时，log 的微分算子范数不超过 pi/2；这里保守取 2。
        const double angularVelocityBound =
            (2.0 * std::abs(m_c.a1) + angularVelocity.norm()) * angularCorrection.norm();
        for (int dof : state.freeDofs)
            if (dof >= 0)
                velocityCorrection[dof] = angularVelocityBound;
    }
    if (constraints && !constraints->Empty())
    {
        if (!constraints->IsValid(dofCount))
            return false;
        const int constraintCount = static_cast<int>(constraints->value.size());
        Eigen::MatrixXd slaveJacobian(constraintCount, constraintCount);
        for (int column = 0; column < constraintCount; ++column)
            slaveJacobian.col(column) = constraints->jacobian.col(constraints->slaveDofs[column]);
        Eigen::FullPivLU<Eigen::MatrixXd> factor(slaveJacobian);
        factor.setThreshold(1.0e-12);
        if (factor.rank() != constraintCount)
            return false;

        // 从变量的投影速度变化还包含 -Gs^-1*(G*deltaV + deltaG*v)，不能仅检查 a1*dx。
        Vec projectionRightBound = constraints->jacobian.cwiseAbs() * velocityCorrection;
        const bool hasSparseHessians = static_cast<int>(constraints->hessians.size()) == constraintCount;
        const bool hasLocalHessianEntries = static_cast<int>(constraints->hessianEntries.size()) == constraintCount;
        for (int row = 0; row < constraintCount; ++row)
        {
            if (hasSparseHessians)
                for (int outer = 0; outer < constraints->hessians[row].outerSize(); ++outer)
                    for (SpMat::InnerIterator entry(constraints->hessians[row], outer); entry; ++entry)
                        projectionRightBound[row] +=
                            std::abs(entry.value() * m_dx[entry.row()] * velocity[entry.col()]);
            if (hasLocalHessianEntries)
                for (const auto& entry : constraints->hessianEntries[row])
                    projectionRightBound[row] +=
                        std::abs(entry.value() * m_dx[entry.row()] * velocity[entry.col()]);
        }
        const Vec slaveVelocityBound = factor.inverse().cwiseAbs() * projectionRightBound;
        if (!slaveVelocityBound.allFinite())
            return false;
        for (int column = 0; column < constraintCount; ++column)
            velocityCorrection[constraints->slaveDofs[column]] += slaveVelocityBound[column];
    }
    for (int index = 0; index < dofCount; ++index)
    {
        const double velocityScale =
            settings.velocityAbsoluteTolerance +
            settings.velocityRelativeTolerance * std::max(std::abs(previousVelocity[index]), std::abs(velocity[index]));
        if (!std::isfinite(velocityCorrection[index]) || velocityCorrection[index] > 1.0e-3 * velocityScale)
            return false;
    }

    const Vec independentExternal =
        reduction ? Vec(reduction->masterTransformation.transpose() * externalForce) : externalForce;
    if (rhs.size() == 0 || rhs.size() != independentExternal.size() || tangent.rows() != rhs.size() ||
        tangent.cols() != rhs.size() || !rhs.allFinite() ||
        !independentExternal.allFinite())
        return false;
    Vec independentCoordinateScale(rhs.size());
    for (Eigen::Index index = 0; index < rhs.size(); ++index)
    {
        const Eigen::Index fullDof = reduction ? reduction->masterDofs[static_cast<std::size_t>(index)] : index;
        if (fullDof < 0 || fullDof >= increment.size())
            return false;
        independentCoordinateScale[index] = std::max(1.0, std::abs(increment[fullDof]));
    }
    // 精度停止必须计入同一当前切线的整行耦合；正常 Newton 平衡容限保持原定义。
    scaledResidual = EstimateNewtonPrecisionResidual(rhs, independentExternal, tangent,
                                                     independentCoordinateScale);
    return scaledResidual <= precisionLimit;
}

bool SolverNewmark::SolveLinear(const SpMat& stiffness,
                                const Vec& rhs, Vec& solution, std::uint64_t patternVersion)
{
    return m_linearSolver.Solve(stiffness, rhs, solution, patternVersion);
}
}
