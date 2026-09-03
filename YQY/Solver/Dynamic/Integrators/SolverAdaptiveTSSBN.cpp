/**
 * @file SolverAdaptiveTSSBN.cpp
 * @brief 用于非线性动力学的自适应 rho∞-TSSBN 积分器。
 */
#include "SolverAdaptiveTSSBN.h"
#include "Solver/Constraint/NonlinearMPC.h"

#include <Eigen/Dense>
#include <QDebug>
#include <QString>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace SolverNameSpace
{
namespace
{
constexpr double coefficientTolerance = 1.0e-12;
constexpr double timeTolerance = 1.0e-12;
constexpr double minimumControllerError = 1.0e-14;

bool IsFinitePositive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

double ScaledVectorNorm(const Vec& difference, const Vec& referenceA, const Vec& referenceB, double absoluteTolerance,
                        double relativeTolerance)
{
    if (difference.size() == 0)
        return 0.0;
    const double scale = absoluteTolerance + relativeTolerance * std::max(referenceA.norm(), referenceB.norm());
    return difference.norm() / scale;
}
}

SolverAdaptiveTSSBN::SolverAdaptiveTSSBN(Params parameters)
    : parameters_(std::move(parameters))
{
    ValidateParameters();
    ComputeMethodCoefficients();
}

void SolverAdaptiveTSSBN::ValidateParameters() const
{
    if (!IsFinitePositive(parameters_.initialTimeStep) || !IsFinitePositive(parameters_.minimumTimeStep) ||
        !IsFinitePositive(parameters_.maximumTimeStep) || parameters_.minimumTimeStep > parameters_.maximumTimeStep)
    {
        throw std::invalid_argument("自适应 TSSBN 时间步上下限无效");
    }
    if (parameters_.spectralRadiusInfinity < 0.0 || parameters_.spectralRadiusInfinity > 1.0 ||
        !std::isfinite(parameters_.spectralRadiusInfinity))
    {
        throw std::invalid_argument("自适应 TSSBN 谱半径必须位于 [0, 1]");
    }
    if (!IsFinitePositive(parameters_.relativeTolerance) || !IsFinitePositive(parameters_.absoluteTolerance) ||
        !IsFinitePositive(parameters_.nonlinearTolerance) || parameters_.maximumNewtonIterations < 1 ||
        parameters_.targetNewtonIterations < 1 || parameters_.maximumTotalNewtonIterations < 1 ||
        parameters_.maximumRejectedAttempts < 1)
    {
        throw std::invalid_argument("自适应 TSSBN 容限参数无效");
    }
    if (!(parameters_.safetyFactor > 0.0 && parameters_.safetyFactor <= 1.0) ||
        !(parameters_.shrinkFactor > 0.0 && parameters_.shrinkFactor < 1.0) || parameters_.maximumGrowthFactor < 1.0 ||
        !std::isfinite(parameters_.maximumGrowthFactor) || parameters_.derivativeGain < 0.0 ||
        !std::isfinite(parameters_.derivativeGain) || !(parameters_.minimumDerivativeFactor > 0.0) ||
        parameters_.minimumDerivativeFactor > parameters_.maximumDerivativeFactor ||
        !std::isfinite(parameters_.maximumDerivativeFactor))
    {
        throw std::invalid_argument("自适应 TSSBN 步长控制系数无效");
    }
    if (parameters_.aerodynamicTangentMode < AerodynamicTangentMode::Disabled ||
        parameters_.aerodynamicTangentMode > AerodynamicTangentMode::EveryNewtonIteration)
    {
        throw std::invalid_argument("气动切线模式无效");
    }
}

void SolverAdaptiveTSSBN::ComputeMethodCoefficients()
{
    const double spectralRadius = parameters_.spectralRadiusInfinity;
    if (std::abs(spectralRadius - 1.0) <= coefficientTolerance)
    {
        baseCoefficients_.firstStageTime = 0.25;
        baseCoefficients_.secondStageTime = 0.75;
        baseCoefficients_.secondStageDiagonalFraction = 1.0 / 3.0;
        baseCoefficients_.firstStageWeight = 0.5;
    }
    else
    {
        const double firstStageTime = (std::sqrt(2.0 * (spectralRadius + 1.0)) - 2.0) / (2.0 * (spectralRadius - 1.0));
        const double secondStageTime = (2.0 * spectralRadius * firstStageTime + 1.0) /
                                       (2.0 * spectralRadius * firstStageTime - 2.0 * firstStageTime + 2.0);
        const double diagonalFraction =
            -(2.0 * firstStageTime - 1.0) / (2.0 * secondStageTime - 2.0 * firstStageTime * secondStageTime +
                                             2.0 * spectralRadius * firstStageTime * secondStageTime);
        const double firstWeight = -(2.0 * secondStageTime - 1.0) / (2.0 * firstStageTime - 2.0 * secondStageTime);

        baseCoefficients_.firstStageTime = firstStageTime;
        baseCoefficients_.secondStageTime = secondStageTime;
        baseCoefficients_.secondStageDiagonalFraction = diagonalFraction;
        baseCoefficients_.firstStageWeight = firstWeight;
    }

    Eigen::Matrix3d weightConditions;
    weightConditions << 1.0, 1.0, 1.0, baseCoefficients_.firstStageTime, baseCoefficients_.secondStageTime,
        embeddedCoefficients_.lastStageTime, baseCoefficients_.firstStageTime * baseCoefficients_.firstStageTime,
        baseCoefficients_.secondStageTime * baseCoefficients_.secondStageTime,
        embeddedCoefficients_.lastStageTime * embeddedCoefficients_.lastStageTime;
    const Eigen::Vector3d requiredMoments(1.0, 0.5, 1.0 / 3.0);
    const Eigen::Vector3d embeddedWeights = weightConditions.colPivHouseholderQr().solve(requiredMoments);
    if (!(weightConditions * embeddedWeights - requiredMoments).allFinite() ||
        (weightConditions * embeddedWeights - requiredMoments).norm() > 1.0e-10)
    {
        throw std::runtime_error("自适应 TSSBN 无法计算嵌入格式权重");
    }

    embeddedCoefficients_.firstWeight = embeddedWeights[0];
    embeddedCoefficients_.secondWeight = embeddedWeights[1];
    embeddedCoefficients_.lastWeight = embeddedWeights[2];

    Eigen::Matrix2d lastStageConditions;
    lastStageConditions << 1.0, 1.0, embeddedCoefficients_.lastWeight * baseCoefficients_.firstStageTime,
        embeddedCoefficients_.lastWeight * baseCoefficients_.secondStageTime;
    const double mixedSecondStageMoment = baseCoefficients_.firstStageTime * baseCoefficients_.secondStageTime *
                                              (1.0 - baseCoefficients_.secondStageDiagonalFraction) +
                                          baseCoefficients_.secondStageTime * baseCoefficients_.secondStageTime *
                                              baseCoefficients_.secondStageDiagonalFraction;
    const Eigen::Vector2d lastStageMoments(embeddedCoefficients_.lastStageTime,
                                           1.0 / 6.0 -
                                               embeddedCoefficients_.firstWeight * baseCoefficients_.firstStageTime *
                                                   baseCoefficients_.firstStageTime -
                                               embeddedCoefficients_.secondWeight * mixedSecondStageMoment);
    const Eigen::Vector2d lastStageCoefficients = lastStageConditions.colPivHouseholderQr().solve(lastStageMoments);
    if ((lastStageConditions * lastStageCoefficients - lastStageMoments).norm() > 1.0e-10)
    {
        throw std::runtime_error("自适应 TSSBN 显式末阶段系数奇异");
    }

    embeddedCoefficients_.lastStageFirstCoefficient = lastStageCoefficients[0];
    embeddedCoefficients_.lastStageSecondCoefficient = lastStageCoefficients[1];

    qDebug().noquote() << QStringLiteral("自适应 TSSBN 系数：c1=%1，c2=%2，alpha=%3，b1=%4")
                              .arg(baseCoefficients_.firstStageTime, 0, 'g', 10)
                              .arg(baseCoefficients_.secondStageTime, 0, 'g', 10)
                              .arg(baseCoefficients_.secondStageDiagonalFraction, 0, 'g', 10)
                              .arg(baseCoefficients_.firstStageWeight, 0, 'g', 10);
}

bool SolverAdaptiveTSSBN::SolveImplicitStage(IAnalysisModel& model, double stepStartTime, double timeStep,
                                             const Vec& previousVelocity, const ImplicitStageDefinition& definition,
                                             StageState& stage)
{
    const double diagonalTime = definition.diagonalCoefficient * timeStep;
    if (!IsFinitePositive(diagonalTime))
    {
        lastStageFailureReason_ = QStringLiteral("阶段对角时间系数无效");
        return false;
    }

    model.RollbackDynamicStep();
    lastStageFailureReason_.clear();
    lastResidualNorm_ = std::numeric_limits<double>::quiet_NaN();
    lastResidualLimit_ = std::numeric_limits<double>::quiet_NaN();

    stage.stepIncrement =
        timeStep * (definition.knownDisplacementRate + definition.diagonalCoefficient * definition.initialVelocity);
    model.ApplyIncrement(stage.stepIncrement);
    model.GetStepIncrement(stage.stepIncrement);

    const double velocityDerivative = 1.0 / diagonalTime;
    const double accelerationDerivative = velocityDerivative * velocityDerivative;
    Eigen::MatrixXd dampingFactor;
    model.GetStructuralDampingFactor(dampingFactor);
    if (dampingFactor.rows() != model.GetFreeDofs())
    {
        lastStageFailureReason_ = QStringLiteral("结构阻尼低秩因子尺寸无效");
        return false;
    }
    Vec constrainedForce;
    Vec externalForce;
    bool externalTangentAssembled = false;
    if (parameters_.aerodynamicTangentMode == AerodynamicTangentMode::Disabled)
    {
        externalTangent_.resize(model.GetFreeDofs(), model.GetFreeDofs());
        externalTangent_.setZero();
        externalTangentAssembled = true;
    }

    for (int iteration = 1; iteration <= parameters_.maximumNewtonIterations; ++iteration)
    {
        stage.newtonIterations = iteration;
        stage.velocity =
            (stage.stepIncrement / timeStep - definition.knownDisplacementRate) / definition.diagonalCoefficient;
        stage.acceleration =
            (stage.velocity - previousVelocity - timeStep * definition.knownVelocityRate) / diagonalTime;
        model.SetTssbnStageKinematics(definition.stageIndex, timeStep, baseCoefficients_.firstStageTime,
                                      baseCoefficients_.secondStageTime, baseCoefficients_.secondStageDiagonalFraction,
                                      stage.velocity, stage.acceleration);

        // 气动力及其他状态相关荷载必须在当前试算速度、构型确定后更新。
        model.ComputeExternalForce(stepStartTime + definition.timeFraction * timeStep, 1.0, constrainedForce,
                                   externalForce);
        model.AssembleEffectiveTangent(accelerationDerivative, velocityDerivative, effectiveTangent_);
        if (!externalTangentAssembled ||
            parameters_.aerodynamicTangentMode == AerodynamicTangentMode::EveryNewtonIteration)
        {
            model.AssembleExternalLoadTangent(stepStartTime + definition.timeFraction * timeStep, 1.0,
                                              velocityDerivative, externalTangent_);
            externalTangentAssembled = true;
        }
        effectiveTangent_ -= externalTangent_;
        model.ComputeResidual(externalForce, residual_);

        // Use the same consistently linearized nonlinear MPC
        // elimination as Newmark for every implicit TSSBN stage.
        NonlinearMPCData constraints;
        if (!model.AssembleNonlinearMPC(constraints))
        {
            lastStageFailureReason_ = QStringLiteral("MPC assembly failed");
            return false;
        }
        SpMat solveTangent;
        Vec solveRhs;
        Eigen::MatrixXd solveDampingFactor;
        NonlinearMPCReduction reduction;
        double constraintNorm = 0.0;
        if (constraints.Empty())
        {
            solveTangent = effectiveTangent_;
            solveRhs = residual_;
            solveDampingFactor = dampingFactor;
        }
        else
        {
            if (!NonlinearMPC::Reduce(effectiveTangent_, residual_, constraints, reduction))
            {
                lastStageFailureReason_ = QStringLiteral("MPC reduction failed");
                return false;
            }
            solveTangent = reduction.tangent;
            solveRhs = reduction.rhs;
            solveDampingFactor = reduction.masterTransformation.transpose() * dampingFactor;
            if (dampingFactor.cols() > 0)
            {
                solveRhs -= velocityDerivative * solveDampingFactor *
                            (dampingFactor.transpose() * reduction.particularCorrection);
            }
            constraintNorm = reduction.constraintNorm;
        }

        const double residualScale = std::max(1.0, externalForce.norm());
        lastResidualNorm_ = solveRhs.norm();
        lastResidualLimit_ = parameters_.nonlinearTolerance * residualScale;
        if (solveRhs.allFinite() && std::isfinite(constraintNorm) && lastResidualNorm_ <= lastResidualLimit_ &&
            constraintNorm <= parameters_.nonlinearTolerance)
        {
            if (!constraints.Empty())
                model.SetNonlinearMPCMultipliers(reduction.multipliers);
            return true;
        }
        if (!solveRhs.allFinite() || !std::isfinite(constraintNorm))
        {
            lastStageFailureReason_ = QStringLiteral("非线性残差包含非有限数值");
            return false;
        }
        if (!constraints.Empty())
            effectiveSystemSolver_.Reset();
        Vec independentCorrection;
        const bool linearSolveSucceeded = effectiveSystemSolver_.SolveLowRank(
            solveTangent, solveDampingFactor, velocityDerivative, solveRhs, independentCorrection);
        if (!linearSolveSucceeded)
        {
            lastStageFailureReason_ = QStringLiteral("有效切线矩阵分解或求解失败");
            return false;
        }

        correction_ =
            constraints.Empty() ? independentCorrection : reduction.RecoverFullIncrement(independentCorrection);
        if (correction_.size() != residual_.size() || !correction_.allFinite())
        {
            lastStageFailureReason_ = QStringLiteral("invalid MPC correction");
            return false;
        }
        model.ApplyIncrement(correction_);
        model.GetStepIncrement(stage.stepIncrement);
    }
    lastStageFailureReason_ = QStringLiteral("达到 Newton 最大迭代次数");
    return false;
}

bool SolverAdaptiveTSSBN::EvaluateExplicitLastStage(double timeStep, const Vec& previousVelocity,
                                                    const StageState& firstStage, const StageState& secondStage,
                                                    StageState& lastStage)
{
    lastStage.stepIncrement = timeStep * (embeddedCoefficients_.lastStageFirstCoefficient * firstStage.velocity +
                                          embeddedCoefficients_.lastStageSecondCoefficient * secondStage.velocity);
    lastStage.velocity =
        previousVelocity + timeStep * (embeddedCoefficients_.lastStageFirstCoefficient * firstStage.acceleration +
                                       embeddedCoefficients_.lastStageSecondCoefficient * secondStage.acceleration);

    const double stageSeparation = baseCoefficients_.secondStageTime - baseCoefficients_.firstStageTime;
    if (std::abs(stageSeparation) <= coefficientTolerance)
        return false;
    const double extrapolation =
        (embeddedCoefficients_.lastStageTime - baseCoefficients_.secondStageTime) / stageSeparation;
    lastStage.acceleration =
        secondStage.acceleration + extrapolation * (secondStage.acceleration - firstStage.acceleration);
    lastStage.newtonIterations = 0;
    return lastStage.stepIncrement.allFinite() && lastStage.velocity.allFinite() && lastStage.acceleration.allFinite();
}

void SolverAdaptiveTSSBN::EstablishAcceptedState(IAnalysisModel& model, _OUT Vec& stepIncrement, const Vec& velocity,
                                                 const Vec& acceleration)
{
    model.RollbackDynamicStep();
    model.ApplyIncrement(stepIncrement);
    model.GetStepIncrement(stepIncrement);
    model.SetTrialKinematics(velocity, acceleration);
}

double SolverAdaptiveTSSBN::EstimateNormalizedError(const Vec& baseIncrement, const Vec& embeddedIncrement,
                                                    const Vec& baseVelocity, const Vec& embeddedVelocity,
                                                    const Vec& previousDisplacement, const Vec& previousVelocity) const
{
    // LTE 取两个更新量的差值；相对尺度必须基于物理状态。
    // 若按本步增量缩放，dt 变小时容限会被错误地逐步收紧。
    const double displacementError =
        ScaledVectorNorm(baseIncrement - embeddedIncrement, previousDisplacement, previousDisplacement + baseIncrement,
                         parameters_.absoluteTolerance, parameters_.relativeTolerance);
    const double velocityError = ScaledVectorNorm(baseVelocity - embeddedVelocity, previousVelocity, baseVelocity,
                                                  parameters_.absoluteTolerance, parameters_.relativeTolerance);
    return std::sqrt(0.5 * (displacementError * displacementError + velocityError * velocityError));
}

double SolverAdaptiveTSSBN::ProposeAcceptedTimeStep(double currentTimeStep, double normalizedError,
                                                    int totalStageIterations)
{
    // Mahnken ELDIRK controller: the accepted second-order solution has
    // a third-order local error estimate. Newton work is guarded by the
    // separate total-iteration limit and is not multiplied into LTE
    // prediction, which otherwise creates grow/reject oscillations.
    static_cast<void>(totalStageIterations);
    const double error = std::max(normalizedError, minimumControllerError);
    const double adjustmentFactor = std::clamp(parameters_.safetyFactor * std::pow(error, -1.0 / 3.0),
                                               parameters_.shrinkFactor, parameters_.maximumGrowthFactor);
    return std::clamp(currentTimeStep * adjustmentFactor, parameters_.minimumTimeStep, parameters_.maximumTimeStep);
}

double SolverAdaptiveTSSBN::ProposeRejectedTimeStep(double currentTimeStep, double normalizedError) const
{
    const double error = std::max(normalizedError, 1.0);
    double factor = parameters_.safetyFactor * std::pow(error, -1.0 / 3.0);
    factor = std::min(factor, parameters_.shrinkFactor);
    return std::max(parameters_.minimumTimeStep, currentTimeStep * factor);
}

SolverAdaptiveTSSBN::ImplicitStageDefinition SolverAdaptiveTSSBN::CreateFirstStageDefinition(int dofCount) const
{
    const Vec zero = Vec::Zero(dofCount);
    return {1, baseCoefficients_.firstStageTime, baseCoefficients_.firstStageTime, zero, zero, zero};
}

SolverAdaptiveTSSBN::ImplicitStageDefinition SolverAdaptiveTSSBN::CreateSecondStageDefinition(
    double timeStep, const StageState& firstStage) const
{
    const double previousStageCoefficient =
        baseCoefficients_.secondStageTime * (1.0 - baseCoefficients_.secondStageDiagonalFraction);
    const double diagonalCoefficient =
        baseCoefficients_.secondStageTime * baseCoefficients_.secondStageDiagonalFraction;
    const Vec knownDisplacementRate = previousStageCoefficient * firstStage.velocity;
    const Vec knownVelocityRate = previousStageCoefficient * firstStage.acceleration;
    const Vec initialVelocity = (firstStage.stepIncrement / timeStep - knownDisplacementRate) / diagonalCoefficient;
    return {2,
            baseCoefficients_.secondStageTime,
            diagonalCoefficient,
            knownDisplacementRate,
            knownVelocityRate,
            initialVelocity};
}

bool SolverAdaptiveTSSBN::Solve(IAnalysisModel& model, double duration)
{
    const int dofCount = model.GetFreeDofs();
    if (dofCount <= 0 || !IsFinitePositive(duration))
        return false;

    tangentStiffness_.resize(dofCount, dofCount);
    dynamicTangent_.resize(dofCount, dofCount);
    effectiveTangent_.resize(dofCount, dofCount);
    residual_.resize(dofCount);
    correction_.resize(dofCount);
    effectiveSystemSolver_.Reset();

    acceptedStepCount_ = 0;
    rejectedStepCount_ = 0;
    totalStageNewtonIterations_ = 0;
    maximumStageNewtonIterations_ = 0;
    averageTimeStep_ = 0.0;

    Vec previousDisplacement;
    Vec previousVelocity;
    Vec previousAcceleration;
    model.GetState(previousDisplacement, previousVelocity, previousAcceleration);

    double currentTime = 0.0;
    double nextTimeStep =
        std::clamp(parameters_.initialTimeStep, parameters_.minimumTimeStep, parameters_.maximumTimeStep);
    double acceptedTimeStepSum = 0.0;

    qDebug().noquote() << QStringLiteral("开始自适应 TSSBN：dt=%1，相对 LTE 容限=%2，ρ∞=%3，气动切线模式=%4")
                              .arg(nextTimeStep, 0, 'g', 8)
                              .arg(parameters_.relativeTolerance, 0, 'g', 8)
                              .arg(parameters_.spectralRadiusInfinity, 0, 'g', 8)
                              .arg(static_cast<int>(parameters_.aerodynamicTangentMode));

    while (currentTime < duration - timeTolerance)
    {
        if (model.IsCancellationRequested())
            return false;

        const double remainingTime = duration - currentTime;
        double attemptedTimeStep = std::min(nextTimeStep, remainingTime);
        model.BackupStepState();
        bool stepAccepted = false;
        QString lastFailedOperation;
        double lastNormalizedError = std::numeric_limits<double>::quiet_NaN();

        for (int attempt = 1; attempt <= parameters_.maximumRejectedAttempts; ++attempt)
        {
            QString failedOperation;
            // 与参考 TSSBN 的阶段初始化一致：q_c1 从 q_n 开始，
            // 因此 v_c1 的初始值为零。
            const ImplicitStageDefinition firstDefinition = CreateFirstStageDefinition(dofCount);
            StageState firstStage;
            bool stagesConverged = SolveImplicitStage(model, currentTime, attemptedTimeStep, previousVelocity,
                                                      firstDefinition, firstStage);
            if (!stagesConverged)
            {
                failedOperation = QStringLiteral("C1");
                lastFailedOperation = failedOperation;
            }

            StageState secondStage;
            if (stagesConverged)
            {
                const ImplicitStageDefinition secondDefinition =
                    CreateSecondStageDefinition(attemptedTimeStep, firstStage);
                stagesConverged = SolveImplicitStage(model, currentTime, attemptedTimeStep, previousVelocity,
                                                     secondDefinition, secondStage);
                if (!stagesConverged)
                {
                    failedOperation = QStringLiteral("C2");
                    lastFailedOperation = failedOperation;
                }
            }

            if (stagesConverged)
            {
                const int trialControlIterations =
                    std::max(0, firstStage.newtonIterations + secondStage.newtonIterations - 2);
                if (trialControlIterations > parameters_.maximumTotalNewtonIterations)
                {
                    stagesConverged = false;
                    failedOperation = QStringLiteral("C1+C2");
                    lastFailedOperation = failedOperation;
                    lastStageFailureReason_ = QStringLiteral("牛顿修正总数超过自适应上限");
                }
            }

            StageState explicitLastStage;
            if (stagesConverged)
            {
                stagesConverged = EvaluateExplicitLastStage(attemptedTimeStep, previousVelocity, firstStage,
                                                            secondStage, explicitLastStage);
                if (!stagesConverged)
                {
                    failedOperation = QStringLiteral("显式末阶段");
                    lastFailedOperation = failedOperation;
                }
            }

            if (!stagesConverged)
            {
                model.RollbackDynamicStep();
                ++rejectedStepCount_;
                if (attemptedTimeStep <= parameters_.minimumTimeStep * (1.0 + 1.0e-9))
                {
                    const QString failureMessage = QStringLiteral("自适应 TSSBN 的 %1 在 t=%2 失败，"
                                                                  "最小 dt=%3：%4（残差=%5，限值=%6）")
                                                       .arg(failedOperation)
                                                       .arg(currentTime, 0, 'g', 10)
                                                       .arg(attemptedTimeStep, 0, 'g', 10)
                                                       .arg(lastStageFailureReason_)
                                                       .arg(lastResidualNorm_, 0, 'g', 8)
                                                       .arg(lastResidualLimit_, 0, 'g', 8);
                    model.ReportProgress(currentTime / duration, failureMessage);
                    qDebug().noquote() << failureMessage;
                    qDebug().noquote() << QStringLiteral("自适应 TSSBN 在最小时间步未收敛");
                    return false;
                }
                attemptedTimeStep = std::max(parameters_.minimumTimeStep, attemptedTimeStep * parameters_.shrinkFactor);
                continue;
            }

            const double firstWeight = baseCoefficients_.firstStageWeight;
            Vec baseIncrement =
                attemptedTimeStep * (firstWeight * firstStage.velocity + (1.0 - firstWeight) * secondStage.velocity);
            Vec baseVelocity = previousVelocity + attemptedTimeStep * (firstWeight * firstStage.acceleration +
                                                                       (1.0 - firstWeight) * secondStage.acceleration);
            Vec embeddedIncrement = attemptedTimeStep * (embeddedCoefficients_.firstWeight * firstStage.velocity +
                                                         embeddedCoefficients_.secondWeight * secondStage.velocity +
                                                         embeddedCoefficients_.lastWeight * explicitLastStage.velocity);
            Vec embeddedVelocity =
                previousVelocity +
                attemptedTimeStep * (embeddedCoefficients_.firstWeight * firstStage.acceleration +
                                     embeddedCoefficients_.secondWeight * secondStage.acceleration +
                                     embeddedCoefficients_.lastWeight * explicitLastStage.acceleration);

            Vec acceptedAcceleration = explicitLastStage.acceleration;
            model.CorrectTssbnStepStates(
                attemptedTimeStep, baseCoefficients_.firstStageTime, baseCoefficients_.secondStageTime,
                embeddedCoefficients_.lastStageTime, firstWeight, embeddedCoefficients_.firstWeight,
                embeddedCoefficients_.secondWeight, embeddedCoefficients_.lastWeight,
                embeddedCoefficients_.lastStageFirstCoefficient, embeddedCoefficients_.lastStageSecondCoefficient,
                baseIncrement, baseVelocity, embeddedIncrement, embeddedVelocity, acceptedAcceleration);

            const double normalizedError =
                EstimateNormalizedError(baseIncrement, embeddedIncrement, baseVelocity, embeddedVelocity,
                                        previousDisplacement, previousVelocity);
            lastNormalizedError = normalizedError;
            if (!std::isfinite(normalizedError))
            {
                model.RollbackDynamicStep();
                return false;
            }

            if (normalizedError > 1.0)
            {
                lastFailedOperation = QStringLiteral("LTE");
                lastStageFailureReason_ = QStringLiteral("局部误差 LTE 未满足容限");
                if (attemptedTimeStep <= parameters_.minimumTimeStep * (1.0 + 1.0e-9))
                {
                    model.RollbackDynamicStep();
                    qDebug().noquote() << QStringLiteral("自适应 TSSBN 在最小 dt=%1 无法满足局部误差 "
                                                         "LTE 容限（误差=%2）")
                                              .arg(attemptedTimeStep, 0, 'g', 8)
                                              .arg(normalizedError, 0, 'g', 8);
                    const QString failureMessage = QStringLiteral("自适应 TSSBN 在 t=%1 出现 LTE 失败，"
                                                                  "最小 dt=%2，归一化误差=%3")
                                                       .arg(currentTime, 0, 'g', 10)
                                                       .arg(attemptedTimeStep, 0, 'g', 10)
                                                       .arg(normalizedError, 0, 'g', 8);
                    model.ReportProgress(currentTime / duration, failureMessage);
                    qDebug().noquote() << failureMessage;
                    return false;
                }
                model.RollbackDynamicStep();
                ++rejectedStepCount_;
                attemptedTimeStep = ProposeRejectedTimeStep(attemptedTimeStep, normalizedError);
                continue;
            }

            EstablishAcceptedState(model, baseIncrement, baseVelocity, acceptedAcceleration);

            model.CommitState();
            currentTime += attemptedTimeStep;
            ++acceptedStepCount_;
            acceptedTimeStepSum += attemptedTimeStep;
            model.GetState(previousDisplacement, previousVelocity, previousAcceleration);

            const int totalStageIterations = firstStage.newtonIterations + secondStage.newtonIterations;
            const int controlIterations = std::max(1, totalStageIterations - 2);
            totalStageNewtonIterations_ += controlIterations;
            maximumStageNewtonIterations_ = std::max(maximumStageNewtonIterations_, controlIterations);
            model.RecordStepIterations(currentTime, controlIterations);
            if (stepCallback_)
            {
                stepCallback_(acceptedStepCount_, currentTime, previousDisplacement);
            }
            model.OnStepCompleted(currentTime);
            model.ReportProgress(std::min(1.0, currentTime / duration),
                                 QStringLiteral("自适应 TSSBN 第 %1 步，dt=%2，误差=%3")
                                     .arg(acceptedStepCount_)
                                     .arg(attemptedTimeStep, 0, 'g', 6)
                                     .arg(normalizedError, 0, 'g', 4));

            nextTimeStep = ProposeAcceptedTimeStep(attemptedTimeStep, normalizedError, controlIterations);
            stepAccepted = true;
            break;
        }

        if (!stepAccepted)
        {
            model.RollbackDynamicStep();
            const QString failureMessage = QStringLiteral("自适应 TSSBN 在 t=%1 达到拒绝重试上限，"
                                                          "最后 dt=%2，阶段=%3，原因=%4，"
                                                          "残差=%5（限值=%6），归一化误差=%7")
                                               .arg(currentTime, 0, 'g', 10)
                                               .arg(attemptedTimeStep, 0, 'g', 10)
                                               .arg(lastFailedOperation)
                                               .arg(lastStageFailureReason_)
                                               .arg(lastResidualNorm_, 0, 'g', 8)
                                               .arg(lastResidualLimit_, 0, 'g', 8)
                                               .arg(lastNormalizedError, 0, 'g', 8);
            model.ReportProgress(currentTime / duration, failureMessage);
            qDebug().noquote() << failureMessage;
            return false;
        }
    }

    averageTimeStep_ = acceptedStepCount_ > 0 ? acceptedTimeStepSum / static_cast<double>(acceptedStepCount_) : 0.0;
    qDebug().noquote() << QStringLiteral("自适应 TSSBN 完成：接受=%1，拒绝=%2，"
                                         "平均 dt=%3，平均 Newton=%4，最大 Newton=%5")
                              .arg(acceptedStepCount_)
                              .arg(rejectedStepCount_)
                              .arg(averageTimeStep_, 0, 'g', 8)
                              .arg(GetAverageStageNewtonIterations(), 0, 'g', 6)
                              .arg(maximumStageNewtonIterations_);
    return true;
}
}
