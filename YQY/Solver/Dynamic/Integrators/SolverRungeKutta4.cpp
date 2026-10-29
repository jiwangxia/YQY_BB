/** @file SolverRungeKutta4.cpp @brief 显式四阶 Runge-Kutta 动力求解器。 */
#include "SolverRungeKutta4.h"
#include "../TimeStepping/StateCheckpointGuard.h"
#include "../TimeStepping/TimeStepError.h"

#include <QDebug>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace SolverNameSpace
{
SolverRungeKutta4::SolverRungeKutta4(Params parameters)
    : parameters_(std::move(parameters))
{
}

bool SolverRungeKutta4::Solve(IAnalysisModel& model, double duration)
{
    timeStepStatistics_ = {};
    massPatternVersion_ = 0;
    massPatternValid_ = false;
    massFactorizationValid_ = false;
    massStateDependent_ = model.HasStateDependentMass();
    massPatternAnalysisCount_ = 0;
    massFactorizationCount_ = 0;
    massFactorizationReuseCount_ = 0;
    massAssemblyCount_ = 0;
    massAssemblyReuseCount_ = 0;
    cachedMass_.resize(0, 0);
    if (!std::isfinite(parameters_.timeStep) || parameters_.timeStep <= 0.0 || !std::isfinite(duration) ||
        duration <= 0.0 ||
        (parameters_.timeStepMode != TimeStepMode::Fixed && parameters_.timeStepMode != TimeStepMode::Adaptive))
        return false;
    if (model.HasActiveNonlinearMPC())
    {
        model.ReportProgress(
            0.0, QStringLiteral("Runge-Kutta 4 显式积分暂不支持非线性 MPC，请改用 Newmark 或自适应 TSSBN。"));
        return false;
    }
    std::unique_ptr<TimeStepController> controller;
    std::vector<FiniteRotationState> initialRotations;
    std::vector<Eigen::Matrix3d> initialAuxiliaryRotations;
    model.GetFiniteRotationStates(initialRotations);
    for (const auto& state : initialRotations)
    {
        const int activeAxes = static_cast<int>(std::count_if(state.freeDofs.begin(), state.freeDofs.end(),
                                                              [](int dof)
                                                              {
                                                                  return dof >= 0;
                                                              }));
        if (activeAxes != 2)
            continue;
        const QString message = QStringLiteral("RK4 尚不支持两个转动轴活动且另一个转动轴固定的有限转动约束，"
                                               "请用完整三维转动与非线性约束描述。");
        model.ReportProgress(0.0, message);
        qDebug().noquote() << message;
        return false;
    }
    if (parameters_.timeStepMode == TimeStepMode::Adaptive)
    {
        // CR 的已接受 Gauss 姿态使整个增广状态只有二阶；空辅助状态保持经典四阶控制。
        // 首次查询在检查点之前初始化惰性历史，避免首次装配晚于备份。
        model.GetAuxiliaryFiniteRotationStates(initialAuxiliaryRotations);
        try
        {
            controller = std::make_unique<TimeStepController>(parameters_.adaptiveTimeStep);
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    double time = 0.0;
    double nextTimeStep = parameters_.timeStep;
    if (parameters_.timeStepMode == TimeStepMode::Adaptive)
    {
        nextTimeStep = std::clamp(nextTimeStep, parameters_.adaptiveTimeStep.minimumTimeStep,
                                  parameters_.adaptiveTimeStep.maximumTimeStep);
    }
    int acceptedSteps = 0;
    int rejectedAttempts = 0;
    const int estimatedSteps = static_cast<int>(std::ceil(duration / parameters_.timeStep));
    while (time < duration - 1.0e-12)
    {
        const double timeStep = SelectTimeStep(time, duration, nextTimeStep);
        if (model.IsCancellationRequested())
            return false;

        if (parameters_.timeStepMode == TimeStepMode::Fixed)
        {
            Vec displacement;
            Vec velocity;
            Vec acceleration;
            if (!SolveSingleStep(model, time, timeStep, displacement, velocity, acceleration))
                return false;
            model.CommitState();
            ++acceptedSteps;
            timeStepStatistics_.RecordAccepted(timeStep);
            time += timeStep;
            model.RecordStepIterations(time, 1);
            if (callback_)
                callback_(acceptedSteps, time, displacement);
            model.OnStepCompleted(time);
            model.ReportProgress(std::min(1.0, time / duration),
                                 QStringLiteral("RK4 第 %1/%2 步").arg(acceptedSteps).arg(estimatedSteps));
            continue;
        }

        StateCheckpointGuard checkpoint(model);
        if (!checkpoint.IsActive())
            return false;
        std::vector<Eigen::Matrix3d> stepStartAuxiliaryRotations;
        model.GetAuxiliaryFiniteRotationStates(stepStartAuxiliaryRotations);
        if (stepStartAuxiliaryRotations.size() != initialAuxiliaryRotations.size())
            return false;
        const int methodOrder = stepStartAuxiliaryRotations.empty() ? 4 : 2;
        Vec coarseDisplacement;
        Vec coarseVelocity;
        Vec coarseAcceleration;
        std::vector<FiniteRotationState> coarseRotations;
        std::vector<Eigen::Matrix3d> coarseAuxiliaryRotations;
        bool solved = SolveSingleStep(model, time, timeStep, coarseDisplacement, coarseVelocity, coarseAcceleration);
        if (solved)
        {
            model.GetFiniteRotationStates(coarseRotations);
            model.GetAuxiliaryFiniteRotationStates(coarseAuxiliaryRotations);
        }
        if (!checkpoint.Restore())
            return false;

        Vec halfDisplacement;
        Vec halfVelocity;
        Vec halfAcceleration;
        if (solved)
            solved = SolveSingleStep(model, time, 0.5 * timeStep, halfDisplacement, halfVelocity, halfAcceleration);
        if (solved)
            model.CommitState();
        Vec fineDisplacement;
        Vec fineVelocity;
        Vec fineAcceleration;
        if (solved)
            solved = SolveSingleStep(model, time + 0.5 * timeStep, 0.5 * timeStep, fineDisplacement, fineVelocity,
                                     fineAcceleration);
        std::vector<FiniteRotationState> fineRotations;
        std::vector<Eigen::Matrix3d> fineAuxiliaryRotations;
        if (solved)
        {
            model.GetFiniteRotationStates(fineRotations);
            model.GetAuxiliaryFiniteRotationStates(fineAuxiliaryRotations);
        }
        double error = std::numeric_limits<double>::infinity();
        if (solved)
        {
            const double stateError = EstimateStepError(coarseDisplacement, coarseVelocity, fineDisplacement,
                                                        fineVelocity, coarseRotations, fineRotations, methodOrder);
            const double auxiliaryError = EstimateAuxiliaryRotationError(
                stepStartAuxiliaryRotations, coarseAuxiliaryRotations, fineAuxiliaryRotations);
            error = std::max(stateError, auxiliaryError);
        }
        const TimeStepDecision decision =
            solved ? controller->Decide(timeStep, error, methodOrder) : controller->DecideSolveFailure(timeStep);
        if (!decision.accepted)
        {
            if (!checkpoint.Restore())
                return false;
            if (decision.nextTimeStep >= timeStep ||
                ++rejectedAttempts > parameters_.adaptiveTimeStep.maximumRejectedAttempts)
                return false;
            timeStepStatistics_.RecordRejected();
            nextTimeStep = decision.nextTimeStep;
            continue;
        }
        model.CommitState();
        checkpoint.Release();
        ++acceptedSteps;
        timeStepStatistics_.RecordAccepted(timeStep);
        time += timeStep;
        nextTimeStep = decision.nextTimeStep;
        rejectedAttempts = 0;
        model.RecordStepIterations(time, 3);
        if (callback_)
            callback_(acceptedSteps, time, fineDisplacement);
        model.OnStepCompleted(time);
        model.ReportProgress(std::min(1.0, time / duration), QStringLiteral("自适应 RK4 第 %1 步，dt=%2，误差=%3")
                                                                 .arg(acceptedSteps)
                                                                 .arg(timeStep, 0, 'g', 6)
                                                                 .arg(error, 0, 'g', 4));
    }
    return true;
}

bool SolverRungeKutta4::SolveSingleStep(IAnalysisModel& model, double time, double timeStep, _OUT Vec& displacement,
                                        _OUT Vec& velocity, _OUT Vec& acceleration)
{
    model.BackupStepState();
    Vec initialDisplacement;
    Vec initialVelocity;
    Vec initialAcceleration;
    model.GetState(initialDisplacement, initialVelocity, initialAcceleration);
    std::vector<FiniteRotationState> rotations;
    model.GetFiniteRotationStates(rotations);
    const Vec zeroIncrement = Vec::Zero(initialVelocity.size());
    Vec increment2;
    Vec increment3;
    Vec increment4;
    Vec rate1;
    Vec rate2;
    Vec rate3;
    Vec rate4;
    Vec a1;
    Vec a2;
    Vec a3;
    Vec a4;
    bool solved = EvaluateAcceleration(model, time, zeroIncrement, initialVelocity, a1) &&
                  ComputeConfigurationRate(rotations, zeroIncrement, initialVelocity, rate1);
    if (solved)
    {
        increment2 = 0.5 * timeStep * rate1;
        const Vec velocity2 = initialVelocity + 0.5 * timeStep * a1;
        solved = EvaluateAcceleration(model, time + 0.5 * timeStep, increment2, velocity2, a2) &&
                 ComputeConfigurationRate(rotations, increment2, velocity2, rate2);
    }
    if (solved)
    {
        increment3 = 0.5 * timeStep * rate2;
        const Vec velocity3 = initialVelocity + 0.5 * timeStep * a2;
        solved = EvaluateAcceleration(model, time + 0.5 * timeStep, increment3, velocity3, a3) &&
                 ComputeConfigurationRate(rotations, increment3, velocity3, rate3);
    }
    if (solved)
    {
        increment4 = timeStep * rate3;
        const Vec velocity4 = initialVelocity + timeStep * a3;
        solved = EvaluateAcceleration(model, time + timeStep, increment4, velocity4, a4) &&
                 ComputeConfigurationRate(rotations, increment4, velocity4, rate4);
    }
    if (!solved)
    {
        model.RollbackDynamicStep();
        return false;
    }

    const Vec stepIncrement = timeStep * (rate1 + 2.0 * rate2 + 2.0 * rate3 + rate4) / 6.0;
    velocity = initialVelocity + timeStep * (a1 + 2.0 * a2 + 2.0 * a3 + a4) / 6.0;
    if (!EvaluateAcceleration(model, time + timeStep, stepIncrement, velocity, acceleration))
    {
        model.RollbackDynamicStep();
        return false;
    }
    model.GetState(displacement, velocity, acceleration);
    return displacement.allFinite() && velocity.allFinite() && acceleration.allFinite();
}

bool SolverRungeKutta4::ComputeConfigurationRate(const std::vector<FiniteRotationState>& rotations,
                                                 const Vec& stepIncrement, const Vec& velocity, _OUT Vec& rate) const
{
    if (stepIncrement.size() != velocity.size() || !stepIncrement.allFinite() || !velocity.allFinite())
        return false;
    rate = velocity;
    for (const auto& state : rotations)
    {
        Eigen::Vector3d increment = Eigen::Vector3d::Zero();
        Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
        for (int component = 0; component < 3; ++component)
        {
            const int dof = state.freeDofs[component];
            if (dof < 0)
                continue;
            if (dof >= velocity.size())
                return false;
            increment[component] = stepIncrement[dof];
            angularVelocity[component] = velocity[dof];
        }
        // 四阶 RKMK 保留 dexp^{-1} 至 ad^2；省略项为 O(dt^4)，不改变四阶精度。
        const Eigen::Vector3d firstCommutator = increment.cross(angularVelocity);
        const Eigen::Vector3d rotationRate =
            angularVelocity - 0.5 * firstCommutator + increment.cross(firstCommutator) / 12.0;
        for (int component = 0; component < 3; ++component)
            if (state.freeDofs[component] >= 0)
                rate[state.freeDofs[component]] = rotationRate[component];
    }
    return rate.allFinite();
}

double SolverRungeKutta4::EstimateStepError(const Vec& coarseDisplacement, const Vec& coarseVelocity,
                                            const Vec& fineDisplacement, const Vec& fineVelocity,
                                            const std::vector<FiniteRotationState>& coarseRotations,
                                            const std::vector<FiniteRotationState>& fineRotations,
                                            int methodOrder) const
{
    if (coarseRotations.size() != fineRotations.size())
        return std::numeric_limits<double>::infinity();
    Vec coarseTranslation = coarseDisplacement;
    Vec fineTranslation = fineDisplacement;
    const double denominator = std::pow(2.0, static_cast<double>(methodOrder)) - 1.0;
    double rotationError = 0.0;
    for (std::size_t node = 0; node < coarseRotations.size(); ++node)
    {
        const auto& coarse = coarseRotations[node];
        const auto& fine = fineRotations[node];
        if (coarse.freeDofs != fine.freeDofs || !coarse.rotation.allFinite() || !fine.rotation.allFinite())
            return std::numeric_limits<double>::infinity();
        const Eigen::AngleAxisd difference(fine.rotation * coarse.rotation.transpose());
        const Eigen::Vector3d rotationDifference = difference.angle() * difference.axis();
        const double rotationScale =
            parameters_.adaptiveTimeStep.absoluteTolerance +
            parameters_.adaptiveTimeStep.relativeTolerance *
                std::max(Eigen::AngleAxisd(coarse.rotation).angle(), Eigen::AngleAxisd(fine.rotation).angle());
        if (!std::isfinite(rotationScale) || rotationScale <= 0.0 || !rotationDifference.allFinite())
            return std::numeric_limits<double>::infinity();
        for (int component = 0; component < 3; ++component)
        {
            const int dof = coarse.freeDofs[component];
            if (dof < 0)
                continue;
            if (dof >= coarseTranslation.size() || dof >= fineTranslation.size())
                return std::numeric_limits<double>::infinity();
            coarseTranslation[dof] = 0.0;
            fineTranslation[dof] = 0.0;
            rotationError =
                std::max(rotationError, std::abs(rotationDifference[component]) / (denominator * rotationScale));
        }
    }
    return std::max(rotationError, EstimateStepDoublingError(coarseTranslation, coarseVelocity, fineTranslation,
                                                             fineVelocity, parameters_.adaptiveTimeStep, methodOrder));
}

double SolverRungeKutta4::EstimateAuxiliaryRotationError(
    const std::vector<Eigen::Matrix3d>& stepStartRotations, const std::vector<Eigen::Matrix3d>& coarseRotations,
    const std::vector<Eigen::Matrix3d>& fineRotations) const
{
    if (coarseRotations.size() != stepStartRotations.size() || fineRotations.size() != stepStartRotations.size())
        return std::numeric_limits<double>::infinity();
    double error = 0.0;
    for (std::size_t index = 0; index < stepStartRotations.size(); ++index)
    {
        const auto& initial = stepStartRotations[index];
        const auto& coarse = coarseRotations[index];
        const auto& fine = fineRotations[index];
        for (const auto* rotation : {&initial, &coarse, &fine})
            if (!rotation->allFinite() ||
                (rotation->transpose() * *rotation - Eigen::Matrix3d::Identity()).norm() > 1.0e-8 ||
                std::abs(rotation->determinant() - 1.0) > 1.0e-8)
                return std::numeric_limits<double>::infinity();
        const double difference = Eigen::AngleAxisd(fine * coarse.transpose()).angle();
        // 辅助姿态的相对尺度以当前步初为基准，不随梁参考轴 R0 或整体坐标朝向改变。
        const double rotationScale = parameters_.adaptiveTimeStep.absoluteTolerance +
                                     parameters_.adaptiveTimeStep.relativeTolerance *
                                         std::max(Eigen::AngleAxisd(coarse * initial.transpose()).angle(),
                                                  Eigen::AngleAxisd(fine * initial.transpose()).angle());
        if (!std::isfinite(difference) || !std::isfinite(rotationScale) || rotationScale <= 0.0)
            return std::numeric_limits<double>::infinity();
        // 当前 CR 辅助状态为二阶，接受两半步细解的局部误差是粗细差的 1/(2^2-1)。
        error = std::max(error, difference / (3.0 * rotationScale));
    }
    return error;
}

bool SolverRungeKutta4::EvaluateAcceleration(IAnalysisModel& model, double time, const Vec& stepIncrement,
                                             const Vec& velocity, _OUT Vec& acceleration)
{
    model.RollbackDynamicStep();
    model.ApplyIncrement(stepIncrement);
    acceleration = Vec::Zero(velocity.size());
    model.SetTrialKinematics(velocity, acceleration);

    NonlinearMPCData constraints;
    if (!model.AssembleNonlinearMPC(constraints))
    {
        model.ReportProgress(0.0, QStringLiteral("Runge-Kutta 4 非线性 MPC 状态检查失败。"));
        return false;
    }
    if (!constraints.Empty())
    {
        model.ReportProgress(
            0.0, QStringLiteral("Runge-Kutta 4 显式积分暂不支持非线性 MPC，请改用 Newmark 或自适应 TSSBN。"));
        return false;
    }
    // RK 每个阶段都有独立的试探状态，统一由系统评估层刷新相应的单元贡献。
    const std::uint64_t currentPatternVersion = model.GetSparsePatternVersion();
    const bool patternChanged =
        !massPatternValid_ || currentPatternVersion == 0 || massPatternVersion_ != currentPatternVersion;
    const bool assembleMass = massStateDependent_ || cachedMass_.rows() == 0 || patternChanged;
    SystemEvaluationResult evaluation;
    model.EvaluateSystem(SystemEvaluationRequest::ExplicitDynamics(time, assembleMass), evaluation);
    if (assembleMass)
    {
        cachedMass_ = std::move(evaluation.mass);
        ++massAssemblyCount_;
    }
    else
    {
        ++massAssemblyReuseCount_;
    }
    const SpMat& mass = cachedMass_;
    Vec externalForce = std::move(evaluation.externalForce);
    Vec residual;
    model.ComputeResidual(externalForce, residual);
    const bool evaluatedPatternChanged = !massPatternValid_ || evaluation.sparsePatternVersion == 0 ||
                                         massPatternVersion_ != evaluation.sparsePatternVersion;
    if (evaluatedPatternChanged)
    {
        massSolver_.analyzePattern(mass);
        ++massPatternAnalysisCount_;
        massPatternValid_ = true;
        massFactorizationValid_ = false;
        massPatternVersion_ = evaluation.sparsePatternVersion;
    }

    if (massStateDependent_ || !massFactorizationValid_)
    {
        massSolver_.factorize(mass);
        ++massFactorizationCount_;
        massFactorizationValid_ = massSolver_.info() == Eigen::Success;
        if (!massFactorizationValid_)
            return false;
    }
    else
    {
        ++massFactorizationReuseCount_;
    }

    acceleration = massSolver_.solve(residual);
    if (massSolver_.info() != Eigen::Success || !acceleration.allFinite())
        return false;
    model.SetTrialKinematics(velocity, acceleration);
    return true;
}
}
