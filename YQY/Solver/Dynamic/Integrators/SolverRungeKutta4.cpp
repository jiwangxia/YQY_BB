/** @file SolverRungeKutta4.cpp @brief 显式四阶 Runge-Kutta 动力求解器。 */
#include "SolverRungeKutta4.h"
#include "../TimeStepping/StateCheckpointGuard.h"
#include "../TimeStepping/TimeStepError.h"

#include <Eigen/SparseLU>
#include <QDebug>
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
    if (!std::isfinite(parameters_.timeStep) || parameters_.timeStep <= 0.0 || !std::isfinite(duration) ||
        duration <= 0.0 ||
        (parameters_.timeStepMode != TimeStepMode::Fixed && parameters_.timeStepMode != TimeStepMode::Adaptive))
        return false;
    std::unique_ptr<TimeStepController> controller;
    if (parameters_.timeStepMode == TimeStepMode::Adaptive)
    {
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
        Vec coarseDisplacement;
        Vec coarseVelocity;
        Vec coarseAcceleration;
        bool solved = SolveSingleStep(model, time, timeStep, coarseDisplacement, coarseVelocity, coarseAcceleration);
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

        const double error = solved ? EstimateStepDoublingError(coarseDisplacement, coarseVelocity, fineDisplacement,
                                                                fineVelocity, parameters_.adaptiveTimeStep, 4)
                                    : std::numeric_limits<double>::infinity();
        const TimeStepDecision decision =
            solved ? controller->Decide(timeStep, error, 4) : controller->DecideSolveFailure(timeStep);
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

bool SolverRungeKutta4::SolveSingleStep(IAnalysisModel& model, double time, double timeStep, Vec& displacement,
                                        Vec& velocity, Vec& acceleration)
{
    model.BackupStepState();
    Vec initialDisplacement;
    Vec initialVelocity;
    Vec initialAcceleration;
    model.GetState(initialDisplacement, initialVelocity, initialAcceleration);
    Vec a1;
    Vec a2;
    Vec a3;
    Vec a4;
    if (!EvaluateAcceleration(model, time, initialDisplacement, initialDisplacement, initialVelocity, a1) ||
        !EvaluateAcceleration(model, time + 0.5 * timeStep, initialDisplacement,
                              initialDisplacement + 0.5 * timeStep * initialVelocity,
                              initialVelocity + 0.5 * timeStep * a1, a2) ||
        !EvaluateAcceleration(model, time + 0.5 * timeStep, initialDisplacement,
                              initialDisplacement + 0.5 * timeStep * (initialVelocity + 0.5 * timeStep * a1),
                              initialVelocity + 0.5 * timeStep * a2, a3) ||
        !EvaluateAcceleration(model, time + timeStep, initialDisplacement,
                              initialDisplacement + timeStep * (initialVelocity + 0.5 * timeStep * a2),
                              initialVelocity + timeStep * a3, a4))
    {
        model.RollbackDynamicStep();
        return false;
    }

    displacement = initialDisplacement + timeStep * initialVelocity + timeStep * timeStep * (a1 + a2 + a3) / 6.0;
    velocity = initialVelocity + timeStep * (a1 + 2.0 * a2 + 2.0 * a3 + a4) / 6.0;
    if (!EvaluateAcceleration(model, time + timeStep, initialDisplacement, displacement, velocity, acceleration))
    {
        model.RollbackDynamicStep();
        return false;
    }
    return true;
}

bool SolverRungeKutta4::EvaluateAcceleration(IAnalysisModel& model, double time, const Vec& initialDisplacement,
                                             const Vec& displacement, const Vec& velocity, Vec& acceleration)
{
    model.RollbackDynamicStep();
    model.ApplyIncrement(displacement - initialDisplacement);
    acceleration = Vec::Zero(velocity.size());
    model.SetTrialKinematics(velocity, acceleration);

    NonlinearMPCData constraints;
    if (!model.AssembleNonlinearMPC(constraints) || !constraints.Empty())
        return false;
    // RK 每个阶段都有独立的试探位移，必须刷新该状态对应的单元内力。
    model.UpdateInternalForce();
    SpMat mass;
    SpMat velocityTangent;
    SpMat configurationTangent;
    model.AssembleDynamicSystem(mass, velocityTangent, configurationTangent);
    Vec constrainedForce;
    Vec externalForce;
    Vec residual;
    model.ComputeExternalForce(time, 1.0, constrainedForce, externalForce);
    model.ComputeResidual(externalForce, residual);
    Eigen::SparseLU<SpMat> solver;
    solver.analyzePattern(mass);
    solver.factorize(mass);
    if (solver.info() != Eigen::Success)
        return false;
    acceleration = solver.solve(residual);
    if (solver.info() != Eigen::Success || !acceleration.allFinite())
        return false;
    model.SetTrialKinematics(velocity, acceleration);
    return true;
}
}
