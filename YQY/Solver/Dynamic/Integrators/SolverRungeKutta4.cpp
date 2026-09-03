/** @file SolverRungeKutta4.cpp @brief 显式四阶 Runge-Kutta 动力求解器。 */
#include "SolverRungeKutta4.h"
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
        if (!model.PushStateCheckpoint())
            return false;
        model.DiscardStateCheckpoint();
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
    while (time < duration - 1.0e-12)
    {
        const double timeStep = std::min(nextTimeStep, duration - time);
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
            time += timeStep;
            if (callback_)
                callback_(acceptedSteps, time, displacement);
            model.OnStepCompleted(time);
            continue;
        }

        if (!model.PushStateCheckpoint())
            return false;
        Vec coarseDisplacement;
        Vec coarseVelocity;
        Vec coarseAcceleration;
        bool solved = SolveSingleStep(model, time, timeStep, coarseDisplacement, coarseVelocity, coarseAcceleration);
        if (!model.RestoreStateCheckpoint())
            return false;

        Vec halfDisplacement;
        Vec halfVelocity;
        Vec halfAcceleration;
        if (solved)
            solved = SolveSingleStep(model, time, 0.5 * timeStep, halfDisplacement, halfVelocity, halfAcceleration);
        Vec fineDisplacement;
        Vec fineVelocity;
        Vec fineAcceleration;
        if (solved)
            solved = SolveSingleStep(model, time + 0.5 * timeStep, 0.5 * timeStep, fineDisplacement, fineVelocity,
                                     fineAcceleration);

        const double error = solved ? EstimateStepDoublingError(coarseDisplacement, coarseVelocity, fineDisplacement,
                                                                  fineVelocity, parameters_.adaptiveTimeStep, 4)
                                    : std::numeric_limits<double>::infinity();
        const TimeStepDecision decision = solved ? controller->DecideAccepted(timeStep, error, 4)
                                                  : controller->DecideRejected(timeStep, error, 4);
        if (!decision.accepted)
        {
            model.RestoreStateCheckpoint();
            model.DiscardStateCheckpoint();
            if (decision.nextTimeStep >= timeStep ||
                ++rejectedAttempts > parameters_.adaptiveTimeStep.maximumRejectedAttempts)
                return false;
            nextTimeStep = decision.nextTimeStep;
            continue;
        }
        model.CommitState();
        model.DiscardStateCheckpoint();
        ++acceptedSteps;
        time += timeStep;
        nextTimeStep = decision.nextTimeStep;
        rejectedAttempts = 0;
        if (callback_)
            callback_(acceptedSteps, time, fineDisplacement);
        model.OnStepCompleted(time);
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

    displacement = initialDisplacement + timeStep * initialVelocity +
                   timeStep * timeStep * (a1 + a2 + a3) / 6.0;
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
