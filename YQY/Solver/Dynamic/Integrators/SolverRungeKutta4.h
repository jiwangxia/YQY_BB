/** @file SolverRungeKutta4.h @brief 显式四阶 Runge-Kutta 动力求解器。 */
#pragma once

#include "../TimeStepping/TimeStepControl.h"
#include "../../Interface/ISolver.h"

namespace SolverNameSpace
{
class SolverRungeKutta4 final : public ISolver
{
public:
    struct Params
    {
        double timeStep = 0.01;
        TimeStepMode timeStepMode = TimeStepMode::Fixed;
        AdaptiveTimeStepSettings adaptiveTimeStep;
    };

    explicit SolverRungeKutta4(Params parameters);
    bool Solve(IAnalysisModel& model, double duration) override;
    const char* GetName() const override { return "Runge-Kutta 4"; }
    SolverType GetType() const override
    {
        return parameters_.timeStepMode == TimeStepMode::Adaptive ? SolverType::AdaptiveRungeKutta4
                                                                    : SolverType::RungeKutta4;
    }
    void SetStepCallback(StepCallback callback) override { callback_ = std::move(callback); }

private:
    bool SolveSingleStep(IAnalysisModel& model, double time, double timeStep, _OUT Vec& displacement,
                         _OUT Vec& velocity, _OUT Vec& acceleration);
    bool EvaluateAcceleration(IAnalysisModel& model, double time, const Vec& initialDisplacement,
                              const Vec& displacement, const Vec& velocity, _OUT Vec& acceleration);

    Params parameters_;
    StepCallback callback_;
};
}
