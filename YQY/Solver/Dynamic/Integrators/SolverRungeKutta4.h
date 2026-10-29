/** @file SolverRungeKutta4.h @brief 显式四阶 Runge-Kutta 动力求解器。 */
#pragma once

#include "../TimeStepping/TimeStepControl.h"
#include "../../Interface/ISolver.h"

#include <Eigen/SparseLU>

#include <cstdint>

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
    const char* GetName() const override
    {
        return "Runge-Kutta 4";
    }
    SolverType GetType() const override
    {
        return parameters_.timeStepMode == TimeStepMode::Adaptive ? SolverType::AdaptiveRungeKutta4
                                                                  : SolverType::RungeKutta4;
    }
    void SetStepCallback(StepCallback callback) override
    {
        callback_ = std::move(callback);
    }
    const TimeStepStatistics& GetTimeStepStatistics() const
    {
        return timeStepStatistics_;
    }
    int GetMassPatternAnalysisCount() const
    {
        return massPatternAnalysisCount_;
    }
    int GetMassFactorizationCount() const
    {
        return massFactorizationCount_;
    }
    int GetMassFactorizationReuseCount() const
    {
        return massFactorizationReuseCount_;
    }
    int GetMassAssemblyCount() const
    {
        return massAssemblyCount_;
    }
    int GetMassAssemblyReuseCount() const
    {
        return massAssemblyReuseCount_;
    }

private:
    bool SolveSingleStep(IAnalysisModel& model, double time, double timeStep, _OUT Vec& displacement,
                         _OUT Vec& velocity, _OUT Vec& acceleration);
    bool EvaluateAcceleration(IAnalysisModel& model, double time, const Vec& stepIncrement, const Vec& velocity,
                              _OUT Vec& acceleration);
    bool ComputeConfigurationRate(const std::vector<FiniteRotationState>& rotations, const Vec& stepIncrement,
                                  const Vec& velocity, _OUT Vec& rate) const;
    double EstimateStepError(const Vec& coarseDisplacement, const Vec& coarseVelocity, const Vec& fineDisplacement,
                             const Vec& fineVelocity, const std::vector<FiniteRotationState>& coarseRotations,
                             const std::vector<FiniteRotationState>& fineRotations, int methodOrder) const;
    double EstimateAuxiliaryRotationError(const std::vector<Eigen::Matrix3d>& stepStartRotations,
                                           const std::vector<Eigen::Matrix3d>& coarseRotations,
                                           const std::vector<Eigen::Matrix3d>& fineRotations) const;

    Params parameters_;
    TimeStepStatistics timeStepStatistics_;
    StepCallback callback_;
    Eigen::SparseLU<SpMat> massSolver_;
    SpMat cachedMass_;
    std::uint64_t massPatternVersion_ = 0;
    bool massPatternValid_ = false;
    bool massFactorizationValid_ = false;
    bool massStateDependent_ = true;
    int massPatternAnalysisCount_ = 0;
    int massFactorizationCount_ = 0;
    int massFactorizationReuseCount_ = 0;
    int massAssemblyCount_ = 0;
    int massAssemblyReuseCount_ = 0;
};
}
