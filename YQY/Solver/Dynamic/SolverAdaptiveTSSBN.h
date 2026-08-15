/**
 * @file SolverAdaptiveTSSBN.h
 * @brief 用于非线性动力学的自适应 rho∞-TSSBN 积分器。
 */
#pragma once

#include "AdaptiveTssbnSettings.h"
#include "../Interface/ISolver.h"
#include "Solver/LinearSystemSolver.h"
#include <limits>

namespace SolverNameSpace
{
class SolverAdaptiveTSSBN final : public ISolver
{
public:
    struct Params : AdaptiveTssbnSettings
    {
        double initialTimeStep = 0.01;
        double nonlinearTolerance = 1.0e-6;
        int maximumNewtonIterations = 20;
        AerodynamicTangentMode aerodynamicTangentMode = AerodynamicTangentMode::EveryNewtonIteration;

        Params() = default;
        Params(double stepSize, double equilibriumTolerance, int newtonIterationLimit,
               const AdaptiveTssbnSettings& adaptiveSettings)
            : AdaptiveTssbnSettings(adaptiveSettings)
            , initialTimeStep(stepSize)
            , nonlinearTolerance(equilibriumTolerance)
            , maximumNewtonIterations(newtonIterationLimit)
        {
        }
    };

    explicit SolverAdaptiveTSSBN(Params parameters);

    bool Solve(IAnalysisModel& model, double duration) override;
    const char* GetName() const override
    {
        return "Adaptive-TSSBN";
    }
    SolverType GetType() const override
    {
        return SolverType::AdaptiveTSSBN;
    }
    void SetStepCallback(StepCallback callback) override
    {
        stepCallback_ = std::move(callback);
    }

    const Params& GetParams() const
    {
        return parameters_;
    }
    int GetAcceptedStepCount() const
    {
        return acceptedStepCount_;
    }
    int GetRejectedStepCount() const
    {
        return rejectedStepCount_;
    }
    double GetAverageTimeStep() const
    {
        return averageTimeStep_;
    }
    double GetAverageStageNewtonIterations() const
    {
        return acceptedStepCount_ > 0
                   ? static_cast<double>(totalStageNewtonIterations_) / static_cast<double>(acceptedStepCount_)
                   : 0.0;
    }
    int GetMaximumStageNewtonIterations() const
    {
        return maximumStageNewtonIterations_;
    }

private:
    struct BaseMethodCoefficients
    {
        double firstStageTime = 0.0;
        double secondStageTime = 0.0;
        double secondStageDiagonalFraction = 0.0;
        double firstStageWeight = 0.0;
    };

    struct EmbeddedMethodCoefficients
    {
        double lastStageTime = 1.0;
        double firstWeight = 0.0;
        double secondWeight = 0.0;
        double lastWeight = 0.0;
        double lastStageFirstCoefficient = 0.0;
        double lastStageSecondCoefficient = 0.0;
    };

    struct StageState
    {
        Vec stepIncrement;
        Vec velocity;
        Vec acceleration;
        int newtonIterations = 0;
    };

    struct ImplicitStageDefinition
    {
        int stageIndex = 0;
        double timeFraction = 0.0;
        double diagonalCoefficient = 0.0;
        Vec knownDisplacementRate;
        Vec knownVelocityRate;
        Vec initialVelocity;
    };

    Params parameters_;
    BaseMethodCoefficients baseCoefficients_;
    EmbeddedMethodCoefficients embeddedCoefficients_;
    StepCallback stepCallback_;

    int acceptedStepCount_ = 0;
    int rejectedStepCount_ = 0;
    long long totalStageNewtonIterations_ = 0;
    int maximumStageNewtonIterations_ = 0;
    double averageTimeStep_ = 0.0;
    double lastResidualNorm_ = std::numeric_limits<double>::quiet_NaN();
    double lastResidualLimit_ = std::numeric_limits<double>::quiet_NaN();
    QString lastStageFailureReason_;

    SpMat tangentStiffness_;
    SpMat dynamicTangent_;
    SpMat effectiveTangent_;
    SpMat externalTangent_;
    Vec residual_;
    Vec correction_;

    LinearSystemSolver effectiveSystemSolver_;
    void ValidateParameters() const;
    void ComputeMethodCoefficients();

    ImplicitStageDefinition CreateFirstStageDefinition(int dofCount) const;
    ImplicitStageDefinition CreateSecondStageDefinition(double timeStep, const StageState& firstStage) const;

    bool SolveImplicitStage(IAnalysisModel& model, double stepStartTime, double timeStep, const Vec& previousVelocity,
                            const ImplicitStageDefinition& definition, _OUT StageState& stage);

    bool EvaluateExplicitLastStage(double timeStep, const Vec& previousVelocity, const StageState& firstStage,
                                   const StageState& secondStage, _OUT StageState& lastStage);

    void EstablishAcceptedState(IAnalysisModel& model, _OUT Vec& stepIncrement, const Vec& velocity,
                                const Vec& acceleration);

    double EstimateNormalizedError(const Vec& baseIncrement, const Vec& embeddedIncrement, const Vec& baseVelocity,
                                   const Vec& embeddedVelocity, const Vec& previousDisplacement,
                                   const Vec& previousVelocity) const;

    double ProposeAcceptedTimeStep(double currentTimeStep, double normalizedError, int totalStageIterations);

    double ProposeRejectedTimeStep(double currentTimeStep, double normalizedError) const;
};
}
