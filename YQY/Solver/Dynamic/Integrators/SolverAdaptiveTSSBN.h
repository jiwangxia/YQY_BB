/**
 * @file SolverAdaptiveTSSBN.h
 * @brief 用于非线性动力学的自适应 rho∞-TSSBN 积分器。
 */
#pragma once

#include "../TimeStepping/AdaptiveTssbnSettings.h"
#include "../../Interface/ISolver.h"
#include "Solver/Linear/LinearSystemSolver.h"
#include <Eigen/SparseLU>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace SolverNameSpace
{
struct NonlinearMPCData;
struct NonlinearMPCReduction;
struct FiniteRotationState;

class SolverAdaptiveTSSBN final : public ISolver
{
public:
    struct CheckpointState
    {
        double time = 0.0;
        double nextTimeStep = 0.0;
        double previousAcceptedError = 1.0;
        double previousAcceptedTimeStep = 0.0;
        double acceptedTimeStepSum = 0.0;
        int acceptedStepCount = 0;
        int rejectedStepCount = 0;
        long long totalStageNewtonIterations = 0;
        int maximumStageNewtonIterations = 0;
    };
    using CheckpointWriter = std::function<bool(const CheckpointState&, _OUT QString&)>;

    struct Params : AdaptiveTssbnSettings
    {
        double initialTimeStep = 0.01;
        double nonlinearTolerance = 1.0e-6;
        int maximumNewtonIterations = 20;
        AerodynamicTangentMode aerodynamicTangentMode = AerodynamicTangentMode::EveryNewtonIteration;
        bool diagnosticCacheExternalForcePerStage = false;
        bool diagnosticErrorTrace = false;
        bool referenceGalloping = false;

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
        return parameters_.referenceGalloping ? "Reference-TSSBN-Galloping" : "Adaptive-TSSBN";
    }
    SolverType GetType() const override
    {
        return parameters_.referenceGalloping ? SolverType::ReferenceTSSBN : SolverType::AdaptiveTSSBN;
    }
    void SetStepCallback(StepCallback callback) override
    {
        stepCallback_ = std::move(callback);
    }
    void SetResumeState(const CheckpointState& state)
    {
        resumeState_ = state;
    }
    void SetCheckpointWriter(double targetTime, bool stopAfterWrite, CheckpointWriter writer)
    {
        checkpointTargetTime_ = targetTime;
        stopAfterCheckpoint_ = stopAfterWrite;
        checkpointWriter_ = std::move(writer);
    }
    bool WasStoppedAtCheckpoint() const
    {
        return stoppedAtCheckpoint_;
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
    long long GetLocalResidualDelayCount() const
    {
        return diagnosticLocalResidualDelayCount_;
    }
    long long GetPrecisionLimitedStageCount() const
    {
        return precisionLimitedStageCount_;
    }
    long long GetPrecisionCandidateSolveCount() const
    {
        return precisionCandidateSolveCount_;
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
        Vec finalResidual;
        double finalResidualNorm = std::numeric_limits<double>::quiet_NaN();
        double finalResidualLimit = std::numeric_limits<double>::quiet_NaN();
        int newtonIterations = 0;
    };

    struct StepErrorState
    {
        Vec displacement;
        Vec velocity;
        Vec acceleration;
        std::vector<FiniteRotationState> rotations;
    };

    struct ImplicitStageDefinition
    {
        int stageIndex = 0;
        double timeFraction = 0.0;
        double diagonalCoefficient = 0.0;
        Vec knownDisplacementRate;
        Vec knownVelocityRate;
        Vec initialVelocity;
        const StageState* previousStage = nullptr;
    };

    Params parameters_;
    double previousAcceptedError_ = 1.0;
    BaseMethodCoefficients baseCoefficients_;
    EmbeddedMethodCoefficients embeddedCoefficients_;
    Eigen::Vector3d velocityQuadratureWeights_ = Eigen::Vector3d::Zero();
    StepCallback stepCallback_;
    std::optional<CheckpointState> resumeState_;
    CheckpointWriter checkpointWriter_;
    double checkpointTargetTime_ = std::numeric_limits<double>::quiet_NaN();
    bool stopAfterCheckpoint_ = false;
    bool stoppedAtCheckpoint_ = false;
    bool checkpointWritten_ = false;
    bool referenceStagePermutationReady_ = false;

    int acceptedStepCount_ = 0;
    int rejectedStepCount_ = 0;
    long long totalStageNewtonIterations_ = 0;
    int maximumStageNewtonIterations_ = 0;
    long long diagnosticExternalForceAuditCount_ = 0;
    long long diagnosticExternalForceAuditMismatchCount_ = 0;
    long long explicitLastStageEvaluationCount_ = 0;
    long long explicitLastStageMassSolveCount_ = 0;
    long long acceptedEndpointEvaluationCount_ = 0;
    long long acceptedEndpointMassSolveCount_ = 0;
    long long acceptedEndpointMpcSolveCount_ = 0;
    long long explicitLastStageDisplacementSkipCount_ = 0;
    long long diagnosticAcceptedVelocityDominantCount_ = 0;
    long long diagnosticAcceptedDisplacementDominantCount_ = 0;
    long long diagnosticStageEvaluationCount_ = 0;
    long long diagnosticStageLinearSolveCount_ = 0;
    long long diagnosticFusedResidualTangentCount_ = 0;
    long long diagnosticReusedTangentCount_ = 0;
    long long diagnosticFirstStageNewtonIterations_ = 0;
    long long diagnosticSecondStageNewtonIterations_ = 0;
    long long diagnosticLocalResidualDelayCount_ = 0;
    double diagnosticMaximumLocalResidual_ = 0.0;
    long long precisionLimitedStageCount_ = 0;
    long long precisionCandidateSolveCount_ = 0;
    double precisionLimitedMaximumResidual_ = 0.0;
    double precisionLimitedMaximumScaledResidual_ = 0.0;
    double precisionLimitedMaximumCorrection_ = 0.0;
    double diagnosticAcceptedAttemptMilliseconds_ = 0.0;
    double diagnosticRejectedAttemptMilliseconds_ = 0.0;
    double diagnosticFirstStageMilliseconds_ = 0.0;
    double diagnosticSecondStageMilliseconds_ = 0.0;
    double diagnosticStageEvaluationMilliseconds_ = 0.0;
    double diagnosticStageLinearSolveMilliseconds_ = 0.0;
    double diagnosticY3Milliseconds_ = 0.0;
    double diagnosticY3EvaluationMilliseconds_ = 0.0;
    double diagnosticY3MassSolveMilliseconds_ = 0.0;
    double diagnosticAcceptedEndpointMilliseconds_ = 0.0;
    double diagnosticAcceptedTailMilliseconds_ = 0.0;
    double averageTimeStep_ = 0.0;
    double lastResidualNorm_ = std::numeric_limits<double>::quiet_NaN();
    double lastResidualLimit_ = std::numeric_limits<double>::quiet_NaN();
    QString lastStageFailureReason_;
    QString diagnosticNewtonDirectory_;
    double diagnosticNewtonStartTime_ = 0.0;
    long long diagnosticNewtonSequence_ = 0;

    SpMat effectiveTangent_;
    SpMat externalTangent_;
    Vec residual_;
    Vec correction_;

    LinearSystemSolver effectiveSystemSolver_;
    LinearSystemSolver acceptedMpcAccelerationSolver_;
    Eigen::SparseLU<SpMat> explicitMassSolver_;
    SpMat cachedExplicitMass_;
    std::uint64_t explicitMassPatternVersion_ = 0;
    bool explicitMassPatternValid_ = false;
    bool explicitMassFactorizationValid_ = false;
    bool explicitMassStateDependent_ = true;
    void ValidateParameters() const;
    void ComputeMethodCoefficients();
    void ApplyTrialIncrement(IAnalysisModel& model, const Vec& increment) const;
    void SetTrialKinematics(IAnalysisModel& model, const Vec& velocity, const Vec& acceleration) const;

    ImplicitStageDefinition CreateFirstStageDefinition(int dofCount) const;
    ImplicitStageDefinition CreateSecondStageDefinition(double timeStep, const StageState& firstStage) const;

    bool SolveImplicitStage(IAnalysisModel& model, double stepStartTime, double timeStep, const Vec& previousVelocity,
                            const ImplicitStageDefinition& definition,
                            _OUT StageState& stage);
    double EstimateScaledStageResidual(const Vec& rhs, const Vec& externalForce, const SpMat& tangent,
                                       const Vec& stepIncrement, const NonlinearMPCReduction* reduction) const;
    double EstimatePrecisionVelocityError(const IAnalysisModel& model, const Vec& correction, const Vec& stepIncrement,
                                          const Vec& previousVelocity, const Vec& velocity,
                                          double velocityDerivative) const;
    void TraceNewtonPrecisionSystem(const IAnalysisModel& model, double stepStartTime, double timeStep,
                                    const ImplicitStageDefinition& definition, int iteration, const StageState& stage,
                                    const Vec& previousVelocity, const Vec& rhs, const Vec& externalForce,
                                    const SpMat& tangent, double velocityDerivative, const Vec& independentCorrection,
                                    const NonlinearMPCReduction* reduction, bool precisionCandidate,
                                    bool precisionConverged);

    bool EvaluateExplicitLastStage(double timeStep, const Vec& previousVelocity, const StageState& firstStage,
                                   const StageState& secondStage, _OUT StageState& lastStage);
    bool SolveExplicitLastStageAcceleration(IAnalysisModel& model, double stageTime, StageState& lastStage,
                                            _OUT Vec& acceleration, bool acceptedEndpoint = false);

    bool EstablishAcceptedState(IAnalysisModel& model, _OUT Vec& stepIncrement, const Vec& velocity,
                                const Vec& acceleration);
    bool RecoverAcceptedMpcAcceleration(IAnalysisModel& model, double time, const Vec& velocity,
                                        _OUT Vec& acceleration);
    bool ProjectAcceptedMpcState(IAnalysisModel& model, _OUT Vec& stepIncrement, _OUT Vec& velocity,
                                 _OUT Vec& acceleration);
    static bool ProjectMpcKinematics(const NonlinearMPCData& constraints, _OUT Vec& velocity, _OUT Vec& acceleration);
    static bool SolveMpcSlaveCorrection(const NonlinearMPCData& constraints, const Vec& rightHandSide,
                                        _OUT Vec& correction);

    double EstimateErrorComponent(const IAnalysisModel& model, const Vec& difference, const Vec& referenceA,
                                  const Vec& referenceB, bool velocity = false) const;
    double EstimateNormalizedError(const IAnalysisModel& model, const Vec& baseIncrement, const Vec& embeddedIncrement,
                                   const Vec& baseVelocity, const Vec& embeddedVelocity,
                                   const Vec& previousDisplacement, const Vec& previousVelocity,
                                   const Vec* displacementDifferenceOverride = nullptr,
                                   const Vec* velocityDifferenceOverride = nullptr) const;

    bool SolveBaseStepForErrorCheck(IAnalysisModel& model, double startTime, double timeStep,
                                    _OUT int& stageIterations);
    bool ComputeStepDoublingError(const IAnalysisModel& model, const StepErrorState& coarseState,
                                  const StepErrorState& fineState, const Vec& previousDisplacement,
                                  const Vec& previousVelocity, _OUT double& normalizedError,
                                  _OUT double& displacementDiagnosticError, _OUT double& velocityDiagnosticError,
                                  _OUT int& displacementWorstDof, _OUT int& velocityWorstDof);
    bool EstimateFiniteRotationStepError(IAnalysisModel& model, double startTime, double timeStep,
                                         const Vec& previousDisplacement, const Vec& previousVelocity,
                                         _OUT double& normalizedError, _OUT int& stageIterations,
                                         _OUT bool& stateRestored, _OUT bool& fineStateAccepted,
                                         _OUT double& displacementDiagnosticError, _OUT double& velocityDiagnosticError,
                                         _OUT int& displacementWorstDof, _OUT int& velocityWorstDof);

    double ProposeAcceptedTimeStep(double currentTimeStep, double normalizedError, int totalStageIterations);

    double ProposeRejectedTimeStep(double currentTimeStep, double normalizedError) const;
};
}
