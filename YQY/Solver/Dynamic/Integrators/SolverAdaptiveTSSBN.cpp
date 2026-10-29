/**
 * @file SolverAdaptiveTSSBN.cpp
 * @brief 用于非线性动力学的自适应 rho∞-TSSBN 积分器。
 */
#include "SolverAdaptiveTSSBN.h"
#include "Solver/Constraint/NonlinearMPC.h"
#include "Solver/Dynamic/TimeStepping/NewtonPrecisionScale.h"
#include "Solver/Dynamic/TimeStepping/TssbnRotationPrecision.h"
#include "Solver/Dynamic/TimeStepping/TssbnLinearPrecision.h"
#include "Solver/Dynamic/TimeStepping/StateCheckpointGuard.h"
#include "Utility/CR.h"
#include "Utility/Logger/Logger.h"

#include <Eigen/Dense>
#include <QDebug>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

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
                        double relativeTolerance, TssbnErrorNormMode mode, bool velocity = false)
{
    if (difference.size() != referenceA.size() || difference.size() != referenceB.size() || !difference.allFinite() ||
        !referenceA.allFinite() || !referenceB.allFinite())
        return std::numeric_limits<double>::infinity();
    if (difference.size() == 0)
        return 0.0;
    if (mode == TssbnErrorNormMode::LegacyGlobal || mode == TssbnErrorNormMode::GlobalMax ||
        (!velocity && mode == TssbnErrorNormMode::DisplacementGlobalVelocityMax))
    {
        const double scale = absoluteTolerance + relativeTolerance * std::max(referenceA.norm(), referenceB.norm());
        return std::isfinite(scale) && scale > 0.0 ? difference.norm() / scale
                                                   : std::numeric_limits<double>::infinity();
    }

    // 各自由度独立缩放，避免静挠度或其他节点的位移放宽当前自由度的局部误差。
    double maximumError = 0.0;
    double sumSquaredError = 0.0;
    for (Eigen::Index i = 0; i < difference.size(); ++i)
    {
        const double magnitude = mode == TssbnErrorNormMode::PaperMixedRms
                                     ? std::abs(referenceB[i])
                                     : std::max(std::abs(referenceA[i]), std::abs(referenceB[i]));
        const double scale = absoluteTolerance + relativeTolerance * magnitude;
        if (!std::isfinite(scale) || scale <= 0.0)
            return std::numeric_limits<double>::infinity();
        const double componentError = std::abs(difference[i]) / scale;
        maximumError = std::max(maximumError, componentError);
        if ((velocity && mode == TssbnErrorNormMode::DisplacementMaxVelocityRms) ||
            mode == TssbnErrorNormMode::PaperMixedRms)
            sumSquaredError += componentError * componentError;
    }
    if (mode == TssbnErrorNormMode::PaperMixedRms)
        return std::sqrt(sumSquaredError / static_cast<double>(difference.size()));
    if (velocity && mode == TssbnErrorNormMode::DisplacementMaxVelocityRms)
        return std::sqrt(sumSquaredError / static_cast<double>(difference.size()));
    return maximumError;
}

struct ScaledErrorDiagnostic
{
    double normalizedError = std::numeric_limits<double>::infinity();
    Eigen::Index worstDof = -1;
    double difference = std::numeric_limits<double>::quiet_NaN();
    double scale = std::numeric_limits<double>::quiet_NaN();
};

// 按当前误差范数记录总误差，并定位贡献最大的自由度。
ScaledErrorDiagnostic DiagnoseScaledError(const Vec& difference, const Vec& referenceA, const Vec& referenceB,
                                          double absoluteTolerance, double relativeTolerance, TssbnErrorNormMode mode,
                                          bool velocity = false)
{
    ScaledErrorDiagnostic result;
    result.normalizedError =
        ScaledVectorNorm(difference, referenceA, referenceB, absoluteTolerance, relativeTolerance, mode, velocity);
    if (difference.size() == 0 || difference.size() != referenceA.size() || difference.size() != referenceB.size() ||
        !difference.allFinite() || !referenceA.allFinite() || !referenceB.allFinite())
        return result;

    const bool useGlobalScale = mode == TssbnErrorNormMode::LegacyGlobal || mode == TssbnErrorNormMode::GlobalMax ||
                               (!velocity && mode == TssbnErrorNormMode::DisplacementGlobalVelocityMax);
    const double globalScale =
        useGlobalScale ? absoluteTolerance + relativeTolerance * std::max(referenceA.norm(), referenceB.norm()) : 0.0;
    double worstContribution = -1.0;
    for (Eigen::Index i = 0; i < difference.size(); ++i)
    {
        const double scale = useGlobalScale
                                 ? globalScale
                                 : absoluteTolerance + relativeTolerance * (mode == TssbnErrorNormMode::PaperMixedRms
                                                                                ? std::abs(referenceB[i])
                                                                                : std::max(std::abs(referenceA[i]),
                                                                                           std::abs(referenceB[i])));
        if (!std::isfinite(scale) || scale <= 0.0)
            return result;
        const double contribution = std::abs(difference[i]) / scale;
        if (contribution > worstContribution)
        {
            worstContribution = contribution;
            result.worstDof = i;
            result.difference = difference[i];
            result.scale = scale;
        }
    }
    return result;
}

QString FormatScaledErrorDiagnostic(const QString& label, const ScaledErrorDiagnostic& diagnostic,
                                    const IAnalysisModel& model)
{
    if (diagnostic.worstDof < 0)
        return QStringLiteral("%1=%2（无可定位自由度）").arg(label).arg(diagnostic.normalizedError, 0, 'g', 10);
    return QStringLiteral("%1=%2（%3，差值=%4，尺度=%5）")
        .arg(label)
        .arg(diagnostic.normalizedError, 0, 'g', 10)
        .arg(model.DescribeFreeDof(static_cast<int>(diagnostic.worstDof)))
        .arg(diagnostic.difference, 0, 'g', 10)
        .arg(diagnostic.scale, 0, 'g', 10);
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
        !IsFinitePositive(parameters_.velocityAbsoluteTolerance) || !IsFinitePositive(parameters_.nonlinearTolerance) ||
        parameters_.maximumNewtonIterations < 1 || parameters_.targetNewtonIterations < 1 ||
        parameters_.maximumTotalNewtonIterations < 1 || parameters_.maximumRejectedAttempts < 1)
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
    if (parameters_.errorNormMode != TssbnErrorNormMode::ComponentwiseMax &&
        parameters_.errorNormMode != TssbnErrorNormMode::LegacyGlobal &&
        parameters_.errorNormMode != TssbnErrorNormMode::DisplacementMaxVelocityRms &&
        parameters_.errorNormMode != TssbnErrorNormMode::PaperMixedRms &&
        parameters_.errorNormMode != TssbnErrorNormMode::DisplacementGlobalVelocityMax &&
        parameters_.errorNormMode != TssbnErrorNormMode::GlobalMax)
        throw std::invalid_argument("自适应 TSSBN 误差范数模式无效");
    if (parameters_.diagnosticCacheExternalForcePerStage &&
        parameters_.aerodynamicTangentMode != AerodynamicTangentMode::Disabled)
    {
        throw std::invalid_argument("阶段外载缓存只能与关闭气动切线联合使用");
    }
}

void SolverAdaptiveTSSBN::ComputeMethodCoefficients()
{
    const double spectralRadius = parameters_.spectralRadiusInfinity;
    if (!parameters_.referenceGalloping)
    {
        // 有理化后在 rho=1 及其邻域也保持精度，避免两个接近数相减后再相除。
        const double gamma = 1.0 / (2.0 + std::sqrt(2.0 * (1.0 + spectralRadius)));
        const double secondTime = 1.0 - gamma;
        const double separation = 1.0 - 2.0 * gamma;
        baseCoefficients_.firstStageTime = gamma;
        baseCoefficients_.secondStageTime = secondTime;
        baseCoefficients_.secondStageDiagonalFraction = gamma / secondTime;
        baseCoefficients_.firstStageWeight = 0.5;
        embeddedCoefficients_.firstWeight = (2.0 - 3.0 * gamma) / (6.0 * separation * secondTime);
        embeddedCoefficients_.secondWeight = (1.0 - 3.0 * gamma) / (6.0 * gamma * separation);
        embeddedCoefficients_.lastWeight = 1.0 - 1.0 / (6.0 * gamma * secondTime);
        // 三阶条件在整个参数区间给出 a31=1、a32=0；末阶段加速度的取法由误差策略决定。
        embeddedCoefficients_.lastStageFirstCoefficient = 1.0;
        embeddedCoefficients_.lastStageSecondCoefficient = 0.0;
        velocityQuadratureWeights_ << embeddedCoefficients_.lastWeight, embeddedCoefficients_.secondWeight,
            embeddedCoefficients_.firstWeight;
        Logger::Instance().Diagnostic(QStringLiteral("自适应 TSSBN 系数：c1=%1，c2=%2，alpha=%3，b1=%4")
                                          .arg(gamma, 0, 'g', 10)
                                          .arg(secondTime, 0, 'g', 10)
                                          .arg(baseCoefficients_.secondStageDiagonalFraction, 0, 'g', 10)
                                          .arg(baseCoefficients_.firstStageWeight, 0, 'g', 10));
        return;
    }
    const bool limitingSpectralRadius = parameters_.referenceGalloping
                                            ? std::abs(spectralRadius - 1.0) < 1.0e-9
                                            : std::abs(spectralRadius - 1.0) <= coefficientTolerance;
    if (limitingSpectralRadius)
    {
        baseCoefficients_.firstStageTime = 0.25;
        baseCoefficients_.secondStageTime = 0.75;
        baseCoefficients_.secondStageDiagonalFraction = 1.0 / 3.0;
        baseCoefficients_.firstStageWeight = 0.5;
    }
    else
    {
        const double root = parameters_.referenceGalloping ? std::sqrt(2.0) * std::sqrt(spectralRadius + 1.0)
                                                           : std::sqrt(2.0 * (spectralRadius + 1.0));
        const double firstStageTime = (root - 2.0) / (2.0 * (spectralRadius - 1.0));
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

    // 默认的非 MPC 工况随后会重新求解真实 Y3 加速度；MPC 暂沿用步初加速度的三点求积探针。
    Eigen::Matrix3d velocityConditions;
    velocityConditions << 1.0, 1.0, 1.0, 0.0, baseCoefficients_.firstStageTime, baseCoefficients_.secondStageTime, 0.0,
        baseCoefficients_.firstStageTime * baseCoefficients_.firstStageTime,
        baseCoefficients_.secondStageTime * baseCoefficients_.secondStageTime;
    velocityQuadratureWeights_ = velocityConditions.colPivHouseholderQr().solve(requiredMoments);
    if (!velocityQuadratureWeights_.allFinite() ||
        (velocityConditions * velocityQuadratureWeights_ - requiredMoments).norm() > 1.0e-10)
        throw std::runtime_error("自适应 TSSBN 无法计算速度误差求积权重");

    Logger::Instance().Diagnostic(QStringLiteral("自适应 TSSBN 系数：c1=%1，c2=%2，alpha=%3，b1=%4")
                                      .arg(baseCoefficients_.firstStageTime, 0, 'g', 10)
                                      .arg(baseCoefficients_.secondStageTime, 0, 'g', 10)
                                      .arg(baseCoefficients_.secondStageDiagonalFraction, 0, 'g', 10)
                                      .arg(baseCoefficients_.firstStageWeight, 0, 'g', 10));
}

double SolverAdaptiveTSSBN::EstimateScaledStageResidual(const Vec& rhs, const Vec& externalForce, const SpMat& tangent,
                                                        const Vec& stepIncrement,
                                                        const NonlinearMPCReduction* reduction) const
{
    const Vec independentExternal =
        reduction ? Vec(reduction->masterTransformation.transpose() * externalForce) : externalForce;
    if (rhs.size() == 0 || rhs.size() != independentExternal.size() || tangent.rows() != rhs.size() ||
        tangent.cols() != rhs.size() || !rhs.allFinite() ||
        !independentExternal.allFinite() || !stepIncrement.allFinite())
        return std::numeric_limits<double>::infinity();

    double maximum = 0.0;
    for (Eigen::Index index = 0; index < rhs.size(); ++index)
    {
        const Eigen::Index fullDof = reduction ? reduction->masterDofs[static_cast<std::size_t>(index)] : index;
        if (fullDof < 0 || fullDof >= stepIncrement.size())
            return std::numeric_limits<double>::infinity();
        const double tangentScale = std::abs(tangent.coeff(index, index));
        const double coordinateScale = std::max(1.0, std::abs(stepIncrement[fullDof]));
        const double scale =
            std::max({1.0e-12, std::abs(independentExternal[index]), std::abs(independentExternal[index] - rhs[index]),
                      tangentScale * coordinateScale});
        if (!std::isfinite(scale))
            return std::numeric_limits<double>::infinity();
        maximum = std::max(maximum, std::abs(rhs[index]) / scale);
    }
    return maximum;
}

double SolverAdaptiveTSSBN::EstimatePrecisionVelocityError(const IAnalysisModel& model, const Vec& correction,
                                                           const Vec& stepIncrement, const Vec& previousVelocity,
                                                           const Vec& velocity, double velocityDerivative) const
{
    if (correction.size() != stepIncrement.size() || correction.size() != velocity.size() ||
        correction.size() != previousVelocity.size() || !correction.allFinite() || !stepIncrement.allFinite() ||
        !velocity.allFinite() || !previousVelocity.allFinite())
        return std::numeric_limits<double>::infinity();
    Vec velocityCorrection = (velocityDerivative * correction).cwiseAbs();
    std::vector<FiniteRotationState> rotations;
    model.GetFiniteRotationStates(rotations);
    const bool useStageMapping = !parameters_.referenceGalloping && model.SupportsTssbnRotationPrecisionEstimate();
    Vec currentStepIncrement;
    if (useStageMapping && !rotations.empty())
    {
        // Node 的阶段刷新可能重建步内主转角，预算须使用同一当前状态而非先前缓存。
        model.GetStepIncrement(currentStepIncrement);
        if (currentStepIncrement.size() != correction.size() || !currentStepIncrement.allFinite())
            return std::numeric_limits<double>::infinity();
    }
    for (const auto& rotation : rotations)
    {
        Eigen::Vector3d angularIncrement = Eigen::Vector3d::Zero();
        Eigen::Vector3d angularCorrection = Eigen::Vector3d::Zero();
        Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
        int activeComponents = 0;
        for (int component = 0; component < 3; ++component)
        {
            const int dof = rotation.freeDofs[component];
            if (dof < 0)
                continue;
            if (dof >= correction.size())
                return std::numeric_limits<double>::infinity();
            ++activeComponents;
            angularIncrement[component] = useStageMapping ? currentStepIncrement[dof] : stepIncrement[dof];
            angularCorrection[component] = correction[dof];
            angularVelocity[component] = velocity[dof];
        }
        if (activeComponents == 0)
            continue;
        // 当前接口未返回固定转动分量的实际角速度，部分转动固定时不使用精度受限停止。
        if (activeComponents != 3 || angularIncrement.norm() >= std::acos(-1.0) - 1.0e-6)
            return std::numeric_limits<double>::infinity();
        if (useStageMapping)
        {
            TssbnRotationPrecisionEstimate estimate;
            if (!EstimateTssbnRotationVelocityCorrection(angularIncrement, angularCorrection, angularVelocity,
                                                        velocityDerivative, estimate))
                return std::numeric_limits<double>::infinity();
            for (int component = 0; component < 3; ++component)
                velocityCorrection[rotation.freeDofs[component]] = estimate.absoluteVelocityCorrectionBound[component];
            continue;
        }
        // 主步转角小于 pi 时，log 微分算子范数不超过 pi/2；保守取 2，并计入姿态旋转角速度项。
        const double angularVelocityBound =
            (2.0 * std::abs(velocityDerivative) + angularVelocity.norm()) * angularCorrection.norm();
        for (int dof : rotation.freeDofs)
            velocityCorrection[dof] = angularVelocityBound;
    }
    return EstimateErrorComponent(model, velocityCorrection, previousVelocity, velocity, true);
}

void SolverAdaptiveTSSBN::TraceNewtonPrecisionSystem(
    const IAnalysisModel& model, double stepStartTime, double timeStep, const ImplicitStageDefinition& definition,
    int iteration, const StageState& stage, const Vec& previousVelocity, const Vec& rhs, const Vec& externalForce,
    const SpMat& tangent, double velocityDerivative,
    const Vec& independentCorrection, const NonlinearMPCReduction* reduction, bool precisionCandidate,
    bool precisionConverged)
{
    // 只导出同一当前切线/RHS 的实际完整修正，不额外组装系统，不改变求解或接受条件。
    if (diagnosticNewtonDirectory_.isEmpty() || stepStartTime < diagnosticNewtonStartTime_ ||
        iteration < parameters_.maximumNewtonIterations - 2 || rhs.size() == 0 ||
        independentCorrection.size() != rhs.size() || tangent.rows() != rhs.size() ||
        tangent.cols() != rhs.size())
        return;
    Vec coordinateScale(rhs.size());
    for (Eigen::Index index = 0; index < rhs.size(); ++index)
    {
        const Eigen::Index fullDof = reduction ? reduction->masterDofs[index] : index;
        coordinateScale[index] = std::max(1.0, std::abs(stage.stepIncrement[fullDof]));
    }
    const Vec independentExternal = reduction ? (reduction->masterTransformation.transpose() * externalForce).eval()
                                             : externalForce;
    Vec rowScale = tangent.cwiseAbs() * coordinateScale;
    Vec correctionProduct = tangent * independentCorrection;
    Vec backwardScale = tangent.cwiseAbs() * independentCorrection.cwiseAbs() + rhs.cwiseAbs();
    const Vec linearResidual = rhs - correctionProduct;
    double fullRowResidual = 0.0;
    double backwardError = 0.0;
    double scaledCorrection = 0.0;
    Eigen::Index worstRow = 0;
    for (Eigen::Index index = 0; index < rhs.size(); ++index)
    {
        rowScale[index] = std::max({1.0e-12, rowScale[index], std::abs(independentExternal[index]),
                                   std::abs(independentExternal[index] - rhs[index])});
        const double component = std::abs(rhs[index]) / rowScale[index];
        if (component > fullRowResidual)
        {
            fullRowResidual = component;
            worstRow = index;
        }
        backwardError = std::max(backwardError, std::abs(linearResidual[index]) /
                                                  std::max(std::numeric_limits<double>::min(), backwardScale[index]));
    }
    for (Eigen::Index index = 0; index < correction_.size(); ++index)
        scaledCorrection = std::max(scaledCorrection, std::abs(correction_[index]) /
                                                        std::max(1.0, std::abs(stage.stepIncrement[index])));
    const double diagonalResidual = EstimateScaledStageResidual(rhs, externalForce, tangent,
                                                                stage.stepIncrement, reduction);
    QFile iterations(QDir(diagnosticNewtonDirectory_).filePath(QStringLiteral("iterations.csv")));
    if (!iterations.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
        return;
    QTextStream stream(&iterations);
    stream.setRealNumberPrecision(17);
    if (iterations.size() == 0)
        stream << "sequence,step_start,time_step,stage,iteration,residual_norm,diagonal_scaled_residual,"
                  "full_row_scaled_residual,scaled_full_correction,correction_displacement_lte,"
                  "correction_velocity_lte,linear_backward_error,worst_row,worst_rhs,worst_row_scale,"
                  "precision_candidate,precision_converged,has_mpc,velocity_derivative\n";
    stream << ++diagnosticNewtonSequence_ << ',' << stepStartTime << ',' << timeStep << ',' << definition.stageIndex
           << ',' << iteration << ',' << rhs.norm() << ',' << diagonalResidual << ',' << fullRowResidual << ','
           << scaledCorrection << ','
           << EstimateErrorComponent(model, correction_, stage.stepIncrement, stage.stepIncrement) << ','
           << EstimatePrecisionVelocityError(model, correction_, stage.stepIncrement, previousVelocity, stage.velocity,
                                             velocityDerivative)
           << ',' << backwardError << ',' << worstRow << ',' << rhs[worstRow] << ',' << rowScale[worstRow] << ','
           << precisionCandidate << ',' << precisionConverged << ',' << (reduction != nullptr) << ','
           << velocityDerivative << '\n';
    if (iteration < parameters_.maximumNewtonIterations)
        return;

    // 最后一个达到迭代上限的当前系统单独保存，允许高精度独立复算。
    QFile vectors(QDir(diagnosticNewtonDirectory_).filePath(QStringLiteral("last-system-vectors.csv")));
    if (vectors.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        QTextStream output(&vectors);
        output.setRealNumberPrecision(17);
        output << "index,rhs,external,independent_correction,full_correction,step_increment,coordinate_scale,"
                  "row_scale,linear_residual\n";
        for (Eigen::Index index = 0; index < rhs.size(); ++index)
        {
            const Eigen::Index fullDof = reduction ? reduction->masterDofs[index] : index;
            output << index << ',' << rhs[index] << ',' << independentExternal[index] << ','
                   << independentCorrection[index] << ',' << correction_[fullDof] << ',' << stage.stepIncrement[fullDof]
                   << ',' << coordinateScale[index] << ',' << rowScale[index] << ',' << linearResidual[index] << '\n';
        }
    }
    QFile matrix(QDir(diagnosticNewtonDirectory_).filePath(QStringLiteral("last-system-tangent.csv")));
    if (matrix.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        QTextStream output(&matrix);
        output.setRealNumberPrecision(17);
        output << "row,column,value\n";
        for (int outer = 0; outer < tangent.outerSize(); ++outer)
            for (SpMat::InnerIterator entry(tangent, outer); entry; ++entry)
                output << entry.row() << ',' << entry.col() << ',' << entry.value() << '\n';
    }
}

void SolverAdaptiveTSSBN::ApplyTrialIncrement(IAnalysisModel& model, const Vec& increment) const
{
    if (parameters_.referenceGalloping)
        model.ApplyIncrement(increment);
    else
        model.ApplyDynamicCorrection(increment, 0.0, 0.0);
}

void SolverAdaptiveTSSBN::SetTrialKinematics(IAnalysisModel& model, const Vec& velocity, const Vec& acceleration) const
{
    if (parameters_.referenceGalloping)
        model.SetTrialKinematics(velocity, acceleration);
    else
        model.SetTssbnTrialKinematics(velocity, acceleration);
}

bool SolverAdaptiveTSSBN::SolveImplicitStage(IAnalysisModel& model, double stepStartTime, double timeStep,
                                             const Vec& previousVelocity, const ImplicitStageDefinition& definition, StageState& stage)
{
    const double diagonalTime = definition.diagonalCoefficient * timeStep;
    if (!IsFinitePositive(diagonalTime))
    {
        lastStageFailureReason_ = QStringLiteral("阶段对角时间系数无效");
        return false;
    }

    // 原生四自由度 C2 保留 C1 绝对位移，避免先减再加步初位移的舍入。
    const bool retainReferencePreviousStage =
        parameters_.referenceGalloping && definition.previousStage && referenceStagePermutationReady_;
    if (!retainReferencePreviousStage)
        model.RollbackDynamicStep();
    lastStageFailureReason_.clear();
    lastResidualNorm_ = std::numeric_limits<double>::quiet_NaN();
    lastResidualLimit_ = std::numeric_limits<double>::quiet_NaN();

    // 普通阶段以零加速度预测，并独立累积仿射运动学修正，避免每轮从接近的速度反求加速度。
    Vec additiveStageVelocity;
    Vec additiveStageAcceleration;
    if (!parameters_.referenceGalloping)
    {
        additiveStageVelocity = previousVelocity + timeStep * definition.knownVelocityRate;
        additiveStageAcceleration = Vec::Zero(previousVelocity.size());
        stage.stepIncrement = timeStep * definition.knownDisplacementRate + diagonalTime * additiveStageVelocity;
    }
    else
    {
        stage.stepIncrement =
            timeStep * (definition.knownDisplacementRate + definition.diagonalCoefficient * definition.initialVelocity);
    }
    if (parameters_.referenceGalloping && definition.previousStage)
        stage.stepIncrement = definition.previousStage->stepIncrement;
    if (!retainReferencePreviousStage)
        ApplyTrialIncrement(model, stage.stepIncrement);
    model.GetStepIncrement(stage.stepIncrement);
    if (!parameters_.referenceGalloping)
    {
        // 普通有限转动只在本阶段首次由预测构型建立材料运动学，后续完整修正累积到独立阶段状态。
        try
        {
            model.BeginTssbnImplicitStage(definition.stageIndex, timeStep, baseCoefficients_.firstStageTime,
                                          baseCoefficients_.secondStageTime,
                                          baseCoefficients_.secondStageDiagonalFraction);
        }
        catch (const std::runtime_error& error)
        {
            lastStageFailureReason_ = QStringLiteral("普通阶段运动学初始化失败：%1").arg(QString::fromUtf8(error.what()));
            return false;
        }
    }

    const double velocityDerivative = 1.0 / diagonalTime;
    double accelerationDerivative = velocityDerivative * velocityDerivative;
    if (parameters_.referenceGalloping)
    {
        const double c1 = baseCoefficients_.firstStageTime;
        const double c2 = baseCoefficients_.secondStageTime;
        const double alpha = baseCoefficients_.secondStageDiagonalFraction;
        accelerationDerivative = definition.stageIndex == 1 ? 1.0 / (c1 * c1 * timeStep * timeStep)
                                                            : 1.0 / (c2 * c2 * alpha * alpha * timeStep * timeStep);
    }
    Vec externalForce;
    const double stageTime = stepStartTime + definition.timeFraction * timeStep;
    bool externalTangentAssembled = false;
    if (parameters_.aerodynamicTangentMode == AerodynamicTangentMode::Disabled)
    {
        externalTangent_.resize(model.GetFreeDofs(), model.GetFreeDofs());
        externalTangent_.setZero();
        externalTangentAssembled = true;
    }

    Vec previousIterationResidual;
    Vec previousReferenceResidual;
    double referenceCorrectionNorm = std::numeric_limits<double>::infinity();
    double lastConvergenceCorrection = std::numeric_limits<double>::infinity();
    double lastScaledCorrection = std::numeric_limits<double>::infinity();
    double scaledResidual = std::numeric_limits<double>::infinity();
    double precisionScaledResidual = std::numeric_limits<double>::infinity();
    constexpr double precisionLimit = 64.0 * std::numeric_limits<double>::epsilon();
    bool precisionCorrectionAttempted = false;
    double precisionCorrectionResidual = std::numeric_limits<double>::infinity();
    constexpr double precisionProgressSlope = 1.0e-4;
    // 每次获准的完整修正之后都须复查残差；末次评估不得再产生额外 Newton 更新。
    const int maximumEvaluations = parameters_.maximumNewtonIterations + 1;
    for (int iteration = 1; iteration <= maximumEvaluations; ++iteration)
    {
        stage.newtonIterations = iteration;
        if (parameters_.referenceGalloping)
        {
            if (definition.previousStage)
            {
                const double c2 = baseCoefficients_.secondStageTime;
                const double alpha = baseCoefficients_.secondStageDiagonalFraction;
                const double previousStageTime = c2 * timeStep * (1.0 - alpha);
                const double implicitTime = alpha * c2 * timeStep;
                stage.velocity =
                    (stage.stepIncrement - previousStageTime * definition.previousStage->velocity) / implicitTime;
                stage.acceleration =
                    (stage.velocity - previousVelocity - previousStageTime * definition.previousStage->acceleration) /
                    implicitTime;
            }
            else
            {
                stage.velocity = stage.stepIncrement / diagonalTime;
                stage.acceleration = (stage.velocity - previousVelocity) / diagonalTime;
            }
        }
        else
        {
            stage.velocity = additiveStageVelocity;
            stage.acceleration = additiveStageAcceleration;
        }
        model.SetTssbnStageKinematics(definition.stageIndex, timeStep, baseCoefficients_.firstStageTime,
                                      baseCoefficients_.secondStageTime, baseCoefficients_.secondStageDiagonalFraction,
                                      stage.velocity, stage.acceleration, !parameters_.referenceGalloping);

        NonlinearMPCData constraints;
        const bool mayHaveConstraints = model.HasActiveNonlinearMPC();
        if (mayHaveConstraints)
        {
            if (!model.AssembleNonlinearMPC(constraints))
            {
                lastStageFailureReason_ = QStringLiteral("MPC assembly failed");
                return false;
            }
            if (!constraints.Empty())
            {
                // 阶段位移由 MPC 消元约束，但其有限差分速度和加速度一般不满足曲面运动学。
                // 在动力残差组装前修正从自由度，使阶段力平衡使用切向速度和曲率加速度。
                if (!ProjectMpcKinematics(constraints, stage.velocity, stage.acceleration))
                {
                    lastStageFailureReason_ = QStringLiteral("MPC stage kinematics projection failed");
                    return false;
                }
                SetTrialKinematics(model, stage.velocity, stage.acceleration);
            }
        }

        // 对照模式只在阶段首轮及收敛核查时更新全部外载；正式模式每轮更新。
        const bool requiresExternalTangent =
            !externalTangentAssembled ||
            parameters_.aerodynamicTangentMode == AerodynamicTangentMode::EveryNewtonIteration;
        SystemEvaluationResult evaluation;
        const bool useCachedExternalForce = parameters_.diagnosticCacheExternalForcePerStage && iteration > 1;
        // 原生参考线路的上一修正已满足停止条件时，只刷新真实残差，省去不会使用的切线。
        const bool referenceCorrectionConverged = parameters_.referenceGalloping && referenceStagePermutationReady_ &&
                                                  !explicitMassStateDependent_ &&
                                                  referenceCorrectionNorm < parameters_.nonlinearTolerance;
        // 其余参考迭代合并更新切线和残差；普通线路保留先验收残差、再使用当前切线的顺序。
        const bool trialResidualFirst = (!parameters_.referenceGalloping || referenceCorrectionConverged) &&
                                        iteration > 1 && !mayHaveConstraints &&
                                        !parameters_.diagnosticCacheExternalForcePerStage;
        // 普通线路在同一状态一次取得残差和候选切线；原残差门仍先使用上一轮切线的尺度。
        // 仅原流程确实需要当前切线时才接管它，避免改变正常收敛与精度兜底的判据顺序。
        const bool fusedResidualTangent = !parameters_.referenceGalloping && trialResidualFirst;
        SystemEvaluationRequest request;
        if (fusedResidualTangent)
            request = SystemEvaluationRequest::EffectiveTangent(stageTime, accelerationDerivative, velocityDerivative,
                                                                requiresExternalTangent);
        else if (trialResidualFirst)
            request = SystemEvaluationRequest::TrialResidual(stageTime);
        else if (useCachedExternalForce)
            request = SystemEvaluationRequest::EffectiveTangentOnly(accelerationDerivative, velocityDerivative);
        else
            request = SystemEvaluationRequest::EffectiveTangent(stageTime, accelerationDerivative, velocityDerivative,
                                                                requiresExternalTangent);
        QElapsedTimer evaluationTimer;
        if (parameters_.diagnosticErrorTrace)
            evaluationTimer.start();
        model.EvaluateSystem(request, evaluation);
        if (parameters_.diagnosticErrorTrace)
        {
            ++diagnosticStageEvaluationCount_;
            if (fusedResidualTangent)
                ++diagnosticFusedResidualTangentCount_;
            diagnosticStageEvaluationMilliseconds_ += static_cast<double>(evaluationTimer.nsecsElapsed()) / 1.0e6;
        }
        if (!useCachedExternalForce)
            externalForce = std::move(evaluation.externalForce);
        if (!trialResidualFirst)
        {
            effectiveTangent_ = std::move(evaluation.effectiveTangent);
            if (requiresExternalTangent)
            {
                externalTangent_ = std::move(evaluation.externalTangent);
                externalTangentAssembled = true;
            }
            if (externalTangent_.nonZeros() > 0)
                effectiveTangent_ -= externalTangent_;
        }
        model.ComputeResidual(externalForce, residual_);

        bool deferredTangentAssembled = false;
        const auto assembleDeferredTangent = [&]()
        {
            if (fusedResidualTangent)
            {
                // 本轮尚无状态修正，直接接管同态装配结果，不重复计算 K、M、惯性切线。
                effectiveTangent_ = std::move(evaluation.effectiveTangent);
                if (requiresExternalTangent)
                {
                    externalTangent_ = std::move(evaluation.externalTangent);
                    externalTangentAssembled = true;
                }
                if (externalTangent_.nonZeros() > 0)
                    effectiveTangent_ -= externalTangent_;
                // evaluation 的图样版本对应刚接管的切线，无须沿用上一轮版本。
                deferredTangentAssembled = true;
                if (parameters_.diagnosticErrorTrace)
                    ++diagnosticReusedTangentCount_;
                return;
            }
            SystemEvaluationRequest tangentRequest =
                SystemEvaluationRequest::EffectiveTangentOnly(accelerationDerivative, velocityDerivative);
            if (requiresExternalTangent)
            {
                tangentRequest.Include(SystemContribution::ExternalTangent);
                tangentRequest.time = stageTime;
            }
            SystemEvaluationResult tangentEvaluation;
            QElapsedTimer tangentTimer;
            if (parameters_.diagnosticErrorTrace)
                tangentTimer.start();
            model.EvaluateSystem(tangentRequest, tangentEvaluation);
            if (parameters_.diagnosticErrorTrace)
            {
                ++diagnosticStageEvaluationCount_;
                diagnosticStageEvaluationMilliseconds_ += static_cast<double>(tangentTimer.nsecsElapsed()) / 1.0e6;
            }
            effectiveTangent_ = std::move(tangentEvaluation.effectiveTangent);
            if (requiresExternalTangent)
            {
                externalTangent_ = std::move(tangentEvaluation.externalTangent);
                externalTangentAssembled = true;
            }
            if (externalTangent_.nonZeros() > 0)
                effectiveTangent_ -= externalTangent_;
            evaluation.sparsePatternVersion = tangentEvaluation.sparsePatternVersion;
            deferredTangentAssembled = true;
        };

        // 每个隐式阶段复用与 Newmark 一致线性化的非线性 MPC 消元。
        if (!mayHaveConstraints && !model.AssembleNonlinearMPC(constraints))
        {
            lastStageFailureReason_ = QStringLiteral("MPC assembly failed");
            return false;
        }
        // 无 MPC 时直接借用已装配数据，避免每次 Newton 复制稀疏切线矩阵和阻尼因子。
        const bool hasConstraints = !constraints.Empty();
        if (trialResidualFirst && hasConstraints)
            assembleDeferredTangent();
        if (parameters_.diagnosticCacheExternalForcePerStage && hasConstraints)
        {
            lastStageFailureReason_ = QStringLiteral("阶段外载缓存对照模式不支持非线性 MPC");
            return false;
        }
        const SpMat* solveTangent = &effectiveTangent_;
        const Vec* solveRhs = &residual_;
        Vec reducedRhs;
        NonlinearMPCReduction reduction;
        double constraintNorm = 0.0;
        if (hasConstraints)
        {
            if (!NonlinearMPC::Reduce(effectiveTangent_, residual_, constraints, reduction))
            {
                lastStageFailureReason_ = QStringLiteral("MPC reduction failed");
                return false;
            }
            solveTangent = &reduction.tangent;
            reducedRhs = std::move(reduction.rhs);
            solveRhs = &reducedRhs;
            constraintNorm = reduction.constraintNorm;
        }

        const double residualScale = std::max(1.0, externalForce.norm());
        lastResidualNorm_ = solveRhs->norm();
        lastResidualLimit_ = parameters_.nonlinearTolerance * residualScale;
        const double residualChange = previousIterationResidual.size() == solveRhs->size()
                                          ? (*solveRhs - previousIterationResidual).norm()
                                          : std::numeric_limits<double>::infinity();
        const bool candidateConverged = !parameters_.referenceGalloping && solveRhs->allFinite() &&
                                        std::isfinite(constraintNorm) &&
                                        (lastResidualNorm_ == 0.0 || residualChange <= lastResidualLimit_ ||
                                         lastConvergenceCorrection <= parameters_.nonlinearTolerance) &&
                                        constraintNorm <= parameters_.nonlinearTolerance;
        previousIterationResidual = *solveRhs;
        bool referenceConverged = false;
        if (parameters_.referenceGalloping && iteration > 1)
        {
            Vec referenceResidual;
            model.GetReferenceConvergenceResidual(residual_, evaluation.constrainedExternalForce, referenceResidual);
            if (!referenceResidual.allFinite())
            {
                lastStageFailureReason_ = QStringLiteral("参考 Newton 完整残差包含非有限数值");
                return false;
            }
            if (previousReferenceResidual.size() == 0)
                previousReferenceResidual = Vec::Zero(referenceResidual.size());
            // 参考流程每次线性修正后比较相邻完整 RHS；首轮的上一 RHS 为零。
            lastResidualNorm_ = model.GetReferenceVectorNorm(previousReferenceResidual - referenceResidual);
            lastResidualLimit_ = parameters_.nonlinearTolerance;
            referenceConverged =
                (lastResidualNorm_ < lastResidualLimit_ || referenceCorrectionNorm < parameters_.nonlinearTolerance) &&
                std::isfinite(constraintNorm) && constraintNorm <= parameters_.nonlinearTolerance;
            previousReferenceResidual = std::move(referenceResidual);
        }
        // 位移及有限转动的小步差分均含 h^-2，绝对力残差可能到达舍入下限。
        // 仅当前切线求得的完整修正、当前缩放残差及约束同时足够小时，才允许精度受限停止。
        // 精度兜底保留独立的残差、修正和线性求解预算，不替代主迭代判据。
        const bool precisionCandidate =
            !parameters_.referenceGalloping && !candidateConverged &&
            parameters_.aerodynamicTangentMode != AerodynamicTangentMode::OncePerTimeStepOrStage &&
            !parameters_.diagnosticCacheExternalForcePerStage && iteration > 1 && solveRhs->allFinite() &&
            std::isfinite(constraintNorm) && constraintNorm <= parameters_.nonlinearTolerance;
        if (precisionCandidate && trialResidualFirst && !hasConstraints)
            assembleDeferredTangent();
        else if (candidateConverged && trialResidualFirst && !hasConstraints &&
                 (solveTangent->rows() != solveRhs->size() || solveTangent->cols() != solveRhs->size()))
            assembleDeferredTangent();
        scaledResidual = candidateConverged || precisionCandidate
                             ? EstimateScaledStageResidual(*solveRhs, externalForce, *solveTangent, stage.stepIncrement,
                                                           hasConstraints ? &reduction : nullptr)
                             : std::numeric_limits<double>::infinity();
        precisionScaledResidual = std::numeric_limits<double>::infinity();
        if (precisionCandidate)
        {
            Vec independentCoordinateScale(solveRhs->size());
            for (Eigen::Index index = 0; index < independentCoordinateScale.size(); ++index)
            {
                const Eigen::Index fullDof = hasConstraints ? reduction.masterDofs[index] : index;
                independentCoordinateScale[index] = std::max(1.0, std::abs(stage.stepIncrement[fullDof]));
            }
            const Vec independentExternal =
                hasConstraints ? (reduction.masterTransformation.transpose() * externalForce).eval() : externalForce;
            precisionScaledResidual = EstimateNewtonPrecisionResidual(
                *solveRhs, independentExternal, *solveTangent,
                independentCoordinateScale);
        }
        Vec independentCorrection;
        bool currentCorrectionReady = false;
        double currentScaledCorrection = std::numeric_limits<double>::infinity();
        bool precisionBudgetSatisfied = false;
        bool precisionConverged = false;
        if (precisionCandidate && precisionScaledResidual <= precisionLimit)
        {
            // 候选状态重新使用当前已装配切线和当前 RHS，不能沿用上一轮可能已缩放的修正。
            const std::uint64_t patternVersion =
                !hasConstraints && externalTangent_.nonZeros() == 0 ? evaluation.sparsePatternVersion : 0;
            QElapsedTimer linearTimer;
            if (parameters_.diagnosticErrorTrace)
                linearTimer.start();
            ++precisionCandidateSolveCount_;
            const bool linearSolveSucceeded =
                effectiveSystemSolver_.Solve(*solveTangent, *solveRhs,
                                                    independentCorrection, patternVersion);
            if (parameters_.diagnosticErrorTrace)
            {
                ++diagnosticStageLinearSolveCount_;
                diagnosticStageLinearSolveMilliseconds_ += static_cast<double>(linearTimer.nsecsElapsed()) / 1.0e6;
            }
            if (!linearSolveSucceeded)
            {
                lastStageFailureReason_ = QStringLiteral("精度受限候选的当前有效切线求解失败");
                return false;
            }
            TssbnLinearPrecisionResult refinement;
            QElapsedTimer refinementTimer;
            if (parameters_.diagnosticErrorTrace)
                refinementTimer.start();
            const bool refinementSucceeded = RefineTssbnPrecisionCorrection(
                effectiveSystemSolver_, *solveTangent, *solveRhs,
                precisionLimit, 3, patternVersion, independentCorrection, refinement);
            precisionCandidateSolveCount_ += refinement.refinementSolves;
            if (parameters_.diagnosticErrorTrace)
            {
                diagnosticStageLinearSolveCount_ += refinement.refinementSolves;
                diagnosticStageLinearSolveMilliseconds_ += static_cast<double>(refinementTimer.nsecsElapsed()) / 1.0e6;
            }
            if (!refinementSucceeded)
            {
                lastStageFailureReason_ = QStringLiteral("精度受限候选的真实系统残差精化失败");
                return false;
            }
            correction_ =
                hasConstraints ? reduction.RecoverFullIncrement(independentCorrection) : independentCorrection;
            if (correction_.size() != residual_.size() || !correction_.allFinite())
            {
                lastStageFailureReason_ = QStringLiteral("精度受限候选的完整修正尺寸或数值无效");
                return false;
            }
            currentCorrectionReady = true;
            currentScaledCorrection = 0.0;
            for (Eigen::Index index = 0; index < correction_.size(); ++index)
                currentScaledCorrection =
                    std::max(currentScaledCorrection,
                             std::abs(correction_[index]) / std::max(1.0, std::abs(stage.stepIncrement[index])));
            // 小位移修正除以 h 后也须远小于位移和速度的 LTE 尺度，防止极小步掩盖速度误差。
            precisionBudgetSatisfied =
                currentScaledCorrection <= precisionLimit && refinement.backwardError <= precisionLimit &&
                EstimateErrorComponent(model, correction_, stage.stepIncrement, stage.stepIncrement) <= 1.0e-3 &&
                EstimatePrecisionVelocityError(model, correction_, stage.stepIncrement, previousVelocity,
                                               stage.velocity, velocityDerivative) <= 1.0e-3;
            // 小修正仍可能改变步内状态；先真实施加并重装配，只有已实测停滞才允许精度兜底。
            precisionConverged = precisionBudgetSatisfied && precisionCorrectionAttempted &&
                                 lastResidualNorm_ >= (1.0 - precisionProgressSlope) * precisionCorrectionResidual;
            TraceNewtonPrecisionSystem(model, stepStartTime, timeStep, definition, iteration, stage, previousVelocity,
                                       *solveRhs, externalForce, *solveTangent, velocityDerivative,
                                       independentCorrection, hasConstraints ? &reduction : nullptr, precisionCandidate,
                                       precisionConverged);
        }
        const bool normallyConverged = candidateConverged;
        if (referenceConverged || normallyConverged || precisionConverged)
        {
            if (precisionConverged && !normallyConverged)
            {
                ++precisionLimitedStageCount_;
                precisionLimitedMaximumResidual_ = std::max(precisionLimitedMaximumResidual_, lastResidualNorm_);
                precisionLimitedMaximumScaledResidual_ =
                    std::max(precisionLimitedMaximumScaledResidual_, precisionScaledResidual);
                precisionLimitedMaximumCorrection_ =
                    std::max(precisionLimitedMaximumCorrection_, currentScaledCorrection);
                if (precisionLimitedStageCount_ <= 3)
                {
                    QString diagnostic;
                    QTextStream(&diagnostic)
                        << "tssbn_precision_limited stage=" << definition.stageIndex << " time=" << stageTime
                        << " dt=" << timeStep << " residual=" << lastResidualNorm_ << " scaled=" << precisionScaledResidual
                        << " correction=" << currentScaledCorrection;
                    Logger::Instance().Diagnostic(diagnostic);
                    if (parameters_.diagnosticErrorTrace && !Logger::DetailedOutputEnabled())
                        QTextStream(stdout) << diagnostic << Qt::endl;
                }
            }
            if (parameters_.diagnosticCacheExternalForcePerStage)
            {
                ++diagnosticExternalForceAuditCount_;
                Vec constrainedExternalForce;
                Vec refreshedExternalForce;
                model.ComputeExternalForce(stageTime, 1.0, constrainedExternalForce, refreshedExternalForce);
                if (refreshedExternalForce.size() != model.GetFreeDofs() || !refreshedExternalForce.allFinite())
                {
                    lastStageFailureReason_ = QStringLiteral("真实外载收敛核查的外载尺寸或数值无效");
                    return false;
                }
                model.ComputeResidual(refreshedExternalForce, residual_);
                lastResidualNorm_ = residual_.norm();
                lastResidualLimit_ = parameters_.nonlinearTolerance * std::max(1.0, refreshedExternalForce.norm());
                if (!residual_.allFinite() || !std::isfinite(lastResidualLimit_))
                {
                    lastStageFailureReason_ = QStringLiteral("真实外载收敛核查出现非有限数值");
                    return false;
                }
                externalForce = std::move(refreshedExternalForce);
                if (lastResidualNorm_ > lastResidualLimit_)
                {
                    ++diagnosticExternalForceAuditMismatchCount_;
                    continue;
                }
            }
            if (hasConstraints)
                model.SetNonlinearMPCMultipliers(reduction.multipliers);
            if (parameters_.diagnosticErrorTrace)
            {
                stage.finalResidualNorm = lastResidualNorm_;
                stage.finalResidualLimit = lastResidualLimit_;
                stage.finalResidual = parameters_.diagnosticCacheExternalForcePerStage ? residual_ : *solveRhs;
            }
            return true;
        }
        if (!solveRhs->allFinite() || !std::isfinite(constraintNorm))
        {
            lastStageFailureReason_ = QStringLiteral("非线性残差包含非有限数值");
            return false;
        }
        if (iteration == maximumEvaluations)
            break;
        if (trialResidualFirst && !hasConstraints && !deferredTangentAssembled)
            assembleDeferredTangent();
        if (!currentCorrectionReady)
        {
            const std::uint64_t patternVersion =
                !hasConstraints && externalTangent_.nonZeros() == 0 ? evaluation.sparsePatternVersion : 0;
            QElapsedTimer linearTimer;
            if (parameters_.diagnosticErrorTrace)
                linearTimer.start();
            const bool linearSolveSucceeded =
                effectiveSystemSolver_.Solve(*solveTangent, *solveRhs,
                                                    independentCorrection, patternVersion);
            if (parameters_.diagnosticErrorTrace)
            {
                ++diagnosticStageLinearSolveCount_;
                diagnosticStageLinearSolveMilliseconds_ += static_cast<double>(linearTimer.nsecsElapsed()) / 1.0e6;
            }
            if (!linearSolveSucceeded)
            {
                lastStageFailureReason_ = QStringLiteral("有效切线矩阵分解或求解失败");
                return false;
            }
        }

        correction_ = hasConstraints ? reduction.RecoverFullIncrement(independentCorrection) : independentCorrection;
        if (correction_.size() != residual_.size() || !correction_.allFinite())
        {
            lastStageFailureReason_ = QStringLiteral("invalid MPC correction");
            return false;
        }
        if (parameters_.referenceGalloping)
            referenceCorrectionNorm = model.GetReferenceVectorNorm(correction_);
        lastConvergenceCorrection = correction_.norm() / std::max(1.0e-12, stage.stepIncrement.norm());
        lastScaledCorrection = 0.0;
        for (Eigen::Index index = 0; index < correction_.size(); ++index)
            lastScaledCorrection =
                std::max(lastScaledCorrection,
                         std::abs(correction_[index]) / std::max(1.0, std::abs(stage.stepIncrement[index])));
        if (!currentCorrectionReady)
            TraceNewtonPrecisionSystem(model, stepStartTime, timeStep, definition, iteration, stage, previousVelocity,
                                       *solveRhs, externalForce, *solveTangent, velocityDerivative,
                                       independentCorrection, hasConstraints ? &reduction : nullptr, precisionCandidate,
                                       false);
        if (!parameters_.referenceGalloping)
        {
            precisionCorrectionAttempted = precisionBudgetSatisfied;
            precisionCorrectionResidual = lastResidualNorm_;
        }
        if (parameters_.referenceGalloping)
            ApplyTrialIncrement(model, correction_);
        else
        {
            try
            {
                model.ApplyTssbnImplicitStageCorrection(correction_);
            }
            catch (const std::runtime_error& error)
            {
                lastStageFailureReason_ = QStringLiteral("普通阶段完整修正失败：%1").arg(QString::fromUtf8(error.what()));
                return false;
            }
        }
        model.GetStepIncrement(stage.stepIncrement);
        if (!parameters_.referenceGalloping)
        {
            // 未投影的阶段量独立保存；有限转动由模型映射覆盖，MPC 的本轮投影不能污染下一轮仿射关系。
            additiveStageVelocity += velocityDerivative * correction_;
            additiveStageAcceleration += accelerationDerivative * correction_;
        }
    }
    lastStageFailureReason_ = parameters_.diagnosticCacheExternalForcePerStage
                                  ? QStringLiteral("达到 Newton 最大迭代次数；真实外载复查累计未收敛次数=%1")
                                        .arg(diagnosticExternalForceAuditMismatchCount_)
                                  : QStringLiteral("达到 Newton 最大迭代次数；最后缩放修正=%1，对角缩放残差=%2，"
                                                   "整行精度残差=%3")
                                        .arg(lastScaledCorrection, 0, 'g', 10)
                                        .arg(scaledResidual, 0, 'g', 10)
                                        .arg(precisionScaledResidual, 0, 'g', 10);
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

bool SolverAdaptiveTSSBN::SolveExplicitLastStageAcceleration(IAnalysisModel& model, double stageTime,
                                                             StageState& lastStage, _OUT Vec& acceleration,
                                                             bool acceptedEndpoint)
{
    const int dofCount = model.GetFreeDofs();
    const auto fail = [this, &model](const QString& reason)
    {
        lastStageFailureReason_ = reason;
        model.RollbackDynamicStep();
        return false;
    };

    // 在指定构型和速度建立试探态。动力残差等于还需要补偿的惯性力，
    // 因此一次质量方程 M * deltaA = residual 可恢复该状态的真实加速度。
    // 原生参考端点已在 LTE 通过后建立，中间只读取诊断；复用该状态，避免重复回滚和应用增量。
    const bool reuseAcceptedState = acceptedEndpoint && parameters_.referenceGalloping &&
                                    referenceStagePermutationReady_ && !explicitMassStateDependent_;
    if (!reuseAcceptedState)
    {
        model.RollbackDynamicStep();
        ApplyTrialIncrement(model, lastStage.stepIncrement);
        if (acceptedEndpoint && !parameters_.referenceGalloping)
        {
            // 每个整步/半步均在同一 SO(3) 端点上恢复动力平衡；仅用于 LTE 的 Y3 不改动。
            if (!model.NormalizeTssbnAcceptedRotations())
                return fail(QStringLiteral("接受端点旋转矩阵无效，无法恢复 SO(3) 正交性"));
        }
        model.GetStepIncrement(lastStage.stepIncrement);
        SetTrialKinematics(model, lastStage.velocity, lastStage.acceleration);
    }

    const std::uint64_t currentPatternVersion = model.GetSparsePatternVersion();
    const bool patternChanged = !explicitMassPatternValid_ || currentPatternVersion == 0 ||
                                explicitMassPatternVersion_ != currentPatternVersion;
    const bool assembleMass = explicitMassStateDependent_ || cachedExplicitMass_.rows() == 0 || patternChanged;
    SystemEvaluationResult evaluation;
    if (acceptedEndpoint)
        ++acceptedEndpointEvaluationCount_;
    else
        ++explicitLastStageEvaluationCount_;
    QElapsedTimer evaluationTimer;
    if (parameters_.diagnosticErrorTrace)
        evaluationTimer.start();
    model.EvaluateSystem(SystemEvaluationRequest::ExplicitDynamics(stageTime, assembleMass), evaluation);
    if (parameters_.diagnosticErrorTrace && !acceptedEndpoint)
        diagnosticY3EvaluationMilliseconds_ += static_cast<double>(evaluationTimer.nsecsElapsed()) / 1.0e6;
    if (assembleMass)
        cachedExplicitMass_ = std::move(evaluation.mass);
    if (cachedExplicitMass_.rows() != dofCount || cachedExplicitMass_.cols() != dofCount ||
        evaluation.externalForce.size() != dofCount || !evaluation.externalForce.allFinite())
        return fail(QStringLiteral("显式末阶段质量矩阵或外载尺寸/数值无效"));

    model.ComputeResidual(evaluation.externalForce, residual_);
    if (residual_.size() != dofCount || !residual_.allFinite())
        return fail(QStringLiteral("显式末阶段动力残差尺寸/数值无效"));

    const bool evaluatedPatternChanged = !explicitMassPatternValid_ || evaluation.sparsePatternVersion == 0 ||
                                         explicitMassPatternVersion_ != evaluation.sparsePatternVersion;
    QElapsedTimer massSolveTimer;
    if (parameters_.diagnosticErrorTrace)
        massSolveTimer.start();
    if (evaluatedPatternChanged)
    {
        explicitMassSolver_.analyzePattern(cachedExplicitMass_);
        // SparseLU 的 info() 仅在数值分解后有效，不能用于查询符号分析。
        explicitMassPatternValid_ = true;
        explicitMassFactorizationValid_ = false;
        explicitMassPatternVersion_ = evaluation.sparsePatternVersion;
    }
    if (explicitMassStateDependent_ || !explicitMassFactorizationValid_)
    {
        explicitMassSolver_.factorize(cachedExplicitMass_);
        explicitMassFactorizationValid_ = explicitMassSolver_.info() == Eigen::Success;
        if (!explicitMassFactorizationValid_)
            return fail(QStringLiteral("显式末阶段质量矩阵分解失败"));
    }

    if (acceptedEndpoint)
        ++acceptedEndpointMassSolveCount_;
    else
        ++explicitLastStageMassSolveCount_;
    const Vec accelerationCorrection = explicitMassSolver_.solve(residual_);
    if (parameters_.diagnosticErrorTrace && !acceptedEndpoint)
        diagnosticY3MassSolveMilliseconds_ += static_cast<double>(massSolveTimer.nsecsElapsed()) / 1.0e6;
    if (explicitMassSolver_.info() != Eigen::Success || accelerationCorrection.size() != dofCount ||
        !accelerationCorrection.allFinite())
        return fail(QStringLiteral("显式末阶段质量方程求解失败"));
    if (!parameters_.referenceGalloping)
    {
        // 成功分解并不足以证明质量方程可靠；检查逐行后向误差，兼顾不同质量量级。
        const Vec massEquationResidual = cachedExplicitMass_ * accelerationCorrection - residual_;
        const Vec massEquationScale = cachedExplicitMass_.cwiseAbs() * accelerationCorrection.cwiseAbs() +
                                      residual_.cwiseAbs();
        const double linearTolerance = 128.0 * std::numeric_limits<double>::epsilon() * std::max(1, dofCount);
        for (int dof = 0; dof < dofCount; ++dof)
            if (!std::isfinite(massEquationResidual[dof]) || !std::isfinite(massEquationScale[dof]) ||
                std::abs(massEquationResidual[dof]) >
                    linearTolerance * std::max(massEquationScale[dof], std::numeric_limits<double>::min()))
                return fail(QStringLiteral("显式末阶段质量方程后向误差超限"));
    }
    acceleration = lastStage.acceleration + accelerationCorrection;
    if (!acceleration.allFinite())
        return fail(QStringLiteral("显式末阶段真实加速度包含非有限数值"));

    // 保留该构型的试探态；端点调用者可以直接提交此状态。
    SetTrialKinematics(model, lastStage.velocity, acceleration);
    return true;
}

bool SolverAdaptiveTSSBN::EstablishAcceptedState(IAnalysisModel& model, _OUT Vec& stepIncrement, const Vec& velocity,
                                                 const Vec& acceleration)
{
    model.RollbackDynamicStep();
    ApplyTrialIncrement(model, stepIncrement);
    if (!parameters_.referenceGalloping)
    {
        if (!model.NormalizeTssbnAcceptedRotations())
        {
            lastStageFailureReason_ = QStringLiteral("接受端点旋转矩阵无效，无法恢复 SO(3) 正交性");
            model.RollbackDynamicStep();
            return false;
        }
    }
    model.GetStepIncrement(stepIncrement);
    SetTrialKinematics(model, velocity, acceleration);
    return true;
}

bool SolverAdaptiveTSSBN::RecoverAcceptedMpcAcceleration(IAnalysisModel& model, double time, const Vec& velocity,
                                                         _OUT Vec& acceleration)
{
    const int dofCount = model.GetFreeDofs();
    SystemEvaluationResult evaluation;
    ++acceptedEndpointEvaluationCount_;
    model.EvaluateSystem(SystemEvaluationRequest::ExplicitDynamics(time, true), evaluation);
    if (evaluation.mass.rows() != dofCount || evaluation.mass.cols() != dofCount ||
        evaluation.externalForce.size() != dofCount || !evaluation.externalForce.allFinite())
    {
        lastStageFailureReason_ = QStringLiteral("接受端点质量矩阵或外载尺寸/数值无效");
        return false;
    }
    model.ComputeResidual(evaluation.externalForce, residual_);
    if (residual_.size() != dofCount || !residual_.allFinite())
    {
        lastStageFailureReason_ = QStringLiteral("接受端点动力残差尺寸/数值无效");
        return false;
    }

    NonlinearMPCData constraints;
    if (!model.AssembleNonlinearMPC(constraints) || !constraints.IsValid(dofCount))
    {
        lastStageFailureReason_ = QStringLiteral("接受端点 MPC 约束装配或数据无效");
        return false;
    }

    Vec recoveredAcceleration;
    Vec multipliers;
    QString recoveryError;
    if (!NonlinearMPC::RecoverConsistentAcceleration(evaluation.mass, residual_, velocity, acceleration, constraints,
                                                     acceptedMpcAccelerationSolver_, recoveredAcceleration, multipliers,
                                                     recoveryError))
    {
        lastStageFailureReason_ = QStringLiteral("接受端点 MPC 加速度恢复失败：%1").arg(recoveryError);
        return false;
    }
    ++acceptedEndpointMassSolveCount_;
    ++acceptedEndpointMpcSolveCount_;
    acceleration = std::move(recoveredAcceleration);
    model.SetNonlinearMPCMultipliers(multipliers);
    SetTrialKinematics(model, velocity, acceleration);
    return true;
}

bool SolverAdaptiveTSSBN::SolveMpcSlaveCorrection(const NonlinearMPCData& constraints, const Vec& rightHandSide,
                                                  _OUT Vec& correction)
{
    const int constraintCount = static_cast<int>(constraints.value.size());
    const int dofCount = static_cast<int>(constraints.jacobian.cols());
    if (!constraints.IsValid(dofCount) || rightHandSide.size() != constraintCount || !rightHandSide.allFinite())
        return false;

    Eigen::MatrixXd slaveJacobian(constraintCount, constraintCount);
    bool diagonal = true;
    for (int row = 0; row < constraintCount; ++row)
    {
        for (int column = 0; column < constraintCount; ++column)
        {
            const double value = constraints.jacobian(row, constraints.slaveDofs[column]);
            slaveJacobian(row, column) = value;
            if (row != column && std::abs(value) > 1.0e-12)
                diagonal = false;
        }
    }

    Vec slaveCorrection(constraintCount);
    if (diagonal)
    {
        for (int row = 0; row < constraintCount; ++row)
        {
            const double pivot = slaveJacobian(row, row);
            if (!std::isfinite(pivot) || std::abs(pivot) <= 1.0e-12)
                return false;
            slaveCorrection[row] = rightHandSide[row] / pivot;
        }
    }
    else
    {
        Eigen::FullPivLU<Eigen::MatrixXd> factor(slaveJacobian);
        factor.setThreshold(1.0e-12);
        if (factor.rank() != constraintCount)
            return false;
        slaveCorrection = factor.solve(rightHandSide);
    }
    if (!slaveCorrection.allFinite())
        return false;

    correction = Vec::Zero(dofCount);
    for (int row = 0; row < constraintCount; ++row)
        correction[constraints.slaveDofs[row]] = slaveCorrection[row];
    return true;
}

bool SolverAdaptiveTSSBN::ProjectAcceptedMpcState(IAnalysisModel& model, _OUT Vec& stepIncrement, _OUT Vec& velocity,
                                                  _OUT Vec& acceleration)
{
    if (!model.HasActiveNonlinearMPC())
        return true;

    NonlinearMPCData constraints;
    bool positionConverged = false;
    for (int iteration = 0; iteration < 10; ++iteration)
    {
        if (!model.AssembleNonlinearMPC(constraints))
        {
            lastStageFailureReason_ = QStringLiteral("接受态 MPC 装配失败");
            return false;
        }
        if (constraints.Empty())
            return true;
        if (!constraints.IsValid(model.GetFreeDofs()))
        {
            lastStageFailureReason_ = QStringLiteral("接受态 MPC 数据无效");
            return false;
        }
        if (constraints.value.cwiseAbs().maxCoeff() <= 1.0e-11)
        {
            positionConverged = true;
            break;
        }

        Vec correction;
        if (!SolveMpcSlaveCorrection(constraints, -constraints.value, correction))
        {
            lastStageFailureReason_ = QStringLiteral("接受态 MPC 位置投影求解失败");
            return false;
        }
        ApplyTrialIncrement(model, correction);
        if (!parameters_.referenceGalloping)
        {
            if (!model.NormalizeTssbnAcceptedRotations())
            {
                lastStageFailureReason_ = QStringLiteral("MPC 位置投影后的旋转矩阵无效");
                return false;
            }
        }
        model.GetStepIncrement(stepIncrement);
    }
    if (!positionConverged)
    {
        lastStageFailureReason_ = QStringLiteral("接受态 MPC 位置投影未收敛");
        return false;
    }

    // 位置投影更新了 Jacobian；由最终构型的约束一阶和二阶导数建立端点运动学。
    if (!ProjectMpcKinematics(constraints, velocity, acceleration))
    {
        lastStageFailureReason_ = QStringLiteral("接受态 MPC 运动学投影求解失败");
        return false;
    }
    if (!stepIncrement.allFinite() || !velocity.allFinite() || !acceleration.allFinite())
    {
        lastStageFailureReason_ = QStringLiteral("接受态 MPC 投影出现非有限数值");
        return false;
    }
    SetTrialKinematics(model, velocity, acceleration);
    return true;
}

bool SolverAdaptiveTSSBN::ProjectMpcKinematics(const NonlinearMPCData& constraints, _OUT Vec& velocity,
                                               _OUT Vec& acceleration)
{
    Vec velocityCorrection;
    if (!SolveMpcSlaveCorrection(constraints, -(constraints.jacobian * velocity).eval(), velocityCorrection))
        return false;
    velocity += velocityCorrection;

    Vec accelerationViolation = constraints.jacobian * acceleration;
    const int constraintCount = static_cast<int>(constraints.value.size());
    const bool hasSparseHessians = static_cast<int>(constraints.hessians.size()) == constraintCount;
    const bool hasLocalHessians = static_cast<int>(constraints.hessianEntries.size()) == constraintCount;
    for (int row = 0; row < constraintCount; ++row)
    {
        if (hasSparseHessians && constraints.hessians[row].rows() == velocity.size())
            accelerationViolation[row] += velocity.dot(constraints.hessians[row] * velocity);
        if (hasLocalHessians)
            for (const auto& entry : constraints.hessianEntries[row])
                accelerationViolation[row] += entry.value() * velocity[entry.row()] * velocity[entry.col()];
    }
    Vec accelerationCorrection;
    if (!SolveMpcSlaveCorrection(constraints, -accelerationViolation, accelerationCorrection))
        return false;
    acceleration += accelerationCorrection;
    return velocity.allFinite() && acceleration.allFinite();
}

double SolverAdaptiveTSSBN::EstimateErrorComponent(const IAnalysisModel& model, const Vec& difference,
                                                   const Vec& referenceA, const Vec& referenceB, bool velocity) const
{
    const double absoluteTolerance = velocity ? parameters_.velocityAbsoluteTolerance : parameters_.absoluteTolerance;
    if (!parameters_.referenceGalloping || parameters_.errorNormMode != TssbnErrorNormMode::LegacyGlobal)
        return ScaledVectorNorm(difference, referenceA, referenceB, absoluteTolerance, parameters_.relativeTolerance,
                                parameters_.errorNormMode, velocity);
    if (difference.size() != referenceA.size() || difference.size() != referenceB.size() || !difference.allFinite() ||
        !referenceA.allFinite() || !referenceB.allFinite())
        return std::numeric_limits<double>::infinity();
    const double scale = absoluteTolerance +
                         parameters_.relativeTolerance * std::max(model.GetReferenceVectorNorm(referenceA),
                                                                  model.GetReferenceVectorNorm(referenceB)) +
                         1.0e-30;
    return std::isfinite(scale) && scale > 0.0 ? model.GetReferenceVectorNorm(difference) / scale
                                               : std::numeric_limits<double>::infinity();
}

double SolverAdaptiveTSSBN::EstimateNormalizedError(const IAnalysisModel& model, const Vec& baseIncrement,
                                                    const Vec& embeddedIncrement, const Vec& baseVelocity,
                                                    const Vec& embeddedVelocity, const Vec& previousDisplacement,
                                                    const Vec& previousVelocity,
                                                    const Vec* displacementDifferenceOverride,
                                                    const Vec* velocityDifferenceOverride) const
{
    // LTE 取两个更新量的差值；相对尺度必须基于物理状态。
    // 若按本步增量缩放，dt 变小时容限会被错误地逐步收紧。
    const Vec baseDisplacement = previousDisplacement + baseIncrement;
    const Vec displacementDifference = displacementDifferenceOverride
                                           ? *displacementDifferenceOverride
                                       : parameters_.referenceGalloping
                                           ? (baseDisplacement - (previousDisplacement + embeddedIncrement)).eval()
                                           : (baseIncrement - embeddedIncrement).eval();
    const double displacementError =
        EstimateErrorComponent(model, displacementDifference, previousDisplacement, baseDisplacement);
    const Vec velocityDifference = velocityDifferenceOverride ? *velocityDifferenceOverride
                                                              : (baseVelocity - embeddedVelocity).eval();
    const double velocityError =
        EstimateErrorComponent(model, velocityDifference, previousVelocity, baseVelocity, true);
    if (parameters_.errorNormMode == TssbnErrorNormMode::LegacyGlobal ||
        parameters_.errorNormMode == TssbnErrorNormMode::PaperMixedRms)
    {
        const double squaredError = displacementError * displacementError + velocityError * velocityError;
        return parameters_.referenceGalloping ? std::sqrt(0.5) * std::sqrt(squaredError)
                                              : std::sqrt(0.5 * squaredError);
    }
    return std::max(displacementError, velocityError);
}

bool SolverAdaptiveTSSBN::SolveBaseStepForErrorCheck(IAnalysisModel& model, double startTime, double timeStep,
                                                     _OUT int& stageIterations)
{
    Vec displacement;
    Vec velocity;
    Vec acceleration;
    model.GetState(displacement, velocity, acceleration);
    model.BackupStepState();
    StageState firstStage;
    StageState secondStage;
    if (!SolveImplicitStage(model, startTime, timeStep, velocity, CreateFirstStageDefinition(model.GetFreeDofs()), firstStage))
        return false;
    if (!SolveImplicitStage(model, startTime, timeStep, velocity, CreateSecondStageDefinition(timeStep, firstStage), secondStage))
        return false;
    stageIterations += firstStage.newtonIterations + secondStage.newtonIterations;
    // 每个被接受的半步仍须满足原两阶段牛顿修正上限。
    const int controlIterations = std::max(0, firstStage.newtonIterations + secondStage.newtonIterations - 2);
    if (!parameters_.referenceGalloping && controlIterations > parameters_.maximumTotalNewtonIterations)
    {
        lastStageFailureReason_ = QStringLiteral("半步 C1+C2 牛顿修正总数超过自适应上限");
        return false;
    }
    const double firstWeight = baseCoefficients_.firstStageWeight;
    Vec increment = timeStep * (firstWeight * firstStage.velocity + (1.0 - firstWeight) * secondStage.velocity);
    Vec endpointVelocity =
        velocity + timeStep * (firstWeight * firstStage.acceleration + (1.0 - firstWeight) * secondStage.acceleration);
    Vec unusedEmbeddedIncrement = increment;
    Vec unusedEmbeddedVelocity = endpointVelocity;
    const double extrapolation = (1.0 - baseCoefficients_.secondStageTime) /
                                 (baseCoefficients_.secondStageTime - baseCoefficients_.firstStageTime);
    acceleration = (1.0 + extrapolation) * secondStage.acceleration - extrapolation * firstStage.acceleration;
    model.CorrectTssbnStepStates(timeStep, baseCoefficients_.firstStageTime, baseCoefficients_.secondStageTime,
                                 embeddedCoefficients_.lastStageTime, firstWeight, embeddedCoefficients_.firstWeight,
                                 embeddedCoefficients_.secondWeight, embeddedCoefficients_.lastWeight,
                                 embeddedCoefficients_.lastStageFirstCoefficient,
                                 embeddedCoefficients_.lastStageSecondCoefficient, increment, endpointVelocity,
                                 unusedEmbeddedIncrement, unusedEmbeddedVelocity, acceleration);
    if (!EstablishAcceptedState(model, increment, endpointVelocity, acceleration))
        return false;
    if (parameters_.useReferenceExtrapolatedAcceleration)
    {
        if (model.HasActiveNonlinearMPC() && !ProjectAcceptedMpcState(model, increment, endpointVelocity, acceleration))
            return false;
    }
    else if (model.HasActiveNonlinearMPC())
    {
        if (!ProjectAcceptedMpcState(model, increment, endpointVelocity, acceleration) ||
            !RecoverAcceptedMpcAcceleration(model, startTime + timeStep, endpointVelocity, acceleration))
            return false;
    }
    else
    {
        StageState endpoint;
        endpoint.stepIncrement = increment;
        endpoint.velocity = endpointVelocity;
        endpoint.acceleration = acceleration;
        if (!SolveExplicitLastStageAcceleration(model, startTime + timeStep, endpoint, acceleration, true))
            return false;
    }
    model.CommitState();
    return true;
}

bool SolverAdaptiveTSSBN::ComputeStepDoublingError(const IAnalysisModel& model, const StepErrorState& coarseState,
                                                   const StepErrorState& fineState, const Vec& previousDisplacement,
                                                   const Vec& previousVelocity, _OUT double& normalizedError,
                                                   _OUT double& displacementDiagnosticError,
                                                   _OUT double& velocityDiagnosticError, _OUT int& displacementWorstDof,
                                                   _OUT int& velocityWorstDof)
{
    const Vec& coarseDisplacement = coarseState.displacement;
    const Vec& coarseVelocity = coarseState.velocity;
    const auto& coarseRotations = coarseState.rotations;
    const Vec& fineDisplacement = fineState.displacement;
    const Vec& fineVelocity = fineState.velocity;
    const auto& fineRotations = fineState.rotations;
    const Eigen::Index dofCount = model.GetFreeDofs();
    if (coarseRotations.size() != fineRotations.size() || coarseDisplacement.size() != dofCount ||
        fineDisplacement.size() != dofCount || coarseVelocity.size() != dofCount || fineVelocity.size() != dofCount ||
        previousDisplacement.size() != dofCount || previousVelocity.size() != dofCount ||
        !coarseDisplacement.allFinite() || !fineDisplacement.allFinite() || !coarseVelocity.allFinite() ||
        !fineVelocity.allFinite() || !previousDisplacement.allFinite() || !previousVelocity.allFinite())
    {
        lastStageFailureReason_ = QStringLiteral("整步/两半步状态尺寸或数值无效");
        return false;
    }

    // 继续按粗试步的 4/3 误差预算验收；普通路径接受细解，保留四倍局部误差裕量。
    // 归一化仍用粗试步状态，避免复用细解时同时扩大原粗细差接受门。
    constexpr double errorFactor = 4.0 / 3.0;
    Vec errorScaleDisplacement = coarseDisplacement;
    const Vec& errorScaleVelocity = coarseVelocity;
    Vec displacementDifference = errorFactor * (fineDisplacement - coarseDisplacement);
    Vec displacementStartScale = previousDisplacement;
    for (std::size_t index = 0; index < coarseRotations.size(); ++index)
    {
        const FiniteRotationState& coarse = coarseRotations[index];
        const FiniteRotationState& fine = fineRotations[index];
        if (coarse.freeDofs != fine.freeDofs || !coarse.rotation.allFinite() || !fine.rotation.allFinite())
            return false;
        Eigen::Vector3d difference;
        Eigen::Vector3d errorScaleLog;
        Utility::CR::Extract_RotationVector(fine.rotation * coarse.rotation.transpose(), difference);
        Utility::CR::Extract_RotationVector(coarse.rotation, errorScaleLog);
        if (!difference.allFinite() || !errorScaleLog.allFinite())
            return false;
        for (int axis = 0; axis < 3; ++axis)
        {
            const int dof = coarse.freeDofs[axis];
            if (dof < 0)
                continue;
            if (dof >= displacementDifference.size())
                return false;
            displacementDifference[dof] = errorFactor * difference[axis];
            displacementStartScale[dof] = 0.0;
            errorScaleDisplacement[dof] = errorScaleLog[axis];
        }
    }
    const double displacementError =
        EstimateErrorComponent(model, displacementDifference, displacementStartScale, errorScaleDisplacement);
    const double velocityError = EstimateErrorComponent(model, errorFactor * (fineVelocity - coarseVelocity),
                                                        previousVelocity, errorScaleVelocity, true);
    // 实际整步/两半步的位移、速度及 SO(3) 差共同控制时间步。
    normalizedError = std::max(displacementError, velocityError);
    displacementDiagnosticError = displacementError;
    velocityDiagnosticError = velocityError;
    if (parameters_.diagnosticErrorTrace)
    {
        // 控制值计算完后只定位贡献自由度；不以诊断重算值替换实际接受判据。
        const ScaledErrorDiagnostic displacementDiagnostic = DiagnoseScaledError(
            displacementDifference, displacementStartScale, errorScaleDisplacement, parameters_.absoluteTolerance,
            parameters_.relativeTolerance, parameters_.errorNormMode);
        const ScaledErrorDiagnostic velocityDiagnostic = DiagnoseScaledError(
            errorFactor * (fineVelocity - coarseVelocity), previousVelocity, errorScaleVelocity,
            parameters_.velocityAbsoluteTolerance, parameters_.relativeTolerance, parameters_.errorNormMode, true);
        displacementWorstDof = static_cast<int>(displacementDiagnostic.worstDof);
        velocityWorstDof = static_cast<int>(velocityDiagnostic.worstDof);
    }
    return std::isfinite(normalizedError);
}

bool SolverAdaptiveTSSBN::EstimateFiniteRotationStepError(IAnalysisModel& model, double startTime, double timeStep,
                                                          const Vec& previousDisplacement, const Vec& previousVelocity,
                                                          _OUT double& normalizedError, _OUT int& stageIterations,
                                                          _OUT bool& stateRestored, _OUT bool& fineStateAccepted,
                                                          _OUT double& displacementDiagnosticError,
                                                          _OUT double& velocityDiagnosticError,
                                                          _OUT int& displacementWorstDof, _OUT int& velocityWorstDof)
{
    stateRestored = true;
    fineStateAccepted = false;
    normalizedError = std::numeric_limits<double>::infinity();
    displacementDiagnosticError = std::numeric_limits<double>::infinity();
    velocityDiagnosticError = std::numeric_limits<double>::infinity();
    displacementWorstDof = -1;
    velocityWorstDof = -1;
    // 检查点包含粗试探态、宏步起点及单元历史；失败时先恢复它，再由外层回滚宏步。
    StateCheckpointGuard checkpoint(model, true);
    if (!checkpoint.IsActive())
    {
        lastStageFailureReason_ = QStringLiteral("有限转动误差检查无法创建模型检查点");
        return false;
    }
    bool solved = false;
    try
    {
        StepErrorState coarseState;
        StepErrorState fineState;
        model.GetState(coarseState.displacement, coarseState.velocity, coarseState.acceleration);
        model.GetFiniteRotationStates(coarseState.rotations);
        model.RollbackDynamicStep();
        solved = SolveBaseStepForErrorCheck(model, startTime, 0.5 * timeStep, stageIterations);
        if (solved)
            solved = SolveBaseStepForErrorCheck(model, startTime + 0.5 * timeStep, 0.5 * timeStep, stageIterations);
        if (solved)
        {
            model.GetState(fineState.displacement, fineState.velocity, fineState.acceleration);
            model.GetFiniteRotationStates(fineState.rotations);
            solved = ComputeStepDoublingError(model, coarseState, fineState, previousDisplacement, previousVelocity,
                                              normalizedError, displacementDiagnosticError, velocityDiagnosticError,
                                              displacementWorstDof, velocityWorstDof);
        }
        if (solved && normalizedError <= 1.0 && !parameters_.referenceGalloping)
        {
            // 保留两个半步已提交的全部状态；只释放旧粗解检查点，不再次 Commit。
            checkpoint.Release();
            fineStateAccepted = true;
            return true;
        }
    }
    catch (const std::exception& error)
    {
        solved = false;
        lastStageFailureReason_ = QStringLiteral("整步/两半步检查异常：%1").arg(QString::fromUtf8(error.what()));
    }
    catch (...)
    {
        solved = false;
        lastStageFailureReason_ = QStringLiteral("整步/两半步检查发生未知异常");
    }
    try
    {
        stateRestored = checkpoint.Restore();
    }
    catch (...)
    {
        stateRestored = false;
    }
    if (!stateRestored)
    {
        lastStageFailureReason_ = QStringLiteral("有限转动误差检查无法恢复模型检查点");
        return false;
    }
    checkpoint.Release();
    return solved;
}

double SolverAdaptiveTSSBN::ProposeAcceptedTimeStep(double currentTimeStep, double normalizedError,
                                                    int totalStageIterations)
{
    if (parameters_.referenceGalloping)
    {
        constexpr double epsilonSafe = 1.0e-30;
        double nextTimeStep = currentTimeStep;
        if (normalizedError <= 1.0)
        {
            // 参考路径保留倒数幂和乘法顺序，避免等价改写改变自适应试步分支。
            const double proportionalFactor =
                parameters_.safetyFactor * std::pow(1.0 / (normalizedError + epsilonSafe), 1.0 / 3.0);
            const double errorRatio = normalizedError / (previousAcceptedError_ + epsilonSafe);
            const double derivativeFactor =
                std::clamp(std::pow(errorRatio, -parameters_.derivativeGain), parameters_.minimumDerivativeFactor,
                           parameters_.maximumDerivativeFactor);
            const double iterationFactor =
                std::sqrt(static_cast<double>(parameters_.targetNewtonIterations) / std::max(1, totalStageIterations));
            const double errorFactor = std::clamp(proportionalFactor * derivativeFactor, parameters_.shrinkFactor,
                                                  parameters_.maximumGrowthFactor);
            const double adjustmentFactor = std::min(errorFactor, iterationFactor);
            nextTimeStep = std::clamp(currentTimeStep * adjustmentFactor, parameters_.minimumTimeStep,
                                      parameters_.maximumTimeStep);
        }
        previousAcceptedError_ = normalizedError;
        return nextTimeStep;
    }
    const double error = std::max(normalizedError, minimumControllerError);
    if (parameters_.useDuan2026Controller)
    {
        const double proportionalFactor = std::pow(error, -1.0 / 3.0);
        const double derivativeFactor =
            std::clamp(std::pow(error / std::max(previousAcceptedError_, minimumControllerError),
                               -parameters_.derivativeGain),
                       parameters_.minimumDerivativeFactor, parameters_.maximumDerivativeFactor);
        const double iterationFactor =
            std::sqrt(static_cast<double>(parameters_.targetNewtonIterations) / std::max(1, totalStageIterations));
        // Duan 式 (25)-(29)：三因子先相乘再限幅，安全系数在限幅外。
        const double adjustmentFactor = std::clamp(proportionalFactor * derivativeFactor * iterationFactor,
                                                   parameters_.shrinkFactor, parameters_.maximumGrowthFactor);
        previousAcceptedError_ = error;
        return std::clamp(parameters_.safetyFactor * currentTimeStep * adjustmentFactor, parameters_.minimumTimeStep,
                          parameters_.maximumTimeStep);
    }
    // 默认控制器保持已有的误差预测逻辑。
    static_cast<void>(totalStageIterations);
    const double adjustmentFactor = std::clamp(parameters_.safetyFactor * std::pow(error, -1.0 / 3.0),
                                               parameters_.shrinkFactor, parameters_.maximumGrowthFactor);
    previousAcceptedError_ = error;
    return std::clamp(currentTimeStep * adjustmentFactor, parameters_.minimumTimeStep, parameters_.maximumTimeStep);
}

double SolverAdaptiveTSSBN::ProposeRejectedTimeStep(double currentTimeStep, double normalizedError) const
{
    if (parameters_.referenceGalloping)
    {
        const double nextTimeStep =
            parameters_.safetyFactor * currentTimeStep * std::pow(1.0 / normalizedError, 1.0 / 3.0);
        return std::max(parameters_.minimumTimeStep,
                        std::min(currentTimeStep * parameters_.shrinkFactor, nextTimeStep));
    }
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
            initialVelocity,
            &firstStage};
}

bool SolverAdaptiveTSSBN::Solve(IAnalysisModel& model, double duration)
{
    diagnosticNewtonDirectory_ = QString::fromLocal8Bit(qgetenv("YQY_TSSBN_NEWTON_DIAGNOSTIC_DIR"));
    diagnosticNewtonStartTime_ = QString::fromLocal8Bit(qgetenv("YQY_TSSBN_NEWTON_DIAGNOSTIC_START")).toDouble();
    diagnosticNewtonSequence_ = 0;
    if (!diagnosticNewtonDirectory_.isEmpty() && !QDir().mkpath(diagnosticNewtonDirectory_))
        diagnosticNewtonDirectory_.clear();
    previousAcceptedError_ = parameters_.useDuan2026Controller ? 1.0e-6 : 1.0;
    precisionLimitedStageCount_ = 0;
    precisionCandidateSolveCount_ = 0;
    precisionLimitedMaximumResidual_ = 0.0;
    precisionLimitedMaximumScaledResidual_ = 0.0;
    precisionLimitedMaximumCorrection_ = 0.0;
    stoppedAtCheckpoint_ = false;
    checkpointWritten_ = false;
    const int dofCount = model.GetFreeDofs();
    if (dofCount <= 0 || !IsFinitePositive(duration))
        return false;
    if (resumeState_)
    {
        const CheckpointState& state = *resumeState_;
        if (!std::isfinite(state.time) || state.time < 0.0 || state.time >= duration ||
            !IsFinitePositive(state.nextTimeStep) || state.nextTimeStep < parameters_.minimumTimeStep ||
            state.nextTimeStep > parameters_.maximumTimeStep || !IsFinitePositive(state.previousAcceptedError) ||
            !IsFinitePositive(state.previousAcceptedTimeStep) || !std::isfinite(state.acceptedTimeStepSum) ||
            state.acceptedTimeStepSum <= 0.0 || state.acceptedStepCount < 1 || state.rejectedStepCount < 0 ||
            state.totalStageNewtonIterations < 0 || state.maximumStageNewtonIterations < 0)
        {
            model.ReportProgress(0.0, QStringLiteral("自适应 TSSBN 检查点控制状态无效"));
            return false;
        }
    }
    if (checkpointWriter_ && (!IsFinitePositive(checkpointTargetTime_) || checkpointTargetTime_ > duration ||
                              (resumeState_ && checkpointTargetTime_ <= resumeState_->time)))
    {
        model.ReportProgress(0.0, QStringLiteral("自适应 TSSBN 检查点目标时间无效"));
        return false;
    }
    if (parameters_.diagnosticCacheExternalForcePerStage && model.HasActiveNonlinearMPC())
    {
        model.ReportProgress(0.0, QStringLiteral("阶段外载缓存对照模式不支持非线性 MPC"));
        return false;
    }

    effectiveTangent_.resize(dofCount, dofCount);
    residual_.resize(dofCount);
    correction_.resize(dofCount);
    bool referenceAbsoluteLteEligible = false;
    effectiveSystemSolver_.Reset();
    referenceStagePermutationReady_ = false;
    effectiveSystemSolver_.SetAllowGpuAutomatic(!parameters_.referenceGalloping && model.ShouldPreferGpuLinearSolver());
    if (parameters_.referenceGalloping)
    {
        effectiveSystemSolver_.SetModeOverride(LinearSolverMode::Lu);
        std::vector<int> referencePermutation;
        if (model.GetReferenceLinearSolvePermutation(referencePermutation))
        {
            if (!effectiveSystemSolver_.SetSignedPermutation(referencePermutation))
                return false;
            referenceStagePermutationReady_ = true;
            referenceAbsoluteLteEligible = true;
        }
    }
    if (parameters_.referenceGalloping && parameters_.diagnosticErrorTrace)
        QTextStream(stdout) << "tssbn_reference_ordering canonical=" << referenceAbsoluteLteEligible
                            << " absolute_lte=" << referenceAbsoluteLteEligible << Qt::endl;
    acceptedMpcAccelerationSolver_.Reset();
    acceptedMpcAccelerationSolver_.SetAllowGpuAutomatic(model.ShouldPreferGpuLinearSolver());
    cachedExplicitMass_.resize(0, 0);
    explicitMassPatternVersion_ = 0;
    explicitMassPatternValid_ = false;
    explicitMassFactorizationValid_ = false;
    explicitMassStateDependent_ = model.HasStateDependentMass();

    acceptedStepCount_ = 0;
    rejectedStepCount_ = 0;
    totalStageNewtonIterations_ = 0;
    maximumStageNewtonIterations_ = 0;
    diagnosticExternalForceAuditCount_ = 0;
    diagnosticExternalForceAuditMismatchCount_ = 0;
    explicitLastStageEvaluationCount_ = 0;
    explicitLastStageMassSolveCount_ = 0;
    acceptedEndpointEvaluationCount_ = 0;
    acceptedEndpointMassSolveCount_ = 0;
    acceptedEndpointMpcSolveCount_ = 0;
    explicitLastStageDisplacementSkipCount_ = 0;
    diagnosticAcceptedVelocityDominantCount_ = 0;
    diagnosticAcceptedDisplacementDominantCount_ = 0;
    diagnosticStageEvaluationCount_ = 0;
    diagnosticStageLinearSolveCount_ = 0;
    diagnosticFusedResidualTangentCount_ = 0;
    diagnosticReusedTangentCount_ = 0;
    diagnosticFirstStageNewtonIterations_ = 0;
    diagnosticSecondStageNewtonIterations_ = 0;
    diagnosticLocalResidualDelayCount_ = 0;
    diagnosticMaximumLocalResidual_ = 0.0;
    diagnosticAcceptedAttemptMilliseconds_ = 0.0;
    diagnosticRejectedAttemptMilliseconds_ = 0.0;
    diagnosticFirstStageMilliseconds_ = 0.0;
    diagnosticSecondStageMilliseconds_ = 0.0;
    diagnosticStageEvaluationMilliseconds_ = 0.0;
    diagnosticStageLinearSolveMilliseconds_ = 0.0;
    diagnosticY3Milliseconds_ = 0.0;
    diagnosticY3EvaluationMilliseconds_ = 0.0;
    diagnosticY3MassSolveMilliseconds_ = 0.0;
    diagnosticAcceptedEndpointMilliseconds_ = 0.0;
    diagnosticAcceptedTailMilliseconds_ = 0.0;
    averageTimeStep_ = 0.0;

    Vec previousDisplacement;
    Vec previousVelocity;
    Vec previousAcceleration;
    model.GetState(previousDisplacement, previousVelocity, previousAcceleration);
    std::vector<FiniteRotationState> initialRotations;
    model.GetFiniteRotationStates(initialRotations);
    std::vector<bool> finiteRotationDofs(static_cast<std::size_t>(dofCount), false);
    for (const FiniteRotationState& rotation : initialRotations)
        for (int dof : rotation.freeDofs)
            if (dof >= 0 && dof < dofCount)
                finiteRotationDofs[static_cast<std::size_t>(dof)] = true;
    // MPC 采用整步/两半步比较；无 MPC 采用真实末阶段的位移、速度嵌入估计。
    const bool requiresIndependentStepError =
        !parameters_.referenceGalloping && model.HasActiveNonlinearMPC();
    const bool ordinaryEmbedded = !parameters_.referenceGalloping && !requiresIndependentStepError;
    // 无 MPC 的速度嵌入估计使用真实 Y3；接受端点加速度仍按各自配置更新。
    const bool useEmbeddedStepError = !requiresIndependentStepError;
    if (parameters_.diagnosticErrorTrace && !parameters_.referenceGalloping)
        QTextStream(stdout) << "tssbn_error_strategy mode="
                            << (ordinaryEmbedded ? "two_stage_embedded_with_y3" : "step_doubling")
                            << " extrapolated_y3=0"
                            << " independent_steps=" << (requiresIndependentStepError ? 2 : 0)
                            << " endpoint_recovery=" << (parameters_.useReferenceExtrapolatedAcceleration ? 0 : 1)
                            << Qt::endl;
    long long diagnosticIndependentAttemptCount = 0;
    double diagnosticIndependentMilliseconds = 0.0;
    double diagnosticMaximumIndependentError = 0.0;
    NonlinearMPCData initialConstraints;
    if (!model.AssembleNonlinearMPC(initialConstraints))
    {
        const QString message = QStringLiteral("自适应 TSSBN 初始 MPC 约束组装失败");
        model.ReportProgress(0.0, message);
        qDebug().noquote() << message;
        return false;
    }
    if (!initialConstraints.Empty())
    {
        QString errorMessage;
        if (!NonlinearMPC::ValidateInitialKinematics(initialConstraints, previousDisplacement, previousVelocity,
                                                     previousAcceleration, errorMessage))
        {
            const QString message = QStringLiteral("自适应 TSSBN %1").arg(errorMessage);
            model.ReportProgress(0.0, message);
            qDebug().noquote() << message;
            return false;
        }
    }
    double currentTime = resumeState_ ? resumeState_->time : 0.0;
    double nextTimeStep = resumeState_ ? resumeState_->nextTimeStep
                                       : std::clamp(parameters_.initialTimeStep, parameters_.minimumTimeStep,
                                                    parameters_.maximumTimeStep);
    double acceptedTimeStepSum = resumeState_ ? resumeState_->acceptedTimeStepSum : 0.0;
    double previousAcceptedTimeStep =
        resumeState_ ? resumeState_->previousAcceptedTimeStep : std::numeric_limits<double>::quiet_NaN();
    if (resumeState_)
    {
        previousAcceptedError_ = resumeState_->previousAcceptedError;
        acceptedStepCount_ = resumeState_->acceptedStepCount;
        rejectedStepCount_ = resumeState_->rejectedStepCount;
        totalStageNewtonIterations_ = resumeState_->totalStageNewtonIterations;
        maximumStageNewtonIterations_ = resumeState_->maximumStageNewtonIterations;
    }
    // 避免时间累加的浮点余量触发无意义的终点极小步。
    const double completionTolerance = parameters_.referenceGalloping
                                           ? std::min(0.5 * duration, 1.0e-10)
                                           : std::min(0.5 * duration, std::max(timeTolerance, 1.0e-9 * duration));

    qDebug().noquote() << QStringLiteral("开始自适应 TSSBN：时长=%1，dt=%2，相对 LTE 容限=%3")
                              .arg(duration, 0, 'g', 8)
                              .arg(nextTimeStep, 0, 'g', 8)
                              .arg(parameters_.relativeTolerance, 0, 'g', 8);
    Logger::Instance().Diagnostic(QStringLiteral("自适应 TSSBN 参数：ρ∞=%1，气动切线模式=%2，阶段外载缓存=%3")
                                      .arg(parameters_.spectralRadiusInfinity, 0, 'g', 8)
                                      .arg(static_cast<int>(parameters_.aerodynamicTangentMode))
                                      .arg(parameters_.diagnosticCacheExternalForcePerStage ? 1 : 0));

    // 验证专用的分窗遥测：按模拟时间约 1 s 汇总，而非逐 Newton/逐单元打印。
    // 读取现有累计计数器的差值，不改变默认求解路径。
    QElapsedTimer diagnosticWindowTimer;
    if (parameters_.diagnosticErrorTrace)
        diagnosticWindowTimer.start();
    double diagnosticWindowStartTime = currentTime;
    double diagnosticNextWindowTime = std::floor(currentTime) + 1.0;
    double diagnosticPreviousWallMilliseconds = 0.0;
    double diagnosticPreviousAcceptedTimeStepSum = acceptedTimeStepSum;
    double diagnosticMinimumAcceptedTimeStep = std::numeric_limits<double>::infinity();
    double diagnosticMaximumNormalizedError = 0.0;
    double diagnosticMaximumAcceptedNormalizedError = 0.0;
    double diagnosticMaximumDisplacementError = 0.0;
    double diagnosticMaximumVelocityError = 0.0;
    QString diagnosticMaximumAcceptedComponent = QStringLiteral("none");
    QString diagnosticMaximumAcceptedDof = QStringLiteral("none");
    QString diagnosticMaximumDisplacementDof = QStringLiteral("none");
    QString diagnosticMaximumVelocityDof = QStringLiteral("none");
    int diagnosticPreviousAcceptedSteps = acceptedStepCount_;
    int diagnosticPreviousRejectedSteps = rejectedStepCount_;
    long long diagnosticPreviousFirstStageNewton = diagnosticFirstStageNewtonIterations_;
    long long diagnosticPreviousSecondStageNewton = diagnosticSecondStageNewtonIterations_;
    long long diagnosticPreviousStageEvaluations = diagnosticStageEvaluationCount_;
    long long diagnosticPreviousStageLinearSolves = diagnosticStageLinearSolveCount_;
    long long diagnosticPreviousY3Evaluations = explicitLastStageEvaluationCount_;
    long long diagnosticPreviousY3MassSolves = explicitLastStageMassSolveCount_;
    long long diagnosticPreviousEndpointEvaluations = acceptedEndpointEvaluationCount_;
    long long diagnosticPreviousEndpointMassSolves = acceptedEndpointMassSolveCount_;
    double diagnosticPreviousStageEvaluationMilliseconds = diagnosticStageEvaluationMilliseconds_;
    double diagnosticPreviousStageLinearMilliseconds = diagnosticStageLinearSolveMilliseconds_;
    double diagnosticPreviousY3Milliseconds = diagnosticY3Milliseconds_;
    double diagnosticPreviousEndpointMilliseconds = diagnosticAcceptedEndpointMilliseconds_;
    const auto reportDiagnosticWindow = [&]()
    {
        if (!parameters_.diagnosticErrorTrace || currentTime <= diagnosticWindowStartTime)
            return;
        const double wallMilliseconds = static_cast<double>(diagnosticWindowTimer.nsecsElapsed()) / 1.0e6;
        const int accepted = acceptedStepCount_ - diagnosticPreviousAcceptedSteps;
        const int rejected = rejectedStepCount_ - diagnosticPreviousRejectedSteps;
        const double acceptedStepSum = acceptedTimeStepSum - diagnosticPreviousAcceptedTimeStepSum;
        QTextStream trace(stdout);
        trace.setRealNumberPrecision(12);
        trace << "tssbn_window t0=" << diagnosticWindowStartTime << " t1=" << currentTime
              << " wall_ms=" << wallMilliseconds - diagnosticPreviousWallMilliseconds << " accepted=" << accepted
              << " rejected=" << rejected << " dt_min=" << (accepted > 0 ? diagnosticMinimumAcceptedTimeStep : 0.0)
              << " dt_mean=" << (accepted > 0 ? acceptedStepSum / accepted : 0.0)
              << " lte_max=" << diagnosticMaximumNormalizedError
              << " accepted_lte_max=" << diagnosticMaximumAcceptedNormalizedError
              << " accepted_lte_component=" << diagnosticMaximumAcceptedComponent
              << " accepted_lte_dof=" << diagnosticMaximumAcceptedDof
              << " displacement_lte_max=" << diagnosticMaximumDisplacementError
              << " displacement_lte_dof=" << diagnosticMaximumDisplacementDof
              << " velocity_lte_max=" << diagnosticMaximumVelocityError
              << " velocity_lte_dof=" << diagnosticMaximumVelocityDof
              << " newton_c1=" << diagnosticFirstStageNewtonIterations_ - diagnosticPreviousFirstStageNewton
              << " newton_c2=" << diagnosticSecondStageNewtonIterations_ - diagnosticPreviousSecondStageNewton
              << " stage_eval=" << diagnosticStageEvaluationCount_ - diagnosticPreviousStageEvaluations
              << " stage_eval_ms="
              << diagnosticStageEvaluationMilliseconds_ - diagnosticPreviousStageEvaluationMilliseconds
              << " stage_linear=" << diagnosticStageLinearSolveCount_ - diagnosticPreviousStageLinearSolves
              << " stage_linear_ms="
              << diagnosticStageLinearSolveMilliseconds_ - diagnosticPreviousStageLinearMilliseconds
              << " y3_eval=" << explicitLastStageEvaluationCount_ - diagnosticPreviousY3Evaluations
              << " y3_mass=" << explicitLastStageMassSolveCount_ - diagnosticPreviousY3MassSolves
              << " y3_ms=" << diagnosticY3Milliseconds_ - diagnosticPreviousY3Milliseconds
              << " endpoint_eval=" << acceptedEndpointEvaluationCount_ - diagnosticPreviousEndpointEvaluations
              << " endpoint_mass=" << acceptedEndpointMassSolveCount_ - diagnosticPreviousEndpointMassSolves
              << " endpoint_ms=" << diagnosticAcceptedEndpointMilliseconds_ - diagnosticPreviousEndpointMilliseconds
              << Qt::endl;
        diagnosticWindowStartTime = currentTime;
        diagnosticPreviousWallMilliseconds = wallMilliseconds;
        diagnosticPreviousAcceptedTimeStepSum = acceptedTimeStepSum;
        diagnosticMinimumAcceptedTimeStep = std::numeric_limits<double>::infinity();
        diagnosticMaximumNormalizedError = 0.0;
        diagnosticMaximumAcceptedNormalizedError = 0.0;
        diagnosticMaximumDisplacementError = 0.0;
        diagnosticMaximumVelocityError = 0.0;
        diagnosticMaximumAcceptedComponent = QStringLiteral("none");
        diagnosticMaximumAcceptedDof = QStringLiteral("none");
        diagnosticMaximumDisplacementDof = QStringLiteral("none");
        diagnosticMaximumVelocityDof = QStringLiteral("none");
        diagnosticPreviousAcceptedSteps = acceptedStepCount_;
        diagnosticPreviousRejectedSteps = rejectedStepCount_;
        diagnosticPreviousFirstStageNewton = diagnosticFirstStageNewtonIterations_;
        diagnosticPreviousSecondStageNewton = diagnosticSecondStageNewtonIterations_;
        diagnosticPreviousStageEvaluations = diagnosticStageEvaluationCount_;
        diagnosticPreviousStageLinearSolves = diagnosticStageLinearSolveCount_;
        diagnosticPreviousY3Evaluations = explicitLastStageEvaluationCount_;
        diagnosticPreviousY3MassSolves = explicitLastStageMassSolveCount_;
        diagnosticPreviousEndpointEvaluations = acceptedEndpointEvaluationCount_;
        diagnosticPreviousEndpointMassSolves = acceptedEndpointMassSolveCount_;
        diagnosticPreviousStageEvaluationMilliseconds = diagnosticStageEvaluationMilliseconds_;
        diagnosticPreviousStageLinearMilliseconds = diagnosticStageLinearSolveMilliseconds_;
        diagnosticPreviousY3Milliseconds = diagnosticY3Milliseconds_;
        diagnosticPreviousEndpointMilliseconds = diagnosticAcceptedEndpointMilliseconds_;
    };

    while (currentTime < duration - completionTolerance)
    {
        if (model.IsCancellationRequested())
            return false;

        const double remainingTime = duration - currentTime;
        double attemptedTimeStep = std::min(nextTimeStep, remainingTime);
        // 末两步均分余量，避免留下远小于正常步长的终点微步并放大角加速度差分舍入。
        if (!parameters_.referenceGalloping && remainingTime > nextTimeStep * (1.0 + 1.0e-9) &&
            remainingTime < 2.0 * nextTimeStep && 0.5 * remainingTime >= parameters_.minimumTimeStep)
            attemptedTimeStep = 0.5 * remainingTime;
        model.BackupStepState();
        bool stepAccepted = false;
        QString lastFailedOperation;
        double lastNormalizedError = std::numeric_limits<double>::quiet_NaN();
        double lastEvaluatedTimeStep = attemptedTimeStep;

        for (int attempt = 1; attempt <= parameters_.maximumRejectedAttempts; ++attempt)
        {
            if (!parameters_.referenceGalloping)
            {
                // 本轮阶段失败时尚无 LTE，不能把前一拒步的误差和下一候选步长冒充本轮数据。
                lastNormalizedError = std::numeric_limits<double>::quiet_NaN();
                lastEvaluatedTimeStep = attemptedTimeStep;
            }
            QElapsedTimer attemptTimer;
            if (parameters_.diagnosticErrorTrace)
                attemptTimer.start();
            QString failedOperation;
            // 与参考 TSSBN 的阶段初始化一致：q_c1 从 q_n 开始，
            // 因此 v_c1 的初始值为零。
            const ImplicitStageDefinition firstDefinition = CreateFirstStageDefinition(dofCount);
            StageState firstStage;
            QElapsedTimer stageTimer;
            if (parameters_.diagnosticErrorTrace)
                stageTimer.start();
            bool stagesConverged = SolveImplicitStage(model, currentTime, attemptedTimeStep, previousVelocity,
                                                      firstDefinition, firstStage);
            if (parameters_.diagnosticErrorTrace)
                diagnosticFirstStageNewtonIterations_ += firstStage.newtonIterations;
            if (parameters_.diagnosticErrorTrace)
                diagnosticFirstStageMilliseconds_ += static_cast<double>(stageTimer.nsecsElapsed()) / 1.0e6;
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
                if (parameters_.diagnosticErrorTrace)
                    stageTimer.start();
                stagesConverged = SolveImplicitStage(model, currentTime, attemptedTimeStep, previousVelocity,
                                                     secondDefinition, secondStage);
                if (parameters_.diagnosticErrorTrace)
                    diagnosticSecondStageNewtonIterations_ += secondStage.newtonIterations;
                if (parameters_.diagnosticErrorTrace)
                    diagnosticSecondStageMilliseconds_ += static_cast<double>(stageTimer.nsecsElapsed()) / 1.0e6;
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
            bool hasActiveMpc = false;
            if (stagesConverged)
            {
                stagesConverged = EvaluateExplicitLastStage(attemptedTimeStep, previousVelocity, firstStage,
                                                            secondStage, explicitLastStage);
                if (!stagesConverged)
                {
                    failedOperation = QStringLiteral("显式末阶段");
                    lastFailedOperation = failedOperation;
                }
                else
                {
                    hasActiveMpc = model.HasActiveNonlinearMPC();
                    if (!hasActiveMpc)
                    {
                        model.CorrectTssbnExplicitLastStage(
                            attemptedTimeStep, embeddedCoefficients_.lastStageFirstCoefficient,
                            embeddedCoefficients_.lastStageSecondCoefficient, explicitLastStage.stepIncrement,
                            explicitLastStage.velocity);
                        stagesConverged = explicitLastStage.stepIncrement.size() == dofCount &&
                                          explicitLastStage.velocity.size() == dofCount &&
                                          explicitLastStage.stepIncrement.allFinite() &&
                                          explicitLastStage.velocity.allFinite();
                        if (!stagesConverged)
                        {
                            failedOperation = QStringLiteral("显式末阶段旋转状态");
                            lastFailedOperation = failedOperation;
                            lastStageFailureReason_ = QStringLiteral("显式末阶段旋转修正后尺寸或数值无效");
                        }
                    }
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
                if (parameters_.diagnosticErrorTrace)
                    diagnosticRejectedAttemptMilliseconds_ += static_cast<double>(attemptTimer.nsecsElapsed()) / 1.0e6;
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
            Vec referenceAcceptedIncrement;
            Vec referenceAcceptedVelocity;
            Vec referenceOrder2Displacement;
            Vec referenceOrder3Displacement;
            if (parameters_.referenceGalloping)
            {
                referenceAcceptedIncrement = baseIncrement;
                referenceAcceptedVelocity = baseVelocity;
                // 按参考源程序逐项形成完整状态，保持其浮点运算次序。
                referenceOrder2Displacement = previousDisplacement;
                referenceOrder2Displacement += (attemptedTimeStep * firstWeight) * firstStage.velocity;
                referenceOrder2Displacement += (attemptedTimeStep * (1.0 - firstWeight)) * secondStage.velocity;
                baseIncrement = referenceOrder2Displacement - previousDisplacement;
                baseVelocity = previousVelocity;
                baseVelocity += (attemptedTimeStep * firstWeight) * firstStage.acceleration;
                baseVelocity += (attemptedTimeStep * (1.0 - firstWeight)) * secondStage.acceleration;
                referenceOrder3Displacement = previousDisplacement;
                referenceOrder3Displacement +=
                    (attemptedTimeStep * embeddedCoefficients_.firstWeight) * firstStage.velocity;
                referenceOrder3Displacement +=
                    (attemptedTimeStep * embeddedCoefficients_.secondWeight) * secondStage.velocity;
                referenceOrder3Displacement +=
                    (attemptedTimeStep * embeddedCoefficients_.lastWeight) * explicitLastStage.velocity;
                embeddedIncrement = referenceOrder3Displacement - previousDisplacement;
                embeddedVelocity = previousVelocity;
                embeddedVelocity += (attemptedTimeStep * embeddedCoefficients_.firstWeight) * firstStage.acceleration;
                embeddedVelocity += (attemptedTimeStep * embeddedCoefficients_.secondWeight) * secondStage.acceleration;
                embeddedVelocity +=
                    (attemptedTimeStep * embeddedCoefficients_.lastWeight) * explicitLastStage.acceleration;
            }

            // 外推加速度用于试探态初值；普通默认在误差通过后恢复端点动力平衡。
            // 误差策略不改变参考兼容配置对接受端点加速度的选择。
            Vec acceptedAcceleration = explicitLastStage.acceleration;
            model.CorrectTssbnStepStates(
                attemptedTimeStep, baseCoefficients_.firstStageTime, baseCoefficients_.secondStageTime,
                embeddedCoefficients_.lastStageTime, firstWeight, embeddedCoefficients_.firstWeight,
                embeddedCoefficients_.secondWeight, embeddedCoefficients_.lastWeight,
                embeddedCoefficients_.lastStageFirstCoefficient, embeddedCoefficients_.lastStageSecondCoefficient,
                baseIncrement, baseVelocity, embeddedIncrement, embeddedVelocity, acceptedAcceleration);
            if (parameters_.referenceGalloping && !hasActiveMpc)
            {
                // 参考的接受态按分组加权更新，LTE 状态按逐项相加计算；两者分别保留。
                model.CorrectTssbnStepStates(
                    attemptedTimeStep, baseCoefficients_.firstStageTime, baseCoefficients_.secondStageTime,
                    embeddedCoefficients_.lastStageTime, firstWeight, embeddedCoefficients_.firstWeight,
                    embeddedCoefficients_.secondWeight, embeddedCoefficients_.lastWeight,
                    embeddedCoefficients_.lastStageFirstCoefficient, embeddedCoefficients_.lastStageSecondCoefficient,
                    referenceAcceptedIncrement, referenceAcceptedVelocity, embeddedIncrement, embeddedVelocity,
                    acceptedAcceleration);
            }
            Vec referenceErrorStepStartDisplacement;
            bool useMixedReferenceDisplacementLte = false;
            if (parameters_.referenceGalloping && !referenceAbsoluteLteEligible && !hasActiveMpc)
            {
                referenceErrorStepStartDisplacement = previousDisplacement;
                useMixedReferenceDisplacementLte = model.CorrectReferenceTssbnErrorDisplacements(
                    attemptedTimeStep, firstWeight, embeddedCoefficients_.firstWeight,
                    embeddedCoefficients_.secondWeight, embeddedCoefficients_.lastWeight,
                    embeddedCoefficients_.lastStageFirstCoefficient, embeddedCoefficients_.lastStageSecondCoefficient,
                    referenceErrorStepStartDisplacement, referenceOrder2Displacement, referenceOrder3Displacement);
            }
            // 原生四自由度沿用完整状态；混合节点仅替换参考 LTE 的材料增量和主转角。
            const bool useReferenceDisplacementLte =
                (referenceAbsoluteLteEligible || useMixedReferenceDisplacementLte) && !hasActiveMpc;
            const Vec& displacementStepStartState =
                useMixedReferenceDisplacementLte ? referenceErrorStepStartDisplacement : previousDisplacement;
            const Vec displacementScaleState = useReferenceDisplacementLte
                                                   ? referenceOrder2Displacement
                                                   : (previousDisplacement + baseIncrement).eval();
            Vec displacementDifference =
                useReferenceDisplacementLte ? (referenceOrder2Displacement - referenceOrder3Displacement).eval()
                : parameters_.referenceGalloping
                    ? ((previousDisplacement + baseIncrement) - (previousDisplacement + embeddedIncrement)).eval()
                    : (baseIncrement - embeddedIncrement).eval();
            const bool useStableEmbeddedDifferences = !parameters_.referenceGalloping && !hasActiveMpc;
            if (useStableEmbeddedDifferences)
            {
                const double gamma = baseCoefficients_.firstStageTime;
                const double errorCoefficient =
                    (6.0 * gamma * (1.0 - gamma) - 1.0) / (6.0 * (1.0 - 2.0 * gamma));
                // 普通平移采用代数等价的局部误差，避免大速度积分量相减；转动保留其群映射误差。
                for (int dof = 0; dof < dofCount; ++dof)
                    if (!finiteRotationDofs[static_cast<std::size_t>(dof)])
                        displacementDifference[dof] = errorCoefficient * attemptedTimeStep * attemptedTimeStep *
                                                      (secondStage.acceleration[dof] - firstStage.acceleration[dof]);
            }
            const double displacementError = EstimateErrorComponent(model, displacementDifference,
                                                                    displacementStepStartState, displacementScaleState);
            // LegacyGlobal 合并位移与速度的二范数；位移分量独自超过 sqrt(2) 才能保证拒绝。
            const double displacementLowerBound = (parameters_.errorNormMode == TssbnErrorNormMode::LegacyGlobal ||
                                                   parameters_.errorNormMode == TssbnErrorNormMode::PaperMixedRms)
                                                      ? displacementError / std::sqrt(2.0)
                                                      : displacementError;
            const bool skipExplicitAcceleration =
                useEmbeddedStepError && !parameters_.referenceGalloping && !hasActiveMpc && displacementLowerBound > 1.0;
            Vec explicitAcceleration;
            if (skipExplicitAcceleration)
            {
                ++explicitLastStageDisplacementSkipCount_;
            }
            else if (useEmbeddedStepError && !hasActiveMpc)
            {
                QElapsedTimer y3Timer;
                if (parameters_.diagnosticErrorTrace)
                    y3Timer.start();
                bool explicitAccelerationSolved = SolveExplicitLastStageAcceleration(
                    model, currentTime + attemptedTimeStep, explicitLastStage, explicitAcceleration);
                if (parameters_.diagnosticErrorTrace)
                    diagnosticY3Milliseconds_ += static_cast<double>(y3Timer.nsecsElapsed()) / 1.0e6;
                if (explicitAccelerationSolved)
                {
                    embeddedVelocity =
                        previousVelocity +
                        attemptedTimeStep * (embeddedCoefficients_.firstWeight * firstStage.acceleration +
                                             embeddedCoefficients_.secondWeight * secondStage.acceleration +
                                             embeddedCoefficients_.lastWeight * explicitAcceleration);
                    model.CorrectTssbnEmbeddedVelocity(
                        attemptedTimeStep, embeddedCoefficients_.firstWeight, embeddedCoefficients_.secondWeight,
                        embeddedCoefficients_.lastWeight, embeddedCoefficients_.lastStageFirstCoefficient,
                        embeddedCoefficients_.lastStageSecondCoefficient, explicitAcceleration, embeddedVelocity);
                    model.RollbackDynamicStep();
                    explicitAccelerationSolved = embeddedVelocity.size() == dofCount && embeddedVelocity.allFinite();
                    if (!explicitAccelerationSolved)
                        lastStageFailureReason_ = QStringLiteral("真实显式末阶段旋转嵌入速度尺寸或数值无效");
                }
                if (!explicitAccelerationSolved)
                {
                    lastFailedOperation = QStringLiteral("真实显式末阶段加速度");
                    model.RollbackDynamicStep();
                    ++rejectedStepCount_;
                    if (attemptedTimeStep <= parameters_.minimumTimeStep * (1.0 + 1.0e-9))
                    {
                        const QString failureMessage = QStringLiteral("自适应 TSSBN 在 t=%1 的 %2 失败，"
                                                                      "最小 dt=%3：%4")
                                                           .arg(currentTime, 0, 'g', 10)
                                                           .arg(lastFailedOperation)
                                                           .arg(attemptedTimeStep, 0, 'g', 10)
                                                           .arg(lastStageFailureReason_);
                        model.ReportProgress(currentTime / duration, failureMessage);
                        qDebug().noquote() << failureMessage;
                        return false;
                    }
                    attemptedTimeStep =
                        std::max(parameters_.minimumTimeStep, attemptedTimeStep * parameters_.shrinkFactor);
                    if (parameters_.diagnosticErrorTrace)
                        diagnosticRejectedAttemptMilliseconds_ +=
                            static_cast<double>(attemptTimer.nsecsElapsed()) / 1.0e6;
                    continue;
                }
            }

            // 参考 MPC 保留三点速度探针；无 MPC 的速度估计使用真实 Y3 右端，避免外推差恒零。
            const bool useVelocityProbe = useEmbeddedStepError && hasActiveMpc;
            Vec quadratureVelocity;
            if (useVelocityProbe)
            {
                quadratureVelocity =
                    previousVelocity + attemptedTimeStep * (velocityQuadratureWeights_[0] * previousAcceleration +
                                                            velocityQuadratureWeights_[1] * firstStage.acceleration +
                                                            velocityQuadratureWeights_[2] * secondStage.acceleration);
            }
            Vec velocityDifference = baseVelocity - embeddedVelocity;
            if (useStableEmbeddedDifferences && explicitAcceleration.size() == dofCount)
            {
                for (int dof = 0; dof < dofCount; ++dof)
                    if (!finiteRotationDofs[static_cast<std::size_t>(dof)])
                        velocityDifference[dof] = attemptedTimeStep * embeddedCoefficients_.lastWeight *
                                                  (explicitLastStage.acceleration[dof] - explicitAcceleration[dof]);
            }
            double unprojectedError = useEmbeddedStepError ? displacementLowerBound : 0.0;
            if (useEmbeddedStepError && !skipExplicitAcceleration)
            {
                if (useReferenceDisplacementLte)
                {
                    const double velocityError = EstimateErrorComponent(model, baseVelocity - embeddedVelocity,
                                                                        previousVelocity, baseVelocity, true);
                    if (parameters_.errorNormMode == TssbnErrorNormMode::LegacyGlobal ||
                        parameters_.errorNormMode == TssbnErrorNormMode::PaperMixedRms)
                    {
                        const double squaredError =
                            displacementError * displacementError + velocityError * velocityError;
                        unprojectedError = std::sqrt(0.5) * std::sqrt(squaredError);
                    }
                    else
                    {
                        unprojectedError = std::max(displacementError, velocityError);
                    }
                }
                else
                {
                    unprojectedError =
                        EstimateNormalizedError(model, baseIncrement, embeddedIncrement, baseVelocity, embeddedVelocity,
                                                previousDisplacement, previousVelocity,
                                                useStableEmbeddedDifferences ? &displacementDifference : nullptr,
                                                useStableEmbeddedDifferences ? &velocityDifference : nullptr);
                }
            }
            if (useVelocityProbe)
            {
                unprojectedError = std::max(
                    unprojectedError, ScaledVectorNorm(baseVelocity - quadratureVelocity, previousVelocity,
                                                       baseVelocity, parameters_.velocityAbsoluteTolerance,
                                                       parameters_.relativeTolerance, parameters_.errorNormMode, true));
            }
            if (!std::isfinite(unprojectedError))
            {
                model.RollbackDynamicStep();
                return false;
            }

            double normalizedError = unprojectedError;
            if (unprojectedError <= 1.0)
            {
                // 阶段状态满足 MPC，并不能保证由阶段加权得到的端点仍在约束流形上。
                // 只为可能接受的试步建立端点；投影后的状态也参与 LTE，避免约束修正被误差估计遗漏。
                const bool endpointEstablished =
                    parameters_.referenceGalloping && !hasActiveMpc
                        ? EstablishAcceptedState(model, referenceAcceptedIncrement, referenceAcceptedVelocity,
                                                 acceptedAcceleration)
                        : EstablishAcceptedState(model, baseIncrement, baseVelocity, acceptedAcceleration);
                if (!endpointEstablished || hasActiveMpc)
                {
                    if (!endpointEstablished ||
                        !ProjectAcceptedMpcState(model, baseIncrement, baseVelocity, acceptedAcceleration))
                    {
                        lastFailedOperation = endpointEstablished ? QStringLiteral("端点 MPC 投影")
                                                                  : QStringLiteral("接受端点构型恢复");
                        model.RollbackDynamicStep();
                        ++rejectedStepCount_;
                        if (attemptedTimeStep <= parameters_.minimumTimeStep * (1.0 + 1.0e-9))
                        {
                            const QString failureMessage = QStringLiteral("自适应 TSSBN 在 t=%1 的%2失败，"
                                                                          "最小 dt=%3：%4")
                                                               .arg(currentTime, 0, 'g', 10)
                                                               .arg(lastFailedOperation)
                                                               .arg(attemptedTimeStep, 0, 'g', 10)
                                                               .arg(lastStageFailureReason_);
                            model.ReportProgress(currentTime / duration, failureMessage);
                            qDebug().noquote() << failureMessage;
                            return false;
                        }
                        attemptedTimeStep =
                            std::max(parameters_.minimumTimeStep, attemptedTimeStep * parameters_.shrinkFactor);
                        if (parameters_.diagnosticErrorTrace)
                            diagnosticRejectedAttemptMilliseconds_ +=
                                static_cast<double>(attemptTimer.nsecsElapsed()) / 1.0e6;
                        continue;
                    }
                    if (useEmbeddedStepError)
                    {
                        double projectedError = EstimateNormalizedError(
                            model, baseIncrement, embeddedIncrement, baseVelocity, embeddedVelocity,
                            previousDisplacement, previousVelocity);
                        if (useVelocityProbe)
                            projectedError = std::max(
                                projectedError,
                                ScaledVectorNorm(baseVelocity - quadratureVelocity, previousVelocity, baseVelocity,
                                                 parameters_.velocityAbsoluteTolerance, parameters_.relativeTolerance,
                                                 parameters_.errorNormMode, true));
                        normalizedError = std::max(unprojectedError, projectedError);
                    }
                }
            }
            lastNormalizedError = normalizedError;
            if (!std::isfinite(normalizedError))
            {
                model.RollbackDynamicStep();
                return false;
            }
            ScaledErrorDiagnostic telemetryDisplacementDiagnostic;
            ScaledErrorDiagnostic telemetryVelocityDiagnostic;
            ScaledErrorDiagnostic telemetryProbeDiagnostic;
            if (parameters_.diagnosticErrorTrace && useEmbeddedStepError)
            {
                diagnosticMaximumNormalizedError = std::max(diagnosticMaximumNormalizedError, normalizedError);
                telemetryDisplacementDiagnostic = DiagnoseScaledError(
                    displacementDifference, displacementStepStartState,
                    useReferenceDisplacementLte ? displacementScaleState
                                                : (previousDisplacement + baseIncrement).eval(),
                    parameters_.absoluteTolerance, parameters_.relativeTolerance, parameters_.errorNormMode);
                telemetryDisplacementDiagnostic.normalizedError = displacementError;
                if (telemetryDisplacementDiagnostic.normalizedError > diagnosticMaximumDisplacementError)
                {
                    diagnosticMaximumDisplacementError = telemetryDisplacementDiagnostic.normalizedError;
                    diagnosticMaximumDisplacementDof =
                        model.DescribeFreeDof(static_cast<int>(telemetryDisplacementDiagnostic.worstDof));
                }
                // 位移 LTE 已保证拒绝时，嵌入速度还没有用真实 Y3 加速度更新，不能作为有效诊断。
                if (!skipExplicitAcceleration)
                {
                    telemetryVelocityDiagnostic =
                        DiagnoseScaledError(useStableEmbeddedDifferences ? velocityDifference
                                                                         : (baseVelocity - embeddedVelocity).eval(),
                                            previousVelocity, baseVelocity,
                                            parameters_.velocityAbsoluteTolerance, parameters_.relativeTolerance,
                                            parameters_.errorNormMode, true);
                    telemetryVelocityDiagnostic.normalizedError = EstimateErrorComponent(
                        model, useStableEmbeddedDifferences ? velocityDifference
                                                             : (baseVelocity - embeddedVelocity).eval(),
                        previousVelocity, baseVelocity, true);
                    if (telemetryVelocityDiagnostic.normalizedError > diagnosticMaximumVelocityError)
                    {
                        diagnosticMaximumVelocityError = telemetryVelocityDiagnostic.normalizedError;
                        diagnosticMaximumVelocityDof =
                            model.DescribeFreeDof(static_cast<int>(telemetryVelocityDiagnostic.worstDof));
                    }
                    if (useVelocityProbe)
                        telemetryProbeDiagnostic =
                            DiagnoseScaledError(baseVelocity - quadratureVelocity, previousVelocity, baseVelocity,
                                                parameters_.velocityAbsoluteTolerance, parameters_.relativeTolerance,
                                                parameters_.errorNormMode, true);
                }
            }

            if (parameters_.diagnosticErrorTrace && parameters_.referenceGalloping && acceptedStepCount_ < 3)
            {
                QTextStream trace(stdout);
                trace.setRealNumberPrecision(17);
                trace << "tssbn_reference_attempt time=" << currentTime << " dt=" << attemptedTimeStep
                      << " iterations=" << firstStage.newtonIterations + secondStage.newtonIterations - 2
                      << " base_displacement_norm="
                      << (useReferenceDisplacementLte ? model.GetReferenceVectorNorm(displacementStepStartState)
                                                      : previousDisplacement.norm())
                      << " next_displacement_norm="
                      << (useReferenceDisplacementLte ? model.GetReferenceVectorNorm(displacementScaleState)
                                                      : (previousDisplacement + baseIncrement).norm())
                      << " displacement_difference_norm="
                      << (useReferenceDisplacementLte ? model.GetReferenceVectorNorm(displacementDifference)
                                                      : displacementDifference.norm())
                      << " displacement_scaled=" << displacementError << " velocity_difference_norm="
                      << (useReferenceDisplacementLte ? model.GetReferenceVectorNorm(baseVelocity - embeddedVelocity)
                                                      : (baseVelocity - embeddedVelocity).norm())
                      << " lte=" << normalizedError << Qt::endl;
            }

            if (normalizedError > 1.0)
            {
                lastFailedOperation = QStringLiteral("LTE");
                lastStageFailureReason_ = QStringLiteral("局部误差 LTE 未满足容限");
                if (attemptedTimeStep <= parameters_.minimumTimeStep * (1.0 + 1.0e-9))
                {
                    const ScaledErrorDiagnostic displacementDiagnostic = DiagnoseScaledError(
                        useReferenceDisplacementLte || useStableEmbeddedDifferences
                            ? displacementDifference : (baseIncrement - embeddedIncrement).eval(),
                        displacementStepStartState,
                        useReferenceDisplacementLte ? displacementScaleState
                                                    : (previousDisplacement + baseIncrement).eval(),
                        parameters_.absoluteTolerance, parameters_.relativeTolerance, parameters_.errorNormMode);
                    const ScaledErrorDiagnostic embeddedVelocityDiagnostic =
                        DiagnoseScaledError(useStableEmbeddedDifferences ? velocityDifference
                                                                         : (baseVelocity - embeddedVelocity).eval(),
                                            previousVelocity, baseVelocity,
                                            parameters_.velocityAbsoluteTolerance, parameters_.relativeTolerance,
                                            parameters_.errorNormMode, true);
                    ScaledErrorDiagnostic probeError;
                    QString probeDiagnostic = skipExplicitAcceleration
                                                  ? QStringLiteral("位移 LTE 已保证拒绝，跳过后续误差评估")
                                                  : QStringLiteral("非 MPC 使用真实 Y3 加速度，未使用三点速度探针");
                    if (useVelocityProbe)
                    {
                        probeError =
                            DiagnoseScaledError(baseVelocity - quadratureVelocity, previousVelocity, baseVelocity,
                                                parameters_.velocityAbsoluteTolerance, parameters_.relativeTolerance,
                                                parameters_.errorNormMode, true);
                        probeDiagnostic =
                            FormatScaledErrorDiagnostic(QStringLiteral("三点速度探针"), probeError, model);
                        const Eigen::Index dof = probeError.worstDof;
                        if (dof >= 0 && dof < previousAcceleration.size() && dof < firstStage.acceleration.size() &&
                            dof < secondStage.acceleration.size() && dof < acceptedAcceleration.size())
                        {
                            probeDiagnostic +=
                                QStringLiteral("；探针最坏自由度加速度 a_n=%1，a_c1=%2，a_c2=%3，a_accepted=%4，"
                                               "探针速度差/dt=%5，上一接受 dt=%6，C1/C2 Newton=%7/%8")
                                    .arg(previousAcceleration[dof], 0, 'g', 12)
                                    .arg(firstStage.acceleration[dof], 0, 'g', 12)
                                    .arg(secondStage.acceleration[dof], 0, 'g', 12)
                                    .arg(acceptedAcceleration[dof], 0, 'g', 12)
                                    .arg(probeError.difference / attemptedTimeStep, 0, 'g', 12)
                                    .arg(previousAcceptedTimeStep, 0, 'g', 12)
                                    .arg(firstStage.newtonIterations)
                                    .arg(secondStage.newtonIterations);
                        }
                    }
                    const QString errorBreakdown =
                        FormatScaledErrorDiagnostic(QStringLiteral("位移嵌入"), displacementDiagnostic, model) +
                        QStringLiteral("；") +
                        FormatScaledErrorDiagnostic(QStringLiteral("速度嵌入"), embeddedVelocityDiagnostic, model) +
                        QStringLiteral("；") + probeDiagnostic;
                    model.RollbackDynamicStep();
                    QString acceptedStateResidualDiagnostic = QStringLiteral("；接受态动力残差=未核查");
                    if (probeError.worstDof >= 0 && probeError.worstDof < dofCount)
                    {
                        try
                        {
                            SystemEvaluationResult evaluation;
                            model.EvaluateSystem(SystemEvaluationRequest::ExplicitDynamics(currentTime, true),
                                                 evaluation);
                            Vec acceptedResidual;
                            model.ComputeResidual(evaluation.externalForce, acceptedResidual);
                            const Eigen::Index dof = probeError.worstDof;
                            if (acceptedResidual.size() == dofCount && evaluation.mass.rows() == dofCount &&
                                evaluation.mass.cols() == dofCount && evaluation.externalForce.size() == dofCount &&
                                firstStage.acceleration.size() == dofCount && previousAcceleration.size() == dofCount)
                            {
                                const double massDiagonal = evaluation.mass.coeff(dof, dof);
                                const double diagonalAccelerationCorrection =
                                    std::abs(massDiagonal) > 1.0e-20 ? acceptedResidual[dof] / massDiagonal
                                                                     : std::numeric_limits<double>::quiet_NaN();
                                const Vec accelerationGap = firstStage.acceleration - previousAcceleration;
                                const Vec massTimesAccelerationGap = evaluation.mass * accelerationGap;
                                Eigen::Index worstResidualDof = -1;
                                acceptedResidual.cwiseAbs().maxCoeff(&worstResidualDof);
                                Eigen::Index worstMassGapDof = -1;
                                massTimesAccelerationGap.cwiseAbs().maxCoeff(&worstMassGapDof);
                                struct MassRowTerm
                                {
                                    Eigen::Index column = -1;
                                    double mass = 0.0;
                                    double accelerationGap = 0.0;
                                };
                                std::vector<MassRowTerm> massRowTerms;
                                for (Eigen::Index column = 0; column < dofCount; ++column)
                                {
                                    const double value = evaluation.mass.coeff(dof, column);
                                    if (value != 0.0)
                                        massRowTerms.push_back({column, value, accelerationGap[column]});
                                }
                                std::sort(massRowTerms.begin(), massRowTerms.end(),
                                          [](const MassRowTerm& first, const MassRowTerm& second)
                                          {
                                              return std::abs(first.mass * first.accelerationGap) >
                                                     std::abs(second.mass * second.accelerationGap);
                                          });
                                QStringList dominantMassTerms;
                                for (std::size_t index = 0; index < std::min<std::size_t>(8, massRowTerms.size());
                                     ++index)
                                {
                                    const MassRowTerm& term = massRowTerms[index];
                                    dominantMassTerms << QStringLiteral("[%1：M=%2，Δa=%3，积=%4]")
                                                             .arg(model.DescribeFreeDof(static_cast<int>(term.column)))
                                                             .arg(term.mass, 0, 'g', 9)
                                                             .arg(term.accelerationGap, 0, 'g', 9)
                                                             .arg(term.mass * term.accelerationGap, 0, 'g', 9);
                                }
                                acceptedStateResidualDiagnostic =
                                    QStringLiteral("；接受态动力残差 %1：R_i=%2，||R||2=%3，||Fext||2=%4，"
                                                   "M_ii=%5，R_i/M_ii(仅对角粗估)=%6，"
                                                   "[M(a_c1-a_n)]_i=%7，||MΔa||2=%8，"
                                                   "最大残差=%9(%10)，最大 MΔa=%11(%12)，质量行主要项=%13")
                                        .arg(model.DescribeFreeDof(static_cast<int>(dof)))
                                        .arg(acceptedResidual[dof], 0, 'g', 12)
                                        .arg(acceptedResidual.norm(), 0, 'g', 12)
                                        .arg(evaluation.externalForce.norm(), 0, 'g', 12)
                                        .arg(massDiagonal, 0, 'g', 12)
                                        .arg(diagonalAccelerationCorrection, 0, 'g', 12)
                                        .arg(massTimesAccelerationGap[dof], 0, 'g', 12)
                                        .arg(massTimesAccelerationGap.norm(), 0, 'g', 12)
                                        .arg(model.DescribeFreeDof(static_cast<int>(worstResidualDof)))
                                        .arg(acceptedResidual[worstResidualDof], 0, 'g', 12)
                                        .arg(model.DescribeFreeDof(static_cast<int>(worstMassGapDof)))
                                        .arg(massTimesAccelerationGap[worstMassGapDof], 0, 'g', 12)
                                        .arg(dominantMassTerms.join(QStringLiteral("，")));
                            }
                            else
                            {
                                acceptedStateResidualDiagnostic =
                                    QStringLiteral("；接受态动力残差=返回向量/矩阵尺寸无效");
                            }
                        }
                        catch (const std::exception& exception)
                        {
                            acceptedStateResidualDiagnostic =
                                QStringLiteral("；接受态动力残差核查异常：%1").arg(exception.what());
                        }
                    }
                    const QString failureMessage =
                        QStringLiteral("自适应 TSSBN 在 t=%1 出现 LTE 失败，"
                                       "最小 dt=%2，归一化误差=%3，MPC=%4，投影前误差=%5；%6%7")
                            .arg(currentTime, 0, 'g', 10)
                            .arg(attemptedTimeStep, 0, 'g', 10)
                            .arg(normalizedError, 0, 'g', 8)
                            .arg(hasActiveMpc ? 1 : 0)
                            .arg(unprojectedError, 0, 'g', 8)
                            .arg(errorBreakdown)
                            .arg(acceptedStateResidualDiagnostic);
                    model.ReportProgress(currentTime / duration, failureMessage);
                    qDebug().noquote() << failureMessage;
                    return false;
                }
                model.RollbackDynamicStep();
                ++rejectedStepCount_;
                attemptedTimeStep = ProposeRejectedTimeStep(attemptedTimeStep, normalizedError);
                if (parameters_.diagnosticErrorTrace)
                    diagnosticRejectedAttemptMilliseconds_ += static_cast<double>(attemptTimer.nsecsElapsed()) / 1.0e6;
                continue;
            }

            const auto recordAcceptedEmbeddedDiagnostics = [&]()
            {
                if (!parameters_.diagnosticErrorTrace || !useEmbeddedStepError)
                    return;
                if (telemetryVelocityDiagnostic.normalizedError >= telemetryDisplacementDiagnostic.normalizedError)
                    ++diagnosticAcceptedVelocityDominantCount_;
                else
                    ++diagnosticAcceptedDisplacementDominantCount_;

                if (normalizedError > diagnosticMaximumAcceptedNormalizedError)
                {
                    diagnosticMaximumAcceptedNormalizedError = normalizedError;
                    const ScaledErrorDiagnostic* governingDiagnostic = &telemetryDisplacementDiagnostic;
                    diagnosticMaximumAcceptedComponent = QStringLiteral("displacement");
                    if (telemetryVelocityDiagnostic.normalizedError >= governingDiagnostic->normalizedError)
                    {
                        governingDiagnostic = &telemetryVelocityDiagnostic;
                        diagnosticMaximumAcceptedComponent = QStringLiteral("velocity");
                    }
                    if (useVelocityProbe &&
                        telemetryProbeDiagnostic.normalizedError >= governingDiagnostic->normalizedError)
                    {
                        governingDiagnostic = &telemetryProbeDiagnostic;
                        diagnosticMaximumAcceptedComponent = QStringLiteral("velocity_probe");
                    }
                    diagnosticMaximumAcceptedDof =
                        model.DescribeFreeDof(static_cast<int>(governingDiagnostic->worstDof));
                }

                const int nextAcceptedStep = acceptedStepCount_ + 1;
                // 末尾及检查点前逐步记录 Y3 相关误差，便于检查稀疏采样未显示的转角振动。
                const double acceptedTime = currentTime + attemptedTimeStep;
                const bool traceTailStep = duration - acceptedTime <= 0.1 ||
                                           (checkpointWriter_ && checkpointTargetTime_ - acceptedTime <= 0.1);
                if (nextAcceptedStep <= 20 || nextAcceptedStep % 100 == 0 || traceTailStep)
                {
                    const ScaledErrorDiagnostic& displacementDiagnostic = telemetryDisplacementDiagnostic;
                    const ScaledErrorDiagnostic& velocityDiagnostic = telemetryVelocityDiagnostic;
                    QTextStream trace(stdout);
                    trace << "tssbn_lte_sample step=" << nextAcceptedStep << " time=" << currentTime + attemptedTimeStep
                          << " dt=" << attemptedTimeStep << " displacement=" << displacementDiagnostic.normalizedError
                          << " velocity=" << velocityDiagnostic.normalizedError
                          << " newton1=" << firstStage.newtonIterations << " newton2=" << secondStage.newtonIterations
                          << " c1_residual_norm=" << firstStage.finalResidualNorm
                          << " c1_residual_limit=" << firstStage.finalResidualLimit
                          << " c2_residual_norm=" << secondStage.finalResidualNorm
                          << " c2_residual_limit=" << secondStage.finalResidualLimit
                          << " rejected_total=" << rejectedStepCount_
                          << " worst_velocity_difference=" << velocityDiagnostic.difference
                          << " worst_velocity_scale=" << velocityDiagnostic.scale << " worst_displacement="
                          << model.DescribeFreeDof(static_cast<int>(displacementDiagnostic.worstDof))
                          << " worst_velocity=" << model.DescribeFreeDof(static_cast<int>(velocityDiagnostic.worstDof));
                    const Eigen::Index worstVelocityDof = velocityDiagnostic.worstDof;
                    if (worstVelocityDof >= 0 && worstVelocityDof < explicitAcceleration.size())
                    {
                        if (firstStage.finalResidual.size() == dofCount && secondStage.finalResidual.size() == dofCount)
                            trace << " c1_residual_i=" << firstStage.finalResidual[worstVelocityDof]
                                  << " c2_residual_i=" << secondStage.finalResidual[worstVelocityDof];
                        trace << " base_velocity=" << baseVelocity[worstVelocityDof]
                              << " embedded_velocity=" << embeddedVelocity[worstVelocityDof]
                              << " a3_extrapolated=" << explicitLastStage.acceleration[worstVelocityDof]
                              << " a3_actual=" << explicitAcceleration[worstVelocityDof] << " a3_correction="
                              << explicitAcceleration[worstVelocityDof] -
                                     explicitLastStage.acceleration[worstVelocityDof];
                    }
                    trace << Qt::endl;
                    if (checkpointWriter_ && currentTime < checkpointTargetTime_ &&
                        acceptedTime >= checkpointTargetTime_ && explicitAcceleration.size() == dofCount)
                    {
                        QTextStream vectorTrace(stdout);
                        vectorTrace << "tssbn_y3_correction_vector time=" << QString::number(acceptedTime, 'g', 17)
                                    << " size=" << dofCount << " values=";
                        for (Eigen::Index index = 0; index < dofCount; ++index)
                            vectorTrace << (index == 0 ? "" : ",")
                                        << QString::number(explicitAcceleration[index] -
                                                               explicitLastStage.acceleration[index],
                                                           'g', 17);
                        vectorTrace << Qt::endl;
                        vectorTrace << "tssbn_embedded_velocity_difference_vector time="
                                    << QString::number(acceptedTime, 'g', 17) << " size=" << dofCount << " values=";
                        for (Eigen::Index index = 0; index < dofCount; ++index)
                            vectorTrace << (index == 0 ? "" : ",")
                                        << QString::number(baseVelocity[index] - embeddedVelocity[index], 'g', 17);
                        vectorTrace << Qt::endl;
                    }
                }
            };
            recordAcceptedEmbeddedDiagnostics();

            if (!parameters_.useReferenceExtrapolatedAcceleration)
            {
                // 恢复加速度必须使用实际接受的状态，不能重新提交仅用于 LTE 的运算顺序状态。
                Vec& acceptedIncrement =
                    parameters_.referenceGalloping && !hasActiveMpc ? referenceAcceptedIncrement : baseIncrement;
                const Vec& acceptedVelocity =
                    parameters_.referenceGalloping && !hasActiveMpc ? referenceAcceptedVelocity : baseVelocity;
                bool endpointRecovered = false;
                QElapsedTimer endpointTimer;
                if (parameters_.diagnosticErrorTrace)
                    endpointTimer.start();
                if (hasActiveMpc)
                {
                    endpointRecovered = RecoverAcceptedMpcAcceleration(model, currentTime + attemptedTimeStep,
                                                                       acceptedVelocity, acceptedAcceleration);
                }
                else
                {
                    StageState acceptedEndpoint;
                    acceptedEndpoint.stepIncrement = acceptedIncrement;
                    acceptedEndpoint.velocity = acceptedVelocity;
                    acceptedEndpoint.acceleration = acceptedAcceleration;
                    Vec recoveredAcceleration;
                    endpointRecovered = SolveExplicitLastStageAcceleration(
                        model, currentTime + attemptedTimeStep, acceptedEndpoint, recoveredAcceleration, true);
                    if (endpointRecovered)
                    {
                        acceptedIncrement = std::move(acceptedEndpoint.stepIncrement);
                        acceptedAcceleration = std::move(recoveredAcceleration);
                    }
                }
                if (parameters_.diagnosticErrorTrace)
                    diagnosticAcceptedEndpointMilliseconds_ +=
                        static_cast<double>(endpointTimer.nsecsElapsed()) / 1.0e6;
                if (!endpointRecovered)
                {
                    lastFailedOperation = QStringLiteral("接受端点加速度恢复");
                    model.RollbackDynamicStep();
                    ++rejectedStepCount_;
                    if (attemptedTimeStep <= parameters_.minimumTimeStep * (1.0 + 1.0e-9))
                    {
                        const QString failureMessage = QStringLiteral("自适应 TSSBN 在 t=%1 的%2失败，"
                                                                      "最小 dt=%3：%4")
                                                           .arg(currentTime, 0, 'g', 10)
                                                           .arg(lastFailedOperation)
                                                           .arg(attemptedTimeStep, 0, 'g', 10)
                                                           .arg(lastStageFailureReason_);
                        model.ReportProgress(currentTime / duration, failureMessage);
                        qDebug().noquote() << failureMessage;
                        return false;
                    }
                    attemptedTimeStep =
                        std::max(parameters_.minimumTimeStep, attemptedTimeStep * parameters_.shrinkFactor);
                    if (parameters_.diagnosticErrorTrace)
                        diagnosticRejectedAttemptMilliseconds_ +=
                            static_cast<double>(attemptTimer.nsecsElapsed()) / 1.0e6;
                    continue;
                }
            }

            int rotationCheckIterations = 0;
            bool fineStateAccepted = false;
            if (requiresIndependentStepError)
            {
                double rotationError = std::numeric_limits<double>::infinity();
                double displacementDiagnosticError = std::numeric_limits<double>::infinity();
                double velocityDiagnosticError = std::numeric_limits<double>::infinity();
                int displacementWorstDof = -1;
                int velocityWorstDof = -1;
                bool stateRestored = true;
                QElapsedTimer independentTimer;
                if (parameters_.diagnosticErrorTrace)
                    independentTimer.start();
                const bool errorCheckSucceeded = EstimateFiniteRotationStepError(
                    model, currentTime, attemptedTimeStep, previousDisplacement, previousVelocity, rotationError,
                    rotationCheckIterations, stateRestored, fineStateAccepted, displacementDiagnosticError,
                    velocityDiagnosticError, displacementWorstDof, velocityWorstDof);
                if (parameters_.diagnosticErrorTrace)
                {
                    ++diagnosticIndependentAttemptCount;
                    diagnosticIndependentMilliseconds += static_cast<double>(independentTimer.nsecsElapsed()) / 1.0e6;
                    diagnosticMaximumIndependentError = std::max(diagnosticMaximumIndependentError, rotationError);
                    diagnosticMaximumNormalizedError = std::max(diagnosticMaximumNormalizedError, rotationError);
                    if (errorCheckSucceeded && displacementDiagnosticError > diagnosticMaximumDisplacementError)
                    {
                        diagnosticMaximumDisplacementError = displacementDiagnosticError;
                        diagnosticMaximumDisplacementDof = model.DescribeFreeDof(displacementWorstDof);
                    }
                    if (errorCheckSucceeded && velocityDiagnosticError > diagnosticMaximumVelocityError)
                    {
                        diagnosticMaximumVelocityError = velocityDiagnosticError;
                        diagnosticMaximumVelocityDof = model.DescribeFreeDof(velocityWorstDof);
                    }
                    if (errorCheckSucceeded && rotationError <= 1.0)
                    {
                        const bool velocityDominant = velocityDiagnosticError >= displacementDiagnosticError;
                        if (velocityDominant)
                            ++diagnosticAcceptedVelocityDominantCount_;
                        else
                            ++diagnosticAcceptedDisplacementDominantCount_;
                        if (rotationError > diagnosticMaximumAcceptedNormalizedError)
                        {
                            diagnosticMaximumAcceptedNormalizedError = rotationError;
                            diagnosticMaximumAcceptedComponent =
                                velocityDominant ? QStringLiteral("velocity") : QStringLiteral("displacement");
                            diagnosticMaximumAcceptedDof =
                                model.DescribeFreeDof(velocityDominant ? velocityWorstDof : displacementWorstDof);
                        }
                    }
                }
                if (!stateRestored)
                {
                    model.ReportProgress(currentTime / duration, lastStageFailureReason_);
                    return false;
                }
                normalizedError = std::max(normalizedError, rotationError);
                lastNormalizedError = normalizedError;
                if (!errorCheckSucceeded || normalizedError > 1.0)
                {
                    model.RollbackDynamicStep();
                    ++rejectedStepCount_;
                    lastFailedOperation = QStringLiteral("有限转动/MPC 整步/两半步误差检查");
                    if (attemptedTimeStep <= parameters_.minimumTimeStep * (1.0 + 1.0e-9))
                    {
                        const QString message = QStringLiteral("自适应 TSSBN 在 t=%1 的%2失败，最小 dt=%3，"
                                                               "归一化误差=%4：%5")
                                                    .arg(currentTime, 0, 'g', 10)
                                                    .arg(lastFailedOperation)
                                                    .arg(attemptedTimeStep, 0, 'g', 10)
                                                    .arg(normalizedError, 0, 'g', 10)
                                                    .arg(lastStageFailureReason_);
                        model.ReportProgress(currentTime / duration, message);
                        return false;
                    }
                    attemptedTimeStep =
                        errorCheckSucceeded
                            ? ProposeRejectedTimeStep(attemptedTimeStep, normalizedError)
                            : std::max(parameters_.minimumTimeStep, attemptedTimeStep * parameters_.shrinkFactor);
                    if (parameters_.diagnosticErrorTrace)
                        diagnosticRejectedAttemptMilliseconds_ +=
                            static_cast<double>(attemptTimer.nsecsElapsed()) / 1.0e6;
                    continue;
                }
                // 细解包含两次半步提交的物理状态及历史；派生内力/惯性缓存按最终接受态刷新。
                SystemEvaluationResult restoredEndpoint;
                model.EvaluateSystem(SystemEvaluationRequest::TrialResidual(currentTime + attemptedTimeStep),
                                     restoredEndpoint);
            }
            if (ordinaryEmbedded)
            {
                // 物理端点恢复发布了新加速度；提交前同步该接受态的内力和惯性缓存。
                SystemEvaluationResult acceptedEndpoint;
                model.EvaluateSystem(SystemEvaluationRequest::TrialResidual(currentTime + attemptedTimeStep),
                                     acceptedEndpoint);
            }

            QElapsedTimer tailTimer;
            if (parameters_.diagnosticErrorTrace)
                tailTimer.start();
            // 两半步已完整提交节点及单元历史，不能再次提交或换回整步的局部数组。
            if (!fineStateAccepted)
                model.CommitState();
            currentTime += attemptedTimeStep;
            if (std::abs(duration - currentTime) <= completionTolerance)
                currentTime = duration;
            ++acceptedStepCount_;
            acceptedTimeStepSum += attemptedTimeStep;
            if (parameters_.diagnosticErrorTrace)
                diagnosticMinimumAcceptedTimeStep = std::min(diagnosticMinimumAcceptedTimeStep, attemptedTimeStep);
            previousAcceptedTimeStep = attemptedTimeStep;
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
            if (parameters_.diagnosticErrorTrace)
            {
                diagnosticAcceptedTailMilliseconds_ += static_cast<double>(tailTimer.nsecsElapsed()) / 1.0e6;
                diagnosticAcceptedAttemptMilliseconds_ += static_cast<double>(attemptTimer.nsecsElapsed()) / 1.0e6;
            }

            // 两阶段末轮只检查残差，没有 Newton 更新；性能因子与累计修正次数使用同一计数。
            nextTimeStep = ProposeAcceptedTimeStep(attemptedTimeStep, normalizedError, controlIterations);
            if (parameters_.diagnosticErrorTrace && currentTime + completionTolerance >= diagnosticNextWindowTime)
            {
                reportDiagnosticWindow();
                do
                {
                    diagnosticNextWindowTime += 1.0;
                } while (currentTime + completionTolerance >= diagnosticNextWindowTime);
            }
            if (checkpointWriter_ && !checkpointWritten_ && currentTime >= checkpointTargetTime_ - completionTolerance)
            {
                const CheckpointState state{currentTime,
                                            nextTimeStep,
                                            previousAcceptedError_,
                                            previousAcceptedTimeStep,
                                            acceptedTimeStepSum,
                                            acceptedStepCount_,
                                            rejectedStepCount_,
                                            totalStageNewtonIterations_,
                                            maximumStageNewtonIterations_};
                QString checkpointError;
                if (!checkpointWriter_(state, checkpointError))
                {
                    const QString message = QStringLiteral("自适应 TSSBN 检查点写入失败：%1").arg(checkpointError);
                    model.ReportProgress(currentTime / duration, message);
                    qDebug().noquote() << message;
                    return false;
                }
                checkpointWritten_ = true;
                stoppedAtCheckpoint_ = stopAfterCheckpoint_;
            }
            stepAccepted = true;
            break;
        }

        if (!stepAccepted)
        {
            model.RollbackDynamicStep();
            QString failureMessage = QStringLiteral("自适应 TSSBN 在 t=%1 达到拒绝重试上限，"
                                                          "最后 dt=%2，阶段=%3，原因=%4，"
                                                          "残差=%5（限值=%6），归一化误差=%7")
                                               .arg(currentTime, 0, 'g', 10)
                                               .arg(attemptedTimeStep, 0, 'g', 10)
                                               .arg(lastFailedOperation)
                                               .arg(lastStageFailureReason_)
                                               .arg(lastResidualNorm_, 0, 'g', 8)
                                               .arg(lastResidualLimit_, 0, 'g', 8)
                                               .arg(lastNormalizedError, 0, 'g', 8);
            if (!parameters_.referenceGalloping)
            {
                failureMessage = QStringLiteral("自适应 TSSBN 在 t=%1 达到拒绝重试上限，"
                                                "本轮 dt=%2，下一候选 dt=%3，阶段=%4，原因=%5，"
                                                "残差=%6（限值=%7），本轮归一化误差=%8")
                                     .arg(currentTime, 0, 'g', 10)
                                     .arg(lastEvaluatedTimeStep, 0, 'g', 10)
                                     .arg(attemptedTimeStep, 0, 'g', 10)
                                     .arg(lastFailedOperation)
                                     .arg(lastStageFailureReason_)
                                     .arg(lastResidualNorm_, 0, 'g', 8)
                                     .arg(lastResidualLimit_, 0, 'g', 8)
                                     .arg(std::isfinite(lastNormalizedError)
                                              ? QString::number(lastNormalizedError, 'g', 8)
                                              : QStringLiteral("未评估"));
            }
            model.ReportProgress(currentTime / duration, failureMessage);
            qDebug().noquote() << failureMessage;
            return false;
        }
        if (stoppedAtCheckpoint_)
            break;
    }

    if (parameters_.diagnosticErrorTrace)
        reportDiagnosticWindow();

    if (checkpointWriter_ && !checkpointWritten_)
    {
        model.ReportProgress(currentTime / duration, QStringLiteral("自适应 TSSBN 未到达检查点目标时间"));
        return false;
    }

    averageTimeStep_ = acceptedStepCount_ > 0 ? acceptedTimeStepSum / static_cast<double>(acceptedStepCount_) : 0.0;
    qDebug().noquote() << QStringLiteral("自适应 TSSBN 完成：接受=%1，拒绝=%2，平均 dt=%3")
                              .arg(acceptedStepCount_)
                              .arg(rejectedStepCount_)
                              .arg(averageTimeStep_, 0, 'g', 8);
    Logger::Instance().Diagnostic(QStringLiteral("自适应 TSSBN 迭代统计：平均 Newton=%1，最大 Newton=%2，"
                                                 "真实 Y3 评估=%3，质量求解=%4，位移预筛跳过=%5，"
                                                 "端点加速度恢复=%6")
                                      .arg(GetAverageStageNewtonIterations(), 0, 'g', 6)
                                      .arg(maximumStageNewtonIterations_)
                                      .arg(explicitLastStageEvaluationCount_)
                                      .arg(explicitLastStageMassSolveCount_)
                                      .arg(explicitLastStageDisplacementSkipCount_)
                                      .arg(acceptedEndpointEvaluationCount_));
    QString y3Diagnostic;
    QTextStream(&y3Diagnostic) << "tssbn_y3 evaluations=" << explicitLastStageEvaluationCount_
                               << " mass_solves=" << explicitLastStageMassSolveCount_
                               << " displacement_skips=" << explicitLastStageDisplacementSkipCount_
                               << " endpoint_evaluations=" << acceptedEndpointEvaluationCount_
                               << " endpoint_mass_solves=" << acceptedEndpointMassSolveCount_
                               << " endpoint_mpc_solves=" << acceptedEndpointMpcSolveCount_;
    Logger::Instance().Diagnostic(y3Diagnostic);
    if (parameters_.diagnosticErrorTrace && !Logger::DetailedOutputEnabled())
        QTextStream(stdout) << y3Diagnostic << Qt::endl;
    if (precisionLimitedStageCount_ > 0)
    {
        qWarning().noquote() << QStringLiteral("自适应 TSSBN 有 %1 个阶段达到浮点精度限制，已使用精度收敛判据。")
                                    .arg(precisionLimitedStageCount_);
        QString diagnostic;
        QTextStream(&diagnostic) << "tssbn_precision_summary stages=" << precisionLimitedStageCount_
                                 << " candidate_solves=" << precisionCandidateSolveCount_
                                 << " max_residual=" << precisionLimitedMaximumResidual_
                                 << " max_scaled_residual=" << precisionLimitedMaximumScaledResidual_
                                 << " max_scaled_correction=" << precisionLimitedMaximumCorrection_;
        Logger::Instance().Diagnostic(diagnostic);
        if (parameters_.diagnosticErrorTrace && !Logger::DetailedOutputEnabled())
            QTextStream(stdout) << diagnostic << Qt::endl;
    }
    if (parameters_.diagnosticErrorTrace)
    {
        if (!parameters_.referenceGalloping)
            QTextStream(stdout) << "tssbn_matrix_reuse fused_residual_tangent=" << diagnosticFusedResidualTangentCount_
                                << " reused_tangent=" << diagnosticReusedTangentCount_
                                << " unused_tangent="
                                << diagnosticFusedResidualTangentCount_ - diagnosticReusedTangentCount_ << Qt::endl;
        if (requiresIndependentStepError)
            QTextStream(stdout) << "tssbn_independent_error attempts=" << diagnosticIndependentAttemptCount
                                << " evaluation_ms=" << diagnosticIndependentMilliseconds
                                << " maximum=" << diagnosticMaximumIndependentError << Qt::endl;
        QTextStream(stdout) << "tssbn_lte_summary velocity_dominant=" << diagnosticAcceptedVelocityDominantCount_
                            << " displacement_dominant=" << diagnosticAcceptedDisplacementDominantCount_ << Qt::endl;
        QTextStream(stdout) << "tssbn_stage_residual_gate delayed=" << diagnosticLocalResidualDelayCount_
                            << " max_scaled=" << diagnosticMaximumLocalResidual_
                            << " tolerance=" << parameters_.nonlinearTolerance << Qt::endl;
        QTextStream(stdout) << "tssbn_cost_profile accepted_attempt_ms=" << diagnosticAcceptedAttemptMilliseconds_
                            << " rejected_attempt_ms=" << diagnosticRejectedAttemptMilliseconds_
                            << " stage1_ms=" << diagnosticFirstStageMilliseconds_
                            << " stage2_ms=" << diagnosticSecondStageMilliseconds_
                            << " stage_evaluation_ms=" << diagnosticStageEvaluationMilliseconds_
                            << " stage_evaluation_count=" << diagnosticStageEvaluationCount_
                            << " stage_linear_ms=" << diagnosticStageLinearSolveMilliseconds_
                            << " stage_linear_count=" << diagnosticStageLinearSolveCount_
                            << " y3_ms=" << diagnosticY3Milliseconds_
                            << " y3_evaluation_ms=" << diagnosticY3EvaluationMilliseconds_
                            << " y3_mass_ms=" << diagnosticY3MassSolveMilliseconds_
                            << " accepted_tail_ms=" << diagnosticAcceptedTailMilliseconds_ << Qt::endl;
    }
    if (parameters_.diagnosticCacheExternalForcePerStage)
    {
        QTextStream(stdout) << "tssbn_force_audit checks=" << diagnosticExternalForceAuditCount_
                            << " mismatches=" << diagnosticExternalForceAuditMismatchCount_ << Qt::endl;
    }
    return true;
}
}
