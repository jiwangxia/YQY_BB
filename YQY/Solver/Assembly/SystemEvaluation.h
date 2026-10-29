#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include <cstdint>

namespace SolverNameSpace
{
/// @brief 当前系统评估需要计算的物理贡献。
enum class SystemContribution : std::uint32_t
{
    None = 0,
    Stiffness = 1U << 0U,
    Mass = 1U << 1U,
    Damping = 1U << 2U,
    InternalForce = 1U << 3U,
    InertiaForce = 1U << 4U,
    VelocityTangent = 1U << 5U,
    ConfigurationTangent = 1U << 6U,
    ExternalForce = 1U << 7U,
    ExternalTangent = 1U << 8U,
    EffectiveDynamicTangent = 1U << 9U,
    EffectiveTangent = 1U << 10U,
    ResidualState = 1U << 11U
};

using SystemContributionMask = std::uint32_t;

constexpr SystemContributionMask ToMask(SystemContribution contribution)
{
    return static_cast<SystemContributionMask>(contribution);
}

constexpr SystemContributionMask operator|(SystemContribution lhs, SystemContribution rhs)
{
    return ToMask(lhs) | ToMask(rhs);
}

/// @brief 时间积分器向系统评估层提出的统一请求。
struct SystemEvaluationRequest
{
    SystemContributionMask contributions = ToMask(SystemContribution::None);
    double time = 0.0;
    double loadFactor = 1.0;
    double accelerationDerivative = 0.0;
    double velocityDerivative = 0.0;

    static SystemEvaluationRequest EffectiveTangent(double time, double accelerationDerivative,
                                                     double velocityDerivative, bool includeExternalTangent)
    {
        SystemEvaluationRequest request;
        request.contributions = ToMask(SystemContribution::ExternalForce) |
                                ToMask(SystemContribution::EffectiveTangent);
        request.time = time;
        request.accelerationDerivative = accelerationDerivative;
        request.velocityDerivative = velocityDerivative;
        if (includeExternalTangent)
            request.Include(SystemContribution::ExternalTangent);
        return request;
    }

    static SystemEvaluationRequest ExternalTangent(double time, double velocityDerivative)
    {
        SystemEvaluationRequest request;
        request.contributions = ToMask(SystemContribution::ExternalTangent);
        request.time = time;
        request.velocityDerivative = velocityDerivative;
        return request;
    }

    static SystemEvaluationRequest TrialResidual(double time)
    {
        SystemEvaluationRequest request;
        request.contributions =
            ToMask(SystemContribution::ExternalForce) | ToMask(SystemContribution::ResidualState);
        request.time = time;
        return request;
    }

    static SystemEvaluationRequest EffectiveTangentOnly(double accelerationDerivative, double velocityDerivative)
    {
        SystemEvaluationRequest request;
        request.contributions = ToMask(SystemContribution::EffectiveTangent);
        request.accelerationDerivative = accelerationDerivative;
        request.velocityDerivative = velocityDerivative;
        return request;
    }

    static SystemEvaluationRequest ExplicitDynamics(double time, bool includeMass = true)
    {
        SystemEvaluationRequest request;
        request.contributions = ToMask(SystemContribution::ResidualState) |
                                ToMask(SystemContribution::ExternalForce);
        if (includeMass)
            request.Include(SystemContribution::Mass);
        request.time = time;
        return request;
    }

    void Include(SystemContribution contribution)
    {
        contributions |= ToMask(contribution);
    }

    bool Requires(SystemContribution contribution) const
    {
        return (contributions & ToMask(contribution)) != 0U;
    }
};

/// @brief 系统评估层返回给时间积分器的整体结果。
struct SystemEvaluationResult
{
    Eigen::SparseMatrix<double> stiffness;
    Eigen::SparseMatrix<double> mass;
    Eigen::SparseMatrix<double> damping;
    Eigen::SparseMatrix<double> velocityTangent;
    Eigen::SparseMatrix<double> configurationTangent;
    Eigen::SparseMatrix<double> effectiveDynamicTangent;
    Eigen::SparseMatrix<double> effectiveTangent;
    Eigen::VectorXd internalForce;
    Eigen::VectorXd inertiaForce;
    Eigen::VectorXd constrainedExternalForce;
    Eigen::VectorXd externalForce;
    Eigen::SparseMatrix<double> externalTangent;
    // 非零结构不变时由模型提供稳定版本号，0 表示调用方必须自行检查结构。
    std::uint64_t sparsePatternVersion = 0;
    const double* gpuCscValues = nullptr;
    int gpuCscValueCount = 0;
};
}
