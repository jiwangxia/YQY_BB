/**
 * @file TimeStepError.cpp
 * @brief 时间积分步长加倍误差估计。
 */
#include "TimeStepError.h"
#include "TimeStepControl.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace SolverNameSpace
{
static double ScaledMaximumError(const Vec& difference, const Vec& referenceA, const Vec& referenceB,
                                 const AdaptiveTimeStepSettings& settings)
{
    if (difference.size() != referenceA.size() || difference.size() != referenceB.size())
        return std::numeric_limits<double>::infinity();
    if (difference.size() == 0)
        return 0.0;

    // 逐自由度按 yA、yB 的较大幅值缩放，避免不同自由度的误差在整体范数中相互抵消。
    double maximumError = 0.0;
    for (Eigen::Index index = 0; index < difference.size(); ++index)
    {
        if (!std::isfinite(difference[index]) || !std::isfinite(referenceA[index]) ||
            !std::isfinite(referenceB[index]))
            return std::numeric_limits<double>::infinity();

        const double scale = settings.absoluteTolerance +
                             settings.relativeTolerance * std::max(std::abs(referenceA[index]),
                                                                   std::abs(referenceB[index]));
        if (!std::isfinite(scale) || scale <= 0.0)
            return std::numeric_limits<double>::infinity();

        maximumError = std::max(maximumError, std::abs(difference[index]) / scale);
    }
    return maximumError;
}

double EstimateStepDoublingError(const Vec& coarseDisplacement, const Vec& coarseVelocity,
                                 const Vec& fineDisplacement, const Vec& fineVelocity,
                                 const AdaptiveTimeStepSettings& settings, int methodOrder)
{
    if (methodOrder < 1 || !std::isfinite(settings.absoluteTolerance) || settings.absoluteTolerance <= 0.0 ||
        !std::isfinite(settings.relativeTolerance) || settings.relativeTolerance <= 0.0)
        return std::numeric_limits<double>::infinity();

    // Richardson 步长加倍关系：细解误差约为 (fine - coarse) / (2^p - 1)。
    const double denominator = std::pow(2.0, static_cast<double>(methodOrder)) - 1.0;
    if (!std::isfinite(denominator) || denominator <= 0.0)
        return std::numeric_limits<double>::infinity();
    const double displacementError =
        ScaledMaximumError(fineDisplacement - coarseDisplacement, coarseDisplacement, fineDisplacement, settings) /
        denominator;
    if (!settings.includeVelocityInError)
        return displacementError;

    const double velocityError =
        ScaledMaximumError(fineVelocity - coarseVelocity, coarseVelocity, fineVelocity, settings) / denominator;
    return std::max(displacementError, velocityError);
}
}
