/**
 * @file TimeStepControl.cpp
 * @brief 时间步接受判定与步长调整策略。
 */
#include "TimeStepControl.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace SolverNameSpace
{
// 避免误差为零时计算 error 的负幂而溢出；不改变实际接受判定。
constexpr double minimumControllerError = 1.0e-14;

static bool IsFinitePositive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

TimeStepController::TimeStepController(AdaptiveTimeStepSettings settings)
    : settings_(settings)
{
    Validate();
}

void TimeStepController::Validate() const
{
    if (!IsFinitePositive(settings_.minimumTimeStep) || !IsFinitePositive(settings_.maximumTimeStep) ||
        settings_.minimumTimeStep > settings_.maximumTimeStep || !IsFinitePositive(settings_.relativeTolerance) ||
        !IsFinitePositive(settings_.absoluteTolerance) ||
        !(settings_.safetyFactor > 0.0 && settings_.safetyFactor <= 1.0) ||
        !(settings_.shrinkFactor > 0.0 && settings_.shrinkFactor < 1.0) ||
        !IsFinitePositive(settings_.maximumGrowthFactor) || settings_.maximumGrowthFactor < 1.0 ||
        settings_.maximumRejectedAttempts < 1)
    {
        throw std::invalid_argument("自适应时间步参数无效");
    }
}

TimeStepDecision TimeStepController::DecideAccepted(double currentTimeStep, double normalizedError,
                                                     int methodOrder) const
{
    if (methodOrder < 1 || !IsFinitePositive(currentTimeStep) || !std::isfinite(normalizedError) ||
        normalizedError < 0.0)
        return {false, currentTimeStep};

    // 局部截断误差为 O(dt^(p+1))，故反解目标步长时使用 -1/(p+1)。
    const double exponent = -1.0 / static_cast<double>(methodOrder + 1);
    const double factor =
        std::clamp(settings_.safetyFactor * std::pow(std::max(normalizedError, minimumControllerError), exponent),
                                     settings_.shrinkFactor, settings_.maximumGrowthFactor);
    return {normalizedError <= 1.0,
            std::clamp(currentTimeStep * factor, settings_.minimumTimeStep, settings_.maximumTimeStep)};
}

TimeStepDecision TimeStepController::DecideRejected(double currentTimeStep, double normalizedError,
                                                     int methodOrder) const
{
    if (methodOrder < 1 || !IsFinitePositive(currentTimeStep))
        return {false, currentTimeStep};

    // Newton 不收敛时没有可信误差，按固定缩步因子回退。
    double factor = settings_.shrinkFactor;
    if (std::isfinite(normalizedError) && normalizedError > 1.0)
    {
        const double exponent = -1.0 / static_cast<double>(methodOrder + 1);
        factor = std::min(settings_.shrinkFactor, settings_.safetyFactor * std::pow(normalizedError, exponent));
    }
    return {false, std::max(settings_.minimumTimeStep, currentTimeStep * factor)};
}
}
