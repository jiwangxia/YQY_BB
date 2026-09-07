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

double SelectTimeStep(double currentTime, double duration, double proposedTimeStep)
{
    const double remainingTime = duration - currentTime;
    constexpr double endTimeSnapRelativeTolerance = 1.0e-6;
    return remainingTime <= proposedTimeStep * (1.0 + endTimeSnapRelativeTolerance) ? remainingTime : proposedTimeStep;
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
        !IsFinitePositive(settings_.absoluteTolerance) || !IsFinitePositive(settings_.velocityRelativeTolerance) ||
        !IsFinitePositive(settings_.velocityAbsoluteTolerance) ||
        !(settings_.safetyFactor > 0.0 && settings_.safetyFactor <= 1.0) ||
        !(settings_.minimumScaleFactor > 0.0 && settings_.minimumScaleFactor < 1.0) ||
        !IsFinitePositive(settings_.maximumGrowthFactor) || settings_.maximumGrowthFactor < 1.0 ||
        !(settings_.solveFailureFactor > 0.0 && settings_.solveFailureFactor < 1.0) ||
        settings_.maximumRejectedAttempts < 1)
    {
        throw std::invalid_argument("自适应时间步参数无效");
    }
}

TimeStepDecision TimeStepController::Decide(double currentTimeStep, double normalizedError, int methodOrder) const
{
    if (methodOrder < 1 || !IsFinitePositive(currentTimeStep) || !std::isfinite(normalizedError) ||
        normalizedError < 0.0)
        return {false, currentTimeStep};

    // 局部截断误差为 O(dt^(p+1))，故反解目标步长时使用 -1/(p+1)。
    const double exponent = -1.0 / static_cast<double>(methodOrder + 1);
    const double factor =
        std::clamp(settings_.safetyFactor * std::pow(std::max(normalizedError, minimumControllerError), exponent),
                   settings_.minimumScaleFactor, settings_.maximumGrowthFactor);
    return {normalizedError <= 1.0,
            std::clamp(currentTimeStep * factor, settings_.minimumTimeStep, settings_.maximumTimeStep)};
}

TimeStepDecision TimeStepController::DecideSolveFailure(double currentTimeStep) const
{
    if (!IsFinitePositive(currentTimeStep))
        return {false, currentTimeStep};

    return {false, std::max(settings_.minimumTimeStep, currentTimeStep * settings_.solveFailureFactor)};
}
}
