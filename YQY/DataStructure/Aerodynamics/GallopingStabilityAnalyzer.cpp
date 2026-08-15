#include "GallopingStabilityAnalyzer.h"

#include <algorithm>
#include <cmath>

static constexpr double kRadiansPerDegree = 0.01745329251994329576923690768489;
static constexpr double kTolerance = 1.0e-12;

static double ZeroCrossing(double firstAngle, double secondAngle, double firstValue, double secondValue)
{
    const double difference = secondValue - firstValue;
    if (std::abs(difference) <= kTolerance)
        return firstAngle;
    return firstAngle + (secondAngle - firstAngle) * (-firstValue / difference);
}
std::vector<GallopingInstabilityInterval> GallopingStabilityAnalyzer::FindNegativeDampingIntervals(
    const std::vector<BladeModel>& models, double angleStepDegrees)
{
    std::vector<GallopingInstabilityInterval> intervals;
    if (!(angleStepDegrees > 0.0) || !std::isfinite(angleStepDegrees))
        return intervals;

    const double angleStepRadians = angleStepDegrees * kRadiansPerDegree;
    for (int profileId = 0; profileId < static_cast<int>(models.size()); ++profileId)
    {
        const BladeModel& model = models[profileId];
        const std::size_t count = std::min(model.lift.size(), model.drag.size());
        if (count < 2)
            continue;

        for (std::size_t index = 0; index + 1 < count; ++index)
        {
            const double liftSlope = (model.lift[index + 1] - model.lift[index]) / angleStepRadians;
            const double firstH = model.drag[index] + liftSlope;
            const double secondH = model.drag[index + 1] + liftSlope;
            if (std::min(firstH, secondH) >= -kTolerance)
                continue;

            const double firstAngle = static_cast<double>(index) * angleStepDegrees;
            const double secondAngle = firstAngle + angleStepDegrees;
            const double negativeStart =
                firstH < 0.0 ? firstAngle : ZeroCrossing(firstAngle, secondAngle, firstH, secondH);
            const double negativeEnd =
                secondH < 0.0 ? secondAngle : ZeroCrossing(firstAngle, secondAngle, firstH, secondH);
            const double minimumH = std::min(firstH, secondH);

            if (!intervals.empty())
            {
                GallopingInstabilityInterval& previous = intervals.back();
                if (previous.profileId == profileId && std::abs(previous.endAngleDegrees - negativeStart) <= kTolerance)
                {
                    previous.endAngleDegrees = negativeEnd;
                    previous.minimumDenHartog = std::min(previous.minimumDenHartog, minimumH);
                    continue;
                }
            }
            intervals.push_back({profileId, negativeStart, negativeEnd, minimumH});
        }
    }
    return intervals;
}
