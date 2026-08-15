#include "BundleAeroMapper.h"

#include <cmath>
#include <limits>
#include <stdexcept>

static double StartAngle(int bundleCount)
{
    const double pi = std::acos(-1.0);
    switch (bundleCount)
    {
        case 1:
            return 0.0;
        case 2:
            return 0.0;
        case 4:
            return -pi / 4.0;
        case 6:
            return -pi / 3.0;
        case 8:
            return -3.0 * pi / 8.0;
        default:
            return 0.0;
    }
}

static double PeriodicDistance(double first, double second)
{
    const double period = 2.0 * std::acos(-1.0);
    return std::abs(std::remainder(first - second, period));
}
int BundleAeroMapper::ResolveProfile(int bundleCount, int wireId, const Eigen::Vector3d& elementAxis,
                                     const Eigen::Vector3d& modelUp, const Eigen::Vector3d& windVelocityGlobal,
                                     double bundleTwistRadians)
{
    constexpr double tolerance = 1.0e-12;
    if (bundleCount <= 0 || wireId < 0 || wireId >= bundleCount)
        throw std::invalid_argument("Invalid bundle count or wire id");
    if (bundleCount == 1)
        return 0;

    Eigen::Vector3d axis = elementAxis.normalized();
    Eigen::Index dominant = 0;
    axis.cwiseAbs().maxCoeff(&dominant);
    if (axis[dominant] < 0.0)
        axis = -axis;
    Eigen::Vector3d up = modelUp - modelUp.dot(axis) * axis;
    if (!axis.allFinite() || up.norm() <= tolerance)
        throw std::invalid_argument("Model up direction must not be parallel to element axis");
    up.normalize();
    const Eigen::Vector3d right = axis.cross(up).normalized();

    Eigen::Vector3d normalWind = windVelocityGlobal - windVelocityGlobal.dot(axis) * axis;
    if (!normalWind.allFinite() || normalWind.norm() <= tolerance)
        return wireId;
    normalWind.normalize();

    const double windAngle = std::atan2(normalWind.dot(up), normalWind.dot(right));
    const double twoPi = 2.0 * std::acos(-1.0);
    const double step = twoPi / bundleCount;
    const double physicalAngle = StartAngle(bundleCount) + step * wireId + bundleTwistRadians;
    const double windRelativeAngle = physicalAngle - windAngle;

    int bestProfile = 0;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (int profile = 0; profile < bundleCount; ++profile)
    {
        const double referenceAngle = StartAngle(bundleCount) + step * profile;
        const double distance = PeriodicDistance(windRelativeAngle, referenceAngle);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            bestProfile = profile;
        }
    }
    return bestProfile;
}
