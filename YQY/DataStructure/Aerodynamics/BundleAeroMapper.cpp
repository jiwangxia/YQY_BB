#include "BundleAeroMapper.h"

#include "Utility/CR.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

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
void BundleAeroMapper::ValidateAxisConvention(const Eigen::Vector3d& elementAxis,
                                              const Eigen::Vector3d& modelUp)
{
    constexpr double tolerance = 1.0e-12;
    if (!elementAxis.allFinite() || elementAxis.norm() <= tolerance ||
        !modelUp.allFinite() || modelUp.norm() <= tolerance)
        throw std::invalid_argument("Bundled galloping axis and model-up direction must be finite and nonzero");

    const Eigen::Vector3d axis = elementAxis.normalized();
    const Eigen::Vector3d up = modelUp.normalized();
    const Eigen::Vector3d horizontal = axis - axis.dot(up) * up;
    if (horizontal.norm() <= tolerance)
        throw std::invalid_argument("Bundled galloping element axis is parallel to model-up direction");
    Eigen::Index dominant = 0;
    horizontal.cwiseAbs().maxCoeff(&dominant);
    const Eigen::Vector3d horizontalCanonical = horizontal[dominant] < 0.0 ? -axis : axis;
    if (horizontalCanonical.dot(Utility::CR::CanonicalAxis(elementAxis)) < 0.0)
    {
        const std::string chord = "(" + std::to_string(elementAxis.x()) + ", " +
                                  std::to_string(elementAxis.y()) + ", " +
                                  std::to_string(elementAxis.z()) + ")";
        throw std::runtime_error("Unsupported steep bundled galloping element: horizontal bundle-section axis "
                                 "opposes the 3D cable/aerodynamic axis; chord=" + chord);
    }
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

    ValidateAxisConvention(elementAxis, modelUp);
    Eigen::Vector3d axis = elementAxis.normalized();
    const Eigen::Vector3d modelUpUnit = modelUp.normalized();
    // 子导线几何的右向量由水平档距确定。用法平面内的档距方向规范轴向，
    // 避免大高差或局部垂度使主分量切换到竖向后反转剖面顺序。
    const Eigen::Vector3d horizontalAxis = axis - axis.dot(modelUpUnit) * modelUpUnit;
    if (horizontalAxis.norm() <= tolerance)
        throw std::invalid_argument("Model up direction must not be parallel to element axis");
    Eigen::Index dominant = 0;
    horizontalAxis.cwiseAbs().maxCoeff(&dominant);
    if (horizontalAxis[dominant] < 0.0)
        axis = -axis;
    Eigen::Vector3d up = modelUpUnit - modelUpUnit.dot(axis) * axis;
    if (up.norm() <= tolerance)
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

std::array<int, 4> BundleAeroMapper::ResolveQuadProfiles(
    const std::array<Eigen::Vector3d, 4>& wireCenters, const Eigen::Vector3d& elementAxis,
    const Eigen::Vector3d& modelUp, const Eigen::Vector3d& windVelocityGlobal,
    double initialAttackDegrees)
{
    constexpr double tolerance = 1.0e-12;
    // 10% 的位置偏差约对应 5.7 度的分裂导线转角；
    // 更大的姿态误差不得直接沿用同一组 P 列。
    constexpr double maximumRelativePositionError = 0.10;
    ValidateAxisConvention(elementAxis, modelUp);
    if (!windVelocityGlobal.allFinite() || windVelocityGlobal.norm() <= tolerance ||
        !std::isfinite(initialAttackDegrees))
        throw std::invalid_argument("Four-wire aerodynamic registration needs finite wind and attack angle");

    const Eigen::Vector3d upUnit = modelUp.normalized();
    Eigen::Vector3d spanAxis = elementAxis.normalized();
    spanAxis -= spanAxis.dot(upUnit) * upUnit;
    if (spanAxis.norm() <= tolerance)
        throw std::invalid_argument("Four-wire span is parallel to model-up direction");
    spanAxis.normalize();

    // 风洞示意图沿导线轴向观察：向右为来流方向，
    // 向上为物理竖向；先去除沿导线轴向的来流分量。
    Eigen::Vector3d flow = windVelocityGlobal - windVelocityGlobal.dot(spanAxis) * spanAxis;
    if (flow.norm() <= tolerance)
        throw std::invalid_argument("Four-wire wind is parallel to the span");
    flow.normalize();
    Eigen::Vector3d sectionUp = upUnit - upUnit.dot(spanAxis) * spanAxis;
    sectionUp -= sectionUp.dot(flow) * flow;
    if (sectionUp.norm() <= tolerance)
        throw std::invalid_argument("Four-wire wind is parallel to the model-up direction");
    sectionUp.normalize();

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3d& position : wireCenters)
    {
        if (!position.allFinite())
            throw std::invalid_argument("Four-wire center position is not finite");
        centroid += position;
    }
    centroid /= 4.0;

    std::array<Eigen::Vector3d, 4> offsets;
    double radius = 0.0;
    for (int wire = 0; wire < 4; ++wire)
    {
        offsets[wire] = wireCenters[wire] - centroid;
        offsets[wire] -= offsets[wire].dot(spanAxis) * spanAxis;
        radius += offsets[wire].norm();
    }
    radius /= 4.0;
    if (radius <= tolerance)
        throw std::invalid_argument("Four-wire centers have zero transverse spacing");

    // 实测零姿态中：P1 (-flow)、P2 (+up)、P3 (+flow)、
    // P4 (-up)。正攻角使整个菱形在来流/竖向示意图中逆时针旋转，
    // 再将各 P 列映射到对应的物理子导线。
    const double angle = std::remainder(initialAttackDegrees, 360.0) * std::acos(-1.0) / 180.0;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const std::array<Eigen::Vector3d, 4> expected{
        radius * (-cosine * flow - sine * sectionUp),
        radius * (-sine * flow + cosine * sectionUp),
        radius * (cosine * flow + sine * sectionUp),
        radius * (sine * flow - cosine * sectionUp)};

    std::array<int, 4> permutation{0, 1, 2, 3};
    std::array<int, 4> bestPermutation = permutation;
    double bestSquaredError = std::numeric_limits<double>::infinity();
    double nextSquaredError = std::numeric_limits<double>::infinity();
    double bestMaximumError = std::numeric_limits<double>::infinity();
    do
    {
        double squaredError = 0.0;
        double maximumError = 0.0;
        for (int wire = 0; wire < 4; ++wire)
        {
            const double error = (offsets[wire] - expected[permutation[wire]]).norm();
            squaredError += error * error;
            maximumError = std::max(maximumError, error);
        }
        if (squaredError < bestSquaredError)
        {
            nextSquaredError = std::min(nextSquaredError, bestSquaredError);
            bestSquaredError = squaredError;
            bestMaximumError = maximumError;
            bestPermutation = permutation;
        }
        else if (squaredError < nextSquaredError)
        {
            nextSquaredError = squaredError;
        }
    } while (std::next_permutation(permutation.begin(), permutation.end()));

    if (bestMaximumError > maximumRelativePositionError * radius ||
        nextSquaredError - bestSquaredError <= 0.1 * radius * radius)
        throw std::runtime_error("四线截面与初始攻角旋转后的风洞零位不吻合；请旋转束几何或使用显式剖面绑定");
    return bestPermutation;
}
