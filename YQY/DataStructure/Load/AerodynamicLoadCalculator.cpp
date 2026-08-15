#include "AerodynamicLoadCalculator.h"

#include "Utility/CR.h"

#include <cmath>
#include <stdexcept>

AerodynamicSectionResult AerodynamicLoadCalculator::ComputeKinematics(const AerodynamicSectionState& state)
{
    constexpr double tolerance = 1.0e-12;
    const Eigen::Vector3d chord = state.secondPosition - state.firstPosition;
    if (chord.norm() <= tolerance)
        throw std::runtime_error("Aerodynamic element length must be positive");
    if (state.radius < 0.0)
        throw std::runtime_error("Aerodynamic radius must not be negative");

    AerodynamicSectionResult result;
    // 单元节点顺序反转时气动力方向不能改变，因此导线轴采用唯一确定的方向。
    result.axis = Utility::CR::CanonicalAxis(chord);

    // 气动力仅作用于导线轴的法平面。
    const Eigen::Vector3d normalWind = state.windVelocity - state.windVelocity.dot(result.axis) * result.axis;
    if (normalWind.norm() <= tolerance)
        return result;

    result.windNormal = normalWind.normalized();
    result.transverse = result.windNormal.cross(result.axis).normalized();

    const Eigen::Vector3d midpointVelocity = 0.5 * (state.firstVelocity + state.secondVelocity);
    const double twistRate = 0.5 * (state.firstTwistRate + state.secondTwistRate);

    // 计算覆冰导线代表表面的相对气流速度，精确保留 r*omega 项，不采用 (v+r*omega)/U 线性化。
    const double relativeTransverse = -midpointVelocity.dot(result.transverse) - state.radius * twistRate;
    const double relativeWindNormal = normalWind.norm() - midpointVelocity.dot(result.windNormal);
    result.relativeWind = relativeTransverse * result.transverse + relativeWindNormal * result.windNormal;
    result.relativeSpeed = std::hypot(relativeTransverse, relativeWindNormal);
    if (result.relativeSpeed <= tolerance)
        return result;

    result.flowAngle = std::atan2(-relativeTransverse, relativeWindNormal);
    const double meanTwist = 0.5 * (state.firstTwist + state.secondTwist);
    result.attackAngle = state.initialAttack + meanTwist - result.flowAngle;
    return result;
}

void AerodynamicLoadCalculator::ComputeLineLoad(_OUT AerodynamicSectionResult& result, double airDensity,
                                                double diameter, double liftCoefficient, double dragCoefficient,
                                                double momentCoefficient)
{
    if (airDensity < 0.0 || diameter < 0.0)
        throw std::runtime_error("Aerodynamic density and diameter must not be negative");
    if (result.relativeSpeed <= 1.0e-12)
    {
        result.lineForce.setZero();
        result.lineMoment.setZero();
        return;
    }

    const Eigen::Vector3d dragDirection = result.relativeWind / result.relativeSpeed;
    const Eigen::Vector3d liftDirection = dragDirection.cross(result.axis);
    const double pressureWidth = 0.5 * airDensity * diameter * result.relativeSpeed * result.relativeSpeed;
    result.lineForce = pressureWidth * (dragCoefficient * dragDirection + liftCoefficient * liftDirection);
    result.lineMoment = pressureWidth * diameter * momentCoefficient * result.axis;
}
