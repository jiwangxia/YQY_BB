#include "AerodynamicLoadCalculator.h"

#include <cmath>
#include <stdexcept>

namespace
{
Eigen::Vector3d CanonicalAxis(const Eigen::Vector3d& chord)
{
    Eigen::Vector3d axis = chord.normalized();
    Eigen::Index dominant = 0;
    axis.cwiseAbs().maxCoeff(&dominant);
    if (axis[dominant] < 0.0)
        axis = -axis;
    return axis;
}
}

AerodynamicSectionResult AerodynamicLoadCalculator::ComputeKinematics(
    const AerodynamicSectionState& state)
{
    constexpr double tolerance = 1.0e-12;
    const Eigen::Vector3d chord = state.secondPosition - state.firstPosition;
    if (chord.norm() <= tolerance)
        throw std::runtime_error("Aerodynamic element length must be positive");
    if (state.radius < 0.0)
        throw std::runtime_error("Aerodynamic radius must not be negative");

    AerodynamicSectionResult result;
    // Aerodynamic directions must not change when an element's node order is
    // reversed. Use one deterministic orientation for the conductor axis.
    result.axis = CanonicalAxis(chord);

    // Aerodynamics acts only in the plane normal to the conductor axis.
    const Eigen::Vector3d normalWind = state.windVelocity
        - state.windVelocity.dot(result.axis) * result.axis;
    if (normalWind.norm() <= tolerance)
        return result;

    // e3 follows the undisturbed cross-wind.  For a horizontal conductor and
    // horizontal wind, e2=e3xe1 is vertical, matching the THOP convention.
    result.windNormal = normalWind.normalized();
    result.transverse = result.windNormal.cross(result.axis).normalized();

    const Eigen::Vector3d midpointVelocity =
        0.5 * (state.firstVelocity + state.secondVelocity);
    const double twistRate =
        0.5 * (state.firstTwistRate + state.secondTwistRate);

    // Relative air velocity at the representative iced-conductor surface.
    // The r*omega term is retained exactly; no (v+r*omega)/U linearization.
    const double relativeTransverse =
        -midpointVelocity.dot(result.transverse) - state.radius * twistRate;
    const double relativeWindNormal =
        normalWind.norm() - midpointVelocity.dot(result.windNormal);
    result.relativeWind = relativeTransverse * result.transverse
        + relativeWindNormal * result.windNormal;
    result.relativeSpeed = std::hypot(relativeTransverse, relativeWindNormal);
    if (result.relativeSpeed <= tolerance)
        return result;

    result.flowAngle = std::atan2(-relativeTransverse, relativeWindNormal);
    const double meanTwist = 0.5 * (state.firstTwist + state.secondTwist);
    result.attackAngle = state.initialAttack + meanTwist - result.flowAngle;
    return result;
}

void AerodynamicLoadCalculator::ComputeLineLoad(
    AerodynamicSectionResult& result,
    double airDensity,
    double diameter,
    double liftCoefficient,
    double dragCoefficient,
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

    const Eigen::Vector3d dragDirection =
        result.relativeWind / result.relativeSpeed;
    const Eigen::Vector3d liftDirection = dragDirection.cross(result.axis);
    const double pressureWidth = 0.5 * airDensity * diameter
        * result.relativeSpeed * result.relativeSpeed;
    result.lineForce = pressureWidth
        * (dragCoefficient * dragDirection + liftCoefficient * liftDirection);
    result.lineMoment = pressureWidth * diameter * momentCoefficient * result.axis;
}
