#pragma once

#include <Eigen/Dense>

struct AerodynamicSectionState
{
    Eigen::Vector3d firstPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d secondPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d firstVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d secondVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d windVelocity = Eigen::Vector3d::Zero();
    double firstTwist = 0.0;
    double secondTwist = 0.0;
    double firstTwistRate = 0.0;
    double secondTwistRate = 0.0;
    double radius = 0.0;
    double initialAttack = 0.0;
};

struct AerodynamicSectionResult
{
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    Eigen::Vector3d transverse = Eigen::Vector3d::Zero();
    Eigen::Vector3d windNormal = Eigen::Vector3d::Zero();
    Eigen::Vector3d relativeWind = Eigen::Vector3d::Zero();
    double relativeSpeed = 0.0;
    double flowAngle = 0.0;
    double attackAngle = 0.0;
    Eigen::Vector3d lineForce = Eigen::Vector3d::Zero();
    Eigen::Vector3d lineMoment = Eigen::Vector3d::Zero();
};

class AerodynamicLoadCalculator
{
public:
    // Angles are radians. Cl/Cd/Cm are evaluated by the caller at the returned
    // attackAngle, allowing AeroManager to remain the coefficient repository.
    static AerodynamicSectionResult ComputeKinematics(
        const AerodynamicSectionState& state);

    static void ComputeLineLoad(AerodynamicSectionResult& result,
        double airDensity,
        double diameter,
        double liftCoefficient,
        double dragCoefficient,
        double momentCoefficient);
};
