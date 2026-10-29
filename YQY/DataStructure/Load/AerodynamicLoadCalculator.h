#pragma once

#include "Base/EmptyOUT.h"

#include <Eigen/Dense>

struct AerodynamicSectionState
{
    Eigen::Vector3d firstPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d secondPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d firstVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d secondVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d windVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d modelUp = Eigen::Vector3d::UnitZ();
    // 普通空间梁保留截面方向，避免节点主值转角跨 pi 后平均得到错误攻角。
    Eigen::Vector3d firstSectionAxis = Eigen::Vector3d::UnitX();
    Eigen::Vector3d secondSectionAxis = Eigen::Vector3d::UnitX();
    Eigen::Vector3d firstSectionDirection = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d secondSectionDirection = Eigen::Vector3d::UnitZ();
    double firstTwist = 0.0;
    double secondTwist = 0.0;
    double firstTwistRate = 0.0;
    double secondTwistRate = 0.0;
    double radius = 0.0;
    double initialAttack = 0.0;
    bool referenceTssbn = false;
    bool diagnosticReferenceFixedWindSpeed = false;
    bool useBeamSectionDirections = false;
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
    // 攻角单位为弧度；调用方按返回攻角查询气动系数。
    static AerodynamicSectionResult ComputeKinematics(const AerodynamicSectionState& state);

    static void ComputeLineLoad(_OUT AerodynamicSectionResult& result, double airDensity, double diameter,
                                double liftCoefficient, double dragCoefficient, double momentCoefficient,
                                bool diagnosticFixedForceDirections = false);

private:
    static Eigen::Vector3d AlignBeamSectionDirection(const Eigen::Vector3d& sectionAxis,
                                                     const Eigen::Vector3d& sectionDirection,
                                                     const Eigen::Vector3d& commonAxis);
    static double ComputeBeamMeanTwist(const AerodynamicSectionState& state,
                                       const AerodynamicSectionResult& frame);
};
