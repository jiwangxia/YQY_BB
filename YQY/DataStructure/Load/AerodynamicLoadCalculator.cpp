#include "AerodynamicLoadCalculator.h"

#include "Utility/CR.h"

#include <algorithm>
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
    if (!state.modelUp.allFinite() || state.modelUp.norm() <= tolerance)
        throw std::runtime_error("Aerodynamic model-up direction must be finite and nonzero");
    if (state.referenceTssbn || state.diagnosticReferenceFixedWindSpeed)
    {
        const double nominalWindSpeed = state.windVelocity.norm();
        if (nominalWindSpeed <= tolerance)
            return result;
        result.transverse = state.modelUp.normalized();
        result.windNormal = state.windVelocity / nominalWindSpeed;
        if (std::abs(result.transverse.dot(result.windNormal)) > 1.0e-6)
            throw std::runtime_error("Fixed-speed galloping requires orthogonal wind and up axes");
        result.relativeSpeed = nominalWindSpeed;
        const Eigen::Vector3d midpointVelocity = 0.5 * (state.firstVelocity + state.secondVelocity);
        const double twistRate = 0.5 * (state.firstTwistRate + state.secondTwistRate);
        result.flowAngle = (midpointVelocity.dot(result.transverse) + state.radius * twistRate) /
                           nominalWindSpeed;
        // 参考流程按名义风速线性化来流偏角，力方向由装配入口独立选择。
        result.relativeWind = nominalWindSpeed *
                              (-std::sin(result.flowAngle) * result.transverse +
                               std::cos(result.flowAngle) * result.windNormal);
        const double meanTwist = state.useBeamSectionDirections ? ComputeBeamMeanTwist(state, result)
                                                               : 0.5 * (state.firstTwist + state.secondTwist);
        result.attackAngle = state.initialAttack + meanTwist - result.flowAngle;
        return result;
    }
    const Eigen::Vector3d upNormal = state.modelUp - result.axis.dot(state.modelUp) * result.axis;

    // 气动力仅作用于导线轴的法平面。
    const Eigen::Vector3d normalWind = state.windVelocity - state.windVelocity.dot(result.axis) * result.axis;
    if (normalWind.norm() > tolerance)
    {
        result.windNormal = normalWind.normalized();
        // 局部正横向方向随模型竖直方向确定，避免 Y 向上模型误用 Z 向上约定。
        result.transverse = result.axis.cross(result.windNormal).normalized();
        if (upNormal.norm() > tolerance && result.transverse.dot(upNormal) < 0.0)
            result.transverse = -result.transverse;
    }
    else
    {
        // 外风与导线轴平行时，横向运动仍产生相对气流。用模型竖直方向固定法平面基。
        if (upNormal.norm() > tolerance)
            result.transverse = upNormal.normalized();
        else
        {
            Eigen::Index leastAligned = 0;
            result.axis.cwiseAbs().minCoeff(&leastAligned);
            Eigen::Vector3d reference = Eigen::Vector3d::Zero();
            reference[leastAligned] = 1.0;
            result.transverse = (reference - result.axis.dot(reference) * result.axis).normalized();
        }
        result.windNormal = result.transverse.cross(result.axis).normalized();
    }

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
    const double meanTwist = state.useBeamSectionDirections ? ComputeBeamMeanTwist(state, result)
                                                           : 0.5 * (state.firstTwist + state.secondTwist);
    result.attackAngle = state.initialAttack + meanTwist - result.flowAngle;
    return result;
}

Eigen::Vector3d AerodynamicLoadCalculator::AlignBeamSectionDirection(const Eigen::Vector3d& sectionAxis,
                                                                    const Eigen::Vector3d& sectionDirection,
                                                                    const Eigen::Vector3d& commonAxis)
{
    constexpr double tolerance = 1.0e-12;
    if (!sectionAxis.allFinite() || !sectionDirection.allFinite() || !commonAxis.allFinite() ||
        sectionAxis.norm() <= tolerance || sectionDirection.norm() <= tolerance || commonAxis.norm() <= tolerance)
        throw std::runtime_error("Beam aerodynamic section orientation is invalid");

    const Eigen::Vector3d source = sectionAxis.normalized();
    const Eigen::Vector3d target = commonAxis.normalized();
    const double cosine = std::clamp(source.dot(target), -1.0, 1.0);
    if (1.0 + cosine <= 1.0e-10)
        throw std::runtime_error("Beam aerodynamic section axis is antiparallel to the current chord");

    // 最小 swing 只对齐截面轴，不增加绕共同弦轴的扭转。
    const Eigen::Vector3d cross = source.cross(target);
    Eigen::Vector3d direction = sectionDirection + cross.cross(sectionDirection) +
                                cross.cross(cross.cross(sectionDirection)) / (1.0 + cosine);
    direction -= direction.dot(target) * target;
    if (!direction.allFinite() || direction.norm() <= tolerance)
        throw std::runtime_error("Beam aerodynamic section direction is degenerate in the current normal plane");
    return direction.normalized();
}

double AerodynamicLoadCalculator::ComputeBeamMeanTwist(const AerodynamicSectionState& state,
                                                       const AerodynamicSectionResult& frame)
{
    constexpr double tolerance = 1.0e-10;
    const Eigen::Vector3d first =
        AlignBeamSectionDirection(state.firstSectionAxis, state.firstSectionDirection, frame.axis);
    const Eigen::Vector3d second =
        AlignBeamSectionDirection(state.secondSectionAxis, state.secondSectionDirection, frame.axis);
    const Eigen::Vector3d sum = first + second;
    if (!sum.allFinite() || sum.norm() <= tolerance)
        throw std::runtime_error("Beam aerodynamic section directions differ by 180 degrees; refine the element");

    Eigen::Vector3d zeroDirection = frame.transverse - frame.transverse.dot(frame.axis) * frame.axis;
    if (!zeroDirection.allFinite() || zeroDirection.norm() <= tolerance)
        throw std::runtime_error("Beam aerodynamic zero direction is parallel to the current chord");
    zeroDirection.normalize();
    const Eigen::Vector3d meanDirection = sum.normalized();
    // 保持既有 CanonicalAxis 正方向；平均的是方向，最终角度允许按 2*pi 周期换支。
    return std::atan2(frame.axis.dot(zeroDirection.cross(meanDirection)), zeroDirection.dot(meanDirection));
}

void AerodynamicLoadCalculator::ComputeLineLoad(_OUT AerodynamicSectionResult& result, double airDensity,
                                                double diameter, double liftCoefficient, double dragCoefficient,
                                                double momentCoefficient, bool diagnosticFixedForceDirections)
{
    if (airDensity < 0.0 || diameter < 0.0)
        throw std::runtime_error("Aerodynamic density and diameter must not be negative");
    if (result.relativeSpeed <= 1.0e-12)
    {
        result.lineForce.setZero();
        result.lineMoment.setZero();
        return;
    }

    // 按局部流偏角构造正升力方向，使其与 THOP 的局部 Fy=Fl*cos(beta)、
    // Fz=Fl*sin(beta) 约定一致，而不再由 axis × dragDirection 决定正负号。
    const Eigen::Vector3d liftDirection = std::cos(result.flowAngle) * result.transverse +
                                          std::sin(result.flowAngle) * result.windNormal;
    const double pressureWidth = 0.5 * airDensity * diameter * result.relativeSpeed * result.relativeSpeed;

    // 阻力必须沿当前相对风方向作用。冻结为静止风向会漏掉横向阻力分量，
    // 从而破坏包含 Cd 项的完整气动阻尼，并改变舞动稳定性判定。
    const Eigen::Vector3d dragDirection = result.relativeWind / result.relativeSpeed;
    result.lineForce = diagnosticFixedForceDirections
                           ? pressureWidth * (dragCoefficient * result.windNormal +
                                              liftCoefficient * result.transverse)
                           : pressureWidth * (dragCoefficient * dragDirection + liftCoefficient * liftDirection);
    result.lineMoment = pressureWidth * diameter * momentCoefficient * result.axis;
}
