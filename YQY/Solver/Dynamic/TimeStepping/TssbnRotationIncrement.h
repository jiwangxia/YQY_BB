#pragma once

#include "Base/EmptyOUT.h"
#include <Eigen/Geometry>
#include <cmath>

namespace SolverNameSpace
{
/**
 * @brief 普通 TSSBN 完整左乘修正的主值转角及稳定有限差。
 * @details 返回 Log(Exp(delta)Exp(phi)) 及其与 phi 的有限差。微小非共轴修正用四元数增量、
 *          范数差有理化和 atan2 差计算，不将逆 Jacobian 的一阶映射冒充有限更新。
 *          较大修正采用完整四元数复合；主值换支时有限差可以包含约 2pi 的跳变。
 *          deltaPhi 独立保存小变化，不能由已舍入的 updatedPhi-currentPhi 重新计算。
 *          参考 TSSBN 不调用本函数；返回 false 时调用方须明确处理无效输入。
 * @param currentPhi 当前独立步内主转角，范数不得超过 pi
 * @param spatialCorrection 完整空间左修正
 * @param[out] updatedPhi 完整复合后主转角的 binary64 表示
 * @param[out] deltaPhi 完整复合造成的稳定有限转角差
 * @author   yangqiuyou
 * @date     2026.10.23
 */
inline bool ComputeTssbnRotationIncrement(const Eigen::Vector3d& currentPhi, const Eigen::Vector3d& spatialCorrection,
                                          _OUT Eigen::Vector3d& updatedPhi, _OUT Eigen::Vector3d& deltaPhi)
{
    if (!currentPhi.allFinite() || !spatialCorrection.allFinite())
        return false;
    const double angle = currentPhi.stableNorm();
    const double correctionAngle = spatialCorrection.stableNorm();
    const double pi = std::acos(-1.0);
    if (!std::isfinite(angle) || !std::isfinite(correctionAngle) || angle > pi)
        return false;
    if (correctionAngle == 0.0)
    {
        updatedPhi = currentPhi;
        deltaPhi.setZero();
        return true;
    }
    if (angle == 0.0 && correctionAngle < pi)
    {
        updatedPhi = spatialCorrection;
        deltaPhi = spatialCorrection;
        return true;
    }

    const double oldScalar = std::cos(0.5 * angle);
    Eigen::Vector3d oldVector = Eigen::Vector3d::Zero();
    if (angle > 0.0)
        oldVector = (std::sin(0.5 * angle) / angle) * currentPhi;
    const double correctionScalar = std::cos(0.5 * correctionAngle);
    const Eigen::Vector3d correctionVector = (std::sin(0.5 * correctionAngle) / correctionAngle) * spatialCorrection;
    const double sineQuarter = std::sin(0.25 * correctionAngle);
    const double scalarDifferenceFactor = -2.0 * sineQuarter * sineQuarter;
    const Eigen::Vector3d vectorDifference =
        scalarDifferenceFactor * oldVector + oldScalar * correctionVector + correctionVector.cross(oldVector);
    const double scalarDifference = scalarDifferenceFactor * oldScalar - correctionVector.dot(oldVector);
    const double newScalar = oldScalar + scalarDifference;

    if (angle > 0.0 && correctionAngle < 0.125 * angle && newScalar >= 0.0)
    {
        const double oldMagnitude = oldVector.stableNorm();
        if (!(oldMagnitude > 0.0))
            return false;
        const Eigen::Vector3d oldAxis = oldVector / oldMagnitude;
        const Eigen::Vector3d relativeDifference = vectorDifference / oldMagnitude;
        const double squaredMagnitudeRatioDifference =
            2.0 * oldAxis.dot(relativeDifference) + relativeDifference.squaredNorm();
        const double newMagnitudeRatio = std::sqrt(1.0 + squaredMagnitudeRatioDifference);
        const double newMagnitude = oldMagnitude * newMagnitudeRatio;
        if (!(newMagnitude > 0.0) || !std::isfinite(newMagnitude))
            return false;
        // (s_new-s_old) 不能通过两个接近的范数相减获得。
        const double magnitudeDifference = oldMagnitude * squaredMagnitudeRatioDifference / (newMagnitudeRatio + 1.0);
        const double angleDifference =
            2.0 * std::atan2(std::fma(oldScalar, magnitudeDifference, -oldMagnitude * scalarDifference),
                             oldScalar * newScalar + oldMagnitude * newMagnitude);
        const double oldFactor = angle / oldMagnitude;
        const double newFactor = (angle + angleDifference) / newMagnitude;
        // 差商直接计算 f_new-f_old；其小量始终与 vectorDifference 保持同一增量尺度。
        const double factorDifference = std::fma(-oldFactor, magnitudeDifference, angleDifference) / newMagnitude;
        deltaPhi = newFactor * vectorDifference + factorDifference * oldVector;
        updatedPhi = currentPhi + deltaPhi;
        return deltaPhi.allFinite() && updatedPhi.allFinite();
    }

    // 当修正与原转角可比时，完整主值结果与原转角相减不会引入小修正/大转角的消减问题。
    // 若跨越主值切面，则结果有有限跳变，此分支也保留该完整跳变。
    double scalar = correctionScalar * oldScalar - correctionVector.dot(oldVector);
    Eigen::Vector3d vector =
        correctionScalar * oldVector + oldScalar * correctionVector + correctionVector.cross(oldVector);
    if (scalar < 0.0)
    {
        scalar = -scalar;
        vector = -vector;
    }
    const double magnitude = vector.stableNorm();
    if (magnitude == 0.0)
        updatedPhi.setZero();
    else
        updatedPhi = (2.0 * std::atan2(magnitude, scalar) / magnitude) * vector;
    deltaPhi = updatedPhi - currentPhi;
    return updatedPhi.allFinite() && deltaPhi.allFinite();
}
}
