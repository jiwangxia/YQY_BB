#pragma once

#include "Base/EmptyOUT.h"
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace SolverNameSpace
{
/** @brief 当前 Node 材料阶段映射对完整空间左修正的速度影响预算。 */
struct TssbnRotationPrecisionEstimate
{
    Eigen::Vector3d linearizedVelocityCorrection = Eigen::Vector3d::Zero();
    Eigen::Vector3d absoluteVelocityCorrectionBound = Eigen::Vector3d::Zero();
    double nonlinearRemainderBound = 0.0;
    double arithmeticAllowance = 0.0;
};

/**
 * @brief 估计 Node 的 C1/C2 阶段角速度在完整左乘转动修正下的分量变化。
 * @details 两阶段均满足 omega=Exp(phi)*(phi-k)/d，k 是冻结的已知阶段量。
 *          因此 B*delta=Jl(phi)^-1*delta/d+(phi/d-omega) cross delta；C2 的已知项不能丢弃。
 *          只在整个修正路径位于主值图内使用。对 |phi|<pi，有 |Jl^-1|<4、|D Jl^-1|<1.5，
 *          给出二阶余项；浮点计算余量只覆盖此公式，不冒充节点绝对姿态存储误差的上界。
 * @param spatialIncrement 当前相对步初的空间主转角
 * @param spatialCorrection 当前完整 Newton 空间左修正
 * @param angularVelocity 当前实际阶段空间角速度
 * @param velocityDerivative 正阶段对角时间的倒数 1/d
 * @param[out] estimate 一阶变化及保留二阶余项的逐分量绝对预算
 * @author   yangqiuyou
 * @date     2026.10.22
 */
inline bool EstimateTssbnRotationVelocityCorrection(const Eigen::Vector3d& spatialIncrement,
                                                    const Eigen::Vector3d& spatialCorrection,
                                                    const Eigen::Vector3d& angularVelocity, double velocityDerivative,
                                                    _OUT TssbnRotationPrecisionEstimate& estimate)
{
    if (!spatialIncrement.allFinite() || !spatialCorrection.allFinite() || !angularVelocity.allFinite() ||
        !std::isfinite(velocityDerivative) || velocityDerivative <= 0.0)
        return false;
    const double angle = spatialIncrement.norm();
    const double correctionNorm = spatialCorrection.norm();
    if (!std::isfinite(angle) || !std::isfinite(correctionNorm) || angle + correctionNorm >= std::acos(-1.0) - 1.0e-6)
        return false;

    const double angleSquared = angle * angle;
    double coefficient;
    if (angleSquared < 1.0e-8)
    {
        coefficient =
            1.0 / 12.0 + angleSquared * (1.0 / 720.0 + angleSquared * (1.0 / 30240.0 + angleSquared / 1209600.0));
    }
    else
    {
        coefficient = (1.0 - (0.5 * angle) / std::tan(0.5 * angle)) / angleSquared;
    }
    const Eigen::Vector3d firstCross = spatialIncrement.cross(spatialCorrection);
    const Eigen::Vector3d inverseJacobianCorrection =
        spatialCorrection - 0.5 * firstCross + coefficient * spatialIncrement.cross(firstCross);
    const Eigen::Vector3d knownVelocity = velocityDerivative * spatialIncrement - angularVelocity;
    estimate.linearizedVelocityCorrection =
        velocityDerivative * inverseJacobianCorrection + knownVelocity.cross(spatialCorrection);

    // Bernoulli 级数在 |phi|<=pi 给出上述两个算子界；Taylor 积分余项不超过下式。
    estimate.nonlinearRemainderBound = 0.5 * correctionNorm * correctionNorm *
                                       (angularVelocity.norm() + velocityDerivative * (14.0 + 4.0 * correctionNorm));
    const double arithmeticScale =
        correctionNorm *
        (velocityDerivative * (1.0 + 1.5 * angle + std::abs(coefficient) * angleSquared) + angularVelocity.norm());
    estimate.arithmeticAllowance = 64.0 * std::numeric_limits<double>::epsilon() * arithmeticScale;
    estimate.absoluteVelocityCorrectionBound = estimate.linearizedVelocityCorrection.cwiseAbs();
    estimate.absoluteVelocityCorrectionBound.array() += estimate.nonlinearRemainderBound + estimate.arithmeticAllowance;
    return estimate.linearizedVelocityCorrection.allFinite() && estimate.absoluteVelocityCorrectionBound.allFinite();
}
}
