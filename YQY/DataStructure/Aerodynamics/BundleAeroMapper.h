#pragma once

#include <Eigen/Core>

class BundleAeroMapper
{
public:
    // 截面列按导线构建器的正多边形顺序排列，参考风向为局部截面右方向。
    // wireId 始终表示固定的物理子导线，仅返回的运行时截面编号随风向变化。
    static int ResolveProfile(int bundleCount, int wireId, const Eigen::Vector3d& elementAxis,
                              const Eigen::Vector3d& modelUp, const Eigen::Vector3d& windVelocityGlobal,
                              double bundleTwistRadians = 0.0);
};
