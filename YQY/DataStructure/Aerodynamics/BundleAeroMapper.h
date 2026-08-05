#pragma once

#include <Eigen/Core>

class BundleAeroMapper
{
public:
    // Profile columns use the conductor builder's regular-polygon order under
    // a reference wind along local section-right. wireId remains the fixed
    // physical identity; only the returned runtime profile changes.
    static int ResolveProfile(
        int bundleCount,
        int wireId,
        const Eigen::Vector3d& elementAxis,
        const Eigen::Vector3d& modelUp,
        const Eigen::Vector3d& windVelocityGlobal,
        double bundleTwistRadians = 0.0);
};
