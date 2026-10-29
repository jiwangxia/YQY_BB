#pragma once

#include <Eigen/Core>
#include <array>

class BundleAeroMapper
{
public:
    // 通用正多边形的旧解析器；完整四分裂气动表由下方风洞姿态配准解析。
    // wireId 始终表示固定的物理子导线。
    // 分裂导线若水平规范轴与索/气动力使用的三维规范轴相反，现有模型无法
    // 唯一解释截面方位；入口绑定和每轮气动力装配均需调用此检查。
    static void ValidateAxisConvention(const Eigen::Vector3d& elementAxis, const Eigen::Vector3d& modelUp);
    static int ResolveProfile(int bundleCount, int wireId, const Eigen::Vector3d& elementAxis,
                              const Eigen::Vector3d& modelUp, const Eigen::Vector3d& windVelocityGlobal,
                              double bundleTwistRadians = 0.0);

    // 2021 四分裂风洞表的零位是 P1 上游、P2 上方、P3 下游、P4 下方。
    // 输入四个固定 WIRE_ID 的同一截面中心，按初始攻角刚体旋转风洞零位后
    // 做唯一几何配准；结果在整段动力分析中始终绑定到这些物理子导线。
    static std::array<int, 4> ResolveQuadProfiles(
        const std::array<Eigen::Vector3d, 4>& wireCenters, const Eigen::Vector3d& elementAxis,
        const Eigen::Vector3d& modelUp, const Eigen::Vector3d& windVelocityGlobal,
        double initialAttackDegrees);
};
