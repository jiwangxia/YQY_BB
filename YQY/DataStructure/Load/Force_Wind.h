#pragma once
#include "LoadBase.h"
#include <Eigen/Core>
class Force_Wind : public LoadBase
{
public:
	Force_Wind() { m_LoadType = EnumKeyword::LoadType::FORCE_WIND; }
    double m_velocity = 10.0;   // 风速大小
    Eigen::Vector3d m_direction = Eigen::Vector3d::UnitY(); // 全局风向，使用时归一化
    double m_windDensity = 1.225;   // 风密度，海平面标准空气密度 kg/m³

    Eigen::Vector3d GetWindVelocityGlobal() const
    {
        const double norm = m_direction.norm();
        return norm > 1.0e-12
            ? (m_velocity * m_direction / norm).eval()
            : Eigen::Vector3d::Zero();
    }
};
