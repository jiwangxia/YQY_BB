#pragma once
#include "LoadBase.h"
#include <Eigen/Core>
class Force_Wind : public LoadBase
{
public:
    Force_Wind()
    {
        m_LoadType = EnumKeyword::LoadType::FORCE_WIND;
    }
    double m_velocity = 10.0; // 风速大小
    // 导线沿X、竖直为Z时，-Y风向对应旧TSSBN的+Z风向。
    Eigen::Vector3d m_direction = -Eigen::Vector3d::UnitY(); // 全局风向，使用时归一化
    double m_windDensity = 1.225;                            // 风密度，海平面标准空气密度 kg/m³
    bool m_EnableAerodynamicMoment = true;                   // 舞动气动力矩开关，验证时可只保留升力和阻力
    bool m_DiagnosticReferenceFixedForceDirections = false; // 验证参考公式：升阻力方向冻结在静止风向
    bool m_DiagnosticReferenceFixedWindSpeed = false;       // 旧参考索模型以名义风速计算动压和攻角分母
    bool m_DiagnosticReferenceExactFittedAero = false;      // 旧参考单线五阶气动公式直接求值
    bool m_DiagnosticPaperBoundedFittedAero = false;         // 论文拟合仅用于0°到100°，域外采用360°试验数据

    Eigen::Vector3d GetWindVelocityGlobal() const
    {
        const double norm = m_direction.norm();
        return norm > 1.0e-12 ? (m_velocity * m_direction / norm).eval() : Eigen::Vector3d::Zero();
    }
};
