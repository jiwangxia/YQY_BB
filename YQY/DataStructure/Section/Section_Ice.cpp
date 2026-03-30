#include "Section_Ice.h"
#include <cmath>
#include <algorithm>

/**
 * @brief 计算流程：
 * 1. 数值积分求冰层对原点(导线圆心)的面积、静矩、惯性矩
 * 2. 组合导线与冰层的特性
 * 3. 应用平行移轴定理转移到组合形心
 */
void Section_Ice::Calculate()
{
    if (m_Radius <= 0.0) return;

    // 导线自身特性（圆形截面）
    double wireArea = m_Area;
    double I_wire = (PI * std::pow(m_Radius, 4)) / 4.0; // 导线对圆心的惯性矩

    // 1. 积分获取冰层特性 (相对于原点 O)
    double ice_A, ice_My, ice_Mz, ice_Iyy0, ice_Izz0, ice_Iyz0;
    CalculateCrescentProperties(ice_A, ice_My, ice_Mz, ice_Iyy0, ice_Izz0, ice_Iyz0);

    m_IceArea = ice_A;
    m_TotalArea = wireArea + ice_A;

    // 2. 计算组合形心坐标 (Centroid)
    // 公式：yc = Sigma(y_i * A_i) / A_total = (0*Aw + ice_Mz) / A_total
    yc = ice_Mz / m_TotalArea;
    zc = ice_My / m_TotalArea;

    // 3. 计算对原点的总惯性矩
    double total_Iyy0 = I_wire + ice_Iyy0;
    double total_Izz0 = I_wire + ice_Izz0;
    double total_Iyz0 = 0.0 + ice_Iyz0;

    // 4. 平行移轴定理 (Parallel Axis Theorem)
    // 公式：I_centroid = I_origin - A * d^2
    // Iyy = Integral( (z - zc)^2 dA ) = Iyy0 - A * zc^2
    Iyy = total_Iyy0 - m_TotalArea * std::pow(zc, 2);
    Izz = total_Izz0 - m_TotalArea * std::pow(yc, 2);
    Iyz = total_Iyz0 - m_TotalArea * yc * zc;
    J = Iyy + Izz;
}

void Section_Ice::CalculateCrescentProperties(double& area, double& my, double& mz,
    double& iyy, double& izz, double& iyz)
{
    area = my = mz = iyy = izz = iyz = 0.0;
    const int n = 180;
    double dtheta = (2.0 * m_IceHalfAngle) / n;
    double theta_start = m_IceAngle - m_IceHalfAngle;

    for (int i = 0; i < n; ++i)
    {
        double theta = theta_start + (i + 0.5) * dtheta;
        double t = GetIceThicknessAt(theta);
        if (t <= 1e-9) continue;

        double R1 = m_Radius;
        double R2 = R1 + t;

        // 极坐标积分公式 (针对每一个 d_theta 分段):
        // dA = 1/2 * (R2^2 - R1^2) * d_theta
        // dMy = 1/3 * (R2^3 - R1^3) * sin(theta) * d_theta
        // dMz = 1/3 * (R2^3 - R1^3) * (-cos(theta)) * d_theta
        // dIyy = 1/4 * (R2^4 - R1^4) * sin^2(theta) * d_theta
        // dIzz = 1/4 * (R2^4 - R1^4) * (-cos(theta))^2 * d_theta
        // dIyz = 1/4 * (R2^4 - R1^4) * (-cos(theta)) * sin(theta) * d_theta

        double R2_2 = R2 * R2, R1_2 = R1 * R1;
        double R2_3 = R2_2 * R2, R1_3 = R1_2 * R1;
        double R2_4 = R2_3 * R2, R1_4 = R1_3 * R1;

        double s = std::sin(theta);
        double c = -std::cos(theta); // 对应代码定义的 0度在-y轴

        area += 0.5 * (R2_2 - R1_2) * dtheta;
        my += (1.0 / 3.0) * (R2_3 - R1_3) * s * dtheta;
        mz += (1.0 / 3.0) * (R2_3 - R1_3) * c * dtheta;
        iyy += (1.0 / 4.0) * (R2_4 - R1_4) * s * s * dtheta;
        izz += (1.0 / 4.0) * (R2_4 - R1_4) * c * c * dtheta;
        iyz += (1.0 / 4.0) * (R2_4 - R1_4) * s * c * dtheta;
    }
}

double Section_Ice::GetIceThicknessAt(double theta) const
{
    // 计算角度偏差并归一化到 [-PI, PI]
    double delta = fmod(theta - m_IceAngle + PI, 2.0 * PI);
    if (delta < 0) delta += 2.0 * PI;
    delta -= PI;

    if (std::abs(delta) > m_IceHalfAngle) return 0.0;

    // 余弦分布公式：t = t_max * cos^k( (delta/beta) * (PI/2) )
    double ratio = delta / m_IceHalfAngle;
    return m_IceThickness * std::pow(std::cos(ratio * PI / 2.0), m_IceShapeFactor);
}

void Section_Ice::SetIceParameters(double iceThickness, double iceAngle, double iceHalfAngle, double shapeFactor)
{
    m_IceThickness = iceThickness;
    m_IceAngle = iceAngle * PI / 180.0;
    m_IceHalfAngle = iceHalfAngle * PI / 180.0;
    m_IceShapeFactor = shapeFactor;
    Calculate();
}