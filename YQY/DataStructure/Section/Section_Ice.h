#pragma once
#include "SectionBase.h"

/**
 * @brief 覆冰导线截面特性计算类 - 新月形覆冰模型
 * * 理论公式说明：
 * 1. 冰厚分布：t(theta) = t_max * cos^k( (theta - alpha_0)/beta * PI/2 )
 * 2. 坐标转换：y = -r * cos(theta), z = r * sin(theta) (0度在-y轴)
 * 3. 极坐标积分微元：dA = 0.5 * (R_out^2 - R_in^2) d_theta
 */
class Section_Ice : public SectionBase
{
public:
    // --- 输入参数 ---
    double m_IceThickness = 0.0;       // 最大冰厚 t_max (m)
    double m_IceAngle = 0.0;           // 覆冰中心角度 alpha_0 (rad)
    double m_IceHalfAngle = PI / 2.0;  // 覆冰半角范围 beta (rad)
    double m_IceShapeFactor = 1.0;     // 形状系数 k
    const double m_IceDensity = 917.0; // 冰密度 (kg/m^3)

    // --- 计算结果（相对于组合截面形心） ---
    double m_IceArea = 0.0;    // 覆冰截面积 (m^2)
    double m_TotalArea = 0.0;  // 总截面积 (m^2)
    double yc = 0.0, zc = 0.0; // 形心位置 (相对于导线圆心)

    double Iyy = 0.0; // 绕形心 y 轴惯性矩 (m^4)
    double Izz = 0.0; // 绕形心 z 轴惯性矩 (m^4)
    double Iyz = 0.0; // 形心惯性积 (m^4)
    double J = 0.0;   // 极惯性矩 (m^4)

    void Calculate();
    void SetIceParameters(double iceThickness, double iceAngle, double iceHalfAngle = 90.0, double shapeFactor = 1.0);

private:
    double GetIceThicknessAt(double theta) const;
    void CalculateCrescentProperties(_OUT double& area, _OUT double& my, _OUT double& mz, _OUT double& iyy,
                                     _OUT double& izz, _OUT double& iyz);
};
