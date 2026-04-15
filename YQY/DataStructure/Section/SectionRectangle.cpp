#include "SectionRectangle.h"

void SectionRectangle::Calculate_Area()
{
    m_Area = m_Width * m_Height;
}

void SectionRectangle::Calculate_I(double& Iy, double& Iz,double& J)
{
    Calculate_Area();
    Iy = (m_Width * pow(m_Height, 3)) / 12.0;  // 绕y轴的惯性矩
    Iz = (m_Height * pow(m_Width, 3)) / 12.0;  // 绕z轴的惯性矩

    double a = std::max(m_Width, m_Height);
    double b = std::min(m_Width, m_Height);
    double ratio = b / a;
    J = a * b * b * b * (1.0 / 3.0 - 0.21 * ratio * (1.0 - std::pow(ratio, 4) / 12.0));
}
