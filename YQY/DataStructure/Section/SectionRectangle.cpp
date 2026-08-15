#include "SectionRectangle.h"

void SectionRectangle::Calculate_Area()
{
    m_Area = m_Width * m_Height;
}

void SectionRectangle::Calculate_I(double& Iy, double& Iz, double& J)
{
    if (m_HasExplicitSectionInertia)
    {
        Iy = m_ExplicitIy;
        Iz = m_ExplicitIz;
        J = m_ExplicitJ;
        Io = Iy + Iz;
        return;
    }

    Calculate_Area();
    Iy = (m_Width * pow(m_Height, 3)) / 12.0; // 绕y轴的惯性矩
    Iz = (m_Height * pow(m_Width, 3)) / 12.0; // 绕z轴的惯性矩

    double a = std::max(m_Width, m_Height);
    double b = std::min(m_Width, m_Height);
    double ratio = b / a;
    J = Iy + Iz;
    Io = J;
    Irr = (9 * pow(m_Width, 5) * m_Height + 9 * pow(m_Height, 5) * m_Width + 10 * pow(m_Height, 3) * pow(m_Width, 3)) /
          720.0;
}
