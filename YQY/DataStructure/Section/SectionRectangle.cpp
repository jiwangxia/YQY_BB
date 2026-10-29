#include "SectionRectangle.h"

void SectionRectangle::Calculate_Area()
{
    m_Area = m_Width * m_Height;
}

void SectionRectangle::Calculate_I(_OUT double& Iy, _OUT double& Iz, _OUT double& J)
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

    const double a = std::max(m_Width, m_Height);
    const double b = std::min(m_Width, m_Height);
    Io = Iy + Iz;
    J = 0.0;
    if (b > 0.0)
    {
        // Saint-Venant 矩形自由扭转常数；极惯量 Io 只用于截面转动质量。
        const double pi = std::acos(-1.0);
        double series = 0.0;
        for (int n = 1; n < 1000; n += 2)
            series += std::tanh(n * pi * a / (2.0 * b)) / std::pow(static_cast<double>(n), 5);
        J = a * b * b * b / 3.0 * (1.0 - 192.0 * b * series / (std::pow(pi, 5) * a));
    }
    Irr = (9 * pow(m_Width, 5) * m_Height + 9 * pow(m_Height, 5) * m_Width + 10 * pow(m_Height, 3) * pow(m_Width, 3)) /
          720.0;
}
