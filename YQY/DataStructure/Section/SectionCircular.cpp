#include "SectionCircular.h"

#include "Base/Base.h"

void SectionCircular::Calculate_Radius()
{
    m_Radius = sqrt(m_Area / PI);
}

void SectionCircular::Calculate_I(double& Iy, double& Iz,double& J)
{
    if (m_HasExplicitSectionInertia)
    {
        Iy = m_ExplicitIy;
        Iz = m_ExplicitIz;
        J = m_ExplicitJ;
        Io = Iy + Iz;
        return;
    }

    Calculate_Radius();
    Iy = Iz = PI * pow(m_Radius, 4) / 4.0;
    J = Iy + Iz;  // 极惯性矩近似
    Io = J;
    Irr = PI * pow(m_Radius, 6) / 3.0;
}
