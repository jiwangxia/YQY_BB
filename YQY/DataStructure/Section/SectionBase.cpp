#include "SectionBase.h"

#include "SectionCircular.h"
#include "SectionRectangle.h"
#include "Section_Ice.h"

#include <algorithm>
#include <cmath>

SectionBase::SectionBase()
{
}

bool SectionBase::TryGetEnergyInertia(_OUT double& Iy, _OUT double& Iz, _OUT double& J) const
{
    const auto* circular = dynamic_cast<const SectionCircular*>(this);
    const auto* rectangle = dynamic_cast<const SectionRectangle*>(this);
    const auto* ice = dynamic_cast<const Section_Ice*>(this);
    std::array<double, 6> parameters{};
    if (circular || rectangle)
    {
        parameters = {m_HasExplicitSectionInertia ? 1.0 : 0.0, m_ExplicitIy, m_ExplicitIz, m_ExplicitJ,
                      rectangle ? rectangle->m_Width : m_Area, rectangle ? rectangle->m_Height : 0.0};
    }
    else if (ice)
    {
        if (m_Radius <= 0.0)
            return false;
        parameters = {m_Area, m_Radius, ice->m_IceThickness, ice->m_IceAngle, ice->m_IceHalfAngle,
                      ice->m_IceShapeFactor};
    }
    else
        return false;
    if (!std::all_of(parameters.cbegin(), parameters.cend(), [](double value) { return std::isfinite(value); }))
        return false;
    if (m_energyInertiaValid && parameters == m_energyInertiaParameters)
    {
        Iy = m_energyInertia[0];
        Iz = m_energyInertia[1];
        J = m_energyInertia[2];
        return true;
    }

    // 参数改变时才调用既有公式，避免每个单元、每帧重复矩形级数或覆冰数值积分。
    if (circular)
    {
        SectionCircular copy(*circular);
        copy.Calculate_I(Iy, Iz, J);
    }
    else if (rectangle)
    {
        SectionRectangle copy(*rectangle);
        copy.Calculate_I(Iy, Iz, J);
    }
    else
    {
        Section_Ice copy(*ice);
        copy.Calculate_I(Iy, Iz, J);
    }
    if (!std::isfinite(Iy) || !std::isfinite(Iz) || !std::isfinite(J) || Iy < 0.0 || Iz < 0.0 || J < 0.0)
        return false;
    m_energyInertiaParameters = parameters;
    m_energyInertia = {Iy, Iz, J};
    m_energyInertiaValid = true;
    return true;
}
