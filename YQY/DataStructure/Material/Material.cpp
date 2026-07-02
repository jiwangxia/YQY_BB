#include "Material.h"

Material::Material()
{
}

Material1DResult Material::Update1D(
    double strain,
    const Material1DState& oldState) const
{
    return CalculateMaterial1D(
        m_Model,
        m_Young,
        m_YieldStress,
        m_Hardening,
        strain,
        oldState);
}
