#include "ElementSpringBase.h"
#include "ElementSpring1.h"
#include "ElementSpring2.h"
#include "ElementSpringA.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

bool ElementSpringBase::TryGetEnergyDisplacement(_OUT double& displacement) const
{
    const auto first = m_pNode.isEmpty() ? nullptr : m_pNode[0].lock();
    if (!first)
        return false;
    if (const auto* spring = dynamic_cast<const ElementSpring1*>(this))
    {
        if (spring->m_DOF < 0 || spring->m_DOF >= static_cast<int>(first->m_Displacement.size()))
            return false;
        displacement = first->m_Displacement[spring->m_DOF];
        return std::isfinite(displacement);
    }
    const auto second = m_pNode.size() > 1 ? m_pNode[1].lock() : nullptr;
    if (!second)
        return false;
    if (const auto* spring = dynamic_cast<const ElementSpring2*>(this))
    {
        if (spring->m_FirstDOF < 0 || spring->m_SecondDOF < 0 ||
            spring->m_FirstDOF >= static_cast<int>(first->m_Displacement.size()) ||
            spring->m_SecondDOF >= static_cast<int>(second->m_Displacement.size()))
            return false;
        displacement = first->m_Displacement[spring->m_FirstDOF] - second->m_Displacement[spring->m_SecondDOF];
        return std::isfinite(displacement);
    }
    if (!dynamic_cast<const ElementSpringA*>(this) || first->m_Displacement.size() < 3 ||
        second->m_Displacement.size() < 3 || L0 <= 0.0)
        return false;
    const Vector3d chord(second->m_X + second->m_Displacement[0] - first->m_X - first->m_Displacement[0],
                          second->m_Y + second->m_Displacement[1] - first->m_Y - first->m_Displacement[1],
                          second->m_Z + second->m_Displacement[2] - first->m_Z - first->m_Displacement[2]);
    displacement = chord.norm() - L0;
    return std::isfinite(displacement);
}

bool ElementSpringBase::TryGetStrainEnergy(_OUT double& energy) const
{
    energy = std::numeric_limits<double>::quiet_NaN();
    const auto behavior = m_pSpringBehavior.lock();
    double displacement = 0.0;
    if (!behavior || behavior->m_Points.size() < 2 || !TryGetEnergyDisplacement(displacement))
        return false;
    const auto& points = behavior->m_Points;
    const double lower = std::min(0.0, displacement);
    const double upper = std::max(0.0, displacement);
    double integral = 0.0;
    for (std::size_t index = 0; index < points.size(); ++index)
    {
        const auto& current = points[index];
        if (!std::isfinite(current.force) || !std::isfinite(current.displacement))
            return false;
        if (index == 0)
            continue;
        const auto& previous = points[index - 1];
        if (current.displacement <= previous.displacement)
            return false;
        const double left = std::max(lower, previous.displacement);
        const double right = std::min(upper, current.displacement);
        if (right <= left)
            continue;
        const double slope = (current.force - previous.force) / (current.displacement - previous.displacement);
        integral += (right - left) *
                    (previous.force + 0.5 * slope * (left + right - 2.0 * previous.displacement));
    }
    // 与生产插值一致，曲线区间外保持端点常力；能量零点取相对位移零。
    integral += std::max(0.0, std::min(upper, points.front().displacement) - lower) * points.front().force;
    integral += std::max(0.0, upper - std::max(lower, points.back().displacement)) * points.back().force;
    const double value = displacement < 0.0 ? -integral : integral;
    if (!std::isfinite(value))
        return false;
    energy = value;
    return true;
}

bool ElementSpringBase::TryGetKineticEnergy(_OUT double& energy)
{
    energy = 0.0;
    return true;
}

void ElementSpringBase::Get_me_Lumped(_OUT MatrixXd& me)
{
    std::vector<int> localDofCounts;
    GetNodeLocalDOFCounts(localDofCounts);
    const int dofCount = std::accumulate(localDofCounts.cbegin(), localDofCounts.cend(), 0);
    me = MatrixXd::Zero(dofCount, dofCount);
}

void ElementSpringBase::Get_me_Consistent(_OUT MatrixXd& me)
{
    Get_me_Lumped(me);
}

void ElementSpringBase::Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce)
{
    Q_UNUSED(damping);
    Get_me_Lumped(ce);
}

double ElementSpringBase::GetCurrentTangent() const
{
    const auto behavior = m_pSpringBehavior.lock();
    if (!behavior || behavior->m_Points.size() < 2)
        return 0.0;

    const auto& points = behavior->m_Points;
    const double relativeDisplacement = m_CurrentRelativeDisplacement;
    if (relativeDisplacement < points.front().displacement || relativeDisplacement >= points.back().displacement)
        return 0.0;

    const auto upper = std::upper_bound(points.cbegin(), points.cend(), relativeDisplacement,
                                        [](double value, const SpringForceDisplacementPoint& point)
                                        { return value < point.displacement; });
    const SpringForceDisplacementPoint& second = *upper;
    const SpringForceDisplacementPoint& first = *(upper - 1);
    return (second.force - first.force) / (second.displacement - first.displacement);
}

void ElementSpringBase::EvaluateBehavior(double relativeDisplacement, _OUT double& force, _OUT double& tangent)
{
    const auto behavior = m_pSpringBehavior.lock();
    if (!behavior || behavior->m_Points.size() < 2)
        throw std::runtime_error("Spring behavior is missing or has fewer than two data points.");

    const auto& points = behavior->m_Points;
    // 首点进入第一段，首点之前保持区间外常力、零切线。
    if (relativeDisplacement < points.front().displacement)
    {
        force = points.front().force;
        tangent = 0.0;
        m_CurrentForce = force;
        m_CurrentRelativeDisplacement = relativeDisplacement;
        return;
    }
    if (relativeDisplacement >= points.back().displacement)
    {
        force = points.back().force;
        tangent = 0.0;
        m_CurrentForce = force;
        m_CurrentRelativeDisplacement = relativeDisplacement;
        return;
    }

    const auto upper = std::upper_bound(points.cbegin(), points.cend(), relativeDisplacement,
                                        [](double value, const SpringForceDisplacementPoint& point)
                                        { return value < point.displacement; });
    const SpringForceDisplacementPoint& second = *upper;
    const SpringForceDisplacementPoint& first = *(upper - 1);
    tangent = (second.force - first.force) / (second.displacement - first.displacement);
    force = first.force + tangent * (relativeDisplacement - first.displacement);
    m_CurrentForce = force;
    m_CurrentRelativeDisplacement = relativeDisplacement;
}
