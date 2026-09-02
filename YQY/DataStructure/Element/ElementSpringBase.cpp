#include "ElementSpringBase.h"

#include <algorithm>
#include <numeric>
#include <stdexcept>

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
