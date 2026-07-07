#include "Constraint.h"

#include <cmath>
#include <stdexcept>
#include <utility>

void Constraint::SetTimePoints(std::vector<ConstraintTimePoint> points)
{
    if (points.size() < 2)
    {
        throw std::invalid_argument(
            "Constraint TABULAR history requires at least two points");
    }

    for (size_t i = 0; i < points.size(); ++i)
    {
        if (!std::isfinite(points[i].time)
            || !std::isfinite(points[i].value))
        {
            throw std::invalid_argument(
                "Constraint TABULAR history contains a non-finite value");
        }
        if (i > 0 && points[i].time <= points[i - 1].time)
        {
            throw std::invalid_argument(
                "Constraint TABULAR time values must increase strictly");
        }
    }

    m_TimePoints = std::move(points);
}

bool Constraint::HasTimePoints() const
{
    return !m_TimePoints.empty();
}

double Constraint::GetValue(double currentTime, double factor) const
{
    // 旧四字段约束没有时程，继续保持 value × factor 的原有行为。
    if (m_TimePoints.empty())
    {
        return m_Value * factor;
    }

    if (currentTime <= m_TimePoints.front().time)
    {
        return m_Value * m_TimePoints.front().value;
    }
    if (currentTime >= m_TimePoints.back().time)
    {
        return m_Value * m_TimePoints.back().value;
    }

    for (size_t i = 1; i < m_TimePoints.size(); ++i)
    {
        const ConstraintTimePoint& right = m_TimePoints[i];
        if (currentTime <= right.time)
        {
            const ConstraintTimePoint& left = m_TimePoints[i - 1];
            const double ratio =
                (currentTime - left.time) / (right.time - left.time);
            const double scale =
                left.value + ratio * (right.value - left.value);
            return m_Value * scale;
        }
    }

    throw std::runtime_error("Constraint TABULAR interpolation failed");
}
