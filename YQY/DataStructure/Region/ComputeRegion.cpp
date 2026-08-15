#include "ComputeRegion.h"

#include <algorithm>
#include <iterator>

static bool HasIntersection(const std::set<int>& lhs, const std::set<int>& rhs)
{
    auto lhsIt = lhs.cbegin();
    auto rhsIt = rhs.cbegin();
    while (lhsIt != lhs.cend() && rhsIt != rhs.cend())
    {
        if (*lhsIt == *rhsIt)
        {
            return true;
        }
        if (*lhsIt < *rhsIt)
        {
            ++lhsIt;
        }
        else
        {
            ++rhsIt;
        }
    }
    return false;
}
bool ComputeRegion::ContainsNode(int nodeId) const
{
    return m_NodeIds.find(nodeId) != m_NodeIds.cend();
}

bool ComputeRegion::ContainsElement(int elementId) const
{
    return m_ElementIds.find(elementId) != m_ElementIds.cend();
}

bool ComputeRegion::Overlaps(const ComputeRegion& other) const
{
    return HasIntersection(m_NodeIds, other.m_NodeIds) || HasIntersection(m_ElementIds, other.m_ElementIds);
}

void ComputeRegion::MergeFrom(const ComputeRegion& other)
{
    m_SourceSetIds.insert(other.m_SourceSetIds.cbegin(), other.m_SourceSetIds.cend());
    m_DirectNodeIds.insert(other.m_DirectNodeIds.cbegin(), other.m_DirectNodeIds.cend());
    m_DirectElementIds.insert(other.m_DirectElementIds.cbegin(), other.m_DirectElementIds.cend());
    m_NodeIds.insert(other.m_NodeIds.cbegin(), other.m_NodeIds.cend());
    m_ElementIds.insert(other.m_ElementIds.cbegin(), other.m_ElementIds.cend());
    m_Enabled = m_Enabled || other.m_Enabled;
}
