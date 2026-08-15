#include "ResultFrameUtilities.h"

#include "DataStructure/Structure/StructureData.h"

#include <algorithm>
#include <cmath>

namespace ResultFrameUtilities
{
bool modelsMatch(const std::shared_ptr<StructureData>& current, const std::shared_ptr<StructureData>& embedded)
{
    if (!current || !embedded || current->m_Nodes.size() != embedded->m_Nodes.size() ||
        current->m_Elements.size() != embedded->m_Elements.size())
    {
        return false;
    }

    constexpr double coordinateTolerance = 1.0e-10;
    for (const auto& [nodeId, embeddedNode] : embedded->m_Nodes)
    {
        const auto found = current->m_Nodes.find(nodeId);
        if (found == current->m_Nodes.end() || !found->second || !embeddedNode ||
            std::abs(found->second->m_X - embeddedNode->m_X) > coordinateTolerance ||
            std::abs(found->second->m_Y - embeddedNode->m_Y) > coordinateTolerance ||
            std::abs(found->second->m_Z - embeddedNode->m_Z) > coordinateTolerance)
        {
            return false;
        }
    }

    for (const auto& [elementId, embeddedElement] : embedded->m_Elements)
    {
        const auto found = current->m_Elements.find(elementId);
        if (found == current->m_Elements.end() || !found->second || !embeddedElement ||
            found->second->m_pNode.size() != embeddedElement->m_pNode.size())
        {
            return false;
        }
        for (int nodeIndex = 0; nodeIndex < embeddedElement->m_pNode.size(); ++nodeIndex)
        {
            const auto currentNode = found->second->m_pNode[nodeIndex].lock();
            const auto embeddedNode = embeddedElement->m_pNode[nodeIndex].lock();
            if (!currentNode || !embeddedNode || currentNode->m_Id != embeddedNode->m_Id)
                return false;
        }
    }
    return true;
}

Hdf5ResultFrame interpolate(const Hdf5ResultFrame& first, const Hdf5ResultFrame& second, double interpolation)
{
    interpolation = std::clamp(interpolation, 0.0, 1.0);
    const double firstWeight = 1.0 - interpolation;
    Hdf5ResultFrame result = first;
    result.info.time = first.info.time * firstWeight + second.info.time * interpolation;
    result.info.loadFactor = first.info.loadFactor * firstWeight + second.info.loadFactor * interpolation;

    if (result.nodes.size() == second.nodes.size())
    {
        for (std::size_t index = 0; index < result.nodes.size(); ++index)
        {
            Hdf5NodalResult& target = result.nodes[index];
            const Hdf5NodalResult& next = second.nodes[index];
            if (target.id != next.id)
                continue;
            for (int component = 0; component < 3; ++component)
            {
                target.displacement[component] =
                    target.displacement[component] * firstWeight + next.displacement[component] * interpolation;
                target.currentCoordinate[component] = target.currentCoordinate[component] * firstWeight +
                                                      next.currentCoordinate[component] * interpolation;
            }
        }
    }

    if (result.elements.size() == second.elements.size())
    {
        for (std::size_t index = 0; index < result.elements.size(); ++index)
        {
            Hdf5ElementResult& target = result.elements[index];
            const Hdf5ElementResult& next = second.elements[index];
            if (target.id != next.id)
                continue;
            target.axialForce = target.axialForce * firstWeight + next.axialForce * interpolation;
            target.currentStress = target.currentStress * firstWeight + next.currentStress * interpolation;
            target.strain = target.strain * firstWeight + next.strain * interpolation;
        }
    }
    return result;
}
}
