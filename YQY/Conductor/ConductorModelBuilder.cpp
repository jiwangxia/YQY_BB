#include "ConductorModelBuilder.h"

#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Element/ElementBase.h"
#include "DataStructure/Element/ElementTruss.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Node/Node.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionBase.h"
#include "DataStructure/Structure/StructureData.h"

#include <cmath>
#include <set>

namespace Conductor
{
    int LineBuildResult::NodeCount() const
    {
        std::set<int> nodeIds;
        for (const auto& pair : subConductors)
        {
            nodeIds.insert(pair.second.nodeIds.begin(), pair.second.nodeIds.end());
        }
        for (const auto& spacer : innerSpacers)
        {
            nodeIds.insert(spacer.nodeIds.begin(), spacer.nodeIds.end());
        }
        return static_cast<int>(nodeIds.size());
    }

    int LineBuildResult::ElementCount() const
    {
        std::set<int> elementIds;
        for (const auto& pair : subConductors)
        {
            elementIds.insert(pair.second.elementIds.begin(), pair.second.elementIds.end());
        }
        for (const auto& spacer : innerSpacers)
        {
            elementIds.insert(spacer.elementIds.begin(), spacer.elementIds.end());
        }
        return static_cast<int>(elementIds.size());
    }

    ConductorModelBuilder::ConductorModelBuilder(std::shared_ptr<StructureData> structure)
        : m_ownedStructure(structure),
          m_structure(structure.get())
    {
    }

    ConductorModelBuilder::ConductorModelBuilder(StructureData* structure)
        : m_structure(structure)
    {
    }

    bool ConductorModelBuilder::BuildLine(const LineBuildConfig& config, LineBuildResult& result, std::string& error)
    {
        result = LineBuildResult{};

        if (m_structure == nullptr)
        {
            error = "StructureData 为空，无法生成导线";
            return false;
        }

        if (config.elementType == EnumKeyword::ElementType::UNKNOWN)
        {
            error = "导线单元类型不能为 UNKNOWN";
            return false;
        }

        auto property = config.property;
        if (!ValidateProperty(property, "导线", error))
        {
            return false;
        }

        auto material = property->m_pMaterial.lock();
        auto section = property->m_pSection.lock();
        if (material->m_Density <= 0.0)
        {
            error = "导线材料密度必须大于 0";
            return false;
        }

        if (section->m_Area <= 0.0)
        {
            error = "导线截面面积必须大于 0";
            return false;
        }

        if (config.conductor.nBundle <= 0)
        {
            error = "导线分裂数必须大于 0";
            return false;
        }

        if (config.conductor.nBundle != 1 &&
            config.conductor.nBundle != 2 &&
            config.conductor.nBundle != 4 &&
            config.conductor.nBundle != 6 &&
            config.conductor.nBundle != 8)
        {
            error = "导线分裂数仅支持 1、2、4、6、8";
            return false;
        }

        if (config.conductor.connecttype != ConnectionMode::Parallel)
        {
            error = "当前阶段仅支持 Parallel 导线连接方式";
            return false;
        }

        if (config.conductor.segments <= 0)
        {
            error = "导线离散段数必须大于 0";
            return false;
        }

        if (config.conductor.stress0 <= 0.0)
        {
            error = "导线初始应力必须大于 0";
            return false;
        }

        if (config.conductor.spacing < 0.0)
        {
            error = "子导线间距不能小于 0";
            return false;
        }

        Vector2d horizontalSpan = (config.end - config.start).head<2>();
        if (horizontalSpan.norm() <= 1e-7)
        {
            error = "导线水平档距过小，无法生成悬链线";
            return false;
        }

        result.start = config.start;
        result.end = config.end;
        result.spanLength = horizontalSpan.norm();

        ConductorConfig conductor = config.conductor;
        conductor.unitWeight = material->m_Density * 9.81;

        BundleResult raw = Generator::CreateBundle(
            config.start.data(),
            config.end.data(),
            config.leftCutLength,
            config.rightCutLength,
            conductor);

        if (raw.wiresNode.empty())
        {
            error = "导线几何生成失败，没有生成任何子导线节点";
            return false;
        }

        result.property = property;

        if (!AddNodes(raw, result, error))
        {
            return false;
        }

        if (!AddElements(raw, config, property, result, error))
        {
            for (const auto& pair : result.subConductors)
            {
                for (int nodeId : pair.second.nodeIds)
                {
                    m_structure->m_Nodes.erase(nodeId);
                }
            }
            result = LineBuildResult{};
            return false;
        }

        return true;
    }

    int ConductorModelBuilder::NextNodeId() const
    {
        auto nextId = [](const auto& values)
            {
                if (values.empty())
                {
                    return 1;
                }
                return values.rbegin()->first + 1;
            };

        return nextId(m_structure->m_Nodes);
    }

    int ConductorModelBuilder::NextElementId() const
    {
        auto nextId = [](const auto& values)
            {
                if (values.empty())
                {
                    return 1;
                }
                return values.rbegin()->first + 1;
            };

        return nextId(m_structure->m_Elements);
    }

    bool ConductorModelBuilder::ValidateProperty(std::shared_ptr<Property> property, const std::string& objectName, std::string& error) const
    {
        if (!property)
        {
            error = objectName + "属性为空";
            return false;
        }

        auto material = property->m_pMaterial.lock();
        if (!material)
        {
            error = objectName + "属性没有关联材料";
            return false;
        }

        auto section = property->m_pSection.lock();
        if (!section)
        {
            error = objectName + "属性没有关联截面";
            return false;
        }

        if (section->m_Area <= 0.0)
        {
            error = objectName + "截面面积必须大于 0";
            return false;
        }

        return true;
    }

    std::shared_ptr<ElementBase> ConductorModelBuilder::CreateLineElement(EnumKeyword::ElementType elementType, std::string& error) const
    {
        switch (elementType)
        {
        case EnumKeyword::ElementType::T3D2:
            return std::make_shared<ElementTruss>();
        case EnumKeyword::ElementType::CABLE:
            return std::make_shared<ElementCable>();
        case EnumKeyword::ElementType::CR3D:
        {
            auto beam = std::make_shared<ElementBeam_CR>();
            beam->q0 = Vector3d::UnitZ();
            return beam;
        }
        default:
            error = "导线单元类型未知，无法生成导线单元";
            return nullptr;
        }
    }

    void ConductorModelBuilder::PrepareElementLocalFrame(std::shared_ptr<ElementBase> element) const
    {
        auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(element);
        if (!beam)
        {
            return;
        }

        auto pNode0 = element->m_pNode[0].lock();
        auto pNode1 = element->m_pNode[1].lock();
        if (!pNode0 || !pNode1)
        {
            beam->q0 = Vector3d::UnitZ();
            return;
        }

        Vector3d p0(pNode0->m_X, pNode0->m_Y, pNode0->m_Z);
        Vector3d p1(pNode1->m_X, pNode1->m_Y, pNode1->m_Z);
        Vector3d direction = p1 - p0;
        if (direction.norm() <= 1e-10)
        {
            beam->q0 = Vector3d::UnitZ();
            return;
        }

        direction.normalize();
        beam->q0 = std::abs(direction.dot(Vector3d::UnitZ())) < 0.9 ? Vector3d::UnitZ() : Vector3d::UnitY();
    }

    int ConductorModelBuilder::FindNodeIdByRatio(const SubConductorModel& sub, double ratio) const
    {
        if (sub.nodeIds.empty())
        {
            return -1;
        }

        auto clampRatio = [](double value)
            {
                if (value < 0.0)
                {
                    return 0.0;
                }
                if (value > 1.0)
                {
                    return 1.0;
                }
                return value;
            };

        double clampedRatio = clampRatio(ratio);
        int maxIndex = static_cast<int>(sub.nodeIds.size()) - 1;
        int index = static_cast<int>(std::round(clampedRatio * maxIndex));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > maxIndex)
        {
            index = maxIndex;
        }
        return sub.nodeIds[index];
    }

    bool ConductorModelBuilder::AddElement(
        int iNodeId,
        int jNodeId,
        EnumKeyword::ElementType elementType,
        std::shared_ptr<Property> property,
        double initStress,
        int& elementId,
        std::string& error)
    {
        auto iNode = m_structure->FindNode(iNodeId);
        auto jNode = m_structure->FindNode(jNodeId);
        if (!iNode || !jNode)
        {
            error = "单元端点节点不存在";
            return false;
        }

        elementId = NextElementId();
        auto element = CreateLineElement(elementType, error);
        if (!element)
        {
            return false;
        }

        element->m_Id = elementId;
        element->m_pNode[0] = iNode;
        element->m_pNode[1] = jNode;
        element->m_pProperty = property;
        element->m_InitStress = initStress;
        PrepareElementLocalFrame(element);
        element->Get_L0();

        auto inserted = m_structure->m_Elements.insert({ elementId, element });
        if (!inserted.second)
        {
            error = "添加单元失败";
            return false;
        }

        return true;
    }

    bool ConductorModelBuilder::BuildInnerSpacer(LineBuildResult& line, const InnerSpacerConfig& config, InnerSpacerModel& spacer, std::string& error)
    {
        spacer = InnerSpacerModel{};
        std::vector<int> createdNodeIds;

        if (m_structure == nullptr)
        {
            error = "StructureData 为空，无法生成相内间隔棒";
            return false;
        }

        if (line.subConductors.size() < 2)
        {
            error = "单分裂导线不需要相内间隔棒";
            return false;
        }

        if (line.spanLength <= 1e-7)
        {
            error = "导线档距无效，无法定位相内间隔棒";
            return false;
        }

        if (config.elementType == EnumKeyword::ElementType::UNKNOWN)
        {
            error = "相内间隔棒单元类型不能为 UNKNOWN";
            return false;
        }

        if (!ValidateProperty(config.property, "相内间隔棒", error))
        {
            return false;
        }

        int expectedNodeCount = -1;
        for (const auto& pair : line.subConductors)
        {
            int currentCount = static_cast<int>(pair.second.nodeIds.size());
            if (currentCount == 0)
            {
                error = "相内间隔棒子导线节点为空";
                return false;
            }

            if (expectedNodeCount == -1)
            {
                expectedNodeCount = currentCount;
            }
            else if (currentCount != expectedNodeCount)
            {
                error = "相内间隔棒要求各子导线节点数量一致";
                return false;
            }
        }

        auto clampRatio = [](double value)
            {
                if (value < 0.0)
                {
                    return 0.0;
                }
                if (value > 1.0)
                {
                    return 1.0;
                }
                return value;
            };

        double ratio = config.useRatio ? config.position : config.position / line.spanLength;
        ratio = clampRatio(ratio);
        spacer.ratio = ratio;
        spacer.position = ratio * line.spanLength;

        std::vector<int> wireNodeIds;
        wireNodeIds.reserve(line.subConductors.size());

        Vector3d center = Vector3d::Zero();
        for (const auto& pair : line.subConductors)
        {
            int nodeId = FindNodeIdByRatio(pair.second, ratio);
            auto node = m_structure->FindNode(nodeId);
            if (!node)
            {
                error = "相内间隔棒找不到子导线节点";
                return false;
            }

            wireNodeIds.push_back(nodeId);
            spacer.nodeIds.push_back(nodeId);
            center += Vector3d(node->m_X, node->m_Y, node->m_Z);
        }

        center /= static_cast<double>(wireNodeIds.size());

        int centerNodeId = -1;
        if (wireNodeIds.size() > 2 && config.createCenterNode)
        {
            centerNodeId = NextNodeId();
            auto centerNode = std::make_shared<Node>();
            centerNode->m_Id = centerNodeId;
            centerNode->m_X = center.x();
            centerNode->m_Y = center.y();
            centerNode->m_Z = center.z();

            auto inserted = m_structure->m_Nodes.insert({ centerNodeId, centerNode });
            if (!inserted.second)
            {
                error = "添加相内间隔棒中心节点失败";
                return false;
            }

            createdNodeIds.push_back(centerNodeId);
            spacer.centerNodeId = centerNodeId;
            spacer.nodeIds.push_back(centerNodeId);
        }

        auto rollback = [&]()
            {
                for (int elementId : spacer.elementIds)
                {
                    m_structure->m_Elements.erase(elementId);
                }
                for (int nodeId : createdNodeIds)
                {
                    m_structure->m_Nodes.erase(nodeId);
                }
                spacer = InnerSpacerModel{};
            };

        auto addSpacerElement = [&](int iNodeId, int jNodeId) -> bool
            {
                int elementId = -1;
                if (!AddElement(iNodeId, jNodeId, config.elementType, config.property, 0.0, elementId, error))
                {
                    return false;
                }

                if (spacer.elementIds.empty())
                {
                    spacer.id = elementId;
                }
                spacer.elementIds.push_back(elementId);
                return true;
            };

        if (wireNodeIds.size() == 2)
        {
            if (!addSpacerElement(wireNodeIds[0], wireNodeIds[1]))
            {
                error = "添加二分裂相内间隔棒失败";
                rollback();
                return false;
            }
        }
        else
        {
            for (int i = 0; i < static_cast<int>(wireNodeIds.size()); ++i)
            {
                int j = (i + 1) % static_cast<int>(wireNodeIds.size());
                if (!addSpacerElement(wireNodeIds[i], wireNodeIds[j]))
                {
                    error = "添加相内间隔棒外框单元失败";
                    rollback();
                    return false;
                }
            }

            if (centerNodeId != -1)
            {
                for (int nodeId : wireNodeIds)
                {
                    if (!addSpacerElement(nodeId, centerNodeId))
                    {
                        error = "添加相内间隔棒星形单元失败";
                        rollback();
                        return false;
                    }
                }
            }
        }

        line.innerSpacers.push_back(spacer);
        return true;
    }

    bool ConductorModelBuilder::BuildInnerSpacers(LineBuildResult& line, const std::vector<InnerSpacerConfig>& configs, std::string& error)
    {
        for (const auto& config : configs)
        {
            InnerSpacerModel spacer;
            if (!BuildInnerSpacer(line, config, spacer, error))
            {
                return false;
            }
        }
        return true;
    }

    bool ConductorModelBuilder::AddNodes(BundleResult& raw, LineBuildResult& result, std::string& error)
    {
        std::vector<int> createdNodeIds;

        for (auto& pair : raw.wiresNode)
        {
            int wireId = pair.first;
            auto& rawNodes = pair.second;

            if (rawNodes.empty())
            {
                error = "子导线节点列表为空";
                return false;
            }

            SubConductorModel sub;
            sub.wireId = wireId;
            sub.nodeIds.reserve(rawNodes.size());

            for (auto& rawNode : rawNodes)
            {
                int nodeId = NextNodeId();
                auto node = std::make_shared<Node>();
                node->m_Id = nodeId;
                node->m_X = rawNode.x;
                node->m_Y = rawNode.y;
                node->m_Z = rawNode.z;

                auto inserted = m_structure->m_Nodes.insert({ nodeId, node });
                if (!inserted.second)
                {
                    error = "添加导线节点失败";
                    for (int createdNodeId : createdNodeIds)
                    {
                        m_structure->m_Nodes.erase(createdNodeId);
                    }
                    result = LineBuildResult{};
                    return false;
                }

                createdNodeIds.push_back(nodeId);
                rawNode.id = nodeId;
                sub.nodeIds.push_back(nodeId);
            }

            result.subConductors.insert({ wireId, sub });
        }

        return true;
    }

    bool ConductorModelBuilder::AddElements(const BundleResult& raw, const LineBuildConfig& config, std::shared_ptr<Property> property, LineBuildResult& result, std::string& error)
    {
        std::vector<int> createdElementIds;

        for (const auto& pair : raw.wiresElement)
        {
            int wireId = pair.first;
            const auto& rawElements = pair.second;
            auto rawNodeIt = raw.wiresNode.find(wireId);
            auto subIt = result.subConductors.find(wireId);

            if (rawNodeIt == raw.wiresNode.end() || subIt == result.subConductors.end())
            {
                error = "导线单元找不到对应的子导线节点";
                return false;
            }

            const auto& rawNodes = rawNodeIt->second;
            auto& sub = subIt->second;
            sub.elementIds.reserve(rawElements.size());

            for (const auto& rawElement : rawElements)
            {
                int iIndex = rawElement.iNode - 1;
                int jIndex = rawElement.jNode - 1;
                if (iIndex < 0 || jIndex < 0 ||
                    iIndex >= static_cast<int>(rawNodes.size()) ||
                    jIndex >= static_cast<int>(rawNodes.size()))
                {
                    error = "导线单元局部节点索引越界";
                    return false;
                }

                int elementId = -1;
                if (!AddElement(rawNodes[iIndex].id, rawNodes[jIndex].id, config.elementType, property, rawElement.stress0, elementId, error))
                {
                    error = "添加导线单元失败：" + error;
                    for (int createdElementId : createdElementIds)
                    {
                        m_structure->m_Elements.erase(createdElementId);
                    }
                    return false;
                }

                createdElementIds.push_back(elementId);
                sub.elementIds.push_back(elementId);
            }
        }

        return true;
    }
}
