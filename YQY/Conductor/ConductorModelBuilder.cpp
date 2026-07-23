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

#include <algorithm>
#include <cmath>
#include <limits>
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
        if (leftSupportNodeId > 0)
            nodeIds.insert(leftSupportNodeId);
        if (rightSupportNodeId > 0)
            nodeIds.insert(rightSupportNodeId);
        nodeIds.insert(leftTensionEnd.groupNodeIds.begin(), leftTensionEnd.groupNodeIds.end());
        nodeIds.insert(rightTensionEnd.groupNodeIds.begin(), rightTensionEnd.groupNodeIds.end());
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
        elementIds.insert(leftTensionEnd.yokeElementIds.begin(), leftTensionEnd.yokeElementIds.end());
        elementIds.insert(rightTensionEnd.yokeElementIds.begin(), rightTensionEnd.yokeElementIds.end());
        if (leftTensionEnd.stabilizerElementId > 0)
            elementIds.insert(leftTensionEnd.stabilizerElementId);
        if (rightTensionEnd.stabilizerElementId > 0)
            elementIds.insert(rightTensionEnd.stabilizerElementId);
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

        if (config.bundleEndTransitionLength < 0.0)
        {
            error = "导线端部过渡长度不能小于 0";
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

        double leftCutLength = config.leftCutLength;
        double rightCutLength = config.rightCutLength;
        const bool convergeEnds = config.convergeBundleEnds && conductor.nBundle > 1;
        if (convergeEnds && config.bundleEndTransitionLength > 0.0)
        {
            // CreateBundle 的截断长度就是公共支点至首/末分裂截面的弧长。
            // 未显式设置截断长度时保留一段实际过渡，避免分裂半径在近零距离内突变。
            const double transitionLength =
                std::min(config.bundleEndTransitionLength, result.spanLength * 0.1);
            if (leftCutLength <= 1e-7)
                leftCutLength = transitionLength;
            if (rightCutLength <= 1e-7)
                rightCutLength = transitionLength;
        }

        BundleResult raw = Generator::CreateBundle(
            config.start.data(),
            config.end.data(),
            leftCutLength,
            rightCutLength,
            conductor);

        if (raw.wiresNode.empty())
        {
            error = "导线几何生成失败，没有生成任何子导线节点";
            return false;
        }

        result.property = property;

        if (!AddNodes(raw, config, result, error))
        {
            return false;
        }

        auto rollbackBuiltLine = [&]()
            {
                std::set<int> elementIds;
                std::set<int> nodeIds;
                for (const auto& pair : result.subConductors)
                {
                    elementIds.insert(pair.second.elementIds.begin(), pair.second.elementIds.end());
                    nodeIds.insert(pair.second.nodeIds.begin(), pair.second.nodeIds.end());
                }
                elementIds.insert(result.leftTensionEnd.yokeElementIds.begin(), result.leftTensionEnd.yokeElementIds.end());
                elementIds.insert(result.rightTensionEnd.yokeElementIds.begin(), result.rightTensionEnd.yokeElementIds.end());
                if (result.leftTensionEnd.stabilizerElementId > 0)
                    elementIds.insert(result.leftTensionEnd.stabilizerElementId);
                if (result.rightTensionEnd.stabilizerElementId > 0)
                    elementIds.insert(result.rightTensionEnd.stabilizerElementId);
                nodeIds.insert(result.leftTensionEnd.groupNodeIds.begin(), result.leftTensionEnd.groupNodeIds.end());
                nodeIds.insert(result.rightTensionEnd.groupNodeIds.begin(), result.rightTensionEnd.groupNodeIds.end());
                if (result.leftSupportNodeId > 0)
                    nodeIds.insert(result.leftSupportNodeId);
                if (result.rightSupportNodeId > 0)
                    nodeIds.insert(result.rightSupportNodeId);
                for (int elementId : elementIds)
                    m_structure->m_Elements.erase(elementId);
                for (int nodeId : nodeIds)
                    m_structure->m_Nodes.erase(nodeId);
                result = LineBuildResult{};
            };

        if (!AddElements(raw, config, property, result, error))
        {
            rollbackBuiltLine();
            return false;
        }

        if (!AddTensionEndElements(config, result, error))
        {
            rollbackBuiltLine();
            return false;
        }

        if (!CreateSubConductorSets(config, result, error))
        {
            rollbackBuiltLine();
            return false;
        }

        if (!RenumberLineModel(result, error))
        {
            rollbackBuiltLine();
            return false;
        }

        return true;
    }

    bool ConductorModelBuilder::BuildSpanConductor(const SpanConductorBuildConfig& config, LineBuildResult& result, std::string& error)
    {
        if (!BuildLine(config.line, result, error))
        {
            return false;
        }

        auto rollbackLine = [&]()
            {
                for (const auto& spacer : result.innerSpacers)
                {
                    for (int elementId : spacer.elementIds)
                    {
                        m_structure->m_Elements.erase(elementId);
                    }
                    if (spacer.centerNodeId != -1)
                    {
                        m_structure->m_Nodes.erase(spacer.centerNodeId);
                    }
                }

                for (const auto& pair : result.subConductors)
                {
                    if (pair.second.nodeSetId > 0)
                    {
                        m_structure->m_ModelSets.erase(pair.second.nodeSetId);
                    }
                    if (pair.second.elementSetId > 0)
                    {
                        m_structure->m_ModelSets.erase(pair.second.elementSetId);
                    }
                    for (int elementId : pair.second.elementIds)
                    {
                        m_structure->m_Elements.erase(elementId);
                    }
                    for (int nodeId : pair.second.nodeIds)
                    {
                        m_structure->m_Nodes.erase(nodeId);
                    }
                }
                for (int elementId : result.leftTensionEnd.yokeElementIds)
                    m_structure->m_Elements.erase(elementId);
                for (int elementId : result.rightTensionEnd.yokeElementIds)
                    m_structure->m_Elements.erase(elementId);
                if (result.leftTensionEnd.stabilizerElementId > 0)
                    m_structure->m_Elements.erase(result.leftTensionEnd.stabilizerElementId);
                if (result.rightTensionEnd.stabilizerElementId > 0)
                    m_structure->m_Elements.erase(result.rightTensionEnd.stabilizerElementId);
                if (result.leftSupportNodeId > 0)
                    m_structure->m_Nodes.erase(result.leftSupportNodeId);
                if (result.rightSupportNodeId > 0)
                    m_structure->m_Nodes.erase(result.rightSupportNodeId);
                result = LineBuildResult{};
            };

        if (!config.innerSpacers.empty() && !BuildInnerSpacers(result, config.innerSpacers, error))
        {
            rollbackLine();
            return false;
        }

        if (config.useInnerSpacerLayout)
        {
            std::vector<InnerSpacerConfig> autoSpacers;
            if (!CalculateInnerSpacerConfigs(result, config.innerSpacerLayout, autoSpacers, error) ||
                !BuildInnerSpacers(result, autoSpacers, error))
            {
                rollbackLine();
                return false;
            }
        }

        if (!RenumberLineModel(result, error))
        {
            rollbackLine();
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

    int ConductorModelBuilder::FindNearestNodeOnSubConductor(
        const SubConductorModel& sub,
        const Vector3d& leftBase,
        const Vector2d& direction,
        double targetDistance,
        bool excludeEndpoints) const
    {
        if (sub.nodeIds.empty())
        {
            return -1;
        }

        int bestNodeId = -1;
        double bestDistance = std::numeric_limits<double>::max();

        std::size_t beginIndex = 0;
        std::size_t endIndex = sub.nodeIds.size();
        if (excludeEndpoints && sub.nodeIds.size() > 2)
        {
            beginIndex = 1;
            endIndex -= 1;
        }

        for (std::size_t index = beginIndex; index < endIndex; ++index)
        {
            int nodeId = sub.nodeIds[index];
            auto node = m_structure->FindNode(nodeId);
            if (!node)
            {
                continue;
            }

            Vector2d delta(node->m_X - leftBase.x(), node->m_Y - leftBase.y());
            double projectedDistance = delta.dot(direction);
            double distance = std::abs(projectedDistance - targetDistance);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestNodeId = nodeId;
            }
        }

        return bestNodeId;
    }

    bool ConductorModelBuilder::AddElement(
        int iNodeId,
        int jNodeId,
        EnumKeyword::ElementType elementType,
        std::shared_ptr<Property> property,
        double initStress,
        int& elementId,
        std::string& error,
        ElementRole role,
        int wireId,
        int aeroProfileId)
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
        element->m_Role = role;
        element->m_WireId = wireId;
        element->m_AeroProfileId = aeroProfileId;
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

        Vector2d horizontalSpan = (line.end - line.start).head<2>();
        if (horizontalSpan.norm() <= 1e-7)
        {
            error = "导线水平档距过小，无法生成相内间隔棒";
            return false;
        }
        Vector2d direction = horizontalSpan.normalized();

        std::vector<int> wireNodeIds;
        wireNodeIds.reserve(line.subConductors.size());

        Vector3d center = Vector3d::Zero();
        for (const auto& pair : line.subConductors)
        {
            const bool excludeEndpoints = spacer.position > 1e-7 &&
                                          spacer.position < line.spanLength - 1e-7;
            int nodeId = FindNearestNodeOnSubConductor(
                pair.second, line.start, direction, spacer.position, excludeEndpoints);
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
                if (!AddElement(iNodeId, jNodeId, config.elementType, config.property, 0.0, elementId, error,
                    ElementRole::IntraPhaseSpacer))
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
        int oldSpacerCount = static_cast<int>(line.innerSpacers.size());

        for (const auto& config : configs)
        {
            InnerSpacerModel spacer;
            if (!BuildInnerSpacer(line, config, spacer, error))
            {
                for (int i = static_cast<int>(line.innerSpacers.size()) - 1; i >= oldSpacerCount; --i)
                {
                    for (int elementId : line.innerSpacers[i].elementIds)
                    {
                        m_structure->m_Elements.erase(elementId);
                    }
                    if (line.innerSpacers[i].centerNodeId != -1)
                    {
                        m_structure->m_Nodes.erase(line.innerSpacers[i].centerNodeId);
                    }
                }
                line.innerSpacers.resize(oldSpacerCount);
                return false;
            }
        }
        return true;
    }

    std::vector<double> ConductorModelBuilder::CalculateStandardInnerSpacerPositions(double spanLength)
    {
        std::vector<double> positions;
        if (spanLength <= 1e-7)
            return positions;

        const double anchorOffset = std::min(0.1, spanLength * 0.01);
        positions.push_back(anchorOffset);
        positions.push_back(spanLength - anchorOffset);

        constexpr double firstSpan = 30.0;
        constexpr double secondSpan = 50.0;
        constexpr double maxNextSpan = 62.5;
        constexpr double minAllowableSpan = 20.0;
        constexpr double asymmetryOffset = 5.0;

        if (spanLength <= firstSpan * 2.0)
        {
            if (spanLength > minAllowableSpan)
                positions.push_back(spanLength * 0.5);
        }
        else if (spanLength <= (firstSpan + secondSpan) * 2.0)
        {
            positions.push_back(firstSpan);
            positions.push_back(spanLength - firstSpan - asymmetryOffset);
        }
        else
        {
            const double left1 = firstSpan;
            const double left2 = firstSpan + secondSpan;
            const double right2 = spanLength - (firstSpan + secondSpan);
            const double right1 = spanLength - firstSpan;
            positions.push_back(left1);
            positions.push_back(left2);

            const double midLength = right2 - left2;
            if (midLength > 0.0)
            {
                const int midSegments = static_cast<int>(std::ceil(midLength / maxNextSpan));
                const double midStep = midLength / midSegments;
                if (midStep >= minAllowableSpan)
                {
                    for (int i = 1; i < midSegments; ++i)
                    {
                        const double middle = left2 + i * midStep + asymmetryOffset;
                        if (right2 - middle < minAllowableSpan)
                            continue;
                        if (i == 1 && middle - left2 < minAllowableSpan)
                            continue;
                        positions.push_back(middle);
                    }
                }
            }
            positions.push_back(right2 + asymmetryOffset);
            positions.push_back(right1);
        }

        positions.erase(
            std::remove_if(
                positions.begin(),
                positions.end(),
                [spanLength](double position)
                {
                    return position < 0.0 || position > spanLength;
                }),
            positions.end());
        std::sort(positions.begin(), positions.end());
        positions.erase(
            std::unique(
                positions.begin(),
                positions.end(),
                [](double left, double right) { return std::abs(left - right) < 1e-6; }),
            positions.end());
        return positions;
    }

    bool ConductorModelBuilder::CalculateInnerSpacerConfigs(
        const LineBuildResult& line,
        const InnerSpacerLayoutConfig& layout,
        std::vector<InnerSpacerConfig>& configs,
        std::string& error) const
    {
        configs.clear();

        if (layout.useEqualSpacing && layout.count <= 0)
        {
            return true;
        }

        if (line.subConductors.size() <= 1)
        {
            error = "单分裂导线不需要自动布置相内间隔棒";
            return false;
        }

        if (line.spanLength <= 1e-7)
        {
            error = "导线档距无效，无法自动布置相内间隔棒";
            return false;
        }

        if (!layout.useEqualSpacing)
        {
            const std::vector<double> positions =
                CalculateStandardInnerSpacerPositions(line.spanLength);

            configs.reserve(positions.size());
            for (double position : positions)
            {
                if (position < 0.0 || position > line.spanLength)
                    continue;
                InnerSpacerConfig config = layout.spacer;
                config.position = position;
                config.useRatio = false;
                config.createCenterNode = line.subConductors.size() > 2;
                configs.push_back(config);
            }
            return true;
        }

        double start = layout.startOffset;
        double end = line.spanLength - layout.endOffset;
        if (start < 0.0)
        {
            start = 0.0;
        }
        if (end > line.spanLength)
        {
            end = line.spanLength;
        }

        if (end <= start)
        {
            error = "相内间隔棒自动布置有效区间无效";
            return false;
        }

        double spacing = (end - start) / static_cast<double>(layout.count + 1);
        configs.reserve(layout.count);
        for (int i = 1; i <= layout.count; ++i)
        {
            InnerSpacerConfig config = layout.spacer;
            config.position = start + spacing * i;
            config.useRatio = false;
            config.createCenterNode = line.subConductors.size() > 2;
            configs.push_back(config);
        }

        return true;
    }

    bool ConductorModelBuilder::AddNodes(
        BundleResult& raw,
        const LineBuildConfig& config,
        LineBuildResult& result,
        std::string& error)
    {
        std::vector<int> createdNodeIds;
        auto rollback = [&]()
            {
                for (int createdNodeId : createdNodeIds)
                {
                    m_structure->m_Nodes.erase(createdNodeId);
                }
                result = LineBuildResult{};
            };

        auto addNode = [&](const Vector3d& position, int& nodeId) -> bool
            {
                nodeId = NextNodeId();
                auto node = std::make_shared<Node>();
                node->m_Id = nodeId;
                node->m_X = position.x();
                node->m_Y = position.y();
                node->m_Z = position.z();
                if (!m_structure->m_Nodes.insert({ nodeId, node }).second)
                    return false;
                createdNodeIds.push_back(nodeId);
                return true;
            };

        const bool convergeEnds = config.convergeBundleEnds && config.conductor.nBundle > 1;
        std::map<int, int> leftGroupByWire;
        std::map<int, int> rightGroupByWire;
        if (convergeEnds)
        {
            if (!addNode(config.start, result.leftSupportNodeId) ||
                !addNode(config.end, result.rightSupportNodeId))
            {
                error = "添加耐张导线挂点失败";
                rollback();
                return false;
            }
            result.leftTensionEnd.supportNodeId = result.leftSupportNodeId;
            result.rightTensionEnd.supportNodeId = result.rightSupportNodeId;

            const int wireCount = static_cast<int>(raw.wiresNode.size());
            const int firstGroupCount = wireCount == 2 ? 2 : wireCount / 2;
            const int groupCount = wireCount == 2 ? 1 : 2;
            std::vector<Vector3d> leftSums(groupCount, Vector3d::Zero());
            std::vector<Vector3d> rightSums(groupCount, Vector3d::Zero());
            std::vector<int> groupSizes(groupCount, 0);

            for (const auto& pair : raw.wiresNode)
            {
                if (pair.second.size() < 2)
                {
                    error = "耐张端部至少需要两个子导线节点";
                    rollback();
                    return false;
                }
                const int group = (groupCount == 1 || pair.first < firstGroupCount) ? 0 : 1;
                const auto& nodes = pair.second;
                leftSums[group] += Vector3d(
                    (nodes[0].x + nodes[1].x) * 0.5,
                    (nodes[0].y + nodes[1].y) * 0.5,
                    (nodes[0].z + nodes[1].z) * 0.5);
                const std::size_t last = nodes.size() - 1;
                rightSums[group] += Vector3d(
                    (nodes[last - 1].x + nodes[last].x) * 0.5,
                    (nodes[last - 1].y + nodes[last].y) * 0.5,
                    (nodes[last - 1].z + nodes[last].z) * 0.5);
                ++groupSizes[group];
            }

            for (int group = 0; group < groupCount; ++group)
            {
                int leftGroupId = -1;
                int rightGroupId = -1;
                if (groupSizes[group] <= 0 ||
                    !addNode(leftSums[group] / groupSizes[group], leftGroupId) ||
                    !addNode(rightSums[group] / groupSizes[group], rightGroupId))
                {
                    error = "添加耐张端部分组节点失败";
                    rollback();
                    return false;
                }
                result.leftTensionEnd.groupNodeIds.push_back(leftGroupId);
                result.rightTensionEnd.groupNodeIds.push_back(rightGroupId);
            }

            for (const auto& pair : raw.wiresNode)
            {
                const int group = (groupCount == 1 || pair.first < firstGroupCount) ? 0 : 1;
                leftGroupByWire[pair.first] = result.leftTensionEnd.groupNodeIds[group];
                rightGroupByWire[pair.first] = result.rightTensionEnd.groupNodeIds[group];
            }
        }

        for (auto& pair : raw.wiresNode)
        {
            int wireId = pair.first;
            auto& rawNodes = pair.second;

            if (rawNodes.size() < 2)
            {
                error = "子导线节点列表不足";
                rollback();
                return false;
            }

            SubConductorModel sub;
            sub.wireId = wireId;
            sub.nodeIds.reserve(rawNodes.size());

            std::size_t firstCreatedIndex = 0;
            std::size_t endCreatedIndex = rawNodes.size();
            if (convergeEnds)
            {
                rawNodes.front().id = leftGroupByWire.at(wireId);
                rawNodes.back().id = rightGroupByWire.at(wireId);
                sub.nodeIds.push_back(rawNodes.front().id);
                firstCreatedIndex = 1;
                endCreatedIndex = rawNodes.size() - 1;
            }

            for (std::size_t index = firstCreatedIndex; index < endCreatedIndex; ++index)
            {
                auto& rawNode = rawNodes[index];
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
                    rollback();
                    return false;
                }

                createdNodeIds.push_back(nodeId);
                rawNode.id = nodeId;
                sub.nodeIds.push_back(nodeId);
            }

            if (convergeEnds)
            {
                sub.nodeIds.push_back(rawNodes.back().id);
            }

            result.subConductors.insert({ wireId, sub });
        }

        return true;
    }

    bool ConductorModelBuilder::AddElements(const BundleResult& raw, const LineBuildConfig& config, std::shared_ptr<Property> property, LineBuildResult& result, std::string& error)
    {
        std::vector<int> createdElementIds;
        auto rollback = [&]()
            {
                for (int createdElementId : createdElementIds)
                {
                    m_structure->m_Elements.erase(createdElementId);
                }
                for (auto& pair : result.subConductors)
                {
                    pair.second.elementIds.clear();
                }
            };

        for (const auto& pair : raw.wiresElement)
        {
            int wireId = pair.first;
            const auto& rawElements = pair.second;
            auto rawNodeIt = raw.wiresNode.find(wireId);
            auto subIt = result.subConductors.find(wireId);

            if (rawNodeIt == raw.wiresNode.end() || subIt == result.subConductors.end())
            {
                error = "导线单元找不到对应的子导线节点";
                rollback();
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
                    rollback();
                    return false;
                }

                int elementId = -1;
                if (!AddElement(rawNodes[iIndex].id, rawNodes[jIndex].id, config.elementType, property,
                    rawElement.stress0, elementId, error, ElementRole::Conductor, wireId, wireId))
                {
                    error = "添加导线单元失败：" + error;
                    rollback();
                    return false;
                }

                createdElementIds.push_back(elementId);
                sub.elementIds.push_back(elementId);
            }
        }

        return true;
    }

    bool ConductorModelBuilder::AddTensionEndElements(
        const LineBuildConfig& config,
        LineBuildResult& result,
        std::string& error)
    {
        if (!config.convergeBundleEnds || config.conductor.nBundle <= 1)
            return true;

        if (config.endFittingElementType == EnumKeyword::ElementType::UNKNOWN)
        {
            error = "耐张端部单元类型不能为 UNKNOWN";
            return false;
        }

        auto property = config.endFittingProperty ? config.endFittingProperty : config.property;
        if (!ValidateProperty(property, "耐张端部", error))
            return false;

        std::vector<int> createdElementIds;
        auto rollback = [&]()
            {
                for (int elementId : createdElementIds)
                    m_structure->m_Elements.erase(elementId);
                result.leftTensionEnd.yokeElementIds.clear();
                result.rightTensionEnd.yokeElementIds.clear();
                result.leftTensionEnd.stabilizerElementId = -1;
                result.rightTensionEnd.stabilizerElementId = -1;
            };

        auto buildSide = [&](TensionEndModel& endModel) -> bool
            {
                for (int groupNodeId : endModel.groupNodeIds)
                {
                    int elementId = -1;
                    if (!AddElement(
                            endModel.supportNodeId,
                            groupNodeId,
                            config.endFittingElementType,
                            property,
                            0.0,
                            elementId,
                            error,
                            ElementRole::TensionHardware))
                        return false;
                    createdElementIds.push_back(elementId);
                    endModel.yokeElementIds.push_back(elementId);
                }

                if (endModel.groupNodeIds.size() == 2)
                {
                    int elementId = -1;
                    if (!AddElement(
                            endModel.groupNodeIds[0],
                            endModel.groupNodeIds[1],
                            config.endFittingElementType,
                            property,
                            0.0,
                            elementId,
                            error,
                            ElementRole::TensionHardware))
                        return false;
                    createdElementIds.push_back(elementId);
                    endModel.stabilizerElementId = elementId;
                }
                return true;
            };

        if (!buildSide(result.leftTensionEnd) || !buildSide(result.rightTensionEnd))
        {
            error = "添加 THOP 式耐张端部失败：" + error;
            rollback();
            return false;
        }
        return true;
    }

    bool ConductorModelBuilder::CreateSubConductorSets(
        const LineBuildConfig& config,
        LineBuildResult& result,
        std::string& error)
    {
        std::vector<int> createdSetIds;
        auto rollback = [&]()
            {
                for (int setId : createdSetIds)
                {
                    m_structure->m_ModelSets.erase(setId);
                }
                for (auto& pair : result.subConductors)
                {
                    pair.second.nodeSetId = -1;
                    pair.second.elementSetId = -1;
                }
            };

        QString prefix = config.setNamePrefix.trimmed();
        if (prefix.isEmpty())
        {
            prefix = QStringLiteral("单档导线");
        }

        for (auto& pair : result.subConductors)
        {
            auto& sub = pair.second;
            const QString wireName = QStringLiteral("%1-分裂线%2")
                .arg(prefix)
                .arg(sub.wireId + 1);

            const std::set<int> nodeIds(sub.nodeIds.begin(), sub.nodeIds.end());
            const std::set<int> elementIds(sub.elementIds.begin(), sub.elementIds.end());
            QString setError;

            sub.nodeSetId = m_structure->AddModelSet(
                wireName + QStringLiteral("-节点集"),
                ModelSetType::Node,
                nodeIds,
                &setError);
            if (sub.nodeSetId <= 0)
            {
                error = QStringLiteral("创建子导线节点集合失败：%1").arg(setError).toStdString();
                rollback();
                return false;
            }
            createdSetIds.push_back(sub.nodeSetId);

            sub.elementSetId = m_structure->AddModelSet(
                wireName + QStringLiteral("-气动单元集"),
                ModelSetType::Element,
                elementIds,
                &setError);
            if (sub.elementSetId <= 0)
            {
                error = QStringLiteral("创建子导线单元集合失败：%1").arg(setError).toStdString();
                rollback();
                return false;
            }
            createdSetIds.push_back(sub.elementSetId);
        }

        return true;
    }

    bool ConductorModelBuilder::RenumberLineModel(LineBuildResult& result, std::string& error)
    {
        if (!m_structure)
        {
            error = "StructureData 为空，无法重新编号导线模型";
            return false;
        }

        auto appendUnique = [](std::vector<int>& order, std::set<int>& seen, int id)
            {
                if (id > 0 && seen.insert(id).second)
                    order.push_back(id);
            };

        std::set<int> generatedNodeIds;
        std::set<int> generatedElementIds;
        for (const auto& [wireId, sub] : result.subConductors)
        {
            generatedNodeIds.insert(sub.nodeIds.begin(), sub.nodeIds.end());
            generatedElementIds.insert(sub.elementIds.begin(), sub.elementIds.end());
        }
        if (result.leftSupportNodeId > 0)
            generatedNodeIds.insert(result.leftSupportNodeId);
        if (result.rightSupportNodeId > 0)
            generatedNodeIds.insert(result.rightSupportNodeId);
        generatedNodeIds.insert(
            result.leftTensionEnd.groupNodeIds.begin(), result.leftTensionEnd.groupNodeIds.end());
        generatedNodeIds.insert(
            result.rightTensionEnd.groupNodeIds.begin(), result.rightTensionEnd.groupNodeIds.end());
        generatedElementIds.insert(
            result.leftTensionEnd.yokeElementIds.begin(), result.leftTensionEnd.yokeElementIds.end());
        generatedElementIds.insert(
            result.rightTensionEnd.yokeElementIds.begin(), result.rightTensionEnd.yokeElementIds.end());
        if (result.leftTensionEnd.stabilizerElementId > 0)
            generatedElementIds.insert(result.leftTensionEnd.stabilizerElementId);
        if (result.rightTensionEnd.stabilizerElementId > 0)
            generatedElementIds.insert(result.rightTensionEnd.stabilizerElementId);
        for (const auto& spacer : result.innerSpacers)
        {
            generatedNodeIds.insert(spacer.nodeIds.begin(), spacer.nodeIds.end());
            generatedElementIds.insert(spacer.elementIds.begin(), spacer.elementIds.end());
        }

        std::vector<int> nodeOrder;
        std::set<int> seenNodes;
        const std::size_t groupCount = std::min(
            result.leftTensionEnd.groupNodeIds.size(),
            result.rightTensionEnd.groupNodeIds.size());
        if (groupCount > 0)
        {
            for (std::size_t groupIndex = 0; groupIndex < groupCount; ++groupIndex)
            {
                const int leftGroupId = result.leftTensionEnd.groupNodeIds[groupIndex];
                const int rightGroupId = result.rightTensionEnd.groupNodeIds[groupIndex];
                appendUnique(nodeOrder, seenNodes, leftGroupId);
                for (const auto& [wireId, sub] : result.subConductors)
                {
                    if (sub.nodeIds.size() < 2 || sub.nodeIds.front() != leftGroupId)
                        continue;
                    for (std::size_t index = 1; index + 1 < sub.nodeIds.size(); ++index)
                        appendUnique(nodeOrder, seenNodes, sub.nodeIds[index]);
                }
                appendUnique(nodeOrder, seenNodes, rightGroupId);
            }
        }
        else
        {
            for (const auto& [wireId, sub] : result.subConductors)
            {
                for (int nodeId : sub.nodeIds)
                    appendUnique(nodeOrder, seenNodes, nodeId);
            }
        }

        appendUnique(nodeOrder, seenNodes, result.leftSupportNodeId);
        appendUnique(nodeOrder, seenNodes, result.rightSupportNodeId);
        for (const auto& spacer : result.innerSpacers)
        {
            appendUnique(nodeOrder, seenNodes, spacer.centerNodeId);
        }
        for (int nodeId : generatedNodeIds)
            appendUnique(nodeOrder, seenNodes, nodeId);

        std::vector<int> elementOrder;
        std::set<int> seenElements;
        for (const auto& [wireId, sub] : result.subConductors)
        {
            for (int elementId : sub.elementIds)
                appendUnique(elementOrder, seenElements, elementId);
        }
        for (int elementId : result.leftTensionEnd.yokeElementIds)
            appendUnique(elementOrder, seenElements, elementId);
        appendUnique(elementOrder, seenElements, result.leftTensionEnd.stabilizerElementId);
        for (int elementId : result.rightTensionEnd.yokeElementIds)
            appendUnique(elementOrder, seenElements, elementId);
        appendUnique(elementOrder, seenElements, result.rightTensionEnd.stabilizerElementId);
        for (const auto& spacer : result.innerSpacers)
        {
            for (int elementId : spacer.elementIds)
                appendUnique(elementOrder, seenElements, elementId);
        }
        for (int elementId : generatedElementIds)
            appendUnique(elementOrder, seenElements, elementId);

        if (nodeOrder.size() != generatedNodeIds.size() ||
            elementOrder.size() != generatedElementIds.size())
        {
            error = "导线模型重新编号前的节点或单元索引不完整";
            return false;
        }

        int maxUnrelatedNodeId = 0;
        for (const auto& [nodeId, node] : m_structure->m_Nodes)
        {
            if (generatedNodeIds.find(nodeId) == generatedNodeIds.end())
                maxUnrelatedNodeId = std::max(maxUnrelatedNodeId, nodeId);
        }
        int maxUnrelatedElementId = 0;
        for (const auto& [elementId, element] : m_structure->m_Elements)
        {
            if (generatedElementIds.find(elementId) == generatedElementIds.end())
                maxUnrelatedElementId = std::max(maxUnrelatedElementId, elementId);
        }

        std::map<int, int> nodeIdMap;
        std::map<int, int> elementIdMap;
        for (std::size_t index = 0; index < nodeOrder.size(); ++index)
            nodeIdMap[nodeOrder[index]] = maxUnrelatedNodeId + static_cast<int>(index) + 1;
        for (std::size_t index = 0; index < elementOrder.size(); ++index)
            elementIdMap[elementOrder[index]] = maxUnrelatedElementId + static_cast<int>(index) + 1;

        std::map<int, std::shared_ptr<Node>> renumberedNodes;
        for (const auto& [oldId, node] : m_structure->m_Nodes)
        {
            const auto mapped = nodeIdMap.find(oldId);
            const int newId = mapped == nodeIdMap.end() ? oldId : mapped->second;
            if (!node || !renumberedNodes.emplace(newId, node).second)
            {
                error = "导线节点重新编号发生 ID 冲突";
                return false;
            }
        }
        std::map<int, std::shared_ptr<ElementBase>> renumberedElements;
        for (const auto& [oldId, element] : m_structure->m_Elements)
        {
            const auto mapped = elementIdMap.find(oldId);
            const int newId = mapped == elementIdMap.end() ? oldId : mapped->second;
            if (!element || !renumberedElements.emplace(newId, element).second)
            {
                error = "导线单元重新编号发生 ID 冲突";
                return false;
            }
        }

        for (auto& [newId, node] : renumberedNodes)
            node->m_Id = newId;
        for (auto& [newId, element] : renumberedElements)
            element->m_Id = newId;
        m_structure->m_Nodes = std::move(renumberedNodes);
        m_structure->m_Elements = std::move(renumberedElements);

        auto remapId = [](int& id, const std::map<int, int>& idMap)
            {
                const auto found = idMap.find(id);
                if (found != idMap.end())
                    id = found->second;
            };
        auto remapVector = [&](std::vector<int>& ids, const std::map<int, int>& idMap)
            {
                for (int& id : ids)
                    remapId(id, idMap);
            };
        auto remapSet = [](std::set<int>& ids, const std::map<int, int>& idMap)
            {
                std::set<int> remapped;
                for (int id : ids)
                {
                    const auto found = idMap.find(id);
                    remapped.insert(found == idMap.end() ? id : found->second);
                }
                ids = std::move(remapped);
            };

        remapId(result.leftSupportNodeId, nodeIdMap);
        remapId(result.rightSupportNodeId, nodeIdMap);
        remapId(result.leftTensionEnd.supportNodeId, nodeIdMap);
        remapId(result.rightTensionEnd.supportNodeId, nodeIdMap);
        remapVector(result.leftTensionEnd.groupNodeIds, nodeIdMap);
        remapVector(result.rightTensionEnd.groupNodeIds, nodeIdMap);
        remapVector(result.leftTensionEnd.yokeElementIds, elementIdMap);
        remapVector(result.rightTensionEnd.yokeElementIds, elementIdMap);
        remapId(result.leftTensionEnd.stabilizerElementId, elementIdMap);
        remapId(result.rightTensionEnd.stabilizerElementId, elementIdMap);
        for (auto& [wireId, sub] : result.subConductors)
        {
            remapVector(sub.nodeIds, nodeIdMap);
            remapVector(sub.elementIds, elementIdMap);
        }
        for (auto& spacer : result.innerSpacers)
        {
            remapId(spacer.id, elementIdMap);
            remapId(spacer.centerNodeId, nodeIdMap);
            remapVector(spacer.nodeIds, nodeIdMap);
            remapVector(spacer.elementIds, elementIdMap);
        }

        for (auto& [setId, modelSet] : m_structure->m_ModelSets)
        {
            if (!modelSet)
                continue;
            if (modelSet->m_Type == ModelSetType::Node)
                remapSet(modelSet->m_Ids, nodeIdMap);
            else
                remapSet(modelSet->m_Ids, elementIdMap);
        }
        for (auto& [regionId, region] : m_structure->m_ComputeRegions)
        {
            if (!region)
                continue;
            remapSet(region->m_DirectNodeIds, nodeIdMap);
            remapSet(region->m_NodeIds, nodeIdMap);
            remapSet(region->m_DirectElementIds, elementIdMap);
            remapSet(region->m_ElementIds, elementIdMap);
        }

        return true;
    }
}
