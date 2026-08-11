#include "ConductorModelBuilder.h"

#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Element/ElementBase.h"
#include "DataStructure/Element/ElementTruss.h"
#include "DataStructure/Constraint/NonlinearMPCConstraint.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Node/Node.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionBase.h"
#include "DataStructure/Structure/StructureData.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
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
        nodeIds.insert(leftTensionEnd.supportNodeIds.begin(), leftTensionEnd.supportNodeIds.end());
        nodeIds.insert(rightTensionEnd.supportNodeIds.begin(), rightTensionEnd.supportNodeIds.end());
        nodeIds.insert(leftTensionEnd.groupNodeIds.begin(), leftTensionEnd.groupNodeIds.end());
        nodeIds.insert(rightTensionEnd.groupNodeIds.begin(), rightTensionEnd.groupNodeIds.end());
        for (const auto& suspension : suspensionPoints)
        {
            if (suspension.junctionNodeId > 0)
                nodeIds.insert(suspension.junctionNodeId);
            if (suspension.supportNodeId > 0)
                nodeIds.insert(suspension.supportNodeId);
            nodeIds.insert(suspension.wireNodeIds.begin(), suspension.wireNodeIds.end());
            if (suspension.spacerCenterNodeId > 0)
                nodeIds.insert(suspension.spacerCenterNodeId);
            nodeIds.insert(suspension.spacerInnerNodeIds.begin(), suspension.spacerInnerNodeIds.end());
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
        elementIds.insert(leftTensionEnd.yokeElementIds.begin(), leftTensionEnd.yokeElementIds.end());
        elementIds.insert(rightTensionEnd.yokeElementIds.begin(), rightTensionEnd.yokeElementIds.end());
        if (leftTensionEnd.stabilizerElementId > 0)
            elementIds.insert(leftTensionEnd.stabilizerElementId);
        if (rightTensionEnd.stabilizerElementId > 0)
            elementIds.insert(rightTensionEnd.stabilizerElementId);
        for (const auto& suspension : suspensionPoints)
        {
            elementIds.insert(suspension.yokeElementIds.begin(), suspension.yokeElementIds.end());
            if (suspension.stringElementId > 0)
                elementIds.insert(suspension.stringElementId);
            elementIds.insert(suspension.spacerElementIds.begin(), suspension.spacerElementIds.end());
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

    bool ConductorModelBuilder::BuildLine(const LineBuildConfig& config, _OUT LineBuildResult& result, _OUT std::string& error)
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

        if (config.conductor.segments <= 0)
        {
            error = "导线离散段数必须大于 0";
            return false;
        }

        if (config.conductor.initialStress <= 0.0)
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
        result.end   = config.end;
        result.spanLength = horizontalSpan.norm();

        const double lineWeight = material->m_Density * section->m_Area * 9.81;
        const double horizontalTension = config.conductor.initialStress * section->m_Area;

        double leftCutLength    = config.leftCutLength;
        double rightCutLength   = config.rightCutLength;
        const bool convergeEnds = config.convergeBundleEnds && config.conductor.nBundle > 1;
        // 单分裂同样保留默认的端部过渡长度，避免“精确挂点”与悬链线
        // 第一个内点仅相距 CatenaryModel 的数值下限 1e-4 m，形成极短单元。
        const bool useDefaultEndTransition = config.conductor.nBundle == 1 || convergeEnds;
        if (useDefaultEndTransition && config.bundleEndTransitionLength > 0.0)
        {
            const double transitionLength = std::min(config.bundleEndTransitionLength, result.spanLength * 0.1);
            if (leftCutLength <= 1e-7)
                leftCutLength = transitionLength;
            if (rightCutLength <= 1e-7)
                rightCutLength = transitionLength;
        }

        BundleResult raw = Generator::CreateBundle(
            config.start.data(), config.end.data(),
            leftCutLength, rightCutLength,
            horizontalTension, lineWeight, config.conductor);

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
                nodeIds.insert(result.leftTensionEnd.supportNodeIds.begin(), result.leftTensionEnd.supportNodeIds.end());
                nodeIds.insert(result.rightTensionEnd.supportNodeIds.begin(), result.rightTensionEnd.supportNodeIds.end());
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

    bool ConductorModelBuilder::BuildSpanConductor(const SpanConductorBuildConfig& config, _OUT LineBuildResult& result, _OUT std::string& error)
    {
        if (!BuildLine(config.line, result, error))
        {
            return false;
        }

        auto rollbackLine = [&]()
            {
                for (const auto& spacer : result.innerSpacers)
                {
                    for (int mpcId : spacer.mpcIds)
                    {
                        m_structure->m_MPCConstraints.erase(mpcId);
                    }
                    for (int elementId : spacer.elementIds)
                    {
                        m_structure->m_Elements.erase(elementId);
                    }
                    if (spacer.centerNodeId != -1)
                    {
                        m_structure->m_Nodes.erase(spacer.centerNodeId);
                    }
                    for (int nodeId : spacer.innerNodeIds)
                    {
                        m_structure->m_Nodes.erase(nodeId);
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
                for (int nodeId : result.leftTensionEnd.supportNodeIds)
                    m_structure->m_Nodes.erase(nodeId);
                for (int nodeId : result.rightTensionEnd.supportNodeIds)
                    m_structure->m_Nodes.erase(nodeId);
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

    bool ConductorModelBuilder::BuildMultiSpanConductor(
        const MultiSpanConductorBuildConfig& config,
        _OUT LineBuildResult& result,
        _OUT std::string& error)
    {
        result = LineBuildResult{};
        if (!m_structure)
        {
            error = "StructureData 为空，无法生成多档导线";
            return false;
        }
        if (config.stationCenters.size() < 3)
        {
            error = "多档导线至少需要三个导线束中心坐标";
            return false;
        }

        const LineBuildConfig& lineConfig = config.span.line;
        if (lineConfig.elementType == EnumKeyword::ElementType::UNKNOWN ||
            config.suspensionElementType == EnumKeyword::ElementType::UNKNOWN)
        {
            error = "导线或悬垂串单元类型不能为 UNKNOWN";
            return false;
        }
        if (!ValidateProperty(lineConfig.property, "导线", error))
            return false;
        auto material = lineConfig.property->m_pMaterial.lock();
        auto section = lineConfig.property->m_pSection.lock();
        if (!material || !section || material->m_Density <= 0.0 || section->m_Area <= 0.0)
        {
            error = "导线材料密度和截面面积必须大于 0";
            return false;
        }
        if (lineConfig.conductor.nBundle != 1 &&
            lineConfig.conductor.nBundle != 2 &&
            lineConfig.conductor.nBundle != 4 &&
            lineConfig.conductor.nBundle != 6 &&
            lineConfig.conductor.nBundle != 8)
        {
            error = "导线分裂数仅支持 1、2、4、6、8";
            return false;
        }
        if (lineConfig.conductor.nBundle > 1 && !lineConfig.convergeBundleEnds)
        {
            error = "多档多分裂导线必须启用首末耐张端汇集";
            return false;
        }
        if (lineConfig.endFittingElementType == EnumKeyword::ElementType::UNKNOWN)
        {
            error = "耐张稳定梁单元类型不能为 UNKNOWN";
            return false;
        }
        if (lineConfig.conductor.segments <= 0 ||
            lineConfig.conductor.initialStress <= 0.0 ||
            lineConfig.conductor.spacing < 0.0)
        {
            error = "多档导线的连接方式、离散段数、初始应力或分裂间距无效";
            return false;
        }
        if (config.suspensionStringLength <= 0.0)
        {
            error = "悬垂串长度必须大于 0";
            return false;
        }

        auto suspensionProperty = config.suspensionProperty
            ? config.suspensionProperty : lineConfig.property;
        if (!ValidateProperty(suspensionProperty, "悬垂串", error))
            return false;
        auto fittingProperty = lineConfig.endFittingProperty
            ? lineConfig.endFittingProperty : lineConfig.property;
        if (!ValidateProperty(fittingProperty, "耐张稳定梁", error))
            return false;

        std::set<int> originalNodeIds;
        std::set<int> originalElementIds;
        std::set<int> originalSetIds;
        std::set<int> originalMpcIds;
        for (const auto& [id, node] : m_structure->m_Nodes)
            originalNodeIds.insert(id);
        for (const auto& [id, element] : m_structure->m_Elements)
            originalElementIds.insert(id);
        for (const auto& [id, modelSet] : m_structure->m_ModelSets)
            originalSetIds.insert(id);
        for (const auto& [id, mpc] : m_structure->m_MPCConstraints)
            originalMpcIds.insert(id);
        auto rollback = [&]()
            {
                for (auto it = m_structure->m_MPCConstraints.begin();
                     it != m_structure->m_MPCConstraints.end();)
                {
                    if (originalMpcIds.find(it->first) == originalMpcIds.end())
                        it = m_structure->m_MPCConstraints.erase(it);
                    else
                        ++it;
                }
                for (auto it = m_structure->m_Elements.begin(); it != m_structure->m_Elements.end();)
                {
                    if (originalElementIds.find(it->first) == originalElementIds.end())
                        it = m_structure->m_Elements.erase(it);
                    else
                        ++it;
                }
                for (auto it = m_structure->m_Nodes.begin(); it != m_structure->m_Nodes.end();)
                {
                    if (originalNodeIds.find(it->first) == originalNodeIds.end())
                        it = m_structure->m_Nodes.erase(it);
                    else
                        ++it;
                }
                for (auto it = m_structure->m_ModelSets.begin(); it != m_structure->m_ModelSets.end();)
                {
                    if (originalSetIds.find(it->first) == originalSetIds.end())
                        it = m_structure->m_ModelSets.erase(it);
                    else
                        ++it;
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
                return m_structure->m_Nodes.insert({ nodeId, node }).second;
            };

        const std::size_t spanCount = config.stationCenters.size() - 1;
        const int wireCount = lineConfig.conductor.nBundle;
        result.start = config.stationCenters.front();
        result.end = config.stationCenters.back();
        result.spanCount = static_cast<int>(spanCount);
        result.spanLength = 0.0;
        result.property = lineConfig.property;

        const double lineWeight = material->m_Density * section->m_Area * 9.81;
        const double horizontalTension = lineConfig.conductor.initialStress * section->m_Area;
        std::vector<BundleResult> rawSpans;
        rawSpans.reserve(spanCount);
        for (std::size_t spanIndex = 0; spanIndex < spanCount; ++spanIndex)
        {
            const Vector3d& start = config.stationCenters[spanIndex];
            const Vector3d& end = config.stationCenters[spanIndex + 1];
            const double horizontalLength = (end - start).head<2>().norm();
            if (horizontalLength <= 1.0e-7)
            {
                error = "第 " + std::to_string(spanIndex + 1) + " 档水平档距过小";
                rollback();
                return false;
            }
            result.spanLength += horizontalLength;
            const double transitionLength = std::min(
                std::max(0.0, lineConfig.bundleEndTransitionLength),
                horizontalLength * 0.1);
            BundleResult raw = Generator::CreateBundle(
                start.data(), end.data(), transitionLength, transitionLength,
                horizontalTension, lineWeight, lineConfig.conductor);
            if (raw.wiresNode.size() != static_cast<std::size_t>(wireCount))
            {
                error = "第 " + std::to_string(spanIndex + 1) + " 档导线几何生成失败";
                rollback();
                return false;
            }
            rawSpans.push_back(std::move(raw));
        }

        // 端站子导线节点需要在首末档建模时复用；中间站节点随后写入同一张表。
        std::vector<std::map<int, int>> stationWireNodeIds(config.stationCenters.size());
        const bool directWireSupports = wireCount > 1 &&
            lineConfig.endTopology == BundleEndTopology::DirectWireSupports;
        auto createTensionEnds = [&]() -> bool
            {
                if (wireCount == 1)
                    return true;

                if (directWireSupports)
                {
                    // 简化端部：端站截面中的每个子导线端点直接作为挂点，
                    // 不创建汇集节点、分组节点及耐张端部硬件。
                    for (int wireId = 0; wireId < wireCount; ++wireId)
                    {
                        const auto& leftNodes = rawSpans.front().wiresNode.at(wireId);
                        const auto& rightNodes = rawSpans.back().wiresNode.at(wireId);
                        if (leftNodes.empty() || rightNodes.empty())
                            return false;
                        int leftId = -1;
                        int rightId = -1;
                        if (!addNode(Vector3d(leftNodes.front().x, leftNodes.front().y, leftNodes.front().z), leftId) ||
                            !addNode(Vector3d(rightNodes.back().x, rightNodes.back().y, rightNodes.back().z), rightId))
                            return false;
                        stationWireNodeIds.front()[wireId] = leftId;
                        stationWireNodeIds.back()[wireId] = rightId;
                        result.leftTensionEnd.supportNodeIds.push_back(leftId);
                        result.rightTensionEnd.supportNodeIds.push_back(rightId);
                    }
                    result.leftSupportNodeId = result.leftTensionEnd.supportNodeIds.front();
                    result.rightSupportNodeId = result.rightTensionEnd.supportNodeIds.front();
                    result.leftTensionEnd.supportNodeId = result.leftSupportNodeId;
                    result.rightTensionEnd.supportNodeId = result.rightSupportNodeId;
                    return true;
                }

                const int firstGroupCount = wireCount == 2 ? 2 : wireCount / 2;
                const int groupCount = wireCount == 2 ? 1 : 2;
                std::vector<Vector3d> leftSums(groupCount, Vector3d::Zero());
                std::vector<Vector3d> rightSums(groupCount, Vector3d::Zero());
                std::vector<int> groupSizes(groupCount, 0);
                for (int wireId = 0; wireId < wireCount; ++wireId)
                {
                    const int group = groupCount == 1 || wireId < firstGroupCount ? 0 : 1;
                    const auto& leftNodes = rawSpans.front().wiresNode.at(wireId);
                    const auto& rightNodes = rawSpans.back().wiresNode.at(wireId);
                    if (leftNodes.size() < 2 || rightNodes.size() < 2)
                        return false;
                    leftSums[group] += Vector3d(
                        (leftNodes[0].x + leftNodes[1].x) * 0.5,
                        (leftNodes[0].y + leftNodes[1].y) * 0.5,
                        (leftNodes[0].z + leftNodes[1].z) * 0.5);
                    const std::size_t last = rightNodes.size() - 1;
                    rightSums[group] += Vector3d(
                        (rightNodes[last - 1].x + rightNodes[last].x) * 0.5,
                        (rightNodes[last - 1].y + rightNodes[last].y) * 0.5,
                        (rightNodes[last - 1].z + rightNodes[last].z) * 0.5);
                    ++groupSizes[group];
                }

                std::vector<Vector3d> leftCenters(groupCount);
                std::vector<Vector3d> rightCenters(groupCount);
                for (int group = 0; group < groupCount; ++group)
                {
                    if (groupSizes[group] <= 0)
                        return false;
                    leftCenters[group] = leftSums[group] / groupSizes[group];
                    rightCenters[group] = rightSums[group] / groupSizes[group];
                    int leftGroupId = -1;
                    int rightGroupId = -1;
                    if (!addNode(leftCenters[group], leftGroupId) ||
                        !addNode(rightCenters[group], rightGroupId))
                        return false;
                    result.leftTensionEnd.groupNodeIds.push_back(leftGroupId);
                    result.rightTensionEnd.groupNodeIds.push_back(rightGroupId);
                }

                const bool dualSupport = lineConfig.endTopology == BundleEndTopology::DualSupportByGroup;
                if (!dualSupport)
                {
                    if (!addNode(result.start, result.leftSupportNodeId) ||
                        !addNode(result.end, result.rightSupportNodeId))
                        return false;
                    result.leftTensionEnd.supportNodeId = result.leftSupportNodeId;
                    result.rightTensionEnd.supportNodeId = result.rightSupportNodeId;
                    result.leftTensionEnd.supportNodeIds = { result.leftSupportNodeId };
                    result.rightTensionEnd.supportNodeIds = { result.rightSupportNodeId };
                    return true;
                }

                if (groupCount != 2)
                    return false;
                const double spacing = lineConfig.dualSupportSpacing > 1.0e-7
                    ? lineConfig.dualSupportSpacing : lineConfig.conductor.spacing;
                auto addDual = [&](const Vector3d& base,
                                   const std::vector<Vector3d>& centers,
                                   TensionEndModel& endModel,
                                   int& primaryId) -> bool
                    {
                        Vector3d direction = centers[0] - centers[1];
                        if (direction.norm() <= 1.0e-7 || spacing <= 1.0e-7)
                            return false;
                        direction.normalize();
                        int firstId = -1;
                        int secondId = -1;
                        if (!addNode(base + direction * spacing * 0.5, firstId) ||
                            !addNode(base - direction * spacing * 0.5, secondId))
                            return false;
                        primaryId = firstId;
                        endModel.supportNodeId = firstId;
                        endModel.supportNodeIds = { firstId, secondId };
                        return true;
                    };
                return addDual(result.start, leftCenters, result.leftTensionEnd, result.leftSupportNodeId) &&
                    addDual(result.end, rightCenters, result.rightTensionEnd, result.rightSupportNodeId);
            };
        if (!createTensionEnds())
        {
            error = "创建多档导线耐张端失败";
            rollback();
            return false;
        }

        if (wireCount == 1)
        {
            if (!addNode(result.start, result.leftSupportNodeId) ||
                !addNode(result.end, result.rightSupportNodeId))
            {
                error = "创建单分裂耐张端节点失败";
                rollback();
                return false;
            }
            result.leftTensionEnd.supportNodeId = result.leftSupportNodeId;
            result.rightTensionEnd.supportNodeId = result.rightSupportNodeId;
            result.leftTensionEnd.supportNodeIds = { result.leftSupportNodeId };
            result.rightTensionEnd.supportNodeIds = { result.rightSupportNodeId };
            stationWireNodeIds.front()[0] = result.leftSupportNodeId;
            stationWireNodeIds.back()[0] = result.rightSupportNodeId;
        }

        auto stationDirection = [&](std::size_t stationIndex) -> Vector2d
            {
                Vector2d direction = Vector2d::Zero();
                if (stationIndex > 0)
                {
                    Vector2d previous =
                        (config.stationCenters[stationIndex] - config.stationCenters[stationIndex - 1]).head<2>();
                    if (previous.norm() > 1.0e-7)
                        direction += previous.normalized();
                }
                if (stationIndex + 1 < config.stationCenters.size())
                {
                    Vector2d next =
                        (config.stationCenters[stationIndex + 1] - config.stationCenters[stationIndex]).head<2>();
                    if (next.norm() > 1.0e-7)
                        direction += next.normalized();
                }
                if (direction.norm() <= 1.0e-7 && stationIndex + 1 < config.stationCenters.size())
                    direction = (config.stationCenters[stationIndex + 1] -
                                 config.stationCenters[stationIndex]).head<2>();
                return direction.normalized();
            };
        auto bundleOffset = [&](int wireId, const Vector2d& direction) -> Vector3d
            {
                if (wireCount == 1)
                    return Vector3d::Zero();
                double startAngle = 0.0;
                double radius = 0.0;
                if (wireCount == 2)
                    radius = lineConfig.conductor.spacing * 0.5;
                else if (wireCount == 4)
                {
                    startAngle = -std::numbers::pi / 4.0;
                    radius = lineConfig.conductor.spacing /
                        (2.0 * std::sin(std::numbers::pi / 4.0));
                }
                else if (wireCount == 6)
                {
                    startAngle = -std::numbers::pi / 3.0;
                    radius = lineConfig.conductor.spacing /
                        (2.0 * std::sin(std::numbers::pi / 6.0));
                }
                else if (wireCount == 8)
                {
                    startAngle = -3.0 * std::numbers::pi / 8.0;
                    radius = lineConfig.conductor.spacing /
                        (2.0 * std::sin(std::numbers::pi / 8.0));
                }
                const double angle = startAngle + 2.0 * std::numbers::pi * wireId / wireCount;
                const Vector2d right(direction.y(), -direction.x());
                return Vector3d(
                    right.x() * std::cos(angle) * radius,
                    right.y() * std::cos(angle) * radius,
                    std::sin(angle) * radius);
            };

        for (std::size_t stationIndex = 1; stationIndex + 1 < config.stationCenters.size(); ++stationIndex)
        {
            SuspensionPointModel suspension;
            suspension.stationIndex = static_cast<int>(stationIndex);
            suspension.center = config.stationCenters[stationIndex];
            const Vector2d direction = stationDirection(stationIndex);
            for (int wireId = 0; wireId < wireCount; ++wireId)
            {
                int nodeId = -1;
                if (!addNode(suspension.center + bundleOffset(wireId, direction), nodeId))
                {
                    error = "创建中间悬垂点的子导线节点失败";
                    rollback();
                    return false;
                }
                stationWireNodeIds[stationIndex][wireId] = nodeId;
                suspension.wireNodeIds.push_back(nodeId);
            }

            double rise = 0.0;
            if (wireCount > 1)
            {
                rise = lineConfig.conductor.spacing * 0.5 *
                    (1.0 + 1.0 / std::tan(std::numbers::pi / wireCount));
            }
            const Vector3d junctionPosition = suspension.center + Vector3d(0.0, 0.0, rise);
            if (wireCount == 1)
                suspension.junctionNodeId = suspension.wireNodeIds.front();
            else if (!addNode(junctionPosition, suspension.junctionNodeId))
            {
                error = "创建中间悬垂汇集节点失败";
                rollback();
                return false;
            }
            if (!addNode(
                    junctionPosition + Vector3d(0.0, 0.0, config.suspensionStringLength),
                    suspension.supportNodeId))
            {
                error = "创建中间悬垂约束挂点失败";
                rollback();
                return false;
            }

            std::vector<int> topWireIds;
            if (wireCount == 2)
                topWireIds = { 0, 1 };
            else if (wireCount == 4)
                topWireIds = { 1, 2 };
            else if (wireCount == 6)
                topWireIds = { 2, 3 };
            else if (wireCount == 8)
                topWireIds = { 3, 4 };
            for (int wireId : topWireIds)
            {
                int elementId = -1;
                if (!AddElement(
                        suspension.wireNodeIds[wireId],
                        suspension.junctionNodeId,
                        // H 型悬垂顶部两根联板按杆单元处理，使 J-P 悬垂串和
                        // 上部挂点仅引入平动自由度，避免转角自由度导致奇异。
                        EnumKeyword::ElementType::T3D2,
                        lineConfig.property,
                            lineConfig.conductor.initialStress,
                        elementId,
                        error,
                        ElementRole::SuspensionHardware))
                {
                    error = "创建悬垂端联板失败：" + error;
                    rollback();
                    return false;
                }
                suspension.yokeElementIds.push_back(elementId);
            }
            if (!AddElement(
                    suspension.junctionNodeId,
                    suspension.supportNodeId,
                    config.suspensionElementType,
                    suspensionProperty,
                            lineConfig.conductor.initialStress,
                    suspension.stringElementId,
                    error,
                    ElementRole::SuspensionHardware))
            {
                error = "创建悬垂串失败：" + error;
                rollback();
                return false;
            }

            // 悬垂线夹处的分裂截面同样需要相内间隔棒保持几何连接。
            // 复用界面中选择的间隔棒单元、属性和拓扑形式，而不是使用导线单元。
            const InnerSpacerConfig* suspensionSpacerConfig = nullptr;
            if (!config.span.innerSpacers.empty())
                suspensionSpacerConfig = &config.span.innerSpacers.front();
            else if (config.span.useInnerSpacerLayout)
                suspensionSpacerConfig = &config.span.innerSpacerLayout.spacer;
            if (wireCount > 1 && suspensionSpacerConfig)
            {
                if (!BuildSuspensionSpacer(suspension, *suspensionSpacerConfig, error))
                {
                    error = "创建悬垂端相内间隔棒失败：" + error;
                    rollback();
                    return false;
                }
            }
            result.suspensionPoints.push_back(std::move(suspension));
        }

        std::vector<LineBuildResult> spanResults(spanCount);
        for (int wireId = 0; wireId < wireCount; ++wireId)
        {
            SubConductorModel globalSub;
            globalSub.wireId = wireId;
            result.subConductors[wireId] = std::move(globalSub);
        }

        const int firstGroupCount = wireCount == 2 ? 2 : wireCount / 2;
        const int groupCount = wireCount == 2 ? 1 : 2;
        auto groupForWire = [&](int wireId) -> int
            {
                return groupCount == 1 || wireId < firstGroupCount ? 0 : 1;
            };

        for (std::size_t spanIndex = 0; spanIndex < spanCount; ++spanIndex)
        {
            auto& spanResult = spanResults[spanIndex];
            spanResult.start = config.stationCenters[spanIndex];
            spanResult.end = config.stationCenters[spanIndex + 1];
            spanResult.spanLength = (spanResult.end - spanResult.start).head<2>().norm();
            spanResult.property = lineConfig.property;
            auto& raw = rawSpans[spanIndex];

            for (int wireId = 0; wireId < wireCount; ++wireId)
            {
                auto& rawNodes = raw.wiresNode.at(wireId);
                if (rawNodes.size() < 2)
                {
                    error = "多档子导线节点列表不足";
                    rollback();
                    return false;
                }

                int firstNodeId = -1;
                if (spanIndex == 0)
                {
                    firstNodeId = wireCount == 1 ? result.leftSupportNodeId :
                        (directWireSupports ? stationWireNodeIds.front().at(wireId) :
                            result.leftTensionEnd.groupNodeIds[groupForWire(wireId)]);
                }
                else
                {
                    firstNodeId = stationWireNodeIds[spanIndex].at(wireId);
                }
                int lastNodeId = -1;
                if (spanIndex + 1 == spanCount)
                {
                    lastNodeId = wireCount == 1 ? result.rightSupportNodeId :
                        (directWireSupports ? stationWireNodeIds.back().at(wireId) :
                            result.rightTensionEnd.groupNodeIds[groupForWire(wireId)]);
                }
                else
                {
                    lastNodeId = stationWireNodeIds[spanIndex + 1].at(wireId);
                }
                rawNodes.front().id = firstNodeId;
                rawNodes.back().id = lastNodeId;

                SubConductorModel spanSub;
                spanSub.wireId = wireId;
                spanSub.nodeIds.push_back(firstNodeId);
                for (std::size_t nodeIndex = 1; nodeIndex + 1 < rawNodes.size(); ++nodeIndex)
                {
                    int nodeId = -1;
                    if (!addNode(
                            Vector3d(rawNodes[nodeIndex].x, rawNodes[nodeIndex].y, rawNodes[nodeIndex].z),
                            nodeId))
                    {
                        error = "添加多档导线内部节点失败";
                        rollback();
                        return false;
                    }
                    rawNodes[nodeIndex].id = nodeId;
                    spanSub.nodeIds.push_back(nodeId);
                }
                spanSub.nodeIds.push_back(lastNodeId);
                spanResult.subConductors[wireId] = spanSub;

                auto& globalNodes = result.subConductors[wireId].nodeIds;
                if (globalNodes.empty())
                    globalNodes.push_back(firstNodeId);
                globalNodes.insert(
                    globalNodes.end(),
                    std::next(spanSub.nodeIds.begin()),
                    spanSub.nodeIds.end());
            }

            for (const auto& [wireId, rawElements] : raw.wiresElement)
            {
                auto& rawNodes = raw.wiresNode.at(wireId);
                auto& spanSub = spanResult.subConductors.at(wireId);
                auto& globalSub = result.subConductors.at(wireId);
                for (const auto& rawElement : rawElements)
                {
                    const int iIndex = rawElement.iNode - 1;
                    const int jIndex = rawElement.jNode - 1;
                    if (iIndex < 0 || jIndex < 0 ||
                        iIndex >= static_cast<int>(rawNodes.size()) ||
                        jIndex >= static_cast<int>(rawNodes.size()))
                    {
                        error = "多档导线单元局部节点索引越界";
                        rollback();
                        return false;
                    }
                    int elementId = -1;
                    if (!AddElement(
                            rawNodes[iIndex].id,
                            rawNodes[jIndex].id,
                            lineConfig.elementType,
                            lineConfig.property,
                            rawElement.initialStress,
                            elementId,
                            error,
                            ElementRole::Conductor,
                            wireId,
                            wireId,
                            wireCount))
                    {
                        error = "添加多档导线单元失败：" + error;
                        rollback();
                        return false;
                    }
                    spanSub.elementIds.push_back(elementId);
                    globalSub.elementIds.push_back(elementId);
                }
            }
        }

        if (!AddTensionEndElements(lineConfig, result, error))
        {
            rollback();
            return false;
        }

        for (std::size_t spanIndex = 0; spanIndex < spanCount; ++spanIndex)
        {
            auto& spanResult = spanResults[spanIndex];
            if (!config.span.innerSpacers.empty() &&
                !BuildInnerSpacers(spanResult, config.span.innerSpacers, error))
            {
                rollback();
                return false;
            }
            if (config.span.useInnerSpacerLayout)
            {
                std::vector<InnerSpacerConfig> autoSpacers;
                if (!CalculateInnerSpacerConfigs(
                        spanResult, config.span.innerSpacerLayout, autoSpacers, error) ||
                    !BuildInnerSpacers(spanResult, autoSpacers, error))
                {
                    rollback();
                    return false;
                }
            }
            result.innerSpacers.insert(
                result.innerSpacers.end(),
                spanResult.innerSpacers.begin(),
                spanResult.innerSpacers.end());
        }

        if (!CreateSubConductorSets(lineConfig, result, error) ||
            !RenumberLineModel(result, error))
        {
            rollback();
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

    int ConductorModelBuilder::NextMPCId() const
    {
        if (!m_structure || m_structure->m_MPCConstraints.empty())
            return 1;
        return m_structure->m_MPCConstraints.rbegin()->first + 1;
    }

    bool ConductorModelBuilder::ValidateProperty(std::shared_ptr<Property> property, const std::string& objectName, _OUT std::string& error) const
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

    std::shared_ptr<ElementBase> ConductorModelBuilder::CreateLineElement(EnumKeyword::ElementType elementType, _OUT std::string& error) const
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
        _OUT int& elementId,
        _OUT std::string& error,
        ElementRole role,
        int wireId,
        int aeroProfileId,
        int aeroBundleCount)
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
        element->m_AeroBundleCount = aeroBundleCount;
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

    bool ConductorModelBuilder::BuildSuspensionSpacer(
        _OUT SuspensionPointModel& suspension,
        const InnerSpacerConfig& config,
        _OUT std::string& error)
    {
        if (suspension.wireNodeIds.size() < 2)
            return true;
        const bool rigidCenterMpc =
            config.style == InnerSpacerStyle::RigidCenterMPC;
        if (rigidCenterMpc && suspension.wireNodeIds.size() < 3)
        {
            error = "刚性中心 MPC 间隔棒至少需要三个子导线节点";
            return false;
        }
        if (!rigidCenterMpc &&
            !ValidateProperty(config.property, "悬垂端相内间隔棒", error))
            return false;
        if (!rigidCenterMpc &&
            config.elementType == EnumKeyword::ElementType::UNKNOWN)
        {
            error = "悬垂端相内间隔棒单元类型不能为 UNKNOWN";
            return false;
        }

        std::vector<int> createdNodeIds;
        std::vector<int> createdElementIds;
        std::vector<int> createdMpcIds;
        auto rollback = [&]()
            {
                for (int mpcId : createdMpcIds)
                    m_structure->m_MPCConstraints.erase(mpcId);
                for (int elementId : createdElementIds)
                    m_structure->m_Elements.erase(elementId);
                for (int nodeId : createdNodeIds)
                    m_structure->m_Nodes.erase(nodeId);
                suspension.spacerCenterNodeId = -1;
                suspension.spacerInnerNodeIds.clear();
                suspension.spacerElementIds.clear();
                suspension.spacerMpcIds.clear();
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
        auto addSpacerElement = [&](int iNodeId, int jNodeId) -> bool
            {
                int elementId = -1;
                if (!AddElement(
                        iNodeId,
                        jNodeId,
                        config.elementType,
                        config.property,
                        0.0,
                        elementId,
                        error,
                        ElementRole::IntraPhaseSpacer))
                    return false;
                createdElementIds.push_back(elementId);
                suspension.spacerElementIds.push_back(elementId);
                return true;
            };

        std::vector<Vector3d> wirePositions;
        wirePositions.reserve(suspension.wireNodeIds.size());
        Vector3d center = Vector3d::Zero();
        for (int nodeId : suspension.wireNodeIds)
        {
            const auto node = m_structure->FindNode(nodeId);
            if (!node)
            {
                error = "悬垂端相内间隔棒引用的子导线节点不存在";
                rollback();
                return false;
            }
            const Vector3d position(node->m_X, node->m_Y, node->m_Z);
            wirePositions.push_back(position);
            center += position;
        }
        center /= static_cast<double>(wirePositions.size());

        if (rigidCenterMpc)
        {
            if (!addNode(center, suspension.spacerCenterNodeId))
            {
                error = "添加悬垂端刚性间隔棒中心节点失败";
                rollback();
                return false;
            }
            const auto master =
                m_structure->FindNode(suspension.spacerCenterNodeId);
            master->SetNumDOFs(6);
            for (std::size_t wireId = 0;
                 wireId < suspension.wireNodeIds.size(); ++wireId)
            {
                const auto slave =
                    m_structure->FindNode(suspension.wireNodeIds[wireId]);
                const int mpcId = NextMPCId();
                auto mpc = std::make_shared<RigidOffsetMPCConstraint>();
                mpc->m_Id = mpcId;
                mpc->m_Name = QStringLiteral("悬垂端刚性中心间隔棒_%1")
                    .arg(mpcId);
                mpc->m_pMasterNode = master;
                mpc->m_pSlaveNode = slave;
                mpc->m_Offset = wirePositions[wireId] - center;
                mpc->m_SlaveDirections = {0, 1, 2};
                if (!m_structure->m_MPCConstraints.emplace(mpcId, mpc).second)
                {
                    error = "添加悬垂端刚性间隔棒 MPC 失败";
                    rollback();
                    return false;
                }
                createdMpcIds.push_back(mpcId);
                suspension.spacerMpcIds.push_back(mpcId);
            }
            return true;
        }

        const std::vector<int>& spacerWireNodeIds =
            suspension.wireNodeIds;

        const int wireCount = static_cast<int>(spacerWireNodeIds.size());
        const int outerEdgeCount = wireCount == 2 ? 1 : wireCount;
        for (int wireId = 0; wireId < outerEdgeCount; ++wireId)
        {
            const int nextWireId = wireCount == 2 ? 1 : (wireId + 1) % wireCount;
            if (!addSpacerElement(
                    spacerWireNodeIds[wireId],
                    spacerWireNodeIds[nextWireId]))
            {
                rollback();
                return false;
            }
        }

        if (wireCount == 2 || config.style == InnerSpacerStyle::OuterPolygon)
            return true;

        if (config.style == InnerSpacerStyle::CenterBraced)
        {
            if (!addNode(center, suspension.spacerCenterNodeId))
            {
                error = "添加悬垂端间隔棒中心节点失败";
                rollback();
                return false;
            }
            for (int wireId = 0; wireId < wireCount; ++wireId)
            {
                if (!addSpacerElement(
                        spacerWireNodeIds[wireId],
                        suspension.spacerCenterNodeId))
                {
                    rollback();
                    return false;
                }
            }
            return true;
        }

        const double scale = std::clamp(config.innerPolygonScale, 0.05, 0.95);
        for (int wireId = 0; wireId < wireCount; ++wireId)
        {
            int innerNodeId = -1;
            const Vector3d innerPosition = center + (wirePositions[wireId] - center) * scale;
            if (!addNode(innerPosition, innerNodeId))
            {
                error = "添加悬垂端间隔棒内圈节点失败";
                rollback();
                return false;
            }
            suspension.spacerInnerNodeIds.push_back(innerNodeId);
        }
        for (int wireId = 0; wireId < wireCount; ++wireId)
        {
            const int nextWireId = (wireId + 1) % wireCount;
            if (!addSpacerElement(
                    suspension.spacerInnerNodeIds[wireId],
                    suspension.spacerInnerNodeIds[nextWireId]) ||
                !addSpacerElement(
                    spacerWireNodeIds[wireId],
                    suspension.spacerInnerNodeIds[wireId]))
            {
                rollback();
                return false;
            }
        }
        return true;
    }

    bool ConductorModelBuilder::BuildInnerSpacer(LineBuildResult& line, const InnerSpacerConfig& config, _OUT InnerSpacerModel& spacer, _OUT std::string& error)
    {
        spacer = InnerSpacerModel{};
        std::vector<int> createdNodeIds;
        std::vector<int> createdMpcIds;
        const bool rigidCenterMpc =
            config.style == InnerSpacerStyle::RigidCenterMPC;

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
        if (rigidCenterMpc && line.subConductors.size() < 3)
        {
            error = "刚性中心 MPC 间隔棒至少需要三个子导线节点";
            return false;
        }

        if (line.spanLength <= 1e-7)
        {
            error = "导线档距无效，无法定位相内间隔棒";
            return false;
        }

        if (!rigidCenterMpc &&
            config.elementType == EnumKeyword::ElementType::UNKNOWN)
        {
            error = "相内间隔棒单元类型不能为 UNKNOWN";
            return false;
        }

        if (!rigidCenterMpc &&
            !ValidateProperty(config.property, "相内间隔棒", error))
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

        const double position = std::clamp(config.position, 0.0, line.spanLength);
        const double ratio = position / line.spanLength;
        spacer.ratio = ratio;
        spacer.position = position;

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
        if (wireNodeIds.size() > 2 &&
            (config.style == InnerSpacerStyle::CenterBraced || rigidCenterMpc))
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
            if (rigidCenterMpc)
                centerNode->SetNumDOFs(6);
        }

        auto rollback = [&]()
            {
                for (int mpcId : createdMpcIds)
                {
                    m_structure->m_MPCConstraints.erase(mpcId);
                }
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

        if (rigidCenterMpc)
        {
            if (centerNodeId < 0)
            {
                error = "刚性中心 MPC 间隔棒至少需要三个子导线节点";
                rollback();
                return false;
            }
            const auto master = m_structure->FindNode(centerNodeId);
            for (std::size_t wireId = 0; wireId < wireNodeIds.size(); ++wireId)
            {
                const auto slave = m_structure->FindNode(wireNodeIds[wireId]);
                const int mpcId = NextMPCId();
                auto mpc = std::make_shared<RigidOffsetMPCConstraint>();
                mpc->m_Id = mpcId;
                mpc->m_Name = QStringLiteral("刚性中心间隔棒_%1")
                    .arg(mpcId);
                mpc->m_pMasterNode = master;
                mpc->m_pSlaveNode = slave;
                mpc->m_Offset = Vector3d(
                    slave->m_X, slave->m_Y, slave->m_Z) - center;
                mpc->m_SlaveDirections = {0, 1, 2};
                if (!m_structure->m_MPCConstraints.emplace(mpcId, mpc).second)
                {
                    error = "添加刚性中心间隔棒 MPC 失败";
                    rollback();
                    return false;
                }
                if (spacer.mpcIds.empty())
                    spacer.id = mpcId;
                createdMpcIds.push_back(mpcId);
                spacer.mpcIds.push_back(mpcId);
            }
            line.innerSpacers.push_back(spacer);
            return true;
        }

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

            if (config.style == InnerSpacerStyle::InnerPolygon)
            {
                const double scale = std::clamp(config.innerPolygonScale, 1.0e-4, 0.9999);
                std::vector<int> innerNodeIds;
                innerNodeIds.reserve(wireNodeIds.size());
                for (int nodeId : wireNodeIds)
                {
                    auto outerNode = m_structure->FindNode(nodeId);
                    if (!outerNode)
                    {
                        error = "添加相内间隔棒内圈时找不到外框节点";
                        rollback();
                        return false;
                    }
                    const Vector3d outer(outerNode->m_X, outerNode->m_Y, outerNode->m_Z);
                    const Vector3d inner = center + scale * (outer - center);
                    const int innerNodeId = NextNodeId();
                    auto innerNode = std::make_shared<Node>();
                    innerNode->m_Id = innerNodeId;
                    innerNode->m_X = inner.x();
                    innerNode->m_Y = inner.y();
                    innerNode->m_Z = inner.z();
                    if (!m_structure->m_Nodes.insert({ innerNodeId, innerNode }).second)
                    {
                        error = "添加相内间隔棒内圈节点失败";
                        rollback();
                        return false;
                    }
                    createdNodeIds.push_back(innerNodeId);
                    spacer.nodeIds.push_back(innerNodeId);
                    spacer.innerNodeIds.push_back(innerNodeId);
                    innerNodeIds.push_back(innerNodeId);
                }
                for (int i = 0; i < static_cast<int>(wireNodeIds.size()); ++i)
                {
                    const int j = (i + 1) % static_cast<int>(wireNodeIds.size());
                    if (!addSpacerElement(innerNodeIds[i], innerNodeIds[j]) ||
                        !addSpacerElement(wireNodeIds[i], innerNodeIds[i]))
                    {
                        error = "添加相内间隔棒内圈单元失败";
                        rollback();
                        return false;
                    }
                }
            }
        }

        line.innerSpacers.push_back(spacer);
        return true;
    }

    bool ConductorModelBuilder::BuildInnerSpacers(LineBuildResult& line, const std::vector<InnerSpacerConfig>& configs, _OUT std::string& error)
    {
        int oldSpacerCount = static_cast<int>(line.innerSpacers.size());

        for (const auto& config : configs)
        {
            InnerSpacerModel spacer;
            if (!BuildInnerSpacer(line, config, spacer, error))
            {
                for (int i = static_cast<int>(line.innerSpacers.size()) - 1; i >= oldSpacerCount; --i)
                {
                    for (int mpcId : line.innerSpacers[i].mpcIds)
                    {
                        m_structure->m_MPCConstraints.erase(mpcId);
                    }
                    for (int elementId : line.innerSpacers[i].elementIds)
                    {
                        m_structure->m_Elements.erase(elementId);
                    }
                    if (line.innerSpacers[i].centerNodeId != -1)
                        m_structure->m_Nodes.erase(line.innerSpacers[i].centerNodeId);
                    for (int nodeId : line.innerSpacers[i].innerNodeIds)
                        m_structure->m_Nodes.erase(nodeId);
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
        _OUT std::vector<InnerSpacerConfig>& configs,
        _OUT std::string& error) const
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
            configs.push_back(config);
        }

        return true;
    }

    bool ConductorModelBuilder::AddNodes(
        BundleResult& raw,
        const LineBuildConfig& config,
        _OUT LineBuildResult& result,
        _OUT std::string& error)
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
        const bool directWireSupports = convergeEnds &&
            config.endTopology == BundleEndTopology::DirectWireSupports;
        std::map<int, int> leftGroupByWire;
        std::map<int, int> rightGroupByWire;
        if (convergeEnds && !directWireSupports)
        {
            const bool dualSupport = config.endTopology == BundleEndTopology::DualSupportByGroup;
            if (!dualSupport &&
                (!addNode(config.start, result.leftSupportNodeId) ||
                 !addNode(config.end, result.rightSupportNodeId)))
            {
                error = "添加耐张导线挂点失败";
                rollback();
                return false;
            }
            if (!dualSupport)
            {
                result.leftTensionEnd.supportNodeId = result.leftSupportNodeId;
                result.rightTensionEnd.supportNodeId = result.rightSupportNodeId;
                result.leftTensionEnd.supportNodeIds.push_back(result.leftSupportNodeId);
                result.rightTensionEnd.supportNodeIds.push_back(result.rightSupportNodeId);
            }

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

            std::vector<Vector3d> leftCenters(groupCount, Vector3d::Zero());
            std::vector<Vector3d> rightCenters(groupCount, Vector3d::Zero());
            for (int group = 0; group < groupCount; ++group)
            {
                int leftGroupId = -1;
                int rightGroupId = -1;
                if (groupSizes[group] <= 0)
                {
                    error = "耐张端部分组节点数量无效";
                    rollback();
                    return false;
                }
                leftCenters[group] = leftSums[group] / groupSizes[group];
                rightCenters[group] = rightSums[group] / groupSizes[group];
                if (!addNode(leftCenters[group], leftGroupId) ||
                    !addNode(rightCenters[group], rightGroupId))
                {
                    error = "添加耐张端部分组节点失败";
                    rollback();
                    return false;
                }
                result.leftTensionEnd.groupNodeIds.push_back(leftGroupId);
                result.rightTensionEnd.groupNodeIds.push_back(rightGroupId);
            }

            if (dualSupport)
            {
                if (groupCount != 2)
                {
                    error = "分组双挂点端部要求至少四分裂导线";
                    rollback();
                    return false;
                }
                const double spacing = config.dualSupportSpacing > 1.0e-7
                    ? config.dualSupportSpacing : config.conductor.spacing;
                auto addDualSupports = [&](const Vector3d& base, const std::vector<Vector3d>& centers,
                                           TensionEndModel& endModel, int& primaryId) -> bool
                    {
                        Vector3d direction = centers[0] - centers[1];
                        if (direction.norm() <= 1.0e-7 || spacing <= 1.0e-7)
                            return false;
                        direction.normalize();
                        int firstId = -1;
                        int secondId = -1;
                        if (!addNode(base + direction * (spacing * 0.5), firstId) ||
                            !addNode(base - direction * (spacing * 0.5), secondId))
                            return false;
                        primaryId = firstId;
                        endModel.supportNodeId = firstId;
                        endModel.supportNodeIds = { firstId, secondId };
                        return true;
                    };
                if (!addDualSupports(config.start, leftCenters, result.leftTensionEnd, result.leftSupportNodeId) ||
                    !addDualSupports(config.end, rightCenters, result.rightTensionEnd, result.rightSupportNodeId))
                {
                    error = "添加分组双挂点失败：请检查双挂点间距和分裂导线几何";
                    rollback();
                    return false;
                }
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
            if (convergeEnds && !directWireSupports)
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

            if (convergeEnds && !directWireSupports)
            {
                sub.nodeIds.push_back(rawNodes.back().id);
            }

            if (directWireSupports)
            {
                // 直接端部不引入汇集/分组节点：每根子导线最外侧节点就是约束挂点。
                result.leftTensionEnd.supportNodeIds.push_back(sub.nodeIds.front());
                result.rightTensionEnd.supportNodeIds.push_back(sub.nodeIds.back());
            }

            result.subConductors.insert({ wireId, sub });
        }

        if (directWireSupports)
        {
            // 保留这两个字段用于现有结果浏览接口；完整挂点集合以 supportNodeIds 为准。
            result.leftSupportNodeId = result.leftTensionEnd.supportNodeIds.front();
            result.rightSupportNodeId = result.rightTensionEnd.supportNodeIds.front();
            result.leftTensionEnd.supportNodeId = result.leftSupportNodeId;
            result.rightTensionEnd.supportNodeId = result.rightSupportNodeId;
        }

        return true;
    }

    bool ConductorModelBuilder::AddElements(const BundleResult& raw, const LineBuildConfig& config, std::shared_ptr<Property> property, _OUT LineBuildResult& result, _OUT std::string& error)
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
                    rawElement.initialStress, elementId, error, ElementRole::Conductor, wireId, wireId,
                    config.conductor.nBundle))
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
        _OUT LineBuildResult& result,
        _OUT std::string& error)
    {
        if (!config.convergeBundleEnds || config.conductor.nBundle <= 1 ||
            config.endTopology == BundleEndTopology::DirectWireSupports)
            return true;

        if (config.endFittingElementType == EnumKeyword::ElementType::UNKNOWN)
        {
            error = "耐张端部单元类型不能为 UNKNOWN";
            return false;
        }

        auto stabilizerProperty = config.endFittingProperty ? config.endFittingProperty : config.property;
        if (!ValidateProperty(stabilizerProperty, "耐张稳定梁", error))
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
                for (std::size_t groupIndex = 0; groupIndex < endModel.groupNodeIds.size(); ++groupIndex)
                {
                    const int groupNodeId = endModel.groupNodeIds[groupIndex];
                    const int supportNodeId = endModel.supportNodeIds.size() == endModel.groupNodeIds.size()
                        ? endModel.supportNodeIds[groupIndex] : endModel.supportNodeId;
                    int elementId = -1;
                    if (!AddElement(
                            supportNodeId,
                            groupNodeId,
                            config.elementType,
                            config.property,
                            config.conductor.initialStress,
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
                            stabilizerProperty,
                            config.conductor.initialStress,
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
            error = "添加耐张端部失败：" + error;
            rollback();
            return false;
        }
        return true;
    }

    bool ConductorModelBuilder::CreateSubConductorSets(
        const LineBuildConfig& config,
        _OUT LineBuildResult& result,
        _OUT std::string& error)
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

    bool ConductorModelBuilder::RenumberLineModel(LineBuildResult& result, _OUT std::string& error)
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
        generatedNodeIds.insert(result.leftTensionEnd.supportNodeIds.begin(), result.leftTensionEnd.supportNodeIds.end());
        generatedNodeIds.insert(result.rightTensionEnd.supportNodeIds.begin(), result.rightTensionEnd.supportNodeIds.end());
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
        for (const auto& suspension : result.suspensionPoints)
        {
            if (suspension.junctionNodeId > 0)
                generatedNodeIds.insert(suspension.junctionNodeId);
            if (suspension.supportNodeId > 0)
                generatedNodeIds.insert(suspension.supportNodeId);
            generatedNodeIds.insert(
                suspension.wireNodeIds.begin(), suspension.wireNodeIds.end());
            if (suspension.spacerCenterNodeId > 0)
                generatedNodeIds.insert(suspension.spacerCenterNodeId);
            generatedNodeIds.insert(
                suspension.spacerInnerNodeIds.begin(), suspension.spacerInnerNodeIds.end());
            generatedElementIds.insert(
                suspension.yokeElementIds.begin(), suspension.yokeElementIds.end());
            if (suspension.stringElementId > 0)
                generatedElementIds.insert(suspension.stringElementId);
            generatedElementIds.insert(
                suspension.spacerElementIds.begin(), suspension.spacerElementIds.end());
        }
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

        if (result.leftTensionEnd.supportNodeIds.size() > 1)
        {
            for (int nodeId : result.leftTensionEnd.supportNodeIds)
                appendUnique(nodeOrder, seenNodes, nodeId);
            for (int nodeId : result.rightTensionEnd.supportNodeIds)
                appendUnique(nodeOrder, seenNodes, nodeId);
        }
        else
        {
            appendUnique(nodeOrder, seenNodes, result.leftSupportNodeId);
            appendUnique(nodeOrder, seenNodes, result.rightSupportNodeId);
        }
        for (const auto& suspension : result.suspensionPoints)
        {
            appendUnique(nodeOrder, seenNodes, suspension.junctionNodeId);
            appendUnique(nodeOrder, seenNodes, suspension.supportNodeId);
            appendUnique(nodeOrder, seenNodes, suspension.spacerCenterNodeId);
            for (int nodeId : suspension.spacerInnerNodeIds)
                appendUnique(nodeOrder, seenNodes, nodeId);
        }
        for (const auto& spacer : result.innerSpacers)
        {
            appendUnique(nodeOrder, seenNodes, spacer.centerNodeId);
            for (int nodeId : spacer.innerNodeIds)
                appendUnique(nodeOrder, seenNodes, nodeId);
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
        for (const auto& suspension : result.suspensionPoints)
        {
            for (int elementId : suspension.yokeElementIds)
                appendUnique(elementOrder, seenElements, elementId);
            appendUnique(elementOrder, seenElements, suspension.stringElementId);
            for (int elementId : suspension.spacerElementIds)
                appendUnique(elementOrder, seenElements, elementId);
        }
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
        remapVector(result.leftTensionEnd.supportNodeIds, nodeIdMap);
        remapVector(result.rightTensionEnd.supportNodeIds, nodeIdMap);
        remapVector(result.leftTensionEnd.groupNodeIds, nodeIdMap);
        remapVector(result.rightTensionEnd.groupNodeIds, nodeIdMap);
        remapVector(result.leftTensionEnd.yokeElementIds, elementIdMap);
        remapVector(result.rightTensionEnd.yokeElementIds, elementIdMap);
        remapId(result.leftTensionEnd.stabilizerElementId, elementIdMap);
        remapId(result.rightTensionEnd.stabilizerElementId, elementIdMap);
        for (auto& suspension : result.suspensionPoints)
        {
            remapId(suspension.junctionNodeId, nodeIdMap);
            remapId(suspension.supportNodeId, nodeIdMap);
            remapVector(suspension.wireNodeIds, nodeIdMap);
            remapVector(suspension.yokeElementIds, elementIdMap);
            remapId(suspension.stringElementId, elementIdMap);
            remapId(suspension.spacerCenterNodeId, nodeIdMap);
            remapVector(suspension.spacerInnerNodeIds, nodeIdMap);
            remapVector(suspension.spacerElementIds, elementIdMap);
        }
        for (auto& [wireId, sub] : result.subConductors)
        {
            remapVector(sub.nodeIds, nodeIdMap);
            remapVector(sub.elementIds, elementIdMap);
        }
        for (auto& spacer : result.innerSpacers)
        {
            remapId(spacer.id, elementIdMap);
            remapId(spacer.centerNodeId, nodeIdMap);
            remapVector(spacer.innerNodeIds, nodeIdMap);
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
