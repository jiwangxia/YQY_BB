#include "StructureData.h"
#include <set>
#include <cmath>
#include <vector>
#include <tuple>
#include <map>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iterator>

namespace
{
// 梁单元在只与杆/索单元相连时，其截面转角没有刚度来源。为每个没有
// 转角支承的梁连通分量补一个转角基准，消除刚体转角零模态；平动不受影响。
// 该处理也覆盖由旧版 BDF/H5 导入的模型。
void EnsureBeamRotationGaugeConstraints(StructureData& structure)
{
    std::map<int, int> parent;
    std::map<int, int> degree;
    const auto findRoot = [&parent](const auto& self, int nodeId) -> int {
        const auto it = parent.find(nodeId);
        if (it == parent.end() || it->second == nodeId)
            return nodeId;
        it->second = self(self, it->second);
        return it->second;
    };
    const auto join = [&parent, &findRoot](int first, int second) {
        const int firstRoot = findRoot(findRoot, first);
        const int secondRoot = findRoot(findRoot, second);
        if (firstRoot != secondRoot)
            parent[secondRoot] = firstRoot;
    };

    for (const auto& [elementId, element] : structure.m_Elements)
    {
        if (!element || element->Get_NodeDOF() < 6 || element->m_pNode.size() < 2)
            continue;
        const auto first = element->m_pNode[0].lock();
        const auto second = element->m_pNode[1].lock();
        if (!first || !second)
            continue;
        parent.emplace(first->m_Id, first->m_Id);
        parent.emplace(second->m_Id, second->m_Id);
        ++degree[first->m_Id];
        ++degree[second->m_Id];
        join(first->m_Id, second->m_Id);
    }
    if (parent.empty())
        return;

    std::map<int, std::vector<int>> components;
    for (const auto& [nodeId, ignored] : parent)
        components[findRoot(findRoot, nodeId)].push_back(nodeId);

    std::map<int, std::set<int>> constrainedRotations;
    for (const auto& [constraintId, constraint] : structure.m_Constraint)
    {
        if (!constraint)
            continue;
        const auto node = constraint->m_pNode.lock();
        const int direction = static_cast<int>(constraint->m_Direction);
        if (node && direction >= 3 && direction <= 5)
            constrainedRotations[node->m_Id].insert(direction);
    }

    for (const auto& [root, nodes] : components)
    {
        bool hasRotationSupport = false;
        int referenceNodeId = 0;
        int referenceDegree = -1;
        for (const int nodeId : nodes)
        {
            const auto constrained = constrainedRotations.find(nodeId);
            if (constrained != constrainedRotations.end()
                && constrained->second.count(3) && constrained->second.count(4) && constrained->second.count(5))
            {
                hasRotationSupport = true;
                break;
            }
            const int nodeDegree = degree[nodeId];
            if (nodeDegree > referenceDegree || (nodeDegree == referenceDegree && nodeId < referenceNodeId))
            {
                referenceNodeId = nodeId;
                referenceDegree = nodeDegree;
            }
        }
        if (!hasRotationSupport && referenceNodeId > 0)
            structure.Add_Constraint({ referenceNodeId }, { 3, 4, 5 }, { 0.0, 0.0, 0.0 });
    }
}
}

StructureData::~StructureData() = default;

void StructureData::Clear()
{
    // 先关闭并清理仍可能引用当前模型的结果流，再释放模型实体。
    m_Outputter.Clear();

    m_Nodes.clear();
    m_Elements.clear();
    m_Material.clear();
    m_Section.clear();
    m_Property.clear();
    m_Constraint.clear();
    m_MPCConstraints.clear();
    m_Load.clear();
    m_AnalysisStep.clear();
    m_ModelSets.clear();
    m_ComputeRegions.clear();

    // Clear 表示恢复为一个全新的空模型，不能保留上一次导入或求解的状态。
    m_OutputControl = OutputControl{};
    m_AeroManager = AeroManager{};
}

std::shared_ptr<StructureData> StructureData::CloneForAnalysis(QString* errorMessage) const
{
    const auto fail = [errorMessage](const QString& message) -> std::shared_ptr<StructureData> {
        if (errorMessage)
            *errorMessage = message;
        qWarning().noquote() << QStringLiteral("计算模型复制失败：") << message;
        return nullptr;
    };
    auto clone = std::make_shared<StructureData>();
    clone->m_OutputControl = m_OutputControl;
    clone->m_AeroManager = m_AeroManager;

    for (const auto& [id, source] : m_Nodes)
    {
        if (!source)
            return fail(QStringLiteral("节点 %1 为空").arg(id));
        auto target = std::make_shared<Node>(*source);
        clone->m_Nodes.emplace(id, std::move(target));
    }

    for (const auto& [id, source] : m_Material)
    {
        if (!source)
            return fail(QStringLiteral("材料 %1 为空").arg(id));
        clone->m_Material.emplace(id, std::make_shared<Material>(*source));
    }

    for (const auto& [id, source] : m_Section)
    {
        if (!source)
            return fail(QStringLiteral("截面 %1 为空").arg(id));
        if (const auto circular = std::dynamic_pointer_cast<SectionCircular>(source))
            clone->m_Section.emplace(id, std::make_shared<SectionCircular>(*circular));
        else if (const auto rectangle = std::dynamic_pointer_cast<SectionRectangle>(source))
            clone->m_Section.emplace(id, std::make_shared<SectionRectangle>(*rectangle));
        else
            return fail(QStringLiteral("截面 %1 类型不受支持").arg(id));
    }

    for (const auto& [id, source] : m_Property)
    {
        if (!source)
            return fail(QStringLiteral("属性 %1 为空").arg(id));
        auto target = std::make_shared<Property>();
        target->m_Id = source->m_Id;
        if (const auto material = source->m_pMaterial.lock())
        {
            const auto found = clone->m_Material.find(material->m_Id);
            if (found == clone->m_Material.end())
                return fail(QStringLiteral("属性 %1 的材料引用无效").arg(id));
            target->m_pMaterial = found->second;
        }
        if (const auto section = source->m_pSection.lock())
        {
            const auto found = clone->m_Section.find(section->m_Id);
            if (found == clone->m_Section.end())
                return fail(QStringLiteral("属性 %1 的截面引用无效").arg(id));
            target->m_pSection = found->second;
        }
        clone->m_Property.emplace(id, std::move(target));
    }

    for (const auto& [id, source] : m_Elements)
    {
        if (!source)
            return fail(QStringLiteral("单元 %1 为空").arg(id));

        std::shared_ptr<ElementBase> target;
        if (std::dynamic_pointer_cast<ElementTruss>(source))
            target = std::make_shared<ElementTruss>();
        else if (std::dynamic_pointer_cast<ElementCable>(source))
            target = std::make_shared<ElementCable>();
        else if (const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(source))
        {
            auto beamTarget = std::make_shared<ElementBeam_CR>();
            beamTarget->q0 = beam->q0;
            beamTarget->R0 = beam->R0;
            target = std::move(beamTarget);
        }
        else
            return fail(QStringLiteral("单元 %1 类型不受支持").arg(id));

        target->m_Id = source->m_Id;
        target->L0 = source->L0;
        target->L = source->L;
        target->m_InitStress = source->m_InitStress;
        target->m_Stress = source->m_Stress;
        target->m_Role = source->m_Role;
        target->m_WireId = source->m_WireId;
        target->m_AeroBundleCount = source->m_AeroBundleCount;
        target->m_AeroProfileId = source->m_AeroProfileId;
        target->m_inforce = source->m_inforce;
        target->m_pNode.clear();
        for (const auto& sourceNodeRef : source->m_pNode)
        {
            const auto sourceNode = sourceNodeRef.lock();
            if (!sourceNode)
                return fail(QStringLiteral("单元 %1 的节点引用已失效").arg(id));
            const auto found = clone->m_Nodes.find(sourceNode->m_Id);
            if (found == clone->m_Nodes.end())
                return fail(QStringLiteral("单元 %1 引用了不存在的节点").arg(id));
            target->m_pNode.push_back(found->second);
        }
        if (const auto sourceProperty = source->m_pProperty.lock())
        {
            const auto found = clone->m_Property.find(sourceProperty->m_Id);
            if (found == clone->m_Property.end())
                return fail(QStringLiteral("单元 %1 的属性引用无效").arg(id));
            target->m_pProperty = found->second;
        }
        clone->m_Elements.emplace(id, std::move(target));
    }

    for (const auto& [id, source] : m_Constraint)
    {
        if (!source)
            return fail(QStringLiteral("约束 %1 为空").arg(id));
        auto target = std::make_shared<Constraint>(*source);
        if (const auto sourceNode = source->m_pNode.lock())
        {
            const auto found = clone->m_Nodes.find(sourceNode->m_Id);
            if (found == clone->m_Nodes.end())
                return fail(QStringLiteral("约束 %1 的节点引用无效").arg(id));
            target->m_pNode = found->second;
        }
        clone->m_Constraint.emplace(id, std::move(target));
    }

    for (const auto& [id, source] : m_MPCConstraints)
    {
        if (!source)
            return fail(QStringLiteral("MPC %1 为空").arg(id));
        auto target = source->Clone(clone->m_Nodes);
        if (!target)
            return fail(QStringLiteral("MPC %1 的节点引用无效").arg(id));
        clone->m_MPCConstraints.emplace(id, std::move(target));
    }

    const auto copyLoadBase = [](const LoadBase& source, LoadBase& target) {
        target.m_Id = source.m_Id;
        target.m_Name = source.m_Name;
        target.m_LoadType = source.m_LoadType;
        target.m_Direction = source.m_Direction;
        target.m_StepId = source.m_StepId;
        target.m_StartTime = source.m_StartTime;
        target.m_EndTime = source.m_EndTime;
        target.m_FunctionType = source.m_FunctionType;
        target.m_Amplitude = source.m_Amplitude;
        target.m_Frequency = source.m_Frequency;
        target.m_Phase = source.m_Phase;
        target.m_Offset = source.m_Offset;
        target.m_RampT0 = source.m_RampT0;
        target.m_RampT1 = source.m_RampT1;
        target.m_Decay = source.m_Decay;
        target.m_Period = source.m_Period;
        target.m_DutyCycle = source.m_DutyCycle;
    };

    for (const auto& [id, source] : m_Load)
    {
        if (!source)
            return fail(QStringLiteral("荷载 %1 为空").arg(id));
        std::shared_ptr<LoadBase> target;
        if (const auto nodeLoad = std::dynamic_pointer_cast<Force_Node>(source))
        {
            auto copied = std::make_shared<Force_Node>();
            copied->m_Value = nodeLoad->m_Value;
            if (const auto sourceNode = nodeLoad->m_pNode.lock())
            {
                const auto found = clone->m_Nodes.find(sourceNode->m_Id);
                if (found == clone->m_Nodes.end())
                    return fail(QStringLiteral("节点荷载 %1 的节点引用无效").arg(id));
                copied->m_pNode = found->second;
            }
            target = std::move(copied);
        }
        else if (const auto elementLoad = std::dynamic_pointer_cast<Force_Element>(source))
        {
            auto copied = std::make_shared<Force_Element>();
            copied->m_Value = elementLoad->m_Value;
            if (const auto sourceElement = elementLoad->m_pElement.lock())
            {
                const auto found = clone->m_Elements.find(sourceElement->m_Id);
                if (found == clone->m_Elements.end())
                    return fail(QStringLiteral("单元荷载 %1 的单元引用无效").arg(id));
                copied->m_pElement = found->second;
            }
            target = std::move(copied);
        }
        else if (const auto gravity = std::dynamic_pointer_cast<Force_Gravity>(source))
        {
            auto copied = std::make_shared<Force_Gravity>();
            copied->m_g = gravity->m_g;
            target = std::move(copied);
        }
        else if (const auto wind = std::dynamic_pointer_cast<Force_Wind>(source))
        {
            auto copied = std::make_shared<Force_Wind>();
            copied->m_velocity = wind->m_velocity;
            copied->m_direction = wind->m_direction;
            copied->m_windDensity = wind->m_windDensity;
            target = std::move(copied);
        }
        else
            return fail(QStringLiteral("荷载 %1 类型不受支持").arg(id));
        copyLoadBase(*source, *target);
        clone->m_Load.emplace(id, std::move(target));
    }

    for (const auto& [id, source] : m_AnalysisStep)
    {
        if (!source)
            return fail(QStringLiteral("分析步 %1 为空").arg(id));
        AnalysisStepConfig config;
        config.id = source->m_Id;
        config.name = source->m_Name;
        config.type = source->m_Type;
        config.totalTime = source->m_Time;
        config.stepSize = source->m_StepSize;
        config.tolerance = source->m_Tolerance;
        config.maxIterations = source->m_MaxIterations;
        config.dynamicSolverType = source->m_DynamicSolverType;
        config.initialStaticStepId = source->m_InitialStaticStepId;
        config.adaptiveTssbn = source->m_AdaptiveTssbn;
        config.enableGalloping = source->m_EnableGalloping;
        config.gallopingIceThickness = source->m_GallopingIceThickness;
        config.gallopingInitialAttackDegrees =
            source->m_GallopingInitialAttackDegrees;
        config.regionScope = source->m_RegionScope;
        config.computeRegionIds = source->m_ComputeRegionIds;
        clone->AddAnalysisStep(config);
    }

    for (const auto& [id, source] : m_ModelSets)
    {
        if (!source)
            return fail(QStringLiteral("集合 %1 为空").arg(id));
        clone->m_ModelSets.emplace(id, std::make_shared<ModelSet>(*source));
    }

    for (const auto& [id, source] : m_ComputeRegions)
    {
        if (!source)
            return fail(QStringLiteral("计算区域 %1 为空").arg(id));
        clone->m_ComputeRegions.emplace(id, std::make_shared<ComputeRegion>(*source));
    }

    return clone;
}

int StructureData::AddModelSet(const QString& name, ModelSetType type, const std::set<int>& ids,
    QString* errorMessage)
{
    if (ids.empty())
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("集合不能为空。");
        return 0;
    }

    for (int id : ids)
    {
        const bool exists = type == ModelSetType::Node
            ? m_Nodes.find(id) != m_Nodes.cend()
            : m_Elements.find(id) != m_Elements.cend();
        if (!exists)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("集合引用了不存在的%1 ID %2。")
                    .arg(type == ModelSetType::Node ? QStringLiteral("节点") : QStringLiteral("单元"))
                    .arg(id);
            return 0;
        }
    }

    const int id = m_ModelSets.empty() ? 1 : m_ModelSets.rbegin()->first + 1;
    auto modelSet = std::make_shared<ModelSet>(type);
    modelSet->m_Id = id;
    modelSet->m_Name = name.trimmed().isEmpty() ? QStringLiteral("集合-%1").arg(id) : name.trimmed();
    modelSet->m_Ids = ids;
    m_ModelSets.emplace(id, std::move(modelSet));
    return id;
}

int StructureData::AddComputeRegion(const QString& name, const std::set<int>& nodeIds,
    const std::set<int>& elementIds, const std::set<int>& sourceSetIds,
    bool enabled, QString* errorMessage)
{
    std::set<int> effectiveNodeIds = nodeIds;
    std::set<int> effectiveElementIds = elementIds;
    for (int setId : sourceSetIds)
    {
        const auto setIt = m_ModelSets.find(setId);
        if (setIt == m_ModelSets.cend() || !setIt->second)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("计算区域引用了不存在的集合 ID %1。").arg(setId);
            return 0;
        }
        if (setIt->second->m_Type == ModelSetType::Node)
            effectiveNodeIds.insert(setIt->second->m_Ids.cbegin(), setIt->second->m_Ids.cend());
        else
            effectiveElementIds.insert(setIt->second->m_Ids.cbegin(), setIt->second->m_Ids.cend());
    }
    if (effectiveNodeIds.empty() && effectiveElementIds.empty())
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("计算区域必须包含节点或单元。");
        return 0;
    }

    auto region = std::make_shared<ComputeRegion>();
    region->m_Id = m_ComputeRegions.empty() ? 1 : m_ComputeRegions.rbegin()->first + 1;
    region->m_Name = name.trimmed().isEmpty()
        ? QStringLiteral("计算区域-%1").arg(region->m_Id) : name.trimmed();
    region->m_Enabled = enabled;
    region->m_SourceSetIds = sourceSetIds;
    region->m_DirectNodeIds = nodeIds;
    region->m_DirectElementIds = elementIds;
    region->m_NodeIds = effectiveNodeIds;
    region->m_ElementIds = effectiveElementIds;

    for (int elementId : region->m_ElementIds)
    {
        const auto elementIt = m_Elements.find(elementId);
        if (elementIt == m_Elements.cend() || !elementIt->second)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("计算区域引用了不存在的单元 ID %1。").arg(elementId);
            return 0;
        }
        for (const auto& nodeRef : elementIt->second->m_pNode)
        {
            const auto node = nodeRef.lock();
            if (!node || m_Nodes.find(node->m_Id) == m_Nodes.cend())
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("单元 %1 存在无效端点节点。").arg(elementId);
                return 0;
            }
            region->m_NodeIds.insert(node->m_Id);
        }
    }

    for (int nodeId : region->m_NodeIds)
    {
        if (m_Nodes.find(nodeId) == m_Nodes.cend())
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("计算区域引用了不存在的节点 ID %1。").arg(nodeId);
            return 0;
        }
    }

    if (region->m_ElementIds.empty() && !region->m_NodeIds.empty())
    {
        for (const auto& [elementId, element] : m_Elements)
        {
            if (!element || element->m_pNode.empty())
                continue;
            bool allNodesSelected = true;
            for (const auto& nodeRef : element->m_pNode)
            {
                const auto node = nodeRef.lock();
                if (!node || region->m_NodeIds.find(node->m_Id) == region->m_NodeIds.cend())
                {
                    allNodesSelected = false;
                    break;
                }
            }
            if (allNodesSelected)
                region->m_ElementIds.insert(elementId);
        }
    }

    if (region->m_ElementIds.empty())
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("计算区域中没有可计算单元。");
        return 0;
    }

    if (m_ComputeRegions.size() == 1 && m_ComputeRegions.cbegin()->second
        && m_ComputeRegions.cbegin()->second->m_AutoGenerated)
    {
        m_ComputeRegions.clear();
    }
    const int newId = region->m_Id;
    const int representativeElementId = *region->m_ElementIds.cbegin();
    m_ComputeRegions.emplace(newId, std::move(region));
    if (!RebuildAndMergeComputeRegions(errorMessage))
    {
        m_ComputeRegions.erase(newId);
        return 0;
    }

    for (const auto& [id, candidate] : m_ComputeRegions)
    {
        if (candidate && candidate->m_ElementIds.find(representativeElementId) != candidate->m_ElementIds.cend())
            return id;
    }
    return newId;
}

int StructureData::AddComputeRegionFromSets(const QString& name,
    const std::set<int>& sourceSetIds, bool enabled, QString* errorMessage)
{
    return AddComputeRegion(name, {}, {}, sourceSetIds, enabled, errorMessage);
}

bool StructureData::RemoveComputeRegion(int regionId)
{
    if (m_ComputeRegions.erase(regionId) == 0)
        return false;

    for (auto& [stepId, step] : m_AnalysisStep)
    {
        Q_UNUSED(stepId);
        if (step)
            step->m_ComputeRegionIds.erase(regionId);
    }
    return true;
}

bool StructureData::RebuildAndMergeComputeRegions(QString* errorMessage)
{
    for (auto& [regionId, region] : m_ComputeRegions)
    {
        if (!region)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("计算区域 %1 为空。").arg(regionId);
            return false;
        }
        region->m_NodeIds = region->m_DirectNodeIds;
        region->m_ElementIds = region->m_DirectElementIds;
        for (int setId : region->m_SourceSetIds)
        {
            const auto setIt = m_ModelSets.find(setId);
            if (setIt == m_ModelSets.cend() || !setIt->second)
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("计算区域 %1 引用了不存在的集合 %2。")
                        .arg(regionId).arg(setId);
                return false;
            }
            if (setIt->second->m_Type == ModelSetType::Node)
            {
                region->m_NodeIds.insert(setIt->second->m_Ids.cbegin(), setIt->second->m_Ids.cend());
            }
            else
            {
                region->m_ElementIds.insert(setIt->second->m_Ids.cbegin(), setIt->second->m_Ids.cend());
            }
        }
        if (region->m_ElementIds.empty() && !region->m_NodeIds.empty())
        {
            for (const auto& [elementId, element] : m_Elements)
            {
                if (!element || element->m_pNode.empty())
                    continue;
                bool allNodesSelected = true;
                for (const auto& nodeRef : element->m_pNode)
                {
                    const auto node = nodeRef.lock();
                    if (!node || region->m_NodeIds.find(node->m_Id) == region->m_NodeIds.cend())
                    {
                        allNodesSelected = false;
                        break;
                    }
                }
                if (allNodesSelected)
                    region->m_ElementIds.insert(elementId);
            }
        }
        for (int elementId : region->m_ElementIds)
        {
            const auto elementIt = m_Elements.find(elementId);
            if (elementIt == m_Elements.cend() || !elementIt->second)
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("计算区域 %1 引用了不存在的单元 %2。")
                        .arg(regionId).arg(elementId);
                return false;
            }
            for (const auto& nodeRef : elementIt->second->m_pNode)
            {
                const auto node = nodeRef.lock();
                if (!node)
                {
                    if (errorMessage)
                        *errorMessage = QStringLiteral("计算区域 %1 的单元 %2 存在失效节点引用。")
                            .arg(regionId).arg(elementId);
                    return false;
                }
                region->m_NodeIds.insert(node->m_Id);
            }
        }

        // MPC ownership follows the master node.  Once a master belongs to
        // this region, every dependent node must be available in the same
        // regional solve.  Repeat to support chains such as A->B->C.
        bool mpcExpanded = true;
        while (mpcExpanded)
        {
            mpcExpanded = false;
            for (const auto& [mpcId, mpc] : m_MPCConstraints)
            {
                if (!mpc)
                {
                    if (errorMessage)
                        *errorMessage = QStringLiteral("MPC %1 为空。").arg(mpcId);
                    return false;
                }
                const std::vector<int> nodeIds = mpc->GetNodeIds();
                if (nodeIds.size() != 2)
                {
                    if (errorMessage)
                        *errorMessage = QStringLiteral(
                            "MPC %1 没有有效的主从节点。").arg(mpcId);
                    return false;
                }
                const int masterNodeId = nodeIds[0];
                const int slaveNodeId = nodeIds[1];
                if (m_Nodes.find(masterNodeId) == m_Nodes.cend()
                    || m_Nodes.find(slaveNodeId) == m_Nodes.cend())
                {
                    if (errorMessage)
                        *errorMessage = QStringLiteral(
                            "MPC %1 引用了不存在的节点。").arg(mpcId);
                    return false;
                }
                if (region->ContainsNode(masterNodeId)
                    && region->m_NodeIds.insert(slaveNodeId).second)
                {
                    mpcExpanded = true;
                }
            }
        }
    }

    bool merged = true;
    while (merged)
    {
        merged = false;
        for (auto lhsIt = m_ComputeRegions.begin(); lhsIt != m_ComputeRegions.end() && !merged; ++lhsIt)
        {
            auto rhsIt = std::next(lhsIt);
            while (rhsIt != m_ComputeRegions.end())
            {
                if (lhsIt->second->Overlaps(*rhsIt->second))
                {
                    const int removedId = rhsIt->first;
                    lhsIt->second->MergeFrom(*rhsIt->second);
                    lhsIt->second->m_Name = QStringLiteral("%1 + %2")
                        .arg(lhsIt->second->m_Name, rhsIt->second->m_Name);
                    rhsIt = m_ComputeRegions.erase(rhsIt);
                    for (auto& [stepId, step] : m_AnalysisStep)
                    {
                        Q_UNUSED(stepId);
                        if (step && step->m_ComputeRegionIds.erase(removedId) > 0)
                            step->m_ComputeRegionIds.insert(lhsIt->first);
                    }
                    merged = true;
                    break;
                }
                ++rhsIt;
            }
        }
    }
    return ValidateComputeRegions(errorMessage);
}

bool StructureData::ValidateComputeRegions(QString* errorMessage) const
{
    for (auto lhsIt = m_ComputeRegions.cbegin(); lhsIt != m_ComputeRegions.cend(); ++lhsIt)
    {
        const auto& region = lhsIt->second;
        if (!region || region->m_ElementIds.empty() || region->m_NodeIds.empty())
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("计算区域 %1 没有有效节点或单元。").arg(lhsIt->first);
            return false;
        }
        for (int nodeId : region->m_NodeIds)
        {
            if (m_Nodes.find(nodeId) == m_Nodes.cend())
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("计算区域 %1 引用了不存在的节点 %2。")
                        .arg(lhsIt->first).arg(nodeId);
                return false;
            }
        }
        for (const auto& [mpcId, mpc] : m_MPCConstraints)
        {
            if (!mpc)
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("MPC %1 为空。").arg(mpcId);
                return false;
            }
            const std::vector<int> nodeIds = mpc->GetNodeIds();
            if (nodeIds.size() != 2)
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral(
                        "MPC %1 没有有效的主从节点。").arg(mpcId);
                return false;
            }
            if (region->ContainsNode(nodeIds[0])
                && !region->ContainsNode(nodeIds[1]))
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral(
                        "计算区域 %1 包含 MPC %2 的主节点 %3，"
                        "但没有包含从节点 %4。")
                        .arg(lhsIt->first).arg(mpcId)
                        .arg(nodeIds[0]).arg(nodeIds[1]);
                return false;
            }
        }
        auto rhsIt = std::next(lhsIt);
        for (; rhsIt != m_ComputeRegions.cend(); ++rhsIt)
        {
            if (rhsIt->second && region->Overlaps(*rhsIt->second))
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("计算区域 %1 与 %2 仍有共享节点或单元。")
                        .arg(lhsIt->first).arg(rhsIt->first);
                return false;
            }
        }
    }
    return true;
}

void StructureData::EnsureDefaultAnalysisConfiguration()
{
    if (m_ComputeRegions.empty() && !m_Elements.empty())
    {
        std::set<int> elementIds;
        for (const auto& [elementId, element] : m_Elements)
        {
            if (element)
                elementIds.insert(elementId);
        }
        QString ignoredError;
        const int defaultRegionId = AddComputeRegion(
            QStringLiteral("默认计算区域"), {}, elementIds, {}, true, &ignoredError);
        const auto regionIt = m_ComputeRegions.find(defaultRegionId);
        if (regionIt != m_ComputeRegions.end() && regionIt->second)
            regionIt->second->m_AutoGenerated = true;
    }

    if (m_AnalysisStep.empty() && !m_ComputeRegions.empty())
    {
        AnalysisStepConfig config;
        config.id = 1;
        config.name = QStringLiteral("默认分析步");
        config.type = EnumKeyword::StepType::STATIC;
        config.totalTime = 1.0;
        config.stepSize = 0.01;
        config.tolerance = 1.0e-5;
        config.maxIterations = 50;
        config.regionScope = AnalysisRegionScope::AllEnabledRegions;
        AddAnalysisStep(config);
    }

    EnsureBeamRotationGaugeConstraints(*this);
}

std::vector<int> StructureData::ResolveAnalysisStepRegionIds(const AnalysisStep& step) const
{
    std::vector<int> result;
    for (const auto& [regionId, region] : m_ComputeRegions)
    {
        if (!region || !region->m_Enabled)
            continue;
        if (step.m_RegionScope == AnalysisRegionScope::AllEnabledRegions
            || step.m_ComputeRegionIds.find(regionId) != step.m_ComputeRegionIds.cend())
        {
            result.push_back(regionId);
        }
    }
    return result;
}

std::shared_ptr<StructureData> StructureData::CloneRegionForAnalysis(int regionId,
    int analysisStepId, QString* errorMessage) const
{
    return CloneRegionForAnalysis(regionId, std::set<int>{analysisStepId}, errorMessage);
}

std::shared_ptr<StructureData> StructureData::CloneRegionForAnalysis(int regionId,
    const std::set<int>& analysisStepIds, QString* errorMessage) const
{
    const auto regionIt = m_ComputeRegions.find(regionId);
    if (regionIt == m_ComputeRegions.cend() || !regionIt->second || !regionIt->second->m_Enabled)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("计算区域 %1 不存在或未启用。").arg(regionId);
        return nullptr;
    }
    if (analysisStepIds.empty())
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("区域计算没有指定分析步。");
        return nullptr;
    }
    for (int analysisStepId : analysisStepIds)
    {
        if (m_AnalysisStep.find(analysisStepId) == m_AnalysisStep.cend())
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("分析步 %1 不存在。").arg(analysisStepId);
            return nullptr;
        }
    }

    auto clone = CloneForAnalysis(errorMessage);
    if (!clone)
        return nullptr;

    const auto region = regionIt->second;
    for (auto it = clone->m_Elements.begin(); it != clone->m_Elements.end(); )
    {
        if (region->m_ElementIds.find(it->first) == region->m_ElementIds.cend())
            it = clone->m_Elements.erase(it);
        else
            ++it;
    }
    for (auto it = clone->m_Nodes.begin(); it != clone->m_Nodes.end(); )
    {
        if (region->m_NodeIds.find(it->first) == region->m_NodeIds.cend())
            it = clone->m_Nodes.erase(it);
        else
            ++it;
    }
    for (auto it = clone->m_Constraint.begin(); it != clone->m_Constraint.end(); )
    {
        const auto node = it->second ? it->second->m_pNode.lock() : nullptr;
        if (!node || clone->m_Nodes.find(node->m_Id) == clone->m_Nodes.cend())
            it = clone->m_Constraint.erase(it);
        else
            ++it;
    }
    for (auto it = clone->m_MPCConstraints.begin();
         it != clone->m_MPCConstraints.end(); )
    {
        if (!it->second)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("MPC %1 为空。").arg(it->first);
            return nullptr;
        }
        const std::vector<int> nodeIds = it->second->GetNodeIds();
        if (nodeIds.size() != 2)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral(
                    "MPC %1 没有有效的主从节点。").arg(it->first);
            return nullptr;
        }

        // MPC remains permanently stored in the source model.  A regional
        // analysis clone owns it exactly when it owns the master node.
        if (!region->ContainsNode(nodeIds[0]))
        {
            it = clone->m_MPCConstraints.erase(it);
            continue;
        }
        if (!region->ContainsNode(nodeIds[1])
            || clone->m_Nodes.find(nodeIds[0]) == clone->m_Nodes.cend()
            || clone->m_Nodes.find(nodeIds[1]) == clone->m_Nodes.cend())
        {
            if (errorMessage)
                *errorMessage = QStringLiteral(
                    "计算区域 %1 中 MPC %2 的主从节点不完整。")
                    .arg(regionId).arg(it->first);
            return nullptr;
        }
        ++it;
    }
    for (auto it = clone->m_Load.begin(); it != clone->m_Load.end(); )
    {
        bool keep = static_cast<bool>(it->second);
        if (const auto nodeLoad = std::dynamic_pointer_cast<Force_Node>(it->second))
        {
            const auto node = nodeLoad->m_pNode.lock();
            keep = node && clone->m_Nodes.find(node->m_Id) != clone->m_Nodes.cend();
        }
        else if (const auto elementLoad = std::dynamic_pointer_cast<Force_Element>(it->second))
        {
            const auto element = elementLoad->m_pElement.lock();
            keep = element && clone->m_Elements.find(element->m_Id) != clone->m_Elements.cend();
        }
        if (!keep)
            it = clone->m_Load.erase(it);
        else
            ++it;
    }
    for (auto it = clone->m_AnalysisStep.begin(); it != clone->m_AnalysisStep.end(); )
    {
        if (analysisStepIds.find(it->first) == analysisStepIds.cend())
            it = clone->m_AnalysisStep.erase(it);
        else
        {
            it->second->m_RegionScope = AnalysisRegionScope::SelectedRegions;
            it->second->m_ComputeRegionIds = {regionId};
            ++it;
        }
    }
    for (auto it = clone->m_ComputeRegions.begin(); it != clone->m_ComputeRegions.end(); )
    {
        if (it->first != regionId)
            it = clone->m_ComputeRegions.erase(it);
        else
            ++it;
    }
    return clone;
}

std::shared_ptr<Node> StructureData::FindNode(int id)
{
    auto result = m_Nodes.find(id);
    if (result != m_Nodes.end())
    {
        return result->second;
    }
    return nullptr;
}

std::shared_ptr<ElementBase> StructureData::FindElement(int id)
{
    auto result = m_Elements.find(id);
    if (result != m_Elements.end())
    {
        return result->second;
    }
    return nullptr;
}

std::shared_ptr<Material> StructureData::FindMaterial(int id)
{
    auto result = m_Material.find(id);
    if (result != m_Material.end())
    {
        return result->second;
    }
    qDebug().noquote() << "未找到材料id =" << id;
    return nullptr;
}

std::shared_ptr<SectionBase> StructureData::FindSection(int id)
{
    auto result = m_Section.find(id);
    if (result != m_Section.end())
    {
        return result->second;
    }
    qDebug().noquote() << "未找到截面id =" << id;
    return nullptr;
}

std::shared_ptr<Property> StructureData::FindProperty(int id)
{
    auto result = m_Property.find(id);
    if (result != m_Property.end())
    {
        return result->second;
    }
    qDebug().noquote() << "未找到属性id =" << id;
    return nullptr;
}

std::shared_ptr<Property> StructureData::Create_Property(int id_material, int id_section)
{
    // 检查是否已存在相同材料+截面组合的 Property
    for (const auto& pair : m_Property)
    {
        auto existingProp = pair.second;
        auto existingMat = existingProp->m_pMaterial.lock();
        auto existingSec = existingProp->m_pSection.lock();

        if (existingMat && existingSec &&
            existingMat->m_Id == id_material && existingSec->m_Id == id_section)
        {
            // 已存在相同组合，直接返回
            return existingProp;
        }
    }

    // 不存在，创建新的 Property
    int property_id = static_cast<int>(m_Property.size()) + 1;
    auto property = std::make_shared<Property>();

    auto iterMat = m_Material.find(id_material);
    if (iterMat != m_Material.end())
    {
        property->m_pMaterial = iterMat->second;
    }
    else
    {
        qDebug().noquote() << QStringLiteral("Error: Create_Property 未找到材料 id=") << id_material;
        return nullptr;
    }

    auto iterSec = m_Section.find(id_section);
    if (iterSec != m_Section.end())
    {
        property->m_pSection = iterSec->second;
    }
    else
    {
        qDebug().noquote() << QStringLiteral("Error: Create_Property 未找到截面 id=") << id_section;
        return nullptr;
    }

    property->m_Id = property_id;
    m_Property.insert(std::make_pair(property_id, property));
    return property;
}

void StructureData::Add_Property(double E, double density, double Area, double* v, double* S, double* e)
{
    auto pMaterial = std::make_shared<Material>();
    pMaterial->m_Id = int(m_Material.size()) + 1;
    pMaterial->m_Young = E;
    pMaterial->m_Density = density;

    if (v != nullptr) 
    {
        pMaterial->m_Poisson = *v;
    }
    else
        pMaterial->m_Poisson = 0.0;

    if (S != nullptr) 
    {
        // S 在材料定义中表示极限应力。
        pMaterial->m_MaxStress = *S;
    }
    else
        pMaterial->m_MaxStress = 0.0;

    if (e != nullptr) 
    {
        // 假设m_Expansion是double类型
        pMaterial->m_Expansion = *e;
    }
    else
        pMaterial->m_Expansion = 0.0;

    m_Material.insert(std::make_pair(pMaterial->m_Id, pMaterial));

    int autoId = static_cast<int>(m_Section.size()) + 1;

    auto pSection = std::make_shared<SectionCircular>();
    pSection->m_Id = autoId;
    pSection->m_Area = Area;
    pSection->Calculate_Radius();
    m_Section.insert(std::make_pair(autoId, pSection));
}

int StructureData::Add_Constraint(std::vector<int> Nodeid, std::vector<int> direaction, std::vector<double> value)
{
    if (direaction.size() != value.size())
    {
        qDebug().noquote() << QStringLiteral("Error: 约束方向数量与约束值数量不一致");
        return 0;
    }
    size_t num = value.size();
    for (auto& node : Nodeid)
    {
        auto pNode = FindNode(node);
        if (!pNode)
        {
            qDebug().noquote() << QStringLiteral("Error: 添加约束时未找到节点 id=") << node;
            continue;
        }
        for (int i = 0; i < num; ++i)
        {
            auto pConstraint = std::make_shared<Constraint>();
            pConstraint->m_Id = static_cast<int>(m_Constraint.size()) + 1;
            pConstraint->m_pNode = pNode;
            pConstraint->m_Direction = static_cast<EnumKeyword::Direction>(direaction[i]);
            pConstraint->m_Value = value[i];
            m_Constraint.insert({ pConstraint->m_Id,pConstraint });
        }
    }
    return int(num * Nodeid.size());
}

void StructureData::Add_Gravity(int direction, int idStep)
{
    int autoId = static_cast<int>(m_Load.size()) + 1;

    auto pLoad = std::make_shared<Force_Gravity>();
    pLoad->m_Id = autoId;

    pLoad->m_Direction = static_cast<EnumKeyword::Direction>(direction);

    pLoad->m_StepId = idStep;
    m_Load.insert(std::make_pair(autoId, pLoad));
}

void StructureData::AddAnalysisStep(const AnalysisStepConfig& config)
{
    auto pStep = std::make_shared<AnalysisStep>();
    pStep->m_Id = config.id > 0 ? config.id : static_cast<int>(m_AnalysisStep.size()) + 1;
    pStep->m_Name = config.name.trimmed().isEmpty()
        ? QStringLiteral("Step-%1").arg(pStep->m_Id) : config.name.trimmed();
    pStep->m_Type = config.type;//分析步类型
    pStep->m_Time = config.totalTime;//总时间
    pStep->m_StepSize = config.stepSize;
    pStep->m_Tolerance = config.tolerance;
    pStep->m_MaxIterations = config.maxIterations;
    pStep->m_DynamicSolverType = config.dynamicSolverType;
    pStep->m_InitialStaticStepId = pStep->m_Type == EnumKeyword::StepType::DYNAMIC
        ? config.initialStaticStepId : 0;
    pStep->m_AdaptiveTssbn = config.adaptiveTssbn;
    pStep->m_GallopingIceThickness = AeroManager::isSupportedIceThickness(config.gallopingIceThickness)
        ? config.gallopingIceThickness : AeroManager::supportedIceThicknesses().front();
    pStep->m_GallopingInitialAttackDegrees =
        std::isfinite(config.gallopingInitialAttackDegrees)
        ? config.gallopingInitialAttackDegrees : 45.0;
    pStep->m_RegionScope = config.regionScope;
    pStep->m_ComputeRegionIds = config.computeRegionIds;
    pStep->isDynamic = (config.type == EnumKeyword::StepType::DYNAMIC);
    pStep->m_EnableGalloping = pStep->isDynamic && config.enableGalloping;

    m_AnalysisStep.insert(std::make_pair(pStep->m_Id, pStep));
}

// ===== 模型检查函数 =====
void StructureData::CleanupModel(double tolerance)
{
    int nodesBefore = static_cast<int>(m_Nodes.size());
    int elementsBefore = static_cast<int>(m_Elements.size());

    MergeDuplicateNodes(tolerance);
    RemoveDuplicateElements();
    RemoveOrphanNodes();
    RenumberAll();

    int nodesAfter = static_cast<int>(m_Nodes.size());
    int elementsAfter = static_cast<int>(m_Elements.size());

    if (nodesBefore != nodesAfter || elementsBefore != elementsAfter)          //清除则显示信息
    {
        qDebug().noquote() << QStringLiteral("节点: ") << nodesBefore << QStringLiteral(" -> ") << nodesAfter;
        qDebug().noquote() << QStringLiteral("单元: ") << elementsBefore << QStringLiteral(" -> ") << elementsAfter;
    }

}

//数据优化-----不用看这些代码-------------------
// 临时节点结构（用于扁平化数据，提升缓存效率）
struct TempNode
{
    int id;
    double x, y, z;
};

void StructureData::MergeDuplicateNodes(double tolerance)
{
    if (m_Nodes.empty()) return;

    // Coincident master/slave nodes are meaningful for releases and must
    // remain distinct. Merging either endpoint would destroy the MPC before
    // the analysis starts.
    std::unordered_set<int> mpcNodeIds;
    for (const auto& [mpcId, mpc] : m_MPCConstraints)
    {
        (void)mpcId;
        if (!mpc) continue;
        for (const int nodeId : mpc->GetNodeIds())
            mpcNodeIds.insert(nodeId);
    }

    // 1. 预处理：计算边界并扁平化数据
    std::vector<TempNode> linearNodes;
    linearNodes.reserve(m_Nodes.size());

    double minX = std::numeric_limits<double>::max();
    double minY = minX, minZ = minX;

    for (const auto& pair : m_Nodes)
    {
        auto node = pair.second;
        // 计算最小坐标，用于防止哈希溢出
        if (node->m_X < minX) minX = node->m_X;
        if (node->m_Y < minY) minY = node->m_Y;
        if (node->m_Z < minZ) minZ = node->m_Z;

        linearNodes.push_back({ pair.first, node->m_X, node->m_Y, node->m_Z });
    }

    // 2. 计算 CellSize
    double cellSize = tolerance * 1.01;

    // 3. 空间哈希定义
    using HashKey = std::tuple<long long, long long, long long>;

    struct SpaceHash
    {
        std::size_t operator()(const HashKey& k) const
        {
            auto [x, y, z] = k;
            // 更加离散的哈希组合
            size_t h1 = std::hash<long long>{}(x);
            size_t h2 = std::hash<long long>{}(y);
            size_t h3 = std::hash<long long>{}(z);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };

    // 4. 分桶
    std::unordered_map<HashKey, std::vector<int>, SpaceHash> buckets;
    buckets.reserve(linearNodes.size());

    for (int i = 0; i < static_cast<int>(linearNodes.size()); ++i)
    {
        const auto& node = linearNodes[i];
        // 关键优化：减去 minX，确保从 0 开始计数，大大降低溢出风险
        long long kx = static_cast<long long>(std::floor((node.x - minX) / cellSize));
        long long ky = static_cast<long long>(std::floor((node.y - minY) / cellSize));
        long long kz = static_cast<long long>(std::floor((node.z - minZ) / cellSize));

        buckets[std::make_tuple(kx, ky, kz)].push_back(i);
    }

    // 5. 查找重复
    std::map<int, int> nodeIdMapping;
    std::vector<int> nodesToRemove;
    double tolSq = tolerance * tolerance;

    for (const auto& bucketPair : buckets)
    {
        const auto& key = bucketPair.first;
        const auto& indices = bucketPair.second;

        long long kx = std::get<0>(key);
        long long ky = std::get<1>(key);
        long long kz = std::get<2>(key);

        for (int idx1 : indices)
        {
            int originalId1 = linearNodes[idx1].id;

            // 如果这个节点已经被合并了，跳过
            if (nodeIdMapping.count(originalId1)) continue;

            const auto& n1 = linearNodes[idx1];

            // 搜索 3x3x3 邻域
            for (long long dx = -1; dx <= 1; ++dx)
            {
                for (long long dy = -1; dy <= 1; ++dy)
                {
                    for (long long dz = -1; dz <= 1; ++dz)
                    {

                        HashKey neighborKey = std::make_tuple(kx + dx, ky + dy, kz + dz);
                        auto it = buckets.find(neighborKey);
                        if (it == buckets.end()) continue;

                        for (int idx2 : it->second)
                        {
                            int originalId2 = linearNodes[idx2].id;

                            // 确保 id2 > id1，且避免自身比较
                            if (originalId2 <= originalId1) continue;

                            if (mpcNodeIds.count(originalId1) != 0
                                || mpcNodeIds.count(originalId2) != 0)
                                continue;

                            // 如果对方已经被合并，跳过
                            if (nodeIdMapping.count(originalId2)) continue;

                            const auto& n2 = linearNodes[idx2];

                            // 快速轴比较
                            double dxx = std::abs(n1.x - n2.x);
                            if (dxx > tolerance) continue;
                            double dyy = std::abs(n1.y - n2.y);
                            if (dyy > tolerance) continue;
                            double dzz = std::abs(n1.z - n2.z);
                            if (dzz > tolerance) continue;

                            // 距离平方比较
                            if (dxx * dxx + dyy * dyy + dzz * dzz < tolSq)
                            {
                                nodeIdMapping[originalId2] = originalId1;
                                nodesToRemove.push_back(originalId2);
                            }
                        }
                    }
                }
            }
        }
    }

    if (nodesToRemove.empty()) return;

    // 6. 更新引用
    auto getNewId = [&](int oldId) -> int
        {
            auto it = nodeIdMapping.find(oldId);
            return (it != nodeIdMapping.end()) ? it->second : oldId;
        };

    // 更新单元
    for (auto& elemPair : m_Elements)
    {
        auto& elem = elemPair.second;
        for (int i = 0; i < elem->m_pNode.size(); ++i)
        {
            auto ptr = elem->m_pNode[i].lock();
            if (ptr)
            {
                int newId = getNewId(ptr->m_Id);
                if (newId != ptr->m_Id)
                {
                    elem->m_pNode[i] = m_Nodes[newId];
                }
            }
        }
    }

    // 更新约束
    for (auto& conPair : m_Constraint)
    {
        auto ptr = conPair.second->m_pNode.lock();
        if (ptr)
        {
            int newId = getNewId(ptr->m_Id);
            if (newId != ptr->m_Id)
            {
                conPair.second->m_pNode = m_Nodes[newId];
            }
        }
    }

    // 更新主从约束。Clone 会把节点引用重新绑定到合并后的节点。
    if (!m_MPCConstraints.empty())
    {
        std::map<int, std::shared_ptr<Node>> reboundNodes = m_Nodes;
        for (const auto& [oldId, newId] : nodeIdMapping)
            reboundNodes[oldId] = m_Nodes[newId];

        for (auto& mpcPair : m_MPCConstraints)
        {
            if (auto rebound = mpcPair.second->Clone(reboundNodes))
            {
                rebound->m_Id = mpcPair.second->m_Id;
                mpcPair.second = std::move(rebound);
            }
        }
    }

    // 更新荷载
    for (auto& loadPair : m_Load)
    {
        auto forceNode = std::dynamic_pointer_cast<Force_Node>(loadPair.second);
        if (forceNode)
        {
            auto ptr = forceNode->m_pNode.lock();
            if (ptr)
            {
                int newId = getNewId(ptr->m_Id);
                if (newId != ptr->m_Id)
                {
                    forceNode->m_pNode = m_Nodes[newId];
                }
            }
        }
    }

    // 7. 批量删除
    for (int id : nodesToRemove)
    {
        m_Nodes.erase(id);
    }
}

struct VectorHash
{
    std::size_t operator()(const std::vector<int>& v) const
    {
        std::size_t seed = 0;
        for (int i : v)
        {
            // 经典的 hash combine 算法
            seed ^= std::hash<int>{}(i)+0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

void StructureData::RemoveDuplicateElements()
{
    if (m_Elements.empty()) return;

    std::unordered_set<std::vector<int>, VectorHash> seenTopologies;
    seenTopologies.reserve(m_Elements.size());

    std::vector<int> elementsToRemove;

    std::vector<int> nodeIds;
    nodeIds.reserve(4); // 预留常见单元节点数

    for (auto& elemPair : m_Elements)
    {
        auto& elem = elemPair.second;

        nodeIds.clear();

        bool isValid = true;
        for (auto& weakNode : elem->m_pNode)
        {
            auto node = weakNode.lock();
            if (!node)
            {
                isValid = false;
                break;
            }
            nodeIds.push_back(node->m_Id);
        }

        if (!isValid || nodeIds.empty())
        {
            elementsToRemove.push_back(elemPair.first);
            continue;
        }

        std::sort(nodeIds.begin(), nodeIds.end());

        if (!seenTopologies.insert(nodeIds).second)
        {
            elementsToRemove.push_back(elemPair.first);
        }
    }

    if (elementsToRemove.empty()) return;

    for (int id : elementsToRemove)
    {
        m_Elements.erase(id);
    }
}

void StructureData::RemoveOrphanNodes()
{
    if (m_Nodes.empty()) return;

    // 1. 找出当前最大的节点 ID，确定 vector 大小
    int maxNodeId = m_Nodes.rbegin()->first;

    // 2. 创建标记数组 (比 set 快几十倍)
    std::vector<bool> isNodeUsed(maxNodeId + 1, false);

    // 3. 遍历所有单元，标记用到的节点
    for (const auto& elemPair : m_Elements)
    {
        for (const auto& nodeWeak : elemPair.second->m_pNode)
        {
            auto node = nodeWeak.lock();
            if (node && node->m_Id <= maxNodeId)
            {
                isNodeUsed[node->m_Id] = true;
            }
        }
    }

    // 约束引用的节点也不能删
    for (const auto& conPair : m_Constraint)
    {
        auto node = conPair.second->m_pNode.lock();
        if (node && node->m_Id <= maxNodeId)
        {
            isNodeUsed[node->m_Id] = true;
        }
    }

    for (const auto& mpcPair : m_MPCConstraints)
    {
        if (!mpcPair.second) continue;
        for (const int nodeId : mpcPair.second->GetNodeIds())
        {
            if (nodeId >= 0 && nodeId <= maxNodeId)
                isNodeUsed[nodeId] = true;
        }
    }

    // 荷载引用的节点也不能删
    for (const auto& loadPair : m_Load)
    {
        auto forceNode = std::dynamic_pointer_cast<Force_Node>(loadPair.second);
        if (forceNode)
        {
            auto node = forceNode->m_pNode.lock();
            if (node && node->m_Id <= maxNodeId)
            {
                isNodeUsed[node->m_Id] = true;
            }
        }
    }

    // 4. 收集孤立节点
    std::vector<int> orphanNodes;
    orphanNodes.reserve(m_Nodes.size() / 10);

    for (const auto& nodePair : m_Nodes)
    {
        int id = nodePair.first;
        if (id > maxNodeId || !isNodeUsed[id])
        {
            orphanNodes.push_back(id);
        }
    }

    if (orphanNodes.empty()) return;

    for (int id : orphanNodes)
    {
        m_Nodes.erase(id);
    }
}

void StructureData::RenumberAll()
{
    // 重新编号节点
    std::map<int, std::shared_ptr<Node>> newNodes;
    int newId = 1;
    auto hint = newNodes.begin();

    for (auto& pair : m_Nodes)
    {
        pair.second->m_Id = newId;
        // 使用 emplace_hint 加速有序插入
        hint = newNodes.emplace_hint(hint, newId, pair.second);
        newId++;
    }
    m_Nodes = std::move(newNodes);

    // 重新编号单元
    std::map<int, std::shared_ptr<ElementBase>> newElements;
    newId = 1;
    for (auto& pair : m_Elements)
    {
        pair.second->m_Id = newId;
        newElements[newId] = pair.second;
        newId++;
    }
    m_Elements = std::move(newElements);

    // 重新编号约束
    std::map<int, std::shared_ptr<Constraint>> newConstraints;
    newId = 1;
    for (auto& pair : m_Constraint)
    {
        pair.second->m_Id = newId;
        newConstraints[newId] = pair.second;
        newId++;
    }
    m_Constraint = std::move(newConstraints);

    std::map<int, std::shared_ptr<NonlinearMPCConstraint>> newMpcs;
    newId = 1;
    for (auto& pair : m_MPCConstraints)
    {
        pair.second->m_Id = newId;
        newMpcs[newId] = pair.second;
        ++newId;
    }
    m_MPCConstraints = std::move(newMpcs);

    // 重新编号荷载
    std::map<int, std::shared_ptr<LoadBase>> newLoads;
    newId = 1;
    for (auto& pair : m_Load)
    {
        pair.second->m_Id = newId;
        newLoads[newId] = pair.second;
        newId++;
    }
    m_Load = std::move(newLoads);
}
