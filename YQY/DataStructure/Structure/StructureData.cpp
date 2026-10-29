#include "StructureData.h"
#include "DataStructure/Factory/LoadFactory.h"
#include "DataStructure/Factory/SectionFactory.h"
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
#include <stdexcept>
#include <utility>

template <typename Entity>
static int NextAvailableEntityId(const std::map<int, std::shared_ptr<Entity>>& entities)
{
    if (entities.empty() || entities.rbegin()->first < 1)
        return 1;
    if (entities.rbegin()->first < std::numeric_limits<int>::max())
        return entities.rbegin()->first + 1;

    int candidate = 1;
    for (const auto& [id, entity] : entities)
    {
        Q_UNUSED(entity);
        if (id < candidate)
            continue;
        if (id > candidate)
            break;
        if (candidate == std::numeric_limits<int>::max())
            throw std::overflow_error("模型实体编号已用尽");
        ++candidate;
    }
    return candidate;
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
    m_SpringBehaviors.clear();
    m_Constraint.clear();
    m_MPCConstraints.clear();
    m_RigidBodyInertias.clear();
    m_Load.clear();
    m_AnalysisStep.clear();
    m_ModelSets.clear();
    m_ComputeRegions.clear();

    // Clear 表示恢复为一个全新的空模型，不能保留上一次导入或求解的状态。
    m_OutputControl = OutputControl{};
    m_AeroManager = AeroManager{};
}

void StructureData::ReplaceModelFrom(StructureData&& source)
{
    if (this == &source)
        return;

    Clear();
    m_Nodes = std::move(source.m_Nodes);
    m_Elements = std::move(source.m_Elements);
    m_Material = std::move(source.m_Material);
    m_Section = std::move(source.m_Section);
    m_Property = std::move(source.m_Property);
    m_SpringBehaviors = std::move(source.m_SpringBehaviors);
    m_Constraint = std::move(source.m_Constraint);
    m_MPCConstraints = std::move(source.m_MPCConstraints);
    m_RigidBodyInertias = std::move(source.m_RigidBodyInertias);
    m_Load = std::move(source.m_Load);
    m_AnalysisStep = std::move(source.m_AnalysisStep);
    m_ModelSets = std::move(source.m_ModelSets);
    m_ComputeRegions = std::move(source.m_ComputeRegions);
    m_OutputControl = std::move(source.m_OutputControl);
    m_AeroManager = std::move(source.m_AeroManager);
}

std::shared_ptr<StructureData> StructureData::CloneForAnalysis(QString* errorMessage) const
{
    const auto fail = [errorMessage](const QString& message) -> std::shared_ptr<StructureData>
    {
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
        auto target = SectionFactory::Clone(source);
        if (!target)
            return fail(QStringLiteral("截面 %1 类型不受支持").arg(id));
        clone->m_Section.emplace(id, std::move(target));
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

    for (const auto& [id, source] : m_SpringBehaviors)
    {
        if (!source)
            return fail(QStringLiteral("弹簧行为 %1 为空").arg(id));
        clone->m_SpringBehaviors.emplace(id, std::make_shared<SpringBehavior>(*source));
    }

    for (const auto& [id, source] : m_Elements)
    {
        if (!source)
            return fail(QStringLiteral("单元 %1 为空").arg(id));

        std::shared_ptr<ElementBase> target;
        if (const auto truss = std::dynamic_pointer_cast<ElementTruss>(source))
        {
            // FEM H5 导入的桁架以“无应力长度 + 工程应变”保存预张力。区域求解
            // 必须完整保留该本构标记；否则这里新建的副本会退回到对数应变，导致
            // 即使荷载为零也产生错误的预应力松弛位移。
            auto trussTarget = std::make_shared<ElementTruss>();
            trussTarget->m_UseEngineeringStrain = truss->m_UseEngineeringStrain;
            target = std::move(trussTarget);
        }
        else if (const auto cable = std::dynamic_pointer_cast<ElementCable>(source))
        {
            auto cableTarget = std::make_shared<ElementCable>();
            cableTarget->CopyRuntimeState(*cable);
            target = std::move(cableTarget);
        }
        else if (const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(source))
        {
            auto beamTarget = std::make_shared<ElementBeam_CR>();
            beamTarget->q0 = beam->q0;
            beamTarget->R0 = beam->R0;
            beamTarget->CopyRuntimeState(*beam);
            target = std::move(beamTarget);
        }
        else if (const auto spring1 = std::dynamic_pointer_cast<ElementSpring1>(source))
        {
            auto springTarget = std::make_shared<ElementSpring1>();
            springTarget->m_DOF = spring1->m_DOF;
            target = std::move(springTarget);
        }
        else if (const auto spring2 = std::dynamic_pointer_cast<ElementSpring2>(source))
        {
            auto springTarget = std::make_shared<ElementSpring2>();
            springTarget->m_FirstDOF = spring2->m_FirstDOF;
            springTarget->m_SecondDOF = spring2->m_SecondDOF;
            target = std::move(springTarget);
        }
        else if (std::dynamic_pointer_cast<ElementSpringA>(source))
        {
            target = std::make_shared<ElementSpringA>();
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
        if (const auto sourceSpring = std::dynamic_pointer_cast<ElementSpringBase>(source))
        {
            const auto sourceBehavior = sourceSpring->m_pSpringBehavior.lock();
            const auto found = sourceBehavior ? clone->m_SpringBehaviors.find(sourceBehavior->m_Id)
                                              : clone->m_SpringBehaviors.end();
            if (found == clone->m_SpringBehaviors.end())
                return fail(QStringLiteral("弹簧单元 %1 的弹簧行为引用无效").arg(id));
            const auto targetSpring = std::dynamic_pointer_cast<ElementSpringBase>(target);
            targetSpring->m_pSpringBehavior = found->second;
            targetSpring->m_CurrentForce = sourceSpring->m_CurrentForce;
            targetSpring->m_CurrentRelativeDisplacement = sourceSpring->m_CurrentRelativeDisplacement;
        }
        clone->m_Elements.emplace(id, std::move(target));
    }

    QString bindingCopyError;
    if (!clone->CopyAnalysisBindingsFrom(*this, &bindingCopyError))
        return fail(bindingCopyError);

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
        config.adaptiveTimeStep = source->m_AdaptiveTimeStep;
        config.adaptiveTssbn = source->m_AdaptiveTssbn;
        config.structuralDamping = source->m_StructuralDamping.settings;
        config.enableGalloping = source->m_EnableGalloping;
        config.gallopingProfileBindingMode = source->m_GallopingProfileBindingMode;
        config.gallopingAerodynamicTangentMode = source->m_GallopingAerodynamicTangentMode;
        config.gallopingIceThickness = source->m_GallopingIceThickness;
        config.gallopingInitialAttackDegrees = source->m_GallopingInitialAttackDegrees;
        config.gallopingInitialVerticalPerturbationMeters = source->m_GallopingInitialVerticalPerturbationMeters;
        config.gallopingInitialVerticalPerturbationWaves = source->m_GallopingInitialVerticalPerturbationWaves;
        config.regionScope = source->m_RegionScope;
        config.computeRegionIds = source->m_ComputeRegionIds;
        clone->AddAnalysisStep(config);
        auto& copiedStep = clone->m_AnalysisStep.at(id);
        copiedStep->SetReferenceGravityEquilibrium(source->IsReferenceGravityEquilibrium());
        copiedStep->m_DiagnosticTssbnCacheExternalForcePerStage =
            source->m_DiagnosticTssbnCacheExternalForcePerStage;
        copiedStep->m_DiagnosticTssbnErrorTrace = source->m_DiagnosticTssbnErrorTrace;
        copiedStep->m_DiagnosticProfileGallopingForce = source->m_DiagnosticProfileGallopingForce;
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

bool StructureData::CopyAnalysisBindingsFrom(const StructureData& source, _OUT QString* errorMessage)
{
    if (errorMessage)
        errorMessage->clear();
    const auto fail = [errorMessage](const QString& message)
    {
        if (errorMessage)
            *errorMessage = message;
        return false;
    };
    std::map<int, std::shared_ptr<Constraint>> constraints;
    std::map<int, std::shared_ptr<NonlinearMPCConstraint>> mpcConstraints;
    std::map<int, std::shared_ptr<RigidBodyInertia>> rigidBodyInertias;
    std::map<int, std::shared_ptr<LoadBase>> loads;
    for (const auto& [id, sourceConstraint] : source.m_Constraint)
    {
        if (!sourceConstraint)
            return fail(QStringLiteral("约束 %1 为空").arg(id));
        const auto sourceNode = sourceConstraint->m_pNode.lock();
        const auto found = sourceNode ? m_Nodes.find(sourceNode->m_Id) : m_Nodes.end();
        if (found == m_Nodes.end() || !found->second)
            return fail(QStringLiteral("约束 %1 的节点引用无效").arg(id));
        auto target = std::make_shared<Constraint>(*sourceConstraint);
        target->m_pNode = found->second;
        constraints.emplace(id, std::move(target));
    }
    for (const auto& [id, sourceMpc] : source.m_MPCConstraints)
    {
        if (!sourceMpc)
            return fail(QStringLiteral("MPC %1 为空").arg(id));
        for (int nodeId : sourceMpc->GetNodeIds())
        {
            const auto found = m_Nodes.find(nodeId);
            if (found == m_Nodes.end() || !found->second)
                return fail(QStringLiteral("MPC %1 的节点引用无效").arg(id));
        }
        auto target = sourceMpc->Clone(m_Nodes);
        if (!target)
            return fail(QStringLiteral("MPC %1 的节点引用无效").arg(id));
        mpcConstraints.emplace(id, std::move(target));
    }
    for (const auto& [id, sourceInertia] : source.m_RigidBodyInertias)
    {
        if (!sourceInertia)
            return fail(QStringLiteral("刚体集中惯性 %1 为空").arg(id));
        const auto sourceNode = sourceInertia->m_pNode.lock();
        const auto found = sourceNode ? m_Nodes.find(sourceNode->m_Id) : m_Nodes.end();
        if (found == m_Nodes.end() || !found->second)
            return fail(QStringLiteral("刚体集中惯性 %1 的节点引用无效").arg(id));
        auto target = std::make_shared<RigidBodyInertia>(*sourceInertia);
        target->m_pNode = found->second;
        rigidBodyInertias.emplace(id, std::move(target));
    }
    for (const auto& [id, sourceLoad] : source.m_Load)
    {
        if (!sourceLoad)
            return fail(QStringLiteral("荷载 %1 为空").arg(id));
        auto target = LoadFactory::Clone(sourceLoad);
        if (!target)
            return fail(QStringLiteral("荷载 %1 类型不受支持").arg(id));
        if (const auto nodeLoad = std::dynamic_pointer_cast<Force_Node>(target))
        {
            const auto sourceNode = nodeLoad->m_pNode.lock();
            const auto found = sourceNode ? m_Nodes.find(sourceNode->m_Id) : m_Nodes.end();
            if (found == m_Nodes.end() || !found->second)
                return fail(QStringLiteral("节点荷载 %1 的节点引用无效").arg(id));
            nodeLoad->m_pNode = found->second;
        }
        else if (const auto elementLoad = std::dynamic_pointer_cast<Force_Element>(target))
        {
            const auto sourceElement = elementLoad->m_pElement.lock();
            const auto found = sourceElement ? m_Elements.find(sourceElement->m_Id) : m_Elements.end();
            if (found == m_Elements.end() || !found->second)
                return fail(QStringLiteral("单元荷载 %1 的单元引用无效").arg(id));
            elementLoad->m_pElement = found->second;
        }
        loads.emplace(id, std::move(target));
    }
    m_Constraint.swap(constraints);
    m_MPCConstraints.swap(mpcConstraints);
    m_RigidBodyInertias.swap(rigidBodyInertias);
    m_Load.swap(loads);
    return true;
}

int StructureData::AddModelSet(const QString& name, ModelSetType type, const std::set<int>& ids, QString* errorMessage)
{
    if (ids.empty())
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("集合不能为空。");
        return 0;
    }

    for (int id : ids)
    {
        const bool exists =
            type == ModelSetType::Node ? m_Nodes.find(id) != m_Nodes.cend() : m_Elements.find(id) != m_Elements.cend();
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

int StructureData::AddComputeRegion(const QString& name, const std::set<int>& nodeIds, const std::set<int>& elementIds,
                                    const std::set<int>& sourceSetIds, bool enabled, QString* errorMessage)
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
    region->m_Name = name.trimmed().isEmpty() ? QStringLiteral("计算区域-%1").arg(region->m_Id) : name.trimmed();
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

    if (m_ComputeRegions.size() == 1 && m_ComputeRegions.cbegin()->second &&
        m_ComputeRegions.cbegin()->second->m_AutoGenerated)
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

int StructureData::AddComputeRegionFromSets(const QString& name, const std::set<int>& sourceSetIds, bool enabled,
                                            QString* errorMessage)
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
                    *errorMessage = QStringLiteral("计算区域 %1 引用了不存在的集合 %2。").arg(regionId).arg(setId);
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
                    *errorMessage = QStringLiteral("计算区域 %1 引用了不存在的单元 %2。").arg(regionId).arg(elementId);
                return false;
            }
            for (const auto& nodeRef : elementIt->second->m_pNode)
            {
                const auto node = nodeRef.lock();
                if (!node)
                {
                    if (errorMessage)
                        *errorMessage =
                            QStringLiteral("计算区域 %1 的单元 %2 存在失效节点引用。").arg(regionId).arg(elementId);
                    return false;
                }
                region->m_NodeIds.insert(node->m_Id);
            }
        }

        // MPC 的中心主节点不直接连接单元，例如刚性中心间隔棒。
        // 只按主节点归属会遗漏这类中心节点，导致区域求解时约束被丢弃。
        // 任一端点进入区域时均纳入另一端点；重复执行以覆盖 MPC 链。
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
                        *errorMessage = QStringLiteral("MPC %1 没有有效的主从节点。").arg(mpcId);
                    return false;
                }
                const int masterNodeId = nodeIds[0];
                const int slaveNodeId = nodeIds[1];
                if (m_Nodes.find(masterNodeId) == m_Nodes.cend() || m_Nodes.find(slaveNodeId) == m_Nodes.cend())
                {
                    if (errorMessage)
                        *errorMessage = QStringLiteral("MPC %1 引用了不存在的节点。").arg(mpcId);
                    return false;
                }
                const bool containsMaster = region->ContainsNode(masterNodeId);
                const bool containsSlave = region->ContainsNode(slaveNodeId);
                if (containsMaster && region->m_NodeIds.insert(slaveNodeId).second)
                    mpcExpanded = true;
                if (containsSlave && region->m_NodeIds.insert(masterNodeId).second)
                    mpcExpanded = true;
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
                    lhsIt->second->m_Name = QStringLiteral("%1 + %2").arg(lhsIt->second->m_Name, rhsIt->second->m_Name);
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
    SynchronizeComputeRegionEnablementFromSteps();
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
                    *errorMessage = QStringLiteral("计算区域 %1 引用了不存在的节点 %2。").arg(lhsIt->first).arg(nodeId);
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
                    *errorMessage = QStringLiteral("MPC %1 没有有效的主从节点。").arg(mpcId);
                return false;
            }
            if (region->ContainsNode(nodeIds[0]) && !region->ContainsNode(nodeIds[1]))
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("计算区域 %1 包含 MPC %2 的主节点 %3，"
                                                   "但没有包含从节点 %4。")
                                        .arg(lhsIt->first)
                                        .arg(mpcId)
                                        .arg(nodeIds[0])
                                        .arg(nodeIds[1]);
                return false;
            }
        }
        auto rhsIt = std::next(lhsIt);
        for (; rhsIt != m_ComputeRegions.cend(); ++rhsIt)
        {
            if (rhsIt->second && region->Overlaps(*rhsIt->second))
            {
                if (errorMessage)
                    *errorMessage =
                        QStringLiteral("计算区域 %1 与 %2 仍有共享节点或单元。").arg(lhsIt->first).arg(rhsIt->first);
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
        const int defaultRegionId =
            AddComputeRegion(QStringLiteral("默认计算区域"), {}, elementIds, {}, true, &ignoredError);
        const auto regionIt = m_ComputeRegions.find(defaultRegionId);
        if (regionIt != m_ComputeRegions.end() && regionIt->second)
            regionIt->second->m_AutoGenerated = true;
        RebuildAndMergeComputeRegions(&ignoredError);
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

    SynchronizeComputeRegionEnablementFromSteps();
}

void StructureData::SynchronizeComputeRegionEnablementFromSteps()
{
    if (m_AnalysisStep.empty())
        return;

    bool anyStepUsesAllRegions = false;
    std::set<int> referencedRegionIds;
    for (const auto& [stepId, step] : m_AnalysisStep)
    {
        Q_UNUSED(stepId);
        if (!step)
            continue;
        if (step->m_RegionScope == AnalysisRegionScope::AllEnabledRegions)
            anyStepUsesAllRegions = true;
        else
            referencedRegionIds.insert(step->m_ComputeRegionIds.begin(), step->m_ComputeRegionIds.end());
    }

    for (auto& [regionId, region] : m_ComputeRegions)
    {
        if (region)
            region->m_Enabled = anyStepUsesAllRegions || referencedRegionIds.contains(regionId);
    }
}

std::vector<int> StructureData::ResolveAnalysisStepRegionIds(const AnalysisStep& step) const
{
    std::vector<int> result;
    for (const auto& [regionId, region] : m_ComputeRegions)
    {
        if (!region || !region->m_Enabled)
            continue;
        if (step.m_RegionScope == AnalysisRegionScope::AllEnabledRegions ||
            step.m_ComputeRegionIds.find(regionId) != step.m_ComputeRegionIds.cend())
        {
            result.push_back(regionId);
        }
    }
    return result;
}

std::shared_ptr<StructureData> StructureData::CloneRegionForAnalysis(int regionId, int analysisStepId,
                                                                     QString* errorMessage) const
{
    return CloneRegionForAnalysis(regionId, std::set<int>{analysisStepId}, errorMessage);
}

std::shared_ptr<StructureData> StructureData::CloneRegionForAnalysis(int regionId, const std::set<int>& analysisStepIds,
                                                                     QString* errorMessage) const
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
    for (auto it = clone->m_Elements.begin(); it != clone->m_Elements.end();)
    {
        if (region->m_ElementIds.find(it->first) == region->m_ElementIds.cend())
            it = clone->m_Elements.erase(it);
        else
            ++it;
    }
    for (auto it = clone->m_Nodes.begin(); it != clone->m_Nodes.end();)
    {
        if (region->m_NodeIds.find(it->first) == region->m_NodeIds.cend())
            it = clone->m_Nodes.erase(it);
        else
            ++it;
    }

    // 原模型中导线节点可能与塔架梁相连，因而被提升为 6 自由度。区域计算移除塔架
    // 后若仍保留这些转角自由度，纯桁架区域会出现零刚度行，静力线性方程必然奇异。
    // 依据区域副本中实际保留的单元重新确定每个节点的自由度数。
    std::map<int, int> requiredNodeDofs;
    for (const auto& [nodeId, node] : clone->m_Nodes)
    {
        if (!node)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("计算区域 %1 含有空节点 %2。").arg(regionId).arg(nodeId);
            return nullptr;
        }
        requiredNodeDofs.emplace(nodeId, 3);
    }
    for (const auto& [elementId, element] : clone->m_Elements)
    {
        if (!element)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("计算区域 %1 含有空单元 %2。").arg(regionId).arg(elementId);
            return nullptr;
        }
        const int elementDofs = std::max(3, element->Get_NodeDOF());
        for (const auto& nodeRef : element->m_pNode)
        {
            const auto node = nodeRef.lock();
            if (!node)
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("计算区域 %1 的单元 %2 含有失效节点。").arg(regionId).arg(elementId);
                return nullptr;
            }
            const auto requirement = requiredNodeDofs.find(node->m_Id);
            if (requirement == requiredNodeDofs.cend())
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("计算区域 %1 的单元 %2 引用了区域外节点 %3。")
                                        .arg(regionId)
                                        .arg(elementId)
                                        .arg(node->m_Id);
                return nullptr;
            }
            requirement->second = std::max(requirement->second, elementDofs);
        }
        if (const auto spring = std::dynamic_pointer_cast<ElementSpring1>(element); spring && spring->m_DOF >= 3)
        {
            const auto node = spring->m_pNode[0].lock();
            if (node)
                requiredNodeDofs.at(node->m_Id) = 6;
        }
        if (const auto spring = std::dynamic_pointer_cast<ElementSpring2>(element))
        {
            const auto first = spring->m_pNode[0].lock();
            const auto second = spring->m_pNode[1].lock();
            if (first && spring->m_FirstDOF >= 3)
                requiredNodeDofs.at(first->m_Id) = 6;
            if (second && spring->m_SecondDOF >= 3)
                requiredNodeDofs.at(second->m_Id) = 6;
        }
    }
    for (const auto& [nodeId, requiredDofs] : requiredNodeDofs)
        clone->m_Nodes.at(nodeId)->SetExactNumDOFs(requiredDofs);
    for (auto it = clone->m_Constraint.begin(); it != clone->m_Constraint.end();)
    {
        const auto node = it->second ? it->second->m_pNode.lock() : nullptr;
        if (!node || clone->m_Nodes.find(node->m_Id) == clone->m_Nodes.cend() ||
            static_cast<int>(it->second->m_Direction) >= node->m_DOF.size())
            it = clone->m_Constraint.erase(it);
        else
            ++it;
    }

    // 区域只保留一根导线（或局部构件）时，塔架等外部连接单元会被裁剪。
    // 这些被裁剪单元连接到区域的节点就是子模型的截断边界；若不施加支座，
    // 局部模型会整体刚体漂移，出现“看似求解成功但位移数米/数十米”的假结果。
    // 在分析副本中固定该边界节点的保留自由度，原模型、集合和用户原始约束均不改动。
    std::set<int> cutBoundaryNodeIds;
    for (const auto& [sourceElementId, sourceElement] : m_Elements)
    {
        if (!sourceElement || region->m_ElementIds.contains(sourceElementId))
            continue;
        for (const auto& nodeRef : sourceElement->m_pNode)
        {
            const auto node = nodeRef.lock();
            if (node && clone->m_Nodes.contains(node->m_Id))
                cutBoundaryNodeIds.insert(node->m_Id);
        }
    }
    int nextConstraintId = clone->m_Constraint.empty() ? 1 : clone->m_Constraint.rbegin()->first + 1;
    for (int nodeId : cutBoundaryNodeIds)
    {
        const auto nodeIt = clone->m_Nodes.find(nodeId);
        if (nodeIt == clone->m_Nodes.cend() || !nodeIt->second)
            continue;
        const int dofCount = static_cast<int>(nodeIt->second->m_DOF.size());
        for (int direction = 0; direction < dofCount; ++direction)
        {
            const bool alreadyConstrained = std::any_of(
                clone->m_Constraint.cbegin(), clone->m_Constraint.cend(), [nodeId, direction](const auto& entry)
                {
                    const auto& constraint = entry.second;
                    const auto constraintNode = constraint ? constraint->m_pNode.lock() : nullptr;
                    return constraintNode && constraintNode->m_Id == nodeId &&
                           static_cast<int>(constraint->m_Direction) == direction;
                });
            if (alreadyConstrained)
                continue;
            auto support = std::make_shared<Constraint>();
            support->m_Id = nextConstraintId++;
            support->m_Name = QStringLiteral("区域截断支座");
            support->m_StepId = 0;
            support->m_pNode = nodeIt->second;
            support->m_Direction = static_cast<EnumKeyword::Direction>(direction);
            support->m_Value = 0.0;
            clone->m_Constraint.emplace(support->m_Id, std::move(support));
        }
    }
    for (auto it = clone->m_MPCConstraints.begin(); it != clone->m_MPCConstraints.end();)
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
                *errorMessage = QStringLiteral("MPC %1 没有有效的主从节点。").arg(it->first);
            return nullptr;
        }

        // MPC 始终保留在源模型中；区域分析副本仅保留主节点属于该区域的 MPC。
        if (!region->ContainsNode(nodeIds[0]))
        {
            it = clone->m_MPCConstraints.erase(it);
            continue;
        }
        if (!region->ContainsNode(nodeIds[1]) || clone->m_Nodes.find(nodeIds[0]) == clone->m_Nodes.cend() ||
            clone->m_Nodes.find(nodeIds[1]) == clone->m_Nodes.cend())
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("计算区域 %1 中 MPC %2 的主从节点不完整。").arg(regionId).arg(it->first);
            return nullptr;
        }
        ++it;
    }
    for (auto it = clone->m_RigidBodyInertias.begin(); it != clone->m_RigidBodyInertias.end();)
    {
        const auto node = it->second ? it->second->m_pNode.lock() : nullptr;
        if (!node || clone->m_Nodes.find(node->m_Id) == clone->m_Nodes.cend())
            it = clone->m_RigidBodyInertias.erase(it);
        else
            ++it;
    }
    for (auto it = clone->m_Load.begin(); it != clone->m_Load.end();)
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
    for (auto it = clone->m_AnalysisStep.begin(); it != clone->m_AnalysisStep.end();)
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
    for (auto it = clone->m_ComputeRegions.begin(); it != clone->m_ComputeRegions.end();)
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
        if (!existingProp)
            continue;
        auto existingMat = existingProp->m_pMaterial.lock();
        auto existingSec = existingProp->m_pSection.lock();

        if (existingMat && existingSec && existingMat->m_Id == id_material && existingSec->m_Id == id_section)
        {
            // 已存在相同组合，直接返回
            return existingProp;
        }
    }

    // 不存在，创建新的 Property
    int property_id = NextAvailableEntityId(m_Property);
    auto property = std::make_shared<Property>();

    auto iterMat = m_Material.find(id_material);
    if (iterMat != m_Material.end() && iterMat->second)
    {
        property->m_pMaterial = iterMat->second;
    }
    else
    {
        qDebug().noquote() << QStringLiteral("Error: Create_Property 未找到材料 id=") << id_material;
        return nullptr;
    }

    auto iterSec = m_Section.find(id_section);
    if (iterSec != m_Section.end() && iterSec->second)
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
    pMaterial->m_Id = NextAvailableEntityId(m_Material);
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

    int autoId = NextAvailableEntityId(m_Section);

    auto pSection = SectionFactory::CreateCircular(autoId, Area);
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
            pConstraint->m_Id = NextAvailableEntityId(m_Constraint);
            pConstraint->m_pNode = pNode;
            pConstraint->m_Direction = static_cast<EnumKeyword::Direction>(direaction[i]);
            pConstraint->m_Value = value[i];
            m_Constraint.insert({pConstraint->m_Id, pConstraint});
        }
    }
    return int(num * Nodeid.size());
}

void StructureData::Add_Gravity(int direction, int idStep)
{
    int autoId = NextAvailableEntityId(m_Load);

    auto pLoad = std::make_shared<Force_Gravity>();
    pLoad->m_Id = autoId;

    pLoad->m_Direction = static_cast<EnumKeyword::Direction>(direction);

    pLoad->m_StepId = idStep;
    m_Load.insert(std::make_pair(autoId, pLoad));
}

void StructureData::AddAnalysisStep(const AnalysisStepConfig& config)
{
    auto pStep = std::make_shared<AnalysisStep>();
    pStep->m_Id = config.id > 0 ? config.id : NextAvailableEntityId(m_AnalysisStep);
    pStep->m_Name =
        config.name.trimmed().isEmpty() ? QStringLiteral("Step-%1").arg(pStep->m_Id) : config.name.trimmed();
    pStep->m_Type = config.type;      //分析步类型
    pStep->m_Time = config.totalTime; //总时间
    pStep->m_StepSize = config.stepSize;
    pStep->m_Tolerance = config.tolerance;
    pStep->m_MaxIterations = config.maxIterations;
    pStep->m_DynamicSolverType = config.dynamicSolverType;
    pStep->m_InitialStaticStepId = pStep->m_Type == EnumKeyword::StepType::DYNAMIC ? config.initialStaticStepId : 0;
    pStep->m_AdaptiveTimeStep = config.adaptiveTimeStep;
    pStep->m_AdaptiveTssbn = config.adaptiveTssbn;
    pStep->m_StructuralDamping.settings = config.structuralDamping;
    pStep->m_GallopingIceThickness = AeroManager::isSupportedIceThickness(config.gallopingIceThickness)
                                         ? config.gallopingIceThickness
                                         : AeroManager::supportedIceThicknesses().front();
    pStep->m_GallopingInitialAttackDegrees =
        std::isfinite(config.gallopingInitialAttackDegrees) ? config.gallopingInitialAttackDegrees : 45.0;
    pStep->m_GallopingInitialVerticalPerturbationMeters = config.gallopingInitialVerticalPerturbationMeters;
    pStep->m_GallopingInitialVerticalPerturbationWaves = config.gallopingInitialVerticalPerturbationWaves;
    pStep->m_RegionScope = config.regionScope;
    pStep->m_ComputeRegionIds = config.computeRegionIds;
    pStep->isDynamic = (config.type == EnumKeyword::StepType::DYNAMIC);
    pStep->m_EnableGalloping = config.enableGalloping;
    pStep->m_GallopingProfileBindingMode = config.gallopingProfileBindingMode;
    pStep->m_GallopingAerodynamicTangentMode = config.gallopingAerodynamicTangentMode;

    m_AnalysisStep.insert(std::make_pair(pStep->m_Id, pStep));
    SynchronizeComputeRegionEnablementFromSteps();
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

    if (nodesBefore != nodesAfter || elementsBefore != elementsAfter) //清除则显示信息
    {
        qDebug().noquote() << QStringLiteral("节点: ") << nodesBefore << QStringLiteral(" -> ") << nodesAfter;
        qDebug().noquote() << QStringLiteral("单元: ") << elementsBefore << QStringLiteral(" -> ") << elementsAfter;
    }
}

// 临时节点结构（用于扁平化数据，提升缓存效率）
struct TempNode
{
    int id;
    double x, y, z;
};

void StructureData::MergeDuplicateNodes(double tolerance)
{
    if (m_Nodes.empty())
        return;

    // 重合的主从节点可用于端部释放，必须保持独立；合并任一节点都会破坏 MPC。
    std::unordered_set<int> mpcNodeIds;
    for (const auto& [mpcId, mpc] : m_MPCConstraints)
    {
        (void)mpcId;
        if (!mpc)
            continue;
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
        if (node->m_X < minX)
            minX = node->m_X;
        if (node->m_Y < minY)
            minY = node->m_Y;
        if (node->m_Z < minZ)
            minZ = node->m_Z;

        linearNodes.push_back({pair.first, node->m_X, node->m_Y, node->m_Z});
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
            if (nodeIdMapping.count(originalId1))
                continue;

            const auto& n1 = linearNodes[idx1];

            // 用单层循环遍历 3x3x3 邻域，避免三层控制流嵌套。
            for (int neighborIndex = 0; neighborIndex < 27; ++neighborIndex)
            {
                const long long dx = neighborIndex / 9 - 1;
                const long long dy = neighborIndex / 3 % 3 - 1;
                const long long dz = neighborIndex % 3 - 1;
                const HashKey neighborKey = std::make_tuple(kx + dx, ky + dy, kz + dz);
                const auto bucket = buckets.find(neighborKey);
                if (bucket == buckets.end())
                    continue;

                for (int idx2 : bucket->second)
                {
                    const int originalId2 = linearNodes[idx2].id;
                    if (originalId2 <= originalId1)
                        continue;
                    if (mpcNodeIds.count(originalId1) != 0 || mpcNodeIds.count(originalId2) != 0)
                        continue;
                    if (nodeIdMapping.count(originalId2))
                        continue;

                    const auto& n2 = linearNodes[idx2];
                    const double dxx = std::abs(n1.x - n2.x);
                    const double dyy = std::abs(n1.y - n2.y);
                    const double dzz = std::abs(n1.z - n2.z);
                    if (dxx > tolerance || dyy > tolerance || dzz > tolerance)
                        continue;
                    if (dxx * dxx + dyy * dyy + dzz * dzz >= tolSq)
                        continue;

                    nodeIdMapping[originalId2] = originalId1;
                    nodesToRemove.push_back(originalId2);
                }
            }
        }
    }

    if (nodesToRemove.empty())
        return;

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
            seed ^= std::hash<int>{}(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

void StructureData::RemoveDuplicateElements()
{
    if (m_Elements.empty())
        return;

    std::unordered_set<std::vector<int>, VectorHash> seenTopologies;
    seenTopologies.reserve(m_Elements.size());

    std::vector<int> elementsToRemove;

    std::vector<int> nodeIds;
    nodeIds.reserve(4); // 预留常见单元节点数

    for (auto& elemPair : m_Elements)
    {
        auto& elem = elemPair.second;
        if (std::dynamic_pointer_cast<ElementSpringBase>(elem))
            continue;

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

    if (elementsToRemove.empty())
        return;

    for (int id : elementsToRemove)
    {
        m_Elements.erase(id);
    }
}

void StructureData::RemoveOrphanNodes()
{
    if (m_Nodes.empty())
        return;

    // 外部节点编号可稀疏且接近 int 上限，只记录实际引用，避免按最大编号分配内存。
    std::unordered_set<int> usedNodeIds;
    usedNodeIds.reserve(m_Nodes.size());
    for (const auto& [elementId, element] : m_Elements)
    {
        Q_UNUSED(elementId);
        if (!element)
            continue;
        for (const auto& nodeReference : element->m_pNode)
            if (const auto node = nodeReference.lock())
                usedNodeIds.insert(node->m_Id);
    }
    for (const auto& [constraintId, constraint] : m_Constraint)
    {
        Q_UNUSED(constraintId);
        if (const auto node = constraint ? constraint->m_pNode.lock() : nullptr)
            usedNodeIds.insert(node->m_Id);
    }
    for (const auto& [mpcId, mpc] : m_MPCConstraints)
    {
        Q_UNUSED(mpcId);
        if (!mpc)
            continue;
        for (const int nodeId : mpc->GetNodeIds())
            usedNodeIds.insert(nodeId);
    }
    for (const auto& [inertiaId, inertia] : m_RigidBodyInertias)
    {
        Q_UNUSED(inertiaId);
        if (const auto node = inertia ? inertia->m_pNode.lock() : nullptr)
            usedNodeIds.insert(node->m_Id);
    }
    for (const auto& [loadId, load] : m_Load)
    {
        Q_UNUSED(loadId);
        const auto nodeLoad = std::dynamic_pointer_cast<Force_Node>(load);
        if (const auto node = nodeLoad ? nodeLoad->m_pNode.lock() : nullptr)
            usedNodeIds.insert(node->m_Id);
    }

    for (auto node = m_Nodes.begin(); node != m_Nodes.end();)
    {
        if (usedNodeIds.find(node->first) == usedNodeIds.end())
            node = m_Nodes.erase(node);
        else
            ++node;
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

    std::map<int, std::shared_ptr<RigidBodyInertia>> newRigidBodyInertias;
    newId = 1;
    for (auto& pair : m_RigidBodyInertias)
    {
        pair.second->m_Id = newId;
        newRigidBodyInertias[newId] = pair.second;
        ++newId;
    }
    m_RigidBodyInertias = std::move(newRigidBodyInertias);

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
